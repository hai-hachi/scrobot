// SCROBOT experimental sliding-mode tracking controller for Nav2 Jazzy.
// Control is evaluated in sim time by the controller_server's patched ROS
// clock scheduler, not a wall-time loop. The command router enforces coupled
// physical wheel-speed limits on *both* this plugin and RPP.
#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav2_core/controller.hpp>
#include <nav2_util/node_utils.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

namespace scrobot_tracker_controller
{
constexpr double kWheelHalfBase=0.21;
constexpr double kMaxWheelMps=0.50;

double wrap(double a) { return std::atan2(std::sin(a),std::cos(a)); }
double yawOf(const geometry_msgs::msg::Quaternion &q)
{
  return std::atan2(2.0*(q.w*q.z+q.x*q.y),
                    1.0-2.0*(q.y*q.y+q.z*q.z));
}
double sat(double x) { return std::clamp(x,-1.0,1.0); }

class SMCController : public nav2_core::Controller
{
public:
  void configure(
      const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
      std::string name,
      std::shared_ptr<tf2_ros::Buffer> /*tf*/,
      std::shared_ptr<nav2_costmap_2d::Costmap2DROS> /*costmap*/) override
  {
    node_=parent.lock();
    if (!node_) { throw std::runtime_error("SMC parent node unavailable"); }
    name_=name;
    auto declare=[this](const std::string &field,double value) {
      nav2_util::declare_parameter_if_not_declared(
        node_,name_+"."+field,rclcpp::ParameterValue(value));
      return node_->get_parameter(name_+"."+field).as_double();
    };
    base_speed_=declare("linear_speed",0.30);
    cross_gain_=declare("cross_gain",1.8);
    heading_gain_=declare("heading_gain",2.0);
    switch_gain_=declare("switch_gain",1.1);
    boundary_=declare("boundary_layer",0.30);
    lookahead_=declare("lookahead_m",0.32);
    if (base_speed_<=0 || base_speed_>.50 || cross_gain_<=0 ||
        heading_gain_<=0 || switch_gain_<=0 || boundary_<=0 ||
        lookahead_<=0)
      throw std::runtime_error("Invalid SMC tracking gains");
  }
  void cleanup() override { plan_.poses.clear(); lengths_.clear(); node_.reset(); }
  void activate() override {}
  void deactivate() override {}
  void reset() override { last_s_=-1; }
  void setPlan(const nav_msgs::msg::Path &path) override
  {
    if (path.poses.size()<2)
      throw std::runtime_error("SMC requires at least 2 path points");
    plan_=path;
    lengths_.assign(plan_.poses.size(),0.0);
    for(size_t i=1;i<plan_.poses.size();++i) {
      const auto &a=plan_.poses[i-1].pose.position;
      const auto &b=plan_.poses[i].pose.position;
      lengths_[i]=lengths_[i-1]+std::hypot(b.x-a.x,b.y-a.y);
    }
    last_s_=-1.0;
  }
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
      const geometry_msgs::msg::PoseStamped &pose,
      const geometry_msgs::msg::Twist & /*velocity*/,
      nav2_core::GoalChecker * /*goal_checker*/) override
  {
    if (plan_.poses.size()<2)
      throw std::runtime_error("SMC plan is empty");
    const double x=pose.pose.position.x, y=pose.pose.position.y;
    const double yaw=yawOf(pose.pose.orientation);
    const auto &goal=plan_.poses.back().pose;
    const double goal_dist=std::hypot(x-goal.position.x,y-goal.position.y);
    const double goal_yaw=yawOf(goal.orientation);
    geometry_msgs::msg::TwistStamped command;
    command.header.stamp=node_->now();
    command.header.frame_id="base_footprint";

    // Near the end, align to exact final goal yaw. The Nav2 goal checker
    // still owns success/failure; this controller doesn't fake completion.
    if(goal_dist<0.14) {
      command.twist.angular.z=std::clamp(
        2.0*wrap(goal_yaw-yaw),-1.2,1.2);
      return command;
    }

    double min_dist=std::numeric_limits<double>::infinity();
    size_t idx=0;
    double best_s=0, best_x=0, best_y=0, best_yaw=0;
    for(size_t i=0;i+1<plan_.poses.size();++i) {
      const auto &a=plan_.poses[i].pose.position;
      const auto &b=plan_.poses[i+1].pose.position;
      double dx=b.x-a.x,dy=b.y-a.y;
      double norm=dx*dx+dy*dy;
      if(norm<1e-12)continue;
      double t=std::clamp(((x-a.x)*dx+(y-a.y)*dy)/norm,0.0,1.0);
      double px=a.x+t*dx,py=a.y+t*dy;
      double distance=std::hypot(x-px,y-py);
      double s=lengths_[i]+t*std::sqrt(norm);
      // Restrict ambiguous closest-segment choices on a loop, but allow
      // initial robot placement at an arbitrary progress along the path.
      if(last_s_>=0 && (s<last_s_-.25 || s>last_s_+1.15))continue;
      if(distance<min_dist) {
        min_dist=distance;idx=i;best_s=s;
        best_x=px;best_y=py;best_yaw=std::atan2(dy,dx);
      }
    }
    if(!std::isfinite(min_dist)) {
      // Lost the last progress window: one unrestricted reacquisition is
      // acceptable. A truly degenerate plan must fail cleanly, not recurse.
      if(last_s_<0)throw std::runtime_error(
          "SMC cannot project robot onto a nondegenerate path segment");
      last_s_=-1;
      return computeVelocityCommands(pose,geometry_msgs::msg::Twist{},nullptr);
    }
    last_s_=std::max(last_s_,best_s);
    const double ey=-(x-best_x)*std::sin(best_yaw)+
                     (y-best_y)*std::cos(best_yaw);
    const double heading_error=wrap(yaw-best_yaw);
    // Sliding surface combines heading and lateral error.
    const double surface=heading_error+std::atan(cross_gain_*ey);
    // Reference curvature feed-forward over a short lookahead span.
    const double ahead_s=best_s+lookahead_;
    double ahead_yaw=best_yaw;
    for(size_t k=idx;k+1<lengths_.size();++k) {
      if(lengths_[k+1]>=ahead_s || k+2==lengths_.size()) {
        const auto &a=plan_.poses[k].pose.position;
        const auto &b=plan_.poses[k+1].pose.position;
        ahead_yaw=std::atan2(b.y-a.y,b.x-a.x);
        break;
      }
    }
    const double curvature=wrap(ahead_yaw-best_yaw)/
        std::max(lookahead_,0.1);
    double v=std::min(base_speed_*speed_ratio_,
                      std::max(0.08,0.85*goal_dist));
    if(std::abs(heading_error)>0.8)v*=0.55;
    if(std::abs(heading_error)>1.3)v=0.0;
    double omega=v*curvature-heading_gain_*heading_error-
                 switch_gain_*sat(surface/boundary_);
    omega=std::clamp(omega,-2.3,2.3);
    // Hard coupled constraint is repeated in the external command router.
    double demand=std::abs(v)+kWheelHalfBase*std::abs(omega);
    if(demand>kMaxWheelMps) {
      double ratio=kMaxWheelMps/demand;
      v*=ratio;omega*=ratio;
    }
    command.twist.linear.x=v;
    command.twist.angular.z=omega;
    return command;
  }
  void setSpeedLimit(const double &limit,const bool &percentage) override
  {
    speed_ratio_=percentage?std::clamp(limit/100.0,0.0,1.0):
      std::clamp(limit/std::max(base_speed_,0.01),0.0,1.0);
  }
private:
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::string name_;
  nav_msgs::msg::Path plan_;
  std::vector<double> lengths_;
  double last_s_{-1},speed_ratio_{1},base_speed_{.30};
  double cross_gain_{1.8},heading_gain_{2},switch_gain_{1.1},
         boundary_{.30},lookahead_{.32};
};
}  // namespace scrobot_tracker_controller

PLUGINLIB_EXPORT_CLASS(
  scrobot_tracker_controller::SMCController,
  nav2_core::Controller)
