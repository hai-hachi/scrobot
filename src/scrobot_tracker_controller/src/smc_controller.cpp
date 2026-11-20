// Full SCROBOT SMC-1s / SMC-2s implementations from hai-hachi/smc,
// smc_lab/core.py (2026-11-20). One shared PP-circle reference generator.
// Nominal carrot (vR,wR) is NOT guaranteed to be the actual carrot derivative.
// Nav2 + physical wheel speed limiting invalidate ideal reaching identities.
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
#include <std_msgs/msg/float64_multi_array.hpp>

namespace scrobot_tracker_controller
{
constexpr double kWheelHalfBase=0.21;
constexpr double kMaxWheelMps=0.50;
constexpr double kEps=1e-9;
double wrap(double a) {return std::atan2(std::sin(a),std::cos(a));}
double sat(double x) {return std::clamp(x,-1.0,1.0);}
double yawOf(const geometry_msgs::msg::Quaternion &q)
{
  return std::atan2(2.0*(q.w*q.z+q.x*q.y),1.0-2.0*(q.y*q.y+q.z*q.z));
}
struct Position {double x=0,y=0;};
struct Reference {
  Position p;
  double yaw=0,kappa=0,vr=0,wr=0,s=0;
  bool fallback=false;
};
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
    if(!node_) throw std::runtime_error("SMC parent node unavailable");
    name_=name;
    one_surface_=(name_=="SMC_1S");
    if(!one_surface_ && name_!="SMC_2S")
      throw std::runtime_error("Full SMC plugin must be named SMC_1S or SMC_2S");
    auto param=[this](const std::string &key,double value) {
      nav2_util::declare_parameter_if_not_declared(
        node_,name_+"."+key,rclcpp::ParameterValue(value));
      return node_->get_parameter(name_+"."+key).as_double();
    };
    auto boolparam=[this](const std::string &key,bool value) {
      nav2_util::declare_parameter_if_not_declared(
        node_,name_+"."+key,rclcpp::ParameterValue(value));
      return node_->get_parameter(name_+"."+key).as_bool();
    };
    vr_nominal_=param("linear_speed",.30);
    lookahead_=param("lookahead_radius_m",.60);
    curvature_window_=param("curvature_window_m",.14);
    lambda_y_=param("lambda_y",2.5);
    if(one_surface_)lambda_x_=param("lambda_x",.6);
    ks_=param("k_s",1.2);
    kx_=param("k_x",.8);
    eta_=param("eta",.40);
    phi_=param("phi",.30);
    etax_=param("eta_x",.25);
    phix_=param("phi_x",.30);
    use_saturation_=boolparam("use_saturation",true);
    singular_guard_=param("singular_guard",.06);
    max_angular_=param("max_angular_cmd",1.50);
    accel_linear_=param("max_linear_accel",1.0);
    accel_angular_=param("max_angular_accel",2.0);
    if(vr_nominal_<=0 || vr_nominal_>.50 || lookahead_<=0 ||
       curvature_window_<=0 || lambda_y_<=0 || lambda_x_<0 ||
       ks_<=0 || kx_<=0 || eta_<0 || etax_<0 ||
       phi_<=0 || phix_<=0 || singular_guard_<=0 ||
       max_angular_<=0 || max_angular_>2.3 ||
       accel_linear_<=0 || accel_angular_<=0)
      throw std::runtime_error("Invalid full SMC tuning parameters");

    carrot_pub_=node_->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/tracker/smc_carrot",10);
    diagnostics_pub_=node_->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/tracker/smc_diagnostics",10);
    reset();
  }
  void cleanup() override {
    plan_.poses.clear();lengths_.clear();node_.reset();
    carrot_pub_.reset();diagnostics_pub_.reset();
  }
  void activate() override {}
  void deactivate() override {}
  void reset() override {
    last_path_s_=-1.;
    last_d_=std::numeric_limits<double>::quiet_NaN();
    last_command_stamp_=-1.;
    last_v_=0.;last_w_=0.;
  }
  void setPlan(const nav_msgs::msg::Path &path) override
  {
    if(path.poses.size()<2)
      throw std::runtime_error("SMC requires at least two path positions");
    plan_=path;
    lengths_.assign(plan_.poses.size(),0.0);
    for(size_t i=1;i<plan_.poses.size();++i) {
      const auto a=position(i-1),b=position(i);
      lengths_[i]=lengths_[i-1]+std::hypot(b.x-a.x,b.y-a.y);
    }
    if(lengths_.back()<kEps)
      throw std::runtime_error("SMC reference path has zero length");
    reset();
  }

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped &pose,
    const geometry_msgs::msg::Twist & /*velocity*/,
    nav2_core::GoalChecker * /*goal_checker*/) override
  {
    if(plan_.poses.size()<2)
      throw std::runtime_error("SMC plan not initialized");
    const double x=pose.pose.position.x,y=pose.pose.position.y;
    const double theta=yawOf(pose.pose.orientation);
    const auto goal=position(plan_.poses.size()-1);
    const double goal_dist=std::hypot(goal.x-x,goal.y-y);
    auto now=node_->now();
    geometry_msgs::msg::TwistStamped command;
    command.header.stamp=now;
    command.header.frame_id="base_footprint";

    const double near_s=nearestS(x,y);
    const Reference ref=carrot(x,y,near_s);
    const double dx=ref.p.x-x,dy=ref.p.y-y;
    const double ex=std::cos(theta)*dx+std::sin(theta)*dy;
    const double ey=-std::sin(theta)*dx+std::cos(theta)*dy;
    const double etheta=wrap(ref.yaw-theta);
    const double D=1.0+lambda_y_*ex;
    // The denominator guard is part of both *full* control laws. Do not
    // silently replace it with an epsilon or use a truncated controller.
    if(std::abs(D)<=singular_guard_ ||
       (std::isfinite(last_d_) && D*last_d_<0.0))
      throw std::runtime_error("Full SMC singular denominator D=1+lambda_y*e_x");
    last_d_=D;

    const double gx=kx_*ex+(use_saturation_?etax_*sat(ex/phix_):0.0);
    const double s2=etheta+lambda_y_*ey;
    const double s1=one_surface_?s2+lambda_x_*ex:ex;
    const double active=one_surface_?s1:s2;
    const double q=ks_*active+(use_saturation_?eta_*sat(active/phi_):0.0);
    // Exact full laws (see SMC_1s_vs_2s_summary):
    // 1s: omega=(wR+ly*vR*sin(etheta)-lx*gx+q(s))/D
    // 2s: omega=(wR+ly*vR*sin(etheta)+q(s2))/D
    // both: v=vR*cos(etheta)+ey*omega+gx
    double omega=(ref.wr+lambda_y_*ref.vr*std::sin(etheta)+
                   q-(one_surface_?lambda_x_*gx:0.0))/D;
    double v=ref.vr*std::cos(etheta)+ey*omega+gx;
    const double desired_v=v,desired_w=omega;
    // No reverse movement in baseline SMC lab. Near end use the same
    // final-goal-heading behavior for both full controllers.
    v=std::max(0.,v);
    if(goal_dist<.14) {
      v=0.;
      omega=std::clamp(2.*wrap(yawOf(plan_.poses.back().pose.orientation)-theta),
                       -max_angular_,max_angular_);
    }
    v=std::clamp(v,0.,kMaxWheelMps*speed_ratio_);
    omega=std::clamp(omega,-max_angular_,max_angular_);

    // Rate-limits happen AFTER computing the exact full-law requests.
    const double t=now.seconds();
    if(last_command_stamp_>=0 && t>last_command_stamp_) {
      const double dt=std::min(.5,t-last_command_stamp_);
      v=std::clamp(v,last_v_-accel_linear_*dt,last_v_+accel_linear_*dt);
      omega=std::clamp(omega,last_w_-accel_angular_*dt,
                          last_w_+accel_angular_*dt);
    }
    double wheel_demand=std::abs(v)+kWheelHalfBase*std::abs(omega);
    if(wheel_demand>kMaxWheelMps) {
      const double factor=kMaxWheelMps/wheel_demand;
      v*=factor;omega*=factor;
    }
    last_v_=v;last_w_=omega;last_command_stamp_=t;
    command.twist.linear.x=v;
    command.twist.angular.z=omega;

    geometry_msgs::msg::PoseStamped cp;
    cp.header.frame_id="odom";cp.header.stamp=now;
    cp.pose.position.x=ref.p.x;cp.pose.position.y=ref.p.y;
    cp.pose.orientation.z=std::sin(ref.yaw/2);
    cp.pose.orientation.w=std::cos(ref.yaw/2);
    carrot_pub_->publish(cp);
    // Field order matches tracking/runner.py; this is the *actual* carrot
    // used by SMC and nominal (not measured) vR,wR at each control cycle.
    std_msgs::msg::Float64MultiArray diag;
    diag.data={ref.p.x,ref.p.y,ref.yaw,ref.vr,ref.wr,
      ex,ey,etheta,s1,s2,D,gx,q,lookahead_,
      ref.fallback?1.0:0.0,desired_v,desired_w,ref.kappa};
    diagnostics_pub_->publish(diag);
    return command;
  }

  void setSpeedLimit(const double &limit,const bool &percentage) override
  {
    speed_ratio_=percentage?std::clamp(limit/100.0,0.0,1.0):
      std::clamp(limit/std::max(vr_nominal_,.01),0.0,1.0);
  }

private:
  Position position(size_t i) const {
    const auto &p=plan_.poses[i].pose.position;
    return {p.x,p.y};
  }
  Position sample(double s) const {
    s=std::clamp(s,0.,lengths_.back());
    const auto i=std::lower_bound(lengths_.begin()+1,lengths_.end(),s);
    const size_t k=std::min(static_cast<size_t>(i-lengths_.begin()),
                            lengths_.size()-1);
    const auto a=position(k-1),b=position(k);
    const double span=lengths_[k]-lengths_[k-1];
    const double t=span>kEps?(s-lengths_[k-1])/span:0.0;
    return {a.x+t*(b.x-a.x),a.y+t*(b.y-a.y)};
  }
  double nearestS(double x,double y)
  {
    double minDist=std::numeric_limits<double>::infinity();
    double best=0.;
    for(size_t i=0;i+1<lengths_.size();++i) {
      const auto a=position(i),b=position(i+1);
      const double dx=b.x-a.x,dy=b.y-a.y,span=dx*dx+dy*dy;
      if(span<1e-12)continue;
      const double t=std::clamp(((x-a.x)*dx+(y-a.y)*dy)/span,0.,1.);
      const double s=lengths_[i]+t*std::sqrt(span);
      if(last_path_s_>=0 && (s<last_path_s_-.25 || s>last_path_s_+1.15))
        continue;
      const double dist=std::hypot(x-(a.x+t*dx),y-(a.y+t*dy));
      if(dist<minDist) {minDist=dist;best=s;}
    }
    if(!std::isfinite(minDist)) {
      last_path_s_=-1.;return nearestS(x,y);
    }
    last_path_s_=std::max(last_path_s_,best);
    return best;
  }
  Reference carrot(double x,double y,double nearest)
  {
    Reference ref;
    ref.s=nearest;
    ref.fallback=true;
    // Exact first forward path/circle intersection on polyline segments.
    // Start the search at the robot projection; never choose a point behind it.
    bool found=false;
    for(size_t i=0;i+1<lengths_.size();++i) {
      if(lengths_[i+1]<=nearest+1e-8)continue;
      const double lo=std::max(nearest,lengths_[i]);
      const double hi=lengths_[i+1];
      const auto a=sample(lo),b=sample(hi);
      const double dx=b.x-a.x,dy=b.y-a.y;
      const double ox=a.x-x,oy=a.y-y;
      const double aa=dx*dx+dy*dy;
      if(aa<1e-14)continue;
      const double bb=ox*dx+oy*dy;
      const double cc=ox*ox+oy*oy-lookahead_*lookahead_;
      const double disc=bb*bb-aa*cc;
      if(disc< -1e-12)continue;
      const double root=std::sqrt(std::max(0.,disc));
      for(double candidate:{(-bb-root)/aa,(-bb+root)/aa}) {
        if(candidate< -1e-8 || candidate>1.+1e-8)continue;
        const double s=lo+std::clamp(candidate,0.,1.)*(hi-lo);
        if(s>nearest+1e-7 && (!found||s<ref.s)) {
          ref.s=s;ref.fallback=false;found=true;
        }
      }
    }
    if(!found) {
      auto end=position(plan_.poses.size()-1);
      if(std::hypot(end.x-x,end.y-y)<lookahead_+1e-7)
        ref.s=lengths_.back();
    }
    ref.p=sample(ref.s);
    // Three-point signed circumcircle curvature, as in Python SMC lab.
    const double length=lengths_.back();
    const double half=std::min(curvature_window_,length/2.);
    const double start=std::clamp(ref.s-half,0.,std::max(0.,length-2*half));
    const auto a=sample(start),b=sample(start+half),
               c=sample(start+2*half);
    const double abx=b.x-a.x,aby=b.y-a.y;
    const double bcx=c.x-b.x,bcy=c.y-b.y;
    const double acx=c.x-a.x,acy=c.y-a.y;
    const double den=std::hypot(abx,aby)*std::hypot(bcx,bcy)*
                     std::hypot(acx,acy);
    ref.kappa=den>1e-12?2*(abx*acy-aby*acx)/den:0.;
    ref.yaw=std::atan2(acy,acx);
    if(std::hypot(acx,acy)<1e-12) {
      // Degenerate near an exact 180-degree vertex: use local segment.
      const auto p0=sample(std::max(0.,ref.s-.01)),
                 p1=sample(std::min(length,ref.s+.01));
      ref.yaw=std::atan2(p1.y-p0.y,p1.x-p0.x);
    }
    const double remaining=std::max(0.,length-nearest);
    ref.vr=vr_nominal_*speed_ratio_*
      std::min(1.0,remaining/.75);
    if(remaining<=.025)ref.vr=0.;
    ref.wr=ref.vr*ref.kappa;
    return ref;
  }

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::string name_;
  bool one_surface_=true,use_saturation_=true;
  nav_msgs::msg::Path plan_;
  std::vector<double> lengths_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr carrot_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr diagnostics_pub_;
  double last_path_s_{-1.},last_d_{std::numeric_limits<double>::quiet_NaN()};
  double last_command_stamp_{-1.},last_v_{0.},last_w_{0.};
  double speed_ratio_{1.},vr_nominal_{.30};
  double lookahead_{.60},curvature_window_{.14};
  double lambda_y_{2.5},lambda_x_{.6},ks_{1.2},kx_{.8};
  double eta_{.4},phi_{.3},etax_{.25},phix_{.3};
  double singular_guard_{.06},max_angular_{1.5};
  double accel_linear_{1.0},accel_angular_{2.0};
};
} // namespace scrobot_tracker_controller

PLUGINLIB_EXPORT_CLASS(
  scrobot_tracker_controller::SMCController,
  nav2_core::Controller)
