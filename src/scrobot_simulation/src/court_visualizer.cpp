#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/color_rgba.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

using namespace std::chrono_literals;

class CourtVisualizer : public rclcpp::Node
{
public:
  CourtVisualizer()
  : Node("court_visualizer")
  {
    // ==========================================================
    // Parameters
    // ==========================================================

    frame_id_ = declare_parameter<std::string>(
      "frame_id", "court");

    // Floor
    floor_length_ = declare_parameter<double>(
      "floor_length", 18.0);

    floor_width_ = declare_parameter<double>(
      "floor_width", 10.0);

    floor_thickness_ = declare_parameter<double>(
      "floor_thickness", 0.02);

    // Court
    court_length_ = declare_parameter<double>(
      "court_length", 13.40);

    court_width_ = declare_parameter<double>(
      "court_width", 6.10);

    singles_width_ = declare_parameter<double>(
      "singles_width", 5.18);

    line_width_ = declare_parameter<double>(
      "line_width", 0.04);

    line_thickness_ = declare_parameter<double>(
      "line_thickness", 0.001);

    short_service_x_ = declare_parameter<double>(
      "short_service_x", 1.98);

    long_service_x_ = declare_parameter<double>(
      "long_service_x", 5.94);

    // Pole
    pole_x_ = declare_parameter<double>(
      "pole_x", 0.0);

    pole_y_ = declare_parameter<double>(
      "pole_y", 3.05);

    pole_diameter_ = declare_parameter<double>(
      "pole_diameter", 0.05);

    pole_height_ = declare_parameter<double>(
      "pole_height", 1.55);

    // Net
    net_thickness_ = declare_parameter<double>(
      "net_thickness", 0.005);

    net_width_ = declare_parameter<double>(
      "net_width", 6.10);

    net_height_ = declare_parameter<double>(
      "net_height", 0.76);

    net_center_z_ = declare_parameter<double>(
      "net_center_z", 1.17);

    // Net tape
    net_tape_thickness_ = declare_parameter<double>(
      "net_tape_thickness", 0.01);

    net_tape_width_ = declare_parameter<double>(
      "net_tape_width", 6.10);

    net_tape_height_ = declare_parameter<double>(
      "net_tape_height", 0.075);

    net_tape_center_z_ = declare_parameter<double>(
      "net_tape_center_z", 1.5125);

    // ==========================================================
    // Publisher
    // ==========================================================

    publisher_ =
      create_publisher<visualization_msgs::msg::MarkerArray>(
      "/court_markers",
      rclcpp::QoS(1).transient_local());

    timer_ = create_wall_timer(
      1s,
      std::bind(
        &CourtVisualizer::publishCourt,
        this));

    publishCourt();

    RCLCPP_INFO(
      get_logger(),
      "Publishing Gazebo-matched badminton court in frame '%s'",
      frame_id_.c_str());
  }


private:

  // ==========================================================
  // Colors
  // ==========================================================

  std_msgs::msg::ColorRGBA makeColor(
    float r,
    float g,
    float b,
    float a = 1.0f)
  {
    std_msgs::msg::ColorRGBA color;

    color.r = r;
    color.g = g;
    color.b = b;
    color.a = a;

    return color;
  }


  // ==========================================================
  // Base marker
  // ==========================================================

  visualization_msgs::msg::Marker makeMarker(
    int id,
    const std::string & ns,
    int type)
  {
    visualization_msgs::msg::Marker marker;

    marker.header.frame_id = frame_id_;
    marker.header.stamp = now();

    marker.ns = ns;
    marker.id = id;

    marker.type = type;
    marker.action =
      visualization_msgs::msg::Marker::ADD;

    marker.pose.orientation.w = 1.0;

    return marker;
  }


  // ==========================================================
  // Generic box
  // ==========================================================

  visualization_msgs::msg::Marker makeBox(
    int id,
    const std::string & ns,
    double x,
    double y,
    double z,
    double sx,
    double sy,
    double sz,
    const std_msgs::msg::ColorRGBA & color)
  {
    auto marker = makeMarker(
      id,
      ns,
      visualization_msgs::msg::Marker::CUBE);

    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = z;

    marker.scale.x = sx;
    marker.scale.y = sy;
    marker.scale.z = sz;

    marker.color = color;

    return marker;
  }


  // ==========================================================
  // Floor
  // ==========================================================

  void addFloor(
    visualization_msgs::msg::MarkerArray & markers)
  {
    markers.markers.push_back(
      makeBox(
        0,
        "floor",

        0.0,
        0.0,
        -0.01,

        floor_length_,
        floor_width_,
        floor_thickness_,

        makeColor(
          0.15f,
          0.45f,
          0.25f,
          1.0f)));
  }


  // ==========================================================
  // Court markings
  // ==========================================================

  void addCourtLines(
    visualization_msgs::msg::MarkerArray & markers)
  {
    const auto white =
      makeColor(
        1.0f,
        1.0f,
        1.0f,
        1.0f);

    /*
     * Gazebo:
     *
     * <pose> ... 0.0005 ... </pose>
     * <size> ... ... 0.001 </size>
     */
    constexpr double z = 0.0005;

    int id = 0;

    // ----------------------------------------------------------
    // Doubles sidelines
    //
    // Gazebo uses length 13.44 because line width extends
    // beyond both baselines by 0.02 m.
    // ----------------------------------------------------------

    markers.markers.push_back(
      makeBox(
        id++,
        "court_lines",
        0.0,
        3.05,
        z,
        13.44,
        line_width_,
        line_thickness_,
        white));

    markers.markers.push_back(
      makeBox(
        id++,
        "court_lines",
        0.0,
        -3.05,
        z,
        13.44,
        line_width_,
        line_thickness_,
        white));

    // ----------------------------------------------------------
    // Singles sidelines
    // ----------------------------------------------------------

    markers.markers.push_back(
      makeBox(
        id++,
        "court_lines",
        0.0,
        2.59,
        z,
        13.40,
        line_width_,
        line_thickness_,
        white));

    markers.markers.push_back(
      makeBox(
        id++,
        "court_lines",
        0.0,
        -2.59,
        z,
        13.40,
        line_width_,
        line_thickness_,
        white));

    // ----------------------------------------------------------
    // Baselines
    //
    // Gazebo:
    // x = +/-6.70
    // width = 6.14
    // ----------------------------------------------------------

    markers.markers.push_back(
      makeBox(
        id++,
        "court_lines",
        6.70,
        0.0,
        z,
        line_width_,
        6.14,
        line_thickness_,
        white));

    markers.markers.push_back(
      makeBox(
        id++,
        "court_lines",
        -6.70,
        0.0,
        z,
        line_width_,
        6.14,
        line_thickness_,
        white));

    // ----------------------------------------------------------
    // Short service lines
    // ----------------------------------------------------------

    markers.markers.push_back(
      makeBox(
        id++,
        "court_lines",
        short_service_x_,
        0.0,
        z,
        line_width_,
        court_width_,
        line_thickness_,
        white));

    markers.markers.push_back(
      makeBox(
        id++,
        "court_lines",
        -short_service_x_,
        0.0,
        z,
        line_width_,
        court_width_,
        line_thickness_,
        white));

    // ----------------------------------------------------------
    // Long service lines
    // ----------------------------------------------------------

    markers.markers.push_back(
      makeBox(
        id++,
        "court_lines",
        long_service_x_,
        0.0,
        z,
        line_width_,
        court_width_,
        line_thickness_,
        white));

    markers.markers.push_back(
      makeBox(
        id++,
        "court_lines",
        -long_service_x_,
        0.0,
        z,
        line_width_,
        court_width_,
        line_thickness_,
        white));

    // ----------------------------------------------------------
    // Centre service lines
    //
    // Gazebo:
    //
    // positive:
    // center = +4.34
    // length = 4.72
    //
    // negative:
    // center = -4.34
    // length = 4.72
    // ----------------------------------------------------------

    markers.markers.push_back(
      makeBox(
        id++,
        "court_lines",
        4.34,
        0.0,
        z,
        4.72,
        line_width_,
        line_thickness_,
        white));

    markers.markers.push_back(
      makeBox(
        id++,
        "court_lines",
        -4.34,
        0.0,
        z,
        4.72,
        line_width_,
        line_thickness_,
        white));
  }


  // ==========================================================
  // Net poles
  // ==========================================================

  void addPoles(
    visualization_msgs::msg::MarkerArray & markers)
  {
    const auto pole_color =
      makeColor(
        0.15f,
        0.15f,
        0.15f,
        1.0f);

    for (int i = 0; i < 2; ++i)
    {
      auto pole = makeMarker(
        i,
        "net_poles",
        visualization_msgs::msg::Marker::CYLINDER);

      pole.pose.position.x =
        pole_x_;

      pole.pose.position.y =
        (i == 0)
        ? pole_y_
        : -pole_y_;

      pole.pose.position.z =
        pole_height_ / 2.0;

      pole.scale.x =
        pole_diameter_;

      pole.scale.y =
        pole_diameter_;

      pole.scale.z =
        pole_height_;

      pole.color =
        pole_color;

      markers.markers.push_back(
        pole);
    }
  }


  // ==========================================================
  // Net
  // ==========================================================

  void addNet(
    visualization_msgs::msg::MarkerArray & markers)
  {
    /*
     * Gazebo net_visual:
     *
     * pose:
     *   0 0 1.17
     *
     * size:
     *   0.005 6.10 0.76
     *
     * Therefore:
     *
     * bottom = 1.17 - 0.76/2 = 0.79
     * top    = 1.17 + 0.76/2 = 1.55
     */

    markers.markers.push_back(
      makeBox(
        0,
        "net",

        0.0,
        0.0,
        net_center_z_,

        net_thickness_,
        net_width_,
        net_height_,

        makeColor(
          0.05f,
          0.05f,
          0.05f,
          0.35f)));

    /*
     * White top tape.
     *
     * Gazebo:
     *
     * pose z = 1.5125
     * size   = 0.01 × 6.10 × 0.075
     */

    markers.markers.push_back(
      makeBox(
        0,
        "net_top_tape",

        0.0,
        0.0,
        net_tape_center_z_,

        net_tape_thickness_,
        net_tape_width_,
        net_tape_height_,

        makeColor(
          1.0f,
          1.0f,
          1.0f,
          1.0f)));
  }


  // ==========================================================
  // Publish
  // ==========================================================

  void publishCourt()
  {
    visualization_msgs::msg::MarkerArray markers;

    addFloor(markers);
    addCourtLines(markers);
    addPoles(markers);
    addNet(markers);

    publisher_->publish(markers);
  }


  // ==========================================================
  // ROS objects
  // ==========================================================

  rclcpp::Publisher<
    visualization_msgs::msg::MarkerArray>::SharedPtr
    publisher_;

  rclcpp::TimerBase::SharedPtr
    timer_;


  // ==========================================================
  // Parameters
  // ==========================================================

  std::string frame_id_;

  double floor_length_;
  double floor_width_;
  double floor_thickness_;

  double court_length_;
  double court_width_;
  double singles_width_;

  double line_width_;
  double line_thickness_;

  double short_service_x_;
  double long_service_x_;

  double pole_x_;
  double pole_y_;
  double pole_diameter_;
  double pole_height_;

  double net_thickness_;
  double net_width_;
  double net_height_;
  double net_center_z_;

  double net_tape_thickness_;
  double net_tape_width_;
  double net_tape_height_;
  double net_tape_center_z_;
};


int main(
  int argc,
  char ** argv)
{
  rclcpp::init(
    argc,
    argv);

  rclcpp::spin(
    std::make_shared<CourtVisualizer>());

  rclcpp::shutdown();

  return 0;
}