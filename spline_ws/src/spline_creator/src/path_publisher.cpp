#include "spline_creator/path_publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "spline_creator/catmull_rom.hpp"  // Ensure you include this for the interpolation function.
#include "spline_creator/point2d.hpp"

namespace spline_creator {

PathPublisher::PathPublisher(const std::string & node_name)
    : rclcpp::Node(node_name)
{
    // Create a publisher for nav_msgs::msg::Path messages on the "spline_path" topic.
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("spline_path", 10);

    // Set up a timer to publish the path every second.
    timer_ = this->create_wall_timer(std::chrono::seconds(1), [this]() {
        // Define the 4 required control points.
        std::vector<Point2D> control_points{
            {0.0, 0.0},   // P0
            {1.0, 2.0},   // P1
            {4.0, 2.0},   // P2
            {5.0, 0.0}    // P3
        };

        // Use the interpolation function to compute the smooth spline.
        // This should generate many points (depending on resolution) between P1 and P2.
        std::vector<Point2D> spline = CatmullRomSpline::interpolateSegment(
            control_points[0],
            control_points[1],
            control_points[2],
            control_points[3],
            0.1  // resolution in meters
        );

        // Publish the computed spline path instead of the raw control points.
        this->publishPath(spline, "map");

        RCLCPP_INFO(this->get_logger(), "Published path with %zu poses", spline.size());
    });
}

void PathPublisher::publishPath(const std::vector<Point2D>& points, const std::string& frame_id)
{
    nav_msgs::msg::Path path_msg;
    // Set header fields including current time and coordinate frame.
    path_msg.header.stamp = this->now();
    path_msg.header.frame_id = frame_id;

    // Convert each Point2D to a PoseStamped message.
    for (const auto & point : points) {
        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header.stamp = path_msg.header.stamp;
        pose_stamped.header.frame_id = frame_id;
        pose_stamped.pose.position.x = point.x;
        pose_stamped.pose.position.y = point.y;
        pose_stamped.pose.position.z = 0.0;  // 2D path: z set to zero.
        // Default orientation (no rotation).
        pose_stamped.pose.orientation.w = 1.0;
        pose_stamped.pose.orientation.x = 0.0;
        pose_stamped.pose.orientation.y = 0.0;
        pose_stamped.pose.orientation.z = 0.0;
        path_msg.poses.push_back(pose_stamped);
    }

    // Publish the path.
    path_pub_->publish(path_msg);
    RCLCPP_INFO(this->get_logger(), "Published path with %zu poses", path_msg.poses.size());
}

}  // namespace spline_creator
