#ifndef CATMULL_ROM_PATH_PUBLISHER_PATH_PUBLISHER_HPP
#define CATMULL_ROM_PATH_PUBLISHER_PATH_PUBLISHER_HPP

#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "spline_creator/point2d.hpp"

namespace spline_creator {

/**
 * @brief A ROS node for publishing a Catmull–Rom spline path.
 *
 * The PathPublisher class is responsible for converting a vector of 2D points (represented by Point2D)
 * into a nav_msgs::msg::Path message and publishing it over a designated topic. The path is published
 * in the specified coordinate frame (default "map").
 */
class PathPublisher : public rclcpp::Node {
public:
    /**
     * @brief Construct a new Path Publisher object.
     * @param node_name The name of this ROS node. Defaults to "path_publisher".
     */
    explicit PathPublisher(const std::string & node_name = "path_publisher");

    /**
     * @brief Publishes the spline path constructed from the provided 2D control points.
     *
     * This function takes in a vector of Point2D objects (which represent the interpolated Catmull–Rom spline)
     * and converts them into a nav_msgs::msg::Path message. The path is then published on a pre-configured topic.
     *
     * @param points A vector of 2D points representing the discretized spline.
     * @param frame_id The coordinate frame id in which the path is defined. Default is "map".
     */
    void publishPath(const std::vector<Point2D>& points, const std::string& frame_id = "map");

private:
    // The ROS publisher for nav_msgs::msg::Path messages.
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

};

} // namespace spline_creator

#endif // CATMULL_ROM_PATH_PUBLISHER_PATH_PUBLISHER_HPP
