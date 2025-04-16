#include "rclcpp/rclcpp.hpp"
#include "spline_creator/path_publisher.hpp"
// If you’re no longer using the spline generator here explicitly, you can remove these.
#include "spline_creator/catmull_rom.hpp"
#include "spline_creator/point2d.hpp"

using namespace spline_creator;

int main(int argc, char ** argv)
{
    // Initialize the ROS 2 system.
    rclcpp::init(argc, argv);

    // Create an instance of our PathPublisher node.
    auto node = std::make_shared<PathPublisher>("spline_path_publisher");

    // The node now publishes the path every second via the internal timer,
    // so you don't need to compute the spline or call publishPath() here.

    // Keep the node alive so that ROS processes the published data.
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
