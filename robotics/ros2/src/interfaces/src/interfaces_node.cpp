/*! @package interfaces
    Code Information:
        Maintainer: Eng. Davidson Daniel Rojas Cediel
        Mail: davidson@kiwibot.com
        Kiwi Campus / AI & Robotics Team
*/

/* Submodules */

#include "modules/speaker.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    rclcpp::executors::SingleThreadedExecutor executor;

    auto speaker_node = std::make_shared<Speaker>(options);

    uint8_t time = 100;
    executor.add_node(speaker_node);

    auto period = std::chrono::milliseconds(time);
    rclcpp::Rate r(period);
    while (rclcpp::ok())
    {
        speaker_node->publishData();
        executor.spin_some();
        r.sleep();
    }
    rclcpp::shutdown();

    return 0;
}
