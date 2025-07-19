#include "mppi_h/mppi_h.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<controller_mppi_h::MPPI>();

    // Use multi-threaded executor for better performance in control applications
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    
    rclcpp::shutdown();
    return 0;
};
