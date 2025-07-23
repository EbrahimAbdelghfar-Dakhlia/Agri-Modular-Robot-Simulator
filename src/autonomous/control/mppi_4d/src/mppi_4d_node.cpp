#include "mppi_4d/mppi_4d.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto mppi_node = std::make_shared<controller::MPPI>();
    rclcpp::spin(mppi_node);
    rclcpp::shutdown();
    return 0;
}
