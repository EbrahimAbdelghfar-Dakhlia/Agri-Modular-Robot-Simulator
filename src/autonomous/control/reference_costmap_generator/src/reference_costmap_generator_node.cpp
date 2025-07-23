#include "reference_costmap_generator/reference_costmap_generator.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto reference_costmap_generator_node = std::make_shared<planning::ReferenceCostmapGenerator>();
    rclcpp::spin(reference_costmap_generator_node);
    rclcpp::shutdown();
    return 0;
}
