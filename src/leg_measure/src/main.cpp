#include "measure_node.hpp"

int main(int argc,char**argv)
{
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<LegMeasureNode>());
    rclcpp::shutdown();
    return 0;
}