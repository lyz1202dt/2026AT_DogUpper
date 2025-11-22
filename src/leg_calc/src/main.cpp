#include <rclcpp/rclcpp.hpp>
#include "mynode.h"
#include "param.h"

LegParam_t left_back_param;

int main(int argc,char** argv)
{
    rclcpp::init(argc,argv);
    LeftBackLegParamInit(&left_back_param);
    rclcpp::spin(std::make_shared<LegControl>(left_back_param,"left_back_leg"));
    rclcpp::shutdown();
    return 0;
}
