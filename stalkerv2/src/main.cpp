#include "stalkerv2/PersonFollower.cpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<stalkerv2::PersonFollower>());
    rclcpp::shutdown();
    return 0;
}