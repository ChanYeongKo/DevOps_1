#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "dxl_rapi5/dxl.hpp"
#include <memory>
#include <functional>

using std::placeholders::_1;

class DxlSubscriber : public rclcpp::Node
{
public:
    DxlSubscriber(Dxl& dxl)
    : Node("dxl_rapi5"), dxl_(dxl)
    {
        auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
        sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(
            "vel_cmd/topic", qos,
            std::bind(&DxlSubscriber::callback, this, _1));

        RCLCPP_INFO(this->get_logger(), "dxl_rapi5 node started. Waiting for topic_dxlpub...");
    }

private:
    void callback(const geometry_msgs::msg::Vector3::SharedPtr msg)
    {
        int rpm_left  = (int)msg->x;
        int rpm_right = (int)msg->y;

        RCLCPP_INFO(this->get_logger(),
            "Received: L=%d rpm, R=%d rpm", rpm_left, rpm_right);

        if (!dxl_.setVelocity(rpm_left, rpm_right)) {
            RCLCPP_ERROR(this->get_logger(), "setVelocity failed!");
        }
    }

    Dxl& dxl_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr sub_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    Dxl dxl;
    auto node = std::make_shared<rclcpp::Node>("dxl_rapi5_init");

    if (!dxl.open()) {
        RCLCPP_ERROR(node->get_logger(), "Dynamixel open failed!");
        rclcpp::shutdown();
        return -1;
    }

    auto dxl_node = std::make_shared<DxlSubscriber>(dxl);
    rclcpp::spin(dxl_node);

    dxl.close();
    rclcpp::shutdown();
    return 0;
}