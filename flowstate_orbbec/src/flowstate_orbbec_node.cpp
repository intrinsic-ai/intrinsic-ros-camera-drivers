#include "rclcpp/rclcpp.hpp"
#include "orbbec_camera/ob_camera_node_driver.h"
#include "snapshot_interfaces/srv/discover.hpp"

#include <memory>

#if 0
void discover(const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
          std::shared_ptr<example_interfaces::srv::AddTwoInts::Response>      response)
{
  response->sum = request->a + request->b;
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\na: %ld" " b: %ld",
                request->a, request->b);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: [%ld]", (long int)response->sum);
}
#endif

using snapshot_interfaces::srv::Discover;

class FlowstateOrbbecNode : public rclcpp::Node {
 public:
  FlowstateOrbbecNode() : Node("flowstate_orbbec") {
    discover_service_ = create_service<Discover>(
      "/cameras/discover",
      [this](const std::shared_ptr<rmw_request_id_t>,
             const std::shared_ptr<Discover::Request>,
             const std::shared_ptr<Discover::Response> response) {
        RCLCPP_INFO(get_logger(), "Discover service called");
        ob::Context::setLoggerSeverity(OBLogSeverity::OB_LOG_SEVERITY_OFF);
        auto context = std::make_unique<ob::Context>();
        auto list = context->queryDeviceList();
        for (size_t i = 0; i < list->deviceCount(); i++) {
          if (std::string(list->getConnectionType(i)) != std::string("Ethernet")) {
            continue;
          }
          std::string serial = list->serialNumber(i);
          RCLCPP_INFO(get_logger(), "Found Orbbec device: %s", serial.c_str());
        }

        response->success = true;
      });
  }

 private:
  rclcpp::Service<Discover>::SharedPtr discover_service_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  std::shared_ptr<FlowstateOrbbecNode> node = std::make_shared<FlowstateOrbbecNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
}
