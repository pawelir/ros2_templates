#include "ros2_package/ros2_package_node.hpp"

namespace ros2_package
{

ROS2Package::ROS2Package() : Node("ros2_package")
{
  using namespace std::placeholders;

  declare_parameter("obligatory_parameter", rclcpp::ParameterType::PARAMETER_STRING);
  auto obligatory_param = get_parameter("obligatory_parameter").as_string();

  declare_parameter("timer_frequency", 10);
  auto timer_frequency = get_parameter("timer_frequency").as_int();

  declare_parameter("server_response_timeout_ms", 1000);
  server_response_timeout_ =
    std::chrono::milliseconds(get_parameter("server_response_timeout_ms").as_int());

  declare_parameter("parameter_family.parameter_1", false);
  auto family_param_1 = get_parameter("parameter_family.parameter_1").as_bool();

  sub_ = create_subscription<StringMsg>(
    "/namespace/topic_in", rclcpp::SystemDefaultsQoS(),
    std::bind(&ROS2Package::subscriptionCb, this, _1));

  pub_ =
    create_publisher<StringMsg>("~/topic_out", rclcpp::SystemDefaultsQoS().reliable().keep_last(1));

  srv_ = create_service<SetBoolSrv>(
    "~/set_bool", std::bind(&ROS2Package::serviceCb, this, _1, _2),
    rmw_qos_profile_services_default);

  client_ = create_client<SetBoolSrv>("/namespace/set_bool", rmw_qos_profile_services_default);

  timer_ = create_wall_timer(
    std::chrono::milliseconds(1000 / timer_frequency), std::bind(&ROS2Package::timerCb, this));

  RCLCPP_INFO(get_logger(), "Node is initialized");
};

void ROS2Package::subscriptionCb(const StringMsg::UniquePtr msg)
{
  RCLCPP_DEBUG(get_logger(), "Started subscriptionCb() callback routine");

  RCLCPP_DEBUG(get_logger(), "Finished subscriptionCb() callback routine");
}

void ROS2Package::publishTopic() const
{
  RCLCPP_DEBUG(get_logger(), "Started publishTopic() routine");

  auto msg = std::make_unique<StringMsg>();
  msg->data = "value";

  pub_->publish(std::move(msg));

  RCLCPP_DEBUG(get_logger(), "Finished publishTopic() routine");
}

void ROS2Package::serviceCb(
  const SetBoolSrv::Request::SharedPtr req, const SetBoolSrv::Response::SharedPtr res)
{
  RCLCPP_DEBUG(get_logger(), "Received request for '/set_bool' service");
  try {
    res->success = true;
    res->message = "";
  } catch (const std::exception& e) {
    res->success = false;
    res->message = e.what();
  }
  RCLCPP_DEBUG(get_logger(), "Finished '/set_bool' service");
}

void ROS2Package::callService()
{
  if (!client_->wait_for_service(std::chrono::seconds(1))) {
    throw std::runtime_error("Timeout on '/namespace/set_bool' - service unavailable!");
  }

  auto request = std::make_shared<SetBoolSrv::Request>();
  request->data = true;

  auto result = client_->async_send_request(request);
  RCLCPP_DEBUG(get_logger(), "Sent request on '/namespace/set_bool' service.");

  auto status = result.wait_for(server_response_timeout_);

  if (status != std::future_status::ready) {
    throw std::runtime_error("Timeout on '/namespace/set_bool' - service didn't response!");
  }

  if (result.get()->success) {
    RCLCPP_INFO(this->get_logger(), "Service <TODO:> call is successful!");
  } else {
    RCLCPP_WARN(
      this->get_logger(), "Service <TODO:> call is not successful: %s",
      result.get()->message.c_str());
  }
}

void ROS2Package::timerCb() const
{
  RCLCPP_DEBUG(get_logger(), "Started timerCb() callback routine");

  RCLCPP_DEBUG(get_logger(), "Finished timerCb() callback routine");
}

}  // namespace ros2_package