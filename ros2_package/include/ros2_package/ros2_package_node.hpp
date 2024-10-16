#ifndef ROS2_PACKAGE_ROS2_PACKAGE_NODE_HPP_
#define ROS2_PACKAGE_ROS2_PACKAGE_NODE_HPP_

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include "ros2_package/types.hpp"

namespace ros2_package
{
using StringMsg = std_msgs::msg::String;
using SetBoolSrv = std_srvs::srv::SetBool;

/**
 * @brief ROS2Package node. Implements the functionality of <TODO:>
 *
 */
class ROS2Package : public rclcpp::Node
{
public:
  /**
   * @brief Construct a new ROS2Package object.
   *
   */
  explicit ROS2Package();

private:
  /**
   * @brief Subscription callback routine responsible for <TODO:>
   *
   */
  void subscriptionCb(const StringMsg::UniquePtr msg);

  /**
   * @brief Publish the <TODO:>
   *
   */
  void publishTopic() const;

  /**
   * @brief Service callback, allows to <TODO:>
   *
   * @param req Service request <TODO:>
   * @param res Service response <TODO:>
   */
  void serviceCb(
    const SetBoolSrv::Request::SharedPtr req, const SetBoolSrv::Response::SharedPtr res);

  /**
   * @brief Call <TODO:> service
   *
   */
  void callService();

  /**
   * @brief Timer callback routine responsible for <TODO:>
   *
   */
  void timerCb() const;

  /// @brief Timeout for receiving the response form service server.
  std::chrono::milliseconds server_response_timeout_;

  /// @brief Subscription of the <TODO:>
  rclcpp::Subscription<StringMsg>::SharedPtr sub_;

  /// @brief Publisher of the <TODO:>
  rclcpp::Publisher<StringMsg>::SharedPtr pub_;

  /// @brief Service server for <TODO:>
  rclcpp::Service<SetBoolSrv>::SharedPtr srv_;

  /// @brief Service client for <TODO:>
  rclcpp::Client<SetBoolSrv>::SharedPtr client_;

  /// @brief Timer ticking a <TODO:> routine
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace ros2_package

#endif  // ROS2_PACKAGE_ROS2_PACKAGE_NODE_HPP_