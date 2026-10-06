#pragma once

#include <primitive_manager/primitive_base.hpp>

#include <chrono>
#include <cstdint>
#include <memory>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

namespace ur10_hardware_primitives::services
{
template<typename T>
T parameter(rclcpp::Node & node, const std::string & name, const T & default_value)
{
  if (!node.has_parameter(name)) {
    node.declare_parameter<T>(name, default_value);
  }
  return node.get_parameter(name).get_value<T>();
}

// Common asynchronous protocol: request -> ordered motion IDs -> final end event.
// No blocking waits or nested executors: the manager owns the ROS executor.
class MotionServicePrimitive : public primitive_manager::PrimitiveBase
{
public:
  void initialize(rclcpp::Node & node, const std::string & name) final;
  bool execute(const std::vector<std::string> & args) final;
  void tick() final;
  void cancel() final;
  void reset() final;
  bool busy() const final {return active_;}

protected:
  virtual void configure() = 0;
  virtual void dispatch(const std::vector<std::string> & args) = 0;
  virtual void record_completion() = 0;

  template<typename Service>
  void send_request(
    const typename rclcpp::Client<Service>::SharedPtr & client,
    const typename Service::Request::SharedPtr & request)
  {
    if (!client->service_is_ready()) {
      throw std::runtime_error("Service unavailable: " + std::string(client->get_service_name()));
    }
    begin_request();
    client->async_send_request(request,
      [this](typename rclcpp::Client<Service>::SharedFuture future) {
        try {
          const auto response = future.get();
          receive_response(response->success, response->motion_ids);
        } catch (const std::exception & error) {
          fail(std::string("Service response error: ") + error.what());
        }
      });
  }

  rclcpp::Node * node_{nullptr};
  std::string name_;

private:
  using Clock = std::chrono::steady_clock;
  rclcpp::SubscriptionBase::SharedPtr subscribe_to_events(
    const std::string & topic, const std::string & message_type, bool start);
  void begin_request();
  void receive_response(bool accepted, const std::vector<std::uint64_t> & ids);
  void receive_string_event(const std::string & text, bool start);
  void receive_event(std::uint64_t id, bool start);
  void update_execution();
  void fail(const std::string & reason);

  bool active_{false};
  bool response_received_{false};
  bool ids_known_{false};
  bool faulted_{false};
  std::uint64_t first_id_{0}, last_id_{0};
  double response_timeout_{5.0}, execution_timeout_{180.0};
  Clock::time_point requested_at_{};
  std::set<std::uint64_t> starts_, ends_;
  rclcpp::SubscriptionBase::SharedPtr start_sub_, end_sub_;
};
}  // namespace ur10_hardware_primitives::services
