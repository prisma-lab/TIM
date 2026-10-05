#pragma once

#include <primitive_manager/primitive_base.hpp>
#include <std_msgs/msg/string.hpp>

#include <chrono>
#include <map>
#include <memory>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

namespace ur10_hardware_primitives::services
{
// These are inferred task effects, not measurements from a grasp sensor.
// All service plugins in one manager share this state.
struct TaskState
{
  std::string arm_location;
  std::string held_object;
  std::map<std::string, std::string> placements;
};

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
  std::vector<primitive_manager::Observation> observe() const final;

protected:
  virtual void configure() = 0;
  virtual void dispatch(const std::vector<std::string> & args) = 0;
  virtual void record_completion() = 0;

  void require_location(const std::string & location) const;
  void require_object(const std::string & object) const;

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
  std::vector<std::string> locations_;
  std::vector<std::string> objects_;
  std::shared_ptr<TaskState> task_;

private:
  using Clock = std::chrono::steady_clock;
  void begin_request();
  void receive_response(bool accepted, const std::vector<int64_t> & ids);
  void receive_event(const std::string & text, bool start);
  void update_execution();
  void fail(const std::string & reason);

  bool active_{false};
  bool response_received_{false};
  bool ids_known_{false};
  bool faulted_{false};
  int64_t first_id_{0}, last_id_{0};
  double response_timeout_{5.0}, execution_timeout_{180.0};
  Clock::time_point requested_at_{};
  std::set<int64_t> starts_, ends_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr start_sub_, end_sub_;
};
}  // namespace ur10_hardware_primitives::services
