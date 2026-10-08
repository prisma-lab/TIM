#include "primitive_manager/primitive_base.hpp"

#include <pluginlib/class_list_macros.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace ur10_hardware_primitives::services
{
class SafeStopPrimitive : public primitive_manager::PrimitiveBase
{
public:
  void initialize(rclcpp::Node & node, const std::string & name) override
  {
    name_ = name;

    if (!node.has_parameter("safe_stop.service")) {
      node.declare_parameter<std::string>("safe_stop.service", "/safe_stop");
    }
    service_name_ = node.get_parameter("safe_stop.service").as_string();
    if (service_name_.empty()) {
      throw std::invalid_argument("safe_stop.service must not be empty");
    }

    if (!node.has_parameter("safe_stop.response_timeout")) {
      node.declare_parameter<double>("safe_stop.response_timeout", 5.0);
    }
    response_timeout_seconds_ = node.get_parameter("safe_stop.response_timeout").as_double();
    if (!std::isfinite(response_timeout_seconds_) || response_timeout_seconds_ <= 0.0) {
      throw std::invalid_argument("safe_stop.response_timeout must be positive and finite");
    }

    client_ = node.create_client<Trigger>(service_name_);
  }

  bool execute(const std::vector<std::string> & args) override
  {
    if (response_pending_) {
      throw std::logic_error("A safe-stop request is already awaiting its response");
    }
    if (!args.empty()) {
      status_ = primitive_manager::Status::FAILED;
      feedback_ = name_ + " takes no arguments";
      return false;
    }
    if (!client_->service_is_ready()) {
      status_ = primitive_manager::Status::FAILED;
      feedback_ = "Service unavailable: " + service_name_;
      return false;
    }

    auto request = std::make_shared<Trigger::Request>();
    request_started_at_ = std::chrono::steady_clock::now();
    response_pending_ = true;
    timeout_reported_ = false;
    status_ = primitive_manager::Status::RUNNING;
    feedback_ = "Waiting for safe-stop confirmation from " + service_name_;

    try {
      client_->async_send_request(request, [this](TriggerClient::SharedFuture response_future) {
          receive_response(response_future);
        });
    } catch (const std::exception & error) {
      response_pending_ = false;
      status_ = primitive_manager::Status::FAILED;
      feedback_ = "Could not send safe-stop request: " + std::string(error.what());
      return false;
    }
    return true;
  }

  void tick() override
  {
    if (!response_pending_ || timeout_reported_) {
      return;
    }
    const auto elapsed = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - request_started_at_).count();
    if (elapsed < response_timeout_seconds_) {
      return;
    }

    // A timeout cannot tell us whether the provider stopped the robot. Retain
    // the pending request so a later response can still confirm its outcome.
    timeout_reported_ = true;
    status_ = primitive_manager::Status::FAILED;
    feedback_ = "Safe-stop response timed out; stop outcome is unknown, still awaiting response";
  }

  void cancel() override
  {
    if (response_pending_) {
      feedback_ = "An outstanding safe-stop service request cannot be cancelled; "
        "still awaiting its response";
    }
  }

  void reset() override
  {
    if (response_pending_) {
      throw std::logic_error("Cannot reset while the safe-stop response is pending");
    }
    status_ = primitive_manager::Status::IDLE;
    feedback_.clear();
    timeout_reported_ = false;
  }

  bool busy() const override
  {
    return response_pending_;
  }

private:
  using Trigger = std_srvs::srv::Trigger;
  using TriggerClient = rclcpp::Client<Trigger>;

  void receive_response(const TriggerClient::SharedFuture & response_future)
  {
    response_pending_ = false;
    try {
      const auto response = response_future.get();
      if (response->success) {
        status_ = primitive_manager::Status::SUCCEEDED;
        feedback_ = "Safe stop confirmed";
      } else {
        status_ = primitive_manager::Status::FAILED;
        feedback_ = "Safe-stop service did not confirm stopping";
      }
      if (!response->message.empty()) {
        feedback_ += ": " + response->message;
      }
    } catch (const std::exception & error) {
      status_ = primitive_manager::Status::FAILED;
      feedback_ = "Safe-stop service response failed: " + std::string(error.what());
    }
  }

  std::string name_;
  std::string service_name_;
  TriggerClient::SharedPtr client_;
  double response_timeout_seconds_{5.0};
  std::chrono::steady_clock::time_point request_started_at_;
  bool response_pending_{false};
  bool timeout_reported_{false};
};
}  // namespace ur10_hardware_primitives::services

PLUGINLIB_EXPORT_CLASS(
  ur10_hardware_primitives::services::SafeStopPrimitive,
  primitive_manager::PrimitiveBase)
