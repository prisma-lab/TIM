#pragma once

#include <string>
#include <stdexcept>
#include <vector>
#include <rclcpp/rclcpp.hpp>

namespace primitive_manager
{
enum class Status {IDLE, RUNNING, CANCELLING, SUCCEEDED, FAILED, CANCELLED};

inline std::string status_name(Status status)
{
  switch (status) {
    case Status::RUNNING: return "running";
    case Status::CANCELLING: return "cancelling";
    case Status::SUCCEEDED: return "succeeded";
    case Status::FAILED: return "failed";
    case Status::CANCELLED: return "cancelled";
    default: return "idle";
  }
}

struct Observation
{
  std::string fact;
  bool value;
};

// The manager owns plugins and outlives them. All methods and ROS callbacks run
// in its single-threaded executor. execute(), tick(), and cancel() must not block.
class PrimitiveBase
{
public:
  virtual ~PrimitiveBase() = default;
  virtual void initialize(rclcpp::Node & node, const std::string & name) = 0;
  virtual bool execute(const std::vector<std::string> & args) = 0;
  virtual void tick() = 0;
  virtual void cancel() = 0;
  virtual void reset() = 0;

  // Failure may be reported before a controller confirms cancellation. The
  // manager must retain the active plugin while busy(), even if status=FAILED.
  virtual bool busy() const = 0;
  virtual std::vector<Observation> observe() const {return {};}

  // A separate stop service can cancel the current remote request. These hooks
  // suppress its completion while stopping, then retire only that request.
  virtual bool supports_pause() const {return false;}
  virtual void prepare_pause()
  {
    throw std::logic_error("This primitive does not support an external pause");
  }
  virtual void confirm_pause()
  {
    throw std::logic_error("This primitive does not support an external pause");
  }

  Status status() const {return status_;}
  const std::string & feedback() const {return feedback_;}

protected:
  Status status_{Status::IDLE};
  std::string feedback_;
};
}  // namespace primitive_manager
