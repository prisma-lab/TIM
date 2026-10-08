#include <chrono>
#include <iomanip>
#include <map>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <pluginlib/class_loader.hpp>
#include <std_msgs/msg/string.hpp>
#include "primitive_manager/command.hpp"
#include "primitive_manager/primitive_base.hpp"

namespace primitive_manager
{
using Clock = std::chrono::steady_clock;

std::string json_string(const std::string & text)
{
  std::ostringstream out;
  out << '"';
  for (unsigned char ch : text) {
    if (ch == '"' || ch == '\\') {out << '\\' << ch;}
    else if (ch < 0x20) {out << "\\u" << std::hex << std::setw(4) << std::setfill('0') << int(ch);}
    else {out << ch;}
  }
  out << '"';
  return out.str();
}

class Manager : public rclcpp::Node
{
public:
  Manager() : Node("primitive_manager"), loader_("primitive_manager", "primitive_manager::PrimitiveBase")
  {
    const auto command_topic = declare_parameter("command_topic", "/ur10/gripper/command");
    const auto status_topic = declare_parameter("status_topic", "/ur10/gripper/status");
    const auto state_topic = declare_parameter("seed_state_topic", "/seed_ur10/state");
    failure_fact_ = declare_parameter("failure_fact", "primitives.failed");
    failure_aliases_ = declare_parameter("additional_failure_facts", std::vector<std::string>{});
    pause_primitive_name_ = declare_parameter("pause_primitive", std::string{});
    status_pub_ = create_publisher<std_msgs::msg::String>(status_topic, 100);
    state_pub_ = create_publisher<std_msgs::msg::String>(state_topic, 100);
    auto names = declare_parameter("primitives", std::vector<std::string>{});
    if (names.empty()) {throw std::invalid_argument("Configure at least one primitive");}
    for (const auto & name : names) {
      auto command = parse_command(name);
      if (command.text != name || !command.args.empty() || name == "reset" || name == "cancel" ||
        name == "stop" || plugins_.count(name)) {throw std::invalid_argument("Invalid/duplicate primitive: " + name);}
      const auto type = declare_parameter(name + ".plugin", std::string{});
      auto plugin = loader_.createSharedInstance(type);
      plugin->initialize(*this, name);
      plugins_.emplace(name, plugin);
      RCLCPP_INFO(get_logger(), "Loaded %s -> %s", name.c_str(), type.c_str());
    }
    if (!pause_primitive_name_.empty()) {
      auto pause = plugins_.find(pause_primitive_name_);
      if (pause == plugins_.end()) {
        throw std::invalid_argument("The configured pause primitive must be loaded");
      }
      pause_primitive_ = pause->second;
      for (const auto & item : plugins_) {
        if (item.first != pause_primitive_name_ && !item.second->supports_pause()) {
          throw std::invalid_argument("Primitive does not support external pause: " + item.first);
        }
      }
    }
    auto topics = declare_parameter("command_alias_topics", std::vector<std::string>{});
    topics.push_back(command_topic);
    for (const auto & topic : topics) {
      command_subs_.push_back(create_subscription<std_msgs::msg::String>(topic, 10,
        [this](std_msgs::msg::String::ConstSharedPtr msg) {on_command(msg->data);}));
    }
    timer_ = create_wall_timer(std::chrono::milliseconds(50), [this]() {tick();});
    RCLCPP_INFO(get_logger(), "Listening on %s", command_topic.c_str());
  }

private:
  void report(const std::string & command, const std::string & status, const std::string & detail,
    bool log = true)
  {
    std_msgs::msg::String msg;
    msg.data = "{\"command\":" + json_string(command) + ",\"status\":" + json_string(status) +
      ",\"detail\":" + json_string(detail) + "}";
    status_pub_->publish(msg);
    if (log) {RCLCPP_INFO(get_logger(), "%s: %s: %s", command.c_str(), status.c_str(), detail.c_str());}
  }

  void on_command(const std::string & text)
  {
    Command command;
    try {command = parse_command(text);}
    catch (const std::exception & error) {report(text, "rejected", error.what()); return;}

    std::cout << "";
    std::cout << "Received command: " << command.name << std::endl;
    for (const auto & arg : command.args) {std::cout << "  Arg: " << arg << std::endl;}

    // The pause service must be reachable while an ordinary request is active
    // or failed. It has separate execution tracking from the interrupted step.
    if (pause_primitive_ && command.name == pause_primitive_name_) {
      request_pause(command);
      return;
    }
    if (command.name == "reset") {
      reset_execution(command);
      return;
    }
    if (pause_request_active_ || pause_unconfirmed_) {
      report(command.text, "rejected", "Safe stop has not been confirmed");
      return;
    }

    if (command.name == "cancel" || command.name == "stop") {
      if (!command.args.empty()) {report(command.text, "rejected", "This command takes no arguments"); return;}
      if (active_) {
        active_->cancel();
        report(command.text, "accepted", "Waiting for the active controller command to end");
        update_active();
      } else {report(command.text, "succeeded", "No active command");}
      publish_states();
      return;
    }
    if (active_) {
      if (command.text != active_command_) {report(command.text, "rejected", "Another command is still active");}
      return;
    }
    auto found = plugins_.find(command.name);
    if (found == plugins_.end()) {report(command.text, "rejected", "Unknown primitive"); return;}
    if (failed_) {report(command.text, "rejected", "Send reset after resolving the failure", false); return;}
    // SEED repeats rosAct commands. A terminal result stays latched until a
    // different command or explicit reset; identical commands never restart it.
    const bool resumes_paused_request = command.text == paused_command_;
    if (command.text == last_command_ && !resumes_paused_request) {
      report(command.text, status_name(last_status_), "Previous invocation already ended; reset to repeat", false);
      return;
    }
    active_ = found->second;
    active_command_ = command.text;
    published_status_ = Status::IDLE;
    published_feedback_.clear();
    execution_[command.text] = Status::IDLE;
    if (pause_primitive_) {
      // A new normal invocation makes the previous stop result obsolete.
      execution_[pause_primitive_name_] = Status::IDLE;
    }
    paused_command_.clear();
    active_->reset();
    report(command.text, "accepted", "Dispatching to plugin " + command.name);
    const bool started = active_->execute(command.args);
    if (!started && active_->status() != Status::FAILED) {
      throw std::logic_error("Plugin execute(false) must report FAILED");
    }
    update_active();
    publish_states();
  }

  void request_pause(const Command & command)
  {
    if (!command.args.empty()) {
      report(command.text, "rejected", "Safe stop takes no arguments");
      return;
    }
    if (pause_request_active_) {
      return;  // SEED repeats rosAct while waiting for its completion goal.
    }
    auto previous = execution_.find(command.text);
    if (previous != execution_.end() &&
      (previous->second == Status::SUCCEEDED || previous->second == Status::FAILED)) {
      report(command.text, status_name(previous->second), "Previous safe-stop request already ended", false);
      return;
    }

    // Preserve a motion that finished before this stop request was received.
    update_active();
    if (active_) {
      active_->prepare_pause();
      update_active();
    }
    pause_unconfirmed_ = true;
    pause_request_active_ = true;
    published_pause_status_ = Status::IDLE;
    published_pause_feedback_.clear();
    execution_[command.text] = Status::IDLE;
    pause_primitive_->reset();
    report(command.text, "accepted", "Requesting safe stop without discarding the SEED sequence");
    const bool started = pause_primitive_->execute(command.args);
    if (!started && pause_primitive_->status() != Status::FAILED) {
      throw std::logic_error("Plugin execute(false) must report FAILED");
    }
    update_pause();
    publish_states();
  }

  void update_pause()
  {
    if (!pause_request_active_) {
      return;
    }
    const auto status = pause_primitive_->status();
    if (status == Status::FAILED) {
      failed_ = true;
    }
    if (status != published_pause_status_ || pause_primitive_->feedback() != published_pause_feedback_) {
      execution_[pause_primitive_name_] = status;
      report(pause_primitive_name_, status_name(status), pause_primitive_->feedback());
      published_pause_status_ = status;
      published_pause_feedback_ = pause_primitive_->feedback();
    }
    if (pause_primitive_->busy()) {
      return;
    }

    pause_request_active_ = false;
    if (status == Status::SUCCEEDED) {
      pause_unconfirmed_ = false;
      if (active_) {
        // Only the unfinished service request is cancelled. Earlier sequence
        // steps and their execution records remain completed.
        paused_command_ = active_command_;
        active_->confirm_pause();
        update_active();
      }
    }
    pause_primitive_->reset();
  }

  void reset_execution(const Command & command)
  {
    if (!command.args.empty()) {
      report(command.text, "rejected", "Reset takes no arguments");
      return;
    }
    if (pause_request_active_) {
      report(command.text, "rejected", "The safe-stop response is still pending");
      return;
    }
    if (pause_unconfirmed_) {
      // A failed stop can be retried explicitly. The original request remains
      // held, and normal execution stays blocked until stopping is confirmed.
      pause_primitive_->reset();
      execution_[pause_primitive_name_] = Status::IDLE;
      failed_ = false;
      report(command.text, "succeeded", "Safe stop can be retried; the unfinished request remains held");
      publish_states();
      return;
    }
    if (active_) {
      report(command.text, "rejected", "Another command is still active");
      return;
    }
    failed_ = false;
    last_command_.clear();
    paused_command_.clear();
    for (auto & item : plugins_) {
      item.second->reset();
    }
    for (auto & item : execution_) {
      item.second = Status::IDLE;
    }
    report(command.text, "succeeded", "Failure and execution results cleared; no motion commanded");
    publish_states();
  }

  void update_active()
  {
    if (!active_) {return;}
    const auto status = active_->status();
    if (status == Status::FAILED) {failed_ = true;}
    if (status != published_status_ || active_->feedback() != published_feedback_) {
      execution_[active_command_] = status;
      report(active_command_, status_name(status), active_->feedback());
      published_status_ = status;
      published_feedback_ = active_->feedback();
    }
    if (!pause_request_active_ && !pause_unconfirmed_ && !active_->busy() &&
      (status == Status::SUCCEEDED || status == Status::FAILED || status == Status::CANCELLED)) {
      last_command_ = active_command_;
      last_status_ = status;
      active_->reset();
      active_.reset();
      active_command_.clear();
    }
  }

  void tick()
  {
    if (active_) {active_->tick(); update_active();}
    if (pause_request_active_) {
      pause_primitive_->tick();
      update_pause();
    }
    publish_states();
  }

  void publish_states()
  {
    std::map<std::string, bool> facts;
    facts[failure_fact_] = failed_;
    for (const auto & fact : failure_aliases_) {facts[fact] = failed_;}
    for (const auto & item : plugins_) {
      for (const auto & observation : item.second->observe()) {
        // An idle safe stop does not undo a previously completed location.
        // An interrupted request stays in active_ until stop confirmation.
        facts[observation.fact] = !active_ && !failed_ && observation.value;
      }
    }
    for (const auto & item : execution_) {
      for (auto state : {Status::RUNNING, Status::CANCELLING, Status::SUCCEEDED, Status::FAILED, Status::CANCELLED}) {
        facts[status_name(state) + "(" + item.first + ")"] = item.second == state;
      }
    }
    const auto now = Clock::now();
    const bool heartbeat = now - last_heartbeat_ >= std::chrono::seconds(1);
    for (const auto & fact : facts) {
      auto old = last_facts_.find(fact.first);
      if (heartbeat || old == last_facts_.end() || old->second != fact.second) {
        std_msgs::msg::String msg;
        msg.data = (fact.second ? "" : "-") + fact.first;
        state_pub_->publish(msg);
      }
    }
    last_facts_ = std::move(facts);
    if (heartbeat) {last_heartbeat_ = now;}
  }

  // Declared first: loader outlives every plugin and its callbacks.
  pluginlib::ClassLoader<PrimitiveBase> loader_;
  std::map<std::string, std::shared_ptr<PrimitiveBase>> plugins_;
  std::shared_ptr<PrimitiveBase> active_;
  std::shared_ptr<PrimitiveBase> pause_primitive_;
  std::string pause_primitive_name_;
  std::string paused_command_;
  bool pause_request_active_{false};
  bool pause_unconfirmed_{false};
  Status published_pause_status_{Status::IDLE};
  std::string published_pause_feedback_;
  std::string active_command_, last_command_, failure_fact_;
  Status published_status_{Status::IDLE}, last_status_{Status::IDLE};
  std::string published_feedback_;
  bool failed_{false};
  std::map<std::string, Status> execution_;
  std::map<std::string, bool> last_facts_;
  Clock::time_point last_heartbeat_{};
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_, state_pub_;
  std::vector<std::string> failure_aliases_;
  std::vector<rclcpp::Subscription<std_msgs::msg::String>::SharedPtr> command_subs_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace primitive_manager

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {rclcpp::spin(std::make_shared<primitive_manager::Manager>());}
  catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("primitive_manager"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
