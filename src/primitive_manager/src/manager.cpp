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
    if (command.name == "reset") {
      if (!command.args.empty()) {report(command.text, "rejected", "Reset takes no arguments"); return;}
      failed_ = false;
      last_command_.clear();
      for (auto & item : plugins_) {item.second->reset();}
      for (auto & item : execution_) {item.second = Status::IDLE;}
      report(command.text, "succeeded", "Failure and execution results cleared; no motion commanded");
      publish_states();
      return;
    }
    auto found = plugins_.find(command.name);
    if (found == plugins_.end()) {report(command.text, "rejected", "Unknown primitive"); return;}
    if (failed_) {report(command.text, "rejected", "Send reset after resolving the failure", false); return;}
    // SEED repeats rosAct commands. A terminal result stays latched until a
    // different command or explicit reset; identical commands never restart it.
    if (command.text == last_command_) {
      report(command.text, status_name(last_status_), "Previous invocation already ended; reset to repeat", false);
      return;
    }
    active_ = found->second;
    active_command_ = command.text;
    published_status_ = Status::IDLE;
    execution_[command.text] = Status::IDLE;
    active_->reset();
    report(command.text, "accepted", "Dispatching to plugin " + command.name);
    const bool started = active_->execute(command.args);
    if (!started && active_->status() != Status::FAILED) {
      throw std::logic_error("Plugin execute(false) must report FAILED");
    }
    update_active();
    publish_states();
  }

  void update_active()
  {
    if (!active_) {return;}
    const auto status = active_->status();
    if (status == Status::FAILED) {failed_ = true;}
    if (status != published_status_) {
      execution_[active_command_] = status;
      report(active_command_, status_name(status), active_->feedback());
      published_status_ = status;
    }
    if (!active_->busy() && (status == Status::SUCCEEDED || status == Status::FAILED || status == Status::CANCELLED)) {
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
    publish_states();
  }

  void publish_states()
  {
    std::map<std::string, bool> facts;
    facts[failure_fact_] = failed_;
    for (const auto & fact : failure_aliases_) {facts[fact] = failed_;}
    for (const auto & item : plugins_) {
      for (const auto & observation : item.second->observe()) {
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
  std::string active_command_, last_command_, failure_fact_;
  Status published_status_{Status::IDLE}, last_status_{Status::IDLE};
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
