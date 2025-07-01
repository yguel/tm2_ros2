#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "tm_msgs/srv/send_script.hpp"
#include "tm_msgs/msg/sct_response.hpp"
#include "tm_msgs/srv/go_to_relative_position.hpp"

#include <chrono>
#include <cstdlib>

#include <sstream>
#include <string>

using namespace std::chrono_literals;

using std::placeholders::_1;
using std::placeholders::_2;
typedef typename tm_msgs::srv::SendScript SendScript;

class RelativeMotionServiceNode : public rclcpp::Node
{
public:
  RelativeMotionServiceNode()
      : Node("Omron_relative_motion_service")
  {
    m_send_script_client = this->create_client<tm_msgs::srv::SendScript>("send_script");
    if (!m_send_script_client->wait_for_service(1s))
    {
      RCLCPP_ERROR_STREAM(
          rclcpp::get_logger("rclcpp"), "Service 'send_script' not available, exiting.");
      rclcpp::shutdown();
      return;
    }

    auto do_nothing = [](tm_msgs::msg::SctResponse::UniquePtr)
    { assert(false); };

    m_sct_response_subscription = this->create_subscription<tm_msgs::msg::SctResponse>(
        "sct_response", 10, do_nothing);

    // Add our subscription to our wait_set
    m_wait_set.add_subscription(m_sct_response_subscription);

    // Create service server for relative motion commands
    m_relative_motion_service = this->create_service<tm_msgs::srv::GoToRelativePosition>(
        "go_to_relative_position",
        std::bind(&RelativeMotionServiceNode::command_callback, this, _1, _2));
  }

  std::string create_id() const
  {
    unsigned int clamped_id = m_cmd_id_counter % 100;
    std::ostringstream id_ss;
    id_ss << "rm" << clamped_id;
    return id_ss.str();
  }

  bool send_cmd(const std::string &cmd)
  {
    auto request = std::make_shared<SendScript::Request>();
    m_last_cmd_id = create_id();
    request->id = m_last_cmd_id;
    request->script = cmd;

    while (!m_send_script_client->wait_for_service(1s))
    {
      if (!rclcpp::ok())
      {
        RCLCPP_ERROR_STREAM(
            rclcpp::get_logger(
                "rclcpp"),
            "Interrupted while waiting for the service. Exiting.");
        return false;
      }
      RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }

    m_send_script_client->async_send_request(request);
    // Increment the command ID counter
    m_cmd_id_counter++;

    // 2. Block until ready or timeout
    auto result = m_wait_set.wait(m_sct_timeout_ms);
    if (result.kind() != rclcpp::WaitResultKind::Ready)
    {
      RCLCPP_ERROR_STREAM(
          rclcpp::get_logger(
              "rclcpp"),
          "No SCT response after sending command. Timeout expired.");
      // timed out or interrupted
      return false;
    }

    // 3. “Take” the message out
    tm_msgs::msg::SctResponse incoming;
    rclcpp::MessageInfo msg_info;
    auto status = m_sct_response_subscription->take(incoming, msg_info);
    if (status)
    {
      // Decode sct message
      // Check that id is correct
      if (m_last_cmd_id != incoming.id)
      {
        std::string err = std::string("Wrong id in SCT response after sending command. Expected: ") + m_last_cmd_id + std::string(" got ") + incoming.id;
        RCLCPP_ERROR_STREAM(
            rclcpp::get_logger(
                "rclcpp"),
            err.c_str());
        return false;
      }
      if (incoming.script != std::string("OK"))
      {
        RCLCPP_ERROR_STREAM(
            rclcpp::get_logger(
                "rclcpp"),
            "SCT response after sending command is not OK: " + incoming.script);
        return false;
      }
      return true;
    }
    else
    {
      RCLCPP_ERROR_STREAM(
          rclcpp::get_logger(
              "rclcpp"),
          "Invalid SCT response after sending command.");
      return false;
    }
    return false;
  }

protected:
  void command_callback(
      const std::shared_ptr<tm_msgs::srv::GoToRelativePosition::Request> request,
      std::shared_ptr<tm_msgs::srv::GoToRelativePosition::Response> response)
  {
    // For command format see Software_Expression - Editor - and - Listen - Node_1 .84_Rev1 .00_EN.pdf page 252

    RCLCPP_INFO_STREAM(
        this->get_logger(), std::endl
                                << "x : " << request->x << std::endl
                                << "y : " << request->y << std::endl
                                << "z : " << request->z << std::endl
                                << "Rx_deg : " << request->rx_deg << std::endl
                                << "Ry_deg : " << request->ry_deg << std::endl
                                << "Rz_deg : " << request->rz_deg << std::endl
                                << "speed_percent : " << request->speed_percent << std::endl
                                << "frame : " << request->frame);

    std::string base = request->frame; //"LASER_ROS2";
    if (base != std::string(""))
    {
      std::string cmd = "ChangeBase(\"" + base + "\")";
      RCLCPP_INFO_STREAM(
          this->get_logger(), "Changing base to: " << base);
      // Send the command to change the base
      if (!send_cmd(cmd))
      {
        RCLCPP_ERROR_STREAM(
            this->get_logger(),
            "Failed to change base to: " << base);
        response->cmd_sent = false;
        return;
      }
      else
      {
        RCLCPP_INFO_STREAM(
            this->get_logger(),
            "Base changed to: " << base);
      }
    }

    std::ostringstream ss;
    // C: expressed in current base, P: Speed format as a percentage, P: blending format as a percentage
    ss << "Move_PTP(\"CPP\",";
    // relative motion parameters
    ss << request->x << "," << request->y << "," << request->z << ","
       << request->rx_deg << "," << request->ry_deg << "," << request->rz_deg << ",";
    // speed percentage
    ss << request->speed_percent << ",";
    // The time interval to accelerate to top speed (ms)
    ss << "200,";
    // blending percentage
    ss << "0,";
    // Disable precise positioning
    ss << "false)";

    // Log the command
    const std::string cmd = ss.str();
    RCLCPP_INFO_STREAM(
        this->get_logger(),
        "Relative motion command: " << ss.str());
    response->cmd_sent = send_cmd(cmd);
  }

protected:
  rclcpp::Subscription<tm_msgs::msg::SctResponse>::SharedPtr m_sct_response_subscription;

  rclcpp::Service<tm_msgs::srv::GoToRelativePosition>::SharedPtr m_relative_motion_service;

  rclcpp::Client<tm_msgs::srv::SendScript>::SharedPtr m_send_script_client;

  rclcpp::WaitSet m_wait_set;

  unsigned int m_cmd_id_counter = 0;
  std::string m_last_cmd_id;
  std::chrono::milliseconds m_sct_timeout_ms = 1000ms;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RelativeMotionServiceNode>());
  rclcpp::shutdown();
  return 0;
}
