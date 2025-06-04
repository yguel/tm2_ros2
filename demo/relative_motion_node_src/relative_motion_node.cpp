#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "tm_msgs/srv/send_script.hpp"
#include "tm_msgs/msg/svr_response.hpp"
#include "tm_msgs/srv/go_to_relative_position.hpp"

#include <chrono>
#include <cstdlib>

#include <sstream>
#include <string>


using namespace std::chrono_literals;

using std::placeholders::_1;
using std::placeholders::_2;
typedef typename tm_msgs::srv::SendScript SendScript;

class RelativeMotionNode : public rclcpp::Node
{
public:
  RelativeMotionNode()
  : Node("Omron_relative_motion")
  {
    m_send_script_client = this->create_client<tm_msgs::srv::SendScript>("send_script");
    if (!m_send_script_client->wait_for_service(1s)) {
      RCLCPP_ERROR_STREAM(
        rclcpp::get_logger("rclcpp"), "Service 'send_script' not available, exiting.");
      rclcpp::shutdown();
      return;
    }

    m_svr_response_subscription = this->create_subscription<tm_msgs::msg::SvrResponse>(
      "svr_response", 10, std::bind(&RelativeMotionNode::svr_response_callback, this, _1));

    // Create service server for relative motion commands
    m_relative_motion_service = this->create_service<tm_msgs::srv::GoToRelativePosition>(
      "go_to_relative_position",
      std::bind(&RelativeMotionNode::command_callback, this, _1, _2));
  }

  bool send_cmd(const std::string & cmd, const std::string & id, const std::chrono::duration<double> timeout=300s)
  {
    auto request = std::make_shared<SendScript::Request>();
    request->id = "demo";//id;
    request->script = cmd;

    while (!m_send_script_client->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR_STREAM(
          rclcpp::get_logger(
            "rclcpp"), "Interrupted while waiting for the service. Exiting.");
        return false;
      }
      RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }

    auto result_future = m_send_script_client->async_send_request(request);
    // Waits for the result to become available. 
    // Blocks until specified timeout_duration has elapsed or the result 
    // becomes available, whichever comes first.
    // See: https://en.cppreference.com/w/cpp/thread/future/wait_for
    // Use a timeout of 300s as a default
    std::future_status status = result_future.wait_for(timeout);


    if (status == std::future_status::ready) {
      RCLCPP_INFO(this->get_logger(), "Received response");
      std::shared_ptr<SendScript::Response> response = result_future.get();
      // Do something with response
      if (response.get()->ok) {
        std::string msg = id + std::string(" ") + cmd;
        RCLCPP_INFO_STREAM(
          rclcpp::get_logger("rclcpp"), (std::string(
            "Relative motion OK: ") + msg).c_str());
        return true;
      } else {
        RCLCPP_INFO_STREAM(
          rclcpp::get_logger("rclcpp"), (std::string(
            "Relative motion failed: ") + cmd).c_str());
        return false;
      }
    } else {
      RCLCPP_ERROR_STREAM(
        rclcpp::get_logger("rclcpp"),
        (std::string("Failed to call service for relative motion: ") + cmd).c_str());
      // Remove the service call (See: https://docs.ros.org/en/humble/p/rclcpp/generated/classrclcpp_1_1Client.html#_CPPv4N6rclcpp6Client18async_send_requestE13SharedRequest)
      m_send_script_client->remove_pending_request(result_future);
      return false;
    }
    return false;
  }

protected:
  void svr_response_callback(const tm_msgs::msg::SvrResponse::SharedPtr msg) const
  {

    RCLCPP_INFO_STREAM(
      this->get_logger(),
      "SvrResponse: id is = " << msg->id << ", mode is " << (int)msg->mode << ", content is " << msg->content << ", error code is " <<
        (int)msg->error_code);
  }

  void command_callback(
    const std::shared_ptr<tm_msgs::srv::GoToRelativePosition::Request> request,
    std::shared_ptr<tm_msgs::srv::GoToRelativePosition::Response> response)
  {
    // For command format see Software_Expression - Editor - and - Listen - Node_1 .84_Rev1 .00_EN.pdf page 252

    RCLCPP_INFO_STREAM(
      this->get_logger(), std::endl <<
      "x : " << request->x << std::endl <<
      "y : " << request->y << std::endl <<
      "z : " << request->z << std::endl <<
      "Rx_deg : " << request->rx_deg << std::endl <<
      "Ry_deg : " << request->ry_deg << std::endl <<
      "Rz_deg : " << request->rz_deg << std::endl <<
      "speed_percent : " << request->speed_percent << std::endl <<
      "timeout_s : " << request->timeout_s);

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

    // Setup the ID for the command
    std::ostringstream id_ss;
    id_ss << "RelativeGoTo_" << m_cmd_id_counter;

    if( 0 == request->timeout_s ){
      response->goal_reached = send_cmd(cmd, id_ss.str()); }
    else{
      const std::chrono::duration<double> timeout(request->timeout_s);
      response->goal_reached = send_cmd(cmd, id_ss.str(),timeout);
    }
    m_cmd_id_counter++;
  }

protected:
  rclcpp::Subscription<tm_msgs::msg::SvrResponse>::SharedPtr m_svr_response_subscription;

  rclcpp::Service<tm_msgs::srv::GoToRelativePosition>::SharedPtr m_relative_motion_service;

  rclcpp::Client<tm_msgs::srv::SendScript>::SharedPtr m_send_script_client;

  unsigned int m_cmd_id_counter = 0;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RelativeMotionNode>());
  rclcpp::shutdown();
  return 0;
}
