#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tm_msgs/srv/send_script.hpp"
#include "tm_msgs/msg/sct_response.hpp"
#include "tm_msgs/msg/sta_response.hpp"
#include "tm_msgs/action/go_to_relative_position.hpp"

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
  using GoToRelativePosition = tm_msgs::action::GoToRelativePosition;
  using GoalHandle_GoToRelativePosition = rclcpp_action::ServerGoalHandle<GoToRelativePosition>;

  RelativeMotionNode(double feedback_rate_Hz = 50)
  : Node("Omron_relative_motion")
    , m_feedback_rate_Hz(feedback_rate_Hz)
    , m_sct_waiter(m_sct_waiter_frequency)
    , m_sta_waiter(feedback_rate_Hz)
  {
    m_send_script_client = this->create_client<tm_msgs::srv::SendScript>("send_script");
    if (!m_send_script_client->wait_for_service(1s)) {
      RCLCPP_ERROR_STREAM(
        rclcpp::get_logger("rclcpp"), "Service 'send_script' not available, exiting.");
      rclcpp::shutdown();
      return;
    }

    m_sct_response_subscription = this->create_subscription<tm_msgs::msg::SctResponse>(
      "sct_response", 10, std::bind(&RelativeMotionNode::sct_callback, this, _1));

    m_sta_response_subscription = this->create_subscription<tm_msgs::msg::StaResponse>(
      "sta_response", 10, std::bind(&RelativeMotionNode::sta_callback, this, _1));

    // Create action server for relative motion commands
    m_relative_motion_action_server = rclcpp_action::create_server<GoToRelativePosition>(
      this,
      "go_to_relative_position",
      std::bind(&RelativeMotionNode::handle_goal, this, _1, _2),
      std::bind(&RelativeMotionNode::handle_cancel, this, _1),
      std::bind(&RelativeMotionNode::handle_accepted, this, _1));
  }

  void sta_callback(tm_msgs::msg::StaResponse::UniquePtr msg)
  {
    m_sta_msg = std::move(msg);
    m_sta_msg_counter++;
  }

  void sct_callback(tm_msgs::msg::SctResponse::UniquePtr msg)
  {
    m_sct_msg = std::move(msg);
    m_sct_msg_counter++;
  }

  std::string create_id() const
  {
    unsigned int rounded_cmd_id = m_cmd_counter % 100;
    std::ostringstream id_ss;
    id_ss << "rm" << rounded_cmd_id;
    return id_ss.str();
  }

  inline unsigned int motion_cmd_id() const
  {
    // Round motion counter because only 14 ids are available
    unsigned int rounded_motion_id = m_motion_counter % 14;
    // Motion IDs are between 1 and 15
    return rounded_motion_id + 1;
  }

  bool send_cmd(const std::string & cmd)
  {
    auto request_motion = std::make_shared<SendScript::Request>();
    m_last_cmd_id = create_id();
    request_motion->id = m_last_cmd_id;
    request_motion->script = cmd;

    // 1. Ensure the send script service is available
    while (!m_send_script_client->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR_STREAM(
          rclcpp::get_logger(
            "rclcpp"),
          "Interrupted while waiting for the service. Exiting.");
        return false;
      }
      RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }

    // 2. Send command
    unsigned int sct_count = m_sct_msg_counter;
    m_send_script_client->async_send_request(request_motion);

    // Increment the command ID counter
    m_cmd_counter++;

    // 3. Wait for the SCT response
    auto current_time = this->get_clock()->now();
    while (m_sct_msg_counter == sct_count &&
      (this->get_clock()->now() - current_time) < m_sct_timeout_ms)
    {
      // Wait for the SCT response to be available
      m_sct_waiter.sleep();
    }
    if (m_sct_msg_counter == sct_count) {
      RCLCPP_ERROR_STREAM(
        rclcpp::get_logger(
          "rclcpp"),
        "No SCT response after sending command: " << cmd);
      return false;
    }
    // Copy the SCT message
    tm_msgs::msg::SctResponse incoming = std::move(*m_sct_msg);

    // Decode sct message
    // Check that id is correct
    if (m_last_cmd_id != incoming.id) {
      std::string err =
        std::string("Wrong id in SCT response after sending command. Expected: ") +
        m_last_cmd_id + std::string(" got ") + incoming.id;
      RCLCPP_ERROR_STREAM(
        rclcpp::get_logger(
          "rclcpp"),
        err.c_str());
      return false;
    }
    if (incoming.script != std::string("OK")) {
      RCLCPP_ERROR_STREAM(
        rclcpp::get_logger(
          "rclcpp"),
        "SCT response after sending command is not OK: " + incoming.script);
      return false;
    }

    return true;
  }

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const GoToRelativePosition::Goal> goal)
  {
    RCLCPP_INFO(
      this->get_logger(), "Received goal request (dx,dy,dz,rx,ry,rz) [speed,time to top speed, frame] : (%fmm,%fmm,%fmm,%f°,%f°,%f°) [%d%%,%dms,%s]",
      goal->x, goal->y, goal->z,
      goal->rx_deg, goal->ry_deg, goal->rz_deg,
      goal->speed_percent,
      goal->time_to_top_speed_ms,
      goal->frame.c_str());
    (void)uuid; //< Unused parameter, avoid unused parameter warning
    if (m_cmd_in_progress) {
      RCLCPP_ERROR(this->get_logger(), "A command is already in progress, rejecting new goal");
      return rclcpp_action::GoalResponse::REJECT;
    }
    m_cmd_in_progress = true;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  void stop_motion()
  {
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandle_GoToRelativePosition> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    if (!m_cmd_in_progress) {
      RCLCPP_WARN(this->get_logger(), "No command in progress, nothing to cancel");
      // If no command is in progress, we can just return ACCEPT
      return rclcpp_action::CancelResponse::REJECT;
    }

    if (m_motion_in_progress) {
      RCLCPP_INFO(this->get_logger(), "Stopping current motion");
      // A motion is in progress, we need to stop it
      stop_motion();
    }
    auto result = std::make_shared<GoToRelativePosition::Result>();
    cancel_goal(goal_handle, result);
    m_cmd_in_progress = false; // Reset the command in progress flag
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  inline void cancel_goal(
    const std::shared_ptr<GoalHandle_GoToRelativePosition> goal_handle,
    std::shared_ptr<GoToRelativePosition::Result> result)
  {
    stop_motion(); // Stop the motion, because we do not know if it is still in progress
    result->goal_reached = false;
    goal_handle->canceled(result);
    m_motion_in_progress = false; // Reset the motion in progress flag
    m_cmd_in_progress = false; // Reset the command in progress flag
    RCLCPP_INFO(this->get_logger(), "Goal canceled");
  }

  inline void failed_goal(
    const std::shared_ptr<GoalHandle_GoToRelativePosition> goal_handle,
    std::shared_ptr<GoToRelativePosition::Result> result)
  {
    //Motion is supposed to have stopped, we do not call stop_motion
    result->goal_reached = false;
    goal_handle->abort(result);
    m_motion_in_progress = false; // Reset the motion in progress flag
    m_cmd_in_progress = false; // Reset the command in progress flag
  }

  inline void goal_reached(
    const std::shared_ptr<GoalHandle_GoToRelativePosition> goal_handle,
    std::shared_ptr<GoToRelativePosition::Result> result)
  {
    //Motion is supposed to have stopped, we do not call stop_motion
    result->goal_reached = true;
    goal_handle->succeed(result);
    m_motion_in_progress = false; // Reset the motion in progress flag
    m_cmd_in_progress = false; // Reset the command in progress flag
  }

  void handle_accepted(const std::shared_ptr<GoalHandle_GoToRelativePosition> goal_handle)
  {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&RelativeMotionNode::execute, this, _1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandle_GoToRelativePosition> goal_handle)
  {

    auto feedback = std::make_shared<GoToRelativePosition::Feedback>();
    auto result = std::make_shared<GoToRelativePosition::Result>();
    const auto goal = goal_handle->get_goal();
    // For command format see Software_Expression - Editor - and - Listen - Node_1 .84_Rev1 .00_EN.pdf page 252

    // 1. Change the base if specified
    //=================================
    const std::string & base = goal->frame; //"LASER_ROS2";
    if (base != std::string("")) {
      std::string cmd = "ChangeBase(\"" + base + "\")";
      RCLCPP_INFO_STREAM(
        this->get_logger(), "Changing base to: " << base);
      // Send the command to change the base
      if (!send_cmd(cmd)) {
        RCLCPP_ERROR_STREAM(
          this->get_logger(),
          "Failed to change base to: " << base);
        cancel_goal(goal_handle, result);
        return;
      } else {
        RCLCPP_INFO_STREAM(
          this->get_logger(),
          "Base changed to: " << base);
      }
    }

    // 2. Create and send the relative motion command
    //================================================
    // Check if there is a cancel request
    {
      if (goal_handle->is_canceling()) {
        cancel_goal(goal_handle, result);
        return;
      }

      std::ostringstream ss;
      // C: expressed in current base, P: Speed format as a percentage,
      // P: blending format as a percentage
      ss << "Move_PTP(\"CPP\",";
      // relative motion parameters
      ss << goal->x << "," << goal->y << "," << goal->z << ","
         << goal->rx_deg << "," << goal->ry_deg << "," << goal->rz_deg << ",";
      // speed percentage
      ss << goal->speed_percent << ",";
      // The time interval to accelerate to top speed (ms)
      ss << "200,";
      // blending percentage
      ss << "0,";
      // Disable precise positioning
      ss << "false)";

      // Log the command
      const std::string motion_cmd = ss.str();
      RCLCPP_INFO_STREAM(
        this->get_logger(),
        "Relative motion command: " << motion_cmd);
      m_cmd_sent_time = this->get_clock()->now();
      if (!send_cmd(motion_cmd)) {
        failed_goal(goal_handle, result);
        return;
      }
    }

    unsigned int sta_count = 0;

    // 3. Record and queue a tag to be able to detect when the motion is
    // finished
    //====================================================================
    {
      std::ostringstream ss;
      const unsigned int motion_id = motion_cmd_id();
      ss << "QueueTag(" << motion_id << ")";
      const std::string tag_cmd = ss.str();
      // Log the tag command
      RCLCPP_INFO_STREAM(
        this->get_logger(),
        "QueueTag for motion id: " << motion_id);

      sta_count = m_sta_msg_counter;
      if (!send_cmd(tag_cmd)) {
        RCLCPP_ERROR_STREAM(
          this->get_logger(),
          "Failed to tag the motion: " << motion_id <<
            "impossible to determine when the motion will finish");
        stop_motion(); // As we do not know if the motion was started or not, we stop it
        failed_goal(goal_handle, result);
        return;
      } else {
        std::ostringstream tag_ss;
        // 2 digits for the motion id, with a leading zero if necessary
        tag_ss << std::setw(2) << std::setfill('0') << motion_id;
        m_motion_tag = tag_ss.str(); // Store the tag for later use
        m_motion_in_progress = true; // Set the motion in progress flag
        RCLCPP_INFO_STREAM(
          this->get_logger(),
          "QueueTag command sent for motion id: " << motion_id);
      }
    }

    // 4. Increment the motion counter
    //=================================
    // This is used to create a unique ID for each motion command.
    ++m_motion_counter;

    // 5. Provide feedback to the action client
    //=========================================
    {
      while (m_motion_in_progress) {
        // Check if there is a cancel request
        if (goal_handle->is_canceling()) {
          cancel_goal(goal_handle, result);
          return;
        }

        if (sta_count != m_sta_msg_counter) {
          // Copy the STA message
          tm_msgs::msg::StaResponse sta_response = std::move(*m_sta_msg);

          // Check if the motion is finished and
          // check if the STA response is related to the motion command we just sent
          if (sta_response.subcmd == "01") {
            if (sta_response.subdata == m_motion_tag + ",true") {
              RCLCPP_INFO_STREAM(
                this->get_logger(),
                "Motion with id: " << m_motion_tag << " finished successfully.");
              goal_reached(goal_handle, result);
              return;
            } else {
              if (sta_response.subdata == m_motion_tag + ",false") {
                RCLCPP_ERROR_STREAM(
                  this->get_logger(),
                  "Motion with id: " << m_motion_tag << " failed.");
                failed_goal(goal_handle, result);
                return;
              } else {
                // QueueTag answer, a motion has stopped, but it is not the one we are waiting for
                // This can happen if multiple motions are sent in a row
                RCLCPP_WARN_STREAM(
                  this->get_logger(),
                  "Motion info id,completed: " << sta_response.subdata << ". But it is not the one we were waiting for ("
                                               << m_motion_tag << ").");
                // We do not stop the motion, as it is not the one we are waiting for
                sta_count = m_sta_msg_counter; // Reset the STA count to wait for the next STA message
              }
            }
          }
        }

        // TODO(@yguel) add a timeout based on velocity and distance
        // Wait for the STA response to be available
        m_sta_waiter.sleep();

        // Provide feedback to the action client
        feedback->motion_id = m_last_cmd_id;
        if (m_motion_in_progress) {
          feedback->status = "In progress";
        } else {
          feedback->status = "Finished";
        }
        goal_handle->publish_feedback(feedback);
      }

      // Release the motion in progress and the command in progress flags
      m_motion_in_progress = false;
      m_cmd_in_progress = false;
      return;
    }
  }

protected:
  double m_feedback_rate_Hz;   // Feedback rate in Hz

  rclcpp::Subscription<tm_msgs::msg::SctResponse>::SharedPtr m_sct_response_subscription;

  rclcpp::Subscription<tm_msgs::msg::StaResponse>::SharedPtr m_sta_response_subscription;

  rclcpp_action::Server<tm_msgs::action::GoToRelativePosition>::SharedPtr
    m_relative_motion_action_server;

  rclcpp::Client<tm_msgs::srv::SendScript>::SharedPtr m_send_script_client;

  rclcpp::WaitSet m_wait_set;

  unsigned int m_sta_msg_counter = 0; // Counter for the number of STA messages received
  tm_msgs::msg::StaResponse::UniquePtr m_sta_msg; // Last STA message
  unsigned int m_sct_msg_counter = 0; // Counter for the number of SCT messages
  tm_msgs::msg::SctResponse::UniquePtr m_sct_msg; // Last SCT message

  bool m_cmd_in_progress = false;   // Flag to indicate if a command is in progress
  unsigned int m_cmd_counter = 0;
  unsigned int m_motion_counter = 0;   // Counter for motion commands
  std::string m_last_cmd_id = ""; // Last command ID sent to the robot
  std::chrono::milliseconds m_sct_timeout_ms = 5000ms;
  std::string m_motion_tag = "";     // Tag for the current motion command
  double m_sct_waiter_frequency = 200.0; // Frequency for the SCT waiter in Hz (200 Hz = 5ms)
  rclcpp::Rate m_sct_waiter; // Sleeping object for waiting for SCT responses
  rclcpp::Rate m_sta_waiter; // Sleeping object for waiting for STA responses

  // Store the time when the last command was sent
  // in order to make the matching between an sta response that acknowledges
  // that the motion order by the command has been executed.
  rclcpp::Time m_cmd_sent_time;
  bool m_motion_in_progress = false;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::ExecutorOptions options;
  size_t num_threads = 2;
  auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>(options, num_threads);
  auto node = std::make_shared<RelativeMotionNode>();
  executor->add_node(node);

  RCLCPP_INFO_STREAM(
    rclcpp::get_logger("rclcpp"),
    "Relative motion node started, waiting for commands...");

  std::thread spin_thread([executor]() {
      executor->spin();
    });

  executor->cancel();
  spin_thread.join();

  RCLCPP_INFO_STREAM(
    rclcpp::get_logger("rclcpp"),
    "Relative motion node stopped, exiting...");
  rclcpp::shutdown();
  return 0;
}
