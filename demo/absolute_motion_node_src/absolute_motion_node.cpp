#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tm_msgs/srv/send_script.hpp"
#include "tm_msgs/msg/sct_response.hpp"
#include "tm_msgs/msg/sta_response.hpp"
#include "tm_msgs/action/go_to_position.hpp"

#include <chrono>
#include <cstdlib>

#include <sstream>
#include <string>

using namespace std::chrono_literals;

using std::placeholders::_1;
using std::placeholders::_2;
typedef typename tm_msgs::srv::SendScript SendScript;

/**
 * @brief Node to handle absolute motion commands for the Omron robot.
 *
 * This node listens for action requests to move the robot to an absolute position
 * and handles the communication with the robot using SCT and STA messages.
 * It moves the robot using the `PTP` command (see "Omron TM Collaborative
 * Robot: TMscript Language" I664-E-02, pages 296-297) and provides feedback
 * on the motion status through the action server feedback.
 */
class AbsoluteMotionNode : public rclcpp::Node
{
public:
  using GoToPosition = tm_msgs::action::GoToPosition;
  using GoalHandle_GoToPosition = rclcpp_action::ServerGoalHandle<GoToPosition>;

  AbsoluteMotionNode(double feedback_rate_Hz = 50)
  : Node("Omron_absolute_motion"), m_feedback_rate_Hz(feedback_rate_Hz), m_sct_waiter(
      m_sct_waiter_frequency), m_sta_waiter(feedback_rate_Hz)
  {
    m_send_script_client = this->create_client<tm_msgs::srv::SendScript>("send_script");
    if (!m_send_script_client->wait_for_service(600s)) {
      RCLCPP_ERROR_STREAM(
        rclcpp::get_logger("rclcpp"), "Service 'send_script' not available, exiting.");
      rclcpp::shutdown();
      return;
    }

    m_sct_response_subscription = this->create_subscription<tm_msgs::msg::SctResponse>(
      "sct_response", 10, std::bind(&AbsoluteMotionNode::sct_callback, this, _1));

    m_sta_response_subscription = this->create_subscription<tm_msgs::msg::StaResponse>(
      "sta_response", 10, std::bind(&AbsoluteMotionNode::sta_callback, this, _1));

    // Create action server for absolute motion commands
    m_absolute_motion_action_server = rclcpp_action::create_server<GoToPosition>(
      this,
      "go_to_absolute_position",
      std::bind(&AbsoluteMotionNode::handle_goal, this, _1, _2),
      std::bind(&AbsoluteMotionNode::handle_cancel, this, _1),
      std::bind(&AbsoluteMotionNode::handle_accepted, this, _1));
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
    std::shared_ptr<const GoToPosition::Goal> goal)
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
    const std::shared_ptr<GoalHandle_GoToPosition> goal_handle)
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
    auto result = std::make_shared<GoToPosition::Result>();
    cancel_goal(goal_handle, result);
    m_cmd_in_progress = false; // Reset the command in progress flag
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  inline void cancel_goal(
    const std::shared_ptr<GoalHandle_GoToPosition> goal_handle,
    std::shared_ptr<GoToPosition::Result> result)
  {
    stop_motion(); // Stop the motion, because we do not know if it is still in progress
    result->goal_reached = false;
    goal_handle->canceled(result);
    m_motion_in_progress = false; // Reset the motion in progress flag
    m_cmd_in_progress = false;    // Reset the command in progress flag
    RCLCPP_INFO(this->get_logger(), "Goal canceled");
  }

  inline void failed_goal(
    const std::shared_ptr<GoalHandle_GoToPosition> goal_handle,
    std::shared_ptr<GoToPosition::Result> result)
  {
    // Motion is supposed to have stopped, we do not call stop_motion
    result->goal_reached = false;
    goal_handle->abort(result);
    m_motion_in_progress = false; // Reset the motion in progress flag
    m_cmd_in_progress = false;    // Reset the command in progress flag
  }

  inline void goal_reached(
    const std::shared_ptr<GoalHandle_GoToPosition> goal_handle,
    std::shared_ptr<GoToPosition::Result> result)
  {
    // Motion is supposed to have stopped, we do not call stop_motion
    result->goal_reached = true;
    goal_handle->succeed(result);
    m_motion_in_progress = false; // Reset the motion in progress flag
    m_cmd_in_progress = false;    // Reset the command in progress flag
  }

  void handle_accepted(const std::shared_ptr<GoalHandle_GoToPosition> goal_handle)
  {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&AbsoluteMotionNode::execute, this, _1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandle_GoToPosition> goal_handle)
  {

    auto feedback = std::make_shared<GoToPosition::Feedback>();
    auto result = std::make_shared<GoToPosition::Result>();
    const auto goal = goal_handle->get_goal();
    // For command format see Software_Expression - Editor - and - Listen - Node_1 .84_Rev1 .00_EN.pdf page 252

    // 1. Change the base if specified
    //=================================
    const std::string & base = goal->frame; //example: "LASER_ROS2";
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

    // 2. Create and send the absolute motion command
    //===============================================
    // Check if there is a cancel request
    {
      if (goal_handle->is_canceling()) {
        cancel_goal(goal_handle, result);
        return;
      }

      std::ostringstream ss;
      // C: expressed in current base, P: Speed format as a percentage,
      // P: blending format as a percentage
      ss << "PTP(\"CPP\",";
      // Absolute motion parameters
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
        "Absolute motion command: " << motion_cmd);
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

public:
  /** Compute a duration estimation for a motion
   * @param distance The distance to move in mm
   * @param velocity The velocity in mm/s
   * @param speed_percent The speed percentage (0-100)
   * @param time_to_top_speed_ms The time to reach top speed in milliseconds
   * @return The estimated duration of the motion in milliseconds
   */
  double motion_duration_estimation(
    double distance, double velocity, double speed_percent,
    double time_to_top_speed_ms)
  {
    // Convert speed percentage to actual speed in mm/s
    double actual_speed = (velocity * speed_percent) / 100.0;

    // Calculate the time to reach top speed in seconds
    double time_to_top_speed_s = time_to_top_speed_ms / 1000.0;

    double acceleration = actual_speed / time_to_top_speed_s; // mm/s^2

    // Calculate the distance covered during acceleration to top speed
    // Using the formula: distance = 0.5 * acceleration * time^2
    // where acceleration = (final speed - initial speed) / time
    // Here, initial speed is 0, so we can simplify to:
    // distance_acceleration = 0.5 * actual_speed * time_to_top_speed_s
    // This is the distance covered during the acceleration phase considered as
    // a triangle area: constant acceleration from 0 to actual_speed over time_to_top_speed_s seconds.
    // The speed follows a linear profile during the acceleration phase, then is constant at actual_speed.
    double distance_acceleration = 0.5 * actual_speed * time_to_top_speed_s;

    // We consider that the motion has 3 phases:
    // 1. Acceleration phase: from 0 to actual_speed over time_to_top_speed_s seconds
    // 2. Constant speed phase: at actual_speed for the remaining distance
    // 3. Deceleration phase: from actual_speed to 0 over time_to_top_speed_s seconds

    // If the distance is less than the distance covered during acceleration and deceleration
    // we suppose that the robot will not reach the top speed
    if (distance <= 2 * distance_acceleration) {
      // TODO(@yguel) add a more precise estimation of the time in this case
      //  For now, we assume that the robot will take the same time to accelerate and decelerate
      //  and will not reach the top speed.
      //  The time to accelerate to top speed is the same as the time to decelerate
      //  and the distance covered during acceleration is the same as the distance covered during deceleration.
      // d = 0.5 * a * t^2
      // => t = sqrt(2 * d / a)
      double max_speed_distance = distance / 2.0; // Half of the distance for acceleration and deceleration
      double time_to_max_speed = std::sqrt(2 * max_speed_distance / acceleration);
      // The total time is the time to accelerate to max speed and the time to decelerate
      // to 0, which is the same as the time to accelerate.
      // So the total time is 2 * time_to_max_speed.
      return (2 * time_to_max_speed) * 1000.0; // Return in milliseconds
    }

    // Otherwise, calculate the total time including acceleration, deceleration and constant speed phases
    double remaining_distance = distance - 2 * distance_acceleration;
    double constant_speed_time = remaining_distance / actual_speed;

    return (2 * time_to_top_speed_s + constant_speed_time) * 1000.0; // Return in milliseconds
  }

protected:
  double m_feedback_rate_Hz; // Feedback rate in Hz

  rclcpp::Subscription<tm_msgs::msg::SctResponse>::SharedPtr m_sct_response_subscription;

  rclcpp::Subscription<tm_msgs::msg::StaResponse>::SharedPtr m_sta_response_subscription;

  rclcpp_action::Server<tm_msgs::action::GoToPosition>::SharedPtr
    m_absolute_motion_action_server;

  rclcpp::Client<tm_msgs::srv::SendScript>::SharedPtr m_send_script_client;

  rclcpp::WaitSet m_wait_set;

  unsigned int m_sta_msg_counter = 0;             // Counter for the number of STA messages received
  tm_msgs::msg::StaResponse::UniquePtr m_sta_msg; // Last STA message
  unsigned int m_sct_msg_counter = 0;             // Counter for the number of SCT messages
  tm_msgs::msg::SctResponse::UniquePtr m_sct_msg; // Last SCT message

  bool m_cmd_in_progress = false; // Flag to indicate if a command is in progress
  unsigned int m_cmd_counter = 0;
  unsigned int m_motion_counter = 0; // Counter for motion commands
  std::string m_last_cmd_id = "";    // Last command ID sent to the robot
  std::chrono::milliseconds m_sct_timeout_ms = 5000ms;
  std::string m_motion_tag = "";         // Tag for the current motion command
  double m_sct_waiter_frequency = 200.0; // Frequency for the SCT waiter in Hz (200 Hz = 5ms)
  rclcpp::Rate m_sct_waiter;             // Sleeping object for waiting for SCT responses
  rclcpp::Rate m_sta_waiter;             // Sleeping object for waiting for STA responses

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
  auto node = std::make_shared<AbsoluteMotionNode>();
  executor->add_node(node);

  RCLCPP_INFO_STREAM(
    rclcpp::get_logger("rclcpp"),
    "Absolute motion node started, waiting for commands...");

  std::thread spin_thread([executor]()
    {executor->spin();});

  executor->cancel();
  spin_thread.join();

  RCLCPP_INFO_STREAM(
    rclcpp::get_logger("rclcpp"),
    "Absolute motion node stopped, exiting...");
  rclcpp::shutdown();
  return 0;
}
