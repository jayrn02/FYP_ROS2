#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp> // For error codes

#include <memory>
#include <string>
#include <thread>
#include <chrono> // For timeouts and sleeping
#include <atomic> // For signaling completion


using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
using namespace std::chrono_literals; // For time units like 1s, 10ms

// Use a global atomic flag to signal when processing is done or should stop
std::atomic<bool> g_request_shutdown(false);
std::atomic<bool> g_message_processed(false);

class PointGoalExecutor : public rclcpp::Node
{
public: // Make constructor and relevant methods public
    PointGoalExecutor() : Node("point_goal_executor_once")
    {
        RCLCPP_INFO(this->get_logger(), "Initializing PointGoalExecutor Node (Run Once)...");

        // --- MoveGroupInterface Setup ---
        move_group_node_ = rclcpp::Node::make_shared("point_goal_move_group_interface_node_once", this->get_node_options());
        mgi_executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
        mgi_executor_->add_node(move_group_node_);
        mgi_executor_thread_ = std::thread([this]() { this->mgi_executor_->spin(); });

        rclcpp::sleep_for(1s);

        move_group_interface_ = std::make_shared<MoveGroupInterface>(move_group_node_, "arm");

        if (!move_group_interface_->getPlanningFrame().empty()) {
             RCLCPP_INFO(this->get_logger(), "MoveGroupInterface connected for group '%s'. Planning frame: '%s'",
                        move_group_interface_->getName().c_str(),
                        move_group_interface_->getPlanningFrame().c_str());
        } else {
            RCLCPP_ERROR(this->get_logger(), "MoveGroupInterface failed to initialize properly after 1s wait.");
            g_request_shutdown = true;
            // Cleanup MGI thread immediately if initialization failed
            this->cleanup_mgi_resources();
            return;
        }

        move_group_interface_->setPlanningTime(10.0);
        move_group_interface_->setNumPlanningAttempts(5);
        move_group_interface_->setPlannerId("RRTConnectkConfigDefault");
        end_effector_link_ = "tcp";
        planning_frame_ = move_group_interface_->getPlanningFrame();
    }

    // Destructor ensures cleanup on normal exit
    ~PointGoalExecutor() {
        RCLCPP_INFO(this->get_logger(), "PointGoalExecutor node shutting down.");
        this->cleanup_mgi_resources(); // Call cleanup function
    }

    // Public function to handle MGI resource cleanup
    void cleanup_mgi_resources() {
        // Check if already cleaned up to prevent double execution
        if (!mgi_cleaned_up_) {
             if (mgi_executor_) {
                mgi_executor_->cancel();
                RCLCPP_DEBUG(this->get_logger(), "MGI executor cancelled.");
            }
            if (mgi_executor_thread_.joinable()) {
                mgi_executor_thread_.join();
                RCLCPP_DEBUG(this->get_logger(), "MGI executor thread joined.");
            }
            mgi_cleaned_up_ = true; // Mark as cleaned up
        }
    }


    // Function to process the received point and command MoveIt
    void process_point_and_move(const geometry_msgs::msg::PointStamped::SharedPtr msg)
    {
        // Ensure MGI is valid before proceeding
        if (!move_group_interface_ || g_request_shutdown) {
             RCLCPP_WARN(this->get_logger(), "Skipping processing: MGI not valid or shutdown requested.");
             g_message_processed = true; // Still mark as processed to exit loop
             g_request_shutdown = true;
             return;
        }

        RCLCPP_INFO(this->get_logger(), "Processing received point in frame '%s': [x: %.4f, y: %.4f, z: %.4f]",
                    msg->header.frame_id.c_str(), msg->point.x, msg->point.y, msg->point.z);

        // --- Frame Check ---
        if (msg->header.frame_id != planning_frame_) {
            RCLCPP_ERROR(this->get_logger(), "Received point is in frame '%s', but the planning frame is '%s'. Aborting move.",
                         msg->header.frame_id.c_str(), planning_frame_.c_str());
             g_message_processed = true;
             g_request_shutdown = true;
             return;
        }

        // --- Set Target Position ---
        RCLCPP_INFO(this->get_logger(), "Setting target position for link '%s' in planning frame '%s': Pos(%.3f, %.3f, %.3f)",
             end_effector_link_.c_str(), planning_frame_.c_str(),
             msg->point.x, msg->point.y, msg->point.z);

        bool target_set_success = move_group_interface_->setPositionTarget(
            msg->point.x, msg->point.y, msg->point.z,
            end_effector_link_
        );

        if (!target_set_success) {
             RCLCPP_ERROR(this->get_logger(), "Failed to set position target in MoveGroupInterface.");
             g_message_processed = true;
             g_request_shutdown = true;
             return;
        }

        // --- Plan ---
        RCLCPP_INFO(this->get_logger(), "Planning trajectory...");
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        moveit::core::MoveItErrorCode planning_result = move_group_interface_->plan(my_plan);

        if (planning_result == moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_INFO(this->get_logger(), "Planning successful. Executing trajectory...");

            // --- Execute ---
            moveit::core::MoveItErrorCode execute_result = move_group_interface_->execute(my_plan);
            if (execute_result == moveit::core::MoveItErrorCode::SUCCESS) {
                 RCLCPP_INFO(this->get_logger(), "Execution successful.");
            } else {
                 RCLCPP_ERROR(this->get_logger(), "Execution failed with error code: %d", execute_result.val);
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "Planning failed with error code: %d", planning_result.val);
             try { // Wrap getCurrentPose in try-catch as it can sometimes fail
                 geometry_msgs::msg::PoseStamped current_pose = move_group_interface_->getCurrentPose(end_effector_link_);
                 RCLCPP_INFO(this->get_logger(), "Current Pose: Pos(%.3f, %.3f, %.3f) in frame %s",
                    current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z,
                    current_pose.header.frame_id.c_str());
             } catch (const std::exception& e) {
                 RCLCPP_WARN(this->get_logger(), "Could not get current pose for debugging: %s", e.what());
             }
        }

        g_message_processed = true;
        g_request_shutdown = true;
    }


    // Callback for the one-shot subscription
    void topic_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
    {
        if (g_message_processed || g_request_shutdown) {
            return;
        }

        RCLCPP_INFO(this->get_logger(), "First message received from topic.");
        process_point_and_move(msg);

        if(subscription_) {
             RCLCPP_INFO(this->get_logger(), "Unsubscribing from topic.");
             subscription_.reset();
        }
    }

    // Public member to hold the subscription so main can create it
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr subscription_;

private: // Keep internal implementation details private
    std::shared_ptr<MoveGroupInterface> move_group_interface_;
    std::shared_ptr<rclcpp::Node> move_group_node_;
    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> mgi_executor_;
    std::thread mgi_executor_thread_;

    std::string end_effector_link_;
    std::string planning_frame_;
    std::atomic<bool> mgi_cleaned_up_{false}; // Flag to prevent double cleanup
};


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<PointGoalExecutor>();

    rclcpp::executors::SingleThreadedExecutor main_executor;
    main_executor.add_node(node);


    // Check if MGI initialized correctly (g_request_shutdown flag set in constructor)
    if (g_request_shutdown) {
         // Use the node's logger now that the node exists
         RCLCPP_FATAL(node->get_logger(), "MoveGroupInterface failed during initialization. Shutting down."); // <-- FIX: Use logger
         rclcpp::shutdown();
         // Cleanup function is called automatically by destructor when node goes out of scope
         // Or explicitly if needed before shutdown: node->cleanup_mgi_resources(); (already called in constructor on fail)
         return 1;
    }

    const std::string target_topic = "/detected_object/position_world";
    // QoS 1 for "get the latest one available" (if publisher is reliable) or "get the next one"
    rclcpp::QoS qos_profile(1);
    qos_profile.best_effort(); // Use best_effort for sensor data

    node->subscription_ = node->create_subscription<geometry_msgs::msg::PointStamped>(
        target_topic,
        qos_profile, // Use QoS profile
        std::bind(&PointGoalExecutor::topic_callback, node.get(), std::placeholders::_1));

    RCLCPP_INFO(node->get_logger(), "Waiting for one message on topic '%s'...", target_topic.c_str());

    auto timeout_duration = 15s; // Use C++ literal for duration
    auto start_time = node->get_clock()->now();

    while (!g_message_processed && !g_request_shutdown && rclcpp::ok())
    {
        main_executor.spin_some(100ms);

        if ((node->get_clock()->now() - start_time) > timeout_duration) {
            RCLCPP_ERROR(node->get_logger(), "Timeout waiting for message on topic '%s'. Shutting down.", target_topic.c_str());
            g_request_shutdown = true;
            break;
        }
    }

    if(g_message_processed) {
         RCLCPP_INFO(node->get_logger(), "Message processed. Node will exit.");
    } else if (g_request_shutdown) {
         RCLCPP_INFO(node->get_logger(), "Shutdown requested (e.g., timeout or error). Node will exit.");
    }

    rclcpp::sleep_for(500ms);

    // Explicitly call shutdown - this will trigger node destruction & cleanup
    RCLCPP_INFO(node->get_logger(), "Requesting ROS shutdown.");
    rclcpp::shutdown();

    // Destructor of 'node' will call cleanup_mgi_resources

    RCLCPP_INFO(rclcpp::get_logger("point_goal_executor_main"), "Main function finished."); // Use generic logger after shutdown
    return 0;
}