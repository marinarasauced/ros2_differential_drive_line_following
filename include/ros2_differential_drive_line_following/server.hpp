#ifndef MESS2_UGV_LINE_FOLLOWING_SERVER_HPP
#define MESS2_UGV_LINE_FOLLOWING_SERVER_HPP

#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <iostream>
#include <limits>
#include <memory>
#include <queue>
#include <string>
#include <thread>
#include <tuple>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "mess2_msgs/action/ugv_line_following.hpp"
#include "mess2_plugins/common.hpp"

using Point = geometry_msgs::msg::Point;
using State = geometry_msgs::msg::Pose2D;
using TransformStamped = geometry_msgs::msg::TransformStamped;
using Twist = geometry_msgs::msg::Twist;
using LineFollowingAction = mess2_msgs::action::UGVLineFollowing;
using LineFollowingGoalHandle = rclcpp_action::ServerGoalHandle<LineFollowingAction>;

namespace mess2_ugv_actions {


    /**
     * @brief defines the onboard action server necessary for a differential drive vehcile to follow lines with control input integration and VICON localization.
     */
    class LineFollowingActionServer : public rclcpp::Node
    {
    public:
        std::string name;
        std::string model;
        std::string vicon_topic;
        std::string cmd_vel_topic;
        std::string mode = "r1";
        double timeout_localization = 1.0;

        explicit LineFollowingActionServer() : Node("line_following_action_server")
        {
            this->declare_parameter("name", "burger1");
            this->get_parameter("name", name);
            vicon_topic = "/vicon/" + name + "/" + name;
            cmd_vel_topic = "/" + name + "/cmd_vel";

            this->declare_parameter("model", "burger");
            this->get_parameter("model", model);
            if (model == "burger") {
                v_lin_max_ = 0.22;
                v_ang_max_ = 2.84;
            } else if (model == "wafflepi" || model == "waffle" || model == "waffle_pi") {
                v_lin_max_ = 0.26;
                v_ang_max_ = 1.82;
            } else {
                v_lin_max_ = 0.22;
                v_ang_max_ = 1.82;
            }

            auto handle_goal = [this](
            const rclcpp_action::GoalUUID & uuid,
            std::shared_ptr<const LineFollowingAction::Goal> goal)
            {
                RCLCPP_INFO(this->get_logger(), "received goal request");
                (void)uuid;

                // case : the goal is outside the boundaries (two-dimensional check)
                if (goal->x_target.x < goal->boundaries_min.x || goal->x_target.x > goal->boundaries_max.x || goal->x_target.y < goal->boundaries_min.y || goal->x_target.y > goal->boundaries_max.y) {
                    RCLCPP_INFO(this->get_logger(), "rejected goal because target is outside boundaries");
                    return rclcpp_action::GoalResponse::REJECT;

                // case : the control input gains are not positive
                } else if (goal->k1 <= 0.0 || goal->k2 <= 0.0) {
                    RCLCPP_INFO(this->get_logger(), "rejected goal because control input gains must be non-negative and non-zero");
                    return rclcpp_action::GoalResponse::REJECT;

                // case : the velocity ratio is not within (0.0, 1.0]
                } else if (goal->v_ratio <= 0.0 || goal->v_ratio > 1.0) {
                    RCLCPP_INFO(this->get_logger(), "rejected goal because velocity limiting ratio must be in the range (0.0, 1.0]");
                    return rclcpp_action::GoalResponse::REJECT;

                // case : the error tolerances are negative or zero
                } else if (goal->tolerances.x <= 0.0 || goal->tolerances.y <= 0.0 || goal->tolerances.theta <= 0.0) {
                    RCLCPP_INFO(this->get_logger(), "rejected goal because the error tolerances must be non-negative and non-zero");

                // case : the vehicle is busy with a previous goal
                } else if (busy_ == true) {
                    RCLCPP_INFO(this->get_logger(), "rejected goal because actor is busy");
                    return rclcpp_action::GoalResponse::REJECT;

                // case : the boundaries are not consistent
                } else if (goal->boundaries_max.x < goal->boundaries_min.x || goal->boundaries_max.y < goal->boundaries_min.y) {
                    RCLCPP_INFO(this->get_logger(), "rejected goal because boundaries are not consistent; i.e., max < min");
                    return rclcpp_action::GoalResponse::REJECT;

                // case : model is empty string or none
                } else if (goal->model == "") {
                    RCLCPP_INFO(this->get_logger(), "rejected goal because model cannot be \"\"");
                    return rclcpp_action::GoalResponse::REJECT;
                }

                // case : the vehicle is available and the goal is valid
                return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
            };


            auto handle_cancel = [this](
            const std::shared_ptr<LineFollowingGoalHandle> goal_handel)
            {
                RCLCPP_INFO(this->get_logger(), "received request to cancel goal");
                (void)goal_handel;
                return rclcpp_action::CancelResponse::ACCEPT;
            };


            auto handle_accepted = [this](
            const std::shared_ptr<LineFollowingGoalHandle> goal_handle)
            {
                auto execute_in_thread = [this, goal_handle](){return this->execute(goal_handle);};
                std::thread{execute_in_thread}.detach();
            };

            this->line_following_server_ = rclcpp_action::create_server<LineFollowingAction>(
                this,
                name + "_line_following_action_server",
                handle_goal,
                handle_cancel,
                handle_accepted
            );

            vicon_subscription_ = this->create_subscription<TransformStamped>(vicon_topic, 10, std::bind(&LineFollowingActionServer::ugv_localization, this, std::placeholders::_1));
            cmd_vel_publisher_ = this->create_publisher<Twist>(cmd_vel_topic, 10);
        }

    private:
        rclcpp_action::Server<LineFollowingAction>::SharedPtr line_following_server_;
        bool busy_ = true; // a boolean indicating whether the vehicle is currently busy.
        bool receiving_ = false; // a boolean indicating whether localization data is being received.
        rclcpp::Subscription<TransformStamped>::SharedPtr vicon_subscription_;
        rclcpp::Publisher<Twist>::SharedPtr cmd_vel_publisher_;

        double v_lin_max_;
        double v_ang_max_;
        double u_lin_max_;
        double u_ang_max_;


        /**
         * @brief publishes control input values to the vehicle.
         * 
         * @param u_lin the linear velocity to be published.
         * @param u_ang the angular velocity to be published.
         */
        void ugv_control(const double u_lin, const double u_ang);


        State tolerances_; // the error tolerances for each state.
        Point boundaries_min_; // the minimum x, y, z values the vehicle is permitted to travel to (not including vehicle geometry).
        Point boundaries_max_; // the maximum x, y, z values the vehicle is permitted to travel to (not including vehicle geometry).

        Point x_source_; // the source point of the vehicle during a transition.
        State x_target_; // the target state of the vehicle during a transition.
        State e_local_; // the local error of the vehicle during a transition.
        State x_global_; // the global state of the vehicle.
        
        rclcpp::Time vicon_stamp_; // a time stamp of when localization callbacks are received.

        // rclcpp::Time time_of_localization_; // the time of the last localization callback (current implementation uses VICON localization)
        // bool is_localization_active_ = false; // the status of the localization callback


        /**
         * @brief updates the state attribute upon VICON localization callbacks.
         * 
         * @param msg the ROS2 TransformStamped message passed to the callback function via the ROS2 subscription.
         */
        void ugv_localization(const TransformStamped::SharedPtr msg);


        /**
         * @brief checks goal handle for cancellation updates during while loops in the goal execution.
         * 
         * @param goal_handle a shared pointer to the goal handle.
         * @param result a shared pointer to the goal's result.
         */
        void handle_loop(std::shared_ptr<LineFollowingGoalHandle> goal_handle, std::shared_ptr<LineFollowingAction::Result> result);


        /**
         * @brief handles the control input publishing necessary to rotate the vehicle either towards its target point before a translation or target heading after a tranlsation.
         * 
         * @param goal_handle a shared pointer to the goal handle.
         * @param result a shared pointer to the goal's result.
         * @param feedback a shared pointer to the goal's feedback.
         */
        void handle_rotation(std::shared_ptr<LineFollowingGoalHandle> goal_handle, std::shared_ptr<LineFollowingAction::Result> result, std::shared_ptr<LineFollowingAction::Feedback> feedback);


        /**
         * @brief handles the control input publishing necessary to translate the vehicle to its target point once it is facing towards that point.
         * 
         * @param goal_handle a shared pointer to the goal handle.
         * @param result a shared pointer to the goal's result.
         * @param feedback a shared pointer to the goal's feedback.
         */
        void handle_translation(std::shared_ptr<LineFollowingGoalHandle> goal_handle, std::shared_ptr<LineFollowingAction::Result> result, std::shared_ptr<LineFollowingAction::Feedback> feedback);


        /**
         * @brief handles the execution of the goal, including rotating toward a target point, translation to that point, and then optionally rotating towards a target heading.
         * 
         * @param goal_handle a shared pointer to the goal handle.
         */
        void execute(std::shared_ptr<LineFollowingGoalHandle> goal_handle);

    };

}

#endif  // MESS2_UGV_LINE_FOLLOWING_SERVER_HPP
