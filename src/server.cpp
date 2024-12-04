
#include "ros2_differential_drive_line_following/server.hpp"

namespace mess2_ugv_actions {


    void LineFollowingActionServer::ugv_control(const double u_lin, const double u_ang)
    {
        Twist msg;
        msg.linear.x = u_lin;
        msg.angular.z = u_ang;
        if (std::abs(msg.linear.x) > u_lin_max_) {
            msg.linear.x = std::copysign(u_lin_max_, u_lin);
        }
        if (std::abs(msg.angular.z) > u_ang_max_) {
            msg.angular.z = std::copysign(u_ang_max_, u_ang);
        }
        cmd_vel_publisher_->publish(msg);
    }


    void LineFollowingActionServer::ugv_localization(const TransformStamped::SharedPtr msg)
    {
        // time_of_localization_ = msg->header.stamp;

        auto pos = msg->transform.translation; // the position structure from the message.
        x_global_.x = pos.x;
        x_global_.y = pos.y;

        auto rotq = msg->transform.rotation; // the orientation structure from the message as a quaternion.
        auto rote = mess2_plugins::convert_quaternion_to_euler_angles(rotq); // the orientation structure from the message as euler angles.
        x_global_.theta = mess2_plugins::wrap_angle_to_pi(rote.yaw);

        // add logic for alternative localization methods incase of VICON occlusion.

        if (busy_ == true) {

            auto compute_vector_norm = [](const std::vector<double> &v1) {
                double sum = 0.0;
                for (double val : v1) {
                    sum += val * val;
                }
                return std::sqrt(sum);
            };

            auto compute_vector_dot = [](const std::vector<double> &v1, const std::vector<double> &v2) {
                double sum = 0.0;
                for (size_t i = 0; i < v1.size(); ++i) {
                    sum += v1[i] * v2[i];
                }
                return sum;
            };

            // local error w.r.t the point of the target state
            if (mode == "r1" || mode == "t1") {
                std::vector<double> A = {x_target_.x - x_source_.x, x_target_.y - x_source_.y};
                std::vector<double> B = {x_target_.x - x_global_.x, x_target_.y - x_global_.y};
                std::vector<double> C = {x_global_.x - x_source_.x, x_global_.y - x_source_.y};
                double a = compute_vector_norm(A);
                double b = compute_vector_norm(B);
                double alpha = std::acos(compute_vector_dot(A, B) / (a * b));
                double beta = std::atan2(C[1], C[0]);
                double theta = std::atan2(A[1], A[0]);
                e_local_.x = b * std::cos(alpha);
                e_local_.y = b * std::sin(alpha) * std::copysign(1.0, beta - theta);
                e_local_.theta = mess2_plugins::wrap_angle_to_pi(x_global_.theta - theta);
            // local error w.r.t the heading of the target state
            } else if (mode == "r2") {
                e_local_.x = x_target_.x - x_global_.x;
                e_local_.y = x_target_.y - x_global_.y;
                e_local_.theta = mess2_plugins::wrap_angle_to_pi(x_global_.theta - x_target_.x);
            }
        }

        receiving_ = true;
    }


    void LineFollowingActionServer::handle_loop(std::shared_ptr<LineFollowingGoalHandle> goal_handle, std::shared_ptr<LineFollowingAction::Result> result)
    {
        auto current_stamp = this->get_clock()->now();
        // auto vicon_stamp = rclcpp::Time(vicon_stamp_, rclcpp::Clock::ROS_TIME);
        if (goal_handle->is_canceling()) {
            (void) ugv_control(0.0, 0.0);
            result->success = false;
            goal_handle->canceled(result);
            RCLCPP_INFO(this->get_logger(), "goal cancelled by request");
            busy_ = false;
            return;
        } else if (x_global_.x < boundaries_min_.x || x_global_.x > boundaries_max_.x || x_global_.y < boundaries_min_.y || x_global_.y > boundaries_max_.y) {
            (void) ugv_control(0.0, 0.0);
            result->success = false;
            goal_handle->canceled(result);
            RCLCPP_INFO(this->get_logger(), "goal cancelled due to actor exceeding experimental boundaries");
            busy_ = false;
            return;
        // } else if ((current_stamp - vicon_stamp_).seconds() > timeout_localization) {
        //     (void) ugv_control(0.0, 0.0);
        //     result->success = false;
        //     goal_handle->canceled(result);
        //     RCLCPP_INFO(this->get_logger(), "goal cancelled due to actor localization timeout");
        //     busy_ = false;
        //     return;
        }
    }


    void LineFollowingActionServer::handle_rotation(std::shared_ptr<LineFollowingGoalHandle> goal_handle, std::shared_ptr<LineFollowingAction::Result> result, std::shared_ptr<LineFollowingAction::Feedback> feedback)
    {
        rclcpp::Rate rate(10);
        auto goal = goal_handle->get_goal();
        while (rclcpp::ok() && std::abs(e_local_.theta > tolerances_.theta)) {
            double u_ang = -goal->k2 * e_local_.theta;
            (void) ugv_control(0.0, u_ang);

            feedback->error = e_local_;
            goal_handle->publish_feedback(feedback);
            (void) handle_loop(goal_handle, result);
            rate.sleep();
        }
        (void) ugv_control(0.0, 0.0);
    }


    void LineFollowingActionServer::handle_translation(std::shared_ptr<LineFollowingGoalHandle> goal_handle, std::shared_ptr<LineFollowingAction::Result> result, std::shared_ptr<LineFollowingAction::Feedback> feedback)
    {
        rclcpp::Rate rate(10);
        auto goal = goal_handle->get_goal();
        while (rclcpp::ok() && std::abs(e_local_.theta > tolerances_.theta)) {
            double u_ang = -goal->k1 * e_local_.y -goal->k2 * e_local_.theta;
            (void) ugv_control(u_lin_max_, u_ang);

            feedback->error = e_local_;
            goal_handle->publish_feedback(feedback);
            (void) handle_loop(goal_handle, result);
            rate.sleep();
        }
        (void) ugv_control(0.0, 0.0);
    }


    void LineFollowingActionServer::execute(std::shared_ptr<LineFollowingGoalHandle> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "executing goal");
        rclcpp::Rate rate(10);
        busy_ = true;

        auto goal = goal_handle->get_goal();
        auto result = std::make_shared<LineFollowingAction::Result>();
        auto feedback = std::make_shared<LineFollowingAction::Feedback>();

        u_lin_max_ = v_lin_max_ * goal->v_ratio;
        u_ang_max_ = v_ang_max_ * goal->v_ratio;

        boundaries_min_ = goal->boundaries_min;
        boundaries_max_ = goal->boundaries_max;

        e_local_.x = std::numeric_limits<double>::infinity();
        e_local_.y = std::numeric_limits<double>::infinity();
        e_local_.theta = std::numeric_limits<double>::infinity();

        x_target_ = goal->x_target;
        x_target_.theta = mess2_plugins::wrap_angle_to_pi(x_target_.theta);

        rclcpp::sleep_for(std::chrono::milliseconds(50));

        while (rclcpp::ok() && receiving_ == false) {
            RCLCPP_INFO(this->get_logger(), "waiting to receive localization");
            (void) handle_loop(goal_handle, result);
            rate.sleep();
        }

        x_source_.x = x_global_.x;
        x_source_.y = x_global_.y;

        mode = "r1";
        (void) handle_rotation(goal_handle, result, feedback);

        mode = "t1";
        (void) handle_translation(goal_handle, result, feedback);

        mode = "r2";
        (void) handle_rotation(goal_handle, result, feedback);
        
        mode = "r1";
        if (rclcpp::ok()) {
            (void) ugv_control(0.0, 0.0);
            result->success = true;
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "goal succeeded");
            busy_ = false;
        }
    }
} // namespace mess2_ugv_actions


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<mess2_ugv_actions::LineFollowingActionServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
