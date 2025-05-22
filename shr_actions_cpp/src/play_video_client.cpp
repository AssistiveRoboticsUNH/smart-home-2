#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <shr_msgs/action/play_video_request.hpp>

using PlayVideoRequest = shr_msgs::action::PlayVideoRequest;

class PlayVideoClient : public rclcpp::Node {
public:
    using GoalHandlePlayVideo = rclcpp_action::ClientGoalHandle<PlayVideoRequest>;

    PlayVideoClient() : Node("play_video_client") {
        this->client_ = rclcpp_action::create_client<PlayVideoRequest>(this, "play_video");

        // Wait for server
        while (!this->client_->wait_for_action_server(std::chrono::seconds(2))) {
            RCLCPP_WARN(this->get_logger(), "⏳ Waiting for action server...");
        }

        send_goal("example_video.mp4");  // Change the filename here
    }

private:
    rclcpp_action::Client<PlayVideoRequest>::SharedPtr client_;

    void send_goal(const std::string &file_name) {
        auto goal_msg = PlayVideoRequest::Goal();
        goal_msg.file_name = file_name;

        RCLCPP_INFO(this->get_logger(), "🎯 Sending goal: %s", file_name.c_str());

        auto send_goal_options = rclcpp_action::Client<PlayVideoRequest>::SendGoalOptions();
        send_goal_options.result_callback = 
            [this](const GoalHandlePlayVideo::WrappedResult &result) {
                if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                    RCLCPP_INFO(this->get_logger(), "✅ Result: %s", result.result->status.c_str());
                } else {
                    RCLCPP_ERROR(this->get_logger(), "❌ Action failed or was canceled.");
                }
                rclcpp::shutdown();  // Exit after getting result
            };

        this->client_->async_send_goal(goal_msg, send_goal_options);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PlayVideoClient>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
