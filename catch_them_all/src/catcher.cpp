#include "rclcpp/rclcpp.hpp"
#include <random>
#include <cmath>
#include <algorithm>
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/action/rotate_absolute.hpp"
#include "turtlesim/msg/pose.hpp"
#include "turtlesim/srv/spawn.hpp"
#include "turtlesim/srv/kill.hpp"
#include "std_msgs/msg/string.hpp"

using namespace turtlesim::srv;
constexpr float SCREEN_SIZE = 10.0;
constexpr uint16_t dt_ms = 1;

using namespace geometry_msgs::msg;
using namespace turtlesim::msg;


class Catcher : public rclcpp::Node
{
public:
    Catcher(const std::string &turtle_name = "turtle1") 
    : Node("catcher_node"), USER_TURTLE_NAME(turtle_name) {

        // Set initial target position
        auto initial_target = turtlesim::msg::Pose();
        initial_target.x = 5.0;
        initial_target.y = 5.0;
        target_pose_ = initial_target;

        // Create publisher, subscribers, and timers
        velocity_pub_ = this->create_publisher<Twist>(USER_TURTLE_NAME + "/cmd_vel", 10);
        pose_sub_ = this->create_subscription<Pose>(
            USER_TURTLE_NAME + "/pose", 10,
            std::bind(&Catcher::pose_callback, this, std::placeholders::_1));

        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(dt_ms), 
            std::bind(&Catcher::move_turtle_callback, this));

        spawn_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1200), 
            std::bind(&Catcher::spawn_radom_turtle_callback, this));

        spawn_random_turtle_client_ = this->create_client<Spawn>("spawn_random_turtle");
        kill_turtle_client_ = this->create_client<Kill>("kill");
        
        // need_to_update_values_ = true;
    }
private:
    void move_turtle_callback();
    void spawn_radom_turtle_callback();
    void pose_callback(const turtlesim::msg::Pose::SharedPtr msg);
    void target_pose_callback(const turtlesim::msg::Pose::SharedPtr msg);
    void set_next_goal();
    void kill_target_turtle();
    void update_target_turtle_pose_sub_();
    void log_debug_data() const;

    bool need_to_update_values_;
    std::vector<std::string> turtle_names_;
    std::string USER_TURTLE_NAME;
    std::string target_turtle_name;
    turtlesim::msg::Pose current_pose_;
    turtlesim::msg::Pose target_pose_;
    rclcpp::Subscription<Pose>::SharedPtr pose_sub_, target_turtle_pose_sub_;
    rclcpp::Publisher<Twist>::SharedPtr velocity_pub_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    rclcpp::TimerBase::SharedPtr spawn_timer_;
    rclcpp::Client<Kill>::SharedPtr kill_turtle_client_;
    rclcpp::Client<Spawn>::SharedPtr spawn_random_turtle_client_;
};


void Catcher::set_next_goal(){
    // log_debug_data();

    if (turtle_names_.empty()) {
        // RCLCPP_INFO(this->get_logger(), "No turtles to catch.");
        return;
    }

    if(!need_to_update_values_) {
        kill_target_turtle();
        RCLCPP_INFO(this->get_logger() ,"%s caught!", target_turtle_name.c_str());
        need_to_update_values_ = true;
    }
}

void Catcher::kill_target_turtle(){
    if (turtle_names_.empty()) {
        RCLCPP_ERROR(this->get_logger(), "No turtles available to kill.");
        return;
    }

    auto request = std::make_shared<Kill::Request>();
    request->name = turtle_names_.begin()->c_str();
    auto future = kill_turtle_client_->async_send_request(request);        

    turtle_names_.erase(turtle_names_.begin());
    need_to_update_values_ = true;
}


void Catcher::update_target_turtle_pose_sub_(){
    if(turtle_names_.empty() || !need_to_update_values_) {
        return;
    }

    target_turtle_name = turtle_names_.front();
    target_turtle_pose_sub_ = this->create_subscription<Pose>(
        target_turtle_name + "/pose", 10,
        std::bind(&Catcher::target_pose_callback, this, std::placeholders::_1)); 
}


void Catcher::log_debug_data() const {
    if (turtle_names_.empty()) {
        RCLCPP_INFO(this->get_logger(), "No turtles available.");
        return;
    }

    std::string names_list = std::accumulate(
        turtle_names_.begin(), turtle_names_.end(), std::string(),
         [](const std::string &a, const std::string &b){ return a + (a.empty() ? "" : " ") + b;}); 
    
    RCLCPP_INFO(this->get_logger(), "Turtules:%s", names_list.c_str());
    
    RCLCPP_INFO(this->get_logger(), "Target:%s", target_turtle_name.c_str());
    
    if(target_turtle_pose_sub_){
        RCLCPP_INFO(this->get_logger(), "Topic:%s", std::to_string(target_turtle_pose_sub_->get_publisher_count()).c_str());
    }
}


void Catcher::move_turtle_callback(){
    double distance_x = target_pose_.x - current_pose_.x;
    double distance_y = target_pose_.y - current_pose_.y;
    double distance = std::hypot(distance_x, distance_y);
    double angle = atan2(distance_y, distance_x);
    double diff_theta = angle - current_pose_.theta;

    // Normalize angle to [-pi, pi]
    diff_theta = std::fmod(diff_theta + M_PI, 2 * M_PI) - M_PI;
    
    Twist msg; 
    if(distance < 0.5 ) {
        msg.linear.x = 0.0;
        msg.angular.z = 0.0;
        set_next_goal();
    } else {
        msg.linear.x = 2 * distance;
        msg.angular.z = 6 * diff_theta;
    }

    update_target_turtle_pose_sub_();
    velocity_pub_->publish(msg); 
}

void Catcher::spawn_radom_turtle_callback(){
    static uint16_t counter = 1; 
    
    auto request = std::make_shared<Spawn::Request>();
    request->name = "turtle_num_"+ std::to_string(counter);

    if (!spawn_random_turtle_client_->service_is_ready()) {     
        RCLCPP_ERROR(this->get_logger(), "Spawn service not available after waiting");
        return;
    }

    auto future = spawn_random_turtle_client_->async_send_request(request);
    
    if(request->name != ""){ // check for new valid turtle name
        if (turtle_names_.empty()){
            target_turtle_name = request->name;
            need_to_update_values_ = true;
            update_target_turtle_pose_sub_();
        }; 
        turtle_names_.push_back(request->name);
    }

    counter++;
}

void Catcher::pose_callback(const turtlesim::msg::Pose::SharedPtr msg){
    current_pose_ = *msg;
}

void Catcher::target_pose_callback(const turtlesim::msg::Pose::SharedPtr msg){
    target_pose_ = *msg;
    need_to_update_values_ = false;
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto mover_node = std::make_shared<Catcher>();
    rclcpp::spin(mover_node);
    rclcpp::shutdown();
    return 0;
}