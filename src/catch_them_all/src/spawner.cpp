
#include "rclcpp/rclcpp.hpp"
#include "random"
#include "cmath"
#include "turtlesim/srv/spawn.hpp"
#include "std_msgs/msg/string.hpp"
#include "turtlesim/srv/spawn.hpp"

using namespace turtlesim::srv;
using namespace std::placeholders;

constexpr float SCREEN_SIZE = 10.0;

class Spawner : public rclcpp::Node {
public:
        Spawner() : Node("spawner_node"){
        spawn_client_ = this->create_client<Spawn>("spawn");
        spawn_server_ = this->create_service<turtlesim::srv::Spawn>("spawn_random_turtle",
            std::bind(&Spawner::spawn_random_turtle_callback, this, _1, _2));
    }
private:
    void spawn_random_turtle_callback(const Spawn::Request::SharedPtr, Spawn::Response::SharedPtr);
    std::pair<float, float> generate_random_postion();
    float generate_random_theta();
    void publish_turtle_names();

    rclcpp::Client<Spawn>::SharedPtr spawn_client_;
    rclcpp::Service<Spawn>::SharedPtr spawn_server_;
};


void Spawner::spawn_random_turtle_callback(const Spawn::Request::SharedPtr request, Spawn::Response::SharedPtr response){
    static uint8_t counter = 1; 
    auto [x, y] = generate_random_postion(); 
    float theta = generate_random_theta();

    request->x = x;
    request->y = y;
    request->theta = theta;
    request->name = "turtle_num_"+ std::to_string(counter);

    // Check if the client is ready
    if (!spawn_client_->wait_for_service(std::chrono::seconds(1))) {     
        RCLCPP_ERROR(this->get_logger(), "Service not available after waiting");
        return;
    }

    auto future = spawn_client_->async_send_request(request);
    counter++;
    response->name = request->name;
}

std::pair<float, float> Spawner::generate_random_postion() {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<float> dis(0.0, SCREEN_SIZE);
    return {dis(gen), dis(gen)};
}

float Spawner::generate_random_theta(){
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<float> theta_dis(0.0, M_PI);
    return theta_dis(gen);
}

int main(int argc, char **argv) {    
    rclcpp::init(argc, argv);
    auto my_spawner_node = std::make_shared<Spawner>();
    rclcpp::spin(my_spawner_node);
    rclcpp::shutdown();
    return 0;
}