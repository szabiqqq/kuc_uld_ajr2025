#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/srv/teleport_absolute.hpp"
#include "turtlesim/srv/set_pen.hpp"
#include <chrono>

using namespace std::chrono_literals;

class DrawHouse : public rclcpp::Node {
public:
    DrawHouse() : Node("tervrajz") {
        velocity_pub = create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
        teleport_client = create_client<turtlesim::srv::TeleportAbsolute>("/turtle1/teleport_absolute");
        pen_client = create_client<turtlesim::srv::SetPen>("/turtle1/set_pen");

        RCLCPP_INFO(get_logger(), "Spawning in center, then teleporting...");

        rclcpp::sleep_for(1s); 
        set_pen(false);
        teleport(4.0, 3.0, 0.0);  
        rclcpp::sleep_for(1s);  
        set_pen(true);

        move_forward(3.0);  // Move RIGHT
        set_pen(false); teleport(7.0, 3.0, M_PI_2); set_pen(true);  // Rotate UP
        move_forward(3.0);  // Move UP
        set_pen(false); teleport(7.0, 6.0, M_PI); set_pen(true);  // Rotate LEFT
        move_forward(3.0);  // Move LEFT
        set_pen(false); teleport(4.0, 6.0, -M_PI_2); set_pen(true);  // Rotate DOWN
        move_forward(3.0);  // Move DOWN (back to start)

        set_pen(false);
        RCLCPP_INFO(get_logger(), "Square complete! Now adding roof...");

        set_pen(false);
        teleport(4.0, 6.0, M_PI_4); // Face diagonally up-right
        set_pen(true);

        move_forward(2.1);  // Move up-right to peak at (5.5, 7.4)

        set_pen(false);
        teleport(5.5, 7.4, -M_PI_4);
        set_pen(true);

        move_forward(2.1);  // Move down-right to (7.0, 6.0)

        set_pen(false);
        RCLCPP_INFO(get_logger(), "House complete!");
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_pub;
    rclcpp::Client<turtlesim::srv::TeleportAbsolute>::SharedPtr teleport_client;
    rclcpp::Client<turtlesim::srv::SetPen>::SharedPtr pen_client;

    void teleport(double x, double y, double theta) {
        auto request = std::make_shared<turtlesim::srv::TeleportAbsolute::Request>();
        request->x = x; request->y = y; request->theta = theta;
        teleport_client->async_send_request(request);
        rclcpp::sleep_for(500ms);
    }

    void set_pen(bool on) {
        auto request = std::make_shared<turtlesim::srv::SetPen::Request>();
        request->r = 0; request->g = 0; request->b = 0; request->width = 3;
        request->off = !on;
        pen_client->async_send_request(request);
    }

    void move_forward(double distance) {
        geometry_msgs::msg::Twist msg;
        msg.linear.x = 1.0;  // Move speed
        rclcpp::Time start = now();
        while ((now() - start).seconds() < distance / msg.linear.x) {
            velocity_pub->publish(msg);
            rclcpp::sleep_for(100ms);
        }
        velocity_pub->publish(geometry_msgs::msg::Twist());  // Stop
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DrawHouse>());
    rclcpp::shutdown();
    return 0;
}