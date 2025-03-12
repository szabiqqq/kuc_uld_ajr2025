#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/srv/teleport_absolute.hpp"
#include "turtlesim/srv/set_pen.hpp"
#include <chrono>

using namespace std::chrono_literals;

class DrawPyramid : public rclcpp::Node {
public:
    DrawPyramid() : Node("pyramid_drawing") {
        velocity_pub = create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
        teleport_client = create_client<turtlesim::srv::TeleportAbsolute>("/turtle1/teleport_absolute");
        pen_client = create_client<turtlesim::srv::SetPen>("/turtle1/set_pen");

        RCLCPP_INFO(get_logger(), "Rajzolás kezdete: Gúla");

        // Gúla alapnégyzetének rajzolása
        teleport(3.0, 3.0, 0.0); set_pen(true);
        move_forward(4.0); // jobbra
        turn(90);
        move_forward(4.0); // felfelé
        turn(90);
        move_forward(4.0); // balra
        turn(90);
        move_forward(4.0); // lefelé
        turn(90);
        
        // Gúla csúcsának összekötése az alap pontjaival
        double peak_x = 5.0, peak_y = 7.0;
        set_pen(false);
        teleport(3.0, 3.0, 0.0); set_pen(true);
        draw_line_to(peak_x, peak_y);
        set_pen(false);
        teleport(7.0, 3.0, 0.0); set_pen(true);
        draw_line_to(peak_x, peak_y);
        set_pen(false);
        teleport(3.0, 7.0, 0.0); set_pen(true);
        draw_line_to(peak_x, peak_y);
        set_pen(false);
        teleport(7.0, 7.0, 0.0); set_pen(true);
        draw_line_to(peak_x, peak_y);

        RCLCPP_INFO(get_logger(), "Gúla kész!");
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

    void turn(double angle) {
        geometry_msgs::msg::Twist msg;
        msg.angular.z = M_PI / 2;  // 90 fokos fordulás
        rclcpp::Time start = now();
        while ((now() - start).seconds() < (angle / 90.0)) {
            velocity_pub->publish(msg);
            rclcpp::sleep_for(100ms);
        }
        velocity_pub->publish(geometry_msgs::msg::Twist());  // Stop
    }

    void draw_line_to(double x, double y) {
        teleport(x, y, 0.0);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DrawPyramid>());
    rclcpp::shutdown();
    return 0;
}
