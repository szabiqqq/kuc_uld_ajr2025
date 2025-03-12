#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/srv/teleport_absolute.hpp"
#include "turtlesim/srv/set_pen.hpp"
#include <chrono>

using namespace std::chrono_literals;

class DrawPyramid : public rclcpp::Node {
public:
    DrawPyramid() : Node("tervrajz") {
        velocity_pub = create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
        teleport_client = create_client<turtlesim::srv::TeleportAbsolute>("/turtle1/teleport_absolute");
        pen_client = create_client<turtlesim::srv::SetPen>("/turtle1/set_pen");

        RCLCPP_INFO(get_logger(), "Rajzolás kezdete: Gúla");

        // Négyzet sarokpontjai
        double base_x1 = 3.0, base_y1 = 3.0;
        double base_x2 = 7.0, base_y2 = 3.0;
        double base_x3 = 7.0, base_y3 = 7.0;
        double base_x4 = 3.0, base_y4 = 7.0;
        double peak_x = 5.0, peak_y = 9.0; // Magasabbra emeltem a csúcsot

        // 1. Alapnégyzet megrajzolása
        teleport(base_x1, base_y1, 0.0);
        set_pen(true);
        draw_line_to(base_x2, base_y2);
        draw_line_to(base_x3, base_y3);
        draw_line_to(base_x4, base_y4);
        draw_line_to(base_x1, base_y1);

        // 2. Gúla oldallapjainak rajzolása
        draw_pyramid_sides(peak_x, peak_y, base_x1, base_y1, base_x2, base_y2, base_x3, base_y3, base_x4, base_y4);

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

    void draw_line_to(double x, double y) {
        teleport(x, y, 0.0);
    }

    void draw_pyramid_sides(double peak_x, double peak_y, 
                             double x1, double y1, double x2, double y2, 
                             double x3, double y3, double x4, double y4) {
        // Oldallapok rajzolása (minden sarokból a csúcsba)
        set_pen(false);
        teleport(x1, y1, 0.0); set_pen(true);
        draw_line_to(peak_x, peak_y);

        set_pen(false);
        teleport(x2, y2, 0.0); set_pen(true);
        draw_line_to(peak_x, peak_y);

        set_pen(false);
        teleport(x3, y3, 0.0); set_pen(true);
        draw_line_to(peak_x, peak_y);

        set_pen(false);
        teleport(x4, y4, 0.0); set_pen(true);
        draw_line_to(peak_x, peak_y);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DrawPyramid>());
    rclcpp::shutdown();
    return 0;
}
