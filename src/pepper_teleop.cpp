#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <termios.h>
#include <unistd.h>
#include <iostream>

class TeleopPepper : public rclcpp::Node
{
public:
    TeleopPepper() : Node("teleop_pepper")
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        std::cout << "Use WASD keys to control the robot, Press i to stop the robot. Press Q to quit." << std::endl;
    }

    void keyLoop()
    {
        char c;
        struct termios oldt, newt;

        // Get the terminal settings for stdin
        tcgetattr(STDIN_FILENO, &oldt);
        newt = oldt;
        // Disable canonical mode and echo
        newt.c_lflag &= ~(ICANON | ECHO);
        // Set the new terminal settings
        tcsetattr(STDIN_FILENO, TCSANOW, &newt);

        while (rclcpp::ok())
        {
            // Get the next character from the keyboard
            c = getchar();

            geometry_msgs::msg::Twist twist;
            switch (c)
            {
            case 'w':
                twist.linear.x = 0.5;
                twist.angular.z = 0.0;
                break;
            case 's':
                twist.linear.x = -0.5;
                twist.angular.z = 0.0;
                break;
            case 'a':
                twist.linear.x = 0.0;
                twist.angular.z = 0.5;
                break;
            case 'd':
                twist.linear.x = 0.0;
                twist.angular.z = -0.5;
                break;
            case 'q':
                // Restore the terminal settings
                tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
                return;
            case 'i':
                twist.linear.x = 0.0;
                twist.angular.z = 0.0;
                break;
            default:
                twist.linear.x = 0.0;
                twist.angular.z = 0.0;
                break;
            }

            publisher_->publish(twist);
        }

        // Restore the terminal settings
        tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TeleopPepper>();
    node->keyLoop();
    rclcpp::shutdown();
    return 0;
}