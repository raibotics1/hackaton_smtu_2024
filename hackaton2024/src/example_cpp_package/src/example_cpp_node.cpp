#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

class ExampleNode : public rclcpp::Node
{
public:
    ExampleNode() : Node("example_cpp_node")
    {
        // Подписка на топик лидара
        lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&ExampleNode::lidar_callback, this, std::placeholders::_1));
        
        // Подписка на топик камеры
        camera_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_raw", 10, std::bind(&ExampleNode::camera_callback, this, std::placeholders::_1));
        
        // Подписка на топик команды движения
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, std::bind(&ExampleNode::cmd_vel_callback, this, std::placeholders::_1));
        
        // Публикация команды движения
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        
        // Запуск движения вперед на 1 метр
        move_forward();
    }

private:
    void lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Lidar data received");
    }

    void camera_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Camera image received");
        
        // Преобразование изображения ROS2 в OpenCV
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }
        
        // Отображение изображения
        cv::imshow("Camera", cv_ptr->image);
        cv::waitKey(1);
    }

    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Command velocity received");
        cmd_vel_msg_ = *msg;
    }

    void move_forward()
    {
        // Команда движения вперед на 1 метр
        cmd_vel_msg_.linear.x = 0.5;  // Скорость движения вперед
        cmd_vel_msg_.angular.z = 0.0;  // Без вращения
        
        // Публикация команды движения
        cmd_vel_pub_->publish(cmd_vel_msg_);
        
        // Пауза для движения на 1 метр
        RCLCPP_INFO(this->get_logger(), "Moving forward for 1 meter");
        std::this_thread::sleep_for(std::chrono::seconds(2));  // Движение в течение 2 секунд
        
        // Остановка движения
        cmd_vel_msg_.linear.x = 0.0;
        cmd_vel_pub_->publish(cmd_vel_msg_);
        RCLCPP_INFO(this->get_logger(), "Stopping");
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    geometry_msgs::msg::Twist cmd_vel_msg_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ExampleNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}