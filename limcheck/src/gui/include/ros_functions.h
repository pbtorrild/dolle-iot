#ifndef ROS_FUNCTIONS_H
#define ROS_FUNCTIONS_H
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <QApplication>
#include <QtQml/qqml.h>
#include <QMetaType>

class MinimalPublisher : public rclcpp::Node
{
  public:
    MinimalPublisher()
    : Node("minimal_publisher"), count_(0)
    {
      publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
      timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this));
    }

  private:
    void timer_callback()
    {
      auto message = std_msgs::msg::String();
      message.data = "Hello, world! " + std::to_string(count_++);
      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      publisher_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;
};

class ROS_QT_BRIDGE : public QObject{
  Q_OBJECT
private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

public:
    //Qt constructor 
    explicit ROS_QT_BRIDGE(QObject* parent = 0) : QObject(parent), Node("minimal_publisher"){
      publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    } 
    //Destructor
    ~ROS_QT_BRIDGE();
    //Qt Stuff
    Q_INVOKABLE void publish_msg(std::string msg_in="empty_message"){
      auto message = std_msgs::msg::String();
      message.data = msg_in;
      publisher_->publish(message);
    } 
    
};

Q_DECLARE_METATYPE(ROS_QT_BRIDGE)
#endif // ROS_FUNCTIONS_H