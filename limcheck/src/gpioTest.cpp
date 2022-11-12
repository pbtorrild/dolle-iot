#include <gpio.h>
#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    /* code */
    gpio GPIO("18");
    
    for (int i = 0; i < 100; i++) {
        GPIO.set_value("1");
        usleep(100000);
        GPIO.set_value("0");
        usleep(100000);
    }

    rclcpp::shutdown();
    return 0;
}
