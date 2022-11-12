#if !defined(GPIO_H)
#define GPIO_H

#include <string>
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include "rclcpp/logger.hpp"
#include "rcutils/logging_macros.h"

class gpio
{
private:
    /* data */
    std::string pin_num_;
    std::string value_file_path;
public:
    gpio(std::string pin_num);
    
    //Sets the value of the pin can be "1" or "0"
    void set_value(std::string val);

    ~gpio();

};

gpio::gpio(std::string pin_num)
{
    
    pin_num_ = pin_num;
    value_file_path = "/sys/class/gpio/gpio"+ pin_num +"/value";

    // Export the desired pin by writing to /sys/class/gpio/export
    int fd = open("/sys/class/gpio/export", O_WRONLY);
    if (fd == -1) {
        RCUTILS_LOG_FATAL("Unable to open /sys/class/gpio/export");
    }

    if (write(fd, pin_num.c_str(), strlen(pin_num.c_str())) != (int)strlen(pin_num.c_str())) {
        RCUTILS_LOG_FATAL("Error writing to /sys/class/gpio/export");
    }
    close(fd);
    usleep(100000);
    fd = open(std::string("/sys/class/gpio/gpio"+pin_num+"/direction").c_str(), O_WRONLY);
    if (fd == -1) {
        RCUTILS_LOG_FATAL(std::string("Unable to open /sys/class/gpio/gpio"+pin_num+"/direction").c_str());
    }
    if (write(fd, "out", 3) != 3) {
        RCUTILS_LOG_FATAL("Error writing to /sys/class/gpio/gpio%s/direction",pin_num_.c_str());
        
    }
    close(fd);
}

void gpio::set_value(std::string val)
{
    int fd = open(value_file_path.c_str(), O_WRONLY);
    if (fd == -1) {
        RCUTILS_LOG_ERROR("Unable to open /sys/class/gpio/gpio%s/value",pin_num_.c_str());
    }
    if (write(fd, val.c_str(), 1) != 1) 
    {
        RCUTILS_LOG_ERROR("Unable to open /sys/class/gpio/gpio%s/value",pin_num_.c_str());
    }
    close(fd);
}

gpio::~gpio()
{
    // Unexport the pin by writing to /sys/class/gpio/unexport
    int fd = open("/sys/class/gpio/unexport", O_WRONLY);
    if (fd == -1) {
        RCUTILS_LOG_ERROR("Unable to open /sys/class/gpio/unexport");
    }

    if (write(fd, pin_num_.c_str(), 2) != 2) 
    {
        RCUTILS_LOG_ERROR("Error writing to /sys/class/gpio/unexport");
    }

    close(fd);
}


#endif