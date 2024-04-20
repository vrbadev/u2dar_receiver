#include <ros/ros.h>
#include <ros/package.h>
#include <csignal>

#include <std_srvs/SetBool.h>

extern "C" {
#include "rpi5_periph/rp1.h"
#include "rpi5_periph/rp1_spi.h"
#include "rpi5_periph/i2c.h"
#include "max9939/max9939.h"
#include "alsa_pcm1822/alsa_pcm1822.h"
}

std::function<void(void)> shutdown_handler;

class RecorderNode {
public:
    RecorderNode(ros::NodeHandle& nh) : node_(nh) {
        ROS_INFO("[RecorderNode]: Initializing recorder node...");

        nh.param<std::string>("i2c_dev", i2c_dev_, "/dev/i2c-1");

        i2c_handle_ = (i2c_handle_t) { .dev_path = (const char*) i2c_dev_.c_str(), .handle = -1 };
        if (i2c_init(&i2c_handle_) != 0) {
            ROS_ERROR("[RecorderNode]: Failed to open I2C device '%s'!", i2c_handle_.dev_path);
            return;
        }

        initialized_ = true;
        shutdown_handler = [&]() { this->~RecorderNode(); };

        ROS_INFO("[RecorderNode]: Recorder node initialized.");
    };

    ~RecorderNode() {
        ROS_INFO("[RecorderNode]: Shutting down recorder node...");
        i2c_deinit(&i2c_handle_);
    };

    void run() {
        if (!initialized_) return;

        ROS_INFO("[RecorderNode]: Running recorder node...");

        while (ros::ok()) {

            ros::spinOnce();
        }
    };
private:
    bool initialized_ = false;
    ros::NodeHandle& node_;

    std::string i2c_dev_;
    
    i2c_handle_t i2c_handle_;
    rp1_t rp1_handle_;
    rp1_spi_t rp1_spi_handle_;
    alsa_pcm1822_t alsa_pcm1822_handle_;

};

int32_t main(int32_t argc, char** argv) {
    ros::init(argc, argv, "recorder_node");

    ros::NodeHandle nh("~");
    RecorderNode rn(nh);
    std::signal(SIGINT, [](int) { if(shutdown_handler) { shutdown_handler(); } exit(0); });
    rn.run();

    ros::spin();
    return 0;
}
