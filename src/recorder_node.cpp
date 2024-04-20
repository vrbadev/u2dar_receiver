#include <ros/ros.h>
#include <ros/package.h>
#include <csignal>

#include <std_srvs/SetBool.h>

extern "C" {
#include "rpi5_periph/rp1.h"
#include "rpi5_periph/i2c.h"
#include "rpi5_periph/spi.h"
#include "max9939/max9939.h"
#include "alsa_pcm1822/alsa_pcm1822.h"
}

#define abs(x) ((x) < 0 ? -(x) : (x))
#define NODE_NAME "[RecorderNode] "
std::function<void(void)> shutdown_handler;

class RecorderNode {
public:
    RecorderNode(ros::NodeHandle& nh) : node_(nh) {
        ROS_INFO(NODE_NAME "Initializing recorder node...");

        nh.param<std::string>("i2c_dev", i2c_dev_, "/dev/i2c-1");
        nh.param<std::string>("spi_dev", spi_dev_, "/dev/spidev0.0");
        nh.param<std::string>("alsa_dev", alsa_dev_, "hw:2");
        nh.param<double>("pga_gain", pga_gain_, (double) 1.0f);
        nh.param<double>("pga_offset", pga_offset_, (double) 0.0f);

        i2c_handle_ = (i2c_handle_t*) malloc(sizeof(i2c_handle_t));
        i2c_handle_->dev_path = (const char*) i2c_dev_.c_str();
        if (i2c_init(i2c_handle_) != 0) {
            ROS_ERROR(NODE_NAME "Failed to open I2C device '%s'!", i2c_handle_->dev_path);
            return;
        }

        alsa_pcm1822_handle_ = (alsa_pcm1822_t*) malloc(sizeof(alsa_pcm1822_t));
        alsa_pcm1822_handle_->dev_name = (const char*) alsa_dev_.c_str();
        alsa_pcm1822_handle_->sample_rate = 192000;
        alsa_pcm1822_handle_->buffer_frames_num = alsa_pcm1822_handle_->sample_rate;
        if (alsa_pcm1822_init(alsa_pcm1822_handle_)) {
            ROS_ERROR(NODE_NAME "Failed to open ALSA recorder '%s'!", alsa_pcm1822_handle_->dev_name);
            return;
        }

        spi_handle_ = (spi_handle_t*) malloc(sizeof(spi_handle_t));
        spi_handle_->dev_path = (const char*) spi_dev_.c_str();
        spi_init(spi_handle_);
        spi_set_mode(spi_handle_, SPI_MODE_3 | SPI_CS_WORD);
        spi_set_bits_per_word(spi_handle_, 8);
        spi_set_speed(spi_handle_, 1000000);

        max9939_set_offset(spi_handle_, pga_offset_, false, false);
        max9939_set_gain(spi_handle_, pga_gain_, false, false);

        rp1_handle_ = (rp1_t*) malloc(sizeof(rp1_t));
        if (rp1_init(rp1_handle_)) {
            ROS_ERROR(NODE_NAME "Failed to map /dev/mem!");
            return;
        }
        rp1_gpio_funcsel(rp1_handle_, 15, 0);

        initialized_ = true;
        shutdown_handler = [&]() { this->~RecorderNode(); };

        ROS_INFO(NODE_NAME "Recorder node initialized.");
    };

    ~RecorderNode() {
        ROS_INFO(NODE_NAME "Shutting down recorder node...");

        alsa_pcm1822_deinit(alsa_pcm1822_handle_);
        free(alsa_pcm1822_handle_);

        i2c_deinit(i2c_handle_);
        free(i2c_handle_);

        spi_deinit(spi_handle_);
        free(spi_handle_);

        rp1_deinit(rp1_handle_);
        free(rp1_handle_);

        shutdown_handler = nullptr;
    };

    void run() {
        if (!initialized_) return;

        ROS_INFO(NODE_NAME "Running recorder node...");

        FILE *output = fopen("/home/pi/raw_i32.bin", "wb");

        while (ros::ok()) {
            if (alsa_pcm1822_read(alsa_pcm1822_handle_)) {
                shutdown_handler();
                break;
            }

            fwrite(alsa_pcm1822_handle_->buffer, alsa_pcm1822_handle_->buffer_size, 1, output);
            ROS_INFO(NODE_NAME "Timestamp: %lu.%09lu", alsa_pcm1822_handle_->hstamp_audio.tv_sec, alsa_pcm1822_handle_->hstamp_audio.tv_nsec);

            double sum1 = 0;
            double sum2 = 0;
            for (uint32_t i = 0; i < alsa_pcm1822_handle_->buffer_frames_num; i++) {
                sum1 += abs(alsa_pcm1822_handle_->buffer[2*i]);
                sum2 += abs(alsa_pcm1822_handle_->buffer[2*i+1]);
            }
            ROS_INFO(NODE_NAME "CH2/CH1 sums ratio: %f", sum2 / sum1);

            ros::spinOnce();
        }

        fclose(output);
    };
private:
    bool initialized_ = false;
    ros::NodeHandle& node_;

    std::string i2c_dev_;
    std::string spi_dev_;
    std::string alsa_dev_;
    double pga_gain_;
    double pga_offset_;
    
    i2c_handle_t* i2c_handle_;
    rp1_t* rp1_handle_;
    spi_handle_t* spi_handle_;
    alsa_pcm1822_t* alsa_pcm1822_handle_;
};

int32_t main(int32_t argc, char** argv) {
    ros::init(argc, argv, "recorder_node");

    ros::NodeHandle nh("~");
    RecorderNode rn(nh);
    std::signal(SIGINT, [](int) { if(shutdown_handler) { shutdown_handler(); exit(0); } else { exit(-1); } });
    rn.run();

    return 0;
}
