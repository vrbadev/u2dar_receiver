#include <ros/ros.h>
#include <ros/package.h>
#include <csignal>
#include <time.h>

#include <u2dar_receiver_msgs/AudioPacket.h>
#include <std_srvs/SetBool.h>

extern "C" {
#include "rpi5_periph/rp1.h"
#include "rpi5_periph/i2c.h"
#include "rpi5_periph/spi.h"
#include "max9939/max9939.h"
#include "max262/max262.h"
#include "alsa_pcm1822/alsa_pcm1822.h"
}

#define abs(x) ((x) < 0 ? -(x) : (x))

#define GPIO_PCM1822_MSZ 6
#define GPIO_PCM1822_MD0 16
#define GPIO_PCM1822_MD1 26
#define GPIO_PCM1822_WS 23
#define GPIO_SYNC_EVCAM 15

#define GPIO_MAX262_A0 1 
#define GPIO_MAX262_A1 7
#define GPIO_MAX262_A2 25
#define GPIO_MAX262_A3 24
#define GPIO_MAX262_D0 22 
#define GPIO_MAX262_D1 0
#define GPIO_MAX262_CLKA 12
#define GPIO_MAX262_CLKB 13
#define GPIO_MAX262_NWR 27


#define NODE_NAME "[RecorderNode] "
std::function<void(void)> shutdown_handler;

class RecorderNode {
public:
    RecorderNode(ros::NodeHandle& nh) : node_(nh) {
        ROS_INFO(NODE_NAME "Initializing recorder node...");

        nh.param<std::string>("i2c_dev", i2c_dev_, "/dev/i2c-1");
        nh.param<std::string>("spi_dev", spi_dev_, "/dev/spidev0.0");
        nh.param<std::string>("alsa_dev", alsa_dev_, "hw:0");
        nh.param<double>("pga_gain", pga_gain_, (double) 10.0f);
        nh.param<double>("pga_offset", pga_offset_, (double) 0.0f);

        // setup I2C for sensors
        i2c_handle_ = (i2c_handle_t*) malloc(sizeof(i2c_handle_t));
        i2c_handle_->dev_path = (const char*) i2c_dev_.c_str();
        if (i2c_init(i2c_handle_) != 0) {
            ROS_ERROR(NODE_NAME "Failed to open I2C device '%s'!", i2c_handle_->dev_path);
            return;
        }

        if (setup_audio()) {
            return;
        }

        rp1_pwm_chan_config(rp1_handle_->pwm0, 2, 1, 50, 0, 25);
        rp1_pwm_chan_enable(rp1_handle_->pwm0, 2, 1);

        pub_audio_ = nh.advertise<u2dar_receiver_msgs::AudioPacket>("audio", 1);

        initialized_ = true;
        shutdown_handler = [&]() { this->~RecorderNode(); };

        ROS_INFO(NODE_NAME "Recorder node initialized.");
    };

    ~RecorderNode() {
        if (initialized_) {
            ROS_INFO(NODE_NAME "Shutting down recorder node...");

            alsa_pcm1822_deinit(alsa_handle_);
            free(alsa_handle_);

            max262_deinit(max262_handle_);
            free(max262_handle_);

            i2c_deinit(i2c_handle_);
            free(i2c_handle_);

            spi_deinit(spi_handle_);
            free(spi_handle_);

            rp1_deinit(rp1_handle_);
            free(rp1_handle_);

            shutdown_handler = nullptr;
            initialized_ = false;
        }
    };

    int setup_audio() {
        // setup SPI for MAX9939 PGA
        spi_handle_ = (spi_handle_t*) malloc(sizeof(spi_handle_t));
        spi_handle_->dev_path = (const char*) spi_dev_.c_str();
        spi_init(spi_handle_);
        spi_set_mode(spi_handle_, SPI_MODE_0 | SPI_CS_HIGH);
        spi_set_bits_per_word(spi_handle_, 8);
        spi_set_speed(spi_handle_, 1000000);

        // set PGA gain and offset
        ROS_WARN(NODE_NAME "Writing PGA gain=%.2fx, offset=%.2fmV", pga_gain_, pga_offset_);
        max9939_set_offset(spi_handle_, pga_offset_, false, false);
        max9939_set_gain(spi_handle_, pga_gain_, false, false);

        // setup RP1 for GPIO and PWM control
        rp1_handle_ = (rp1_t*) malloc(sizeof(rp1_t));
        if (rp1_init(rp1_handle_)) {
            ROS_ERROR(NODE_NAME "Failed to map /dev/mem!");
            return -2;
        }

        // setup PCM1822 ADC control pins, set all to output low
        rp1_gpio_funcsel(rp1_handle_, GPIO_PCM1822_MSZ, 5);
        rp1_gpio_config_pulldown(rp1_handle_, GPIO_PCM1822_MSZ);
        rp1_sys_rio_config_output(rp1_handle_, GPIO_PCM1822_MSZ);
        rp1_sys_rio_out_clr(rp1_handle_, GPIO_PCM1822_MSZ);

        rp1_gpio_funcsel(rp1_handle_, GPIO_PCM1822_MD0, 5);
        rp1_gpio_config_pulldown(rp1_handle_, GPIO_PCM1822_MD0);
        rp1_sys_rio_config_output(rp1_handle_, GPIO_PCM1822_MD0);
        rp1_sys_rio_out_clr(rp1_handle_, GPIO_PCM1822_MD0);

        rp1_gpio_funcsel(rp1_handle_, GPIO_PCM1822_MD1, 5);
        rp1_gpio_config_pulldown(rp1_handle_, GPIO_PCM1822_MD1);
        rp1_sys_rio_config_output(rp1_handle_, GPIO_PCM1822_MD1);
        rp1_sys_rio_out_clr(rp1_handle_, GPIO_PCM1822_MD1);

        // setup ALSA audio recorder
        alsa_handle_ = (alsa_pcm1822_t*) malloc(sizeof(alsa_pcm1822_t));
        alsa_handle_->dev_name = (const char*) alsa_dev_.c_str();
        alsa_handle_->sample_rate = 192000;
        alsa_handle_->buffer_frames_num = alsa_handle_->sample_rate;
        alsa_handle_->gpio_sync = GPIO_SYNC_EVCAM;
        alsa_handle_->gpio_pcm_ws = GPIO_PCM1822_WS;
        alsa_handle_->rp1_handle = rp1_handle_;
        if (alsa_pcm1822_init(alsa_handle_)) {
            ROS_ERROR(NODE_NAME "Failed to open ALSA recorder '%s'!", alsa_handle_->dev_name);
            return -1;
        }

        // setup switched cap filter MAX262
        max262_handle_ = (max262_t*) malloc(sizeof(max262_t));
        max262_handle_->gpios_a[0] = GPIO_MAX262_A0;
        max262_handle_->gpios_a[1] = GPIO_MAX262_A1;
        max262_handle_->gpios_a[2] = GPIO_MAX262_A2;
        max262_handle_->gpios_a[3] = GPIO_MAX262_A3;
        max262_handle_->gpios_d[0] = GPIO_MAX262_D0;
        max262_handle_->gpios_d[1] = GPIO_MAX262_D1;
        max262_handle_->gpio_nwr = GPIO_MAX262_NWR;
        max262_handle_->gpios_clk[0] = GPIO_MAX262_CLKA;
        max262_handle_->gpios_clk[1] = GPIO_MAX262_CLKB;
        max262_handle_->rp1_handle = rp1_handle_;
        max262_init(max262_handle_);

        return 0;
    }

    void run() {
        if (!initialized_) return;

        ROS_INFO(NODE_NAME "Running recorder node...");

        u2dar_receiver_msgs::AudioPacket msg;
        //FILE *output = fopen("/home/pi/raw_i32.bin", "wb");

        uint32_t total_frames = alsa_handle_->buffer_frames_num * 2;
        //msg.adc_data = std::vector<int32_t>(total_frames);
        /*msg.adc_ch0_data.clear();
        msg.adc_ch1_data.clear();*/

        uint32_t packet_id = 0;
        ros::Rate rate(100);

        while (ros::ok()) {
            if (alsa_pcm1822_read(alsa_handle_)) {
                //shutdown_handler();
                continue;
            }
            
            msg.adc_data.assign(alsa_handle_->buffer, alsa_handle_->buffer + total_frames);
            /*memcpy(&msg.adc_data[0], alsa_handle_->buffer, alsa_handle_->buffer_size);*/

            /*for (uint32_t i = 0; i < alsa_handle_->buffer_frames_num; i++) {
                msg.adc_ch0_data.push_back(alsa_handle_->buffer[2*i]);
                msg.adc_ch1_data.push_back(alsa_handle_->buffer[2*i+1]);
            }*/

            msg.stamp_trigger = ros::Time(alsa_handle_->hstamp_trigger.tv_sec, alsa_handle_->hstamp_trigger.tv_nsec);
            msg.stamp_audio = ros::Time(alsa_handle_->hstamp_audio.tv_sec, alsa_handle_->hstamp_audio.tv_nsec);
            msg.stamp_sync = ros::Time(alsa_handle_->stamp_sync.tv_sec, alsa_handle_->stamp_sync.tv_nsec);
            msg.pga_gain = pga_gain_;
            msg.pga_offset = pga_offset_;
            msg.sample_rate = alsa_handle_->sample_rate;
            msg.packet_id = packet_id;
            msg.stamp_pub = ros::Time::now();
            pub_audio_.publish(msg);

            /*msg.adc_ch0_data.clear();
            msg.adc_ch1_data.clear();*/
            if (packet_id == 0) {
                ROS_WARN(NODE_NAME "ALSA trigger: %lu.%09lu", alsa_handle_->hstamp_trigger.tv_sec, alsa_handle_->hstamp_trigger.tv_nsec);
                ROS_WARN(NODE_NAME "Event cam trigger: %lu.%09lu", alsa_handle_->stamp_sync.tv_sec, alsa_handle_->stamp_sync.tv_nsec);
            }

            if (packet_id % 1 == 0) {
                ROS_INFO(NODE_NAME "Audio stamp: %lu.%09lu", alsa_handle_->hstamp_audio.tv_sec, alsa_handle_->hstamp_audio.tv_nsec);
            }
            
            packet_id++;
            //fwrite(alsa_handle_->buffer, alsa_handle_->buffer_size, 1, output);

            double sum1 = 0;
            double sum2 = 0;
            for (uint32_t i = 0; i < alsa_handle_->buffer_frames_num; i++) {
                sum1 += abs(alsa_handle_->buffer[2*i]);
                sum2 += abs(alsa_handle_->buffer[2*i+1]);
            }
            ROS_INFO(NODE_NAME "CH2/CH1 sums ratio: %f", sum2 / sum1);

            //ROS_INFO(NODE_NAME "Read done.");

            ros::spinOnce();
            rate.sleep();
        }

        //fclose(output);
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
    alsa_pcm1822_t* alsa_handle_;
    max262_t* max262_handle_;

    ros::Publisher pub_audio_;
};

int32_t main(int32_t argc, char** argv) {
    ros::init(argc, argv, "recorder_node");

    ros::NodeHandle nh("~");
    RecorderNode rn(nh);
    std::signal(SIGINT, [](int) { if(shutdown_handler) { shutdown_handler(); exit(0); } else { exit(-1); } });
    rn.run();

    return 0;
}
