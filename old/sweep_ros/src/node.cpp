
/*The MIT License (MIT)
 *
 * Copyright (c) 2017, Scanse, LLC
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "rclcpp/rclcpp.hpp"
#include <pcl/point_types.h>
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/string.hpp"
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sweep/sweep.hpp>
#include <string>

void publish_scan(
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub,
    const sweep::scan *scan,
    std::string frame_id,
    rclcpp::Time ros_now
)
{
    pcl::PointCloud <pcl::PointXYZ> cloud;
    sensor_msgs::msg::PointCloud2 cloud_msg;

    float angle;
    int32_t range;
    float x;
    float y;
    int i = 0;

    cloud.height = 1;
    cloud.width = scan->samples.size();
    cloud.points.resize(cloud.width * cloud.height);

    for (const sweep::sample& sample : scan->samples)
    {
        range = sample.distance;
        angle = ((float)sample.angle / 1000); //millidegrees to degrees

        //Polar to Cartesian Conversion
        x = (range * cos(DEG2RAD(angle))) / 100;
        y = (range * sin(DEG2RAD(angle))) / 100;

        cloud.points[i].x = x;
        cloud.points[i].y = y;
        i++;
    }

    //Convert pcl PC to ROS PC2
    pcl::toROSMsg(cloud, cloud_msg);
    cloud_msg.header.frame_id = frame_id;
    cloud_msg.header.stamp = ros_now;

    pub->publish(cloud_msg);
}


class SweeperDriver

{
    public:
        std::unique_ptr<sweep::sweep> device;

        SweeperDriver(){

        }

        void setSweep(
            int rotation_speed,
            std::string serial_port,
            int sample_rate
        ){
            //Create Sweep Driver Object
            device.reset(new sweep::sweep{serial_port.c_str(), 115200});

            //Stop scanning
            device->stop_scanning();
            std::cout << "Stop scanning!" << std::endl; 

            //Send Sample Rate
            std::cout << "Setting sample rate..." << std::endl; 
            device->set_sample_rate(sample_rate);

            std::cout << "Setting motor speed..." << std::endl; 
            //Send Rotation Speed
            device->set_motor_speed(rotation_speed);

            //Start Scan
            std::cout << "Start scanning..." << std::endl; 
            device->start_scanning();
            std::cout << "Scanning!" << std::endl; 

        }
};


int main(int argc, char * argv[])
{
    // Sweeper node
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("sweep_publisher");
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr scan_pub = node->create_publisher<sensor_msgs::msg::PointCloud2>("pc2", 1000);
    
    node->declare_parameter("rotation_speed", 5);
    node->declare_parameter("serial_port", "/dev/serial/by-id/usb-FTDI_FT230X_Basic_UART_DM00LPC9-if00-port0");
    node->declare_parameter("sample_rate", 500); 
    node->declare_parameter("frame_id", "laser_frame");


    int rotation_speed = node->get_parameter("rotation_speed").as_int();
    std::string serial_port = node->get_parameter("serial_port").as_string();
    int sample_rate = node->get_parameter("sample_rate").as_int();
    std::string frame_id = node->get_parameter("frame_id").as_string();

    RCLCPP_INFO(node->get_logger(),
    "Rotation speed: %i (Hz), Serial port: %s, Sample rate: %i, Frame id: %s", 
    rotation_speed, serial_port.c_str(), sample_rate, frame_id.c_str());

    // Sweeper driver
    SweeperDriver sweeper_driver;
    sweeper_driver.setSweep(
        rotation_speed,
        serial_port,
        sample_rate
    );
    
    while (rclcpp::ok())
    {
        const sweep::scan scan = sweeper_driver.device->get_scan();
        RCLCPP_DEBUG(node->get_logger(), "Publishing a full scan");
        publish_scan(scan_pub, &scan, frame_id, node->get_clock()->now());

        rclcpp::spin_some(node);
    }
    rclcpp::shutdown();

    return 0;
}

