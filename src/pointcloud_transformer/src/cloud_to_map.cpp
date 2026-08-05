#include <memory>
#include <functional>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

#include <tf2/exceptions.h>


class CloudToMap : public rclcpp::Node
{

public:

    CloudToMap(): Node("cloud_to_map")
    {
        //订阅 FAST_LIO 输出点云  frame_id:camera_init
        cloud_sub_ =
            this->create_subscription<sensor_msgs::msg::PointCloud2>(
                "/cloud_registered",
                20,
                std::bind(&CloudToMap::cloudCallback,this,std::placeholders::_1)
            );

        //发布世界坐标系点云frame_id:map
        cloud_pub_ =
            this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_registered_map",20);

        //TF Buffer
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
       
        //TF Listener
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        RCLCPP_INFO(this->get_logger(),"cloud_to_map started:/cloud_registered(camera_init)-> /cloud_registered_map(map)");
    }
private:
    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        if(msg->data.empty())
        {
            return;
        }

        // FAST_LIO正常情况:cloud frame: camera_init    TF: map->camera_init
        if(msg->header.frame_id.empty())
        {
            RCLCPP_WARN(this->get_logger(),"Cloud frame_id is empty");
            return;
        }
        geometry_msgs::msg::TransformStamped transform;
        try
        {
            transform = tf_buffer_->lookupTransform(
                    "map",
                    msg->header.frame_id,
                    tf2::TimePointZero,  // msg->header.stamp,   //  tf2::TimePointZero
                    tf2::durationFromSec(0.1)    
                );
        }
        catch(const tf2::TransformException &ex)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(),*this->get_clock(),2000,"TF lookup failed: %s",ex.what());
            return;
        }
        sensor_msgs::msg::PointCloud2 cloud_map;
        try
        {
            tf2::doTransform(
                *msg,
                cloud_map,
                transform
            );
        }
        catch(const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(),"PointCloud transform failed: %s",e.what());
            return;
        }
         //强制设置输出坐标系
        cloud_map.header.frame_id = "map";
        //保留原始时间
        cloud_map.header.stamp = msg->header.stamp;
        cloud_pub_->publish(cloud_map);
    }
private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc,char **argv)
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<CloudToMap>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
