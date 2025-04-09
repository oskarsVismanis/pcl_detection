#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <sensor_msgs/msg/point_cloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

void cloud_cb(const std::shared_ptr<const sensor_msgs::msg::PointCloud2> cloud_msg)
{
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg (*cloud_msg, cloud);

    pcl::PCDWriter cloud_writer;
    std::string path = "/home/oskars/workspace/pcl_ws/src/pcl_detection/src/inputs/";
    cloud_writer.write<pcl::PointXYZ>(path + std::string("realsense.pcd"), cloud, false);
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("point_cloud_data_saving");

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub = 
        node->create_subscription<sensor_msgs::msg::PointCloud2>("/camera/camera/depth/color/points", 1, cloud_cb);

    
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}