#ifndef PCL_DETECTION_H_
#define PCL_DETECTION_H_

#include <iostream>
#include <filesystem>
#include <math.h>

#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/surface/mls.h>

#include <rclcpp/rclcpp.hpp>
#include <cstdio>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

#include "ament_index_cpp/get_package_share_directory.hpp"

typedef boost::shared_ptr<tf2_ros::Buffer> tfBufferPtr;

namespace pcl_detection
{

    struct DetectedPlane
    {
        std::string name;

        pcl::PointXYZ min_pt;
        pcl::PointXYZ max_pt;

        // Convenience method to get center
        pcl::PointXYZ getCenter() const {
            return pcl::PointXYZ{
            (min_pt.x + max_pt.x) / 2.0,
            (min_pt.y + max_pt.y) / 2.0,
            (min_pt.z + max_pt.z) / 2.0
            };
        }

        // Convenience method to get dimensions
        pcl::PointXYZ getDimensions() const {
            return pcl::PointXYZ{
            std::abs(max_pt.x - min_pt.x), // length
            std::abs(max_pt.y - min_pt.y), // width
            std::abs(max_pt.z - min_pt.z)  // height
            };
        }
    };

    class PCLDetection
    {
    public:
        PCLDetection(const rclcpp::Node::SharedPtr& node, tfBufferPtr& tf_buffer_ptr);
        // {


        //     // moveit_visual_tools_.reset(new moveit_visual_tools::MoveItVisualTools(node_, "base_link", "/moveit_visual_markers"));
		// 	// moveit_visual_tools_->loadPlanningSceneMonitor();
		// 	// moveit_visual_tools_->loadMarkerPub(true);
		// 	// moveit_visual_tools_->waitForMarkerSub();
		// 	// moveit_visual_tools_->loadRobotStatePub("display_robot_state");
		// 	// moveit_visual_tools_->setManualSceneUpdating();
		// 	// moveit_visual_tools_->loadRemoteControl();
		// 	// moveit_visual_tools_->deleteAllMarkers();
        // }

        ~PCLDetection();

        rclcpp::NodeOptions node_options;
		rclcpp::Node::SharedPtr node_;

        // tfBufferPtr tf_buffer_ptr;
        tfBufferPtr& tf_buffer_ptr_;

        moveit_visual_tools::MoveItVisualToolsPtr moveit_visual_tools_;

        // protected:
        std::vector<DetectedPlane> planes_info;

        // void process_pcl_data(const std::string& input_pcd, const std::string& output_pcd);

        // void publish_collision_plane(
        //     rclcpp::Node::SharedPtr node, 
        //     double width = 0.599983, 
        //     double length = 1.38467, 
        //     double height = 0.01);

        // void publish_center_link(rclcpp::Node::SharedPtr node);

        void voxel_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud, float leafsize);

        void passthrough_filter(
            pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, 
            pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud, 
            const std::map<std::string, std::pair<float, float>>& limits);

        void statistical_outlier_removal(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud);

        void moving_least_squares(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud);

        void all_plane_seg(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud,
            pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud,
            // std::vector<DetectedPlane>& planes_info,
            bool show_plane = true,
            bool aligned_plane = false,
            int max_planes = 10,
            int min_inliers = 100,
            bool horizontal_only = false);

        void addDetectedPlane(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_plane);

        void estimate_plane_bbox(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud);

        void printDetectedPlane(const DetectedPlane& plane);

        void process_test_pcl_data(
            const std::string& input_pcd, 
            const std::string& output_pcd);

        void process_pcl_data(
            const std::string& input_pcd, 
            const std::string& output_pcd);

        geometry_msgs::msg::Point transformPointToFrame(
            const geometry_msgs::msg::Point& point_in,
            const std::string& from_frame,
            const std::string& to_frame,
            rclcpp::Time stamp);

        void publish_center_link(const DetectedPlane& plane);

    }; // class PCLDetection

} // namespace pcl_detection

#endif /*PCL_DETECTION_H_*/