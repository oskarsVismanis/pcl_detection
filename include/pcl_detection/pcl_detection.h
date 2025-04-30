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
#include <geometry_msgs/msg/transform_stamped.hpp>

#include "ament_index_cpp/get_package_share_directory.hpp"

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
            std::abs(max_pt.x - min_pt.x),
            std::abs(max_pt.y - min_pt.y),
            std::abs(max_pt.z - min_pt.z)
            };
        }
    };

    class PCLDetection
    {
    public:
        PCLDetection()
        {
            node_options.use_intra_process_comms(false);
            node_ = std::make_shared<rclcpp::Node>("plane_detection_node", node_options);

            moveit_visual_tools_.reset(new moveit_visual_tools::MoveItVisualTools(node_, "base_link", "/moveit_visual_markers"));
			moveit_visual_tools_->loadPlanningSceneMonitor();
			moveit_visual_tools_->loadMarkerPub(true);
			moveit_visual_tools_->waitForMarkerSub();
			moveit_visual_tools_->loadRobotStatePub("display_robot_state");
			moveit_visual_tools_->setManualSceneUpdating();
			moveit_visual_tools_->loadRemoteControl();
			moveit_visual_tools_->deleteAllMarkers();
        }

        rclcpp::NodeOptions node_options;
		rclcpp::Node::SharedPtr node_;

        moveit_visual_tools::MoveItVisualToolsPtr moveit_visual_tools_;

        // void process_pcl_data(const std::string& input_pcd, const std::string& output_pcd);

        // void publish_collision_plane(
        //     rclcpp::Node::SharedPtr node, 
        //     double width = 0.599983, 
        //     double length = 1.38467, 
        //     double height = 0.01);

        // void publish_center_link(rclcpp::Node::SharedPtr node);

    }; // class PCLDetection

} // namespace pcl_detection

#endif /*PCL_DETECTION_H_*/