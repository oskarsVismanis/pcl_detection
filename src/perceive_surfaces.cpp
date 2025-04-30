// #include <iostream>
// #include <filesystem>
// #include <math.h>

// #include <pcl/common/common.h>
// #include <pcl/common/transforms.h>
// #include <pcl/ModelCoefficients.h>
// #include <pcl/io/pcd_io.h>
// #include <pcl/point_types.h>
// #include <pcl/filters/extract_indices.h>
// #include <pcl/filters/passthrough.h>
// #include <pcl/filters/statistical_outlier_removal.h>
// #include <pcl/features/normal_3d.h>
// #include <pcl/sample_consensus/method_types.h>
// #include <pcl/sample_consensus/model_types.h>
// #include <pcl/segmentation/sac_segmentation.h>
// #include <pcl/filters/voxel_grid.h>

// #include <rclcpp/rclcpp.hpp>
// #include <cstdio>

// #include <moveit/planning_scene_interface/planning_scene_interface.h>
// #include <moveit_visual_tools/moveit_visual_tools.h>

// #include <tf2_ros/static_transform_broadcaster.h>
// #include <geometry_msgs/msg/transform_stamped.hpp>

// #include "ament_index_cpp/get_package_share_directory.hpp"

#include <pcl_detection/pcl_detection.h>

using namespace pcl_detection;

// const rclcpp::Logger LOGGER = rclcpp::get_logger("plane_detection_node");

void voxel_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud, float leafsize)
{
  pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
  voxel_filter.setInputCloud(input_cloud);
  voxel_filter.setLeafSize(leafsize, leafsize, leafsize); // play with (the higher, the larger the distance between the points)
  voxel_filter.filter(*output_cloud);
}

void passthrough_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, 
                        pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud, 
                        const std::map<std::string, std::pair<float, float>>& limits)
{
  pcl::PassThrough<pcl::PointXYZ> pass;

  for (const auto& [axis, range] : limits) {
    pass.setFilterFieldName(axis);
    pass.setFilterLimits(range.first, range.second);
    pass.setInputCloud(input_cloud);
    pass.filter(*output_cloud);
  }

}

void statistical_outlier_removal(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud)
{
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud(input_cloud);
  sor.setMeanK(150);
  sor.setStddevMulThresh(0.5);

  // show outliers
  sor.setNegative (true);
  sor.filter (*output_cloud);
  pcl::PCDWriter writer;
  writer.write<pcl::PointXYZ> ("/home/oskars/workspace/pcl_ws/src/pcl_detection/data/outputs/scene.pcd", *output_cloud, false);
  
  sor.setNegative(false);
  sor.filter(*output_cloud);
}

void moving_least_squares(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud)
{
  
  // Moving Least Squares (MLS) filter (method 2) https://pcl.readthedocs.io/projects/tutorials/en/latest/resampling.html

  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);

  // Output has the PointNormal type in order to store the normals calculated by MLS
  pcl::PointCloud<pcl::PointNormal> mls_points;

  // Init object (second point type is for the normals, even if unused)
  pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;

  mls.setComputeNormals(true);

  // Set parameters
  mls.setInputCloud(input_cloud);
  mls.setPolynomialOrder(2);
  mls.setSearchMethod(tree);
  mls.setSearchRadius(0.03);

  // Reconstruct
  mls.process(mls_points);

  // Save output
  pcl::io::savePCDFile("/home/oskars/workspace/pcl_ws/src/pcl_detection/data/outputs/bun0-mls.pcd", mls_points);

  // convert PointNormal to PointXYZ

  // pcl::PointCloud<pcl::PointXYZ>::Ptr smoothed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  // smoothed_cloud->points.reserve(mls_points.points.size());

  // for (const auto& pt : mls_points.points) {
  //     smoothed_cloud->points.emplace_back(pt.x, pt.y, pt.z);
  // }
  // smoothed_cloud->width = smoothed_cloud->points.size();
  // smoothed_cloud->height = 1;
  // smoothed_cloud->is_dense = true;

  // *output_cloud = *smoothed_cloud; // replace the original filtered cloud
}

void all_plane_seg(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud,
                   pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud,
                   std::vector<DetectedPlane>& planes_info,
                   bool show_plane = true,
                   bool aligned_plane = false,
                   int max_planes = 10,
                   int min_inliers = 100,
                   bool horizontal_only = false)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_remaining(new pcl::PointCloud<pcl::PointXYZ>(*input_cloud));
  pcl::ExtractIndices<pcl::PointXYZ> extractor;
  pcl::SACSegmentation<pcl::PointXYZ> seg;

  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(100);
  seg.setDistanceThreshold(0.005);

  pcl::PointCloud<pcl::PointXYZ>::Ptr all_planes(new pcl::PointCloud<pcl::PointXYZ>);

  for (int i = 0; i < max_planes; ++i)
  {
    pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients);

    seg.setInputCloud(cloud_remaining);
    seg.segment(*inliers_plane, *coefficients_plane);

    if (inliers_plane->indices.size() < min_inliers)
      break; // Stop if too few inliers

    // for filtering horizontal  planes
    float a = coefficients_plane->values[0];
    float b = coefficients_plane->values[1];
    float c = coefficients_plane->values[2];
    Eigen::Vector3f normal(a, b, c);
    normal.normalize();

    float dot_with_y = std::abs(normal.dot(Eigen::Vector3f::UnitY()));
    float dot_with_z = std::abs(normal.dot(Eigen::Vector3f::UnitZ()));

    bool is_horizontal = (dot_with_y > 0.2f);
    // bool is_horizontal = (dot_with_z > 0.4f);

    if (horizontal_only && !is_horizontal) {
      std::cout << "Skipping non-horizontal plane.\n";
    } else {
      // Extract current plane
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>);
      extractor.setInputCloud(cloud_remaining);
      extractor.setIndices(inliers_plane);
      extractor.setNegative(false); // extract plane
      extractor.filter(*cloud_plane);

      *all_planes += *cloud_plane; // accumulate planes if show_plane == true
    
      // Store plane info
      DetectedPlane plane;
      plane.name = "plane_" + std::to_string(i);
      pcl::getMinMax3D(*cloud_plane, plane.min_pt, plane.max_pt);
      planes_info.push_back(plane);
    }

    // Remove current plane from remaining cloud
    extractor.setNegative(true);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_temp(new pcl::PointCloud<pcl::PointXYZ>);
    extractor.filter(*cloud_temp);
    cloud_remaining.swap(cloud_temp);
  }

  if (show_plane)
    *output_cloud = *all_planes;
  else
    *output_cloud = *cloud_remaining;

  // align plane to horizontal plane
}

void plane_segmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, 
                        pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud, 
                        bool show_plane = true, 
                        bool aligned_plane = false)
{
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>());

  seg.setOptimizeCoefficients(true);
  seg.setMethodType(pcl::SACMODEL_PLANE); // search for planes
  seg.setModelType(pcl::SAC_RANSAC); // use RANSAC for plane segmentation
  seg.setMaxIterations(100);
  seg.setDistanceThreshold(0.005); // 0.01
  seg.setInputCloud(input_cloud);
  seg.segment(*inliers_plane, *coefficients_plane);

      // extract plane from image
  pcl::ExtractIndices<pcl::PointXYZ> neg_plane_extracted;
  neg_plane_extracted.setInputCloud(input_cloud);
  neg_plane_extracted.setIndices(inliers_plane);
  
  neg_plane_extracted.setNegative(!show_plane);
  
  neg_plane_extracted.filter(*output_cloud);

  // align plane to horizontal plane
  if(aligned_plane){
      // 1. Extract plane normal
    Eigen::Vector3f normal(coefficients_plane->values[0],
                           coefficients_plane->values[1],
                           coefficients_plane->values[2]);
    normal.normalize();

    // 2. Define target normal (Z axis)
    Eigen::Vector3f z_axis(0.0f, 0.0f, 1.0f);

    // 3. Compute initial angle
    float dot_product = normal.dot(z_axis);
    float angle_rad = std::acos(dot_product);
    float angle_deg = angle_rad * 180.0 / M_PI;

    std::cout << "Initial angle from horizontal (Z-axis): " << angle_deg << " degrees" << std::endl;

    if (angle_deg < 5.0) {
      std::cout << "Plane is already approximately horizontal. No transform applied." << std::endl;
    } else {
      // 4. Compute rotation axis
      Eigen::Vector3f axis = normal.cross(z_axis);

      if (axis.norm() < 1e-6) {
        std::cout << "Normal already aligned with Z-axis (upward or downward)." << std::endl;
      } else {
        axis.normalize();

        // 5. Build and apply rotation
        Eigen::AngleAxisf rotation(angle_rad, axis);
        Eigen::Affine3f transform = Eigen::Affine3f::Identity();
        transform.rotate(rotation);

        pcl::transformPointCloud(*output_cloud, *output_cloud, transform);
        std::cout << "Plane rotated to horizontal alignment." << std::endl;

        // 6. Recompute normal after transform (transform the original normal)
        Eigen::Vector3f rotated_normal = rotation * normal;
        float angle_after_rad = std::acos(rotated_normal.dot(z_axis));
        float angle_after_deg = angle_after_rad * 180.0 / M_PI;

        std::cout << "Angle from horizontal after rotation: " << angle_after_deg << " degrees" << std::endl;
      }
    }
  }
  
}

// void align_plane(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, 
//                  pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud, 
//                  pcl::ModelCoefficients::Ptr coefficients_plane)
// {
  
// }

void estimate_plane_bbox(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud)
{
  pcl::PointXYZ min_pt, max_pt;
  pcl::getMinMax3D(*input_cloud, min_pt, max_pt);
  double length = max_pt.x - min_pt.x;
  double width  = max_pt.y - min_pt.y;
  double height = max_pt.z - min_pt.z;

  std::cout << "Bounding Box Dimensions:" << std::endl;
  std::cout << "Length (X): " << length << " meters" << std::endl;
  std::cout << "Width  (Y): " << width  << " meters" << std::endl;
  std::cout << "Height (Z): " << height << " meters" << std::endl;

  std::cout << "max_pt (X): " << max_pt.x << " meters" << std::endl;
  std::cout << "min_pt (x): " << min_pt.x << " meters" << std::endl;
  std::cout << "max_pt (Y): " << max_pt.y << " meters" << std::endl;
  std::cout << "min_pt (Y): " << min_pt.y << " meters" << std::endl;
  std::cout << "max_pt (X): " << max_pt.z << " meters" << std::endl;
  std::cout << "min_pt (Z): " << min_pt.z << " meters" << std::endl;
}

void printDetectedPlane(const DetectedPlane& plane)
{
    std::cout << "Plane Name: " << plane.name << std::endl;
    std::cout << "Min Point: (" << plane.min_pt.x << ", " << plane.min_pt.y << ", " << plane.min_pt.z << ")" << std::endl;
    std::cout << "Max Point: (" << plane.max_pt.x << ", " << plane.max_pt.y << ", " << plane.max_pt.z << ")" << std::endl;
    std::cout << "Center: (" << plane.getCenter().x << ", " << plane.getCenter().y << ", " << plane.getCenter().z << ")" << std::endl;
    std::cout << "Dimensions: (" << plane.getDimensions().x << ", " << plane.getDimensions().y << ", " << plane.getDimensions().z << ")" << std::endl;
    std::cout << "----------------------------------------" << std::endl;
}

void process_test_pcl_data(const std::string& input_pcd, const std::string& output_pcd, std::vector<DetectedPlane> planes_info)
{
  //define the input pointcloud as a pointer
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

  //read .pcd file  
  pcl::PCDReader cloud_reader;
  cloud_reader.read(input_pcd, *cloud);

  //define the output pointcloud as a pointer
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

  /*
  voxel filter (downsampling)
  */
  voxel_filter(cloud, cloud_filtered, 0.0001);


  /*
  some step to align the point cloud to some normal orientation
  */

  /*
  segment plane
  */
  // plane_segmentation(cloud_filtered, cloud_filtered, true, true);

  all_plane_seg(cloud_filtered, cloud_filtered, planes_info, true, false, 3);

  /*
  passtrough filter for noise filtering
  */
  std::map<std::string, std::pair<float, float>> limits = {
    // {"y", {-0.4f, 0.5f}},
    {"z", {0.5f, 1.05f}}
  };
  // passthrough_filter(cloud_filtered, cloud_filtered, limits);

  /*
  estimate plane bounding box
  */
  estimate_plane_bbox(cloud_filtered);

  // save the filtered .pcd file
  pcl::PCDWriter cloud_writer;
  cloud_writer.write<pcl::PointXYZ>(output_pcd, *cloud_filtered, false);
}

void process_pcl_data(const std::string& input_pcd, const std::string& output_pcd, std::vector<DetectedPlane> planes_info)
{
  
  //define the input pointcloud as a pointer
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

  //read .pcd file  
  pcl::PCDReader cloud_reader;
  cloud_reader.read(input_pcd, *cloud);

  //define the output pointcloud as a pointer
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

  /*
  voxel filter (downsampling)
  */
  voxel_filter(cloud, cloud_filtered, 0.005);


  /*
  some step to align the point cloud to some normal orientation
  */

  /*
  passtrough filter for noise filtering
  */
  std::map<std::string, std::pair<float, float>> limits = {
    {"y", {-0.4f, 0.5f}},
    {"z", {0.5f, 2.0f}}
  };

  passthrough_filter(cloud_filtered, cloud_filtered, limits);

  /*
  statistical outlier filter
  */
  // statistical_outlier_removal(cloud_filtered, cloud_filtered);

  /*
  *Stretch* the planes to remove ripples
  */
    // Moving Least Squares (MLS) filter (method 1)
  // pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> mls;
  // mls.setInputCloud(cloud_filtered);
  // mls.setSearchRadius(0.04); // adjust based on your data scale
  // mls.setPolynomialOrder(1); // quadratic surface fit
  // mls.setUpsamplingMethod(pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ>::NONE);

  // pcl::PointCloud<pcl::PointXYZ>::Ptr smoothed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  // mls.process(*smoothed_cloud);
  // // mls.process(*cloud_filtered);

  // *cloud_filtered = *smoothed_cloud;

  // moving_least_squares(cloud_filtered, cloud_filtered);

  // plane_segmentation(cloud_filtered, cloud_filtered);
  all_plane_seg(cloud_filtered, cloud_filtered, planes_info, true, false, 8, 100, false);

  /*
  filter out non-horizontal planes
  */

  // if (inliers_plane->indices.size() > 0)
  // {
  //   float a = coefficients_plane->values[0];
  //   float b = coefficients_plane->values[1];
  //   float c = coefficients_plane->values[2];

  //   Eigen::Vector3f normal(a, b, c);
  //   normal.normalize();

  //   // Since Y-axis is vertical, horizontal planes will have normals close to Y
  //   float dot_with_y = std::abs(normal.dot(Eigen::Vector3f::UnitY()));

  //   if (dot_with_y > 0.9) // Threshold: adjust for your tolerance
  //   {
  //     // Keep the horizontal plane
  //     pcl::ExtractIndices<pcl::PointXYZ> extract;
  //     extract.setInputCloud(cloud_filtered);
  //     extract.setIndices(inliers_plane);
  //     extract.setNegative(false); // Keep only the plane
  //     extract.filter(*cloud_filtered);
  //   }
  //   else
  //   {
  //       std::cout << "Plane is not horizontal. Skipping.\n";
  //       cloud_filtered->clear(); // Clear if plane is not horizontal
  //   }
  // }

  /*
  passtrough filter
  */

  // pcl::PassThrough<pcl::PointXYZ> pass;
  // pass.setInputCloud(cloud_filtered);

  // pass.setFilterFieldName("z"); // filter the z axis values
  // pass.setFilterLimits(0.5, 1.18); // PLAY -> change the z value and check what is filtered off
  // pass.filter(*cloud_filtered);

  /*
  Example for estimating bounding box size 
  */
  // estimate_plane_bbox(cloud_filtered);


  
  // save the filtered .pcd file
  pcl::PCDWriter cloud_writer;
  cloud_writer.write<pcl::PointXYZ>(output_pcd, *cloud_filtered, false);
  // cloud_writer.write<pcl::PointXYZ>(output_pcd, *smoothed_cloud, false);
}

void publish_collision_plane(
  rclcpp::Node::SharedPtr node, 
  double width = 0.599983, 
  double length = 1.38467, 
  double height = 0.01)
  // , 
  // pcl::PointXYZ min_pt, 
  // pcl::PointXYZ max_pt) 
{

  width = 0.599983;
  length = 1.38467;
  height = 0.01;  // thin height to represent a plane

  pcl::PointXYZ min_pt, max_pt;

  min_pt.x = -0.692448;
  max_pt.x = 0.69222;

  min_pt.y = -0.90901;
  max_pt.y = -0.309027;

  double x = (min_pt.x + max_pt.x) / 2.0;
  double y = (min_pt.y + max_pt.y) / 2.0;
  double z = 0.80;

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  moveit_msgs::msg::CollisionObject collision_object;
  collision_object.header.frame_id = "map";
  collision_object.id = "segmented_plane";

  // Define the box primitive
  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = length;
  primitive.dimensions[1] = width;
  primitive.dimensions[2] = height;

  // Define the pose of the box
  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z + height / 2.0;  // so the base is aligned with the plane
  pose.orientation.w = 1.0;  // no rotation since the plane is already aligned horizontally

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(pose);
  collision_object.operation = collision_object.ADD;

  std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
  collision_objects.push_back(collision_object);

  RCLCPP_INFO(node->get_logger(), "Add an object into the world");
  planning_scene_interface.addCollisionObjects(collision_objects);

}

void publish_plane_center(
  rclcpp::Node::SharedPtr node, 
  DetectedPlane plane)
{
  static tf2_ros::StaticTransformBroadcaster static_broadcaster(node);
  geometry_msgs::msg::TransformStamped static_transform;

  const std::string parent_frame = "map";
  const std::string child_frame = plane.name;

  static_transform.header.stamp = node->get_clock()->now();
  static_transform.header.frame_id = parent_frame;
  static_transform.child_frame_id = child_frame;

  // check the height
  double center_x = plane.getCenter().x; // (min_pt.x + max_pt.x) / 2.0;
  double center_y = plane.getCenter().y; // (min_pt.y + max_pt.y) / 2.0;
  double center_z = plane.getCenter().z; // (min_pt.z + max_pt.z) / 2.0;

  // a transformation from camera to map needs to be done here

  static_transform.transform.translation.x = center_x;
  static_transform.transform.translation.y = center_y;
  static_transform.transform.translation.z = center_z;

  static_transform.transform.rotation.x = 0.0;
  static_transform.transform.rotation.y = 0.0;
  static_transform.transform.rotation.z = 0.0;
  static_transform.transform.rotation.w = 1.0;

  static_broadcaster.sendTransform(static_transform);

  RCLCPP_INFO(node->get_logger(), "Published static transform from %s to %s at (%.3f, %.3f, %.3f)",
              parent_frame.c_str(), child_frame.c_str(), center_x, center_y, center_z);
}

void publish_center_link(
  rclcpp::Node::SharedPtr node
  // , 
  // pcl::PointXYZ min_pt, 
  // pcl::PointXYZ max_pt)
  )
{
  pcl::PointXYZ min_pt, max_pt;

  min_pt.x = -0.692448;
  max_pt.x = 0.69222;

  min_pt.y = -0.90901;
  max_pt.y = -0.309027;

  double x = (min_pt.x + max_pt.x) / 2.0;
  double y = (min_pt.y + max_pt.y) / 2.0;
  double z = 0.80;

  static tf2_ros::StaticTransformBroadcaster static_broadcaster(node);

  geometry_msgs::msg::TransformStamped static_transform;

  const std::string parent_frame = "map";
  const std::string child_frame = "collision_center";

  static_transform.header.stamp = node->get_clock()->now();
  static_transform.header.frame_id = parent_frame;
  static_transform.child_frame_id = child_frame;

  static_transform.transform.translation.x = x;
  static_transform.transform.translation.y = y;
  static_transform.transform.translation.z = z;

  static_transform.transform.rotation.x = 0.0;
  static_transform.transform.rotation.y = 0.0;
  static_transform.transform.rotation.z = 0.0;
  static_transform.transform.rotation.w = 1.0;

  static_broadcaster.sendTransform(static_transform);

  RCLCPP_INFO(node->get_logger(), "Published static transform from %s to %s at (%.3f, %.3f, %.3f)",
              parent_frame.c_str(), child_frame.c_str(), x, y, z);
}

int main(int argc, char ** argv)
{
  // load [test] point cloud 
  std::string package_share_dir = ament_index_cpp::get_package_share_directory("pcl_detection");

  // std::string path_input="/home/oskars/workspace/pcl_ws/src/pcl_detection/src/inputs/";
  std::string path_output="/home/oskars/workspace/pcl_ws/src/pcl_detection/data/outputs/";
  std::string path_input = package_share_dir + "/data/inputs/";

  std::string input_test_pcd = path_input + std::string("test.pcd");
  std::string input_pcd = path_input + std::string("ar4_realsense.pcd");
  std::string output_test_pcd = path_output + std::string("filtered_test.pcd");
  std::string output_pcd = path_output + std::string("filtered_ar4_realsense.pcd");

  rclcpp::init(argc, argv);

  rclcpp::executors::MultiThreadedExecutor executor;

  PCLDetection detection;

  // async node execution
  executor.add_node(detection.node_);

  std::thread executor_thread([&executor]() { executor.spin(); });
  executor_thread.detach();

  // process point cloud
  process_pcl_data(input_pcd, output_pcd, detection.planes_info);
  // process_test_pcl_data(input_test_pcd, output_test_pcd);

  // Print out details of all segmented planes
  // std::cout << "Total planes detected: " << detection.planes_info.size() << std::endl;
  // for (const auto& plane : detection.planes_info) {
  //     printDetectedPlane(plane);  // Print the details of each plane
  // }




  

  // /*
  // - Process PCL to find existing planes and save their dimensions and center
  // - Publish saved planes' centers to tf_tree 
  // - Monitor robots proximity to plane dimensions (?probably not necessary?)
  // - When mob-man has moved in close proximity to the edge of the plane and is stopped, 
  //   publish a collision object with the tf_tree link as its center  
  // - If the mob-man starts moving, remove the collision object
  // */


  // // RCLCPP_INFO(node->get_logger(), "Run publish_collision_plane");
  // // publish_collision_plane(node);

  // RCLCPP_INFO(detection.node_->get_logger(), "Run publish_center_link");
  // publish_center_link(detection.node_);

  // while(rclcpp::ok())
  // {

  // }

  rclcpp::shutdown();
  return 0;
}

// } // namespace pcl_detection