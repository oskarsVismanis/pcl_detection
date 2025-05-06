#include <pcl_detection/pcl_detection.h>

typedef boost::shared_ptr<tf2_ros::TransformListener> tfListenerPtr;
typedef boost::shared_ptr<tf2_ros::Buffer> tfBufferPtr;

using namespace pcl_detection;

// const rclcpp::Logger LOGGER = rclcpp::get_logger("plane_detection_node");

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

  rclcpp::NodeOptions node_options;
  rclcpp::Node::SharedPtr node_;

  node_options.use_intra_process_comms(false);
  node_ = std::make_shared<rclcpp::Node>("plane_detection_node", node_options);

  // async node execution
  executor.add_node(node_);

  std::thread executor_thread([&executor]() { executor.spin(); });
  executor_thread.detach();

  tfBufferPtr tf_buffer_ptr;
  tfListenerPtr tf_listener_ptr;

  tf_buffer_ptr = tfBufferPtr(new tf2_ros::Buffer(node_->get_clock()));
  tf_listener_ptr = tfListenerPtr(new tf2_ros::TransformListener(*tf_buffer_ptr));

  PCLDetection detection(node_, tf_buffer_ptr);



  // process point cloud
  // process_pcl_data(input_pcd, output_pcd);
  detection.process_test_pcl_data(input_test_pcd, output_test_pcd);

  // std::cout << "Printing detected planes: ";
  // detection.printDetectedPlane(detection.planes_info[0]);
  // Print out details of all segmented planes

  std::cout << "Total planes detected: " << detection.planes_info.size() << std::endl;
  for (const auto& plane : detection.planes_info) {
      detection.printDetectedPlane(plane);  // Print the details of each plane
  }

  std::cout << "Publishing plane centers as links" << std::endl;
  for (const auto& plane : detection.planes_info) {
    detection.publish_center_link(plane);  // Print the details of each plane
  }

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

  while(rclcpp::ok())
  {
    detection.checkProximityToPlanes(0.2);
  }

  rclcpp::shutdown();
  return 0;
}

// } // namespace pcl_detection