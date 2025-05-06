#include <pcl_detection/pcl_detection.h>

using namespace std::chrono_literals;

const rclcpp::Logger LOGGER = rclcpp::get_logger("plane_detection_node");

namespace pcl_detection
{
	PCLDetection::PCLDetection(const rclcpp::Node::SharedPtr& node, tfBufferPtr& tf_buffer_ptr) : node_(node), tf_buffer_ptr_(tf_buffer_ptr)
	{
		  // detection.odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
  //   "/odom", 10, std::bind(&PCLDetection::odomCallback, node_, std::placeholders::_1));
		odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>("/mobile_base_controller/odom", 10, std::bind(&PCLDetection::odomCallback, this, std::placeholders::_1));
	}

	PCLDetection::~PCLDetection()
	{}

	void PCLDetection::voxel_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud, float leafsize)
{
		pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
		voxel_filter.setInputCloud(input_cloud);
		voxel_filter.setLeafSize(leafsize, leafsize, leafsize); // play with (the higher, the larger the distance between the points)
		voxel_filter.filter(*output_cloud);
	}

	void PCLDetection::passthrough_filter(
		pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, 
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

	void PCLDetection::statistical_outlier_removal(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud)
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

	void PCLDetection::moving_least_squares(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud)
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

	void PCLDetection::all_plane_seg(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud,
		pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud,
		// std::vector<DetectedPlane>& planes_info,
		bool show_plane,
		bool aligned_plane,
		int max_planes,
		int min_inliers,
		bool horizontal_only)
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
				// DetectedPlane plane;
				// plane.name = "plane_" + std::to_string(i);
				// pcl::getMinMax3D(*cloud_plane, plane.min_pt, plane.max_pt);
				// planes_info.push_back(plane);
				addDetectedPlane(cloud_plane);

				// printDetectedPlane(planes_info[i]);
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

	void PCLDetection::addDetectedPlane(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_plane)
	{
		DetectedPlane new_plane;
		pcl::getMinMax3D(*cloud_plane, new_plane.min_pt, new_plane.max_pt);
		pcl::PointXYZ c1 = new_plane.getCenter();

		float overlap_threshold = 0.01f; // Adjust as needed

		std::cout << "Adding plane" << std::endl;

		for (const auto& existing_plane : planes_info)
    {
        pcl::PointXYZ c2 = existing_plane.getCenter();

        if (std::abs(c1.x - c2.x) < overlap_threshold &&
            std::abs(c1.y - c2.y) < overlap_threshold &&
            std::abs(c1.z - c2.z) < overlap_threshold)
        {
            // Overlapping plane — do not add
            return;
        }
    }

    // Assign name using the current size of planes_info
    new_plane.name = "plane_" + std::to_string(planes_info.size());
    planes_info.push_back(new_plane);
	}

	void PCLDetection::estimate_plane_bbox(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud)
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

	void PCLDetection::printDetectedPlane(const DetectedPlane& plane)
	{
			std::cout << "Plane Name: " << plane.name << std::endl;
			std::cout << "Min Point: (" << plane.min_pt.x << ", " << plane.min_pt.y << ", " << plane.min_pt.z << ")" << std::endl;
			std::cout << "Max Point: (" << plane.max_pt.x << ", " << plane.max_pt.y << ", " << plane.max_pt.z << ")" << std::endl;
			std::cout << "Center: (" << plane.getCenter().x << ", " << plane.getCenter().y << ", " << plane.getCenter().z << ")" << std::endl;
			std::cout << "Dimensions: (" << plane.getDimensions().x << ", " << plane.getDimensions().y << ", " << plane.getDimensions().z << ")" << std::endl;
			std::cout << "----------------------------------------" << std::endl;
	}

	void PCLDetection::process_test_pcl_data(const std::string& input_pcd, const std::string& output_pcd)
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

		all_plane_seg(cloud_filtered, cloud_filtered, true, false, 3, 100, false);

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
		// estimate_plane_bbox(cloud_filtered);

		// save the filtered .pcd file
		pcl::PCDWriter cloud_writer;
		cloud_writer.write<pcl::PointXYZ>(output_pcd, *cloud_filtered, false);
	}

	void PCLDetection::process_pcl_data(const std::string& input_pcd, const std::string& output_pcd)
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
		all_plane_seg(cloud_filtered, cloud_filtered, true, false, 8, 100, false);

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

	geometry_msgs::msg::Point PCLDetection::transformPointToFrame(
    const geometry_msgs::msg::Point& point_in,
    const std::string& from_frame,
    const std::string& to_frame,
    rclcpp::Time stamp)
	{
		// static tf2_ros::Buffer tf_buffer(node_->get_clock());
    // static tf2_ros::TransformListener tf_listener(tf_buffer);
		// tf_buffer_ptr_(node_->get_clock());

		geometry_msgs::msg::TransformStamped transform;

    geometry_msgs::msg::PointStamped input_stamped, output_stamped;
    input_stamped.header.stamp = stamp;
    input_stamped.header.frame_id = from_frame;
    input_stamped.point = point_in;

    try {
        // output_stamped = tf_buffer_ptr_.transform(input_stamped, to_frame, tf2::durationFromSec(0.5));
				transform = tf_buffer_ptr_->lookupTransform(to_frame, from_frame, tf2::TimePointZero);
				tf2::doTransform(input_stamped, output_stamped, transform);
        return output_stamped.point;
    } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN(LOGGER, "Transform failed: %s", ex.what());
        return geometry_msgs::msg::Point(); // Return zero point if transform fails
    }
	}

	void PCLDetection::publish_center_link(const DetectedPlane& plane)
	{
		pcl::PointXYZ center = plane.getCenter();

		// pcl::PointXYZ min_pt, max_pt;
	
		// min_pt.x = -0.692448;
		// max_pt.x = 0.69222;
	
		// min_pt.y = -0.90901;
		// max_pt.y = -0.309027;
	
		// // double x = (min_pt.x + max_pt.x) / 2.0;
		// double x = center.x;
		// // double y = (min_pt.y + max_pt.y) / 2.0;
		// double y = center.y;
		// // double z = 0.80;
		// double z = center.z - 0.5;

		geometry_msgs::msg::Point point_in;
    point_in.x = center.x;
    point_in.y = center.y;
    point_in.z = center.z;

		geometry_msgs::msg::Point point_in_map = transformPointToFrame(point_in, "base_link", "map", node_->get_clock()->now());

		static tf2_ros::StaticTransformBroadcaster static_broadcaster(node_);
	
		geometry_msgs::msg::TransformStamped static_transform;
    static_transform.header.stamp = node_->get_clock()->now();
    static_transform.header.frame_id = "map";
    static_transform.child_frame_id = plane.name;
		
    static_transform.transform.translation.x = point_in_map.x;
    static_transform.transform.translation.y = point_in_map.y;
    static_transform.transform.translation.z = point_in_map.z - 0.8;
	
		static_transform.transform.rotation.x = 0.0;
		static_transform.transform.rotation.y = 0.0;
		static_transform.transform.rotation.z = 0.0;
		static_transform.transform.rotation.w = 1.0;
	
		static_broadcaster.sendTransform(static_transform);
	
    RCLCPP_INFO(LOGGER, "Published static transform from map to %s at (%.3f, %.3f, %.3f)",
                plane.name.c_str(),
                static_transform.transform.translation.x,
                static_transform.transform.translation.y,
                static_transform.transform.translation.z);
	}

// void PCLDetection::checkProximityToPlanes(double threshold)
// {
//     for (const auto& plane : planes_info)
//     {
//         geometry_msgs::msg::TransformStamped tf_robot_to_plane;
//         try {
//             tf_robot_to_plane = tf_buffer_ptr_->lookupTransform(
//                 plane.name, "base_link", tf2::TimePointZero);

//             double x = tf_robot_to_plane.transform.translation.x;
//             double y = tf_robot_to_plane.transform.translation.y;

//             // Calculate distance from robot to the nearest bounding box edge
//             double x_dist = std::min(std::abs(x - plane.min_pt.x), std::abs(plane.max_pt.x - x));
//             double y_dist = std::min(std::abs(y - plane.min_pt.y), std::abs(plane.max_pt.y - y));

//             if (x >= plane.min_pt.x && x <= plane.max_pt.x &&
//                 y >= plane.min_pt.y && y <= plane.max_pt.y)
//             {
//                 // Inside the bounding box, check proximity to edges
//                 if (x_dist < threshold || y_dist < threshold) {
//                     RCLCPP_WARN(LOGGER, "Robot is too close to plane %s (x_dist=%.2f, y_dist=%.2f)",
//                                 plane.name.c_str(), x_dist, y_dist);
//                     // TODO: don't cancel if the robot is moving in the opposite direction!
// 										cancelMoveGoal();
//                 }
//             }

//         } catch (const tf2::TransformException& ex) {
//             RCLCPP_WARN(LOGGER, "TF lookup failed for %s: %s", plane.name.c_str(), ex.what());
//         }
//     }
// }

void PCLDetection::checkProximityToPlanes(double threshold)
{
	for (const auto& plane : planes_info)
	{
		geometry_msgs::msg::TransformStamped tf_robot_to_plane;
		try {
			tf_robot_to_plane = tf_buffer_ptr_->lookupTransform(
				plane.name, "base_link", tf2::TimePointZero);

			double x = tf_robot_to_plane.transform.translation.x;
			double y = tf_robot_to_plane.transform.translation.y;

			double x_dist = std::min(std::abs(x - plane.min_pt.x), std::abs(plane.max_pt.x - x));
			double y_dist = std::min(std::abs(y - plane.min_pt.y), std::abs(plane.max_pt.y - y));

			bool inside = (x >= plane.min_pt.x && x <= plane.max_pt.x &&
											y >= plane.min_pt.y && y <= plane.max_pt.y);

			if (inside && (x_dist < threshold || y_dist < threshold)) {
				// RCLCPP_WARN(LOGGER, "Robot is too close to plane %s (x_dist=%.2f, y_dist=%.2f)",
        //                         plane.name.c_str(), x_dist, y_dist);
				if (isMovingTowardPlane(tf_robot_to_plane, plane)) {
					RCLCPP_WARN(LOGGER, "Too close and heading toward %s", plane.name.c_str());
					cancelMoveGoal();
				}
			}

		} catch (const tf2::TransformException& ex) {
			RCLCPP_WARN(LOGGER, "TF lookup failed for %s: %s", plane.name.c_str(), ex.what());
		}
	}
}

bool PCLDetection::isMovingTowardPlane(
	const geometry_msgs::msg::TransformStamped& tf_robot_to_plane,
	const DetectedPlane& plane)
{
	double vx = current_velocity_.linear.x;
	double vy = current_velocity_.linear.y;

	tf2::Quaternion q(
		tf_robot_to_plane.transform.rotation.x,
		tf_robot_to_plane.transform.rotation.y,
		tf_robot_to_plane.transform.rotation.z,
		tf_robot_to_plane.transform.rotation.w);
	tf2::Matrix3x3 m(q);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);

	double vx_in_plane = vx * cos(yaw) - vy * sin(yaw);
	double vy_in_plane = vx * sin(yaw) + vy * cos(yaw);

	double robot_x = tf_robot_to_plane.transform.translation.x;
	double robot_y = tf_robot_to_plane.transform.translation.y;

	double plane_center_x = (plane.min_pt.x + plane.max_pt.x) / 2.0;
	double plane_center_y = (plane.min_pt.y + plane.max_pt.y) / 2.0;

	double dx = plane_center_x - robot_x;
	double dy = plane_center_y - robot_y;

	double mag = std::sqrt(dx * dx + dy * dy);
	if (mag < 1e-3) return false;

	dx /= mag;
	dy /= mag;

	double velocity_mag = std::sqrt(vx_in_plane * vx_in_plane + vy_in_plane * vy_in_plane);
	if (velocity_mag < 0.01) return false;

	double dot = vx_in_plane * dx + vy_in_plane * dy;

	RCLCPP_INFO(LOGGER, "Velocity in plane (%.2f, %.2f), direction (%.2f, %.2f), dot = %.2f",
							vx_in_plane, vy_in_plane, dx, dy, dot);

	return dot > 0.0;
}

void PCLDetection::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    current_velocity_ = msg->twist.twist;
		// RCLCPP_INFO(LOGGER, "Current velocity: vx = %.2f, vy = %.2f", current_velocity_.linear.x, current_velocity_.linear.y);
}

void PCLDetection::cancelMoveGoal()
{
	nav2_msgs::action::NavigateToPose::Goal goal;

	auto action_client = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(node_, "navigate_to_pose");

	if (action_client->wait_for_action_server()) {
		RCLCPP_INFO(LOGGER, "Canceling move goal!");
    action_client->async_cancel_all_goals();
	}

}


} // namespace pcl_detection