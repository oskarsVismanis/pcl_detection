#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <filesystem>
#include <math.h>
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

void process_pcl_data(const std::string& input_pcd, const std::string& output_pcd)
{
    // define the input pointcloud as a pointer
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // read .pcd file
    pcl::PCDReader cloud_reader;
    cloud_reader.read(input_pcd, *cloud);

    // define the output pointcloud as a pointer
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

    // voxel filter (downsampling)
    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
    voxel_filter.setInputCloud(cloud);
    voxel_filter.setLeafSize(0.0001, 0.0001, 0.0001); // play with (the higher, the larger the distance between the points)
    voxel_filter.filter(*cloud_filtered);

    // plane segmentation
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>());

    seg.setOptimizeCoefficients(true);
    seg.setMethodType(pcl::SACMODEL_PLANE); // search for planes
    seg.setModelType(pcl::SAC_RANSAC); // use RANSAC for plane segmentation
    seg.setMaxIterations(100);
    seg.setDistanceThreshold(0.01);
    seg.setInputCloud(cloud_filtered);
    seg.segment(*inliers_plane, *coefficients_plane);

        // extract plane from image
    pcl::ExtractIndices<pcl::PointXYZ> neg_plane_extracted;
    neg_plane_extracted.setInputCloud(cloud_filtered);
    neg_plane_extracted.setIndices(inliers_plane);
    neg_plane_extracted.setNegative(true); // removes the plane from the original point cloud
    // neg_plane_extracted.setNegative(false); // leaves only the plane and removes everything else
    neg_plane_extracted.filter(*cloud_filtered);

    // passtrough filter
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud_filtered);
    pass.setFilterFieldName("z"); // filter the z axis values
    pass.setFilterLimits(0.5, 1.05); // PLAY -> change the z value and check what is filtered off
    pass.filter(*cloud_filtered);

    // pass.setFilterFieldName("x"); // filter the z axis values
    // pass.setFilterLimits(0.0, 0.2); // PLAY -> change the x value and check what is filtered off
    // pass.filter(*cloud_filtered);

    // pass.setFilterFieldName("y"); // filter the z axis values
    // pass.setFilterLimits(-0.2, 0.0); // PLAY -> change the y value and check what is filtered off
    // pass.filter(*cloud_filtered);

    // remove outliers using a statistical outlier removal filter
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud_filtered);
    sor.setMeanK(50);
    sor.setStddevMulThresh(1.0);
    sor.filter(*cloud_filtered);

    // segment a cylinder using RANSAC
        // get the normals
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals(new pcl::PointCloud<pcl::PointNormal>);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::PointNormal> normals_estimator;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());

    normals_estimator.setSearchMethod(tree);
    normals_estimator.setInputCloud(cloud_filtered);
    normals_estimator.setKSearch(50); // play
    normals_estimator.compute(*cloud_normals);

        // segmentation of a cylinder from the normals
    pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::PointNormal> cylinder_segmentator;
    
    cylinder_segmentator.setOptimizeCoefficients(true);
    cylinder_segmentator.setModelType(pcl::SACMODEL_CYLINDER);
    cylinder_segmentator.setMethodType(pcl::SAC_RANSAC);
    cylinder_segmentator.setNormalDistanceWeight(0.1);
    cylinder_segmentator.setMaxIterations(10000);
    cylinder_segmentator.setDistanceThreshold(0.005); // play
    cylinder_segmentator.setRadiusLimits(0.001, 0.02); //play
    cylinder_segmentator.setInputCloud(cloud_filtered);
    cylinder_segmentator.setInputNormals(cloud_normals);

        // get the inliers and coefficients
    pcl::PointIndices::Ptr inliers_cylinder(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients_cylinder(new pcl::ModelCoefficients);
    cylinder_segmentator.segment(*inliers_cylinder, *coefficients_cylinder);

        // extract the cylinder
    pcl::ExtractIndices<pcl::PointXYZ> cylinder_extracted;
    cylinder_extracted.setInputCloud(cloud_filtered);
    cylinder_extracted.setIndices(inliers_cylinder);
    cylinder_extracted.setNegative(false); // leaves only the extracted cylinder

    cylinder_extracted.filter(*cloud_filtered);

    // Compute the centroid of extracted cylinder points
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cloud_filtered, centroid);

        // print centroid coordinates
    std::cout << "centroid of the cylinder in camera frame: ("
              << centroid[0] << ", "
              << centroid[1] << ", "
              << centroid[2] << ")" << std::endl;

    // save the filtered .pcd file
    pcl::PCDWriter cloud_writer;
    cloud_writer.write<pcl::PointXYZ>(output_pcd, *cloud_filtered, false);
}

int main(int argc, char ** argv)
{
    std::string path_input="/home/oskars/workspace/pcl_ws/src/pcl_detection/src/inputs/";
    std::string path_output="/home/oskars/workspace/pcl_ws/src/pcl_detection/src/outputs/";
    std::string input_pcd = path_input + std::string("test.pcd");
    std::string output_pcd = path_output + std::string("filtered_points.pcd");

    process_pcl_data(input_pcd, output_pcd);
    return 0;
}