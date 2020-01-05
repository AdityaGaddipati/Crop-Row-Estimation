#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
// PCL specific includes
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
ros::Publisher pub;

void process_cloud (const sensor_msgs::PointCloud2ConstPtr& input)
{
  // ... do data processing

//pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_input (new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>), cloud_p (new pcl::PointCloud<pcl::PointXYZRGB>), cloud_f (new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr passthroughX (new pcl::PointCloud<pcl::PointXYZRGB>), passthroughY (new pcl::PointCloud<pcl::PointXYZRGB>), passthroughZ (new pcl::PointCloud<pcl::PointXYZRGB>);
sensor_msgs::PointCloud2::Ptr final_cloud (new sensor_msgs::PointCloud2);


  // Convert to the templated PointCloud
  //pcl::fromROSMsg (*input, *pcl_input);

  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(*input,pcl_pc2);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_input(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromPCLPointCloud2(pcl_pc2,*pcl_input);
  
  //*cloud_filtered = *pcl_input;

  // Create the filtering object: downsample the dataset using a leaf size of 1cm
  pcl::VoxelGrid<pcl::PointXYZRGB> sor;

  sor.setInputCloud (pcl_input); // input is the pcl2 received from /depthcam.
  sor.setLeafSize (0.01f, 0.01f, 0.01f);
  sor.filter (*cloud_filtered);
  
  std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height << " data points." << std::endl;
  
  
  // Create the filtering object
  pcl::PassThrough<pcl::PointXYZRGB> passX;
  passX.setInputCloud (cloud_filtered);
  passX.setFilterFieldName ("x");
  passX.setFilterLimits (-2.0, 2.0);
  //pass.setFilterLimitsNegative (true);
  passX.filter (*passthroughX);
  
  pcl::PassThrough<pcl::PointXYZRGB> passY;
  passY.setInputCloud (passthroughX);
  passY.setFilterFieldName ("y");
  passY.setFilterLimits (-0.5, 1);
  //pass.setFilterLimitsNegative (true);
  passY.filter (*passthroughY);
  
  pcl::PassThrough<pcl::PointXYZRGB> passZ;
  passZ.setInputCloud (passthroughY);
  passZ.setFilterFieldName ("z");
  passZ.setFilterLimits (0.0, 3.0);
  *input_pcl = *cloud_filtered;
  //pass.setFilterLimitsNegative (true);
  passZ.filter (*passthroughZ);
  
  *cloud_filtered = *passthroughZ;
  // Write the downsampled version to disk
  /*pcl::PCDWriter writer;
  writer.write<pcl::PointXYZRGB> ("downsampled.pcd", *cloud_filtered, false);
  */

  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());

  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZRGB> seg;

  // Optional
  seg.setOptimizeCoefficients (true);

  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (1000);
  seg.setDistanceThreshold (0.1);

  // Create the filtering object
  pcl::ExtractIndices<pcl::PointXYZRGB> extract;

  int i = 0, nr_points = (int) cloud_filtered->points.size ();
  // While 30% of the original cloud is still there
  while (cloud_filtered->points.size () > 0.3 * nr_points)
  {
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (cloud_filtered);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
      break;
    }

    // Extract the inliers
    extract.setInputCloud (cloud_filtered);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*cloud_p);
    std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;
    
    // Create the filtering object
    extract.setNegative (true);
    extract.filter (*cloud_f);
    cloud_filtered.swap (cloud_f);
    i++;
  }


  // convert to sensormsg type
  //pcl::toROSMsg (*cloud_p, *final_cloud);
  pcl::toROSMsg (*cloud_filtered, *final_cloud);

  //sensor_msgs::PointCloud2 output;
  // Publish the data
  pub.publish (final_cloud); // publish the new pcl
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "plane_segmentation");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  //ros::Subscriber sub = nh.subscribe ("point_cloud/cloud_registered", 1, process_cloud);
  ros::Subscriber sub = nh.subscribe ("depth_registered/points", 1, process_cloud);
  //ros::Subscriber sub = nh.subscribe ("passthroughY/output", 1, process_cloud);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("new_pcl", 1); // new pcl having only planar points

  // Spin
  ros::spin ();
}
