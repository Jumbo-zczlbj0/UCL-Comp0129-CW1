/* Software License Agreement (MIT License)
 *
 *  Copyright (c) 2019-, Luke Beddow
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once

// ros includes
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Scalar.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <stack> 
#include "std_msgs/Float32.h"
#include <std_msgs/ColorRGBA.h>

// standard c++ library includes (std::string, std::vector)
#include <string>
#include <vector>
#include <numeric> //std::iota
#include <list>
#include <cmath>

#include <cw3_world_spawner/Task1Service.h>
#include <cw3_world_spawner/Task2Service.h>
#include <cw3_world_spawner/Task3Service.h>

///////////////////////////////////////////////////////////////////////////////

using namespace std;
// PCL specific includesdeclared
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/impl/transforms.hpp>

//#include <pcl_ros/point_cloud.h>
#include <pcl/common/centroid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/io/pcd_io.h>
#include <pcl/segmentation/extract_clusters.h>
// Pcl Visualization
//#include <pcl/visualization/cloud_viewer.h>
// BilateralFilter FastBilateralFilter
#include <pcl/kdtree/kdtree_flann.h>
//#include <pcl/filters/bilateral.h> 
#include <pcl/filters/fast_bilateral.h>  

// marker
#include <visualization_msgs/Marker.h>
// ProjectInliers
#include <pcl/filters/project_inliers.h>
// RadiusOutlierRemoval
#include <pcl/filters/radius_outlier_removal.h>
// PCA
#include <Eigen/Core>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/segmentation/region_growing_rgb.h>
//moveit includes
#include <moveit/planning_scene_interface/planning_scene_interface.h>

// TF specific includes
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <stdlib.h>
#include <iostream>

// ROS includes
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointC;
typedef PointC::Ptr PointCPtr;

///////////////////////////////////////////////////////////////////////////////

/** \brief Class advertising MoveIt! ROS services
  *
  * This class contains callback functions for services which use MoveIt to
  * move the robot. Here in this file we declare all of the functions and
  * variables that make up the class. The code for each function is in the
  * moveit_solution.cpp file.
  *
  * \author Zhuo Zeng
  * \author Bowen Jiang
  * \author Yuxuan Dong
  */
class SrvClass
{

public: // set all following functions/variables to public access

  /** \brief  Class constructor. 
    *
    * \input[in] nh ROS node handle
    */
  SrvClass(ros::NodeHandle& nh);

  /** \brief Service callback function for task one. 
    * Given a stack of cubes in diffferent colour, detecting its colour in 
    * order, its position coordinate as well as its oriantation.
    *
    * \input[in] request service request message 
    * \output[out] response service response message
    *  
    * \return true if service succeeds
    */
  bool 
  taskOne(cw3_world_spawner::Task1Service::Request &request,cw3_world_spawner::Task1Service::Response &response);

  /** \brief Service callback function for task two. Given the order of
    * colour cubes, as well as a position and oriantation. Reconstructing
    * a stack as reauirement.
    *
    * \input[in] request service request message 
    * \output[out] response service response message
    *  
    * \return true if service succeeds
    */
  bool 
  taskTwo(cw3_world_spawner::Task2Service::Request &request,cw3_world_spawner::Task2Service::Response &response);

  /** \brief Service callback function for task three. Given the target
    * position of the stack.
    *
    * \input[in] request service request message 
    * \output[out] response service response message
    *  
    * \return true if service succeeds
    */
  bool
  taskThree(cw3_world_spawner::Task3Service::Request &request,cw3_world_spawner::Task3Service::Response &response);

  /** \brief Apply Voxel Grid filtering.
    * 
    * \input[in] cloud_input_msg the input PointCloud2 pointer
    */
  void
  cloudCallBackOne(const sensor_msgs::PointCloud2ConstPtr& cloud_input_msg);
  
  /** \brief Pass though filter to remove the ground as well as convert data type.
    * 
    * \input[in] in_cloud_ptr the input PointCloud2 pointer
    * \input[out] out_cloud_ptr the output PointXYZRGB data
    * \input[in] thrs_min the lower bound of the filter
    * \input[in] thrs_max the upper bound of the filter
    */
  void
  applyPTNew (PointCPtr &in_cloud_ptr,pcl::PointCloud<pcl::PointXYZRGB> &out_cloud_ptr,float thrs_min, float thrs_max);
  
  /** \pass through filter to remove the point cloud of gazebo itself.
    * 
    * \input[in] in_cloud_ptr the input PointCloud2 pointer
    * \input[out] out_cloud_ptr the output PointXYZRGB data
    * \input[in] times to determine the filtering axis
    */
  void
  passThroughFilters (PointCPtr &in_cloud_ptr,PointCPtr &out_cloud_ptr,int times);

  /** \brief Pass though filter to remove the ground.
    * 
    * \input[in] in_cloud_ptr the input PointCloud2 pointer
    * \input[out] out_cloud_ptr the output PointCloud2 pointer
    * \input[in] thrs_min the lower bound of the filter
    * \input[in] thrs_max the upper bound of the filter
    */
  void
  applyPT (PointCPtr &in_cloud_ptr,PointCPtr &out_cloud_ptr,float thrs_min, float thrs_max);
  
  /** \brief Cluster function for task 3 to observe the orientation and colour 
    * order of the only non-black stack. Storing the centeriod of cubes.
    * 
    * \input[in] in_cloud_ptr the input PointCloud2 pointer
    */
  void
  task3Cluster(PointCPtr &in_cloud_ptr);

  /** \brief Find out all the cluster in point cloud
    * 
    * \input[in] in_cloud_ptr the input PointCloud2 pointer
    */
  void
  euclideanClusterExtraction (PointCPtr &in_cloud_ptr);

  /** \brief Find the centroid of point clouds.
    * 
    * \input[in] in_cloud_ptr the input PointCloud2 pointer
    */
  void
  findCylPose (PointCPtr &in_cloud_ptr);  

  /** \The color filter to filter out the stacks in black
    * 
    * \input[in] in_cloud_ptr the input PointCloud2 pointer
    */

  /** \brief Apply Pass Through filtering.
    * 
    * \input[in] in_cloud_ptr the input PointCloud2 pointer
    * \input[out] out_cloud_ptr the output PointCloud2 pointer
    */
  void
  filter_point (PointCPtr &in_cloud_ptr, PointCPtr &out_cloud_ptr);

    /** \brief detecting the cube colour in task3.
    * 
    * \input[in] cube_number the number of cube
    */
  void
  getColor3(PointCPtr &in_cloud_ptr);

  /** \brief detecting the cube colour.
    * 
    * \input[in] cube_number the number of cube
    */
  void
  getColor(PointCPtr &in_cloud_ptr,int cube_number);

  /** \brief filter out the black stacks.
    * 
    * \input[in] cube_number the number of cube
    */
  void
  blackFilter (PointCPtr &in_cloud_ptr, PointCPtr &out_cloud_ptr);

  /** \brief calculating the orientation of given point cloud.
    * 
    * \input[in] in_cloud_ptr the input PointCloud2 pointer
    */
  void
  find_orientation_Proportion(PointCPtr &in_cloud_ptr); 

  /** \brief move arms to starting point before turn on the camera for task1 and 2.
    */  
  bool
  move_to_side();   

  /** \brief move arms to starting point before turn on the camera for task3.
    */
  bool
  moveToCorner();

  /** \brief scan the map smoothly.
    */
  bool
  scan_straight(); 

  /** \brief scan the environment around the robot for task 3.
    */
  bool
  scan_round(); 

  /** \brief Add  collision for the ground in RViz and the MoveIt planning scene
    *
    * \input[in] ground centoid point
    */
  void
  addCollisionCallback(geometry_msgs::Point box_and_ground_centre);
  
  /** \brief addCollisionObject function for adding the collision around
    * the ground and basket, avoiding collision occur while pick and place
    * objects.
    *
    * \input[in] object name
    * \input[in] object centroid point
    * \input[in] object size
    * \input[in] object orientation
    */
  void
  addCollisionObject(std::string object_name,
  geometry_msgs::Point centre, geometry_msgs::Vector3 dimensions,
  geometry_msgs::Quaternion orientation);

  /** \brief MoveIt function for moving the move_group to the target position.
    *
    * \input[in] target_pose pose to move the arm to
    *
    * \return true if moved to target position 
    */
  bool 
  moveArm(geometry_msgs::Pose target_pose);

  /** \brief MoveIt function for moving the gripper fingers to a new position. 
    *
    * \input[in] width desired gripper finger width
    *
    * \return true if gripper fingers are moved to the new position
    */
  bool 
  moveGripper(float width);

  /** \brief MoveIt function for moving the gripper fingers to a new position. 
    *
    * \input[in] object_name in order to remove the collision
    */
  void 
  removeCollisionObject(std::string object_name);

  /** \brief Pick an object up with an object position and place the object to the goal position.
    * 
    * \input[in] position the xyz coordinates where the gripper converges
    * \input[in] position the xyz coordinates where the gripper separates
    * \input[in] cube_number the index of the given cube
    * \input[in] pick_orientation the oriantation when picking the cube
    * \input[in] place_orientation the oriantation when placing the cube
    * 
    * \return true if able to pick and place
    */
  bool
  pickAndPlace(geometry_msgs::Point position_object,
  geometry_msgs::Point position_goal,int cube_number,double pick_orientation, float place_orientation);

//////////////////////////////////////////////////////////////////////////////////

  ros::Publisher vis_pub;
  geometry_msgs::PointStamped centroid_marker; 

  double a;
  double b;
  double c;

  /* Variables */

  // counter to stop merging point cloud
  int g_point_cloud_switch = 0;

  // global cube orientation
  float g_cube_orientation=0;   

  /** \brief Parameters for adding collision */
  // set the object name
  std::string ground_name = "ground";
  std::string box_name = "box";

  /** \brief Define some useful constant values */
  std::string base_frame_ = "panda_link0";
  double gripper_open_ = 80e-3;
  double gripper_closed_ = 0.0;

  /** \brief Parameters to define the pick operation */
  double z_offset_ = 0.125;
  double angle_offset_ = 3.14159 / 4.0;
  double approach_distance_ = 0.25;

  /** \brief Parameters for scanning operation */
  double scan0_pose_x = 0.48;
  double scan0_pose_y = 0.33;
  double scan0_pose_z = 0.7;

  double scan_pose_x = 0.35;
  double scan_pose_y = 0.33;
  double scan_pose_z = 0.73;
 
  double scan1_pose_x = 0.35;
  double scan1_pose_y = 0.0;
  double scan1_pose_z = 0.73;

  double scan2_pose_x = 0.48;
  double scan2_pose_y = 0.0;
  double scan2_pose_z = 0.65;

  double scan3_pose_x = 0.48;
  double scan3_pose_y = -0.3;
  double scan3_pose_z = 0.7;

  double scan4_pose_x = 0.35;
  double scan4_pose_y = -0.3;
  double scan4_pose_z = 0.73;

///////////////////////////////////////////////

  double round0_pose_x = -0.4;
  double round0_pose_y = -0.375;
  double round0_pose_z =  0.8;

  double round_pose_x =  0.4;
  double round_pose_y =  -0.375;
  double round_pose_z =  0.8;

  double round1_pose_x = 0.4;
  double round1_pose_y = 0.375;
  double round1_pose_z = 0.6;

  double round2_pose_x = -0.4;
  double round2_pose_y = 0.375;
  double round2_pose_z = 0.7;

  double round3_pose_x = -0.4;
  double round3_pose_y = 0;
  double round3_pose_z =  0.7;

  double round4_pose_x =  0;
  double round4_pose_y = -0.4;
  double round4_pose_z =  0.6;

  double round5_pose_x = -0.4;
  double round5_pose_y =  0.2;
  double round5_pose_z =  0.6;


  /** \brief RGB Parameters for colour filtering */
  float g_request_r;
  float g_request_b;
  float g_request_g;

  // threshold of color filtering
  float colour_offset = 10;

  // counter to count the number of point cloud message
  int count_cloud = 1;

  // cuboid center 
  std::vector<geometry_msgs::PointStamped> cube_centroid;

  /** \brief centroids and orientation of cudes in different colors for task 2*/
  vector<geometry_msgs::Point> purple_centroids;
  vector<geometry_msgs::Point> red_centroids;
  vector<geometry_msgs::Point> blue_centroids;
  vector<geometry_msgs::Point> yellow_centroids;
  vector<geometry_msgs::Point> orange_centroids;
  vector<geometry_msgs::Point> pink_centroids;

  // Empty vector for storing the orientations of cubes in each color
  vector<float> purple_orientation;
  vector<float> red_orientation;
  vector<float> blue_orientation;
  vector<float> yellow_orientation;
  vector<float> orange_orientation;
  vector<float> pink_orientation;

  // cube stack color name vector
  vector<string> g_stackColorName;

  // cube stack color value vector (readings from sensor)
  std::vector<std_msgs::ColorRGBA> g_colors_stack;

  // vector for storing the standard rgb values rather than the readings from the sensor
  std::vector<std_msgs::ColorRGBA> g_colors_standard;

  //////////////////////////////////////////////////////////////////////////////

  /** \brief Node handle. */
  ros::NodeHandle nh_;

  /** \brief  service servers for advertising ROS services  */
  ros::ServiceServer Task1Service_srv_;
  ros::ServiceServer Task2Service_srv_;
  ros::ServiceServer Task3Service_srv_;

  /** \brief MoveIt interface to move groups to seperate the arm and the gripper,
    * these are defined in urdf. */
  moveit::planning_interface::MoveGroupInterface arm_group_{"panda_arm"};
  moveit::planning_interface::MoveGroupInterface hand_group_{"hand"};

  /** \brief MoveIt interface to interact with the moveit planning scene 
    * (eg collision objects). */
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;

  public:
    /** \brief Node handle. */
    ros::NodeHandle g_nh;
    
    /** \brief The input point cloud frame id. */
    std::string g_input_pc_frame_id_;
    std::string g_frame_id_;

    /** \brief ROS publishers. */
    ros::Publisher g_pub_cloud, g_pub_pose, g_pub_task1, g_pub_cluster,g_pub_cloud_color,g_pub_cloud_stack;
    
    /** \brief ROS geometry message point. */
    geometry_msgs::PointStamped g_cyl_pt_msg;
    
    /** \brief Voxel Grid filter's leaf size. */
    double g_vg_leaf_sz;
    
    /** \brief Point Cloud (input) pointer. */
    PointCPtr g_cloud_ptr;
    
    /** \brief Point Cloud (filtered) pointer. */
    PointCPtr g_cloud_filtered, g_cloud_filtered_ground, g_filter_point, g_add_cloud;
    PointCPtr g_cloudfind_orientation_Proportion, g_cloud_colorful_stack,g_cloud_black_stack;

    /** \brief Point Cloud (input). */
    pcl::PCLPointCloud2 g_pcl_pc;
    
    /** \brief Voxel Grid filter. */
    pcl::VoxelGrid<PointT> g_vx;
    
    /** \brief Pass Through filter. */
    pcl::PassThrough<PointT> g_pt;
    
    /** \brief Pass Through min and max threshold sizes. */
    double g_pt_thrs_min, g_pt_thrs_max;
    
    /** \brief Nearest neighborhooh size for normal estimation. */
    double g_k_nn;
    
    /** \brief cw1Q1: TF listener definition. */
    tf::TransformListener g_listener_, g_listener2_;
    
  protected:
    /** \brief Debug mode. */
    bool debug_;

};
