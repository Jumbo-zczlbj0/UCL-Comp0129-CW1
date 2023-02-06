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

#include <cw3_team_8/moveit_solution.h>
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointC;
typedef PointC::Ptr PointCPtr;

///////////////////////////////////////////////////////////////////////////////

SrvClass::SrvClass(ros::NodeHandle& nh):
  g_cloud_ptr (new PointC), // input point cloud
  g_cloud_filtered (new PointC), // filtered point cloud
  g_cloud_filtered_ground (new PointC),
  g_filter_point(new PointC),
  g_add_cloud(new PointC),
  g_cloud_colorful_stack(new PointC),
  g_cloud_black_stack(new PointC),
  g_cloudfind_orientation_Proportion(new PointC)
{
  /* Constructor function, this is run only when an object of the class is 
  first created. The aim of this function is to initialise the class */

  nh_ = nh;

  // namespace for our ROS services, they will appear as "/namespace/srv_name"
  std::string service_ns = "/task";

  // advertise the services available from this node for task 1,2 and 3
  Task1Service_srv_ = nh_.advertiseService(service_ns + "1_start",
    &SrvClass::taskOne, this);
  Task2Service_srv_ = nh_.advertiseService(service_ns + "2_start",
    &SrvClass::taskTwo, this);
  Task3Service_srv_ = nh_.advertiseService(service_ns + "3_start",
    &SrvClass::taskThree, this);


  // Define the publishers
  g_pub_cloud = nh.advertise<sensor_msgs::PointCloud2> ("filtered_cloud", 1, true);
  g_pub_cloud_color = nh.advertise<sensor_msgs::PointCloud2> ("color_cloud", 1, true);
  g_pub_cloud_stack = nh.advertise<sensor_msgs::PointCloud2> ("example_stack_cloud", 1, true);
  g_pub_pose = nh.advertise<geometry_msgs::PointStamped> ("cyld_pt", 1, true);
  g_pub_task1 = nh.advertise<sensor_msgs::PointCloud2> ("filtered_cloud_task1", 1, true);
  g_pub_cluster = nh_.advertise<sensor_msgs::PointCloud2> ("single_cube", 1, true);
  ros::Publisher vis_pub = nh.advertise<visualization_msgs::Marker> ( "visualization_marker", 0 );

  // Define public variables
  g_vg_leaf_sz = 0.01; // VoxelGrid leaf size: Better in a config file
  g_pt_thrs_min = 0.0; // PassThrough min thres: Better in a config file
  g_pt_thrs_max = 0.7; // PassThrough max thres: Better in a config file
  g_k_nn = 50; // Normals nn size: Better in a config file

}

///////////////////////////////////////////////////////////////////////////////

bool
SrvClass::taskOne(cw3_world_spawner::Task1Service::Request &request,cw3_world_spawner::Task1Service::Response &response)
{
  /* This task given a stack of cubes in diffferent colour, detecting its 
  colour in order, its position coordinate as well as its orientation. */

// variables initilization
int cube_num;
vector<string> stackColorName;
std::vector<std_msgs::ColorRGBA> colors_stack;
vector<geometry_msgs::Point> task1_centroids;

// move to the starting point before turn on the camera
move_to_side();

// turn on the camera on the robot arm
ros::Subscriber sub_cloud =
nh_.subscribe ("/r200/camera/depth_registered/points",
                1,
                &SrvClass::cloudCallBackOne,
                this);

// scan aroung the map
scan_straight();

// find the centroid coordinate of the stack
findCylPose(g_add_cloud);
cube_centroid[0].point.z -= 0.008;    

// print out the coordinate of the centoid
ROS_INFO("/stack_centroid_x:%f",cube_centroid[0].point.x);
ROS_INFO("/stack_centroid_y:%f",cube_centroid[0].point.y);
ROS_INFO("/stack_centroid_z:%f",cube_centroid[0].point.z);

// use the heght to find out the number of cubes for this stack
if (cube_centroid[0].point.z < 0.08)
{
  ROS_INFO("/Number of cube found: 3");
  cube_num = 3;
  }
  else if (cube_centroid[0].point.z > 0.1)
  {
  ROS_INFO("/Number of cube found: 5");
  cube_num = 5;
  }else
  {
  ROS_INFO("/Number of cube found: 4");
  cube_num = 4;
}

// detecting colour of each cube
ROS_INFO("Start get color");
getColor(g_add_cloud,cube_num);

// find the orientation of the stack
find_orientation_Proportion(g_add_cloud);

// outputing the imformation to Response
response.stack_colours = g_colors_standard;
response.stack_point = cube_centroid[0].point;
response.stack_rotation = g_cube_orientation;

}

///////////////////////////////////////////////////////////////////////////////

bool
SrvClass::taskTwo(cw3_world_spawner::Task2Service::Request &request,cw3_world_spawner::Task2Service::Response &response)
{

  // initialization of variables
  std::vector<std_msgs::ColorRGBA> my_vector = request.stack_colours;
  vector<geometry_msgs::Point> pick_centroids_list;
  vector<float> pick_orientation_list;
  float place_orientation = request.stack_rotation;
  geometry_msgs::Point place_position = request.stack_point;

  // move to starting point before turn on the camera
  move_to_side();

  // turn on the camera to record point cloud data
  ros::Subscriber sub_cloud =
  nh_.subscribe ("/r200/camera/depth_registered/points",
                  1,
                  &SrvClass::cloudCallBackOne,
                  this);

  // printing the required orientation at placing position
  ROS_INFO("/The required orientation is %f",place_orientation);

  // scan the front map
  scan_straight();

  // clusting the point cloud to seperate the cubes
  euclideanClusterExtraction(g_add_cloud);

  // initialize the string of colour
  string purple("purple");
  string red("red");
  string blue("blue");
  string yellow("yellow");
  string orange("orange");
  string pink("pink");

  // switch point cloud
  g_point_cloud_switch = 1;

  // initializa the count number of cubes for each colour
  int aa = 1;
  int pur = 0;
  int redd = 0;
  int bluee = 0;
  int yel = 0;
  int ora = 0;
  int pinkk = 0;

  // store the centroid and the corresponding orientation of cubes in order into lists
  for(auto item: my_vector)
  { 
    ROS_INFO("/The %dth required color",aa);
    ROS_INFO("R%f G%f B%f",item.r,item.g,item.b);
    if (item.r <= 0.85 && item.g <= 0.15 && item.b <= 0.85 && item.r >= 0.75 && item.g >= 0.05 && item.b >= 0.75)
      {
       std::cout << purple << "\n"; 
       pick_centroids_list.push_back(purple_centroids[pur]);
       pick_orientation_list.push_back(purple_orientation[pur]);
       pur++;
     }else if (item.r <= 0.85 && item.g <= 0.15 && item.b <= 0.15 && item.r >= 0.65 && item.g >= 0.0 && item.b >= 0.0)
      {
       std::cout << red << "\n";
       pick_centroids_list.push_back(red_centroids[redd]);
       pick_orientation_list.push_back(red_orientation[redd]);
       redd++;
     }else if (item.r <= 0.15 && item.g <= 0.15 && item.b <= 0.85 && item.r >= 0.05 && item.g >= 0.05 && item.b >= 0.75)
      {
       std::cout << blue << "\n";
       pick_centroids_list.push_back(blue_centroids[bluee]);
       pick_orientation_list.push_back(blue_orientation[bluee]);
       bluee++;
     }else if (item.r <= 1.05 && item.g <= 1.05 && item.b <= 0.05 && item.r >= 0.95 && item.g >= 0.95 && item.b >= 0.0)
      {
       std::cout << yellow << "\n";
       pick_centroids_list.push_back(yellow_centroids[yel]);
       pick_orientation_list.push_back(yellow_orientation[yel]);
       yel++;
     }else if (item.r <= 0.95 && item.g <= 0.45 && item.b <= 0.15 && item.r >= 0.85 && item.g >= 0.35 && item.b >= 0.05)
      {
       std::cout << orange << "\n";
       pick_centroids_list.push_back(orange_centroids[ora]);
       pick_orientation_list.push_back(orange_orientation[ora]);
       ora++;
     }else if (item.r <= 0.95 && item.g <= 0.75 && item.b <= 0.75 && item.r >= 0.85 && item.g >= 0.65 && item.b >= 0.65)
      {
       std::cout << pink << "\n";
       pick_centroids_list.push_back(pink_centroids[pinkk]);
       pick_orientation_list.push_back(pink_orientation[pinkk]);
       pinkk++;
      }
    aa++;
  }

  // add collision for ground
  geometry_msgs::Point ground_centre;
  ground_centre.x = 0.0;
  ground_centre.y = 0.0;
  ground_centre.z = 0.0;
  addCollisionCallback(ground_centre);

  // add addCollision for cubes that are going to pick and place
  for( int pick = 0; pick < aa; pick++ )
  {
    // determine the size of the cube collision
    geometry_msgs::Vector3 cube_size;
    cube_size.x = 0.04;
    cube_size.y = 0.04;
    cube_size.z = 0.04;

    // determine the orientation of the cube collision and for grasping
    tf2::Quaternion q_x180deg(-1, 0, 0, 0);
    tf2::Quaternion q_object;
    q_object.setRPY(0, 0, pick_orientation_list[pick]+angle_offset_);
    int neg_offset = -1;
    tf2::Quaternion q_result = q_x180deg * q_object * neg_offset;
    geometry_msgs::Quaternion horizontal_orientation = tf2::toMsg(q_result);

    // converting int to string for cube index
    std::string s = std::to_string(pick);
    addCollisionObject("cube"+s,pick_centroids_list[pick],cube_size,
      horizontal_orientation);
  }

  // pick and place cubes in order
  for( int pick = 0; pick < aa; pick++ )
  {
    pickAndPlace(pick_centroids_list[pick],place_position,pick,pick_orientation_list[pick],place_orientation);
  }

}

///////////////////////////////////////////////////////////////////////////////////////////

bool
SrvClass::taskThree(cw3_world_spawner::Task3Service::Request &request,cw3_world_spawner::Task3Service::Response &response)
{ 
  // switch point cloud
  g_point_cloud_switch = 1;
  // initialization
  vector<geometry_msgs::Point> pick_centroids_list;
  vector<float> pick_orientation_list;
  geometry_msgs::Point place_position = request.stack_point;

  // turn on the camera to record point cloud data
  ros::Subscriber sub_cloud =
  nh_.subscribe ("/r200/camera/depth_registered/points",
                1,
                &SrvClass::cloudCallBackOne,
                this);

  // move to starting point before turn on the camera
  moveToCorner();

  // switch point cloud
  g_point_cloud_switch = 0;

  // scan half of map
  scan_round();
  
  // move to starting point before turn on the camera
  moveToCorner();

  // scan half of map
  scan_round();

  // filtering out the unwanted point clouds
  passThroughFilters(g_add_cloud,g_filter_point,1);
  passThroughFilters(g_add_cloud,g_cloud_filtered_ground,2);

  // Clear point cloud 
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr g_add_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  g_add_cloud->clear();

  // full point cloud
  *g_add_cloud = *g_cloud_filtered_ground + *g_filter_point;

  // Clear point cloud 
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr g_filter_point (new pcl::PointCloud<pcl::PointXYZRGB>);
  g_filter_point->clear();

  applyPT(g_add_cloud,g_filter_point,0.01,3);
  // Clear point cloud 
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr g_cloud_filtered_ground (new pcl::PointCloud<pcl::PointXYZRGB>);
  g_cloud_filtered_ground->clear();

  // black stack Filter
  blackFilter(g_filter_point,g_cloud_filtered_ground);

  // publish the point cloud after filtering
  sensor_msgs::PointCloud2 color_msg;
  pcl::toROSMsg(*g_cloud_filtered_ground, color_msg);
  g_pub_cloud_color.publish(color_msg);

  // clustering the stack and cubes
  task3Cluster(g_cloud_filtered_ground);
  //print out the orientation while placeing the cubes
  float place_orientation;
  place_orientation = g_cube_orientation;
  ROS_INFO("orientation of stack is:%f",g_cube_orientation);

  // string initialization
  string purple("purple");
  string red("red");
  string blue("blue");
  string yellow("yellow");
  string orange("orange");
  string pink("pink");

  // switch point cloud
  g_point_cloud_switch = 1;

  // initialize the count number of cubes for each colour
  int aa = 1;
  int pur = 0;
  int redd = 0;
  int bluee = 0;
  int yel = 0;
  int ora = 0;
  int pinkk = 0;

  // store the centroid and the corresponding orientation of cubes in order into lists
  for(auto item: g_colors_standard)
  { 
    ROS_INFO("/The %dth required color",aa);
    ROS_INFO("R%f G%f B%f",item.r,item.g,item.b);
    if (item.r <= 0.85 && item.g <= 0.15 && item.b <= 0.85 && item.r >= 0.75 && item.g >= 0.05 && item.b >= 0.75)
      {
       std::cout << purple << "\n"; 
       pick_centroids_list.push_back(purple_centroids[pur]);
       pick_orientation_list.push_back(purple_orientation[pur]); 
       pur++;
     }else if (item.r <= 0.85 && item.g <= 0.15 && item.b <= 0.15 && item.r >= 0.65 && item.g >= 0.0 && item.b >= 0.0)
      {
       std::cout << red << "\n";
       pick_centroids_list.push_back(red_centroids[redd]);
       pick_orientation_list.push_back(red_orientation[redd]);
       redd++;
     }else if (item.r <= 0.15 && item.g <= 0.15 && item.b <= 0.85 && item.r >= 0.05 && item.g >= 0.05 && item.b >= 0.75)
      {
       std::cout << blue << "\n";
       pick_centroids_list.push_back(blue_centroids[bluee]);
       pick_orientation_list.push_back(blue_orientation[bluee]);
       bluee++;
     }else if (item.r <= 1.05 && item.g <= 1.05 && item.b <= 0.05 && item.r >= 0.95 && item.g >= 0.95 && item.b >= 0.0)
      {
       std::cout << yellow << "\n";
       pick_centroids_list.push_back(yellow_centroids[yel]);
       pick_orientation_list.push_back(yellow_orientation[yel]);
       yel++;
     }else if (item.r <= 0.95 && item.g <= 0.45 && item.b <= 0.15 && item.r >= 0.85 && item.g >= 0.35 && item.b >= 0.05)
      {
       std::cout << orange << "\n";
       pick_centroids_list.push_back(orange_centroids[ora]);
       pick_orientation_list.push_back(orange_orientation[ora]);
       ora++;
     }else if (item.r <= 0.95 && item.g <= 0.75 && item.b <= 0.75 && item.r >= 0.85 && item.g >= 0.65 && item.b >= 0.65)
      {
       std::cout << pink << "\n";
       pick_centroids_list.push_back(pink_centroids[pinkk]);
       pick_orientation_list.push_back(pink_orientation[pinkk]);
       pinkk++;
      }
    aa++;
  }

  // add collision of the ground
  geometry_msgs::Point ground_centre;
  ground_centre.x = 0.0;
  ground_centre.y = 0.0;
  ground_centre.z = 0.0;
  addCollisionCallback(ground_centre);

  // add addCollision for cubes
  for( int pick = 0; pick < aa; pick++ )
  {
    // determine the size of the cube collision
    geometry_msgs::Vector3 cube_size;
    cube_size.x = 0.04;
    cube_size.y = 0.04;
    cube_size.z = 0.04;

    // determine the orientation of the cube collision and for grasping
    tf2::Quaternion q_x180deg(-1, 0, 0, 0);
    tf2::Quaternion q_object;
    q_object.setRPY(0, 0, pick_orientation_list[pick]+angle_offset_);
    int neg_offset = -1;
    tf2::Quaternion q_result = q_x180deg * q_object * neg_offset;
    geometry_msgs::Quaternion horizontal_orientation = tf2::toMsg(q_result);

    std::string s = std::to_string(pick);
    addCollisionObject("cube"+s,pick_centroids_list[pick],cube_size,
      horizontal_orientation);
  }

  // pick and place cubes in order
  for( int pick = 0; pick < aa; pick++ )
  {
    pickAndPlace(pick_centroids_list[pick],place_position,pick,pick_orientation_list[pick],place_orientation);
  }

}

///////////////////////////////////////////////////////////////////////////////////////////

void
SrvClass::cloudCallBackOne(const sensor_msgs::PointCloud2ConstPtr &cloud_input_msg)
{
  /* This function sum up all the point cloud and publish */

  // input camera parameter
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_camera (new PointC);

  // extract inout point cloud info
  g_input_pc_frame_id_ = cloud_input_msg->header.frame_id;
  
  // convert to PCL data tfilter_pointype
  pcl_conversions::toPCL (*cloud_input_msg, g_pcl_pc);
  pcl::fromPCLPointCloud2 (g_pcl_pc, *g_cloud_ptr);

  pcl_ros::transformPointCloud(base_frame_, *g_cloud_ptr, *input_camera, g_listener2_);

  // perform the filtering
  filter_point(input_camera,g_filter_point);
  
  // filtering out the ground
  applyPT (g_filter_point,g_cloud_filtered_ground,0.04 , 0.45);

  // save all the point cloud message into a global point cloud variable
  if(g_point_cloud_switch == 0)
  {
    if(count_cloud == 1){
      *g_add_cloud = *g_cloud_filtered_ground;
      count_cloud++;
    }
    else{
      *g_add_cloud += *g_cloud_filtered_ground;
    }
  }
  
  // extract merged cloud info
  g_frame_id_ = g_add_cloud->header.frame_id;

  // publish the point cloud so that can be seen in Rviz
  sensor_msgs::PointCloud2 filtered_ros_msg;
  pcl::toROSMsg(*g_add_cloud, filtered_ros_msg);
  g_pub_cloud.publish (filtered_ros_msg);

}

////////////////////////////////////////////////////////////////////////////////

void
SrvClass::applyPTNew (PointCPtr &in_cloud_ptr,pcl::PointCloud<pcl::PointXYZRGB> &out_cloud_ptr,float thrs_min, float thrs_max)
{
  /* This function filter out point cloud for the green map as well as convert data type.*/

  // only keep the point cloud in range (thrs_min,thrs_max)
  g_pt.setInputCloud (in_cloud_ptr);
  g_pt.setFilterFieldName ("z");
  g_pt.setFilterLimits (thrs_min, thrs_max);
  g_pt.filter (out_cloud_ptr);
  return;
}

////////////////////////////////////////////////////////////////////////////////

void
SrvClass::passThroughFilters (PointCPtr &in_cloud_ptr,PointCPtr &out_cloud_ptr,int times)
{
  /* This function filter out point cloud for gazebo itself.*/

  if (times==1)
  {
    // Clear point cloud 
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr g_filter_point (new pcl::PointCloud<pcl::PointXYZRGB>);
    g_filter_point->clear();

    // only keep the point cloud in range
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud(g_add_cloud);	
    pass.setFilterFieldName("x");
    pass.setFilterLimits(-0.25, 0.25);
    pass.setFilterLimitsNegative(true); 
    pass.filter(*out_cloud_ptr);
  }
  else
  {
    // Clear point cloud 
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr g_cloud_filtered_ground (new pcl::PointCloud<pcl::PointXYZRGB>);
    g_cloud_filtered_ground->clear();

    // only keep the point cloud in range
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud(g_add_cloud);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(-0.25, 0.2);
    pass.setFilterLimitsNegative(true);
    pass.filter(*out_cloud_ptr);
  }
    return;
}

////////////////////////////////////////////////////////////////////////////////

void
SrvClass::applyPT (PointCPtr &in_cloud_ptr,PointCPtr &out_cloud_ptr,float thrs_min, float thrs_max)
{
  /* This function filter out point cloud for the green map.*/

  // only keep the point cloud in range (thrs_min,thrs_max)
  g_pt.setInputCloud (in_cloud_ptr);
  g_pt.setFilterFieldName ("z");
  g_pt.setFilterLimits (thrs_min, thrs_max);
  g_pt.filter (*out_cloud_ptr);
  return;
}

////////////////////////////////////////////////////////////////////////////////
void
SrvClass::task3Cluster(PointCPtr &in_cloud_ptr)
{
  // number of stack 
  int number_stack = 0;
  sensor_msgs::PointCloud2 cube_single_msg;
  
  // pass though filter
  pcl::PassThrough<pcl::PointXYZRGB> pass;
  pass.setInputCloud(in_cloud_ptr);
  pass.setFilterFieldName("x");
  pass.setFilterLimits(0.02, 3.0);
  pass.setFilterLimitsNegative(false);
  pass.filter(*g_cloudfind_orientation_Proportion);


  /* This function cluster the point cloud using euclidean cluster */
  cube_centroid.clear();
  // creating kdTree object
  pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
  tree->setInputCloud (in_cloud_ptr);
  std::vector<pcl::PointIndices> g_cloud_filtered;
  // euclidean cluster
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
  // Set the search radius to 1cm
  ec.setClusterTolerance (0.007); 
  ec.setMinClusterSize (230);
  // euclidean cluster input
  ec.setSearchMethod(tree);
  ec.setInputCloud (in_cloud_ptr);
  // euclidean cluster output
  ec.extract (g_cloud_filtered);
  int cluster_count = 0;
  // write clustering results to g_cloud_filtered

  for (std::vector<pcl::PointIndices>::const_iterator it = g_cloud_filtered.begin (); it != g_cloud_filtered .end (); ++it) 
  {
    cluster_count++;
    ROS_INFO("cluster No.%d",cluster_count);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
    std_msgs::ColorRGBA color1;
    geometry_msgs::Point cube_centroid;

    pcl::PointCloud<pcl::PointXYZRGB> each_cube1;
    pcl::PointXYZRGB each_centroid1;

    // loop to find the entire cluster
    for (const auto& idx : it->indices)
    {
      cloud_cluster->points.push_back ((*in_cloud_ptr)[idx]); 
    }
    
    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;
    cloud_cluster->header.frame_id = g_frame_id_;

    // applying filter
    applyPTNew (cloud_cluster,each_cube1,0 , 0.4);
    pcl::computeCentroid(each_cube1, each_centroid1);
    color1.r = each_centroid1.r;
    color1.b = each_centroid1.b;
    color1.g = each_centroid1.g;
    //ROS_INFO("X%f Y%f Z%f",each_centroid1.x,each_centroid1.y,each_centroid1.z);

    // find out the colour order and orientation of the coloured stack 
    if ((each_centroid1.z > 0.095)&&(number_stack==0))
    {
      // number of stack 
      number_stack++;

      //add collision for the stack
      geometry_msgs::Vector3 stack_size;
      stack_size.x = 0.04;
      stack_size.y = 0.04;
      stack_size.z = 0.16;

      geometry_msgs::Quaternion stack_angle;
      stack_angle.x = 0.0;
      stack_angle.y = 0.0;
      stack_angle.z = 0.0;
      stack_angle.w = 1.0;  

      geometry_msgs::Point centre;
      centre.x = each_centroid1.x;
      centre.y = each_centroid1.y;
      centre.z = each_centroid1.z;

      addCollisionObject("stack",centre, stack_size,stack_angle);

      // detecting colour of each cube
      ROS_INFO("Start get color");
      getColor3(cloud_cluster);

      // find the orientation of the stack
      find_orientation_Proportion(cloud_cluster);
    }
    else
    {
      cube_centroid.x = each_centroid1.x;
      cube_centroid.y = each_centroid1.y;
      cube_centroid.z = 0.02;

      ROS_INFO("X%f Y%f Z%f",each_centroid1.x,each_centroid1.y,each_centroid1.z);
      ROS_INFO("R%f G%f B%f",color1.r,color1.g,color1.b);

      find_orientation_Proportion (cloud_cluster);

      string purple("purple");
      string red("red");
      string blue("blue");
      string yellow("yellow");
      string orange("orange");
      string pink("pink");

      // store the orientation and centroid of cubes in each colour
      if ( 120 <= color1.r && color1.r <= 200 && 120 <= color1.b && color1.b <= 200  && color1.g <= 35 )
      {
       //g_stackColorName.push_back("purple");
       std::cout << purple << "\n"; 
       purple_centroids.push_back(cube_centroid);
       purple_orientation.push_back(g_cube_orientation);
      }
      else if ( color1.r <= 180 && color1.r <= 300 &&  color1.b <= 125  && color1.g <= 125 )
      {
       //g_stackColorName.push_back("red");
       std::cout << red << "\n";
       red_centroids.push_back(cube_centroid);
       red_orientation.push_back(g_cube_orientation);
      }
      else if ( color1.r <= 125 &&  170 <= color1.b && color1.b <= 300 && color1.g <= 125 )
      {
       //g_stackColorName.push_back("blue");
       std::cout << blue << "\n";
       blue_centroids.push_back(cube_centroid);
       blue_orientation.push_back(g_cube_orientation);
      }
      else if (color1.b <= 10)
      {
       //g_stackColorName.push_back("yellow");
       std::cout << yellow << "\n";
       yellow_centroids.push_back(cube_centroid);
       yellow_orientation.push_back(g_cube_orientation);
      }
      else if (150 <= color1.r && color1.r <= 230 && 5 <= color1.b && color1.b <= 45 && 25 <= color1.g && color1.g <= 120)
      {
       //g_stackColorName.push_back("orange");
       std::cout << orange << "\n";
       orange_centroids.push_back(cube_centroid);
       orange_orientation.push_back(g_cube_orientation);
      }
      else if (150 <= color1.r && color1.r <= 220 && 80 <= color1.b && color1.b <= 140 && color1.g <= 120)
      {
       //g_stackColorName.push_back("pink");
       std::cout << pink << "\n";
       pink_centroids.push_back(cube_centroid);
       pink_orientation.push_back(g_cube_orientation);
      }   
      pcl::toROSMsg(*cloud_cluster, cube_single_msg);
      g_pub_task1.publish(cube_single_msg);     
    }
  }
  ROS_INFO("clustering finished");
  //pcl::toROSMsg(*in_cloud_ptr, cube_single_msg);
  //g_pub_task1.publish(cube_single_msg);
  return;

}

////////////////////////////////////////////////////////////////////////////////

void
SrvClass::euclideanClusterExtraction(PointCPtr &in_cloud_ptr)
{
  pcl::PassThrough<pcl::PointXYZRGB> pass;
  pass.setInputCloud(in_cloud_ptr);
  pass.setFilterFieldName("x");
  pass.setFilterLimits(0.2, 0.4);
  pass.setFilterLimitsNegative(false);
  pass.filter(*g_cloudfind_orientation_Proportion);

  /* This function cluster the point cloud using euclidean cluster */
  cube_centroid.clear();
  // creating kdTree object
  pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
  tree->setInputCloud (in_cloud_ptr);
  std::vector<pcl::PointIndices> g_cloud_filtered;
  // euclidean cluster
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
  // Set the search radius to 1cm
  ec.setClusterTolerance (0.008); 
  ec.setMinClusterSize (60);
  // euclidean cluster input
  ec.setSearchMethod(tree);
  ec.setInputCloud (in_cloud_ptr);
  //ec.setMinClusterSize (60);
  // euclidean cluster output
  ec.extract (g_cloud_filtered);

  // write clustering results to g_cloud_filtered
  for (std::vector<pcl::PointIndices>::const_iterator it = g_cloud_filtered.begin (); it != g_cloud_filtered .end (); ++it) 
  {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
    std_msgs::ColorRGBA color1;
    geometry_msgs::Point cube_centroid;

    // loop to find the entire cluster
    for (const auto& idx : it->indices)
    {
      cloud_cluster->points.push_back ((*in_cloud_ptr)[idx]); 
    }
    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;
    cloud_cluster->header.frame_id = g_frame_id_;

    pcl::PointCloud<pcl::PointXYZRGB> each_cube1;
    pcl::PointXYZRGB each_centroid1;
    sensor_msgs::PointCloud2 cube_single_msg;
    applyPTNew (cloud_cluster,each_cube1,0 , 0.4);
    pcl::computeCentroid(each_cube1, each_centroid1);
    color1.r = each_centroid1.r;
    color1.b = each_centroid1.b;
    color1.g = each_centroid1.g;

    cube_centroid.x = each_centroid1.x;
    cube_centroid.y = each_centroid1.y;
    cube_centroid.z = 0.02;

    ROS_INFO("X%f Y%f Z%f",each_centroid1.x,each_centroid1.y,each_centroid1.z);
    ROS_INFO("R%f G%f B%f",color1.r,color1.g,color1.b);

    find_orientation_Proportion (cloud_cluster);

    string purple("purple");
    string red("red");
    string blue("blue");
    string yellow("yellow");
    string orange("orange");
    string pink("pink");
    if ( 120 <= color1.r && color1.r <= 200 && 120 <= color1.b && color1.b <= 200  && color1.g <= 35)
    {
     //g_stackColorName.push_back("purple");
     std::cout << purple << "\n"; 
     purple_centroids.push_back(cube_centroid);
     purple_orientation.push_back(g_cube_orientation);
   }else if (color1.r <= 180 && color1.r <= 300 &&  color1.b <= 125  && color1.g <= 125)
    {
     //g_stackColorName.push_back("red");
     std::cout << red << "\n";
     red_centroids.push_back(cube_centroid);
     red_orientation.push_back(g_cube_orientation);
   }else if (color1.r <= 125 &&  170 <= color1.b && color1.b <= 300 && color1.g <= 125)
    {
     //g_stackColorName.push_back("blue");
     std::cout << blue << "\n";
     blue_centroids.push_back(cube_centroid);
     blue_orientation.push_back(g_cube_orientation);
   }else if (color1.b <= 10)
    {
     //g_stackColorName.push_back("yellow");
     std::cout << yellow << "\n";
     yellow_centroids.push_back(cube_centroid);
     yellow_orientation.push_back(g_cube_orientation);
   }else if (150 <= color1.r && color1.r <= 230 && 5 <= color1.b && color1.b <= 45 && 25 <= color1.g && color1.g <= 120)
    {
     //g_stackColorName.push_back("orange");
     std::cout << orange << "\n";
     orange_centroids.push_back(cube_centroid);
     orange_orientation.push_back(g_cube_orientation);
   }else if (150 <= color1.r && color1.r <= 220 && 80 <= color1.b && color1.b <= 140 && color1.g <= 120)
    {
     //g_stackColorName.push_back("pink");
     std::cout << pink << "\n";
     pink_centroids.push_back(cube_centroid);
     pink_orientation.push_back(g_cube_orientation);
    }   
    pcl::toROSMsg(*cloud_cluster, cube_single_msg);
    g_pub_task1.publish(cube_single_msg);
  }

  return;
}

//////////////////////////////////////////////////////////////////////////////

void
SrvClass::findCylPose (PointCPtr &in_cloud_ptr)
{
  /* This function find out the centroid point for each point cloud cluster */

  // enerate the vector of the centroid point of cluster
  Eigen::Vector4f centroid_in;
  pcl::compute3DCentroid(*in_cloud_ptr, centroid_in);
  
  // sign each of the vactor value to the position variable
  g_cyl_pt_msg.header.frame_id = g_frame_id_;
  g_cyl_pt_msg.header.stamp = ros::Time (0);
  g_cyl_pt_msg.point.x = centroid_in[0];
  g_cyl_pt_msg.point.y = centroid_in[1];
  g_cyl_pt_msg.point.z = centroid_in[2];
  //ROS_INFO("X%f Y%f Z%f",centroid_in[0],centroid_in[1],centroid_in[2]);

  centroid_marker.point.x = centroid_in[0];
  centroid_marker.point.y = centroid_in[0];
  centroid_marker.point.z = centroid_in[0];

  // Transform the point to new frame
  geometry_msgs::PointStamped g_cyl_pt_msg_out;
  try
  {
    g_listener_.transformPoint ("panda_link0", 
                                g_cyl_pt_msg,
                                g_cyl_pt_msg_out);
  }
  catch (tf::TransformException& ex)
  {
    ROS_ERROR ("Received a trasnformation exception: %s", ex.what());
  }

  // push back the centroid into global list
  cube_centroid.push_back(g_cyl_pt_msg_out);

  return ;
}

/////////////////////////////////////////////////////////////////////////////
void
SrvClass::filter_point (PointCPtr &in_cloud_ptr, PointCPtr &out_cloud_ptr)
{
  /* This function is used to filter the unwanted point cloud. */

  g_vx.setInputCloud (in_cloud_ptr);
  g_vx.setLeafSize (g_vg_leaf_sz, g_vg_leaf_sz, g_vg_leaf_sz);
  g_vx.filter (*out_cloud_ptr);
  
  return;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void
SrvClass::getColor3(PointCPtr &in_cloud_ptr)
{ 
  /* This function is used to detect the colour of cubes. */
  sensor_msgs::PointCloud2 cube_single_msg;
  pcl::toROSMsg(*in_cloud_ptr, cube_single_msg);
  g_pub_task1.publish(cube_single_msg);

  /** \brief cube. */
  geometry_msgs::Vector3 cube_size;
  cube_size.x = 0.04;
  cube_size.y = 0.04;
  cube_size.z = 0.04;

  geometry_msgs::Quaternion cube_angle;
  cube_angle.x = 0.0;
  cube_angle.y = 0.0;
  cube_angle.z = 0.0;
  cube_angle.w = 1.0;  

  ////////// 1st cube //////////
  pcl::PointCloud<pcl::PointXYZRGB> each_cube1;
  std_msgs::ColorRGBA color1;
  each_cube1.clear();

  // apply pass through filter to find the centroid
  applyPTNew (in_cloud_ptr,each_cube1,0.04 , 0.055);
  pcl::PointXYZRGB each_centroid1;
  pcl::computeCentroid(each_cube1, each_centroid1);

  geometry_msgs::Point centre;
  centre.x = each_centroid1.x;
  centre.y = each_centroid1.y;
  centre.z = each_centroid1.z;

  // add collision for stack
  addCollisionObject("stack1",centre, cube_size,cube_angle);

  //ROS_INFO("X%f Y%f Z%f",each_centroid1.x,each_centroid1.y,each_centroid1.z);

  //ROS_INFO("/each_cube1 size:%d",each_cube1.points.size());
  if (each_cube1.points.size() == 0)
  {  
    ROS_INFO("/No point in this level");
  }else
  {
    color1.r = each_centroid1.r;
    color1.b = each_centroid1.b;
    color1.g = each_centroid1.g;
    g_colors_stack.push_back(color1);
    //ROS_INFO("/red1:%f",color1.r);
    //ROS_INFO("/blue1:%f",color1.b);
    //ROS_INFO("/green1:%f",color1.g);
  }
  
  ////////// 2nd cube //////////
  pcl::PointCloud<pcl::PointXYZRGB> each_cube2;
  std_msgs::ColorRGBA color2;
  each_cube2.clear();
  applyPTNew (in_cloud_ptr,each_cube2,0.06 , 0.08);

  pcl::PointXYZRGB each_centroid2;
  pcl::computeCentroid(each_cube2, each_centroid2);

  //ROS_INFO("X%f Y%f Z%f",each_centroid2.x,each_centroid2.y,each_centroid2.z);

  centre.x = each_centroid2.x;
  centre.y = each_centroid2.y;
  centre.z = each_centroid2.z;

  addCollisionObject("stack2",centre, cube_size,cube_angle);

  //ROS_INFO("/each_cube2 size:%d",each_cube2.points.size());
  if (each_cube2.points.size() == 0)
  {  
    ROS_INFO("/No point in this level");
  }else
  {
    color2.r = each_centroid2.r;
    color2.b = each_centroid2.b;
    color2.g = each_centroid2.g;
    g_colors_stack.push_back(color2);
    //ROS_INFO("/red2:%f",color2.r);
    //ROS_INFO("/blue2:%f",color2.b);
    //ROS_INFO("/green2:%f",color2.g);
  }

  ////////// 3rd cube //////////
  pcl::PointCloud<pcl::PointXYZRGB> each_cube3;
  std_msgs::ColorRGBA color3;
  each_cube3.clear();
  applyPTNew (in_cloud_ptr,each_cube3,0.1 , 0.12);

  pcl::PointXYZRGB each_centroid3;
  pcl::computeCentroid(each_cube3, each_centroid3);

  //ROS_INFO("X%f Y%f Z%f",each_centroid3.x,each_centroid3.y,each_centroid3.z);

  centre.x = each_centroid3.x;
  centre.y = each_centroid3.y;
  centre.z = each_centroid3.z;

  addCollisionObject("stack3",centre, cube_size,cube_angle);

  //ROS_INFO("/each_cube3 size:%d",each_cube3.points.size());
  if (each_cube3.points.size() == 0)
  {  
    ROS_INFO("/No point in this level");
  }else
  {
    color3.r = each_centroid3.r;
    color3.b = each_centroid3.b;
    color3.g = each_centroid3.g;
    g_colors_stack.push_back(color3);
    //ROS_INFO("/red3:%f",color3.r);
    //ROS_INFO("/blue3:%f",color3.b);
    //ROS_INFO("/green3:%f",color3.g);
  }
    
  ////////// 4th cube //////////
  pcl::PointCloud<pcl::PointXYZRGB> each_cube4;
  std_msgs::ColorRGBA color4;
  each_cube4.clear();
  applyPTNew (in_cloud_ptr,each_cube4,0.14 , 0.16);

  pcl::PointXYZRGB each_centroid4;
  pcl::computeCentroid(each_cube4, each_centroid4);

  //ROS_INFO("X%f Y%f Z%f",each_centroid4.x,each_centroid4.y,each_centroid4.z);

  centre.x = each_centroid4.x;
  centre.y = each_centroid4.y;
  centre.z = each_centroid4.z;

  addCollisionObject("stack4",centre, cube_size,cube_angle);

  //ROS_INFO("/each_cube4 size:%d",each_cube4.points.size());
  if (each_cube4.points.size() == 0)
  {  
    ROS_INFO("/No point in this level");
  }else
  {
    color4.r = each_centroid4.r;
    color4.b = each_centroid4.b;
    color4.g = each_centroid4.g;
    g_colors_stack.push_back(color4);
    //ROS_INFO("/red4:%f",color4.r);
    //ROS_INFO("/blue4:%f",color4.b);
    //ROS_INFO("/green4:%f",color4.g);
  }

  // determine the colour of each cube
  string purple("purple");
  string red("red");
  string blue("blue");
  string yellow("yellow");
  string orange("orange");
  string pink("pink");
  int count = 1;

  // determine the colour of each centroid of cube and store them into the correlated vector
  for(auto item: g_colors_stack){
    std_msgs::ColorRGBA cube_color_standard;
    ROS_INFO("/Cube No.%d is in",count);
    count++;
    if ( 95 <= item.r && item.r <= 130 && 65 <= item.b && item.b <= 125 && 5 <= item.g && item.g <= 35)
    {
     g_stackColorName.push_back("purple");
     cube_color_standard.r = 0.8;
     cube_color_standard.g = 0.1;
     cube_color_standard.b = 0.8;
     g_colors_standard.push_back(cube_color_standard);
     std::cout << purple << "\n";
   }else if (95 <= item.r && item.r <= 130 && 10 <= item.b && item.b <= 35 && 5 <= item.g && item.g <= 35)
    {
     g_stackColorName.push_back("red");
     cube_color_standard.r = 0.8;
     cube_color_standard.g = 0.1;
     cube_color_standard.b = 0.1;
     g_colors_standard.push_back(cube_color_standard);
     std::cout << red << "\n";
   }else if (5 <= item.r && item.r <= 35 && 95 <= item.b && item.b <= 130 && 5 <= item.g && item.g <= 35)
    {
     g_stackColorName.push_back("blue");
     cube_color_standard.r = 0.1;
     cube_color_standard.g = 0.1;
     cube_color_standard.b = 0.8;
     g_colors_standard.push_back(cube_color_standard);
     std::cout << blue << "\n";
   }else if (item.b <= 10)
    {
     g_stackColorName.push_back("yellow");
     cube_color_standard.r = 1.0;
     cube_color_standard.g = 1.0;
     cube_color_standard.b = 0.0;
     g_colors_standard.push_back(cube_color_standard);
     std::cout << yellow << "\n";
   }else if (100 <= item.r && item.r <= 150 && 10 <= item.b && item.b <= 35 && 25 <= item.g && item.g <= 80)
    {
     g_stackColorName.push_back("orange");
     cube_color_standard.r = 0.9;
     cube_color_standard.g = 0.4;
     cube_color_standard.b = 0.1;
     g_colors_standard.push_back(cube_color_standard);
     std::cout << orange << "\n";
   }else if (100 <= item.r && item.r <= 150 && 65 <= item.b && item.b <= 120 && 40 <= item.g && item.g <= 120)
    {
     g_stackColorName.push_back("pink");
     cube_color_standard.r = 0.9;
     cube_color_standard.g = 0.7;
     cube_color_standard.b = 0.7;
     g_colors_standard.push_back(cube_color_standard);
     std::cout << pink << "\n";
    }    
 }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
SrvClass::getColor(PointCPtr &in_cloud_ptr,int cube_number)
{ 
  /* This function is used to detect the colour of cubes. */
  sensor_msgs::PointCloud2 cube_single_msg;
  pcl::toROSMsg(*in_cloud_ptr, cube_single_msg);
  g_pub_task1.publish(cube_single_msg);
  if (cube_number == 3)
 {
    ////////// 1st cube //////////
    pcl::PointCloud<pcl::PointXYZRGB> each_cube1;
    std_msgs::ColorRGBA color1;

    // enmpty the point cloud 
    each_cube1.clear();

    // filter the cube in terms of its height and convert data type
    applyPTNew (in_cloud_ptr,each_cube1,0.04 , 0.05);

    pcl::PointXYZRGB each_centroid1;
    // find the centriod of the cube 
    pcl::computeCentroid(each_cube1, each_centroid1);

    //ROS_INFO("X%f Y%f Z%f",each_centroid1.x,each_centroid1.y,each_centroid1.z);

    if (each_cube1.points.size() == 0)
    {  
      ROS_INFO("/No point in this level");
    }else
    {
      // read colour imformation
      color1.r = each_centroid1.r;
      color1.b = each_centroid1.b;
      color1.g = each_centroid1.g;
      g_colors_stack.push_back(color1);
      //ROS_INFO("/red1:%f",color1.r);
      //ROS_INFO("/blue1:%f",color1.b);
      //ROS_INFO("/green1:%f",color1.g);
    }

    ////////// 2nd cube //////////
    pcl::PointCloud<pcl::PointXYZRGB> each_cube2;
    std_msgs::ColorRGBA color2;
    each_cube2.clear();
    applyPTNew (g_add_cloud,each_cube2,0.06 , 0.08);

    pcl::PointXYZRGB each_centroid2;
    pcl::computeCentroid(each_cube2, each_centroid2);

    //ROS_INFO("X%f Y%f Z%f",each_centroid2.x,each_centroid2.y,each_centroid2.z);

    if (each_cube2.points.size() == 0)
    {  
      ROS_INFO("/No point in this level");
    }else
    {
      color2.r = each_centroid2.r;
      color2.b = each_centroid2.b;
      color2.g = each_centroid2.g;
      g_colors_stack.push_back(color2);
      //ROS_INFO("/red2:%f",color2.r);
      //ROS_INFO("/blue2:%f",color2.b);
      //ROS_INFO("/green2:%f",color2.g);
    }

    ////////// 3rd cube //////////
    pcl::PointCloud<pcl::PointXYZRGB> each_cube3;
    std_msgs::ColorRGBA color3;
    each_cube3.clear();
    applyPTNew (g_add_cloud,each_cube3,0.1 , 0.12);

    pcl::PointXYZRGB each_centroid3;
    pcl::computeCentroid(each_cube3, each_centroid3);

    //ROS_INFO("X%f Y%f Z%f",each_centroid3.x,each_centroid3.y,each_centroid3.z);

    //ROS_INFO("/each_cube3 size:%d",each_cube3.points.size());
    if (each_cube3.points.size() == 0)
    {  
      ROS_INFO("/No point in this level");
    }else
    {
      color3.r = each_centroid3.r;
      color3.b = each_centroid3.b;
      color3.g = each_centroid3.g;
      g_colors_stack.push_back(color3);
      //ROS_INFO("/red3:%f",color3.r);
      //ROS_INFO("/blue3:%f",color3.b);
      //ROS_INFO("/green3:%f",color3.g);
    }
  }
  else if (cube_number == 5)
  { 
    ////////// 1st cube //////////
    pcl::PointCloud<pcl::PointXYZRGB> each_cube1;
    std_msgs::ColorRGBA color1;
    each_cube1.clear();
    applyPTNew (g_add_cloud,each_cube1,0.04 , 0.05);

    pcl::PointXYZRGB each_centroid1;
    pcl::computeCentroid(each_cube1, each_centroid1);

    //ROS_INFO("X%f Y%f Z%f",each_centroid1.x,each_centroid1.y,each_centroid1.z);

    //ROS_INFO("/each_cube1 size:%d",each_cube1.points.size());
    if (each_cube1.points.size() == 0)
    {  
      ROS_INFO("/No point in this level");
    }else
    {
      color1.r = each_centroid1.r;
      color1.b = each_centroid1.b;
      color1.g = each_centroid1.g;
      g_colors_stack.push_back(color1);
      //ROS_INFO("/red1:%f",color1.r);
      //ROS_INFO("/blue1:%f",color1.b);
      //ROS_INFO("/green1:%f",color1.g);
    }

    ////////// 2nd cube //////////
    pcl::PointCloud<pcl::PointXYZRGB> each_cube2;
    std_msgs::ColorRGBA color2;
    each_cube2.clear();
    applyPTNew (g_add_cloud,each_cube2,0.06 , 0.08);

    pcl::PointXYZRGB each_centroid2;
    pcl::computeCentroid(each_cube2, each_centroid2);

    //ROS_INFO("X%f Y%f Z%f",each_centroid2.x,each_centroid2.y,each_centroid2.z);

    //ROS_INFO("/each_cube2 size:%d",each_cube2.points.size());
    if (each_cube2.points.size() == 0)
    {  
      ROS_INFO("/No point in this level");
    }else
    {
      color2.r = each_centroid2.r;
      color2.b = each_centroid2.b;
      color2.g = each_centroid2.g;
      g_colors_stack.push_back(color2);
      //ROS_INFO("/red2:%f",color2.r);
      //ROS_INFO("/blue2:%f",color2.b);
      //ROS_INFO("/green2:%f",color2.g);
    }
   
    ////////// 3rd cube //////////
    pcl::PointCloud<pcl::PointXYZRGB> each_cube3;
    std_msgs::ColorRGBA color3;
    each_cube3.clear();
    applyPTNew (g_add_cloud,each_cube3,0.1 , 0.12);

    pcl::PointXYZRGB each_centroid3;
    pcl::computeCentroid(each_cube3, each_centroid3);

    //ROS_INFO("X%f Y%f Z%f",each_centroid3.x,each_centroid3.y,each_centroid3.z);

    //ROS_INFO("/each_cube3 size:%d",each_cube3.points.size());
    if (each_cube3.points.size() == 0)
    {  
      ROS_INFO("/No point in this level");
    }else
    {
      color3.r = each_centroid3.r;
      color3.b = each_centroid3.b;
      color3.g = each_centroid3.g;
      g_colors_stack.push_back(color3);
      //ROS_INFO("/red3:%f",color3.r);
      //ROS_INFO("/blue3:%f",color3.b);
      //ROS_INFO("/green3:%f",color3.g);
    }
      
    ////////// 4th cube //////////
    pcl::PointCloud<pcl::PointXYZRGB> each_cube4;
    std_msgs::ColorRGBA color4;
    each_cube4.clear();
    applyPTNew (g_add_cloud,each_cube4,0.14 , 0.16);

    pcl::PointXYZRGB each_centroid4;
    pcl::computeCentroid(each_cube4, each_centroid4);

    //ROS_INFO("X%f Y%f Z%f",each_centroid4.x,each_centroid4.y,each_centroid4.z);

    //ROS_INFO("/each_cube4 size:%d",each_cube4.points.size());
    if (each_cube4.points.size() == 0)
    {  
      ROS_INFO("/No point in this level");
    }else
    {
      color4.r = each_centroid4.r;
      color4.b = each_centroid4.b;
      color4.g = each_centroid4.g;
      g_colors_stack.push_back(color4);
      //ROS_INFO("/red4:%f",color4.r);
      //ROS_INFO("/blue4:%f",color4.b);
      //ROS_INFO("/green4:%f",color4.g);
    }
    
    ////////// 5th cube //////////
    pcl::PointCloud<pcl::PointXYZRGB> each_cube5;
    std_msgs::ColorRGBA color5;
    each_cube5.clear();
    applyPTNew (g_add_cloud,each_cube5,0.18 , 0.2);

    pcl::PointXYZRGB each_centroid5;
    pcl::computeCentroid(each_cube5, each_centroid5);

    //ROS_INFO("X%f Y%f Z%f",each_centroid5.x,each_centroid5.y,each_centroid5.z);

    //ROS_INFO("/each_cube5 size:%d",each_cube5.points.size());
    if (each_cube5.points.size() == 0)
    {  
      ROS_INFO("/No point in this level");
    }else
    {
      color5.r = each_centroid5.r;
      color5.b = each_centroid5.b;
      color5.g = each_centroid5.g;
      g_colors_stack.push_back(color5);
      //ROS_INFO("/red5:%f",color5.r);
      //ROS_INFO("/blue5:%f",color5.b);
      //ROS_INFO("/green5:%f",color5.g);
    }
  }
  else if (cube_number == 4)
  {
    ////////// 1st cube //////////
    pcl::PointCloud<pcl::PointXYZRGB> each_cube1;
    std_msgs::ColorRGBA color1;
    each_cube1.clear();
    applyPTNew (g_add_cloud,each_cube1,0.04 , 0.05);

    pcl::PointXYZRGB each_centroid1;
    pcl::computeCentroid(each_cube1, each_centroid1);

    //ROS_INFO("X%f Y%f Z%f",each_centroid1.x,each_centroid1.y,each_centroid1.z);

    //ROS_INFO("/each_cube1 size:%d",each_cube1.points.size());
    if (each_cube1.points.size() == 0)
    {  
      ROS_INFO("/No point in this level");
    }else
    {
      color1.r = each_centroid1.r;
      color1.b = each_centroid1.b;
      color1.g = each_centroid1.g;
      g_colors_stack.push_back(color1);
      //ROS_INFO("/red1:%f",color1.r);
      //ROS_INFO("/blue1:%f",color1.b);
      //ROS_INFO("/green1:%f",color1.g);
    }
    
    ////////// 2nd cube //////////
    pcl::PointCloud<pcl::PointXYZRGB> each_cube2;
    std_msgs::ColorRGBA color2;
    each_cube2.clear();
    applyPTNew (g_add_cloud,each_cube2,0.06 , 0.08);

    pcl::PointXYZRGB each_centroid2;
    pcl::computeCentroid(each_cube2, each_centroid2);

    //ROS_INFO("X%f Y%f Z%f",each_centroid2.x,each_centroid2.y,each_centroid2.z);

    //ROS_INFO("/each_cube2 size:%d",each_cube2.points.size());
    if (each_cube2.points.size() == 0)
    {  
      ROS_INFO("/No point in this level");
    }else
    {
      color2.r = each_centroid2.r;
      color2.b = each_centroid2.b;
      color2.g = each_centroid2.g;
      g_colors_stack.push_back(color2);
      //ROS_INFO("/red2:%f",color2.r);
      //ROS_INFO("/blue2:%f",color2.b);
      //ROS_INFO("/green2:%f",color2.g);
    }

    ////////// 3rd cube //////////
    pcl::PointCloud<pcl::PointXYZRGB> each_cube3;
    std_msgs::ColorRGBA color3;
    each_cube3.clear();
    applyPTNew (g_add_cloud,each_cube3,0.1 , 0.12);

    pcl::PointXYZRGB each_centroid3;
    pcl::computeCentroid(each_cube3, each_centroid3);

    //ROS_INFO("X%f Y%f Z%f",each_centroid3.x,each_centroid3.y,each_centroid3.z);

    //ROS_INFO("/each_cube3 size:%d",each_cube3.points.size());
    if (each_cube3.points.size() == 0)
    {  
      ROS_INFO("/No point in this level");
    }else
    {
      color3.r = each_centroid3.r;
      color3.b = each_centroid3.b;
      color3.g = each_centroid3.g;
      g_colors_stack.push_back(color3);
      //ROS_INFO("/red3:%f",color3.r);
      //ROS_INFO("/blue3:%f",color3.b);
      //ROS_INFO("/green3:%f",color3.g);
    }
      
    ////////// 4th cube //////////
    pcl::PointCloud<pcl::PointXYZRGB> each_cube4;
    std_msgs::ColorRGBA color4;
    each_cube4.clear();
    applyPTNew (g_add_cloud,each_cube4,0.14 , 0.16);

    pcl::PointXYZRGB each_centroid4;
    pcl::computeCentroid(each_cube4, each_centroid4);

    //ROS_INFO("X%f Y%f Z%f",each_centroid4.x,each_centroid4.y,each_centroid4.z);

    //ROS_INFO("/each_cube4 size:%d",each_cube4.points.size());
    if (each_cube4.points.size() == 0)
    {  
      ROS_INFO("/No point in this level");
    }else
    {
      color4.r = each_centroid4.r;
      color4.b = each_centroid4.b;
      color4.g = each_centroid4.g;
      g_colors_stack.push_back(color4);
      //ROS_INFO("/red4:%f",color4.r);
      //ROS_INFO("/blue4:%f",color4.b);
      //ROS_INFO("/green4:%f",color4.g);
    }
  }

  // determine the colour of each cube
  string purple("purple");
  string red("red");
  string blue("blue");
  string yellow("yellow");
  string orange("orange");
  string pink("pink");
  int count = 1;

  // determine the colour of each centroid of cube and store them into the correlated vector
  for(auto item: g_colors_stack){
    std_msgs::ColorRGBA cube_color_standard;
    ROS_INFO("/Cube No.%d is in",count);
    count++;
    if ( 95 <= item.r && item.r <= 130 && 65 <= item.b && item.b <= 125 && 5 <= item.g && item.g <= 35)
    {
     g_stackColorName.push_back("purple");
     cube_color_standard.r = 0.8;
     cube_color_standard.g = 0.1;
     cube_color_standard.b = 0.8;
     g_colors_standard.push_back(cube_color_standard);
     std::cout << purple << "\n";
   }else if (95 <= item.r && item.r <= 130 && 10 <= item.b && item.b <= 35 && 5 <= item.g && item.g <= 35)
    {
     g_stackColorName.push_back("red");
     cube_color_standard.r = 0.8;
     cube_color_standard.g = 0.1;
     cube_color_standard.b = 0.1;
     g_colors_standard.push_back(cube_color_standard);
     std::cout << red << "\n";
   }else if (5 <= item.r && item.r <= 35 && 95 <= item.b && item.b <= 130 && 5 <= item.g && item.g <= 35)
    {
     g_stackColorName.push_back("blue");
     cube_color_standard.r = 0.1;
     cube_color_standard.g = 0.1;
     cube_color_standard.b = 0.8;
     g_colors_standard.push_back(cube_color_standard);
     std::cout << blue << "\n";
   }else if (item.b <= 15)
    {
     g_stackColorName.push_back("yellow");
     cube_color_standard.r = 1.0;
     cube_color_standard.g = 1.0;
     cube_color_standard.b = 0.0;
     g_colors_standard.push_back(cube_color_standard);
     std::cout << yellow << "\n";
   }else if (100 <= item.r && item.r <= 150 && 10 <= item.b && item.b <= 35 && 25 <= item.g && item.g <= 80)
    {
     g_stackColorName.push_back("orange");
     cube_color_standard.r = 0.9;
     cube_color_standard.g = 0.4;
     cube_color_standard.b = 0.1;
     g_colors_standard.push_back(cube_color_standard);
     std::cout << orange << "\n";
   }else if (100 <= item.r && item.r <= 150 && 65 <= item.b && item.b <= 120 && 40 <= item.g && item.g <= 120)
    {
     g_stackColorName.push_back("pink");
     cube_color_standard.r = 0.9;
     cube_color_standard.g = 0.7;
     cube_color_standard.b = 0.7;
     g_colors_standard.push_back(cube_color_standard);
     std::cout << pink << "\n";
    }    
 }
}

////////////////////////////////////////////////////////////////////////////

void
SrvClass::blackFilter (PointCPtr &in_cloud_ptr, PointCPtr &out_cloud_ptr)
{
  /* This function filter out the unwanted color */

  // create a colour filter
  pcl::ConditionalRemoval<pcl::PointXYZRGB> color_filter;

  // create and set the requirements of the color filter
  pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr
      red_condition_h(new pcl::PackedRGBComparison<pcl::PointXYZRGB>("r", pcl::ComparisonOps::GT, 50 ));
  pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr
      red_condition_l(new pcl::PackedRGBComparison<pcl::PointXYZRGB>("r", pcl::ComparisonOps::LT, 220));

  pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr
      green_condition_h(new pcl::PackedRGBComparison<pcl::PointXYZRGB>("g", pcl::ComparisonOps::GT, 50 ));
  pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr
      green_condition_l(new pcl::PackedRGBComparison<pcl::PointXYZRGB>("g", pcl::ComparisonOps::LT, 220));

  pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr
      blue_condition_h(new pcl::PackedRGBComparison<pcl::PointXYZRGB>("b", pcl::ComparisonOps::GT, 50 ));
  pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr
      blue_condition_l(new pcl::PackedRGBComparison<pcl::PointXYZRGB>("b", pcl::ComparisonOps::LT, 220));

  // add all requirements under the condition
  pcl::ConditionOr<pcl::PointXYZRGB>::Ptr color_cond (new pcl::ConditionOr<pcl::PointXYZRGB> ());
  color_cond->addComparison (red_condition_h);
  //color_cond->addComparison (red_condition_l);
  //color_cond->addComparison (green_condition_l);
  color_cond->addComparison (green_condition_h);
  color_cond->addComparison (blue_condition_h);
  //color_cond->addComparison (blue_condition_l);

  // Build the filter
  color_filter.setInputCloud(in_cloud_ptr);
  color_filter.setCondition (color_cond);
  color_filter.filter(*out_cloud_ptr);
  
  return;
}

///////////////////////////////////////////////////////////////////////////////

void
SrvClass::find_orientation_Proportion (PointCPtr &in_cloud_ptr)
{
  /* This function is used to calculate the orientation angle. */
 
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
  coefficients->values.resize(4);
  coefficients->values[0] = 0;
  coefficients->values[1] = 0;
  coefficients->values[2] = 1;
  coefficients->values[3] = 0;

  // Create the filtering object
  pcl::ProjectInliers<pcl::PointXYZRGB> proj; 
  proj.setModelType(pcl::SACMODEL_PLANE);  
  proj.setInputCloud(in_cloud_ptr);            
  proj.setModelCoefficients(coefficients); 
  proj.filter(*g_cloudfind_orientation_Proportion);          

	pcl::PointXYZRGB min, max;
	pcl::getMinMax3D(*g_cloudfind_orientation_Proportion, min, max);
  //cout << "find_orientation_Proportion: "  << endl;

  // variables initiallization
  float max_x_y;
  float min_x_y;
  float max_y_x;
  float min_y_x;

  // find out the coordinate of the maximum and minimun x and y value in given point cloud
  for(size_t i=0;i<g_cloudfind_orientation_Proportion->points.size();++i)
  {
    if (g_cloudfind_orientation_Proportion->points[i].x == max.x)
    {
      max_x_y = g_cloudfind_orientation_Proportion->points[i].y;
      //ROS_INFO("max_x_y = %f ",max_x_y);
    }
    else if (g_cloudfind_orientation_Proportion->points[i].x == min.x)
    {
      min_x_y = g_cloudfind_orientation_Proportion->points[i].y;
      //ROS_INFO("min_x_y = %f ",min_x_y);
    }
    else if (g_cloudfind_orientation_Proportion->points[i].y == max.y)
    {
      max_y_x = g_cloudfind_orientation_Proportion->points[i].x;
      //ROS_INFO("max_y_x = %f ",max_y_x);
    }
    else if (g_cloudfind_orientation_Proportion->points[i].y == min.y)
    {
      min_y_x = g_cloudfind_orientation_Proportion->points[i].x;
      //ROS_INFO("min_y_x = %f ",min_y_x);
    }
  }

  // when orientation = 0, it is hard to calculate orientation, so that use difference
  // in lenth to find out this situation.
  if (max.y- min.y <=0.045)
  {
    g_cube_orientation=0;
  }
  else
  {
    // using two edges of the cube forming a vector to calculate the orientation
    g_cube_orientation = acos((max.y-min_x_y)/sqrt(pow(max.y-min_x_y,2)+pow(max_y_x-min.x,2)))*180/M_PI;
  }
  ROS_INFO("find_orientation %f ",g_cube_orientation);
  return;
}

///////////////////////////////////////////////////////////////////////////////

bool
SrvClass::move_to_side()
{  
  tf2::Quaternion q_x180deg(-1, 0, 0, 0);
  // determine the grasping orientation
  tf2::Quaternion q_object;
  q_object.setRPY(0.0, 0.0, angle_offset_);
  tf2::Quaternion q_result = q_x180deg * q_object;
  geometry_msgs::Quaternion grasp_orientation = tf2::toMsg(q_result);

  // move arms to starting pose
  geometry_msgs::Pose grasp_pose1;
  grasp_pose1.position.x = scan0_pose_x;
  grasp_pose1.position.y = scan0_pose_y;
  grasp_pose1.orientation = grasp_orientation;
  grasp_pose1.position.z = scan0_pose_z;
  moveArm(grasp_pose1);
  }

///////////////////////////////////////////////////////////////////////////////

bool
SrvClass::moveToCorner()
{
  tf2::Quaternion q_x180deg(-1, 0, 0, 0);
  // determine the grasping orientation
  tf2::Quaternion q_object;
  q_object.setRPY(0.0, 0.0, angle_offset_);
  tf2::Quaternion q_result = q_x180deg * q_object;
  geometry_msgs::Quaternion grasp_orientation = tf2::toMsg(q_result);

  // move arms to starting pose
  geometry_msgs::Pose starting_pose;
  starting_pose.position.x = 0.4;
  starting_pose.position.y = 0;
  starting_pose.orientation = grasp_orientation;
  starting_pose.position.z = 0.7;
  moveArm(starting_pose);
}

///////////////////////////////////////////////////////////////////////////////

bool
SrvClass::scan_straight()
{

  tf2::Quaternion q_x180deg(-1, 0, 0, 0);
  // determine the grasping orientation
  tf2::Quaternion q_object;
  q_object.setRPY(0.0, 0.0, angle_offset_);
  tf2::Quaternion q_result = q_x180deg * q_object;
  geometry_msgs::Quaternion grasp_orientation = tf2::toMsg(q_result);

  vector<int> num;
  num.push_back(1);
  num.push_back(2);
  num.push_back(3);
  num.push_back(4);
  num.push_back(5);
  num.push_back(6);

  sleep(1);
  for (auto item: num){
    double scan_x_pose = scan0_pose_x + item*(scan_pose_x-scan0_pose_x)/6;
    double scan_y_pose = scan0_pose_y + item*(scan_pose_y-scan0_pose_y)/6;
    double scan_z_pose = scan0_pose_z + item*(scan_pose_z-scan0_pose_z)/6;
    geometry_msgs::Pose grasp_pose0;
    grasp_pose0.position.x = scan_x_pose;
    grasp_pose0.position.y = scan_y_pose;
    grasp_pose0.orientation = grasp_orientation;
    grasp_pose0.position.z = scan0_pose_z;

    moveArm(grasp_pose0);
  }

  ////////// pose0-pose1 //////////
  geometry_msgs::Pose grasp_pose;
  grasp_pose.position.x = scan_pose_x;
  grasp_pose.position.y = scan_pose_y;
  grasp_pose.orientation = grasp_orientation;
  grasp_pose.position.z = scan_pose_z;
  moveArm(grasp_pose);
  for (auto item: num){
    double scan_x = scan_pose_x + item*(scan1_pose_x-scan_pose_x)/6;
    double scan_y = scan_pose_y + item*(scan1_pose_y-scan_pose_y)/6;
    geometry_msgs::Pose pose;
    pose.position.x = scan_x;
    pose.position.y = scan_y;
    pose.orientation = grasp_orientation;
    pose.position.z = scan_pose_z;

    moveArm(pose);
  
  }

  ////////// pose1-pose2 //////////
  geometry_msgs::Pose grasp_pose1;
  grasp_pose1.position.x = scan1_pose_x;
  grasp_pose1.position.y = scan1_pose_y;
  grasp_pose1.orientation = grasp_orientation;
  grasp_pose1.position.z = scan1_pose_z;
  moveArm(grasp_pose1);
  for (auto item: num){
    double scan1_x = scan1_pose_x + item*(scan2_pose_x-scan1_pose_x)/6;
    double scan1_y = scan1_pose_y + item*(scan2_pose_y-scan1_pose_y)/6;
    double scan1_z = scan1_pose_z + item*(scan2_pose_z-scan1_pose_z)/6;
    geometry_msgs::Pose pose1;
    pose1.position.x = scan1_x;
    pose1.position.y = scan1_y;
    pose1.orientation = grasp_orientation;
    pose1.position.z = scan1_pose_z;

    moveArm(pose1);
  }

  ////////// pose2-pose3 //////////
  geometry_msgs::Pose grasp_pose2;
  grasp_pose2.position.x = scan2_pose_x;
  grasp_pose2.position.y = scan2_pose_y;
  grasp_pose2.orientation = grasp_orientation;
  grasp_pose2.position.z = scan2_pose_z;
  moveArm(grasp_pose2);
  for (auto item: num){
    double scan2_x = scan2_pose_x + item*(scan3_pose_x-scan2_pose_x)/6;
    double scan2_y = scan2_pose_y + item*(scan3_pose_y-scan2_pose_y)/6;
    double scan2_z = scan2_pose_z + item*(scan3_pose_z-scan2_pose_z)/6;
    geometry_msgs::Pose pose2;
    pose2.position.x = scan2_x;
    pose2.position.y = scan2_y;
    pose2.orientation = grasp_orientation;
    pose2.position.z = scan2_pose_z;

    moveArm(pose2);
  }

  ////////// pose3-pose4 //////////
  geometry_msgs::Pose grasp_pose3;
  grasp_pose3.position.x = scan3_pose_x;
  grasp_pose3.position.y = scan3_pose_y;
  grasp_pose3.orientation = grasp_orientation;
  grasp_pose3.position.z = scan3_pose_z;
  moveArm(grasp_pose3);
  for (auto item: num){
    double scan3_x = scan3_pose_x + item*(scan4_pose_x-scan3_pose_x)/6;
    double scan3_y = scan3_pose_y + item*(scan4_pose_y-scan3_pose_y)/6;
    geometry_msgs::Pose pose3;
    pose3.position.x = scan3_x;
    pose3.position.y = scan3_y;
    pose3.orientation = grasp_orientation;
    pose3.position.z = scan3_pose_z;

    moveArm(pose3);
  }

  geometry_msgs::Pose grasp_pose4;
  grasp_pose4.position.x = scan4_pose_x;
  grasp_pose4.position.y = scan4_pose_y;
  grasp_pose4.orientation = grasp_orientation;
  grasp_pose4.position.z = scan4_pose_z;

  moveArm(grasp_pose4);
}
///////////////////////////////////////////////////////////////////////////////

bool
SrvClass::scan_round()
{

  tf2::Quaternion q_x180deg(-1, 0, 0, 0);
  // determine the grasping orientation
  tf2::Quaternion q_object;
  q_object.setRPY(0.0, 0.0, angle_offset_);
  tf2::Quaternion q_result = q_x180deg * q_object;
  geometry_msgs::Quaternion grasp_orientation = tf2::toMsg(q_result);

  vector<int> num;
  num.push_back(1);
  num.push_back(2);
  num.push_back(3);
  num.push_back(4);//round1_pose_x
  num.push_back(5);
  num.push_back(6);
  num.push_back(7);
  num.push_back(8);

  if(g_point_cloud_switch==0)
  {
    ////////// pose0-pose1 //////////
    for (auto item: num){
      double round1_x = 0.4;
      double round1_y = 0 + item*(0.375)/8;
      geometry_msgs::Pose pose1;
      pose1.position.x = round1_x;
      pose1.position.y = round1_y;
      pose1.orientation = grasp_orientation;
      pose1.position.z = 0.7;
      moveArm(pose1);
    }
    ////////// pose1-pose2 //////////
    for (auto item: num){
      double round2_x = 0.4 - item*(0.48 + 0.4)/8;
      double round2_y = 0.375;
      geometry_msgs::Pose pose2;
      pose2.position.x = round2_x;
      pose2.position.y = round2_y;
      pose2.orientation = grasp_orientation;
      pose2.position.z = 0.7;
      moveArm(pose2);
    }
    ////////// pose2-pose3 //////////
    for (auto item: num){
      double round3_x = -0.48;
      double round3_y = 0.375 - item*(0.15)/8;
      geometry_msgs::Pose pose3;
      pose3.position.x = round3_x;
      pose3.position.y = round3_y;
      pose3.orientation = grasp_orientation;
      pose3.position.z = 0.7;
      moveArm(pose3);
  
    }
    // switch point cloud
    g_point_cloud_switch = 1;
 
    ////////// pose3-pose2 //////////
    double round4_x = -0.48;
    double round4_y = 0.375;
    geometry_msgs::Pose pose2;
    pose2.position.x = round4_x;
    pose2.position.y = round4_y;
    pose2.orientation = grasp_orientation;
    pose2.position.z = 0.7;
    moveArm(pose2);
    ////////// pose2-pose1.5 //////////
    double round5_x = 0.0;
    double round5_y = 0.375;
    geometry_msgs::Pose pose2_1;
    pose2_1.position.x = round5_x;
    pose2_1.position.y = round5_y;
    pose2_1.orientation = grasp_orientation;
    pose2_1.position.z = 0.6;
    moveArm(pose2_1);
    ////////// pose2-pose1 //////////
    double round6_x = 0.4;
    double round6_y = 0.375;
    geometry_msgs::Pose pose1;
    pose1.position.x = round6_x;
    pose1.position.y = round6_y;
    pose1.orientation = grasp_orientation;
    pose1.position.z = 0.7;
    moveArm(pose1);
  }
  else
  {
    // switch point cloud
    g_point_cloud_switch = 0;

    ////////// pose0-pose4 //////////
    for (auto item: num){
      double round7_x = 0.4;
      double round7_y = 0 + item*(-0.375)/8;
      geometry_msgs::Pose pose4;
      pose4.position.x = round7_x;
      pose4.position.y = round7_y;
      pose4.orientation = grasp_orientation;
      pose4.position.z = 0.7;
      moveArm(pose4);
    }
    ////////// pose4-pose5 //////////
    for (auto item: num){
      double round8_x = 0.4 - item*(0.48 + 0.4)/8;
      double round8_y = -0.375;
      geometry_msgs::Pose pose5;
      pose5.position.x = round8_x;
      pose5.position.y = round8_y;
      pose5.orientation = grasp_orientation;
      pose5.position.z = 0.7;
      moveArm(pose5);
    }
    ////////// pose5-pose6 //////////
    for (auto item: num){
      double round9_x = -0.48;
      double round9_y = -0.375 + item*(0.06)/8;
      geometry_msgs::Pose pose6;
      pose6.position.x = round9_x;
      pose6.position.y = round9_y;
      pose6.orientation = grasp_orientation;
      pose6.position.z = 0.7;
      moveArm(pose6);
    }
  }
}

///////////////////////////////////////////////////////////////////////////////

void
SrvClass::addCollisionCallback(geometry_msgs::Point ground_centre)
{
  /* add  collision for ground in RViz and the MoveIt planning scene */

  // set the dimention
  geometry_msgs::Quaternion orientation;
  orientation.x = 0;
  orientation.y = 0;
  orientation.z = 0;
  orientation.w = 1;

  // set the dimention of the ground
  geometry_msgs::Vector3 ground_dimention;
  ground_dimention.x = 3;
  ground_dimention.y = 3;
  ground_dimention.z = 0.001;

  // set the z value = 0 for ground centre
  ground_centre.z = 0;

  // add both basket and ground with boundary
  addCollisionObject(ground_name, ground_centre, 
  ground_dimention,orientation);

  return;
}

///////////////////////////////////////////////////////////////////////////////

void
SrvClass::addCollisionObject(std::string object_name,
  geometry_msgs::Point centre, geometry_msgs::Vector3 dimensions,
  geometry_msgs::Quaternion orientation)
{
  /* add a collision object in RViz and the MoveIt planning scene */

  // create a collision object message, and a vector of these messages
  moveit_msgs::CollisionObject collision_object;
  std::vector<moveit_msgs::CollisionObject> object_vector;
  
  // input header information
  collision_object.id = object_name;
  collision_object.header.frame_id = base_frame_;

  // define the primitive and its dimensions
  collision_object.primitives.resize(1);
  collision_object.primitives[0].type = collision_object.primitives[0].BOX;
  collision_object.primitives[0].dimensions.resize(3);
  collision_object.primitives[0].dimensions[0] = dimensions.x;
  collision_object.primitives[0].dimensions[1] = dimensions.y;
  collision_object.primitives[0].dimensions[2] = dimensions.z;

  // define the pose of the collision object
  collision_object.primitive_poses.resize(1);
  collision_object.primitive_poses[0].position.x = centre.x;
  collision_object.primitive_poses[0].position.y = centre.y;
  collision_object.primitive_poses[0].position.z = centre.z;
  collision_object.primitive_poses[0].orientation = orientation;

  // define that we will be adding this collision object 
  collision_object.operation = collision_object.ADD;

  // add the collision object to the vector, then apply to planning scene
  object_vector.push_back(collision_object);
  planning_scene_interface_.applyCollisionObjects(object_vector);

  return;
}

///////////////////////////////////////////////////////////////////////////////

bool
SrvClass::moveArm(geometry_msgs::Pose target_pose)
{
  /* This function moves the move_group to the target position */

  // setup the target pose
  ROS_INFO("Setting pose target");
  arm_group_.setPoseTarget(target_pose);

  // create a movement plan for the arm
  ROS_INFO("Attempting to plan the path");
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (arm_group_.plan(my_plan) ==
    moveit::planning_interface::MoveItErrorCode::SUCCESS);

  // google 'c++ conditional operator' to understand this line
  ROS_INFO("Visualising plan %s", success ? "" : "FAILED");

  // execute the planned path
  arm_group_.move();

  return success;
}

///////////////////////////////////////////////////////////////////////////////

bool
SrvClass::moveGripper(float width)
{
  /* this function moves the gripper fingers to a new position. Joints are:
      - panda_finger_joint1
      - panda_finger_joint2
  */

  // safety checks
  if (width > gripper_open_) width = gripper_open_;
  if (width < gripper_closed_) width = gripper_closed_;

  // calculate the joint targets as half each of the requested distance
  double eachJoint = width / 2.0;

  // create a vector to hold the joint target for each joint
  std::vector<double> gripperJointTargets(2);
  gripperJointTargets[0] = eachJoint;
  gripperJointTargets[1] = eachJoint;

  // apply the joint target
  hand_group_.setJointValueTarget(gripperJointTargets);

  // move the robot hand
  ROS_INFO("Attempting to plan the path");
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (hand_group_.plan(my_plan) ==
    moveit::planning_interface::MoveItErrorCode::SUCCESS);

  ROS_INFO("Visualising plan %s", success ? "" : "FAILED");

  hand_group_.move();

  return success;
}

///////////////////////////////////////////////////////////////////////////////

void
SrvClass::removeCollisionObject(std::string object_name)
{
  /* this function remove a collision object from the planning scene */

  moveit_msgs::CollisionObject collision_object;
  std::vector<moveit_msgs::CollisionObject> object_vector;
  
  // input the name and specify we want it removed
  collision_object.id = object_name;
  collision_object.operation = collision_object.REMOVE;

  // apply this collision object removal to the scene
  object_vector.push_back(collision_object);
  planning_scene_interface_.applyCollisionObjects(object_vector);
}

///////////////////////////////////////////////////////////////////////////////

bool
SrvClass::pickAndPlace(geometry_msgs::Point position_object,
  geometry_msgs::Point position_goal,int cube_number,double pick_orientation, float place_orientation)
{
  /* This function picks up an object using a pose. The given point is where 
  the centre of the gripper fingers will converge */
  position_goal.z= (cube_number-1)*0.04-0.03;

  /** \brief cube. */
  geometry_msgs::Vector3 cube_size;
  cube_size.x = 0.04;
  cube_size.y = 0.04;
  cube_size.z = 0.04;
  geometry_msgs::Quaternion cube_angle;
  cube_angle.x = 0.0;
  cube_angle.y = 0.0;
  cube_angle.z = 0.0;
  cube_angle.w = 1.0;

  // define grasping as from above
  tf2::Quaternion q_x180deg(-1, 0, 0, 0);

  // determine the grasping orientation
  tf2::Quaternion q_object;
  q_object.setRPY(0, 0, pick_orientation+angle_offset_);
  int neg_offset = -1;
  tf2::Quaternion q_result = q_x180deg * q_object * neg_offset;
  geometry_msgs::Quaternion horizontal_orientation = tf2::toMsg(q_result);

  // set the desired grasping pose
  geometry_msgs::Pose grasp_pose;
  grasp_pose.position = position_object;
  grasp_pose.orientation = horizontal_orientation;
  grasp_pose.position.z += z_offset_;

  // set the desired pre-grasping pose
  geometry_msgs::Pose approach_pose;
  approach_pose = grasp_pose;
  approach_pose.position.z += approach_distance_;

  tf2::Quaternion q_object_release;
  q_object_release.setRPY(0, 0, place_orientation+angle_offset_);
  tf2::Quaternion q_result_release = q_x180deg * q_object * neg_offset;
  geometry_msgs::Quaternion release_orientation = tf2::toMsg(q_result_release);
  // set the desired release pose
  geometry_msgs::Pose release_pose;
  release_pose.position = position_goal;
  release_pose.orientation = release_orientation;
  release_pose.position.z += approach_distance_;

  /* Now perform the pick and place*/

  position_object.z += 0.027;

  //addCollisionObject(box_name,position_object,cube_size,
  // horizontal_orientation);
  
  bool success = true;

  ROS_INFO("Begining pick operation");


  // move the arm above the object
  success *= moveArm(approach_pose);
  sleep(1);

  if (not success) 
  {
    ROS_ERROR("Moving arm to pick approach pose failed");
    return false;
  }

  // open the gripper
  success *= moveGripper(gripper_open_);
  sleep(1);
  if (not success) 
  {
    ROS_ERROR("Opening gripper prior to pick failed");
    return false;
  }

  grasp_pose.position.z += 0.006;
  // approach to grasping pose
  success *= moveArm(grasp_pose);
  sleep(1);

  if (not success) 
  {
    ROS_ERROR("Moving arm to grasping pose failed");
    return false;
  }

  // remove collision object box
  std::string num = std::to_string(cube_number);
  removeCollisionObject("cube"+num);

  // grasp!
  success *= moveGripper(gripper_closed_);
  sleep(1);

  if (not success) 
  {
    ROS_ERROR("Closing gripper to grasp failed");
    return false;
  }

  ROS_INFO("Pick operation successful, begining place operation");

  grasp_pose.position.z = 0.4;
  // pick up the cube
  success *= moveArm(grasp_pose);
  sleep(1);

  if (not success) 
  {
    ROS_ERROR("Moving arm to grasping pose failed");
    return false;
  }

  // move the arm to the top of the goal position
  success *= moveArm(release_pose);
  sleep(1);

  if (not success) 
  {
    ROS_ERROR("Moving arm to place approach pose failed");
    return false;
  }

  release_pose.position.z -= 0.016;
  success *= moveArm(release_pose);
  sleep(1);

  if (not success) 
  {
    ROS_ERROR("Moving arm to place approach pose failed");
    return false;
  }

  // open the gripper to place the object into box
  success *= moveGripper(gripper_open_);
  sleep(1);

  if (not success) 
  {
    ROS_ERROR("Opening gripper prior to place failed");
    return false;
  }

  release_pose.position.z = 0.4;
  success *= moveArm(release_pose);
  sleep(1);

  if (not success) 
  {
    ROS_ERROR("Moving arm to place approach pose failed");
    return false;
  }

  std::string s = std::to_string(cube_number-1);
  position_goal.z += 0.09;

  addCollisionObject("stack"+s,position_goal,cube_size,
   release_orientation);

  ROS_INFO("Place operation successful");

  return true;
}