#include <stdio.h>
#include <iostream>
#include <math.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>
#include <geometry_msgs/PoseArray.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/crop_box.h>
#include <visualization_msgs/Marker.h>
#include <dynamic_reconfigure/server.h>
#include <padkafilter/PadkaConfig.h>

int rep=128; // number of detection boxes
double slope_param=1.732; //60 deg

std::vector<float> bsin(rep),bcos(rep),nsin(rep),ncos(rep);

ros::Publisher boxfilcloud_pub;
ros::Publisher marker_pub;
ros::Publisher to_py_pub;

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

bool ptcmpx(pcl::PointXYZ a, pcl::PointXYZ b) { return (a.x<b.x); } // comparing points by x coordinates

struct chain
{
  float x,y,slope;
};

float slope(float x0, float y0, float x1, float y1)
{
  return (y1-y0)/(x1-x0);
}

std::vector<float> smoothen (std::vector<float> in_points, std::vector<float> kernel, float dist) //convolution smoothener
{
  int k=floor(kernel.size()/2);
  std::vector<float> pts;
  float d, n, temp=0;
  int s;
  for (int i=0;i<in_points.size();i++)
  {
    temp=in_points[i];
    for (int j=0;j<kernel.size();j++)
    {
      if ((i-k+j)<0) n=in_points[0];
      else if (i-k+j>=in_points.size()) n=in_points[in_points.size()-1];
      else n=in_points[i-k+j];
      d=abs(n-in_points[i]);
      s=sgn(n-in_points[i]);
      //temp+=s*kernel[j]*(1/(dist*1+d))*dist; //nemlineáris smootholás
      temp+=s*kernel[j]*d*dist;
    }
    pts.push_back(temp);
  }
  return pts;
};

void dyncfg_callback(padkafilter::PadkaConfig &config, uint32_t level)
{
  slope_param = tan(config.slope_angle/180*M_PI);
  ROS_INFO("Slope angle: %f [DEG]", config.slope_angle);
};

void callback(const pcl::PCLPointCloud2ConstPtr &cloud)
{
    pcl::PCLPointCloud2 cloud_filtered;

    // Define min and max for X, Y and Z
    float minX = 2.0, minY = -0.1, minZ = -3.0; //minZ = -1.384;
    float maxX = 26.0, maxY = +0.1, maxZ = 1.0; //maxZ = -0.15;

    pcl::CropBox<pcl::PCLPointCloud2> boxFilter;
    boxFilter.setMin(Eigen::Vector4f(minX, minY, minZ, 1.0));
    boxFilter.setMax(Eigen::Vector4f(maxX, maxY, maxZ, 1.0));
    boxFilter.setInputCloud(cloud);
    //
    float boxfi=0.0; //2.8;
    Eigen::Vector3f boxrot = Eigen::Vector3f::Zero();

    boxFilter.filter(cloud_filtered);

    visualization_msgs::Marker marker;
    geometry_msgs::PoseArray to_py_arr;
    to_py_arr.header.stamp = ros::Time::now();
    to_py_arr.header.frame_id = "right_os1/os1_sensor";
    
    marker.header.frame_id = "right_os1/os1_sensor";
    marker.header.stamp = ros::Time::now();
    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "basic_shapes";
    marker.id = 0;

    // Set the marker type.  Initially this was CUBE, and cycled between that and SPHERE, ARROW, and CYLINDER
    marker.type = visualization_msgs::Marker::SPHERE_LIST;

    // Set the marker action.  Options are ADD and DELETE
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = .2;
    marker.scale.y = .2;
    marker.scale.z = .2;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.2f;
    marker.color.g = 0.8f;
    marker.color.b = 0.2f;
    marker.color.a = 0.8f;

    marker.lifetime = ros::Duration();

    // transform

    /* Reminder: how transformation matrices work :

            |-------> This column is the translation
        | 1 0 0 x |  \
        | 0 1 0 y |   }-> The identity 3x3 matrix (no rotation) on the left
        | 0 0 1 z |  /
        | 0 0 0 1 |    -> We do not use this line (and it has to stay 0,0,0,1)

        METHOD #1: Using a Matrix4f
        This is the "manual" method, perfect to understand but error prone !


    Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();

    // Define a rotation matrix (see https://en.wikipedia.org/wiki/Rotation_matrix)
    float theta = M_PI/3; // The angle of rotation in radians
    transform_1 (0,0) = std::cos (theta);
    transform_1 (0,1) = -sin(theta);
    transform_1 (1,0) = sin (theta);
    transform_1 (1,1) = std::cos (theta);
    //    (row, column)

    // Define a translation of 2.5 meters on the x axis.
    transform_1 (0,3) = 2.5;
    */

    pcl::PCLPointCloud2 output_cloud;
    pcl::PointCloud<pcl::PointXYZ> cloud_out_xyz;
    pcl::PointCloud<pcl::PointXYZ> cloud_filt_xyz;
    Eigen::Matrix4f zrotmat = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f zrotback = Eigen::Matrix4f::Identity();
    geometry_msgs::Pose data;

    for (int mc=0;mc<rep;mc++)
    {
      boxfi = mc*(M_PI*2/rep);

      boxrot (2) = boxfi;
      
      boxFilter.setInputCloud(cloud);
      boxFilter.setMin(Eigen::Vector4f(minX, minY, minZ, 1.0));
      boxFilter.setMax(Eigen::Vector4f(maxX, maxY, maxZ, 1.0));
      boxFilter.setRotation(boxrot);
      boxFilter.filter(cloud_filtered);
      if(cloud_filtered.data.size() < 10)
      {
        ROS_WARN_STREAM("Few point");
        continue;
      }
      
      pcl::fromPCLPointCloud2(output_cloud, cloud_out_xyz);
      pcl::fromPCLPointCloud2(cloud_filtered, cloud_filt_xyz);

      int s=cloud_filt_xyz.size();
      
      /*
      zrotmat (0,0) = cos(-boxfi);
      zrotmat (0,1) = -sin(-boxfi);
      zrotmat (1,0) = sin(-boxfi);
      zrotmat (1,1) = cos(-boxfi);

      zrotback (0,0) = cos(boxfi);
      zrotback (0,1) = -sin(boxfi);
      zrotback (1,0) = sin(boxfi);
      zrotback (1,1) = cos(boxfi);
      */

      zrotmat (0,0) = ncos[mc];
      zrotmat (0,1) = -nsin[mc];
      zrotmat (1,0) = nsin[mc];
      zrotmat (1,1) = ncos[mc];

      zrotback (0,0) = bcos[mc];
      zrotback (0,1) = -bsin[mc];
      zrotback (1,0) = bsin[mc];
      zrotback (1,1) = bcos[mc];

      pcl::transformPointCloud(cloud_filt_xyz, cloud_out_xyz, zrotmat);
      std::sort(cloud_out_xyz.begin(),cloud_out_xyz.end(),ptcmpx);

      /*
      std::vector<float> smoothed;
      std::vector<float> kernel = {0.083333, 0.1666666, 0.5, 0.1666666, 0.083333};
      //std::vector<float> testx= {1,2,3,5,7,8,9,10,10.5}, testy= {0.5,1,1.5,2,5,7,4,3,5};
      {
      std::vector<float> to_smooth;
      for (int i=0;i<s;i++) { to_smooth.push_back(cloud_filt_xyz.points[i].z); }
      smoothed=smoothen(to_smooth,kernel,0.35);
      }
      */

      /*

      std::vector<chain> data;
      chain tempchain;

      for (int i=0;i<s;i++)
      {
        tempchain.y=smoothed[i];
        tempchain.x=cloud_filt_xyz.points[i].x;
        tempchain.slope = (i) ? (slope(data[i-1].x,data[i-1].y,tempchain.x,tempchain.y)):(0.0);
        data.push_back(tempchain);
      }
      */

      //pcl::transformPointCloud (cloud_filt_xyz, cloud_out_xyz, transform_1);

      {
        float max=0;
        int maxind=0;
        bool found=false;
        geometry_msgs::Point tempoint;
        for (int i=0;i<s;i++)
        {
          data.position.x = cloud_out_xyz.points[i].x;
          data.position.z = cloud_out_xyz.points[i].z;
          data.position.y = (i) ? (slope(cloud_out_xyz.points[i-1].x,cloud_out_xyz.points[i-1].z,cloud_out_xyz.points[i].x,cloud_out_xyz.points[i].z)):(0.0);
          //if (not false positive)
          if (!found)
          {
            if (data.position.y>max)
            {
              max=data.position.y;
              maxind=i;
              if (data.position.y>slope_param) found=true;
            }
          }

        /*
          data.position.x = testx[i];
          data.position.y = smoothed[i];
          data.position.z = testy[i];
        */

          if (mc==56) {to_py_arr.poses.push_back(data);}
        }
      
        pcl::transformPointCloud(cloud_out_xyz, cloud_filt_xyz, zrotback);

        if (found)
        {
          tempoint.x = cloud_filt_xyz.points[maxind].x;
          tempoint.y = cloud_filt_xyz.points[maxind].y;
          tempoint.z = cloud_filt_xyz.points[maxind].z;
          marker.points.push_back(tempoint);
        }
      }
    }

    //pcl::toPCLPointCloud2(cloud_out_xyz, output_cloud);

    std_msgs::String msg;
    std::stringstream ss;

    ss << "[debug] slope_param = " << slope_param;

    msg.data = ss.str();
    ROS_INFO("%s", msg.data.c_str());

    // Publish the boxfiltered cloud
    boxfilcloud_pub.publish(cloud_filtered);

    // Publish the marker
    marker_pub.publish(marker);

    // Publish the filtered LIDAR data
    to_py_pub.publish(to_py_arr);
    
}

int main(int argc, char **argv)
{
    ros::init(argc,argv,"PadkaFilter");
    ros::NodeHandle nh;
    ROS_INFO("Padkafilter node started.");
    ROS_INFO("init_param: %f", slope_param);

    for (int i=0; i<rep; i++)
    {
      bsin[i] = sin(M_PI*2/rep*i);
      bcos[i] = cos(M_PI*2/rep*i);
      nsin[i] = sin(-M_PI*2/rep*i);
      ncos[i] = cos(-M_PI*2/rep*i);
    }

    ros::Subscriber sub = nh.subscribe("/right_os1/os1_cloud_node/points", 1, callback);
    boxfilcloud_pub = nh.advertise<pcl::PCLPointCloud2>("boxfilter", 1);
    marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    to_py_pub = nh.advertise<geometry_msgs::PoseArray>("/tmp_py", 1);

    dynamic_reconfigure::Server<padkafilter::PadkaConfig> dyncfg_server;
    dynamic_reconfigure::Server<padkafilter::PadkaConfig>::CallbackType cbt;

    cbt = boost::bind(dyncfg_callback, _1, _2);
    dyncfg_server.setCallback(cbt);

    ros::spin();
}