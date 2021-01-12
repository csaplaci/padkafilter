#include <stdio.h>
#include <thread>
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
float width=0.2;
float length=26.0;
float offset = 2.0; //radiális offset
float rmin = 2.0;
float rmax = 26.0;
float Kfi, minfi, maxfi;
double slope_param=1.732; //60 deg
std::string frame_param = "right_os1/os1_sensor"; //lidar frame
std::string topic_param = "/right_os1/os1_cloud_node/points"; //input topic

struct polar {
  int id;
  float r;
  float fi;
};
struct box {
  std::vector<polar> p;
  box *l,*r; //szomszédos filterek - l=left=next, r=right=previous (névütközés miatt)
  bool yx; //y>x (tan -> fi > 45 deg?)
  float fi, o, d; //szög + oldalirányú (tg offset) és hosszirányú (radiális) szorzó paraméterek
};

std::vector<box> beams(rep);
std::vector<box*> beamp(rep+1);

visualization_msgs::Marker marker;

std::vector<geometry_msgs::Pose> data(rep);
std::vector<geometry_msgs::Point> mpoint(rep);

ros::Publisher boxfilcloud_pub;
ros::Publisher marker_pub;

bool ptcmpr(polar a, polar b) { return (a.r<b.r); } // r-koordináta alapú összehasonlítás
struct chain
{
  float x,y,slope;
};

float slope(float x0, float y0, float x1, float y1)
{
  return (y1-y0)/(x1-x0);
}

void threadfunc (const int tid, const pcl::PointCloud<pcl::PointXYZ> *cloud)
{
  int i=0, s=beams[tid].p.size();
  float c;
  if (beams[tid].yx)
  {
    while (i<s)
    {
      c=abs(beams[tid].d * cloud->points[beams[tid].p[i].id].y); //nyaláb középvonalának x-koordinátája a pontnál (ahol a pont y-koordinátája metszi azt)

      if ((c - beams[tid].o) < cloud->points[beams[tid].p[i].id].x < (c + beams[tid].o)) //középvonal +-nyaláb (fél)szélessége x-irányban - beleesik-e a pont
      {
        //beams[tid].p[i].r*=cos(beams[tid].fi-beams[tid].p[i].fi); // transform - nem tudom miért, de zajosabb ettől (fordítva kéne lennie)
        i++;
      }
      else
      {
        beams[tid].p.erase(beams[tid].p.begin()+i);
        s--;
      }
    }
  }
  else
  {
    while (i<s)
    {
      c=abs(beams[tid].d * cloud->points[beams[tid].p[i].id].x); //nyaláb középvonalának y-koordinátája a pontnál (ahol a pont x-koordinátája metszi azt)

      if ((c - beams[tid].o) < cloud->points[beams[tid].p[i].id].y < (c + beams[tid].o)) //középvonal +-nyaláb (fél)szélessége y-irányban - beleesik-e a pont
      {
        //beams[tid].p[i].r*=cos(beams[tid].fi-beams[tid].p[i].fi);
        i++;
      }
      else
      {
        beams[tid].p.erase(beams[tid].p.begin()+i);
        s--;
      }
    }
  }

  std::sort(beams[tid].p.begin(),beams[tid].p.end(),ptcmpr);

  {
    float max=0;
    int maxind=0;
    bool found=false;
    for (int i=0;i<s;i++)
    {
      data[tid].position.x = beams[tid].p[i].r;
      data[tid].position.z = cloud->points[beams[tid].p[i].id].z;
      data[tid].position.y = (i) ? (slope(beams[tid].p[i-1].r,cloud->points[beams[tid].p[i-1].id].z,beams[tid].p[i].r,cloud->points[beams[tid].p[i].id].z)):(0.0); // z/r slope(i-1,i)
      //if (not false positive)
      if (!found)
      {
        if (data[tid].position.y>max)
        {
          max=data[tid].position.y;
          maxind=i;
          if (data[tid].position.y>slope_param) found=true;
        }
      }
    }
  
    if (found)
    {
      mpoint[tid].x = cloud->points[beams[tid].p[maxind].id].x;
      mpoint[tid].y = cloud->points[beams[tid].p[maxind].id].y;
      mpoint[tid].z = cloud->points[beams[tid].p[maxind].id].z;
      marker.points.push_back(mpoint[tid]);
    }
  }
  beams[tid].p.clear();
}

void dyncfg_callback(padkafilter::PadkaConfig &config, uint32_t level)
{
  slope_param = tan(config.slope_angle/180*M_PI);
  frame_param = config.frame;
  topic_param = config.topic;
  ROS_INFO("Slope angle: %f [DEG]", config.slope_angle);
  ROS_INFO("Frame: \"%s\"", frame_param.c_str());
  ROS_INFO("Topic: \"%s\" (node needs to be restarted after reconfiguration)", topic_param.c_str());
};

void callback(const pcl::PCLPointCloud2ConstPtr &cloud_in)
{
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromPCLPointCloud2(*cloud_in,cloud);
    marker.points.clear();
    int f, s=cloud.size();
    float r,fi;

    for (int i=0;i<s;i++)
    {
      r=sqrt(cloud.points[i].x*cloud.points[i].x+cloud.points[i].y*cloud.points[i].y);
      if (r>rmax || r<rmin) continue;
      fi=atan2(cloud.points[i].y,cloud.points[i].x);
      if (fi<0) fi+=2*M_PI;
      f=(int)(fi*Kfi);
      beamp[f]->p.push_back(polar{i,r,fi});
        /*
        if (r<5)
        {

        }//szomszédos nyalábokra is
        */
    }
/*
    std::vector<std::thread> beam_threads;
    for (int i=0; i<rep; i++)
    {
      beam_threads.push_back(std::thread(threadfunc, i, &cloud));
    }

    for (auto &th : beam_threads)
    {
      th.join();
    }
    */

    for (int i=0;i<rep;i++)
    {
      threadfunc(i,&cloud);
    }

    std_msgs::String msg;
    std::stringstream ss;

    ss << "[debug] marker points: " << marker.points.size();

    msg.data = ss.str();
    ROS_INFO("%s", msg.data.c_str());

    // Publish the marker
    marker_pub.publish(marker);
    
}

int main(int argc, char **argv)
{
  ros::init(argc,argv,"PadkaFilter");
  ros::NodeHandle nh;
  ROS_INFO("Padkafilter node started.");
  ROS_INFO("STATUS: INITIALIZING...");
  ROS_INFO("init_param: %f", slope_param);


  { //nyaláb inicializálás
    float fi, off = 0.5*width; // fi + tangenciális offset (szélesség fele)
    for (int i=0; i<rep; i++)
    {
      fi=i*2*M_PI/rep; // nyaláb (x-tengellyel bezárt) szöge
      if (abs(tan(fi))>1) // y>x ? (kb. melyik irányba áll inkább)
      {
        beams[i].yx=true; //y-irányú
        beams[i].d = tan(0.5*M_PI-fi); // =1/tan(fi)
        beams[i].o = off/sin(fi);
        beams[i].fi=fi; // kell?
      }
      else
      {
        beams[i].yx=false; //x-irányú
        beams[i].d = tan(fi);
        beams[i].o = off/cos(fi);
        beams[i].fi=fi;
      }
      beamp[i]=&beams[i]; //pointerek (lásd lentebb [n+1 -> 0])
    }
    beamp[rep+1]=&beams[0]; //az n+1-edik elem a 0-dik elemre hivatkozzon (körkörös hivatkozás kerekítésből adódó hivatkozási hiba megelőzése céljából)
  }
  for (int i=0,j=1; j<rep; i++,j++) //pointerek a szomszédos nyalábokra (könnyebb kezelhetőségért - egybeeső területekhez - kell?)
  {
    beams[i].l=&beams[j];
    beams[j].r=&beams[i];
  }
  beams[0].r=&beams[rep];
  beams[rep].l=&beams[0];

  //Kfi = 2*M_PI/rep; //reciprokkal elvileg gyorsabb, mert akkor a továbbiakban szorozni kell, nem osztani
  Kfi = rep/(2*M_PI);

  dynamic_reconfigure::Server<padkafilter::PadkaConfig> dyncfg_server;
  dynamic_reconfigure::Server<padkafilter::PadkaConfig>::CallbackType cbt;

  cbt = boost::bind(dyncfg_callback, _1, _2);
  dyncfg_server.setCallback(cbt);

  ros::Subscriber sub = nh.subscribe(topic_param, 1, callback);
  //boxfilcloud_pub = nh.advertise<pcl::PCLPointCloud2>("boxfilter", 1);
  marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  //to_py_pub = nh.advertise<geometry_msgs::PoseArray>("/tmp_py", 1);

  marker.header.frame_id = frame_param;
  marker.header.stamp = ros::Time::now();
  // Set the namespace and id for this marker.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  marker.ns = "basic_shapes";
  marker.id = 0;

  marker.lifetime = ros::Duration();
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


  ROS_INFO("STATUS: READY");
  ros::spin();
}