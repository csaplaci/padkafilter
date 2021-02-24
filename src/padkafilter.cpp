#include <stdio.h>
#include <thread>
#include <iostream>
#include <math.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PolygonStamped.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/crop_box.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <dynamic_reconfigure/server.h>
#include <padkafilter/PadkaConfig.h>

int rep = 512; // number of detection beams
int beam_param = 0;
double width = 0.2;
double rmin = 2.0;
double rmax = 26.0;
double polydist = 4.0; // 2m (négyzete - a számításigény csökkentéséért) //todo ezeket paraméterként
double polyangle = 1.0; // (rad)
float Kfi;
double slope_param=1.732; //60 deg
std::string frame_param = "right_os1/os1_sensor"; //lidar frame
std::string topic_param = "/right_os1/os1_cloud_node/points"; //input topic
int ghost_holder, marker_ghost=0; //felesleges markerek eltávolításához
std_msgs::String msg;
std::stringstream ss;


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
  bool found;
};

std::vector<box> beams(rep);
std::vector<box*> beamp(rep+1);

visualization_msgs::Marker marker,arrtemp;
visualization_msgs::MarkerArray marray;

std::vector<geometry_msgs::Pose> data(rep);
std::vector<geometry_msgs::Point> mpoint(rep);

geometry_msgs::Point32 polyp; //polygon pontok (segédváltozó)
geometry_msgs::PolygonStamped poly; //teljes polygon (pontokból áll)


std::vector<int> tempoly; //segédváltozó
std::vector<std::vector<int>> polyvec; // a ténylegesen padkának vélt pontok indexeit tárolja

ros::Publisher boxfilcloud_pub;
ros::Publisher marker_pub;
ros::Publisher marker_array_pub;
ros::Publisher poly_pub;

bool ptcmpr(polar a, polar b) { return (a.r<b.r); } // r-koordináta alapú összehasonlítás
struct chain
{
  float x,y,slope;
};

float slope(float x0, float y0, float x1, float y1)
{
  return (y1-y0)/(x1-x0);
}

void beam_init() //nyaláb inicializálás
{
  {
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
}

void threadfunc (const int tid, const pcl::PointCloud<pcl::PointXYZ> *cloud)
{
  int i=0, s=beams[tid].p.size();
  float c;
  if (beams[tid].yx) //ez szűri a pontokat
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
    beams[tid].found=false;
    for (int i=0;i<s;i++) //élkeresés
    {
      data[tid].position.x = beams[tid].p[i].r;
      data[tid].position.z = cloud->points[beams[tid].p[i].id].z;
      data[tid].position.y = (i) ? (slope(beams[tid].p[i-1].r,cloud->points[beams[tid].p[i-1].id].z,beams[tid].p[i].r,cloud->points[beams[tid].p[i].id].z)):(0.0); // z/r slope(i-1,i)
      //if (not false positive)
      if (!beams[tid].found)
      {
        if (data[tid].position.y>max)
        {
          max=data[tid].position.y;
          maxind=i;
          if (data[tid].position.y>slope_param) beams[tid].found=true;
        }
      }
    }
  
    if (beams[tid].found)
    {
      mpoint[tid].x = cloud->points[beams[tid].p[maxind].id].x;
      mpoint[tid].y = cloud->points[beams[tid].p[maxind].id].y;
      mpoint[tid].z = cloud->points[beams[tid].p[maxind].id].z;
      marker.points.push_back(mpoint[tid]);
    }
  }
  beams[tid].p.clear();
}

void dyncfg_callback(padkafilter::PadkaConfig &config, uint32_t level) //paraméterek állítása
{
  slope_param = tan(config.slope_angle/180*M_PI);
  beam_param = config.number_of_beams;
  width = config.width;
  rmin = config.r_min;
  rmax = config.r_max;
  polydist = config.points_max_distance*config.points_max_distance;
  polyangle = config.points_max_angle/180*M_PI;
  frame_param = config.frame;
  topic_param = config.topic;

  ROS_INFO("[RECONFIGURABLE PARAMETERS]");
  ROS_INFO("[R] = Node needs to be restarted after reconfiguration.\n");
  ROS_INFO("Slope angle: %f [DEG]", config.slope_angle);
  ROS_INFO("[R] Number of detection lines/beams: %d", config.number_of_beams);
  ROS_INFO("Width of the detection lines/beams: %f [m]", config.width);
  ROS_INFO("Minimum radius of detection: %f [m]", config.r_min);
  ROS_INFO("Maximum radius of detection: %f [m]", config.r_max);
  ROS_INFO("Maximum distance allowed between detected (neighbouring) points: %f [m]", config.points_max_distance);
  ROS_INFO("Maximum angle allowed between detected (neighbouring) points: %f [DEG]", config.points_max_angle);
  ROS_INFO("Frame: \"%s\"", frame_param.c_str());
  ROS_INFO("[R] Topic: \"%s\"", topic_param.c_str());
  ROS_INFO("---\n\n");
};

void callback(const pcl::PCLPointCloud2ConstPtr &cloud_in)
{
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromPCLPointCloud2(*cloud_in,cloud);
    marker.points.clear();
    polyvec.clear();
    tempoly.clear();
    poly.polygon.points.clear();
    int f, s=cloud.size();
    float r,fi;

    for (int i=0;i<s;i++)
    {
      r=sqrt(cloud.points[i].x*cloud.points[i].x+cloud.points[i].y*cloud.points[i].y);
      if (r>rmax || r<rmin) continue;
      fi=atan2(cloud.points[i].y,cloud.points[i].x);
      if (fi<0) fi+=2*M_PI;
      // if (fi<minfi || fi>maxfi) continue;
      f=(int)(fi*Kfi);
      beamp[f]->p.push_back(polar{i,r,fi});
        /*
        if (r<5)
        {

        }//szomszédos nyalábokra is - todo?
        */
    }
/*
    std::vector<std::thread> beam_threads; //párhuzamosítás
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

    {
      float xp,xn,yp,yn,dirp,dirn;
      bool first;
      for (int j=0,i=1, s=marker.points.size()-1; i<s; i++) //talált pontokból a (vélhetően) padkavonalat alkotó pontok kiszűrése
      {
        //szomszédos pontok relatív koordinátái
        xp=marker.points[i].x-marker.points[i-1].x;
        xn=marker.points[i+1].x-marker.points[i].x;
        yp=marker.points[i].y-marker.points[i-1].y;
        yn=marker.points[i+1].y-marker.points[i].y;

        if ((xp*xp+yp*yp)<polydist && (xn*xn+yn*yn)<polydist && abs(atan2(yp,xp)-atan2(yn,xn))<polyangle) //két szomszédos pont közötti táv és szög elég kicsi-e, hogy padka-élet alkossanak - todo: bonyolultabb vizsgálat
        {
          if (!first)
          {
            tempoly.push_back(i-1); //első pont se maradjon ki
            first=true;
          }
          tempoly.push_back(i);
        }
        else if (tempoly.size())
        {
          tempoly.push_back(i);  //utolsó pont se maradjon ki
          polyvec.push_back(tempoly);
          j++;
          tempoly.clear();
          first=false;
        }
      }
    }
    if (tempoly.size()) polyvec.push_back(tempoly);
    tempoly.clear();
    marray.markers.clear();
    arrtemp.id=0;
    arrtemp.action = visualization_msgs::Marker::ADD;

    for(int j=0, s0=polyvec.size();j<s0;j++) //polygon megalkotása
    {
      arrtemp.points.clear();
      for (int i=0,s=polyvec[j].size();i<s;i++)
      {
        arrtemp.points.push_back(marker.points[polyvec[j][i]]);

        polyp.x=marker.points[polyvec[j][i]].x;
        polyp.y=marker.points[polyvec[j][i]].y;
        polyp.z=marker.points[polyvec[j][i]].z;
        poly.polygon.points.push_back(polyp);
      }

      if (arrtemp.points.size())
      {
        marray.markers.push_back(arrtemp);
        arrtemp.id++;
      }
    }

    //marker ghost removal
    ghost_holder=polyvec.size();
    arrtemp.id=ghost_holder;
    arrtemp.action = visualization_msgs::Marker::DELETE;
    
    for (int i=ghost_holder;i<marker_ghost;i++)
    {
      marray.markers.push_back(arrtemp);
      arrtemp.id++;
    }
    marker_ghost=ghost_holder;

    /*
    //debug
    ss << "rep: " << rep << ", markers: " << marker.points.size();
    msg.data = ss.str();
    ROS_INFO("%s", msg.data.c_str());
    */

    // publish
    marker_pub.publish(marker);
    marker_array_pub.publish(marray);
    poly_pub.publish(poly);

}

int main(int argc, char **argv)
{
  ros::init(argc,argv,"PadkaFilter");
  ros::NodeHandle nh;
  ROS_INFO("Padkafilter node started.");
  ROS_INFO("STATUS: INITIALIZING...");

  dynamic_reconfigure::Server<padkafilter::PadkaConfig> dyncfg_server;
  dynamic_reconfigure::Server<padkafilter::PadkaConfig>::CallbackType cbt;

  cbt = boost::bind(dyncfg_callback, _1, _2);
  dyncfg_server.setCallback(cbt);

  if (beam_param) rep=beam_param; else rep = 128;

  beams.clear();
  beamp.clear();
  data.clear();
  mpoint.clear();
  beams.resize(rep);
  beamp.resize(rep+1);
  data.resize(rep);
  mpoint.resize(rep);
  beam_init();

  ros::Subscriber sub = nh.subscribe(topic_param, 1, callback);
  //boxfilcloud_pub = nh.advertise<pcl::PCLPointCloud2>("boxfilter", 1);
  poly_pub = nh.advertise<geometry_msgs::PolygonStamped>("padka_polygon", 1);
  marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  marker_array_pub = nh.advertise<visualization_msgs::MarkerArray>("padka_marker_array", 1);
  //to_py_pub = nh.advertise<geometry_msgs::PoseArray>("/tmp_py", 1);

  poly.header.frame_id = frame_param;
  poly.header.stamp = ros::Time::now();

  marker.header.frame_id = frame_param;
  marker.header.stamp = ros::Time::now();
  // Set the namespace and id for this marker.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  marker.ns = "basic_shapes";
  marker.id = 0;

  marker.lifetime = ros::Duration();
  // Set the marker type.
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
  marker.scale.x = .35;
  marker.scale.y = .35;
  marker.scale.z = .35;

  // Set the color -- be sure to set alpha to something non-zero!
  marker.color.r = 1.0f;
  marker.color.g = 0.2f;
  marker.color.b = 0.2f;
  marker.color.a = 1.0f;

  //marker array (padkavonalak)
  arrtemp.header.frame_id = frame_param;
  arrtemp.header.stamp = ros::Time::now();
  arrtemp.ns = "padka_lines";
  arrtemp.id = 0;

  arrtemp.lifetime = ros::Duration();
  arrtemp.type = visualization_msgs::Marker::LINE_STRIP;
  arrtemp.action = visualization_msgs::Marker::ADD;

  arrtemp.pose.position.x = 0;
  arrtemp.pose.position.y = 0;
  arrtemp.pose.position.z = 0;
  arrtemp.pose.orientation.x = 0.0;
  arrtemp.pose.orientation.y = 0.0;
  arrtemp.pose.orientation.z = 0.0;
  arrtemp.pose.orientation.w = 1.0;

  arrtemp.scale.x = .1;

  arrtemp.color.r = 0.0f;
  arrtemp.color.g = 1.0f;
  arrtemp.color.b = 0.5f;
  arrtemp.color.a = 1.0f;


  ROS_INFO("STATUS: READY");
  ros::spin();
}