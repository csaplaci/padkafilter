#include <stdio.h>
#include <iostream>
#include <math.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/crop_box.h>

float
    xmin = 0.0,
    ymin = -8.0,
    zmin = -5.0,
    xmax = 15.0,
    ymax = 8.0,
    zmax = 5.0;
//számításra kijelölt tartomány

float devthresh=0.025, pmin=2; //paraméterek - szórási küszöbérték és min. pontok száma

float cellnum_x=15, cellnum_y=16; //cellák száma x és y irányban
int  cellcenter_x=0, cellcenter_y=7; //középpont cellaindexei

ros::Publisher pubRoad;
ros::Publisher pubPadka;
ros::Publisher pubRaw;

void cb(const pcl::PCLPointCloud2ConstPtr &cloud)
{
    pcl::PCLPointCloud2 fCloud;

//számításra kijelölt tartomány kiválasztása, pontfelhők létrehozása

    pcl::CropBox<pcl::PCLPointCloud2> boxFilter;
    boxFilter.setMin(Eigen::Vector4f(xmin, ymin, zmin, 1.00));
    boxFilter.setMax(Eigen::Vector4f(xmax, ymax, zmax, 1.00));
    boxFilter.setInputCloud(cloud);
    boxFilter.filter(fCloud);

    pcl::PointCloud<pcl::PointXYZ> raw;
    pcl::PointCloud<pcl::PointXYZ> padka;
    pcl::PointCloud<pcl::PointXYZ> road;
    pcl::fromPCLPointCloud2(fCloud,raw);
    pcl::fromPCLPointCloud2(fCloud,road);
    pcl::fromPCLPointCloud2(fCloud,padka);

int s=raw.points.size();

std::vector<std::vector<float>>
    avg(cellnum_x,std::vector<float>(cellnum_y,0)),
    dev(cellnum_x,std::vector<float>(cellnum_y,0)),
    avgy(cellnum_x,std::vector<float>(cellnum_y-1,0)),
    devy(cellnum_x,std::vector<float>(cellnum_y-1,0));
    //átlag és szórás cellánként (+külön a köztes cellákra - csak y irányban szükséges)

std::vector<std::vector<int>>
    np(cellnum_x,std::vector<int>(cellnum_y,0)),
    npy(cellnum_x,std::vector<int>(cellnum_y-1,0));
    //pontok száma cellánként (Number of Points +köztes)

std::vector<std::vector<bool>>
    cellisroad(cellnum_x,std::vector<bool>(cellnum_y,false)),
    cellyisroad(cellnum_x,std::vector<bool>(cellnum_y-1,false)),
    cellispadka(cellnum_x,std::vector<bool>(cellnum_y,false)),
    cellyispadka(cellnum_x,std::vector<bool>(cellnum_y-1,false)),
    isvalid(cellnum_x,std::vector<bool>(cellnum_y,false)),
    isvalidy(cellnum_x,std::vector<bool>(cellnum_y-1,false));
    //változók cellánkénti ellenőrzésekhez (jelenleg kezdetleges funkciókkal)

std::vector<bool> padkapont(s,false);

//átlagszámítás (1/2)
for (int cix,ciy,ciyy, i=0;i<s;i++)
{
    cix=(int)((raw.points[i].x-xmin)/1); ciy=(int)((raw.points[i].y-ymin)/1); //cellaindex x és y irányban
    ciyy=(int)((raw.points[i].y-ymin)/1-(0.5)); //köztes cellák

    avg[cix][ciy] += raw.points[i].z;
    np[cix][ciy]++;

    if (ciyy>=0 && ciyy<cellnum_y-1)
        {
            avgy[cix][ciyy] += raw.points[i].z;
            npy[cix][ciyy]++;
        }
}
// 1 = cellaméret (todo: megadás paraméterként)

//átlagszámítás (2/2)
for (int i=0;i<cellnum_x;i++)
    for (int j=0;j<cellnum_y;j++)
        {
            if (np[i][j]>=pmin)
                {
                    isvalid[i][j]=true;
                    avg[i][j] /= np[i][j];
                }
            if (npy[i][j]>=pmin && j<cellnum_y-1)
                {
                    isvalidy[i][j]=true;
                    avgy[i][j] /= npy[i][j];
                }
        }

// szórás pontokra (1/2)
for (int cix,ciy,ciyy, i=0;i<s;i++)
    {
        cix=(int)((raw.points[i].x-xmin)/1); ciy=(int)((raw.points[i].y-ymin)/1);
        ciyy=(int)((raw.points[i].y-ymin)/1-(0.5));
        if (isvalid[cix][ciy])
            {
                dev[cix][ciy] += abs(avg[cix][ciy]-raw.points[i].z);
            }
        if (isvalidy[cix][ciyy] && ciyy>=0 && ciyy<cellnum_y-1)
            devy[cix][ciyy] += abs(avgy[cix][ciyy]-raw.points[i].z);
    }

// szórás (2/2)
for (int i=0;i<cellnum_x;i++)
    for (int j=0;j<cellnum_y;j++)
        {
            if (isvalid[i][j])
                {
                    dev[i][j] /= np[i][j];
                }
            if (isvalidy[i][j] && j<cellnum_y-1)
                {
                    devy[i][j] /= npy[i][j];
                }
        }

//kiértékelés
for (int cix,ciy,ciyy, i=0;i<s;i++)
    {
        cix=(int)((raw.points[i].x-xmin)/1); ciy=(int)((raw.points[i].y-ymin)/1);
        ciyy=(int)((raw.points[i].y-ymin)/1-(0.5));        

        if (dev[cix][ciy]>devthresh)
            {
                if (raw.points[i].z-avg[cix][ciy]>0)
                {
                    cellispadka[cix][ciy]=true;
                    padkapont[i]=true;
                    continue;
                }
            }

        if (ciyy>=0 && ciyy<cellnum_y-1)
        {
            if (devy[cix][ciyy]>devthresh)
                {
                    if (raw.points[i].z-avgy[cix][ciyy]>0)
                    {
                        cellyispadka[cix][ciyy]=true;
                        padkapont[i]=true;
                    }
                }
        }
    }

// szűrés (todo)
for (int i=0;i<s;i++)
    {
        if (padkapont[i])
            {
                road.points[i].x=0;
                road.points[i].y=0;
                road.points[i].z=0;
            }
        else
            {
                padka.points[i].x=0;
                padka.points[i].y=0;
                padka.points[i].z=0;  
            }
    }

//kiíratás (debug)
    std_msgs::String msg;
    std::stringstream ss;
    int debug_x=5,debug_y=6;
    ss << "avg: " << avg[debug_x][debug_y] << ", dev: " << dev[debug_x][debug_y] << ", np: " << np[debug_x][debug_y] << " ";
    ss << "avgy: " << avgy[debug_x][debug_y] << ", devy: " << devy[debug_x][debug_y] << ", npy: " << npy[debug_x][debug_y] << " ";
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());

    pubRoad.publish(road);
    pubPadka.publish(padka);
    pubRaw.publish(raw);

}


int main(int argc, char** argv)
{
    ros::init(argc,argv,"PadkaFilter");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/points_raw", 1, cb);
    ROS_INFO("Padkafilter node started.");
    pubRoad = nh.advertise<pcl::PCLPointCloud2>("/lidar_pf_road",1); //útnak vélt pontok
    pubPadka = nh.advertise<pcl::PCLPointCloud2>("/lidar_pf_padka",1); //padkának vélt pontok
    pubRaw = nh.advertise<pcl::PCLPointCloud2>("/lidar_pf_debug_raw",1); //nyers bemenet (debug)

    ros::spin();
    return 0;
}