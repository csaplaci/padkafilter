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

const double PI=3.14159265358979323846;

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

// DEBUG
int deb_pts=0;

float
    xmin = -18.0,
    ymin = -8.0,
    zmin = -5.0,
    xmax = -3.0,
    ymax = 8.0,
    zmax = 5.0;
//számításra kijelölt tartomány

float devthresh=0.03, avgthresh=0.04, pmin=3; //paraméterek - szórási küszöbérték és cellánkénti min. pontok száma

// derékszögű koordináta-rendszer cellaparaméterei
int cellnum_x=30, cellnum_y=32; //cellák száma x és y irányba
int cellcenter_x=0, cellcenter_y=cellnum_y/2; //középpont cellaindexei
float cellxdim=(xmax-xmin)/cellnum_x, cellydim=(ymax-ymin)/cellnum_y; //cellák mérete

// poláris koordináta-rendszer cellaparaméterei
float center_x=0, center_y=0; // polárkoordinátarendszer középpontja
int cellnum_f=64, cellradius=1; // polárkoordináta-felosztás esetén körcikkek száma és radiális irányú felosztás hossza
int cellnum_r=22;
//int cellnum_r=ceil(std::max( std::max(abs(pow(center_x-xmin,2)+pow(center_y-ymin,2)), abs(pow(center_x-xmin,2)+pow(center_y-ymax,2))), std::max(abs(pow(center_x-xmax,2)+pow(center_y-ymax,2)), abs(pow(center_x-xmax,2)+pow(center_y-ymin,2))))/cellradius); // cellák maximális száma radiális irányban

bool polar=true; //polár-koordinátarendszer használata

struct node //"láncszem"
{
    int i;
    node* p;
};

struct llist //láncolt lista
{
    node* head;
    llist()
    {
        head = NULL;
    }
    int length=0;
    void add(int ind)
    {
        node* n = new node;
        length++;
        n->i = ind;
        n->p = head;
        head = n;
    }
    ~llist()
    {
        node* d = head;
        while(d!=NULL)
        {
            head = d->p;
            delete d;
            d = head;
        }
    }
};

struct holder //index és egy adat (float) tárolására alkalmas struct
{
    int i;
    float d;
    holder(int a, float b)
    {
        i=a;
        d=b;
    }
};

bool cmp(holder a, holder b) {return (a.d<b.d);} //összehasonlító függvény

bool valind(int x, int y) {/*if (!polar) return (0<=x<cellnum_x && 0<=y<cellnum_y); else */return (0<=x<cellnum_f && 0<=y<cellnum_r);} //cellaindexek érvényessége

void polarize(pcl::PointCloud<pcl::PointXYZ> *raw, std::vector<std::vector<llist*>> ind)
{
    int indi, indj, sx, sy;
    float x, y, f, r;
    for (int s=raw->points.size(),i=0;i<s;i++)
    {
        x=raw->points[i].x-center_x;
        y=raw->points[i].y-center_y;
        //
        sx=sgn(x);
        sy=sgn(y);

        if (sy>0)
        {
            if (sx>0) 
            {
                f=atan(y/x);
            }
            else
            {
                f=atan(x/y);
                f+=PI/2;
            }
        }
        else if (sy<0)
        {
            if (sx<0)
            {
                f=atan(y/x);
            }
            else
            {
                f=atan(x/y);
                f+=PI/2;
            }
            f+=PI;
        }
        else
        {
            f=0;
            if (sx<0) f+=PI;
        }
        //

        //f=atan2(y,x);
        r=(sqrt(pow(x,2)+pow(y,2)));
        indi=std::floor(f/2/PI*cellnum_f);
        indj=std::floor(r/cellradius);
        if (valind(indi,indj)) ind[indi][indj]->add(i);
    }
}


float dev(std::vector<llist*> list, pcl::PointCloud<pcl::PointXYZ> *raw, bool avg=false) //átlag és szórás a megadott cellák összesített pontjaira értelmezve
{
    float a_sum=0, devi=0;
    int num=0;
    for (int i=0;i<list.size();i++)
    {
        if (list[i]->length>=pmin)
        {
            node* avgp=list[i]->head;
            while (avgp!=NULL)
            {
                a_sum+=raw->points[avgp->i].z;
                avgp=avgp->p;
            }
            num+=list[i]->length;
        }
    }
    a_sum/=num;
    if (avg) return a_sum;
    for (int i=0;i<list.size();i++)
    {
        if (list[i]->length>=pmin)
        {
            node* devp=list[i]->head;
            while (devp!=NULL)
            {
                devi+=abs(raw->points[devp->i].z-a_sum);
                devp=devp->p;
            }
        }
    }
    devi/=num;
    return devi;
};

std::vector<int> waver(pcl::PointCloud<pcl::PointXYZ> *raw, std::vector<std::vector<llist*>> ind, int x, int y, int dir, uint max=0) //vizsgáló algoritmus
{
    dir = dir%8;
    int xdir, ydir, imax, jmax, ytox = dir%2;
   switch (dir) //megadott irány értelmezése
   {
    case 0:
       imax=cellnum_x-x;
       jmax=cellnum_y-y;
       xdir=1;
       ydir=1;
       break;
    case 1:
       imax=cellnum_y-y;
       jmax=x;
       xdir=-1;
       ydir=1;
       break;
    case 2:
       imax=x;
       jmax=y;
       xdir=-1;
       ydir=-1;
       break;
    case 3:
       imax=y;
       jmax=cellnum_x-x;
       xdir=1;
       ydir=-1;
       break;
    case 4:
       imax=cellnum_x-x;
       jmax=y;
       xdir=1;
       ydir=-1;
       break;
    case 5:
       imax=cellnum_y-y;
       jmax=cellnum_x-x;
       xdir=1;
       ydir=1;
       break;
    case 6:
       imax=x;
       jmax=cellnum_y-y;
       xdir=-1;
       ydir=1;
       break;
    case 7:
       imax=y;
       jmax=x;
       xdir=-1;
       ydir=-1;
       break;
   
   default:
       break;
   }
    if (0<max<imax) imax=max;
    std::vector<int> edge; //talált padkacellák indexei
    if (ytox) return edge;
    std::vector<llist*> passvec1(1),passvec2(2); //segédváltozók
    bool found;
    int x0,y0,x1,y1,a;
    float avg,avg0,avg1,devi,dev0,dev1;

    for (int i=0;i<imax;i++)
    {
        x0=0;
        y0=0;
        if (ytox)
        {
            y1=y+i*ydir;
            x1=0;
        }
        else
        {
            x1=x+i*xdir;
            y1=0;
        }
            passvec1[0]=ind[x0][y0];
            avg0=dev(passvec1, raw, true);
            passvec1[0]=ind[x1][y1];
            avg1=dev(passvec1, raw, true);

            //if (avg1-avg0>avgthresh) break;

        found=false;
        for (int j=0;j<jmax;j++)
        {
            x0=x1;
            y0=y1;
            if (ytox)
            {
                x1=x+j*xdir;
            }
            else
            {
                y1=y+j*ydir;
            }
            if (ind[x1][y1]->length<0 || !j) continue;
            
            passvec1[0]=ind[x0][y0];
            avg0=dev(passvec1, raw, true);
            passvec1[0]=ind[x1][y1];
            avg1=dev(passvec1, raw, true);
            passvec2[0]=ind[x0][y0];
            passvec2[1]=ind[x1][y1];
            avg=dev(passvec2, raw, true);
            passvec1[0]=ind[x0][y0];
            dev0=dev(passvec1, raw);
            passvec1[0]=ind[x1][y1];
            dev1=dev(passvec1, raw);
            passvec2[0]=ind[x0][y0];
            passvec2[1]=ind[x1][y1];
            devi=dev(passvec2, raw);

            //if (devi>devthresh)
            if (avg1-avg0>avgthresh)
            {
                /*
                std::vector<holder> vec0,vec1; //ideiglenes vektorok a láncolt lista közvetlen rendezése helyett
                node* c=ind[x0][y0]->head; //segédváltozó (struct)
                for (int f=0;f<ind[x0][y0]->length;f++) //láncolt listából vektorba
                {
                    holder tmp(c->i,raw->points[c->i].y);
                    vec0.push_back(tmp);
                    c=c->p;
                }
                std::sort(vec0.begin(),vec0.end(),cmp); //rendezés y-koordináta szerint
                
                a=ceil(vec0.size()/2);
                for (int f=0;f<a;f++) //közelebbi cella pontjainak közelebbi fele (felkerekítve)
                {
                    avg0+=raw->points[vec0[f].i].z;
                }
                avg0/=a; //átlagszámítás z-koordinátából

                c=ind[x1][y1]->head;
                for (int f=0;f<ind[x1][y1]->length;f++)
                {
                    holder tmp(c->i,raw->points[c->i].y);
                    vec1.push_back(tmp);
                    c=c->p;
                }
                std::sort(vec1.begin(),vec1.end(),cmp); //rendezés y-koordináta szerint

                a=floor(vec1.size()/2);
                for (int f=vec1.size()-1;f>a;f--) //távolabbi cella pontjainak távolabbi fele (felkerekítve)
                {
                    avg1+=raw->points[vec1[f].i].z;
                }
                avg1/=a; //átlagszámítás z-koordinátából
                
                if (avg1>avg0+devthresh) //átlagok különbsége meghaladja-e a szórást
                */
                {
                    found=true;
                    edge.push_back(j);
                    break;
                }
            }
        }
        if (!found) edge.push_back(0); // ha nem talált paadkának feltételezett cellát, 0-t ad kimenetnek
    }
   return edge;
}


ros::Publisher pubRoad;
ros::Publisher pubPadka;
ros::Publisher pubRaw;

void cb(const pcl::PCLPointCloud2ConstPtr &cloud)
{
    pcl::PCLPointCloud2 fCloud;
    pcl::PCLPointCloud2 rCloud;

    //számításra kijelölt tartomány kiválasztása, pontfelhők létrehozása

    pcl::CropBox<pcl::PCLPointCloud2> boxFilter;
    boxFilter.setMin(Eigen::Vector4f(xmin, ymin, zmin, 1.00));
    boxFilter.setMax(Eigen::Vector4f(xmax, ymax, zmax, 1.00));
    boxFilter.setInputCloud(cloud);
    boxFilter.filter(fCloud);

    pcl::CropBox<pcl::PCLPointCloud2> negFilter(true);
    negFilter.setMin(Eigen::Vector4f(xmin, ymin, zmin, 1.00));
    negFilter.setMax(Eigen::Vector4f(xmax, ymax, zmax, 1.00));
    negFilter.setInputCloud(cloud);
    negFilter.filter(rCloud);

    pcl::PointCloud<pcl::PointXYZ> raw;
    pcl::PointCloud<pcl::PointXYZ> padka;
    pcl::PointCloud<pcl::PointXYZ> road;
    pcl::fromPCLPointCloud2(rCloud,raw);
    pcl::fromPCLPointCloud2(fCloud,road);
    pcl::fromPCLPointCloud2(fCloud,padka);

    int s=raw.points.size();

    std::vector<std::vector<llist>> ind(cellnum_x,std::vector<llist>(cellnum_y));
    //láncolt lista cellánként - a cellához tartozó pontok indexei

    std::vector<std::vector<llist>> pind(cellnum_f,std::vector<llist>(cellnum_r));
    // polárkoordináta-rendszer szerinti indexek

    std::vector<std::vector<llist*>> indpass;
    std::vector<llist*> row;
    for (int i=0;i<cellnum_f;i++)
    {
        for (int j=0;j<cellnum_r;j++)
        {
            //row.push_back(&ind[i][j]);
            row.push_back(&pind[i][j]);
        }
        indpass.push_back(row);
        row.clear();
    }

    std::vector<std::vector<bool>>
        //checked(cellnum_x,std::vector<bool>(cellnum_y,false)),
        //cellisroad(cellnum_x,std::vector<bool>(cellnum_y,false)),
        cellispadka(cellnum_x,std::vector<bool>(cellnum_y,false)),
        isvalid(cellnum_x,std::vector<bool>(cellnum_y,false));
    //változók cellánkénti ellenőrzésekhez
    std::vector<std::vector<std::vector<bool>>> cellisroad(cellnum_x,std::vector<std::vector<bool>>(cellnum_y,std::vector<bool>(2,false)));

    std::vector<bool> padkapont(s,false); //padka flag (per pont)


//indexelés
    for (int cix,ciy,ciyy, i=0;i<s;i++)
    {
        //cix=(int)((raw.points[i].x-xmin)/cellxdim); ciy=(int)((raw.points[i].y-ymin)/cellydim); //fordított előjel esetén
        cix=(int)((xmax-raw.points[i].x)/cellxdim); ciy=(int)((ymax-raw.points[i].y)/cellydim);
        ind[cix][ciy].add(i);        //pontok cellákba sorolása
    }

// cellák felvétele poláris koordinátarendszerben
    polarize(&raw,indpass);

/*
    std::vector<int> test;
    for (int i=0;i<8;i++)
    {
        int x=5,y=cellcenter_y;
        test=waver(&raw,indpass,x,y,i);
        for (int j=0;j<test.size();j++)
        {
            if (test[j])
            {
                switch (i)
                {
                case 0:
                    for (int k=0;k<test[j];k++)
                    {
                        cellisroad[x+j][y+k][i%2]=true;
                    }
                    cellispadka[x+j][y+test[j]]=true;
                    break;
                case 1:
                    for (int k=0;k<test[j];k++)
                    {
                        cellisroad[x-k][y+j][i%2]=true;
                    }
                    cellispadka[x-test[j]][y+j]=true;
                    break;
                case 2:
                    for (int k=0;k<test[j];k++)
                    {
                        cellisroad[x-j][y-k][i%2]=true;
                    }
                    cellispadka[x-j][y-test[j]]=true;
                    break;
                case 3:
                    for (int k=0;k<test[j];k++)
                    {
                        cellisroad[x+k][y-j][i%2]=true;
                    }
                    cellispadka[x+test[j]][y-j]=true;
                    break;
                case 4:
                    for (int k=0;k<test[j];k++)
                    {
                        cellisroad[x+j][y-k][i%2]=true;
                    }
                    cellispadka[x+j][y-test[j]]=true;
                    break;
                case 5:
                    for (int k=0;k<test[j];k++)
                    {
                        cellisroad[x+k][y+j][i%2]=true;
                    }
                    cellispadka[x+test[j]][y+j]=true;
                    break;
                case 6:
                    for (int k=0;k<test[j];k++)
                    {
                        cellisroad[x-j][y+k][i%2]=true;
                    }
                    cellispadka[x-j][y+test[j]]=true;
                    break;
                case 7:
                    for (int k=0;k<test[j];k++)
                    {
                        cellisroad[x-k][y-j][i%2]=true;
                    }
                    cellispadka[x-test[j]][y-j]=true;
                    break;
                
                default:
                    break;
                }
            }        
        }
    }

    //kiértékelés
    for (int cix,ciy, i=0;i<s;i++)
    {
        //cix=(int)((raw.points[i].x-xmin)/cellxdim); ciy=(int)((raw.points[i].y-ymin)/cellydim); //fordított előjel esetén
        cix=(int)((xmax-raw.points[i].x)/cellxdim); ciy=(int)((ymax-raw.points[i].y)/cellydim);

        if /*(cellispadka[cix][ciy]) // ((!cellisroad[cix][ciy][0]) && (!cellisroad[cix][ciy][1]))
        {
            padkapont[i]=true;
            continue;
        }
    }
    */

    // kiértékelés
    {
        deb_pts=0;
        node* n;
        for (int cif=0;cif<cellnum_f;cif+=2)
        {
            for (int cir=0;cir<cellnum_r;cir++)
            {
                n=pind[cif][cir].head;
                while (n!=NULL)
                {
                    padkapont[n->i]=true;
                    n=n->p;
                    deb_pts++;
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

    ss << " [debug] ";

    ss << deb_pts << " / " << s;

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
    ros::Subscriber sub = nh.subscribe("/right_os1/os1_cloud_node/points", 1, cb);
    ROS_INFO("Padkafilter node started.");
    pubRoad = nh.advertise<pcl::PCLPointCloud2>("/lidar_pf_road",1); //útnak vélt pontok
    pubPadka = nh.advertise<pcl::PCLPointCloud2>("/lidar_pf_padka",1); //padkának vélt pontok
    pubRaw = nh.advertise<pcl::PCLPointCloud2>("/lidar_pf_debug_raw",1); //bemenet detektáláson kívül eső része (debug)

    ros::spin();
    return 0;
}