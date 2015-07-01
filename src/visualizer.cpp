#include<stdio.h>

// pcl includes incase rviz visualization is not sufficient
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/pcl_base.h>
#include <pcl/impl/pcl_base.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>
#include <pcl/io/pcd_io.h>
#include <pcl/conversions.h>
#include <pcl/filters/filter.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <pcl/visualization/point_cloud_handlers.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/impl/flann_search.hpp>
#include <pcl/search/impl/kdtree.hpp>
#include <pcl/features/normal_3d.h>
#include <pcl/features/feature.h>
#include <pcl/features/impl/feature.hpp>
#include <pcl/features/normal_3d.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/filters/voxel_grid.h>

// ros includes for visualization!!!
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/ros/conversions.h>
#include <pcl/conversions.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <math.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_ros/transforms.h>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

//Mustafa
#include <pcl/filters/passthrough.h>
#include <pcl/filters/conditional_removal.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl/filters/crop_hull.h>
#include <geometry_msgs/Pose2D.h>
#include <std_msgs/String.h>

using namespace std;

int MYMAX_INPUT=0;
const int max_scans =28;
float resolution[2] = {0.1,0.2};


double timediff(const timeval& start, const timeval& stop){
    return (stop.tv_sec - start.tv_sec) + 1.0e-6 *(stop.tv_usec - start.tv_usec);
}


std::vector< pcl::PointCloud<pcl::PointXYZRGB> > global_cloud;
std::vector< pcl::PointCloud<pcl::PointXYZRGB> > global_cloud_filtered;
//sensor_msgs::ChannelFloat32 channel;
//sensor_msgs::PointCloud cloud_ros;
ros::Publisher pub_;
pcl::PointCloud<pcl::PointXYZRGB> single_global_cloud;
unsigned int min_, max_, save_pcd=0;



boost::shared_ptr<pcl::visualization::PCLVisualizer> normalsVis (
        pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud, pcl::PointCloud<pcl::Normal>::ConstPtr normals)
{
    // --------------------------------------------------------
    // -----Open 3D viewer and add point cloud and normals-----
    // --------------------------------------------------------
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    //pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZ> rgb(cloud);
    viewer->addPointCloud<pcl::PointXYZRGB> (cloud, "sample cloud");
    viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (cloud, normals, 1, 0.2, "normals");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "normals");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 1, "normals");
    //viewer->addCoordinateSystem (1.0, "global");
    viewer->initCameraParameters ();
    return (viewer);
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> simpleVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
    // --------------------------------------------
    // -----Open 3D viewer and add point cloud-----
    // --------------------------------------------
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
    //viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();
    return (viewer);
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> simpleIntensityVis (pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud)
{
    // --------------------------------------------
    // -----Open 3D viewer and add point cloud-----
    // --------------------------------------------
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> single_color(cloud,"intensity");
    //viewer->PointCloudColorHandlerGenericField<pcl::PointXYZI>(cloud, single_color, "sample cloud");
    viewer->addPointCloud<pcl::PointXYZI> (cloud, single_color, "sample cloud");
    //viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
    //viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();
    return (viewer);
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> simpleRGBVis (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
{
    // --------------------------------------------
    // -----Open 3D viewer and add point cloud-----
    // --------------------------------------------
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> single_color(cloud);
    //viewer->PointCloudColorHandlerGenericField<pcl::PointXYZI>(cloud, single_color, "sample cloud");
    viewer->addPointCloud<pcl::PointXYZRGB> (cloud, single_color, "sample cloud");
    //viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
    //viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();
    return (viewer);
}


int kbhit(void)
{
    struct termios oldt, newt;
    int ch;
    int oldf;

    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

    ch = getchar();

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);

    if(ch != EOF)
    {
        ungetc(ch, stdin);
        return 1;
    }

    return 0;
}

void get_transformation(Eigen::Matrix4f transformation_matrices[83]){
    string tline;
    fstream fid;
    int input, target;
    fid.open("/home/mustafasezer/overall.txt", ios::in);

    getline(fid, tline);

    while(!fid.eof()){

        while(tline.find("scan_")==string::npos && !fid.eof()){
            getline(fid, tline);
        }

        if(fid.eof()){
            std::cout << "Reached the end of initial guess file" << std::endl;
            fid.close();
            break;
        }

        sscanf(tline.c_str(), "scan_%d to scan_%d", &input, &target);

        if(input>MYMAX_INPUT){
            MYMAX_INPUT=input;
        }

        Eigen::Matrix4f transformation;

        for(int i=0; i<4; i++){
            getline(fid, tline);
            sscanf(tline.c_str(), "%f %f %f %f", &transformation_matrices[input-1](i,0), &transformation_matrices[input-1](i,1), &transformation_matrices[input-1](i,2), &transformation_matrices[input-1](i,3));
        }
    }
    fid.close();
}

struct building_coord{
    std::vector<pcl::PointXYZRGB> points;
    pcl::PointXYZRGB center;
};

std::vector< struct building_coord > buildings;

bool markersSaved = false;

void markerArrayCallback(const visualization_msgs::MarkerArray::ConstPtr& msg)
{
    if(markersSaved==false){
        for(int i=0; i<msg->markers.size(); i++){
            string ns(msg->markers[i].ns);
            if(ns.find("buildings_osm") != string::npos){
                /*float minx, miny, maxx, maxy;
                minx = msg->markers[i].points[0].x;
                miny = msg->markers[i].points[0].y;
                maxx = minx;
                maxy = miny;

                for(int j=1; j<msg->markers[i].points.size(); j++){
                    if(msg->markers[i].points[j].x<minx){
                        minx = msg->markers[i].points[j].x;
                    }
                    else if(msg->markers[i].points[j].x>maxx){
                        maxx = msg->markers[i].points[j].x;
                    }
                    if(msg->markers[i].points[j].y<miny){
                        miny = msg->markers[i].points[j].y;
                    }
                    else if(msg->markers[i].points[j].y>maxy){
                        maxy = msg->markers[i].points[j].y;
                    }
                }*/

                struct building_coord current_building;
                /*current_building.max_x = maxx - 691324;
                current_building.max_y = maxy - 5335829;
                current_building.min_x = minx - 691324;
                current_building.min_y = miny - 5335829;

                if(current_building.min_x > 82.5475006104 || current_building.min_y > 714.247619629 || current_building.max_x < -599.743041992 || current_building.max_y < -64.0227966309){
                    continue;
                }
                */
                for(int j=0; j<msg->markers[i].points.size(); j++){
                    pcl::PointXYZRGB p;
                    p.x = msg->markers[i].points[j].x- 691324;
                    p.y = msg->markers[i].points[j].y- 5335829;
                    p.z = msg->markers[i].points[j].z;
                    current_building.points.push_back(p);
                }
                float sum_x=0, sum_y=0;
                for(int j=0; j<current_building.points.size(); j++){
                    sum_x+=current_building.points[j].x;
                    sum_y+=current_building.points[j].y;
                }
                current_building.center.x = sum_x/current_building.points.size();
                current_building.center.y = sum_y/current_building.points.size();
                current_building.center.z = 0;
                buildings.push_back(current_building);
            }
        }
        markersSaved = true;
    }
}

float euclidianDistance(float x1, float y1, float x2, float y2){
    return sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2));
}

void mapCoordinatesCallback(const geometry_msgs::Pose2D::ConstPtr& msg)
{

    while(markersSaved == false){
        std::cout << "Waiting for markers" << std::endl;
        ros::Duration(1.0).sleep();
        ros::spinOnce();
    }

    float x = msg->x, y = msg->y;

    std::cout << x << " " << y << std::endl;

    float min_dist = 100000;
    int index = 0;
    for(int i=0; i<buildings.size(); i++){
        float dist = euclidianDistance(x, y, buildings[i].center.x, buildings[i].center.y);
        if(dist < min_dist){
            min_dist = dist;
            index = i;
        }
    }

    global_cloud_filtered.clear();

    for(unsigned int i=0;i<MYMAX_INPUT;i++){
        pcl::PointCloud<pcl::PointXYZRGB> temp_cloud;
        temp_cloud = global_cloud[i];
        std::vector<pcl::Vertices> vertices;
        pcl::Vertices vt;
        for(int j=0; j<buildings[index].points.size(); j++){
            temp_cloud.push_back(buildings[index].points[j]);
            vt.vertices.push_back(temp_cloud.points.size()-1);
        }
        vertices.push_back(vt);

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr outputCloud(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::CropHull<pcl::PointXYZRGB> cropHull;
        cropHull.setHullIndices(vertices);
        cropHull.setHullCloud(temp_cloud.makeShared());
        cropHull.setInputCloud(temp_cloud.makeShared());
        cropHull.setDim(2);
        cropHull.setCropOutside(true);

        std::vector<int> indices;
        cropHull.filter(indices);
        cropHull.filter(*outputCloud);

        std::cout << "Filtered size: " << outputCloud->points.size() << std::endl;
        global_cloud_filtered.push_back(*outputCloud);
    }

    if(save_pcd){
        single_global_cloud.clear();
    }

    sensor_msgs::PointCloud cloud_ros;
    cloud_ros.header.stamp = ros::Time::now();
    cloud_ros.header.frame_id = "/iser_transform_1";

    sensor_msgs::ChannelFloat32 channel;
    channel.name = "rgb";

    for(unsigned int i=min_;i<=max_;i++){
        if(i>=MYMAX_INPUT){
            break;
        }
        for (unsigned int j=0; j< global_cloud_filtered[i].points.size();j++){
            geometry_msgs::Point32 point;
            point.x = global_cloud_filtered[i].points[j].x;
            point.y = global_cloud_filtered[i].points[j].y;
            point.z = global_cloud_filtered[i].points[j].z;

            channel.values.push_back(global_cloud_filtered[i].points[j].rgb);
            cloud_ros.points.push_back(point);

            if(save_pcd){
                pcl::PointXYZRGB pointRGB;
                pointRGB.x = point.x;
                pointRGB.y = point.y;
                pointRGB.z = point.z;
                pointRGB.rgb = global_cloud_filtered[i].points[j].rgb;
                single_global_cloud.push_back(pointRGB);
            }
        }
    }
    cloud_ros.channels.push_back(channel);
    pub_.publish(cloud_ros);
}


int main(int argc, char** argv){

    ros::init(argc, argv, "myrmap");
    ros::NodeHandle nh("myrmap");

    //ros::Publisher pub_ = nh.advertise<sensor_msgs::PointCloud>("global_cloud",100);
    pub_ = nh.advertise<sensor_msgs::PointCloud>("global_cloud",100);
    ros::Subscriber sub_ = nh.subscribe("/visualization_marker_array", 1000, markerArrayCallback);
    ros::Subscriber osm_coord_sub_ = nh.subscribe("/osm_map_coordinates", 5, mapCoordinatesCallback);
    ros::Publisher address_pub = nh.advertise<std_msgs::String>("/osm_address", 5);

    Eigen::Matrix4f transformation[83];
    get_transformation(transformation);


    //std::vector< pcl::PointCloud<pcl::PointXYZRGB> > global_cloud;
    //std::vector< pcl::PointCloud<pcl::PointXYZRGB> > global_cloud_filtered;

    for(unsigned int i=0;i<MYMAX_INPUT;i++){
        std::stringstream ss3;
        ss3 << "scan_" << i+1;

        //ros::Publisher pub_ = nh.advertise<sensor_msgs::PointCloud>(ss3.str(),100);

        string directory = "/home/mustafasezer/Desktop/downsampled_scaled_rgbclouds/";
        std::stringstream ss;

        std::string extension("0.2");
        if(argc>1)
            extension = std::string(argv[1]);

        ss << "cloud_" << i+1 << "_" << extension << ".pcd";
        std::string pointcloud = directory + ss.str();

        std::cout << pointcloud << std::endl;
        sensor_msgs::PointCloud cloud_ros;

        std::stringstream ss2;
        ss2 << "scene_" << i+1;

        pcl::PointCloud<pcl::PointXYZRGB> cloud;
        pcl::io::loadPCDFile<pcl::PointXYZRGB> (pointcloud, cloud);


        pcl::PointCloud<pcl::PointXYZRGB> temp_cloud;
        pcl::transformPointCloud (cloud, temp_cloud, transformation[i]);

        /*while(markersSaved == false){
            std::cout << "Waiting for markers" << std::endl;
            ros::Duration(1.0).sleep();
            ros::spinOnce();
        }*/

        global_cloud.push_back(temp_cloud);
        global_cloud_filtered.push_back(temp_cloud);
    }

    while(ros::ok()){
        //unsigned int min_, max_;

        string address;

        std::cout << " Enter minimum scan: " << std::endl;
        std::cin >> min_;

        std::cout << " Enter maximum scan: " << std::endl;
        std::cin >> max_;
        getchar();

        std::cout << " Enter address: " << std::endl;
        std::getline(std::cin, address);

        bool publish_filtered = false;

        std::cout << address.length() << std::endl;

        if(address.length()>0){
            std_msgs::String address_msg;
            address_msg.data = address;
            address_pub.publish(address_msg);
            publish_filtered = true;
        }
        else{
            publish_filtered = false;
        }

        std::cout << " Would you like to save point cloud, 0 or 1?: " << std::endl;
        std::cin >> save_pcd;

        sensor_msgs::PointCloud cloud_ros;
        cloud_ros.header.stamp = ros::Time::now();
        cloud_ros.header.frame_id = "/iser_transform_1";

        sensor_msgs::ChannelFloat32 channel;
        channel.name = "rgb";

        pcl::PointCloud<pcl::PointXYZRGB> single_global_cloud;

        for(unsigned int i=min_;i<=max_;i++){
            if(i>=MYMAX_INPUT){
                break;
            }
            if(publish_filtered==false){
                for (unsigned int j=0; j< global_cloud[i].points.size();j++){
                    geometry_msgs::Point32 point;
                    point.x = global_cloud[i].points[j].x;
                    point.y = global_cloud[i].points[j].y;
                    point.z = global_cloud[i].points[j].z;

                    channel.values.push_back(global_cloud[i].points[j].rgb);
                    cloud_ros.points.push_back(point);

                    if(save_pcd){
                        pcl::PointXYZRGB pointRGB;
                        pointRGB.x = point.x;
                        pointRGB.y = point.y;
                        pointRGB.z = point.z;
                        pointRGB.rgb = global_cloud[i].points[j].rgb;
                        single_global_cloud.push_back(pointRGB);
                    }
                }
            }
            /*else{
                for (unsigned int j=0; j< global_cloud_filtered[i].points.size();j++){
                    geometry_msgs::Point32 point;
                    point.x = global_cloud_filtered[i].points[j].x;
                    point.y = global_cloud_filtered[i].points[j].y;
                    point.z = global_cloud_filtered[i].points[j].z;

                    channel.values.push_back(global_cloud_filtered[i].points[j].rgb);
                    cloud_ros.points.push_back(point);

                    if(save_pcd){
                        pcl::PointXYZRGB pointRGB;
                        pointRGB.x = point.x;
                        pointRGB.y = point.y;
                        pointRGB.z = point.z;
                        pointRGB.rgb = global_cloud_filtered[i].points[j].rgb;
                        single_global_cloud.push_back(pointRGB);
                    }
                }
            }*/
        }
        if(save_pcd){
            pcl::io::savePCDFileASCII ("single_cloud.pcd", single_global_cloud);
        }

        if(publish_filtered==false){
            cloud_ros.channels.push_back(channel);
            pub_.publish(cloud_ros);
        }

        //getchar();
        ros::spinOnce();


    }



    return 0;


}



