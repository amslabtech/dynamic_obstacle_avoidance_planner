#include "dynamic_obstacle_avoidance_planner/static_local_costmap_generator.h"

StaticLocalCostmapGenerator::StaticLocalCostmapGenerator(void)
:local_nh("~")
{
    local_nh.getParam("/dynamic_avoidance/ROBOT_FRAME", ROBOT_FRAME);
    local_nh.getParam("/dynamic_avoidance/WORLD_FRAME", WORLD_FRAME);
    local_nh.getParam("/dynamic_avoidance/RADIUS", RADIUS);
    local_nh.param("RADIUS", RADIUS, {0.6});
    local_nh.param("HZ", HZ, {10});
    local_nh.param("MAP_WIDTH", MAP_WIDTH, {20.0});
    local_nh.param("RESOLUTION", RESOLUTION, {0.1});

    map_pub = nh.advertise<nav_msgs::OccupancyGrid>("/local_costmap", 1);
    cloud_sub = nh.subscribe("/cluster/human/removed", 1, &StaticLocalCostmapGenerator::cloud_callback, this);
    cloud_ptr = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    cloud_updated = false;

    std::cout << "=== local_costmap ===" << std::endl;
    std::cout << "RADIUS: " << RADIUS  << std::endl;
    std::cout << "ROBOT_FRAME: " << ROBOT_FRAME  << std::endl;
    std::cout << "WORLD_FRAME: " << WORLD_FRAME  << std::endl;
    std::cout << "HZ: " << HZ  << std::endl;
    std::cout << "MAP_WIDTH: " << MAP_WIDTH  << std::endl;
    std::cout << "RESOLUTION: " << RESOLUTION  << std::endl;
}

void StaticLocalCostmapGenerator::process(void)
{
    ros::Rate loop_rate(HZ);

    local_costmap.header.frame_id = ROBOT_FRAME;
    local_costmap.info.resolution = RESOLUTION;
    local_costmap.info.width = MAP_WIDTH / RESOLUTION;
    local_costmap.info.height = MAP_WIDTH / RESOLUTION;
    local_costmap.info.origin.position.x = -MAP_WIDTH / 2.0;
    local_costmap.info.origin.position.y = -MAP_WIDTH / 2.0;
    local_costmap.info.origin.orientation = tf::createQuaternionMsgFromYaw(0.0);
    local_costmap.data.resize(local_costmap.info.width * local_costmap.info.height);
    RADIUS_GRID = RADIUS / local_costmap.info.resolution;
    for(unsigned int i=0;i<local_costmap.info.height;i++){
        for(unsigned int j=0;j<local_costmap.info.width;j++){
            local_costmap.data[i*local_costmap.info.width+j] = 0;
        }
    }
    const int MAP_SIZE = local_costmap.data.size();

    while(ros::ok()){
        std::cout << "=== local_costmap ===" << std::endl;
        double start = ros::Time::now().toSec();
        if(cloud_updated){
            for(unsigned int i=0;i<local_costmap.info.height;i++){
                for(unsigned int j=0;j<local_costmap.info.width;j++){
                    local_costmap.data[i*local_costmap.info.width+j] = 0;
                }
            }
            pcl_ros::transformPointCloud(local_costmap.header.frame_id, *cloud_ptr, *cloud_ptr, listener);
            int cloud_size = cloud_ptr->points.size();
            std::cout << "cloud size: " << cloud_size << std::endl;
            std::cout << "MAP_SIZE" << MAP_SIZE << std::endl;
            double map_max_limit_x = local_costmap.info.resolution * local_costmap.info.height * 0.5;
            double map_min_limit_x = -local_costmap.info.resolution * local_costmap.info.height * 0.5;
            double map_max_limit_y = local_costmap.info.resolution * local_costmap.info.width * 0.5;
            double map_min_limit_y = -local_costmap.info.resolution * local_costmap.info.width * 0.5;
            for(int i=0;i<cloud_size;i++){
                if(cloud_ptr->points[i].x < map_max_limit_x && cloud_ptr->points[i].x > map_min_limit_x && cloud_ptr->points[i].y < map_max_limit_y && cloud_ptr->points[i].y > map_min_limit_y){
                    int index = get_index(cloud_ptr->points[i].x, cloud_ptr->points[i].y);
                    if(index < MAP_SIZE){
                        local_costmap.data[index] = 100;
                    }
                }
            }
            nav_msgs::OccupancyGrid expanded_costmap = local_costmap;
            for(int i=0;i<MAP_SIZE;i++){
                if(local_costmap.data[i] > 0){
                    int _j = i % local_costmap.info.height;
                    int _k = i / local_costmap.info.height;
                    for(unsigned int j=0;j<local_costmap.info.height;j++){
                        for(unsigned int k=0;k<local_costmap.info.width;k++){
                            int dj = _j - j;
                            int dk = _k - k;
                            if(abs(dj) <= RADIUS_GRID && abs(dk) <= RADIUS_GRID){
                                if(sqrt(dj*dj+dk*dk) <= RADIUS_GRID){
                                    expanded_costmap.data[k * local_costmap.info.height + j] = 100;
                                }
                            }
                        }
                    }
                }
            }
            map_pub.publish(expanded_costmap);
            cloud_updated = false;
        }
        std::cout << "time: " << ros::Time::now().toSec() - start << "[s]" << std::endl;
        ros::spinOnce();
        loop_rate.sleep();
    }
}

void StaticLocalCostmapGenerator::cloud_callback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    sensor_msgs::PointCloud2 _cloud;
    _cloud = *msg;
    pcl::fromROSMsg(_cloud, *cloud_ptr);
    local_costmap.header.stamp = _cloud.header.stamp;
    cloud_updated = true;
}

inline int StaticLocalCostmapGenerator::get_i_from_x(double x)
{
    return floor((x - local_costmap.info.origin.position.x) / local_costmap.info.resolution + 0.5);
}

inline int StaticLocalCostmapGenerator::get_j_from_y(double y)
{
    return floor((y - local_costmap.info.origin.position.y) / local_costmap.info.resolution + 0.5);
}

inline int StaticLocalCostmapGenerator::get_index(double x, double y)
{
    return local_costmap.info.width * get_j_from_y(y) + get_i_from_x(x);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "static_local_costmap_generator");
    StaticLocalCostmapGenerator static_local_costmap_generator;
    static_local_costmap_generator.process();
    return 0;
}

