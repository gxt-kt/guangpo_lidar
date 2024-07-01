//
// Created by hyin on 2020/3/25.
//

// for exploring self-driving car sensors

#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

#include "yaml_config.h"

#include "obstacles.hpp"

using namespace lidar_obstacle_detection;


// Test read Lidar data
void cityBlock(pcl::visualization::PCLVisualizer::Ptr &viewer, ProcessPointClouds<pcl::PointXYZI> *pointProcessorI,
               const pcl::PointCloud<pcl::PointXYZI>::Ptr &inputCloud) {
    // ----------------------------------------------------
    // -----Open 3D viewer and display City Block     -----
    // ----------------------------------------------------

    // Setting hyper parameters

    // FilterCloud
    // float filterRes = 0.4;
    float filterRes=yaml_config["filterRes"].as<float>();
    // Eigen::Vector4f minpoint(-10, -6.5, -2, 1);
    // Eigen::Vector4f maxpoint(30, 6.5, 1, 1);
    double max_x=yaml_config["roi"]["max_x"].as<double>();
    double max_y=yaml_config["roi"]["max_y"].as<double>();
    double max_z=yaml_config["roi"]["max_z"].as<double>();
    double min_x=yaml_config["roi"]["min_x"].as<double>();
    double min_y=yaml_config["roi"]["min_y"].as<double>();
    double min_z=yaml_config["roi"]["min_z"].as<double>();
    Eigen::Vector4f maxpoint(max_x,max_y,max_z,1);
    Eigen::Vector4f minpoint(min_x,min_y,min_z,1);
    // SegmentPlane
    // int maxIterations = 40;
    int maxIterations=yaml_config["ransac"]["maxIterations"].as<int>();
    // float distanceThreshold = 0.3;
    float distanceThreshold=yaml_config["ransac"]["distanceThreshold"].as<float>();
    // 聚类参数设置
    // Clustering
    // float clusterTolerance = 0.5;
    // int minsize = 10;
    // int maxsize = 140;
    float clusterTolerance = yaml_config["cluster"]["clusterTolerance"].as<float>();
    int minsize = yaml_config["cluster"]["minsize"].as<int>();
    int maxsize = yaml_config["cluster"]["maxsize"].as<int>();

    // 1. 降采样+设置roi区域+去除车辆相关点云
    // First:Filter cloud to reduce amount of points
    pcl::PointCloud<pcl::PointXYZI>::Ptr filteredCloud = pointProcessorI->FilterCloud(inputCloud, filterRes, minpoint,
                                                                                      maxpoint);
    // 2. 把点云分离出路面
    // Second: Segment the filtered cloud into obstacles and road
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessorI->RansacSegmentPlane(
            filteredCloud, maxIterations, distanceThreshold);
//    renderPointCloud(viewer, segmentCloud.first, "obstCloud", Color(1, 0, 0));
    renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(0, 1, 0));
//    renderPointCloud(viewer,inputCloud,"inputCloud");
    // Third: Cluster different obstacle cloud
    // std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->EuclideanClustering(segmentCloud.first,
    //                                                                                               clusterTolerance,
    //                                                                                               minsize, maxsize);
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->GxtEuclideanClustering(segmentCloud.first,
                                                                                                  clusterTolerance,
                                                                                                  minsize, maxsize);

    std::vector<Eigen::Vector3f> centroids;
    std::vector<Eigen::Vector3f> sizes;
    std::vector<Eigen::Quaternionf> orientations;
    
    std::cout << "begin ExtractObstacles" << std::endl;
    ExtractObstacles(cloudClusters,centroids,sizes,orientations);
    visualizeObstacles(cloudClusters,centroids,sizes,orientations);
    
    int clusterId = 0;
    std::vector<Color> colors = {Color(1, 0, 0), Color(0, 1, 0), Color(0, 0, 1)};

    for (pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters) {

        std::cout << "cluster size";
        pointProcessorI->numPoints(cluster);
        renderPointCloud(viewer, cluster, "obstCLoud" + std::to_string(clusterId),
                         colors[clusterId % colors.size()]);
        // Fourth: Find bounding boxes for each obstacle cluster
        Box box = pointProcessorI->BoundingBox(cluster);
        renderBox(viewer, box, clusterId);
        ++clusterId;

    }

}


//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr &viewer) {

    viewer->setBackgroundColor(0, 0, 0);

    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;

    switch (setAngle) {
        case XY :
            viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0);
            break;
        case TopDown :
            viewer->setCameraPosition(0, 0, distance, 1, 0, 1);
            break;
        case Side :
            viewer->setCameraPosition(0, -distance, 0, 0, 0, 1);
            break;
        case FPS :
            viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if (setAngle != FPS)
        viewer->addCoordinateSystem(1.0);
}

// 计算旋转矩阵  
Eigen::Matrix4d TransforMatrix(Eigen::Vector3d translation,double roll,double pitch,double yaw) {
    Eigen::Quaterniond q = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())  
                         * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())  
                         * Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());  
    Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity(); // 初始化为单位矩阵  
    transformation_matrix.block<3, 3>(0, 0) = q.toRotationMatrix(); // 设置旋转矩阵部分  
    transformation_matrix.block<3, 1>(0, 3) = translation; // 设置平移向量部分  
    std::cout << "旋转矩阵：" << std::endl << transformation_matrix << std::endl;  
    return transformation_matrix;
}

// char* argv[] means array of char pointers, whereas char** argv means pointer to a char pointer.
int main(int argc, char **argv) {
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);

// For simpleHighway function
//    simpleHighway(viewer);
//    cityBlock(viewer);
//    while (!viewer->wasStopped ())
//    {
//     viewer->spinOnce ();
//    }
//
    std::string pcd_files_path = "src/sensors/data/pcd/data_1";
    if (argc >= 2) {
      pcd_files_path = argv[1];
    }
    std::cout << "Read pcd file path: " << pcd_files_path << std::endl;

//  Stream cityBlock function
    ProcessPointClouds<pcl::PointXYZI> *pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    std::vector<std::filesystem::path> stream = pointProcessorI->streamPcd(pcd_files_path);
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;

    Eigen::Vector3d trans;
    trans<<0.3643,0.2078,1.21;
    Eigen::Matrix4d tran_matrix=TransforMatrix(trans,-179.5*M_PI/180.0,0.5*M_PI/180.0,360*M_PI/180.0);
    
    Eigen::Affine3d affine3d_transform(tran_matrix); // 转换为Affine3d类型  
    Eigen::Affine3f affine3f_transform = affine3d_transform.cast<float>(); // 转换为Affine3f类型
    affine3f_transform=affine3f_transform.inverse();
    // 


    while (!viewer->wasStopped()) {
        // Clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();
        // Load pcd and run obstacle detection process
        inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
        // 坐标系转换
        pcl::transformPointCloud(*inputCloudI, *inputCloudI, affine3f_transform);
        cityBlock(viewer, pointProcessorI, inputCloudI);
        streamIterator++;
        if (streamIterator == stream.end()) {
            streamIterator = stream.begin();
        }
        viewer->spinOnce();
    }
}
