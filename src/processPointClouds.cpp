// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"
#include "yaml_config.h"
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>

#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>

using namespace lidar_obstacle_detection;

//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(PtCdtr<PointT> cloud) { std::cout << cloud->points.size() << std::endl; }

template<typename PointT>
std::pair<PtCdtr<PointT>, PtCdtr<PointT>>
ProcessPointClouds<PointT>::RansacSegmentPlane(PtCdtr<PointT> cloud, int maxIterations, float distanceTol) {
    #if 0
    // Count time
    auto startTime = std::chrono::steady_clock::now();
    srand(time(NULL));

    int num_points = cloud->points.size();
    auto cloud_points = cloud->points;
    Ransac<PointT> RansacSeg(maxIterations, distanceTol, num_points);

    // Get inliers from RANSAC implementation
    std::unordered_set<int> inliersResult = RansacSeg.Ransac3d(cloud);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane ransac-segment took " << elapsedTime.count() << " milliseconds" << std::endl;

    PtCdtr<PointT> out_plane(new pcl::PointCloud<PointT>());
    PtCdtr<PointT> in_plane(new pcl::PointCloud<PointT>());

    for (int i = 0; i < num_points; i++) {
        PointT pt = cloud_points[i];
        if (inliersResult.count(i)) {
            out_plane->points.push_back(pt);
        } else {
            in_plane->points.push_back(pt);
        }
    }
    return std::pair<PtCdtr<PointT>, PtCdtr<PointT>>(in_plane, out_plane);
    #else
// 使用pcl的分割，分离出地面
    // Count time
    auto startTime = std::chrono::steady_clock::now();

    // Create the segmentation object
    pcl::SACSegmentation<PointT> seg;

    // Optional
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceTol);

    // Segment the largest planar component
    // PtCdtr<PointT> cloud_plane(new pcl::PointCloud<PointT>());
    // PtCdtr<PointT> cloud_out(new pcl::PointCloud<PointT>());
    pcl::ModelCoefficients::Ptr cloud_out(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr cloud_plane(new pcl::PointIndices);

    seg.setInputCloud(cloud);
    seg.segment(*cloud_plane, *cloud_out);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane ransac-segment took " << elapsedTime.count() << " milliseconds" << std::endl;
    
    PtCdtr<PointT> in_plane(new pcl::PointCloud<PointT>());
    PtCdtr<PointT> out_plane(new pcl::PointCloud<PointT>());

    // Extract the inliers
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(cloud_plane);
    extract.setNegative(false);
    extract.filter(*in_plane);

    // Extract the outliers
    extract.setNegative(true);
    extract.filter(*out_plane);


    #endif

    bool if_view_segment = yaml_config["view"]["segment"].as<bool>();
    if (if_view_segment) {
      // 创建可视化窗口
      pcl::visualization::PCLVisualizer::Ptr viewer( new pcl::visualization::PCLVisualizer("RANSAC Plane Segmentation"));
      viewer->setWindowName("My Plane Segmentation Viewer");
      viewer->setBackgroundColor(0, 0, 0);

      // 添加坐标系
      viewer->addCoordinateSystem(1.0);
      // 添加栅格
      for (float x = -10.0; x <= 10.0; x += 1.0) {
          viewer->addLine(pcl::PointXYZ(x, -10, 0), pcl::PointXYZ(x, 10, 0),0.5,0.5,0.5,
                          std::string("grid_")+std::to_string(x)+"y");
      }
      for (float y = -10.0; y <= 10.0; y += 1.0) {
          viewer->addLine(pcl::PointXYZ(-10, y, 0), pcl::PointXYZ(10, y, 0),0.5,0.5,0.5,
                          std::string("grid_")+std::to_string(y)+"x");
      }
      // for (float x = -10.0; x <= 10.0; x += 1.0) {
      //   for (float y = -10.0; y <= 10.0; y += 1.0) {
      //     // viewer->addLine(pcl::PointXYZ(x, y, 0), pcl::PointXYZ(x, y + 1, 0),
      //     //                 0.5, 0.5, 0.5,
      //     //                 "grid_" + std::to_string(static_cast<int>(x)) + "_" +
      //     //                     std::to_string(static_cast<int>(y)));
      //     // viewer->addLine(pcl::PointXYZ(x, y, 0), pcl::PointXYZ(x + 1, y, 0),
      //     //                 0.5, 0.5, 0.5,
      //     //                 "grid_" + std::to_string(static_cast<int>(x)) + "_" +
      //     //                     std::to_string(static_cast<int>(y)));
      //   }
      // }
        
      // 可视化地面点云
      pcl::visualization::PointCloudColorHandlerCustom<PointT> plane_color( in_plane, 255, 0, 0);
      viewer->addPointCloud<PointT>(in_plane, plane_color, "plane");
      viewer->setPointCloudRenderingProperties( pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "plane");

      // 可视化非地面点云
      pcl::visualization::PointCloudColorHandlerCustom<PointT> non_plane_color( out_plane, 0, 255, 0);
      viewer->addPointCloud<PointT>(out_plane, non_plane_color, "non_plane");
      viewer->setPointCloudRenderingProperties( pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "non_plane");

      // 可视化整个点云
      // pcl::visualization::PointCloudColorHandlerCustom<PointT>
      // full_cloud_color(cloud, 255, 255, 255);
      // viewer->addPointCloud<PointT>(cloud, full_cloud_color, "full_cloud");
      // viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
      // 2, "full_cloud");

      // 显示可视化窗口
      while (!viewer->wasStopped()) {
        viewer->spinOnce();
      }
    }
    return std::make_pair(out_plane, in_plane);
}


template<typename PointT>
PtCdtr<PointT> ProcessPointClouds<PointT>::FilterCloud(PtCdtr<PointT> cloud, float filterRes, Eigen::Vector4f minPoint,
                                                       Eigen::Vector4f maxPoint) {

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    // Create the filtering object: downsample the dataset using a leaf size of .2m
    // 体素降采样
    pcl::VoxelGrid<PointT> vg;
    PtCdtr<PointT> cloudFiltered(new pcl::PointCloud<PointT>);
    vg.setInputCloud(cloud);
    vg.setLeafSize(filterRes, filterRes, filterRes);
    vg.filter(*cloudFiltered);

    // 设置roi区域
    PtCdtr<PointT> cloudRegion(new pcl::PointCloud<PointT>);
    pcl::CropBox<PointT> region(true);
    region.setMin(minPoint);
    region.setMax(maxPoint);
    region.setInputCloud(cloudFiltered);
    region.filter(*cloudRegion);

    // 去除屋顶点云
    std::vector<int> indices;
    pcl::CropBox<PointT> roof(true);
    // roof.setMin(Eigen::Vector4f(-1.5, -1.7, -1, 1));
    // roof.setMax(Eigen::Vector4f(2.6, 1.7, -0.4, 1));
    double max_x=yaml_config["roof"]["max_x"].as<double>();
    double max_y=yaml_config["roof"]["max_y"].as<double>();
    double max_z=yaml_config["roof"]["max_z"].as<double>();
    double min_x=yaml_config["roof"]["min_x"].as<double>();
    double min_y=yaml_config["roof"]["min_y"].as<double>();
    double min_z=yaml_config["roof"]["min_z"].as<double>();
    roof.setMin(Eigen::Vector4f(min_x, min_y, min_z, 1));
    roof.setMax(Eigen::Vector4f(max_x, max_y, max_z, 1));
    roof.setInputCloud(cloudRegion);
    roof.filter(indices);

    pcl::PointIndices::Ptr inliers{new pcl::PointIndices};
    for (int point : indices) {
        inliers->indices.push_back(point);
    }
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloudRegion);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloudRegion);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

//    return cloud;
    return cloudRegion;
}


template<typename PointT>
std::pair<PtCdtr<PointT>, PtCdtr<PointT>>
ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, PtCdtr<PointT> cloud) {
    // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    PtCdtr<PointT> obstCloud(new pcl::PointCloud<PointT>());
    PtCdtr<PointT> planeCloud(new pcl::PointCloud<PointT>());
    for (int index : inliers->indices) {
        planeCloud->points.push_back(cloud->points[index]);
    }
    // create extraction object
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*obstCloud);
    std::pair<PtCdtr<PointT>, PtCdtr<PointT>> segResult(obstCloud,
                                                        planeCloud);
//    std::pair<PtCdtr<PointT>, PtCdtr<PointT>> segResult(cloud, cloud);
    return segResult;
}


template<typename PointT>
std::pair<PtCdtr<PointT>, PtCdtr<PointT>>
ProcessPointClouds<PointT>::SegmentPlane(PtCdtr<PointT> cloud, int maxIterations, float distanceThreshold) {
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
//	pcl::PointIndices::Ptr inliers; // Build on the stack
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices); // Build on the heap
    // TODO:: Fill in this function to find inliers for the cloud.
    pcl::ModelCoefficients::Ptr coefficient(new pcl::ModelCoefficients);
    pcl::SACSegmentation<PointT> seg;

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);

    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficient);

    if (inliers->indices.size() == 0) {
        std::cerr << "Could not estimate a planar model for the given dataset" << std::endl;
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<PtCdtr<PointT>, PtCdtr<PointT>> segResult = SeparateClouds(
            inliers, cloud);
    return segResult;
}


template<typename PointT>
std::vector<PtCdtr<PointT>>
ProcessPointClouds<PointT>::Clustering(PtCdtr<PointT> cloud, float clusterTolerance, int minSize, int maxSize) {

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<PtCdtr<PointT>> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    // Build Kd-Tree Object
    typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    // Input obstacle point cloud to create KD-tree
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> clusterIndices; // this is point cloud indice type
    pcl::EuclideanClusterExtraction<PointT> ec; // clustering object
    ec.setClusterTolerance(clusterTolerance);
    ec.setMinClusterSize(minSize);
    ec.setMaxClusterSize(maxSize);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud); // feed point cloud
    ec.extract(clusterIndices); // get all clusters Indice

    // For each cluster indice
    for (pcl::PointIndices getIndices: clusterIndices) {
        PtCdtr<PointT> cloudCluster(new pcl::PointCloud<PointT>);
        // For each point indice in each cluster
        for (int index:getIndices.indices) {
            cloudCluster->points.push_back(cloud->points[index]);
        }
        cloudCluster->width = cloudCluster->points.size();
        cloudCluster->height = 1;
        cloudCluster->is_dense = true;
        clusters.push_back(cloudCluster);

    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size()
              << " clusters" << std::endl;

    return clusters;
}

/**
 * 使用欧式聚类对点云进行分割
 * @param input_cloud 输入点云
 * @param cluster_tolerance 聚类距离阈值(单位:米)
 * @param min_cluster_size 最小聚类尺寸
 * @param max_cluster_size 最大聚类尺寸
 * @param output_clouds 输出聚类结果点云向量
 */
template<typename PointT>
std::vector<PtCdtr<PointT>> ProcessPointClouds<PointT>::GxtEuclideanClustering(
    PtCdtr<PointT> input_cloud,
    float cluster_tolerance,
    int min_cluster_size,
    int max_cluster_size) {
    // 创建欧式聚类对象
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(cluster_tolerance);
    ec.setMinClusterSize(min_cluster_size);
    ec.setMaxClusterSize(max_cluster_size);
    ec.setSearchMethod(pcl::search::KdTree<pcl::PointXYZI>::Ptr(new pcl::search::KdTree<PointT>));

    // 提取聚类
    std::vector<pcl::PointIndices> cluster_indices;
    ec.setInputCloud(input_cloud);
    ec.extract(cluster_indices);

    std::vector<PtCdtr<PointT>> output_clouds;
    // 生成聚类结果点云
    output_clouds.clear();
    for (const auto& indices : cluster_indices)
    {
        PtCdtr<PointT> cluster(new pcl::PointCloud<PointT>);
        for (int index : indices.indices)
        {
            cluster->push_back(input_cloud->at(index));
        }
        output_clouds.push_back(cluster);
    }
    if (false) {
      // 可视化聚类结果
      pcl::visualization::PCLVisualizer viewer("Euclidean Clustering");
      viewer.setWindowName("My Plane Segmentation Viewer");
      viewer.setBackgroundColor(0, 0, 0);
      int j = 0;
      for (const auto &indices : cluster_indices) {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cluster( new pcl::PointCloud<pcl::PointXYZI>);
        for (int index : indices.indices) { cluster->push_back(input_cloud->at(index)); }
        viewer.addPointCloud<pcl::PointXYZI>(cluster, "cluster_" + std::to_string(j++)); viewer.setPointCloudRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2,
            "cluster_" + std::to_string(j - 1));
        viewer.setPointCloudRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_COLOR,
            (float)rand() / (float)RAND_MAX, (float)rand() / (float)RAND_MAX,
            (float)rand() / (float)RAND_MAX,
            "cluster_" + std::to_string(j - 1));
      }
      viewer.spin();
      // // 显示可视化窗口
      // while (!viewer.wasStopped()) {
      //   viewer.spinOnce();
      // }
    }
    return output_clouds;
}

template<typename PointT>
std::vector<PtCdtr<PointT>>
ProcessPointClouds<PointT>::EuclideanClustering(PtCdtr<PointT> cloud, float clusterTolerance, int minSize,
                                                int maxSize) {
    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    // std::vector<PtCdtr<PointT>> clusters;

    ClusterPts<PointT> clusterPoints(cloud->points.size(), clusterTolerance, minSize, maxSize);

    std::vector<PtCdtr<PointT>> clusters = clusterPoints.EuclidCluster(cloud);


    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "KDTree clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size()
              << " clusters" << std::endl;

    return clusters;
}


template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(PtCdtr<PointT> cluster) {

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(PtCdtr<PointT> cloud, std::string file) {
    pcl::io::savePCDFileASCII(file, *cloud);
    std::cerr << "Saved " << cloud->points.size() << " data points to " + file << std::endl;
}


template<typename PointT>
PtCdtr<PointT> ProcessPointClouds<PointT>::loadPcd(std::string file) {

    PtCdtr<PointT> cloud(new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT>(file, *cloud) == -1) { //* load the file
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size() << " data points from " + file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<std::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath) {

    std::vector<std::filesystem::path> paths(std::filesystem::directory_iterator{dataPath},
                                               std::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}
