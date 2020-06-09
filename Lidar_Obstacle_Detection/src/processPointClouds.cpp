// PCL lib Functions for processing point clouds

#include "processPointClouds.h"

#include <unordered_set>
//constructor:
template <typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}

//de-constructor:
template <typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}

template <typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    typename pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
    pcl::VoxelGrid<PointT> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(filterRes, filterRes, filterRes);
    sor.filter(*cloud_filtered);

    typename pcl::PointCloud<PointT>::Ptr cloudRegion(new pcl::PointCloud<PointT>);

    pcl::CropBox<PointT> region(true);
    region.setMin(minPoint);
    region.setMax(maxPoint);
    region.setInputCloud(cloud_filtered);
    region.filter(*cloudRegion);

    std::vector<int> indices;

    pcl::CropBox<PointT> roof(true);
    roof.setMin(Eigen::Vector4f(-1.5, -1.7, -1, 1));
    roof.setMax(Eigen::Vector4f(2.6, 1.7, -.4, 1));
    roof.setInputCloud(cloudRegion);
    roof.filter(indices);

    pcl::PointIndices::Ptr inliers{new pcl::PointIndices};

    for (int point : indices)
    {
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

    return cloudRegion;
}

template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud)
{
    // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr obstacles(new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr road(new pcl::PointCloud<PointT>());

    auto size = inliers->indices.size();
    for (int index : inliers->indices)
    {
        road->points.push_back(cloud->points[index]);
    }

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud); //reference cloud
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*obstacles);
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstacles, road); // inlier cloud sepertaed
    return segResult;
}

template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    // TODO:: Fill in this function to find inliers for the cloud.

    pcl::SACSegmentation<PointT> seg;

    pcl::ModelCoefficients::Ptr coefficients{new pcl::ModelCoefficients};

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);

    //Segment the largest planar component from the input cloud
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);
    if (inliers->indices.size() == 0)
    { //indices we found on plane by doing RANSAC
        std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
    }
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers, cloud);
    return segResult;
}

template <typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(clusterTolerance); // 2cm
    ec.setMinClusterSize(minSize);
    ec.setMaxClusterSize(maxSize);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    int j = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
    {
        typename pcl::PointCloud<PointT>::Ptr cloud_cluster(new pcl::PointCloud<PointT>);
        for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
            cloud_cluster->points.push_back(cloud->points[*pit]); //*
        cloud_cluster->width = cloud_cluster->points.size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;
        clusters.push_back(cloud_cluster);

        std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size() << " data points." << std::endl;
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}

template <typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

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

template <typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII(file, *cloud);
    std::cerr << "Saved " << cloud->points.size() << " data points to " + file << std::endl;
}

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT>(file, *cloud) == -1) //* load the file
    {
        PCL_ERROR("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size() << " data points from " + file << std::endl;

    return cloud;
}

template <typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;
}
//My Segmentation and Clustering algorithms
template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::myRANSAC(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
    std::unordered_set<int> inliersResult;

    //pcl::PointIndices::Ptr inliers(new pcl::PointIndices())

    srand(time(NULL));

    // TODO: Fill in this function

    // For max iterations

    // Randomly sample subset and fit line

    // Measure distance between every point and fitted line
    // If distance is smaller than threshold count it as inlier

    // Return indicies of inliers from fitted line with most inliers

    while (maxIterations--)
    {
        //Randeomly pick two pointa

        std::unordered_set<int> inliers;
        while (inliers.size() < 3)
        {
            inliers.insert(rand() % (cloud->points.size())); // randomly generates number between 0 and points.size()
        }

        float x1, y1, z1, x2, y2, z2, x3,y3,z3;

        auto itr = inliers.begin();
        x1 = cloud->points[*itr].x;
        y1 = cloud->points[*itr].y;
        z1 = cloud->points[*itr].z;
        itr++;
        x2 = cloud->points[*itr].x;
        y2 = cloud->points[*itr].y;
        z2 = cloud->points[*itr].z;
        itr++;
        x3 = cloud->points[*itr].x;
        y3 = cloud->points[*itr].y;
        z3 = cloud->points[*itr].z;

        float a = static_cast<float>(((y2 - y1) * (z3 - z1)) - ((z2 - z1) * (y3 - y1)));
        float b = static_cast<float>(((z2 - z1) * (x3 - x1)) - ((x2 - x1) * (z3 - z1)));
        float c = static_cast<float>(((x2 - x1) * (y3 - y1)) - ((y2 - y1) * (x3 - x1)));
        float d = static_cast<float>(-((a * x1) + (b * y1) + (c * z1)));

        for (int index = 0; index < cloud->points.size(); index++)
        {

            // if(inliers.count(index)>0){ //checking if the points are already on the line
            // 	continue;
            // }

            PointT point = cloud->points[index];
            float x4 = point.x;
            float y4 = point.y;
            float z4 = point.z;

            float dist = (std::fabs(a * x4 + b * y4 + c * z4 + d)) / (std::sqrt((a * a) + (b * b) + (c * c)));

            if (dist <= distanceTol)
            {
                inliers.insert(index);
            }
        }
        if (inliers.size() > inliersResult.size())
        {
            inliersResult = inliers;
        }
    }

    typename pcl::PointCloud<PointT>::Ptr cloudInliers(new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr cloudOutliers(new pcl::PointCloud<PointT>());

    if (!inliersResult.empty())
    {
        for (int index = 0; index < cloud->points.size(); index++)
        {
            PointT point = cloud->points[index];
            if (inliersResult.count(index))
                cloudInliers->points.push_back(point);
            else
                cloudOutliers->points.push_back(point);
        }
    }
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> RANSAC_result(cloudOutliers, cloudInliers); // inlier cloud sepertaed
    return RANSAC_result;
}

// template <typename PointT>
// void ProcessPointClouds<PointT>::Proximity(int i, typename pcl::PointCloud<PointT>::Ptr cloud, std::vector<int> &cluster, std::vector<bool> &processed, KdTree *tree, float distanceTol)
// {

//     processed[i] = true;
//     cluster.push_back(i);

//     std::vector<int> nearest = tree->search(cloud->points[i], distanceTol);

//     for (int id : nearest)
//     {
//         if (!processed[id])
//         {
//             Proximity(id, cloud, cluster, processed, tree, distanceTol);
//         }
//     }
// }
template <typename PointT>
void ProcessPointClouds<PointT>::myProximity(int i, const std::vector<std::vector<float>> &points, std::vector<int> & cluster,std::vector<bool> &processed,KdTree* tree,float distanceTol) {

	processed[i] = true;
	cluster.push_back(i);

	std::vector<int> nearest = tree->search(points[i],distanceTol);

	for(int id : nearest){
		if(!processed[id]){
			myProximity(id, points, cluster, processed, tree, distanceTol);
		}
	}
}

template <typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr>
ProcessPointClouds<PointT>::myClustering(typename pcl::PointCloud<PointT>::Ptr cloud,const float distanceTol, const int minSize, const int maxSize){

    // TODO: Fill out this function to return list of indices for each cluster

    //std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
    const auto startTime{ std::chrono::steady_clock::now() };

    int i{ 0 };
    KdTree *tree{ new KdTree() };

    std::vector<std::vector<float>> points;

    for (auto point : cloud->points) {
        const std::vector<float> p{ point.x, point.y, point.z };
        tree->insert(p, i++);
        points.push_back(p);
    }

    std::vector<std::vector<int>>listofIndices;
    std::vector<bool> processed(cloud->points.size(), false);
    int j = 0;
    while (j < cloud->points.size())
    {
        if (processed[j])
        {
            j++;
            continue;
        }
        std::vector<int> cluster;
        myProximity(j, points, cluster,processed,tree,distanceTol);
        listofIndices.push_back(cluster);
        j++;
    }

  std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    for (auto indices : listofIndices) {

        if (indices.size() < minSize || indices.size() > maxSize) { continue; }

        typename pcl::PointCloud<PointT>::Ptr cluster{ new pcl::PointCloud<PointT> };

        for (auto index : indices) { cluster->points.push_back(cloud->points[index]); }

        cluster->width = cluster->points.size();
        cluster->height = 1;
        cluster->is_dense = true;

        clusters.push_back(cluster);
    }

    const auto endTime{ std::chrono::steady_clock::now() };
    const auto elapsedTime{ std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime) };

    std::cout << "clustering took " << elapsedTime.count();
    std::cout << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}

// template <typename PointT>
// std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::euclideanCluster(typename pcl::PointCloud<PointT>::Ptr cloud, KdTree *tree, float distanceTol, int minSize, int maxSize)
// {

//     // TODO: Fill out this function to return list of indices for each cluster

//     std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
//     std::vector<bool> processed(cloud->points.size(), false);
//     int i = 0;
//     while (i < cloud->points.size())
//     {
//         if (processed[i])
//         {
//             i++;
//             continue;
//         }
//         std::vector<int> cluster;
//         Proximity(i, cloud, cluster, processed, tree, distanceTol);
//         typename pcl::PointCloud<PointT>::Ptr cloud_cluster(new pcl::PointCloud<PointT>);
//         if(cluster.size()>=minSize && cluster.size()<=maxSize){ //Added check on the advice of reviewer
//         for (const int &v : cluster)
//         {
            
//             cloud_cluster->points.push_back(cloud->points[v]); //*
//             cloud_cluster->width = cloud_cluster->points.size();
//             cloud_cluster->height = 1;
//             cloud_cluster->is_dense = true;
//             clusters.push_back(cloud_cluster);
        
//         }
//         }

//         i++;
//     }
//     //std::cout<<"Found "<<clusters.size()<<" clusters using my euclidean clustering function"<<std::endl;
//     return clusters;
// }
//----------------------------------------------------------------------//
