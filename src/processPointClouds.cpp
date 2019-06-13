// PCL lib Functions for processing point clouds

#include "processPointClouds.h"
#include "kdtree.h"

// constructor:
template <typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds()
{
}

// de-constructor:
template <typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds()
{
}

template <typename PointT>
void
ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr
ProcessPointClouds<PointT>::Crop(
    typename pcl::PointCloud<PointT>::Ptr cloud, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{
    pcl::CropBox<PointT> crop;
    crop.setInputCloud(cloud);
    crop.setMin(minPoint);
    crop.setMax(maxPoint);
    typename pcl::PointCloud<PointT>::Ptr cloud_cropped(new pcl::PointCloud<PointT>);
    crop.filter(*cloud_cropped);
    return cloud_cropped;
}

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr
ProcessPointClouds<PointT>::RemoveCarTop(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    pcl::CropBox<PointT> crop;
    crop.setInputCloud(cloud);
    crop.setMin(Eigen::Vector4f(-2.5, -1.5, -10, 1));
    crop.setMax(Eigen::Vector4f(2.6, 1.5, 10, 1));
    typename pcl::PointCloud<PointT>::Ptr cloud_cropped(new pcl::PointCloud<PointT>);
    crop.setNegative(true);
    crop.filter(*cloud_cropped);
    return cloud_cropped;
}

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr
ProcessPointClouds<PointT>::DownSample(typename pcl::PointCloud<PointT>::Ptr cloud, float voxelSize)
{
    pcl::VoxelGrid<PointT> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(voxelSize, voxelSize, voxelSize);
    typename pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
    sor.filter(*cloud_filtered);
    return cloud_filtered;
}

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr
ProcessPointClouds<PointT>::FilterCloud(
    typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    auto cloud_cropped = Crop(cloud, minPoint, maxPoint);
    cloud_cropped      = RemoveCarTop(cloud_cropped);
    auto cloud_ds      = DownSample(cloud_cropped, filterRes);

    auto endTime     = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloud_ds;
}

template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr>
ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud)
{
    // DONE: Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr plane(new pcl::PointCloud<PointT>), rest(new pcl::PointCloud<PointT>);
    // Create the filtering object
    pcl::ExtractIndices<PointT> extract;
    // Extract the inliers
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*plane);
    // Extract the outliers
    extract.setNegative(true);
    extract.filter(*rest);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(rest, plane);
    return segResult;
}

template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr>
ProcessPointClouds<PointT>::SegmentPlane(
    typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    auto cloud_ds = DownSample(cloud, 1.0);
    // find the coeffs for the road on downsampled point cloud
    std::vector<float> bestParams = {0.0, 0.0, 1.0, 0.0};
    int mostInliers               = 0;
    size_t size                   = cloud_ds->size();

    // For max iterations
    do
    {
        // Randomly sample subset and fit line
        auto p_i               = cloud_ds->points[rand() % size];
        auto p_j               = cloud_ds->points[rand() % size];
        auto p_k               = cloud_ds->points[rand() % size];
        Eigen::Vector3f v_i    = p_i.getVector3fMap();
        Eigen::Vector3f v_j    = p_j.getVector3fMap();
        Eigen::Vector3f v_k    = p_k.getVector3fMap();
        Eigen::Vector3f v_ij   = v_j - v_i;
        Eigen::Vector3f v_ik   = v_k - v_i;
        Eigen::Vector3f normal = v_ij.cross(v_ik);
        auto d                 = normal.norm();
        if (d < 0.0000001)
            continue;
        normal /= d;
        auto f = [normal](PointT p) -> float { return normal.adjoint() * p.getVector3fMap(); };
        auto c = f(p_i);
        int n  = 0;
        // Measure distance between every point and fitted line
        for (auto p : cloud_ds->points)
        {
            float e = std::abs(f(p) - c);
            // If distance is smaller than threshold count it as inlier
            if (e < distanceThreshold)
                ++n;
        }
        if (n > mostInliers)
        {
            mostInliers   = n;
            bestParams[0] = normal[0];
            bestParams[1] = normal[1];
            bestParams[2] = normal[2];
            bestParams[3] = c;
        }
    } while (--maxIterations);

    // segment out the plane for the full point cloud:
    Eigen::Vector3f normal(bestParams[0], bestParams[1], bestParams[2]);
    float c = bestParams[3];
    auto f  = [normal, c](PointT p) -> float { return std::abs(normal.adjoint() * p.getVector3fMap() - c); };
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    for (size_t k = 0; k < cloud->size(); ++k)
    {
        auto p_k = cloud->points[k];
        auto e   = f(p_k);
        // If distance is smaller than threshold count it as inlier
        if (e < distanceThreshold)
            inliers->indices.push_back(int(k));
    }

    if (inliers->indices.size() == 0)
        std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
    auto endTime     = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult
        = SeparateClouds(inliers, cloud);
    return segResult;
}

template <typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr>
ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance)
{
    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    std::vector<std::vector<float> > points;
    for (size_t k = 0; k < cloud->size(); ++k)
    {
        auto p                      = cloud->points[k];
        std::vector<float> p_coords = {p.x, p.y, p.z};
        points.push_back(p_coords);
    }
    KdTree tree(points);
    auto midTime = std::chrono::steady_clock::now();

    std::function<int(Node*)> depth = [&depth](Node* n) -> int {
        if (n == nullptr)
            return 0;
        if (n->left == nullptr)
            return 1 + depth(n->right);
        return std::max(depth(n->left), depth(n->right)) + 1;
    };

    std::cout << "number of points in the tree: " << cloud->size() << std::endl;
    std::cout << "depth of the tree: " << depth(tree.root) << std::endl;

    std::vector<std::vector<int> > cluster_ids;

    std::vector<bool> processed(points.size(), false);

    for (size_t i = 0; i < points.size(); ++i)
    {
        if (processed[i])
            continue;
        std::vector<int> indices;
        indices.push_back(i);
        size_t k = 0;
        while (k < indices.size())
        {
            int index        = indices[k];
            processed[index] = true;
            auto neighbours  = tree.search(points[index], clusterTolerance);
            for (int n_indx : neighbours)
                if (!processed[n_indx])
                {
                    indices.push_back(n_indx);
                    processed[n_indx] = true;
                }
            ++k;
        }
        cluster_ids.push_back(std::move(indices));
    }

    for (auto cluster : cluster_ids)
    {
        pcl::ExtractIndices<PointT> extract;
        extract.setInputCloud(cloud);
        auto indices_ptr = pcl::PointIndices::Ptr(new pcl::PointIndices());
        for (auto index : cluster)
            indices_ptr->indices.push_back(index);
        extract.setIndices(indices_ptr);
        extract.setNegative(false);
        typename pcl::PointCloud<PointT>::Ptr cloud_cluster(new pcl::PointCloud<PointT>);
        extract.filter(*cloud_cluster);
        clusters.push_back(cloud_cluster);
    }

    auto endTime     = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    auto cosntrTime  = std::chrono::duration_cast<std::chrono::milliseconds>(midTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size()
              << " clusters" << std::endl;
    std::cout << "tree consrt took " << cosntrTime.count() << " milliseconds" << std::endl;

    return clusters;
}

template <typename PointT>
Box
ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
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
void
ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII(file, *cloud);
    std::cerr << "Saved " << cloud->points.size() << " data points to " + file << std::endl;
}

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr
ProcessPointClouds<PointT>::loadPcd(std::string file)
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
std::vector<boost::filesystem::path>
ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{
    std::vector<boost::filesystem::path> paths(
        boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;
}
