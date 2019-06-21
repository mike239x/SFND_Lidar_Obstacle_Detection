/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

std::vector<Car>
initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    Car egoCar(Vect3(0, 0, 0), Vect3(4, 2, 2), Color(0, 1, 0), "egoCar");
    Car car1(Vect3(15, 0, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car1");
    Car car2(Vect3(8, -4, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car2");
    Car car3(Vect3(-12, 4, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car3");

    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if (renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}

void
simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------

    // RENDER OPTIONS
    bool renderScene      = false; // render cars and road
    std::vector<Car> cars = initHighway(renderScene, viewer);

    // DONE:: Create lidar sensor
    Lidar lidar(cars, 0.0);
    auto scan = lidar.scan(); // pcl::PointCloud<pcl::PointXYZ>::Ptr
    // renderRays(viewer, lidar->position, scan);
    // renderPointCloud(viewer, scan, "lidar scan", Color(1,0.5,0.5)); // nice rose color ^.^

    // DONE:: Create point processor
    ProcessPointClouds<pcl::PointXYZ> pointProcessor;
    auto seg = pointProcessor.SegmentPlane(scan, 100, 0.2);
    // renderPointCloud(viewer, seg.first, "rest", Color(1,0.5,0.5));
    renderPointCloud(viewer, seg.second, "plane", Color(0.5, 0.5, 0.5));

    auto cloudClusters = pointProcessor.Clustering(seg.first, 1.0);

    int clusterId = 0;

    std::vector<Color> colors = {Color(1, 0.5, 0.5), Color(0.5, 1, 0.5), Color(0.5, 0.5, 1)};

    for (auto cluster : cloudClusters)
    {
        auto color = colors[clusterId % colors.size()];
        renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId), color);
        Box box = pointProcessor.BoundingBox(cluster);
        renderBox(viewer, box, clusterId, color);
        ++clusterId;
    }
}

void
cityBlock(
    pcl::visualization::PCLVisualizer::Ptr& viewer,
    ProcessPointClouds<pcl::PointXYZI>& pointProcessor,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display City Block     -----
    // ----------------------------------------------------

    float voxel_size = 0.2;
    auto cloud       = pointProcessor.FilterCloud(
        inputCloud, voxel_size, Eigen::Vector4f(-20, -5, -100, 1), Eigen::Vector4f(20, 5, 100, 1));
    auto seg   = pointProcessor.SegmentPlane(cloud, 200, 0.3);
    auto rest  = seg.first;
    auto plane = seg.second;

    renderPointCloud(viewer, plane, "plane", Color(0.2, 0.2, 0.2));

    auto clusters = pointProcessor.Clustering(seg.first, voxel_size * 2.3);

    std::vector<Color> colors = {Color(1, 0.5, 0.5), Color(0.5, 1, 0.5), Color(0.5, 0.5, 1)};

    int clusterId = 0;
    for (auto cluster : clusters)
    {
        auto color = colors[clusterId % colors.size()];
        if (cluster->size() < 600 && cluster->size() > 10) // color & box reasonably sized clusters
        {
            renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId), color);
            Box box = pointProcessor.BoundingBox(cluster);
            renderBox(viewer, box, clusterId, color, 0.3);
        }
        else // draw in white everything else
        {
            renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId), Color(1, 1, 1));
        }
        ++clusterId;
    }
}

// setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void
initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    viewer->setBackgroundColor(0, 0, 0);

    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;

    switch (setAngle)
    {
        case XY: viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown: viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side: viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS: viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if (setAngle != FPS)
        viewer->addCoordinateSystem(1.0);
}

int
main(int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
    // simpleHighway(viewer);

    ProcessPointClouds<pcl::PointXYZI> pointProcessor;
    std::vector<boost::filesystem::path> stream = pointProcessor.streamPcd("../src/sensors/data/pcd/data_1");
    auto streamIterator                         = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud;
    while (!viewer->wasStopped())
    {
        // Clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();
        // Load pcd and run obstacle detection process
        inputCloud = pointProcessor.loadPcd((*streamIterator).string());
        cityBlock(viewer, pointProcessor, inputCloud);
        streamIterator++;
        if (streamIterator == stream.end())
            streamIterator = stream.begin();
        viewer->spinOnce();
    }
}
