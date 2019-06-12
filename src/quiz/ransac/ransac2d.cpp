/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	// Add inliers
  	float scatter = 0.6;
  	for(int i = -5; i < 5; i++)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = i+scatter*rx;
  		point.y = i+scatter*ry;
  		point.z = 0;

  		cloud->points.push_back(point);
  	}
  	// Add outliers
  	int numOutliers = 10;
  	while(numOutliers--)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = 5*rx;
  		point.y = 5*ry;
  		point.z = 0;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));

    std::vector<float> bestParams = { 0.0, 1.0, 0.0 };
    int mostInliers = 0;

	// DONE: Fill in this function

	// For max iterations 
    for (int t = 0; t < maxIterations; ++t) // `while (maxIterations--)` from the solution is better
    {
        // Randomly sample subset and fit line
        size_t i = rand() % cloud->size();
        size_t j = rand() % cloud->size();
        auto p_i = cloud->points[i];
        auto p_j = cloud->points[j];
        auto x = p_j.y - p_i.y;
        auto y = p_i.x - p_j.x;
        auto d = std::sqrt(x*x + y*y);
        if (d < 0.0000001) continue;
        x /= d;
        y /= d;
        auto f = [x,y](pcl::PointXYZ p) -> float {
            return x * p.x + y * p.y;
        };
        auto c = f(p_i);
        int n = 0;
        // Measure distance between every point and fitted line
        for (size_t k = 0; k < cloud->size(); ++k) {
            auto p_k = cloud->points[k];
            auto e = std::abs(f(p_k) - c);
            // If distance is smaller than threshold count it as inlier
            if (e < distanceTol) ++n;
        }
        if (n > mostInliers)
        {
            mostInliers = n;
            bestParams[0] = x;
            bestParams[1] = y;
            bestParams[2] = c;
        }
    }
    auto f = [bestParams](pcl::PointXYZ p) -> float {
        return std::abs(bestParams[0] * p.x + bestParams[1] * p.y - bestParams[2]);
    };
    for (size_t k = 0; k < cloud->size(); ++k) {
        auto p_k = cloud->points[k];
        auto e = f(p_k);
        // If distance is smaller than threshold count it as inlier
        if (e < distanceTol)
            inliersResult.insert(int(k));
    }

	// Return indicies of inliers from fitted line with most inliers
	return inliersResult;

}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData();

	// DONE: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = Ransac(cloud, 1000, 0.3);

	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}


	// Render 2D point cloud with inliers and outliers
	if(inliers.size())
	{
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
	}
  	else
  	{
  		renderPointCloud(viewer,cloud,"data");
  	}
	
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
  	
}
