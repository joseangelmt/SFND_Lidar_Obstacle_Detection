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
	return pointProcessor.loadPcd("../../../../../sensors/data/pcd/simpleHighway.pcd");
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

std::unordered_set<int> RansacLine(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));

	while (maxIterations--) {
		std::unordered_set<int> inliers;
		while (inliers.size() < 2)
			inliers.insert(rand() % cloud->points.size());

		auto iterator = inliers.begin();
		const auto& point1{ cloud->points[*iterator++] };
		const auto& point2{ cloud->points[*iterator++] };

		const auto A{ point1.y - point2.y };
		const auto B{ point2.x - point1.x };
		const auto C{ point1.x * point2.y - point2.x * point1.y };

		const auto denominator{ sqrt(A * A + B * B) };

		for (int i = 0; i < cloud->points.size(); i++) {
			if (inliers.count(i))
				continue;

			const auto& point{ cloud->points[i] };

			const auto d{ fabs(point.x * A + point.y * B + C) / denominator };
			if (d < distanceTol)
				inliers.insert(i);
		}

		if (inliers.size() > inliersResult.size())
			inliersResult = inliers;
	}

	return inliersResult;
}

inline pcl::PointXYZ operator-(const pcl::PointXYZ& a, const pcl::PointXYZ& b)
{
	return pcl::PointXYZ{ a.x - b.x, a.y - b.y, a.z - b.z };
}

inline pcl::PointXYZ operator^(const pcl::PointXYZ& a, const pcl::PointXYZ& b)
{
	//  i   j   k
	// a.x a.y a.z
	// b.x b.y b.z
	return pcl::PointXYZ{
		a.y * b.z - a.z * b.y,
		a.z * b.x - a.x * b.z,
		a.x * b.y - a.y * b.x
	};
}
inline float operator*(const pcl::PointXYZ& a, const pcl::PointXYZ& b)
{
	return a.x * b.x + a.y * b.y + a.z * b.z;
}

std::unordered_set<int> RansacPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));

	while (maxIterations--) {
		std::unordered_set<int> inliers;
		while (inliers.size() < 3)
			inliers.insert(rand() % cloud->points.size());

		auto iterator = inliers.begin();
		const auto& point1{ cloud->points[*iterator++] };
		const auto& point2{ cloud->points[*iterator++] };
		const auto& point3{ cloud->points[*iterator] };

		const auto v1{ point2 - point1 };
		const auto v2{ point3 - point1 };
		const auto cross{ v1 ^ v2 };

		const auto A{ cross.x };
		const auto B{ cross.y };
		const auto C{ cross.z };
		const auto D{ cross * point1 };

		const auto denominator{ sqrt(A * A + B * B + C * C) };

		for (int i = 0; i < cloud->points.size(); i++) {
			if (inliers.count(i))
				continue;

			const auto& point{ cloud->points[i] };

			const auto d{ fabs(point.x * A + point.y * B + point.z * C + D) / denominator };
			if (d < distanceTol)
				inliers.insert(i);
		}

		if (inliers.size() > inliersResult.size())
			inliersResult = inliers;
	}

	return inliersResult;
}
int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	

	std::unordered_set<int> inliers = RansacPlane(cloud, 50, 0.5);

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
