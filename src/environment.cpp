/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");	
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");
  
    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}


void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
	auto lidar = new Lidar(cars, 0);
	auto inputCloud = lidar->scan();

	ProcessPointClouds<pcl::PointXYZ> processor;
	auto segmentCloud = processor.SegmentPlane(inputCloud, 100, 0.2f);

	auto cloudClusters = processor.Clustering(segmentCloud.first, 1.0f, 3, 30);

	int clusterId = 0;
	std::vector<Color> colors = { Color(1,0,0), Color(0,1,0), Color(0,0,1) };

	for (pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
	{
		std::cout << "cluster size ";
		processor.numPoints(cluster);

		renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId), colors[clusterId]);
		++clusterId;

		auto box = processor.BoundingBox(cluster);
		renderBox(viewer, box, clusterId);
	}
}

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* pointProcessorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)
{
	// ----------------------------------------------------
	// -----Open 3D viewer and display City Block     -----
	// ----------------------------------------------------
	auto filterCloud = pointProcessorI->FilterCloud(inputCloud, 0.2f , Eigen::Vector4f(-10 , -6 , -3 , 1), Eigen::Vector4f(30 , 6, 5 , 1));

	auto segmentCloud = pointProcessorI->SegmentPlane(filterCloud, 100, 0.3f);

	Box box;
	box.x_min = -1.5f;
	box.y_min = -1.7f;
	box.z_min = -1.0f;
	box.x_max = 2.6f;
	box.y_max = 1.7f;
	box.z_max = -0.4f;
	renderBox(viewer, box, 0, Color(0.5f, 0.0f, 1.0f));

	renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(0, 1, 0));

	auto cloudClusters = pointProcessorI->Clustering(segmentCloud.first, 0.35f, 3, 300);

	auto clusterId = 0;
	std::vector<Color> colors = { Color(1,0,0), Color(1,1,0), Color(0,0,1) };

	for (const auto cluster : cloudClusters)
	{
		std::cout << "cluster size ";
		pointProcessorI->numPoints(cluster);

		renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId), colors[clusterId % 3]);
		++clusterId;

		auto box = pointProcessorI->BoundingBox(cluster);
		renderBox(viewer, box, clusterId);
	}

}


//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0, 0, 0);
    
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;
    
    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}


int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);

	ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
	pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointProcessorI->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");

	cityBlock(viewer, pointProcessorI, inputCloud);

    while (!viewer->wasStopped ())
    {
        viewer->spinOnce ();
    } 
}