// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"
#include <unordered_set>


//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

	pcl::VoxelGrid<PointT> vg;
	typename pcl::PointCloud<PointT>::Ptr cloudFiltered{ new pcl::PointCloud<PointT> };
	vg.setInputCloud(cloud);
	vg.setLeafSize(filterRes, filterRes, filterRes);
	vg.filter(*cloudFiltered);

	typename pcl::PointCloud<PointT>::Ptr cloudRegion{ new pcl::PointCloud<PointT> };

	pcl::CropBox<PointT> region(true);
	region.setMin(minPoint);
	region.setMax(maxPoint);
	region.setInputCloud(cloudFiltered);
	region.filter(*cloudRegion);

	//std::vector<int> indices;

	//pcl::CropBox<PointT> roof(true);
	//roof.setMin(Eigen::Vector4f(-1.5, -1.7, -1, 1));
	//roof.setMax(Eigen::Vector4f(2.6, 1.7, -0.4, 1));
	//roof.setInputCloud(cloudRegion);
	//roof.filter(indices);

	//pcl::PointIndices::Ptr inliers{ new pcl::PointIndices };
	//inliers->indices = indices;

	//pcl::ExtractIndices<PointT> extract;
	//extract.setInputCloud(cloudRegion);
	//extract.setIndices(inliers);
	//extract.setNegative(true);
	//extract.filter(*cloudRegion);


    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloudRegion;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
	typename pcl::PointCloud<PointT>::Ptr obstacles(new pcl::PointCloud<PointT>());
	typename pcl::PointCloud<PointT>::Ptr road(new pcl::PointCloud<PointT>());

	for (auto index : inliers->indices)
		road->points.push_back(cloud->points[index]);

	pcl::ExtractIndices<PointT> extract;
	extract.setInputCloud(cloud);
	extract.setIndices(inliers);
	extract.setNegative(true);
	extract.filter(*obstacles);

	std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstacles, road);
	return segResult;
}

#define MY_OWN_SEGMENTATION_IMPLEMENTATION

#ifdef MY_OWN_SEGMENTATION_IMPLEMENTATION
inline pcl::PointXYZ operator-(const pcl::PointXYZ& a, const pcl::PointXYZ& b)
{
	return pcl::PointXYZ{ a.x - b.x, a.y - b.y, a.z - b.z };
}

inline pcl::PointXYZ operator^(const pcl::PointXYZ& a, const pcl::PointXYZ& b)
{
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

template<typename CloudType>
static pcl::PointIndices::Ptr RansacPlane(const CloudType& cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));

	while (maxIterations--) {
		std::unordered_set<int> inliers;
		while (inliers.size() < 3)
			inliers.insert(rand() % cloud->points.size());

		auto iterator = inliers.begin();
		pcl::PointXYZ point1{ cloud->points[*iterator].x, cloud->points[*iterator].y, cloud->points[*iterator++].z };
		pcl::PointXYZ point2{ cloud->points[*iterator].x, cloud->points[*iterator].y, cloud->points[*iterator++].z };
		pcl::PointXYZ point3{ cloud->points[*iterator].x, cloud->points[*iterator].y, cloud->points[*iterator].z };

		const auto v1{ point2 - point1 };
		const auto v2{ point3 - point1 };
		const auto cross{ v1 ^ v2 };

		const auto A{ cross.x };
		const auto B{ cross.y };
		const auto C{ cross.z };
		const auto D{ -(cross * point1) };

		const auto denominator{ sqrt(A * A + B * B + C * C) };

		for (int i = 0; i < cloud->points.size(); i++) {
			if (inliers.count(i))
				continue;

			const auto& point{ cloud->points[i] };

			const auto d{ fabs(point.x * A + point.y * B + point.z * C + D) / denominator };
			if (d <= distanceTol)
				inliers.insert(i);
		}

		if (inliers.size() > inliersResult.size())
			inliersResult = inliers;
	}

	pcl::PointIndices::Ptr result{ new pcl::PointIndices() };
	for (auto inlier : inliersResult)
		result->indices.push_back(inlier);
	return result;
}
#endif

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

#ifdef MY_OWN_SEGMENTATION_IMPLEMENTATION
	auto inliers = RansacPlane(cloud, maxIterations, distanceThreshold);
#else
	pcl::PointIndices::Ptr inliers{ new pcl::PointIndices() };
	pcl::ModelCoefficients::Ptr coefficients{ new pcl::ModelCoefficients() };

	// Create the segmentation object
	pcl::SACSegmentation<PointT> seg;
	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setMaxIterations(maxIterations);
	seg.setDistanceThreshold(distanceThreshold);

	seg.setInputCloud(cloud);
	seg.segment(*inliers, *coefficients);
#endif

	if (inliers->indices.size() == 0)
	{
		std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
	}

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}

#define MY_OWN_CLUSTERIGN_IMPLEMENTATION

#ifdef MY_OWN_CLUSTERIGN_IMPLEMENTATION
#include "quiz/cluster/kdtree.h"

static void proximity(std::vector<int>& cluster, const std::vector<std::vector<float>>& points, int currentPoint, bool* processed, KdTree<3>* tree, float distanceTol)
{
	processed[currentPoint] = true;
	cluster.push_back(currentPoint);

	auto nearbyPoints = tree->search(points[currentPoint], distanceTol);
	for (const auto nearbyPoint : nearbyPoints) {
		if (!processed[nearbyPoint])
			proximity(cluster, points, nearbyPoint, processed, tree, distanceTol);
	}
}

static std::vector<std::vector<int>> euclideanCluster(const std::vector<std::vector<float>>& points, KdTree<3>* tree, float distanceTol)
{
	bool* processed = new bool[points.size()];
	for (int i = 0; i < points.size(); i++)
		processed[i] = false;

	std::vector<std::vector<int>> clusters;

	for (auto i = 0; i < points.size(); i++) {
		if (processed[i])
			continue;

		std::vector<int> cluster;
		proximity(cluster, points, i, processed, tree, distanceTol);
		clusters.push_back(cluster);
	}

	delete[] processed;
	return clusters;
}

#endif

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{
	// Time clustering process
	auto startTime = std::chrono::steady_clock::now();

	std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

#ifdef MY_OWN_CLUSTERIGN_IMPLEMENTATION
	KdTree<3> tree;

	std::vector<std::vector<float>> points;
	for (const auto& point : cloud->points) {
		std::vector<float> p{ point.x, point.y, point.z };
		tree.insert(p, points.size());
		points.push_back(p);
	}

	auto cluster_indices = euclideanCluster(points, &tree, clusterTolerance);

	for (const auto pointIndices : cluster_indices)
	{
		if (pointIndices.size() < minSize)
			continue;
		if (pointIndices.size() > maxSize)
			continue;

		typename pcl::PointCloud<PointT>::Ptr cloud_cluster(new pcl::PointCloud<PointT>);
		for (auto pit : pointIndices)
			cloud_cluster->points.push_back(cloud->points[pit]);
		cloud_cluster->width = cloud_cluster->points.size();
		cloud_cluster->height = 1;
		cloud_cluster->is_dense = true;

		clusters.push_back(cloud_cluster);
	}

#else
	typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
	tree->setInputCloud(cloud);

	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<PointT> ec;
	ec.setClusterTolerance(clusterTolerance);
	ec.setMinClusterSize(minSize);
	ec.setMaxClusterSize(maxSize);
	ec.setSearchMethod(tree);
	ec.setInputCloud(cloud);
	ec.extract(cluster_indices);

	for (const auto pointIndices : cluster_indices)
	{
		typename pcl::PointCloud<PointT>::Ptr cloud_cluster(new pcl::PointCloud<PointT>);
		for (auto pit : pointIndices.indices)
			cloud_cluster->points.push_back(cloud->points[pit]);
		cloud_cluster->width = cloud_cluster->points.size();
		cloud_cluster->height = 1;
		cloud_cluster->is_dense = true;

		clusters.push_back(cloud_cluster);
	}
#endif


	auto endTime = std::chrono::steady_clock::now();
	auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
	std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

	return clusters;
}

template<typename PointT>
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


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}