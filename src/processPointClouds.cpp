// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"


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

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    typename pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>());

    pcl::VoxelGrid<PointT> sor;
    sor.setInputCloud (cloud);
    sor.setLeafSize (filterRes, filterRes, filterRes);
    sor.filter (*cloud_filtered);

    typename pcl::PointCloud<PointT>::Ptr cloud_region (new pcl::PointCloud<PointT>());

    pcl::CropBox<PointT> region(true);  //把一定区域内的点云裁剪掉，比如车道外的点，前后太远的点。
    region.setMin(minPoint);
    region.setMax(maxPoint);
    region.setInputCloud(cloud_filtered);
    region.filter(*cloud_region);

    std::vector<int> indice;
    pcl::CropBox<PointT> roof(true);  //把打到车顶上的点云裁剪掉。
    region.setMin(Eigen::Vector4f(-1.5,-1.7,-1.2,1));
    region.setMax(Eigen::Vector4f(2.5,1.7,0,1));
    region.setInputCloud(cloud_region);
    region.filter(indice);

    pcl::PointIndices::Ptr inliers (new pcl::PointIndices());
    for(int ind: indice)
        inliers->indices.push_back(ind);

    // Create the filtering object
    pcl::ExtractIndices<PointT> extract;
    // Extract the inliers
    extract.setInputCloud (cloud_region);
    extract.setIndices (inliers);
    extract.setNegative (true);
    extract.filter (*cloud_region);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloud_region;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr plancloud (new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr obstcloud (new pcl::PointCloud<PointT>());

    for(int index : inliers->indices)
        plancloud->points.push_back(cloud->points[index]);
    
    // Create the filtering object
    pcl::ExtractIndices<PointT> extract;
    // Extract the inliers
    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.setNegative (true);
    extract.filter (*obstcloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(plancloud, obstcloud);
    return segResult;
}


//========================= use pcl to do the ransac segment ====================
template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in this function to find inliers for the cloud.
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices());
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    // Create the segmentation object
    pcl::SACSegmentation<PointT> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (maxIterations);
    seg.setDistanceThreshold (distanceThreshold);

    seg.setInputCloud (cloud);
    seg.segment (*inliers, *coefficients);
 
    if (inliers->indices.size () == 0)
    {
      std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}


//========================= use My own algorithm to do the ransac segment ====================
template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::mySegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices());

    //std::unordered_set<int> inliersResult;
	//std::unordered_set<int> oldResult;
    std::vector<int> inliersResult;   //vector better then unorder_set on speed
    std::vector<int> oldResult;
	srand(time(NULL));
	
	int numcloud = cloud->points.size();
	for(int i=0;i<maxIterations;i++){
		inliersResult.clear();
		PointT p1=cloud->points[(rand() % numcloud)];
		PointT p2=cloud->points[(rand() % numcloud)];
        PointT p3=cloud->points[(rand() % numcloud)];

		float A = (p2.y - p1.y)*(p3.z - p1.z) - (p2.z - p1.z)*(p3.y - p1.y);
		float B = (p2.z - p1.z)*(p3.x - p1.x) - (p2.x - p1.x)*(p3.z - p1.z);
		float C = (p2.x - p1.x)*(p3.y - p1.y) - (p2.y - p1.y)*(p3.x - p1.x);
        float D = -(A*p1.x + B*p1.y + C*p1.z);
		
		for(int k=0; k<numcloud; k++){
			PointT ptem = cloud->points[k];
			float dist = abs(A*ptem.x + B*ptem.y + C*ptem.z + D)/sqrt(A*A + B*B + C*C);
			//if(dist < distanceThreshold) inliersResult.insert(k);
            if(dist < distanceThreshold) inliersResult.push_back(k);
		}
		if(inliersResult.size()>oldResult.size()) oldResult=inliersResult;
	}

    for(int ind: oldResult)
        inliers->indices.push_back(ind);  
    
    if (inliers->indices.size () == 0)
    {
      std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}


//================ use my pcl to do the clustering =====================
template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{
    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();
    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    // Creating the KdTree object for the search method of the extraction
    typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    tree->setInputCloud (cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance (clusterTolerance); 
    ec.setMinClusterSize (minSize);
    ec.setMaxClusterSize (maxSize);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    ec.extract (cluster_indices);

    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        typename pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>);
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
            cloud_cluster->points.push_back (cloud->points[*pit]); 

        cloud_cluster->width = cloud_cluster->points.size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        clusters.push_back(cloud_cluster);
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}


template<typename PointT>           
void ProcessPointClouds<PointT>::clusthelp(int indice, typename pcl::PointCloud<PointT>::Ptr cloud, KdTree* tree, float distanceTol, std::vector<bool> &processed, std::vector<int> &cluster)
{	
	processed[indice] = true;
	cluster.push_back(indice);

	std::vector<int> rawclust = tree->search(cloud->points[indice], distanceTol);
	for(int k : rawclust)
	{
		if(!processed[k]) 
			clusthelp(k, cloud, tree, distanceTol, processed, cluster);
	}
}

//================ use my own kdtree to do the clustering =====================
template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::myClustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{
    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();
    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

	std::vector<std::vector<int>> clusters_indix;
	std::vector<bool> processed (cloud->points.size(), false);

    KdTree* tree = new KdTree;
    for (int i=0; i<cloud->points.size(); i++) 
        tree->insert(cloud->points[i], i); 

	for(int i=0; i< cloud->points.size(); i++)
	{
		if(processed[i]) continue;
		std::vector<int> cluster_indix;
		clusthelp(i, cloud, tree, clusterTolerance, processed, cluster_indix);
		clusters_indix.push_back(cluster_indix);
	}

    for(std::vector<int> indix: clusters_indix)
        {
            if(indix.size() < minSize || indix.size() > maxSize) continue;

            typename pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>);
            for(int ind: indix)
                cloud_cluster->points.push_back(cloud->points[ind]);
            
            cloud_cluster->width = cloud_cluster->points.size ();
            cloud_cluster->height = 1;
            cloud_cluster->is_dense = true;

            clusters.push_back(cloud_cluster);
        }

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

template<typename PointT>   //利用pca寻找最小box.
BoxQ ProcessPointClouds<PointT>::BoundingBoxQ(typename pcl::PointCloud<PointT>::Ptr cluster)
{
    Eigen::Vector4f pcaCentroid;
    pcl::compute3DCentroid(*cluster,pcaCentroid);
    Eigen::Matrix3f covariance;
    pcl::computeCovarianceMatrixNormalized(*cluster, pcaCentroid, covariance);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
    Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
    eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));

    // Transform the original cloud to the origin where the principal components correspond to the axes.
    Eigen::Matrix4f transform(Eigen::Matrix4f::Identity());
    transform.block<3,3>(0,0) = eigenVectorsPCA.transpose();
    transform.block<3,1>(0,3) = -1.f * (transform.block<3,3>(0,0) * pcaCentroid.head<3>());

    typename pcl::PointCloud<PointT>::Ptr transformedCloud(new pcl::PointCloud<PointT>);
    pcl::transformPointCloud(*cluster, *transformedCloud, transform);

    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*transformedCloud, minPoint, maxPoint);
    const Eigen::Vector3f meanDiagonal = 0.5f*(maxPoint.getVector3fMap() + minPoint.getVector3fMap());

    BoxQ boxq;
    boxq.bboxQuaternion = eigenVectorsPCA;
    boxq.bboxTransform = eigenVectorsPCA * meanDiagonal + pcaCentroid.head<3>();
    boxq.cube_length = maxPoint.x - minPoint.x;
    boxq.cube_width = maxPoint.y - minPoint.y;
    boxq.cube_height = maxPoint.z - minPoint.z;

    return boxq;
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