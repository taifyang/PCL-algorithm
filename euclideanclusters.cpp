#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>

/**
 * @description:			欧式聚类
 * @param cloud				输入点云
 * @param clusters			分割结果
 * @param threshold			距离阈值
 * @param min_cluster_size	最小聚类点数
 * @param max_cluster_size	最大聚类点数
 */
void euclideanclusters(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& clusters,
	float threshold, int min_cluster_size, int max_cluster_size)
{
	pcl::KdTreeFLANN<pcl::PointXYZ> tree;
	tree.setInputCloud(cloud);

	std::vector<pcl::PointIndices> cluster_indices;				
	std::vector<bool> processed(cloud->points.size(), false);	
	std::vector<int> nn_indices;								
	std::vector<float> nn_distances;							

	for (int i = 0; i < cloud->points.size(); ++i)   
	{
		if (processed[i])   continue; 
			
		std::vector<int> seed_queue; 

		int sq_idx = 0;				
		seed_queue.push_back(i);	
		processed[i] = true;	

		while (sq_idx < seed_queue.size()) 
		{
			if (!tree.radiusSearch(seed_queue[sq_idx], threshold, nn_indices, nn_distances))
			{
				++sq_idx;
				continue;  
			}

			for (size_t j = 0; j < nn_indices.size(); ++j)    
			{
				if (processed[nn_indices[j]])	continue;   

				seed_queue.push_back(nn_indices[j]);	
				processed[nn_indices[j]] = true;		
			}

			++sq_idx;
		}

		if (seed_queue.size() >= min_cluster_size && seed_queue.size() <= max_cluster_size)
		{
			pcl::PointIndices r;
			r.indices = seed_queue;
			cluster_indices.push_back(r);   
		}
	}

	for (int i = 0; i < cluster_indices.size(); ++i)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_temp(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::copyPointCloud(*cloud, cluster_indices[i].indices, *cluster_temp);
		clusters.push_back(cluster_temp);
	}
}

int main(int argc, char** argv)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters;
	pcl::io::loadPCDFile("part_path.pcd", *cloud);

	euclideanclusters(cloud, clusters, 5, 100, 500);
	for (int i = 0; i < clusters.size(); ++i)
	{
        std::cout << clusters[i]->size() << std::endl;
        pcl::io::savePCDFile("cluster" + std::to_string(i) + ".pcd", *clusters[i]);
	}
	
	system("pause");
	return EXIT_SUCCESS;
}

