#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>

/**
 * @description:			体素滤波
 * @param cloud				输入点云
 * @param cloud_filtered	滤波点云
 * @param leafsize			体素大小
 */
void voxelgrid(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_filtered, float leafsize)
{
	pcl::PointXYZ minPt, maxPt;
	pcl::getMinMax3D(*cloud, minPt, maxPt);
	float lx = maxPt.x - minPt.x;
	float ly = maxPt.y - minPt.y;
	float lz = maxPt.z - minPt.z;
	int nx = lx / leafsize + 1;
	int ny = ly / leafsize + 1;
	int nz = lz / leafsize + 1;

    std::vector<std::pair<int,int>> v;
	for (size_t i = 0; i < cloud->points.size(); i++)
	{
		int ix = (cloud->points[i].x - minPt.x) / leafsize;
		int iy = (cloud->points[i].y - minPt.y) / leafsize;
		int iz = (cloud->points[i].z - minPt.z) / leafsize;
		v.push_back(std::pair<int, int>{ix + iy*nx + iz*nx*ny, i});
	}

    std::sort(v.begin(), v.end(), [](std::pair<int, int> p1, std::pair<int, int>p2) {return p1.first < p2.first; });

	int start = 0;
    std::vector<int> point_id;
	pcl::PointCloud<pcl::PointXYZ>::Ptr ptcloud;
	Eigen::Vector4f centroid;
	for (int i = start; i < v.size() - 1; ++i)
	{
		if (v[i].first != v[i + 1].first)
		{
			for (int id = start; id <= i; ++id)	point_id.push_back(v[i].second);
			ptcloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
			pcl::copyPointCloud(*cloud, point_id, *ptcloud);
			pcl::compute3DCentroid(*ptcloud, centroid);
			cloud_filtered->push_back(pcl::PointXYZ(centroid[0], centroid[1], centroid[2]));
			start = i + 1;
			point_id.clear();
		}
		else if (v[i].first == v[v.size() - 1].first)
		{
			point_id.push_back(v[i].second);
		}
	}
	ptcloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::copyPointCloud(*cloud, point_id, *ptcloud);
	pcl::compute3DCentroid(*ptcloud, centroid);
	cloud_filtered->push_back(pcl::PointXYZ(centroid[0], centroid[1], centroid[2]));
}

int main(int argc, char* argv[])
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile("rabbit.pcd", *cloud);

	voxelgrid(cloud, cloud_filtered, 1.0);
	pcl::io::savePCDFile("myvoxelgrid.pcd", *cloud_filtered);

	system("pause");
	return EXIT_SUCCESS;
}

