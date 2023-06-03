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

	std::vector<pcl::PointCloud<pcl::PointXYZ>> v(nx * ny * nz);
	for (size_t i = 0; i < cloud->points.size(); i++)
	{
		int ix = (cloud->points[i].x - minPt.x) / leafsize;
		int iy = (cloud->points[i].y - minPt.y) / leafsize;
		int iz = (cloud->points[i].z - minPt.z) / leafsize;
		v[ix + iy * nx + iz * nx * ny].push_back(cloud->points[i]);
	}

	for (int i = 0; i < v.size(); ++i)
	{
		if (v[i].size())
		{
			Eigen::Vector4f centroid;
			pcl::compute3DCentroid(v[i], centroid);
			cloud_filtered->push_back(pcl::PointXYZ(centroid.x(), centroid.y(), centroid.z()));
		}
	}
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