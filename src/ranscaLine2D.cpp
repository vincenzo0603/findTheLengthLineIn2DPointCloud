#include <iostream>
#include <unordered_set>
#include <ctime>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

// Ransac寻找最大拟合线
std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));

	// 最大迭代次数
	while (maxIterations--)
	{
		std::unordered_set<int> inliers;

        // 随机生成两个点
        while (inliers.size() < 2)
			inliers.insert(rand() % cloud->points.size());
		float x1, y1, x2, y2;

		// 随机得到cloud中的两个点的索引
		auto itr = inliers.begin();
		x1 = cloud->points[*itr].x;
		y1 = cloud->points[*itr].y;
		itr++;
		x2 = cloud->points[*itr].x;
		y2 = cloud->points[*itr].y;

		// 直线方程 Ax + By + C = 0
		float A, B, C;
		B = x2 - x1;
		A = y1 - y2;
		C = (x1 * y2 - x2 * y1);
        
		for (int index = 0; index < cloud->points.size(); index++)
		{
			if (inliers.count(index) > 0) // 如果此点已经在线上，则continue;
				continue;

			pcl::PointXYZ point = cloud->points[index];
			float x3 = point.x;
			float y3 = point.y;

			// 求每个点和拟合的线之间的距离
			double distance = fabs(A * x3 + B * y3 + C) / sqrt(pow(A, 2) + pow(B, 2));

			// 如果距离小于设定阈值，insert到inlier
			if (distance <= distanceTol)
				inliers.insert(index);
		}

		// 从拟合的线中选出最长的那条
		if (inliers.size() > inliersResult.size())
			inliersResult = inliers;
	}

	return inliersResult;

}

int main(int argc, char** argv)
{
    // 创建一个PointCloud<pcl::PointXYZ>
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());    // 内点点云指针
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());    // 外点点云指针

    // 打开文件
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("../file/corner17.pcd", *cloud) == -1 )
    {
        PCL_ERROR("can not open or fount corner17.pcd.\n");
        return (-1);
    }
    std::cout << "cloud->size():" << cloud->size() << std::endl;

    // 寻找一条最长的拟合直线，Ransac( cloud, 迭代次数， 距离阈值)
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    std::unordered_set<int> inliers = Ransac(cloud, 50, 0.1);
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

    // 保存内点 外点
    // for(int index = 0; index < cloud->points.size(); index++)
	// {
	// 	pcl::PointXYZ point = cloud->points[index];
	// 	if(inliers.count(index))
	// 		cloudInliers->points.push_back(point);
	// 	else
	// 		cloudOutliers->points.push_back(point);
	// }

    // std::cout << "cloud->size(): "        << cloud->size()        << std::endl;
    // std::cout << "cloud->points.size(): " << cloud->points.size() << std::endl;

    for (int index = 0; index < cloud->size(); index++)
    {
        // pcl::PointXYZ point = cloud->points[index];
        if(inliers.count(index))
            cloudInliers->push_back(cloud->points[index]);
        else
            cloudOutliers->push_back(cloud->points[index]);
    }

    // for (size_t i = 0; i < cloudInliers->size(); i++)
    // {
    //     std::cout << cloudInliers->points[i].x << "\t" << cloudInliers->points[i].y << "\t" << cloudInliers->points[i].z << std::endl;
    // }
    
    // 分别保存为pcd文件
    // pcl::io::savePCDFileASCII("../file/corner17_inliers_0d1.pcd", *cloudInliers);
    // pcl::io::savePCDFileASCII("../file/corner17_outliers_0d1.pcd", *cloudOutliers);
    std::cout << "Time taken (s): "
              << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() / 1000000.0
              << std::endl;
}