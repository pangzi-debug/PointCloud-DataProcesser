#include <pcl/ModelCoefficients.h>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

int 
main()
{
    // Read in the cloud data
    pcl::PCDReader reader;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>), cloud_f(new pcl::PointCloud<pcl::PointXYZ>);
    reader.read("plane_Segmentation_Sample.pcd",*cloud);
    std::cout<<"PointCloud before filtering has: "<<cloud->size()<<" data points."<< std::endl; // 输出在过滤之前的点云中的顶点数目

    // 创建过滤后的物体，使用叶子大小为10cm进行降采样
    // Create the filtering object: downsample the dataset using a leaf size of 10cm // 这里的2M
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    vg.setInputCloud(cloud);
    vg.setLeafSize(2.f,2.f,2.f); // 这里是2M
    vg.filter(*cloud_filtered);
    std::cout<<" PointCloud after filtering has: "<< cloud_filtered->size() <<" data points."<< std::endl;

    //创建用于平面物体分割物体 并且设置所有的参数
    //Create the segmentation object for the planar model and set all the parameters
    pcl::SACSegmentation<pcl::PointXYZ> seg ; 
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices); // 存储内部的点(类型是顶点索引)
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients); // 模型的参数
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>()); // 平面
    pcl::PCDWriter writer;
    seg.setOptimizeCoefficients(true); 
    seg.setModelType(pcl::SACMODEL_PLANE); // 模型类型
    seg.setMethodType(pcl::SAC_RANSAC); // 随机采样方法
    seg.setMaxIterations(100); //
    seg.setDistanceThreshold(20);

    int nr_points = (int) cloud_filtered->size();
    
    while(cloud_filtered->size() > 0.5 * nr_points)
    {
        // 从剩余的顶点中分割最大的平面
        // Segment the largets planar component from the remaining cloud
        seg.setInputCloud(cloud_filtered);
		seg.segment(*inliers, *coefficients); // 通过分割后 顶点存储在inliers中 ，系数存储在coefficients中
        if(inliers->indices.size() == 0)
        {
            std::cout<<"Could not estimate a planar model for the given dataset."<< std::endl;
            break; 
        }

		// 构造pcl的提取器，三个参数 1，点云 2，当前提取的索引 3，是否进行提取或者反向提取
		pcl::ExtractIndices<pcl::PointXYZ> extract;
		extract.setInputCloud(cloud_filtered);
		extract.setIndices(inliers); 
		extract.setNegative(false);
        // 得到和平面相关的顶点
        // get the points associated with the planar surface
        extract.filter(*cloud_plane);  
        std::cout << "PointCloud representing the planar component: "<< cloud_plane->size() << " data points."<< std::endl;

        // 移除平面上的点，提取剩余部分
        // Remove the planar inliers, extract the rest 
        extract.setNegative(true);
        extract.filter(*cloud_f); 
        *cloud_filtered = *cloud_f;
    }

    writer.write("plane.pcd",*cloud_filtered); // CHECK 
    return 0;
    
    return 0;
}