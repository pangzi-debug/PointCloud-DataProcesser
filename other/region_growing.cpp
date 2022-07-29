#include <queue>
#include <vector>
#include <pcl/filters/extract_indices.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <iostream>
#include <set>
#include <algorithm>
#include<cstring>
#include <Windows.h> 
#include <string>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/don.h>
#include <pcl/surface/mls.h>
#include "crf\CSF.h"
#include <fstream>
// 使用PointIndices从cloud删除目标索引 https://stackoverflow.com/questions/41960911/how-can-i-get-the-pointindices-of-a-given-pointcloud
// 找到指定的索引是否在点云中存在
// https://blog.csdn.net/qq_41938858/article/details/105076127
// 找到在一个文件夹中的所有的文件的绝对路径
std::vector<std::string> get_all_abspath_within_folder(std::string folder, std::vector<std::string>& names)
{
	std::string search_path = folder + "/*.*";
	WIN32_FIND_DATA fd;
	HANDLE hFind = ::FindFirstFile(search_path.c_str(), &fd);
	if (hFind != INVALID_HANDLE_VALUE) {
		do {
			// read all (real) files in current folder
			// , delete '!' read other 2 default folder . and ..
			if ((fd.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY)) {
				if (std::strcmp(fd.cFileName, ".") != 0 && std::strcmp(fd.cFileName, "..") != 0) {
					//std::cout << fd.cFileName << std::endl;
					//names.push_back(folder + fd.cFileName);
					std::string subfolder = folder + fd.cFileName;
					// std::cout << subfolder << std::endl;
					get_all_abspath_within_folder(subfolder,names);
				}

			}
			else {
				//std::cout << fd.cFileName << std::endl;
				std::string file_prefix = folder + "\\";
				names.push_back(file_prefix + fd.cFileName);
			}
				
			
		} while (::FindNextFile(hFind, &fd));
		::FindClose(hFind);
	}
	return names;
}

void 
savePointCloudTxt(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::string cloudname) {
	// 将pcd文件转换成txt文件存储
	// 使用文本输入输出流
	std::ofstream outfile(cloudname);
	for (int i = 0; i < cloud->points.size(); i++) {
		pcl::PointXYZ curPoint = cloud->points[i];
		outfile << curPoint.x << " " << curPoint.y << " " << curPoint.z << " " << std::endl;
	}
}
void
performCSFFilter(std::string pointClouds_filepath,std::string off_groundpointname) {
	CSF csf;
	csf.readPointsFromFile(pointClouds_filepath);
	csf.params.bSloopSmooth = false;
	csf.params.class_threshold = 0.5;
	csf.params.cloth_resolution = 2;
	csf.params.interations = 500;
	csf.params.rigidness = 3;
	csf.params.time_step = 0.65;

	std::vector<int> groundIndexes, offGroundIndexes;
	csf.do_filtering(groundIndexes, offGroundIndexes, false);
	std::cout << groundIndexes.size() << " belong to ground index" << std::endl;
	std::cout << offGroundIndexes.size() << " belong to offground index" << std::endl;
	csf.savePoints(offGroundIndexes, off_groundpointname);
	
}


pcl::PointCloud<pcl::Normal>::Ptr 
caclNormal(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,float searchRadius = 0.07) {
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);


	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
	normalEstimation.setInputCloud(cloud);
	// For every point, use all neighbors in a radius of 3cm.
	normalEstimation.setRadiusSearch(searchRadius);
	// A kd-tree is a data structure that makes searches efficient. More about it later.
	// The normal estimation object will use it to find nearest neighbors.
	pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
	normalEstimation.setSearchMethod(kdtree);

	// 计算法线 存储在pcl的normals变量中
	// Calculate the normals.
	normalEstimation.compute(*normals);

	return normals;
}

// 进行difference of normals计算 用于提取边界区域
pcl::PointCloud<pcl::PointNormal>::Ptr
performDON(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,float large_scale,float small_scale)
{
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(cloud);
	pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::PointNormal> ne;
	ne.setInputCloud(cloud);
	ne.setSearchMethod(tree);
	ne.setViewPoint(std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
	std::cout << "Calculating normals for scale..." << small_scale << std::endl;
	pcl::PointCloud<pcl::PointNormal>::Ptr normals_small_scale(new pcl::PointCloud<pcl::PointNormal>);
	ne.setRadiusSearch(small_scale);
	ne.compute(*normals_small_scale);

	// calculate normals with the large scale
	std::cout << "Calculating normals for scale..." << large_scale << std::endl;
	pcl::PointCloud<pcl::PointNormal>::Ptr normals_large_scale(new pcl::PointCloud<pcl::PointNormal>);

	ne.setRadiusSearch(large_scale);
	ne.compute(*normals_large_scale);

	// Create output cloud for DoN results
	pcl::PointCloud<pcl::PointNormal>::Ptr doncloud(new pcl::PointCloud<pcl::PointNormal>);
	pcl::copyPointCloud(*cloud, *doncloud);

	std::cout << "Calculating DoN... " << std::endl;
	// Create DoN operator
	pcl::DifferenceOfNormalsEstimation<pcl::PointXYZ, pcl::PointNormal, pcl::PointNormal> don;
	don.setInputCloud(cloud);
	don.setNormalScaleLarge(normals_large_scale);
	don.setNormalScaleSmall(normals_small_scale);
	if (!don.initCompute())
	{
		std::cerr << "Error: Could not initialize DoN feature operator" << std::endl;
		exit(EXIT_FAILURE);
	}
	// Compute DoN
	don.computeFeature(*doncloud);
	return doncloud;


} 

pcl::PointCloud<pcl::PointNormal>
resampling(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
	// Create a KD-Tree
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

	// Output has the PointNormal type in order to store the normals calculated by MLS
	pcl::PointCloud<pcl::PointNormal> mls_points;

	// Init object (second point type is for the normals, even if unused)
	pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;

	mls.setComputeNormals(true);

	// Set parameters
	mls.setInputCloud(cloud);
	mls.setPolynomialOrder(2);
	mls.setSearchMethod(tree);
	mls.setSearchRadius(0.08);

	// Reconstruct
	mls.process(mls_points);

	// Save output
	return mls_points;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr
extractOffGround(std::string inputcloudname, std::string outputcloudname,std::string txtinputfolder,std::string txtoutputfolder ,std::string pcdnameprefix) {
	// read point cloud ,convert it to txt
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	if (pcl::io::loadPCDFile <pcl::PointXYZ>(inputcloudname, *cloud) == -1)
	{
		std::cout << "Cloud reading failed." << std::endl;
		return nullptr;
	}
	std::string txtinputfullpath = txtinputfolder + "//" + pcdnameprefix + ".txt";
	std::string txtoutputfullpath = txtoutputfolder + "//" + pcdnameprefix + ".txt";
	savePointCloudTxt(cloud, txtinputfullpath);
	std::cout << txtinputfullpath << std::endl;
	performCSFFilter(txtinputfullpath, txtoutputfullpath);

	//将文件读取到向量中，转换成pcd文件
	std::ifstream off_ground_pcd(txtoutputfullpath);

	pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud(new pcl::PointCloud<pcl::PointXYZ>);

	float x_coord, y_coord, z_coord;
	while (off_ground_pcd >> x_coord >> y_coord >> z_coord) {
		pcl::PointXYZ curPoint;
		curPoint.x = (x_coord); 
		curPoint.y = (y_coord);
		curPoint.z = (z_coord);
		out_cloud->points.push_back(curPoint);
	}
	out_cloud->height = 1;
	out_cloud->width = out_cloud->points.size();
	out_cloud->is_dense = false;
	// pcl::io::savePCDFile(outputcloudname, *out_cloud);
	return out_cloud;
	// 逐行读取文件中的 字符串
	// 将字符串分割成3列 分别表示点云顶点

}
int
region_growing
(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::string outputcloudname)
{	
	std::cout << "processing " << outputcloudname << std::endl;
	/*pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	if (pcl::io::loadPCDFile <pcl::PointXYZ>(inputcloudname, *cloud) == -1)
	{
		std::cout << "Cloud reading failed." << std::endl;
		return (-1);
	}*/
	// 使用DON对当前点云的区域进行一个插值计算，目的是嫩巩固过滤掉点云的带有边界部分
	pcl::PointCloud<pcl::PointNormal>::Ptr DON_cloud = performDON(cloud, 32, 4);
	// 保存法线到目标文件中
	pcl::PointNormal minPt, maxPt;
	pcl::getMinMax3D(*DON_cloud, minPt, maxPt);
	float cloud_bb_length = maxPt.x - minPt.x;
	float cloud_bb_width = maxPt.y - minPt.y;
	float min_z_coord = 1000;
	float max_z_coord = -1000;
	// 局部z值
	float seed_search_radius = std::min(cloud_bb_length, cloud_bb_width);
	seed_search_radius = seed_search_radius / 6.0; // 原来包围盒的.5倍
	float minmax_search_radius = seed_search_radius / 3.0;// 对于最高点和最低点的搜索我们设置一个比较大的半径
	pcl::KdTreeFLANN<pcl::PointNormal> kdtree;
	kdtree.setInputCloud(DON_cloud);
	// search seed
	std::vector<int> seed_candidates;
	std::vector<float> seed_candidates_distances;
	// search min max point
	std::vector<int> minmax_candidates;
	std::vector<float> minmax_candidates_distances;
	// 在指定位置创建中心节点 用于对中心点半径范围内的顶点进行搜索
	float c_x = (maxPt.x + minPt.x) / 2.0;
	float c_y = (maxPt.y + minPt.y) / 2.0;
	float c_z = (maxPt.z + minPt.z) / 2.0;
	// 指定半径找到 所有的顶点
	pcl::PointNormal centerPoint;
	centerPoint.x = c_x;
	centerPoint.y = c_y;
	centerPoint.z = c_z;
	kdtree.radiusSearch(centerPoint, seed_search_radius, seed_candidates, seed_candidates_distances);
	kdtree.radiusSearch(centerPoint, minmax_search_radius, minmax_candidates, minmax_candidates_distances);
	// z坐标最大的顶点作为种子顶点
	float seed_max_z_coord = -1;
	int seed_max_z_index = 0;
	float curvature_threshold = 0.15;
	int seed = 0; // seed_index
	std::cout << "candidates seed count is " << seed_candidates.size() << std::endl;
	if (seed_candidates.size() == 0) {
		return -1;
	}
	for (int i = 0; i < seed_candidates.size(); i++) {
		/*if ((*DON_cloud)[seed_candidates[i]].curvature >= curvature_threshold*0.8){
			continue;
		}*/
		if ((*DON_cloud)[seed_candidates[i]].z > seed_max_z_coord) {
			std::cout << "max z coordinates is " << (*DON_cloud)[seed_candidates[i]].z << std::endl;

			seed_max_z_coord = (*DON_cloud)[seed_candidates[i]].z;
			seed_max_z_index = seed_candidates[i];
		}
		
	}

	// search min max height
	for (int i = 0; i < minmax_candidates.size(); i++) {
		min_z_coord = std::min(min_z_coord, (*DON_cloud)[minmax_candidates[i]].z);
		max_z_coord = std::max(max_z_coord, (*DON_cloud)[minmax_candidates[i]].z);
	}
	// 因为有些顶点的坐标在倾斜的情况下非常的小 这里使用了局部最小点
	float z_threshold = (max_z_coord - min_z_coord) * 0.3 + min_z_coord;//置最低点向上的一个偏移 （最低点变换不会大）

	std::cout << "max z coordinates is " << seed_max_z_coord << std::endl;
	std::queue<int> q;
	std::set<int> s;
	int neighbor_count = 20;
	int seed_neighbor_count = 40;
	pcl::PointIndices::Ptr region(new pcl::PointIndices());
	region->indices.push_back(seed_max_z_index);
	q.push(seed_max_z_index);
	s.insert(seed_max_z_index);
	bool is_seed = true;
	while (!q.empty()) {
		int curqueuesize = q.size();
		for (int i = 1; i <= curqueuesize; i++) {
			
			int curpoint_idx = q.front(); q.pop();
			pcl::PointNormal curpoint = (*DON_cloud)[curpoint_idx];

			// find this point neighbor
			std::vector<int> neighbors;
			std::vector<float> neighbors_distance;
			int cur_neighbor_count = is_seed ? seed_neighbor_count : neighbor_count; // for seed ,we use more neighbors
			if (kdtree.nearestKSearch(curpoint, cur_neighbor_count, neighbors, neighbors_distance) > 0) {
				// 如果当前的顶点不是种子节点，判断邻居中是否含有曲率在范围外的顶点，如果存在那么将所有的邻居顶点全部不进行考虑，to 不进行过度扩展
				bool ignore_it = false;
				/*if (!is_seed) {
					for (int i = 0; i < neighbors.size(); i++) {
						if ((*DON_cloud)[neighbors[i]].curvature >= curvature_threshold) {
							ignore_it = true;
							break;
						}
					}
				}*/
				
				for (int j = 0; j < neighbors.size(); j++) {
					if (ignore_it) {
						s.insert(neighbors[j]);
						continue;
					}
					// seed we want all its neighbor
					if (is_seed) {
						s.insert(neighbors[j]);
						q.push(neighbors[j]);
						region->indices.push_back(neighbors[j]);
						continue;
					}
					

					if (s.count(neighbors[j]) != 0) {
						continue;
					}
					//std::cout << neighbors_distance[j] << std::endl;
					if (neighbors_distance[j] > 216.0f) { //距离使用的是平方距离
						s.insert(neighbors[j]);
						continue; // 距离不能够超过一定的值
					}
					// 进行条件判断
					if ((*DON_cloud)[neighbors[j]].curvature >= curvature_threshold) {
						s.insert(neighbors[j]);
						continue;
					}
					if ((*DON_cloud)[neighbors[j]].z <= z_threshold) {
						s.insert(neighbors[j]);
						continue;
					}

					s.insert(neighbors[j]);
					q.push(neighbors[j]);
					region->indices.push_back(neighbors[j]);
				}
				is_seed = false;

			}


		}
	}
	pcl::PointCloud<pcl::PointXYZ>::Ptr _cloud(new pcl::PointCloud<pcl::PointXYZ>);

	pcl::ExtractIndices<pcl::PointXYZ> extract;
	//保存到新的点云中
	extract.setInputCloud(cloud);
	extract.setNegative(false);
	extract.setIndices(region);
	extract.filter(*_cloud);

	pcl::io::savePCDFile(outputcloudname, *_cloud);
	return 0;
}

int main() {
	std::string inputcloudfolder;
	std::string outputcloudfolder;
	
	// 先只拿一个文件夹中的进行测试
	std::string stone_pcdfile_dir = "D:\\gt_Points_2.64\\";
	std::string stone_refine_dir = "D:\\gt_Points_2.64_extract\\";
	std::string txt_folder_saved = "D:\\gt_points_gen_2.64_text\\";
	std::string txt_folder_offground = "D:\\gt_points_gen_2.64_text_off_ground\\";
	std::vector<std::string> all_stone_pcd;
    get_all_abspath_within_folder(stone_pcdfile_dir, all_stone_pcd);
	for (int i = 0; i < all_stone_pcd.size(); i++) {
		/// 测试结束
		std::string pcdfullpath = all_stone_pcd[i];
		std::size_t botDirPos = pcdfullpath.find_last_of("\\");
		std::string pcdname = pcdfullpath.substr(botDirPos + 1, pcdfullpath.length());
		std::string pcdpath = pcdfullpath.substr(0, botDirPos);
		std::size_t botDirPos_2  = pcdpath.find_last_of("\\");
		std::string lastsubdir = pcdpath.substr( botDirPos_2+1,pcdpath.length());
		std::cout << pcdname << std::endl;
		std::cout << pcdpath << std::endl;
		std::cout << lastsubdir << std::endl;
		//new to be stored pcd name
		std::string extractedpcdpath = stone_refine_dir + "\\" + lastsubdir+"\\";
		std::string extractedpcdfullpath = extractedpcdpath + pcdname;
		// 计算加载点云的法线
		// std::string pcdnameprefix = pcdname.find_last_of('.');
		int pcdname_dot_position = pcdname.find_last_of('.');
		std::string pcdname_prefix = pcdname.substr(0,pcdname_dot_position+1);
		std::cout << pcdname_prefix << std::endl;
		CreateDirectory(extractedpcdpath.c_str(), NULL);
		CreateDirectory((txt_folder_saved + "\\" + lastsubdir).c_str(), NULL);
		CreateDirectory((txt_folder_offground + "\\" + lastsubdir).c_str(), NULL);
		std::cout<< extractedpcdfullpath  <<std::endl;
		pcl::PointCloud<pcl::PointXYZ>::Ptr offground_cloud = extractOffGround(pcdfullpath, extractedpcdfullpath,txt_folder_saved+"\\"+lastsubdir,txt_folder_offground+"\\"+lastsubdir, pcdname_prefix);
		if (offground_cloud==NULL || offground_cloud->points.size() == 0) {
			continue;
		}
		region_growing(offground_cloud, extractedpcdfullpath);
	}
	return 0;
}