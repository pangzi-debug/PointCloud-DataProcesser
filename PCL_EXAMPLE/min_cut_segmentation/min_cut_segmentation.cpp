#include <iostream>
#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/filter_indices.h> // for pcl::removeNaNFromPointCloud
#include <pcl/segmentation/min_cut_segmentation.h>
#include <pcl/search/kdtree.h>
#include <pcl/common/common.h>
int main (int argc, char** argv)
{
    pcl::PointCloud <pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud <pcl::PointXYZ>);
    if ( pcl::io::loadPCDFile <pcl::PointXYZ> (argv[1] , *cloud) == -1 )
    {
        std::cout << "Cloud reading failed." << std::endl;
        return (-1);
    }

    pcl::IndicesPtr indices (new std::vector <int>);
    pcl::removeNaNFromPointCloud(*cloud, *indices);

    pcl::MinCutSegmentation<pcl::PointXYZ> seg;
    seg.setInputCloud (cloud);
    seg.setIndices (indices);

    pcl::PointCloud<pcl::PointXYZ>::Ptr foreground_points(new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::PointXYZ centerPoint;
	pcl::PointXYZ minPt, maxPt;
	float forgrountPointSearchRadius = std::min((maxPt.x - minPt.x), (maxPt.y - minPt.y)) / 3.0;
	pcl::getMinMax3D(*cloud, minPt, maxPt);
	centerPoint.x = (maxPt.x + minPt.x) / 2.0;
	centerPoint.y = (maxPt.y + minPt.y) / 2.0;
	centerPoint.z = (maxPt.z + minPt.z) / 2.0;
	// todo ���������洢�ھ� �� �ھӾ���� ����
	// ��ǰ��һ�����ĵ���ߵ���Ϊǰǰ����
	std::vector<int> foreground_candidates;
	std::vector<float> foreground_candidates_distances;
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(cloud);
	kdtree.radiusSearch(centerPoint, forgrountPointSearchRadius, foreground_candidates, foreground_candidates_distances);
	pcl::PointXYZ foreground_point;
	float max_z = -1;
	for (int i = 0; i < foreground_candidates.size(); i++) {
		pcl::PointXYZ curPoint = (*cloud)[foreground_candidates[i]];
		if (curPoint.z > max_z) {
			max_z = curPoint.z;
			foreground_point = curPoint;
		}
		
	}

    foreground_points->points.push_back(foreground_point);
    seg.setForegroundPoints (foreground_points);

    seg.setSigma (0.25); // sigma �ǹ⻬�̶ȵĲ���
    seg.setRadius (forgrountPointSearchRadius); // �뾶��ʾ��ǰ����İ뾶
    seg.setNumberOfNeighbours (14);
    seg.setSourceWeight (0.8);

    std::vector <pcl::PointIndices> clusters;
    seg.extract (clusters);
    std::cout << "Maximum flow is " << seg.getMaxFlow () << std::endl;

    pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = seg.getColoredCloud ();
    pcl::visualization::CloudViewer viewer ("Cluster viewer");
    viewer.showCloud(colored_cloud);
    while (!viewer.wasStopped ())
    {
    }

    return (0);
}