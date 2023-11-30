
#ifndef LIB_H
#define LIB_H

#include <iostream>
#include <string>
#include <vector>
#include <filesystem>
#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/kdtree.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/features/normal_3d.h>
#include <pcl/PointIndices.h>

const std::string basePath = "dataset";


struct SegmentInfo {
    pcl::PointXYZ previousPosition;
    pcl::PointXYZ currentPosition;
    bool isVisible = true;
    bool isDetected = false;
    int numOccurrences = 0;
    int segmentSize;
};

extern std::vector<SegmentInfo> segmentInfoList;

double calculateDistance(const pcl::PointXYZ& a, const pcl::PointXYZ& b); //Calculates the distance between 2 3D points
bool arePointsEqual(const pcl::PointXYZ& a, const pcl::PointXYZ& b); //Check equity of the pcl::PointXYZs
pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb_depth_to_point_cloud(const std::string& rgb_file, const std::string& depth_file);//Gets RGB and Depth images and convert them to a point cloud
void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event, void* viewer_void);//Closing callback is activated with pressing 'Esc'
void visualize_cloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, const std::string& viewer_name);//Visualizes point clouds
void compute_normals(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, pcl::PointCloud<pcl::Normal>::Ptr& cloud_normals);//Computes normals of the cloud
void rgb_segmentation(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& colored_cloud, std::vector<pcl::PointIndices>& clusters);//Rgb segmentation on the point cloud, uses region growing algorithm.
void find_horizontal_planes(const pcl::PointCloud<pcl::Normal>::Ptr& cloud_normals,
                            const std::vector<pcl::PointIndices>& clusters,
                            std::vector<pcl::PointIndices>& horizontal_plane_clusters); //Finds horizontally flat surfaces in the image
void visualize_clusters(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
                        const std::vector<pcl::PointIndices>& clusters,
                        const std::vector<pcl::PointXYZ>& previous_centers,
                        const std::vector<pcl::PointXYZ>& current_centers); //Visualizes segmented Rgb clusters on the point cloud
pcl::PointXYZ calculateClusterCenter(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, const pcl::PointIndices& cluster);//Calculates center of a cluster
std::pair<std::vector<std::string>, std::vector<std::string>> find_files_by_extensions(const std::string& path, const std::string& ext1, const std::string& ext2); //Finds files under basePath by checking extensions
void process_frames(const std::string& basePath, const std::vector<std::string>& rgb_filenames, const std::vector<std::string>& depth_filenames, int num_frames); //Processes two frames. All functions are called in this function.


#endif
