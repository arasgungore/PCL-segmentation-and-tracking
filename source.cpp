#include "lib.h"

double calculateDistance(const pcl::PointXYZ& a, const pcl::PointXYZ& b) {
    //Calculates the distance between 2 3D points
    return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2) + pow(a.z - b.z, 2));
}

bool arePointsEqual(const pcl::PointXYZ& a, const pcl::PointXYZ& b) {
    //Check equity of the pcl::PointXYZs
    return a.x == b.x && a.y == b.y && a.z == b.z;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb_depth_to_point_cloud(const std::string& rgb_file, const std::string& depth_file) {
    //Gets RGB and Depth images and convert them to a point cloud
    cv::Mat rgb_img;
    rgb_img = cv::imread(rgb_file, cv::IMREAD_COLOR);
    if (!rgb_img.data) {
        std::cout << "Could not open or find the image" << std::endl;
        return nullptr;
    }

    cv::Mat depth_img;
    depth_img = cv::imread(depth_file, cv::IMREAD_ANYDEPTH);
    if (!depth_img.data) {
        std::cout << "Could not open or find the depth image" << std::endl;
        return nullptr;
    }

    // Resize RGB image to match depth image size
    cv::resize(rgb_img, rgb_img, depth_img.size());

    // After loading the depth image, create a mask of missing values
    cv::Mat depth_mask = (depth_img == 0) | (depth_img > 5000);  // Adjust these conditions

    // Inpaint the missing depth values
    cv::Mat depth_img_inpainted;
    cv::inpaint(depth_img, depth_mask, depth_img_inpainted, 3, cv::INPAINT_NS);
    depth_img = depth_img_inpainted;  // Use inpainted image

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    // Camera intrinsic parameters
    float fx = 1169.62f;
    float fy = 1167.11f;
    float cx = 646.295f;
    float cy = 489.927f;

    // Scale intrinsic parameters to match new image size
    float scale_x = (float)depth_img.cols / rgb_img.cols;
    float scale_y = (float)depth_img.rows / rgb_img.rows;

    fx *= scale_x;
    fy *= scale_y;
    cx *= scale_x;
    cy *= scale_y;

    // Define depth range for valid points
    float min_depth = 0.0f;  // in meters
    float max_depth = 5.0f;  // in meters

    for (int i = 0; i < depth_img.rows; ++i) {
        for (int j = 0; j < depth_img.cols; ++j) {
            float depth = depth_img.at<uint16_t>(i, j) / 1000.0f;  // convert to meters

            // Skip if depth is out of the defined range
            if (depth < min_depth || depth > max_depth) {
                continue;
            }

            pcl::PointXYZRGB point;
            point.x = (j - cx) * depth / fx;
            point.y = (i - cy) * depth / fy;
            point.z = depth;

            cv::Vec3b rgb = rgb_img.at<cv::Vec3b>(i, j);
            uint32_t rgb_val = ((uint32_t)rgb[2] << 16 | (uint32_t)rgb[1] << 8 | (uint32_t)rgb[0]);
            point.rgb = *reinterpret_cast<float*>(&rgb_val);
            cloud->push_back(point);
        }
    }

    return cloud;
}

void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event, void* viewer_void) {
    //Closing callback is activated with pressing 'Esc'
    if (event.getKeySym() == "Escape" && event.keyDown()) {
        pcl::visualization::PCLVisualizer *viewer = static_cast<pcl::visualization::PCLVisualizer *>(viewer_void);
        viewer->close();
    }
}


void visualize_cloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, const std::string& viewer_name) {
    //Visualizes point clouds
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer(viewer_name));
    viewer->registerKeyboardCallback(keyboardEventOccurred, (void*)viewer.get());

    viewer->removeAllPointClouds();
    viewer->removeAllShapes();
    viewer->addPointCloud<pcl::PointXYZRGB>(cloud, "scene_cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "scene_cloud");
    viewer->addCoordinateSystem(1.0, "axis", 0);

    while (!viewer->wasStopped()) {
        viewer->spinOnce();
    }
}

void compute_normals(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, pcl::PointCloud<pcl::Normal>::Ptr& cloud_normals) {
    //Computes normals of the cloud
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
    ne.setInputCloud(cloud);

    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
    ne.setSearchMethod(tree);

    ne.setRadiusSearch(0.03);
    ne.compute(*cloud_normals);
}

void rgb_segmentation(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& colored_cloud, std::vector<pcl::PointIndices>& clusters) {
    pcl::search::Search<pcl::PointXYZRGB>::Ptr tree = std::make_shared<pcl::search::KdTree<pcl::PointXYZRGB>>();
    pcl::RegionGrowingRGB<pcl::PointXYZRGB> reg;
    reg.setInputCloud(cloud);
    reg.setSearchMethod(tree);

    reg.setDistanceThreshold(0.03);
    reg.setPointColorThreshold(3);
    reg.setRegionColorThreshold(2.5);
    reg.setMinClusterSize(5000);

    reg.extract(clusters);
    colored_cloud = reg.getColoredCloud();

}



void find_horizontal_planes(const pcl::PointCloud<pcl::Normal>::Ptr& cloud_normals,
                            const std::vector<pcl::PointIndices>& clusters,
                            std::vector<pcl::PointIndices>& horizontal_plane_clusters)
{
    // Threshold for detecting horizontal planes (adjustable)
    double dot_product_threshold_x = 0.6;
    double dot_product_threshold_y = 0.7;

    for (const auto& cluster : clusters) {
        // Calculate average normal for the cluster
        pcl::Normal avg_normal;
        avg_normal.normal_x = 0;
        avg_normal.normal_y = 0;
        avg_normal.normal_z = 0;
        for (const auto& idx : cluster.indices) {
            avg_normal.normal_x += cloud_normals->points[idx].normal_x;
            avg_normal.normal_y += cloud_normals->points[idx].normal_y;
            avg_normal.normal_z += cloud_normals->points[idx].normal_z;
        }
        avg_normal.normal_x /= cluster.indices.size();
        avg_normal.normal_y /= cluster.indices.size();
        avg_normal.normal_z /= cluster.indices.size();

        // Check if the average normal is close to horizontal (y-axis)
        if (std::abs(avg_normal.normal_x) < dot_product_threshold_x && std::abs(avg_normal.normal_y) > dot_product_threshold_y) {
            // If the cluster is determined to be a horizontal plane, add the cluster to the list of horizontal plane clusters
            horizontal_plane_clusters.push_back(cluster);
        }

    }
}

void visualize_clusters(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
                        const std::vector<pcl::PointIndices>& clusters,
                        const std::vector<pcl::PointXYZ>& previous_centers,
                        const std::vector<pcl::PointXYZ>& current_centers) {
    // Create the visualization object and add the original cloud to it
    pcl::visualization::PCLVisualizer viewer("3D Viewer");
    viewer.setBackgroundColor(0, 0, 0);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
    viewer.addPointCloud<pcl::PointXYZRGB>(cloud, rgb, "input cloud");

    // Register the keyboard callback
    viewer.registerKeyboardCallback(keyboardEventOccurred, (void*)&viewer);

    // Colorize each cluster and add it to the viewer
    unsigned int j = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = clusters.begin(); it != clusters.end(); ++it) {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
        for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
            cluster->points.push_back(cloud->points[*pit]);
        cluster->width = cluster->points.size();
        cluster->height = 1;
        cluster->is_dense = true;

        // Generate a unique string for the cluster ID
        std::string id = "cluster-" + std::to_string(j);

        // Colorize the cluster and add it to the viewer
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> cluster_color_handler(cluster, rand() % 256, rand() % 256, rand() % 256);
        viewer.addPointCloud<pcl::PointXYZRGB>(cluster, cluster_color_handler, id);

        // Add a sphere for the current center
        pcl::PointXYZ current_center = current_centers[j];
        viewer.addSphere(current_center, 0.01, 1.0, 0.0, 0.0, "current_center_" + std::to_string(j));

        // If there was a previous frame and centers are not empty, add a sphere for the previous center
        if (!previous_centers.empty()) {
            pcl::PointXYZ previous_center;
            if (j < previous_centers.size()) {
                previous_center = previous_centers[j];
            } else {
                previous_center = current_center; // Use the same center as current if no previous center available
            }
            viewer.addSphere(previous_center, 0.01, 0.0, 1.0, 0.0, "previous_center_" + std::to_string(j));

            // Add a cylinder connecting the previous center to the current center
            viewer.addLine(previous_center, current_center, 0.5, 0.5, 0.5, "center_link_" + std::to_string(j));
        }

        // Initialize the min and max points
           pcl::PointXYZRGB min_pt, max_pt;
           min_pt.x = min_pt.y = min_pt.z = std::numeric_limits<float>::max();
           max_pt.x = max_pt.y = max_pt.z = -std::numeric_limits<float>::max();

           // Iterate over the points to find the minimum and maximum x, y, and z
           for (const auto& point : cluster->points) {
               min_pt.x = std::min(min_pt.x, point.x);
               min_pt.y = std::min(min_pt.y, point.y);
               min_pt.z = std::min(min_pt.z, point.z);
               max_pt.x = std::max(max_pt.x, point.x);
               max_pt.y = std::max(max_pt.y, point.y);
               max_pt.z = std::max(max_pt.z, point.z);
           }

           // Add a bounding box around the cluster (semi-transparent)
           viewer.addCube(min_pt.x, max_pt.x,
                          min_pt.y, max_pt.y,
                          min_pt.z, max_pt.z,
                          1.0, 0.0, 0.0, id + "_box");
           viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
                                              pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME,
                                              id + "_box");
           viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,
                                              1.0, 0.0, 0.0, id + "_box");
           viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY,
                                              0.7, id + "_box");


        j++;
    }

    // Add the legend to the viewer
    viewer.addText("Black: Input Cloud\nColored: Clustered Cloud\nRed Sphere: Current Centers\nGreen Sphere: Previous Centers\nGrey Line: Connection", 10, 15, "legend", 0);

    // Spin the viewer until 'q' is pressed
    while (!viewer.wasStopped()) {
        viewer.spinOnce();
    }
}


pcl::PointXYZ calculateClusterCenter(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, const pcl::PointIndices& cluster) {
    //Calculation of the center of a cluster
    pcl::PointXYZ center;
    center.x = 0;
    center.y = 0;
    center.z = 0;
    for (const auto& index : cluster.indices) {
        center.x += cloud->points[index].x;
        center.y += cloud->points[index].y;
        center.z += cloud->points[index].z;
    }
    center.x /= cluster.indices.size();
    center.y /= cluster.indices.size();
    center.z /= cluster.indices.size();
    return center;
}

std::pair<std::vector<std::string>, std::vector<std::string>> find_files_by_extensions(const std::string& path, const std::string& ext1, const std::string& ext2) {
    //Finds files under basePath by checking extensions
    std::vector<std::string> filenames1, filenames2;
    for (const auto& entry : std::filesystem::directory_iterator(path)) {
        if (entry.is_regular_file()) {
            std::string extension = entry.path().extension().string();
            if (extension == ext1) {
                filenames1.push_back(entry.path().filename().string());
            } else if (extension == ext2) {
                filenames2.push_back(entry.path().filename().string());
            }
        }
    }

    std::sort(filenames1.begin(), filenames1.end());
    std::sort(filenames2.begin(), filenames2.end());

    return std::make_pair(filenames1, filenames2);
}

void process_frames(const std::string& basePath, const std::vector<std::string>& rgb_filenames, const std::vector<std::string>& depth_filenames, int num_frames) {
    //Processes two frames. All functions are called in this function.


    std::vector<pcl::PointXYZ> previous_frame_centers;
    std::vector<pcl::PointXYZ> previous_horizontal_plane_centers;
    std::stringstream rgb_ss, depth_ss;

    // Loop over frames
    for (int frame = 0; frame < num_frames; frame += 19)
    {
        rgb_ss << basePath << "/" << rgb_filenames.at(frame);
        depth_ss << basePath << "/" << depth_filenames.at(frame);
        // Calling the conversion function
        auto cloud = rgb_depth_to_point_cloud(rgb_ss.str(), depth_ss.str());

        // Visualize original cloud
        visualize_cloud(cloud, "3D Viewer Original");

        // Segment the cloud into different regions based on color
        std::vector<pcl::PointIndices> clusters;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
        rgb_segmentation(cloud, colored_cloud, clusters);
        std::cout << "Number of detected segments: " << clusters.size() << std::endl;

        // Calculate centers of the clusters in the current frame
        std::vector<pcl::PointXYZ> current_frame_centers;
        for (const auto& cluster : clusters) {
            current_frame_centers.push_back(calculateClusterCenter(colored_cloud, cluster));
        }

        // Compare centers from the current and previous frame
        if (!previous_frame_centers.empty()) {
            double threshold = 1.0;  // Adjust this according to your data
            for (const auto& previous_center : previous_frame_centers) {
                for (const auto& current_center : current_frame_centers) {
                    if (calculateDistance(previous_center, current_center) < threshold) {
                        std::cout << "Object tracked between frames.\n";
                    }
                }
            }
        }

        // Update the centers for the next frame
        previous_frame_centers = current_frame_centers;

        // Visualize the segmented clusters
        visualize_cloud(colored_cloud, "3D Viewer Segmented");

        // Compute the normals
        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>());
        compute_normals(cloud, cloud_normals);
        std::vector<pcl::PointIndices> horizontal_plane_clusters;
        find_horizontal_planes(cloud_normals, clusters, horizontal_plane_clusters);
        // Calculate centers of the horizontal planes in the current frame
        std::vector<pcl::PointXYZ> horizontal_plane_centers;
        for (const auto& cluster : horizontal_plane_clusters) {
            horizontal_plane_centers.push_back(calculateClusterCenter(cloud, cluster));
        }
        // Visualize the horizontal plane clusters
        if (!previous_horizontal_plane_centers.empty()) {
            visualize_clusters(cloud, horizontal_plane_clusters, previous_horizontal_plane_centers, horizontal_plane_centers);
        } else {
            visualize_clusters(cloud, horizontal_plane_clusters, horizontal_plane_centers,horizontal_plane_centers);
        }
        // Update the horizontal plane centers for the next frame
        previous_horizontal_plane_centers = horizontal_plane_centers;

        rgb_ss.str("");
        rgb_ss.clear();
        depth_ss.str("");
        depth_ss.clear();
    }
}
