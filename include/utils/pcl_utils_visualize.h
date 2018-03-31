//
// Created by vincentlv on 17-3-16.
//

#ifndef VISION_PCL_UTILS_VISUALIZE_H
#define VISION_PCL_UTILS_VISUALIZE_H

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

template <typename PointCloudPtr>
void viewPointCloud(PointCloudPtr cloud, const std::string viewer_name = "Simple Cloud Viewer")
{
    pcl::visualization::CloudViewer viewer (viewer_name);
    viewer.showCloud (cloud);
    while (!viewer.wasStopped ())
    {
    }
}

#endif //VISION_PCL_UTILS_VISUALIZE_H
