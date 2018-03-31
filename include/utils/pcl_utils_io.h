//
// Created by vincentlv on 17-3-16.
//

#ifndef VISION_PCL_UTILS_IO_H
#define VISION_PCL_UTILS_IO_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

#include <iostream>

#define PRINT(a) std::cout << #a << ": " << a << std::endl;

template <typename PointT>
bool load_cloud_from_pcd(pcl::PointCloud<PointT>& pc_in, std::string pcd_file_path)
{
    if (pcl::io::loadPCDFile<PointT> (pcd_file_path, pc_in) == -1) //* load the file
    {
        return false;
    }

    PRINT(pc_in.width)
    PRINT(pc_in.height)

    return true;
}

#endif //VISION_PCL_UTILS_H
