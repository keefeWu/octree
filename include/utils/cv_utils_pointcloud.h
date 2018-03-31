//
// Created by vincentlv on 17-3-16.
//

#ifndef VISION_CV_UTILS_POINTCLOUD_H
#define VISION_CV_UTILS_POINTCLOUD_H

#include <cassert>

#include <opencv2/opencv.hpp>

//#include <pcl/common/
#include <pcl/point_cloud.h>
#include <pcl/point_traits.h>

template <typename PointT>
inline void get_gray_image_from_pointcloud(cv::Mat& img, const pcl::PointCloud<PointT>& pc_in)
{
    img = cv::Mat(pc_in.height, pc_in.width, CV_8UC1);

    for (int i =0; i < pc_in.size(); ++i){
        const PointT& p_PC = pc_in.at(i);
        img.at<uchar>(i) = 0.299 * p_PC.r + 0.587 * p_PC.g + 0.114 * p_PC.b;
    }
}

template <typename PointT>
inline void get_rgb_image_from_pointcloud(cv::Mat& img, const pcl::PointCloud<PointT>& pc_in)
{
    img = cv::Mat(pc_in.height, pc_in.width, CV_8UC3);

    unsigned int img_idx = 0;
    for (int i =0; i < pc_in.size(); ++i){
        const PointT& p_PC = pc_in.at(i);
        img.at<uchar>(img_idx) = p_PC.b;
        img.at<uchar>(img_idx + 1) = p_PC.g;
        img.at<uchar>(img_idx + 2) = p_PC.r;
        img_idx += 3;
    }
}

template <typename PointT>
inline void get_mask_from_pointcloud(cv::Mat& img, const pcl::PointCloud<PointT>& pc_in, pcl::PointIndices::Ptr mask_indices)
{
    std::vector<int> indices;

    const int idx_size = mask_indices->indices.size();
    indices.reserve(idx_size);
    for (int i = 0; i < idx_size; ++i)
    {
        indices.push_back(mask_indices->indices[i]);
    }
    get_mask_from_pointcloud(img, pc_in, indices);
}

template <typename PointT>
inline void get_mask_from_pointcloud(cv::Mat& img, const pcl::PointCloud<PointT>& pc_in, const std::vector<int>& mask_indices)
{
    img = cv::Mat::zeros(pc_in.height, pc_in.width, CV_8UC1);

    // unsigned int img_idx = 0;
    for (int i = 0; i < mask_indices.size(); ++i)
    {
        // const PointT& p_PC = pc_in.at(i);
        int idx = mask_indices[i];
        img.at<uchar>(idx) = 255;
    }
}

template <typename PointT>
std::vector<int> get_pointcloud_from_mask(pcl::PointCloud<PointT>& subcloud, const pcl::PointCloud<PointT>& cloud, const cv::Mat& mask, const int mask_value=255)
{
    assert(mask.channels() == 1);
    assert(mask.rows <= cloud.height && mask.cols <= cloud.width);

    std::vector<int> subcloud_indices;  // save the indices of the extracted points
    for (int r = 0; r < mask.rows; r += 1) 
    {
        for (int c = 0; c < mask.cols; c += 1) 
        {
            if (mask.at<unsigned char>(r,c) == mask_value)
            {
                int pos = cloud.width * r + c;
                const PointT& pt = cloud.points.at(pos);
                subcloud.points.push_back(pt);
                subcloud_indices.push_back(pos);
            }
        }
    }

    return subcloud_indices;
}

#endif //VISION_CV_UTILS_POINTCLOUD_H
