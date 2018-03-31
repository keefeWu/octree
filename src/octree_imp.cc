#include "voxel_img.hh"

Octree::Octree()
{
}

bool Octree::set_bbox(float x_min, float x_max, float y_min, float y_max, float z_min, float z_max, float length)
{
    if(length <= 0)
      return false;
    if(x_max_ <= x_min_)
      return false;
    if(y_max_ <= y_min_)
      return false;
    if(z_max_ <= z_min_)
      return false;
    x_min_ = x_min;
    y_min_ = y_min;
    z_min_ = z_min;
    x_max_ = x_max;
    y_max_ = y_max;
    z_max_ = z_max;
    length_ = length;
    create_octree();
    return true;
}

bool Octree::create_octree()
{
    voxel_vector_.reserve((x_max_ - x_min_)/length_ * (y_max_ - y_min_)/length_ * (z_max_ - z_min_)/length_);
    return true;
}

bool Octree::set_points(pcl::PointCloud<PointT>::Ptr cloud)
{
    for(int idx = 0; idx < cloud->size(); idx++)
    {
        voxel_vector_[get_voxel_id(cloud->points[idx])].points.push_back(cloud->points[idx]);
    }
    return true;
}


int Octree::get_voxel_id(PointT cloud)
{
    int idx = (cloud.x - x_min_) / length_ * (int)((y_max_ - y_min_) / length_) * (int)((z_max_ - z_min_) / length_) + 
                (cloud.y - y_min_) / length_ * (int)((z_max_ - z_min_) / length_) + 
                 (cloud.z - z_min_) / length_;
    return idx;
}

PointT Octree::get_voxel_position(int idx)
{
    int idx = (cloud.x - x_min_) / length_ * (int)((y_max_ - y_min_) / length_) * (int)((z_max_ - z_min_) / length_) + 
                (cloud.y - y_min_) / length_ * (int)((z_max_ - z_min_) / length_) + 
                 (cloud.z - z_min_) / length_;
    return idx;
}

