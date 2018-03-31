
#ifndef VISION_CV_UTILS_2D_H
#define VISION_CV_UTILS_2D_H

#include <opencv2/opencv.hpp>

#include "cv_utils.h"


template <typename T>
inline float get_2d_angle_rad(const T& pt1, const T& pt2){
//    T pt_dot = pt1.dot(pt2);
    return atan2(pt2.y - pt1.y, pt2.x - pt1.x);
}

template <typename T>
inline float get_2d_angle_degree(const T& pt1, const T& pt2){
    float angle = get_2d_angle_rad(pt1, pt2);
    return RAD2DEG(angle);
}

template <typename T>
T normalize_2d_vector(std::vector<T>& points){

    cv::Scalar mean_scalar = cv::mean(points);
    T _mean_(mean_scalar[0], mean_scalar[1]);

    if (mean_scalar[0] == 0 && mean_scalar[1] == 0)
        return _mean_;

    // subtract centroids from data
    for (int i = 0; i < points.size(); ++i) {
        points[i] -= _mean_;
    }

    return _mean_;
}

template <typename T>
cv::Mat get_2d_vectors_covariance(const std::vector<T>& pts1, const std::vector<T>& pts2){
//// compute a covariance matrix
    auto points1(pts1);
    auto points2(pts2);

    T mean1 = normalize_2d_vector(points1);
    T mean2 = normalize_2d_vector(points2);

    float Cxx = 0.0, Cxy = 0.0, Cyx = 0.0, Cyy = 0.0;
    for (int i = 0; i < points1.size(); i++) {
        Cxx += points1[i].x * points2[i].x;
        Cxy += points1[i].x * points2[i].y;
        Cyx += points1[i].y * points2[i].x;
        Cyy += points1[i].y * points2[i].y;
    }
    cv::Mat Mcov = (cv::Mat_<float>(2, 2) << Cxx, Cxy, Cyx, Cyy);
    return Mcov;
}


template <typename T>
cv::Vec4i get_2d_min_max(const std::vector<T>& pts){
    int max_x1 = pts[0].x, min_x1 = max_x1,
            max_y1 = pts[0].y, min_y1 = max_y1;

    for(int i = 1; i < pts.size(); ++i){
        if (pts[i].x > max_x1) max_x1 = pts[i].x;
        else if (pts[i].x < min_x1) min_x1 = pts[i].x;
        if (pts[i].y > max_y1) max_y1 = pts[i].y;
        else if (pts[i].y < min_y1) min_y1 = pts[i].y;
    }

    return cv::Vec4i(min_x1, min_y1, max_x1, max_y1);
}

template <typename T>
cv::Vec4i get_2d_min_max(const std::vector<T>& pts, cv::Vec4i& min_max_store){

    int max_x1 = pts[0].x, min_x1 = max_x1,
            max_y1 = pts[0].y, min_y1 = max_y1;
    cv::Vec4i indices(0,0,0,0);

    for(int i = 1; i < pts.size(); ++i){
        if (pts[i].x > max_x1){
            max_x1 = pts[i].x;
            indices[2] = i;
        } 
        else if (pts[i].x < min_x1){
            min_x1 = pts[i].x;  
            indices[0] = i;
        } 
        if (pts[i].y > max_y1) {
            max_y1 = pts[i].y; 
            indices[3] = i;
        }
        else if (pts[i].y < min_y1){
            min_y1 = pts[i].y;
            indices[1] = i;
        } 
    }

    min_max_store = cv::Vec4i(min_x1,min_y1,max_x1,max_y1);

    return indices;
}


template <typename T>
int partition_2d_points(std::vector<std::vector<T>> &cluster, const std::vector<T> &pts,
                     double distance_tolerance) {

    // Apply partition
    // All pixels within the radius tolerance distance will belong to the same class (same label)
    std::vector<int> cluster_idx;

    // With functor
    //int n_cluster_idx = partition(pts, cluster_idx, EuclideanDistanceFunctor(th_distance));

    // With lambda function (require C++11)
    int th2 = distance_tolerance * distance_tolerance;
    int n_clusters = cv::partition(pts, cluster_idx, [th2](const T &lhs, const T &rhs) {
        return ((lhs.x - rhs.x) * (lhs.x - rhs.x) + (lhs.y - rhs.y) * (lhs.y - rhs.y)) < th2;
    });


    // You can save all points in the same class in a vector (one for each class), just like findContours
    cluster.resize(n_clusters);
    for (int i = 0; i < pts.size(); ++i) {
        cluster[cluster_idx[i]].push_back(pts[i]);
    }

    return n_clusters;
}

template <typename PointF>
void kmeans_2d_points(std::vector<std::vector<PointF>> &clusters, std::vector<cv::Point2f>& cluster_centers, const std::vector<PointF> &points, const int K)
{
    int attempts = 3;
    std::vector<int> labels;
    cv::Mat centers;

    cv::TermCriteria tc;
    cv::kmeans(points, K, labels, tc, attempts, cv::KMEANS_PP_CENTERS, centers);

    // init memory
    clusters.reserve(K);
    cluster_centers.reserve(K);
    for (int i = 0; i < K; ++i)
    {
        clusters.push_back(std::vector<PointF>{});
    }

    for (int i = 0; i < K; ++i)
    {
        cv::Point2f c_center;
        c_center.x = centers.at<float>( i,0 );  
        c_center.y = centers.at<float>( i,1 );
        cluster_centers.push_back(c_center);
    }

    for (int i=0; i<labels.size(); ++i)
    {
        int idx = labels[i];
        cv::Point2f original_point = points[i];
        clusters[idx].push_back(original_point);
        // cerr << i << " " << idx << " " << original_point << " " << clustered_center << endl;
    }

}

template <typename T>
inline double get_2d_dist(const T& pt1, const T& pt2){
    T diff = pt1 - pt2;
    return cv::sqrt(diff.x*diff.x + diff.y*diff.y);
}
template <typename T>
inline void get_2d_vector_dist(std::vector<double>& norms, std::vector<T> pts1, std::vector<T> pts2){
    norms.reserve(pts1.size() * pts2.size());
    for(int i = 0; i < pts1.size(); ++i){
        for(int j = 0; j < pts2.size(); ++j){
            norms.push_back(get_2d_points_dist(pts1[i], pts2[j]));
        }
    }
}

template <typename T>
void extract_cluster_centroids(std::vector<T> &cluster_centroids,
                          const std::vector<std::vector<T>> &cluster) {
    const int n_clusters = cluster.size();
    cluster_centroids.reserve(n_clusters);
    for (int k = 0; k < n_clusters; ++k) {
        cv::Scalar cluster_mean = cv::mean(cluster[k]);
        cluster_centroids.push_back(T(cluster_mean[0], cluster_mean[1]));
//        int cluster_size = cluster[k].size();
//        int cluster_x_sum = 0;
//        int cluster_y_sum = 0;
//        for (int i = 0; i < cluster_size; ++i) {
//            cluster_y_sum += cluster[k][i].y;
//            cluster_x_sum += cluster[k][i].x;
//        }
//        cluster_centroids.push_back(cv::Point(cluster_x_sum / cluster_size, cluster_y_sum / cluster_size));
    }
}

template <typename T>
void extract_cluster_centroids_bounding_box(std::vector<cv::Rect> &cluster_bounding_boxes,
                                        const std::vector<T> &cluster_means,
                                        const cv::Mat &src, int box_px_width, int box_px_height) {
    int box_px_width_half = box_px_width / 2;
    int box_px_height_half = box_px_height / 2;

    const int n_clusters = cluster_means.size();
    if (n_clusters == 0)
        return;

    cluster_bounding_boxes.reserve(n_clusters);

    // get the bounding boxes
    cv::Mat src_copy = src.clone();
    for (int i = 0; i < n_clusters; ++i) {
        const cv::Point &kp = cluster_means[i];
        int kp_x = kp.x;
        int kp_y = kp.y;
        cv::Point kp_box_top_left(std::max(0, kp_x - box_px_width_half), std::max(0, kp_y - box_px_height_half));
        cv::Point kp_box_bottom_right(std::min(src.cols - 1, kp_x + box_px_width_half),
                                      std::min(src.rows - 1, kp_y + box_px_height_half));

        int bounded_box_width = kp_box_bottom_right.x - kp_box_top_left.x;
        int bounded_box_height = kp_box_bottom_right.y - kp_box_top_left.y;

        // // // PRINT(kp_x)
        // cv::Mat kp_box_img(src_copy, cv::Rect(kp_box_top_left.x,kp_box_top_left.y,box_px_width, box_px_height));
        cluster_bounding_boxes.push_back(
                cv::Rect(kp_box_top_left.x, kp_box_top_left.y, bounded_box_width, bounded_box_height));

    }

}

//template <typename T>
//cv::Mat find_homography(const cv::Mat &template_img_object, const std::vector<T> &image1_keypoints,
//                     const std::vector<T> &image2_keypoints) {
//    double ransacReprojThreshold = 1;
//    int method = LMEDS;
//    H = cv::findHomography(image1_keypoints, image2_keypoints, method, ransacReprojThreshold);
//
//    //-- Get the corners from the image_1 ( the object to be "detected" )
//    std::vector<T> obj_corners(4);
//    obj_corners[0] = T(0, 0);
//    obj_corners[1] = T(template_img_object.cols, 0);
//    obj_corners[2] = T(template_img_object.cols, template_img_object.rows);
//    obj_corners[3] = T(0, template_img_object.rows);
//    std::vector<T> scene_corners(4);
//
//    if (H.empty()) {
//        cout << "Could not find good homography\n";
//        return false;
//    }
//
//    cv::perspectiveTransform(obj_corners, scene_corners, H);
//    for (int i = 0; i < obj_corners.size(); ++i) {
//        PRINT(obj_corners[i])
//        PRINT(scene_corners[i])
//    }
//
//    // //-- Draw lines between the corners (the mapped object in the scene - image_2 )
//    // cv::line( img_matches_mat, scene_corners[0] + cv::Point2f( template_img_object.cols, 0), scene_corners[1] + cv::Point2f( template_img_object.cols, 0), cv::Scalar(0, 255, 0), 4 );
//    // cv::line( img_matches_mat, scene_corners[1] + cv::Point2f( template_img_object.cols, 0), scene_corners[2] + cv::Point2f( template_img_object.cols, 0), cv::Scalar( 0, 255, 0), 4 );
//    // cv::line( img_matches_mat, scene_corners[2] + cv::Point2f( template_img_object.cols, 0), scene_corners[3] + cv::Point2f( template_img_object.cols, 0), cv::Scalar( 0, 255, 0), 4 );
//    // cv::line( img_matches_mat, scene_corners[3] + cv::Point2f( template_img_object.cols, 0), scene_corners[0] + cv::Point2f( template_img_object.cols, 0), cv::Scalar( 0, 255, 0), 4 );
//
//    // //-- Show detected matches
//    // cv::imshow( "Good Matches & Object detection", img_matches_mat );
//
//    // cv::waitKey(0);
//
//    return true;
//}

#endif //VISION_CV_UTILS_H
