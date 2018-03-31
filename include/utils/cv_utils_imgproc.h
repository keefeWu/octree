//
// Created by vincentlv on 17-3-20.
//

#ifndef VISION_CV_UTILS_IMGPROC_H
#define VISION_CV_UTILS_IMGPROC_H

#include <opencv2/opencv.hpp>

void normalize_luminance(const cv::Mat& src, cv::Mat& dst)
{
	// // convert to YUV colorspace to normalize luminance by performing hist equalization on Y channel

 //    cv::cvtColor(src, dst, CV_YUV2BGR); 

	// // convert YUV back to RGB
	// cv::cvtColor(dst, dst, CV_BGR2YUV); 	

	// convert to Lab colorspace to normalize luminance by performing adaptive hist equalization (CLAHE) on luminance L channel	
	cv::Mat lab_image;
	cv::cvtColor(src, lab_image, CV_BGR2Lab);

	// extract L channel
	std::vector<cv::Mat> lab_planes(3);
	cv::split(lab_image, lab_planes);

	// apply the CLAHE algorithm to normalize L channel
	cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
	clahe->setClipLimit(4);
	cv::Mat l_channel;
	clahe->apply(lab_planes[0], l_channel);

	// merge color planes back to Lab image
	l_channel.copyTo(lab_planes[0]);
	cv::merge(lab_planes, lab_image);

	// convert back to BGR
	cv::cvtColor(lab_image, dst, CV_Lab2BGR);
}

//bool isGrayImage( const cv::Mat& img ) // returns true if the given 3 channel image is B = G = R
//{
////    cv::Mat dst;
////    cv::Mat bgr[3];
////    cv::split( img, bgr );
////    cv::absdiff( bgr[0], bgr[1], dst );
////
////    if(cv::countNonZero( dst ))
////        return false;
////
////    cv::absdiff( bgr[0], bgr[2], dst );
////    return !cv::countNonZero( dst );
//    return img.channels() == 1;
//}

#endif //VISION_CV_UTILS_IMGPROC_H
