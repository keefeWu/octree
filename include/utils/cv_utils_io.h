//
// Created by vincentlv on 17-3-20.
//

#ifndef VISION_CV_UTILS_IO_H
#define VISION_CV_UTILS_IO_H

// #include <algorithm>
#include <string>

#include "fileutils.hpp"

const std::vector<std::string> IMAGE_EXT{ ".jpg", ".jpeg", ".jpe", ".bmp", ".gif", ".png" };
const std::vector<std::string> VIDEO_EXT{ ".mp4", ".avi" };


inline std::vector<std::string> get_files_with_image_ext(std::string folder)
{
	return futils::get_files_with_extensions(folder, IMAGE_EXT, false);   // case insensitive
}

inline std::vector<std::string> get_files_with_video_ext(std::string folder)
{
	return futils::get_files_with_extensions(folder, VIDEO_EXT, false);   // case insensitive
}

inline bool is_image_file(std::string file_path)
{
    std::string ext_ = "." + futils::get_file_extension(file_path);  // convert to lowercase
	std::transform(ext_.begin(), ext_.end(), ext_.begin(), ::tolower);
    return std::find(IMAGE_EXT.begin(), IMAGE_EXT.end(), ext_) != IMAGE_EXT.end();
}

inline bool is_video_file(std::string file_path)
{
    std::string ext_ = "." + futils::get_file_extension(file_path);  // convert to lowercase
	std::transform(ext_.begin(), ext_.end(), ext_.begin(), ::tolower);
    return std::find(VIDEO_EXT.begin(), VIDEO_EXT.end(), ext_) != VIDEO_EXT.end();
}

#endif //VISION_CV_UTILS_IO_H
