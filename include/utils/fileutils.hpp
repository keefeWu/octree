//
// Created by vincentlv on 17-3-20.
//

#ifndef __FILEUTILS_HPP__
#define __FILEUTILS_HPP__



#if defined (_WIN32)
#include <Windows.h>
#include <string.h>
#else
#include <glob.h>
#endif
#include <string>
#include <vector>

#include <algorithm>

namespace futils {

struct PathSeparator
{
	bool operator()(char ch) const
	{
#if defined (_WIN32)
		return ch == '\\' || ch == '/';
#else
		return ch == '/';
#endif 
	}
};

inline std::vector<std::string> get_files_in_folder(std::string folder, const std::string& wildcard= "*") {
	std::vector<std::string> files;
	std::string pattern = folder + "/" + wildcard;
#if defined (_WIN32)
	WIN32_FIND_DATA fd;

	HANDLE hFind = FindFirstFile(pattern.c_str(), &fd);
	if (hFind != INVALID_HANDLE_VALUE) {
		do {
			// read all (real) files in current folder
			// , delete '!' read other 2 default folder . and ..
			if (!(fd.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY)) {
				files.push_back(folder + "/" + fd.cFileName);
			}
		} while (::FindNextFile(hFind, &fd));
		::FindClose(hFind);
	}

#else
	glob_t glob_result;
	glob(pattern.c_str(), GLOB_TILDE, NULL, &glob_result);

	for (unsigned int i = 0; i < glob_result.gl_pathc; ++i) {
		files.push_back(std::string(glob_result.gl_pathv[i]));
	}
	globfree(&glob_result);
#endif
	return files;
}

inline std::string get_file_extension(const std::string& file_name)
{
	std::string::size_type idx;

	idx = file_name.rfind('.');

	std::string extension = "";

	if (idx != std::string::npos)
	{
		extension = file_name.substr(idx + 1);
	}

	return extension;
}

inline std::string get_file_basename(const std::string& file_name)
{
	return std::string(
		std::find_if(file_name.rbegin(), file_name.rend(),
			PathSeparator()).base(),
		file_name.end());

}

inline std::string get_file_folder(const std::string& file_name)
{
	std::string bname = get_file_basename(file_name);
	std::string foldername = file_name.substr(0, file_name.find(bname));
	int foldername_len = foldername.length();
	if (foldername_len == 0)
		return foldername;
	return foldername.substr(0,foldername_len-1);  // remove trailing /
}

inline std::vector<std::string> get_files_with_extensions(const std::string& folder, const std::vector<std::string>& extensions, bool case_sensitive = true)
{
	std::vector<std::string> files = get_files_in_folder(folder);
	std::vector<std::string> ext_files;

	std::vector<std::string> extensions_copy = extensions;

	if (!case_sensitive)  // convert to lowercase
	{
		for (int i = 0; i < extensions_copy.size(); ++i)
		{
			std::string& ext_ = extensions_copy[i];
			std::transform(ext_.begin(), ext_.end(), ext_.begin(), ::tolower);
		}
	}

	for (int i = 0; i < files.size(); ++i)
	{
		std::string file_name = files[i];
		std::string ext_ = "." + futils::get_file_extension(file_name);
		// std::transform(ext_.begin(), ext_.end(), ext_.begin(), ::tolower);

		if (ext_.length() > 0)
		{
			if (!case_sensitive)  // convert to lowercase
			{
				std::transform(ext_.begin(), ext_.end(), ext_.begin(), ::tolower);
			}
			if (std::find(extensions_copy.begin(), extensions_copy.end(), ext_) != extensions_copy.end())
			{
				ext_files.push_back(file_name);
			}	
		
		}
	}

	return ext_files;
}

}  /*namespace futils*/

#endif
