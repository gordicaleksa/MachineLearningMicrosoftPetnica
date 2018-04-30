#include "opencv2/core.hpp"
using namespace cv;

#include <fstream>
#include <sstream>
#include <iostream>
#include <vector>
#include <string>

#include <experimental/filesystem> 
namespace stdfs = std::experimental::filesystem;

// returns paths to all the folders inside of the root folder specified by the argument path
std::vector<stdfs::path> get_foldernames(stdfs::path path)
{
	std::vector<stdfs::path> foldernames;

	const stdfs::directory_iterator end{};

	for (stdfs::directory_iterator iter{ path }; iter != end; ++iter)
	{
		// check if it is directory/folder 
		if (stdfs::is_directory(*iter)) {
			foldernames.push_back(iter->path());
		}
	}

	return foldernames;
}

/* 
	(a,b) (c,d) (e,f) are points that define a new coordinate system.
	(g,h) is a point in old coordinate system and it will get transformed. 
*/
void convert(float a, float b, float c, float d, float e, float f, float& g, float& h) {
	// normalize new ix vector
	float ix_p1 = c - a;
	float ix_p2 = d - b;
	float ix_mag = sqrt(ix_p1*ix_p1 + ix_p2 * ix_p2);
	ix_p1 = ix_p1 / ix_mag;
	ix_p2 = ix_p2 / ix_mag;
	// normalize new iy vector
	float iy_p1 = e - a;
	float iy_p2 = f - b;
	float iy_mag = sqrt(iy_p1*iy_p1 + iy_p2 * iy_p2);
	iy_p1 = iy_p1 / iy_mag;
	iy_p2 = iy_p2 / iy_mag;
	// linear transformation matrix
	Mat T = (Mat_<float>(2, 2) << ix_p1, iy_p1, ix_p2, iy_p2);
	Mat Tinv = T.inv(); // even if the determinant is 0 inv() won't break.
	Mat new_origin = (Mat_<float>(2, 1) << a, b);
	Mat point = (Mat_<float>(2, 1) << g, h);
	Mat transformed_point = Tinv * point;
	Mat offset = Tinv * new_origin;
	Mat res = transformed_point - offset;
	g = (float)res.at<float>(0, 0);
	h = res.at<float>(1, 0);
}

int main()
{
	// std::string root = "C:/Users/aleks/Desktop/VisualStudio_workspace/Petnica_ML_homework/B_Keypoints/publicDataSet/set/";
	// Reads in the root directory path through standard input 
	// root directory contains only directories which contain exactly 1 file
	std::string root;
	std::cin >> root;
	// iterate over folders in root directory
	for (const auto& folder : get_foldernames(root)) {
		// iter points to the single .txt file inside of a folder
		stdfs::directory_iterator iter{ folder }; 
		// get the folder and the filename as well as the filepath
		std::string foldername = folder.filename().string();
		std::string filename = iter->path().filename().string();
		std::string filepath = iter->path().string();
		std::ifstream f;
		f.open(filepath); 
		if (f.is_open()) {
			int i = 0;
			std::string line;
			// every line contains 8 floating point numbers
			while (std::getline(f, line)) {
				std::istringstream iss(line);
				float a, b, c, d, e, f, g, h;
				iss >> a >> b >> c >> d >> e >> f >> g >> h;
				// convert the point (g,h) to a new coordinate system defined by the points (a,b), (c,d) and (e,f)
				convert(a, b, c, d, e, f, g, h);
				// print the result in requested format:
				std::cout << foldername << "\\" << filename << " " << i << " " << g << " " << h << std::endl;
				i++;
			}
		}
		else {
			std::cout << "Opening file failed!" << std::endl;
			return -1;
		}
		f.close();
	}
	return 0;
}