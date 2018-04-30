#include "stdafx.h"
#include "opencv2/imgcodecs.hpp" // imread ...
#include "opencv2/imgproc.hpp" // thresholding
#include <iostream>
#include <string>
using namespace cv;
using namespace std;

#include <experimental/filesystem> 
namespace stdfs = std::experimental::filesystem;

/* 
	 Simple tangent method is used to find the square encapsulating the wheel.
	 Tried to do it using Hough Circle Transform, but it's not precise enough,
	 it can happen that the algo takes the outer rim on left side and the inner rime on the right side
	 thus shifting the center point by more than 3 pixels which would give me 0 points.
 */
void calculateBoundaryPoints(Mat gray, int& upper_row, int& bottom_row, int& left_col, int& right_col) {
	// method assumes no random white pixels outside of the wheel itself which was correct assumption for the given datasets
	for (int row = 0; row < gray.rows; row++) {
		for (int col = 0; col < gray.cols; col++) {
			if (gray.at<uchar>(row, col) > 240) { // found a white pixel
				upper_row = row;
			}
			if (upper_row >= 0) break;
		}
		if (upper_row >= 0) break;
	}

	for (int row = gray.rows - 1; row >= 0; row--) {
		for (int col = 0; col < gray.cols; col++) {
			if (gray.at<uchar>(row, col) > 240) { // found a white pixel
				bottom_row = row;
			}
			if (bottom_row >= 0) break;
		}
		if (bottom_row >= 0) break;
	}

	for (int col = 0; col < gray.cols; col++) {
		for (int row = 0; row < gray.rows; row++) {
			if (gray.at<uchar>(row, col) > 240) { // found a white pixel
				left_col = col;
			}
			if (left_col >= 0) break;
		}
		if (left_col >= 0) break;
	}

	for (int col = gray.cols - 1; col >= 0; col--) {
		for (int row = 0; row < gray.rows; row++) {
			if (gray.at<uchar>(row, col) > 240) { // found a white pixel
				right_col = col;
			}
			if (right_col >= 0) break;
		}
		if (right_col >= 0) break;
	}
}

 /* 
	 Rotates the source matrix "angle" degrees ccw around center point
	 and returns the new rotated matrix. 
 */
Mat rotate(Mat source, double angle) {
	Point2f src_center(source.cols / 2.0F, source.rows / 2.0F);
	Mat rot_mat = getRotationMatrix2D(src_center, angle, 1.0);
	Mat dst;
	warpAffine(source, dst, rot_mat, source.size()); // apply affine transformation
	return dst;
}

 /* Used by: get360IntersectionMatrix() function
 
	 Finds the left most point and the right most point that belong to the inner part of the wheel.
	 Tt also memorizes the angle by which we had to rotate the wheel so that we have clear (black) area
	 with no spokes (either broken or whole).

	 We do a traversal from the center point of the wheel located on the hub in the leftward direction until we get to the rim
	 in case we have a spoke on that direction we rotate the wheel a couple of times by small increments
	 (+3 is choosen because of the constraints given) until we get a clear (black) area. 
	 
*/
void relax(Mat wheel, int row, int& col_l, int& col_r, double& correction_angle) {
	int c1 = col_l, c2 = col_r;
	int c1opt = 0, c2opt = 0;
	int lower = 0, upper = 255;

	double angle = 0;
	int NUMBER_OF_CALCULATIONS = 5;
	// try rotating the wheel a couple of times so that we are sure we are measuring
	// the actual distance between the hub and the rim without any spokes/broken spokes
	for (int i = 0; i < NUMBER_OF_CALCULATIONS; i++) {

		Mat rotated = rotate(wheel, angle);
		// initial conditions:
		// 1. in the begining of the traversal we still haven't found the leftmost point
		// 2. we start from the rim => last value we saw is 255
		bool leftfound = false;
		int last = upper;

		for (int col = col_l; col <= col_r; col++) {
			// we found a 255->0 edge == leftmost point
			if (rotated.at<uchar>(row, col) == 0 && !leftfound) {
				leftfound = true;
				c1 = col;
				last = lower; // the last value we saw is 0
			}
			// we found a 0->255 edge == rightmost point
			else if (rotated.at<uchar>(row, col) == 255 && last == lower) {
				c2 = col;
				break;
			}
		}

		// c1 and c2 are the same as col_l and col_r only if we did a traversal 
		// right across the unbroken spoke
		if (c1 != col_l && c2 != col_r) {
			// in the case we did a traversal over a broken spoke we will get smaller distance
			// then when we do it over a clear area that's why we are looking for the max distance
			if ((c2opt - c1opt) < (c2 - c1)) {
				c2opt = c2;
				c1opt = c1;
				correction_angle = angle;
			}
		}

		angle += 3; // increment the angle
	}

	col_l = c1opt;
	col_r = c2opt;
}

  /* Used by: get360IntersectionMatrix() function

	  Every row of the intersection matrix will store a row between the hub and the rim
	  of the rotated wheel for every single degree between 0 and 359. 

  */
void store(int angle, Mat intersection, Mat rotated, int row, int col_l, int col_r) {
	for (int col = col_l; col <= col_r; col++) {
		intersection.at<uchar>(angle, col - col_l) = rotated.at<uchar>(row, col);
	}
}

 /* 
	 Intersection matrix is a matrix we get while rotating the wheel and 
	 focusing only on the area between the hub and the rim, we start from 
	 the wheel shape and end up with a rectangular matrix that contains only spokes. 
 */
Mat get360IntersectionMatrix(Mat wheel) {
	int row = wheel.rows / 2;
	// initial values, they will get stricter after applying relax function
	int col_l = 0;
	int col_r = wheel.cols / 2;

	// find leftmost (col_l) and rightmost (col_r) points
	// that belong to the inner part of the wheel (between the hub and the rim)
	double correction_angle = 0.0;
	relax(wheel, row, col_l, col_r, correction_angle); 

	// allocate space for intersection matrix
	// for every degree we get one row in the intersection matrix
	// and the number of columns is the number of columns between the hub and the rim
	Mat intersection = Mat::zeros(360, col_r - col_l + 1, CV_8UC1); 

	// rotate the wheel so that when traversing from the center point of the hub
	// leftwards towards the rim we have a clear area (with no spokes) as calculated
	// by the relax function
	Mat corrected = rotate(wheel, correction_angle);

	// build the intersection matrix
	for (double angle = 0; angle < 360; angle += 1) {
		Mat rotated = rotate(corrected, angle);
		// store every row between the hub and the rim by rotating by 360 degrees
		store((int)angle, intersection, rotated, row, col_l, col_r);
	}

	return intersection;
}

/* Used by: brokenSpoke() function

	 Calculates the broken spoke length going in rightwards or leftwards direction 
	 depending on the goRight flag.

*/
int calculateSpokeLength(Mat intersection, int upper, int lower, bool goRight) {
	int length = 0;
	if (goRight) {
		for (int col = 0; col < intersection.cols; col++) {
			// ROI is 1 pixel wide rectangle
			Mat ROI(intersection, Rect(col, upper, 1, lower - upper + 1));
			// as soon as we only have black pixels the spoke has ended
			if (mean(ROI).val[0] == 0.0) {
				length = col;
				break;
			}
		}
	}
	else {
		for (int col = intersection.cols - 1; col >= 0; col--) {
			// ROI is 1 pixel wide rectangle
			Mat ROI(intersection, Rect(col, upper, 1, lower - upper + 1));
			// as soon as we only have black pixels the spoke has ended
			if (mean(ROI).val[0] == 0.0) {
				length = intersection.cols - col;
				break;
			}
		}
	}
	return length;
}

/* Used by: calculateSpokes() function

	Calculates the spokelength and return whether the spoke is broken or not.

*/
bool brokenSpoke(Mat intersection, int upper, int lower, int& spokelength, bool checkFromLeft) {
	// if the function was called during the left-traversal of calculateSpokes() method
	if (checkFromLeft) {
		// go rightwards and check if the spoke is broken
		for (int col = 0; col < intersection.cols - 10; col++) {
			// because of sharp edges added 1 pixel to thin out the ROI area
			Mat ROI(intersection, Rect(col, upper + 1, 10, lower - upper + 1 - 1)); 
			Scalar result = mean(ROI);
			// if during traversal we find area whose mean is closer to 0 => we've got a broken spoke
			if (result.val[0] < 15) {
				bool goRight = true;
				spokelength = calculateSpokeLength(intersection, upper, lower, goRight);
				return true;
			}
		}
	}
	// else the function was called during the right-traversal
	else {
		// go leftwards and check if the spoke is broken
		for (int col = intersection.cols - 10; col >= 0; col--) {
			// because of sharp edges added 1 pixel to thin out the ROI area
			Mat ROI(intersection, Rect(col, upper + 1, 10, lower - upper + 1 - 1)); 
			Scalar result = mean(ROI);
			// if during traversal we find area whose mean is closer to 0 => we've got a broken spoke
			if (result.val[0] < 15) {
				bool goRight = false;
				spokelength = calculateSpokeLength(intersection, upper, lower, goRight);
				return true; 
			}
		}
	}
	// otherwise the spoke is not broken return false
	spokelength = intersection.cols;
	return false;
}

 /* 
	 We do a traversal from up to bottom "margin" pixels away from the left border and one more traversal 
	 also from up to bottom "margin" pixels from the right in order to calculate the spokes statistics.
	 (remember rows of the intersection matrix represent different angles by which the wheel was rotated) 
 */
void calculateSpokes(Mat intersection, int& number_of_spokes, int& number_of_broken, vector<pair<pair<int, int>, int>>& leftspokes, vector<pair<pair<int, int>, int>>& rightspokes) {
	number_of_spokes = 0;
	number_of_broken = 0;
	int margin = 6; // experimentally found this value

	int last = 0;
	int upper;
	int lower;
	// first traversal on the left side of the intersection matrix
	for (int row = 0; row < intersection.rows; row++) {
		if (intersection.at<uchar>(row, margin) == 255) {
			// if we came across a white 255 pixel check if the last value was 0
			// this means 0->255 edge
			if (last == 0) {
				upper = row;
				last = 255;
			}
		}
		else {
			if (last == 255) {
				lower = row - 1;
				last = 0;
				number_of_spokes++; // we just crossed a white area => spoke found
				int spokelength;
				// check if the spoke is broken or not
				if (brokenSpoke(intersection, upper, lower, spokelength, true)) {
					number_of_broken++;
				}
				// we remember the positions and the lengths of the spokes on the left side 
				// of the intersection matrix as we will need it later for clearing the noise
				leftspokes.push_back(make_pair(make_pair(upper, lower), spokelength));
			}
		}
	}
	
	last = 0;
	// second traversal on the right side of the intersection matrix
	for (int row = 0; row < intersection.rows; row++) {
		if (intersection.at<uchar>(row, intersection.cols - margin - 1) == 255) {
			if (last == 0) {
				upper = row;
				last = 255;
			}
		}
		else {
			if (last == 255) {
				lower = row - 1;
				last = 0;
				int spokelength;
				// we only care if it's broken because otherwise we already saw it on the left side
				if (brokenSpoke(intersection, upper, lower, spokelength, false)) {
					// we remember the positions and the lengths of the spokes on the right side 
					// of the intersection matrix as we will need it later for clearing the noise
					rightspokes.push_back(make_pair(make_pair(upper, lower), spokelength));
					
					// check if we also have broken spoke on the left side, if not
					// this is a brand new spoke so we increment both number of spokes
					// as well as number of broken spokes
					Mat ROI(intersection, Rect(0, upper, 10, lower - upper + 1));
					if (!(mean(ROI).val[0] > 127)) {
						number_of_broken++;
						number_of_spokes++;
					}
				}
			}
		}
	}

}

/*
	Removes the noise around all of the spokes. Otherwise we won't get requested precision for arc length etc.
*/
void removeNoise(Mat intersection, vector<pair<pair<int, int>, int>> leftspokes, vector<pair<pair<int, int>, int>> rightspokes) {
	for (auto it = leftspokes.begin(); it != leftspokes.end(); ++it) {
		// extract spoke position and length
		int upper = (*it).first.first;
		int lower = (*it).first.second;
		int length = (*it).second;

		for (int row = upper - 5; row <= lower + 5; row++) {
			// check if we are in the valid range
			if (row >= 0 && row < intersection.rows) {
				// if the spoke row has very few white pixels we clear them thus removing the noise
				Mat ROI(intersection, Rect(0, row, length, 1));
				if (mean(ROI).val[0] < 200) { // experimentally found this value
					// clear the line (fill it with black pixels)
					for (int col = 0; col < length; col++) {
						intersection.at<uchar>(row, col) = 0;
					}
				}
			}
		}
	}
	for (auto it = rightspokes.begin(); it != rightspokes.end(); ++it) {
		// extract spoke position and length
		int upper = (*it).first.first;
		int lower = (*it).first.second;
		int length = (*it).second;
		for (int row = upper - 5; row <= lower + 5; row++) {
			// check if we are in the valid range
			if (row >= 0 && row < intersection.rows) {
				// if the spoke row has very few white pixels we clear them thus removing the noise
				Mat ROI(intersection, Rect(intersection.cols - length, row, length, 1));
				if (mean(ROI).val[0] < 200) { 
					// clear the line (fill it with black pixels)
					for (int col = intersection.cols - length; col < intersection.cols; col++) {
						intersection.at<uchar>(row, col) = 0;
					}
				}
			}
		}
	}
}

/*
	Finds the longest arc by doing vertical scans of the intersection matrix.
*/
void longestArc(Mat intersection, int& arc) {
	for (int col = 0; col < intersection.cols; col++) {
		int current_max = 0;
		int first_arc = 0; // length of the first arc during current vertical scan
		bool first = true;
		for (int row = 0; row < intersection.rows; row++) {
			if (intersection.at<uchar>(row, col) == 0) {
				current_max++;
			}
			else {
				if (first) {
					// memorize length of the first arc
					first_arc = current_max;
					first = false;
				}
				arc = max(arc, current_max);
				current_max = 0; // reset
			}
		}
		// because the matrix is circular we combine last arc's length with first arc's length
		current_max += first_arc; 
		arc = max(arc, current_max);
	}
}

/*
	Finds the angle by which the wheel_current must be rotated to become wheel_last.
*/
double findAngleBetween(Mat wheel_last, Mat wheel_current) {
	int rows_min = wheel_last.rows > wheel_current.rows ? wheel_current.rows : wheel_last.rows;
	int cols_min = wheel_last.cols > wheel_current.cols ? wheel_current.cols : wheel_last.cols;
	// get the both wheels to the same size
	Mat wheel_last_n(wheel_last, Rect(0, 0, cols_min, rows_min));
	Mat wheel_current_n(wheel_current, Rect(0, 0, cols_min, rows_min));
	long max = -1;

	double angle_opt = 0;
	// +-50 degrees is the greatest turn in the public data - sat
	// = > hence I assume I can use those boundaries instead of +-180
	// this was the difference between the programm getting the TLE (time limit exceeded) and not getting it
	for (double angle = -50; angle <= 50; angle += 1) {
		Mat rotated = rotate(wheel_current_n, angle);
		Mat result;
		// rotated matrix acts as a mask
		wheel_last_n.copyTo(result, rotated); 
		long cs = sum(result).val[0];
		// the greater the cs value the bigger the correlation
		// for max cs we know we have total correlation and thus we've found the angle
		if (cs > max) {
			max = cs;
			angle_opt = angle;
		}
	}
	return angle_opt;
}

/*
	Calculates the greatest time span during which the arrow can pass through the wheel.
*/
void calculateBiggestTimeSpan(vector<int>& angles, int arc_max, int& k1, int& k2) {
	int k1_opt = 0;
	int k2_opt = 0;
	for (int i = 0; i < angles.size(); i++) {
		int cur_span_left = 0;
		int cur_span_right = 0;
		int sum = 0;
		int j = i;
		for (; j < angles.size(); j++) {
			sum += angles.at(j);
			if (sum > cur_span_right) cur_span_right = sum;
			if (sum < cur_span_left) cur_span_left = sum;
			// if we spanned an angle bigger than arc_max we have to save the last frame
			// for which the value was still smaller than arc_max
			if (abs(cur_span_left) + abs(cur_span_right) >= arc_max) {
				if ((j - i) >(k2_opt - k1_opt + 1)) {
					k1_opt = i;
					k2_opt = j - 1;
				}
				break;
			}
		}
		// in case we didn't overshoot the arc_max during the second for loop
		// check here if we have an optimum
		if (abs(cur_span_left) + abs(cur_span_right) < arc_max) {
			if ((j - i) > (k2_opt - k1_opt + 1)) {
				k1_opt = i;
				k2_opt = j - 1;
			}
			// we can stop the function now because we now we can't get better result
			break; 
		}
	}
	k1 = k1_opt;
	k2 = k2_opt;
}

int main(int argc, char** argv)
{
	/* used for local testing, to_string is used in combination
	 with the for loop so that I could iterate through all of the data sets in an automated fashion
	std::string datasetFolder = "path_to_data_set_without_index" + to_string(i);*/
	
	/*STEP 0: (yes I am a software engineer I count from 0)
	Reads in the data set directory path through standard input 
	and loads the first image */
	std::string datasetFolder;
	cin >> datasetFolder;
	stdfs::directory_iterator iter_last{ datasetFolder }; // used for iterating through the images of a dataset
	const stdfs::directory_iterator end{};

	std::string imagename = iter_last->path().string(); // first image in a dataset to be processed
	const char* filename = argc >= 2 ? argv[1] : imagename.c_str(); // we can also start the program from command line
	Mat src = imread(filename, IMREAD_COLOR); // Loads the image
	if (src.empty()) { // Checks if image is loaded fine, basic error checking
		cout << " Error opening image " << endl;
		return -1;
	}

	// STEP 1: convert to grayscale and apply binary thresholding
	Mat gray;
	cvtColor(src, gray, COLOR_BGR2GRAY); // convert to grayscale (1-channel image)
	threshold(gray, gray, 127, 255, THRESH_BINARY); // experimentally found the threshold

	// STEP 2: calculate center point of the wheel and it's width, height and radius
	int upper_row = -1, bottom_row = -1, left_col = -1, right_col = -1;
	calculateBoundaryPoints(gray, upper_row, bottom_row, left_col, right_col);
	int x = ((left_col + right_col) / 2) + 1;
	int y = ((upper_row + bottom_row) / 2) + 1;
	int width = right_col - left_col + 1;
	int height = bottom_row - upper_row + 1;
	int radius = width / 2;

	// STEP 3: crop only the wheel portion of the image
	Rect ROI(left_col, upper_row, width, height);
	Mat wheel_last(gray, ROI);

	// STEP 4: build intersection matrix
	// intersection matrix is a matrix we get while rotating the wheel and 
	// focusing only on the area between the hub and the rim, we start from 
	// the wheel shape and end up with rectangular matrix that contains only spokes
	Mat intersection = get360IntersectionMatrix(wheel_last);

	// STEP 5: calculate the spokes statistics
	int number_of_spokes = 0;
	int number_of_broken = 0;
	vector<pair<pair<int, int>, int>> leftspokes;
	vector<pair<pair<int, int>, int>> rightspokes;
	calculateSpokes(intersection, number_of_spokes, number_of_broken, leftspokes, rightspokes);

	// STEP 6: clean the noise from intersection matrix and calculate the longest arc
	removeNoise(intersection, leftspokes, rightspokes);
	int arc_max = 0;
	longestArc(intersection, arc_max);

	// STEP 7: iterate over all of the images in a data set and calculate
	// the angle difference between successive frames
	vector<int> angles;
	stdfs::directory_iterator iter_current = iter_last;
	for (iter_current++; iter_current != end; ++iter_current)
	{
		// load the image in
		std::string imagename = iter_current->path().string();
		const char* filename = imagename.c_str();
		Mat src = imread(filename, IMREAD_COLOR);
		if (src.empty()) {
			cout << " Error opening image" << endl;
			return -1;
		}
		// convert to grayscale and apply thresholding
		Mat gray;
		cvtColor(src, gray, COLOR_BGR2GRAY);
		threshold(gray, gray, 127, 255, THRESH_BINARY);
		// calculate center point of the wheel and it's width, height and radius
		int upper_row = -1, bottom_row = -1, left_col = -1, right_col = -1;
		calculateBoundaryPoints(gray, upper_row, bottom_row, left_col, right_col);
		int width = right_col - left_col + 1;
		int height = bottom_row - upper_row + 1;
		// crop the wheel
		Rect ROI(left_col, upper_row, width, height);
		Mat wheel_current(gray, ROI);
		// find the rotation angle between 2 successive wheels and store that info
		int angle = findAngleBetween(wheel_last, wheel_current);
		angles.push_back(angle);
		wheel_last = wheel_current;
	}

	// STEP 8: find the biggest time span during which the arrow can enter 
	// and pass through without touching the wheel (spokes/hub/rim)
	int k1 = 0; // index of the very first frame
	int k2 = 0; // index of the last frame
	calculateBiggestTimeSpan(angles, arc_max, k1, k2);

	// STEP 9: output data in requested format:
	cout << x << " " << y << " " << number_of_spokes << " " << number_of_broken << " " << arc_max << " " << k1 + 1 << " " << k2 + 1 << endl;

	return 0;
}