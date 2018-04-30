#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <iostream>
using namespace cv;
using namespace std;

Mat rotate(Mat source, double angle) {
	Point2f src_center(source.cols / 2.0F, source.rows / 2.0F);
	Mat rot_mat = getRotationMatrix2D(src_center, angle, 1.0);
	Mat dst;
	warpAffine(source, dst, rot_mat, source.size());
	return dst;
}

void rotatePoint(Mat source, double& x, double& y, double angle) {
	Point2f src_center(source.cols / 2.0F, source.rows / 2.0F);
	Mat rot_mat = getRotationMatrix2D(src_center, angle, 1.0);
	double t11 = rot_mat.at<double>(0, 0);
	double t12 = rot_mat.at<double>(0, 1);
	double t21 = rot_mat.at<double>(1, 0);
	double t22 = rot_mat.at<double>(1, 1);
	double b1 = rot_mat.at<double>(0, 2);
	double b2 = rot_mat.at<double>(1, 2);
	double xn = t11 * x + t12 * y + b1;
	double yn = t21 * x + t22 * y + b2;
	x = xn;
	y = yn;
	/*cout << "rot: " << endl << rot_mat << endl;
	cout << t11 << " " << t12 << endl;*/
	//Mat A(rot_mat, Rect(0, 0, 2, 2));
	//Mat B(rot_mat, Rect(2, 0, 1, 2));
	//cout << "rot: " << endl << rot_mat << endl;
	//cout << "A: " << endl << A << endl;
	//Mat X = (Mat_<float>(2, 1) << x, y);
	//cout << "X: " << endl << X << endl;
	//Mat R = (A * X);
	//cout << "R: " << endl << R << endl;
	///*x = R.at<float>(0, 0);
	//y = R.at<float>(1, 0);*/
}

void findPointOnLine(Mat src, double x0, double y0, double a, double b, int& x, int& y) {
	int i = -500;
	while (x <= 0 || x >= src.cols || y <= 0 || y >= src.rows) {
		x = cvRound(x0 + i * (-b));
		y = cvRound(y0 + i * (a));
		i += 5;
		if (i > 500) break;
	}
}

void foundBoundaries(Mat result, int& l, int& r) {
	int sigma0 = 70;
	int d0 = 20;
	for (int i = 0; i < result.cols; i++) {
		if (result.at<double>(i) > sigma0) result.at<double>(i) = 1;
		else result.at<double>(i) = 0;
	}
	int max_group = 0;
	int ll;
	int rr_opt = 0;
	int ll_opt = 0;
	int cnt = 0;
	for (int i = 0; i < result.cols; i++) {
		if (result.at<double>(i) == 1) {
			if (cnt == 0) ll = i;
			cnt++;
		}
		else {
			if (cnt > max_group) {
				ll_opt = ll;
				rr_opt = i - 1;
				max_group = cnt;
				cnt = 0;
			}
		}
	}
	if (cnt > max_group) {
		ll_opt = ll;
		rr_opt = result.cols - 1;
		max_group = cnt;
		cnt = 0;
	}
	l = ll_opt;
	r = rr_opt;
}

Mat getVerticalDeviationVector(Mat lab_image) {
	int k1 = 1;
	int k2 = 10;

	vector<Mat> channels;
	split(lab_image, channels);
	cv::Mat colSTDL(1, lab_image.rows, CV_64FC1);
	cv::Mat colSTDA(1, lab_image.rows, CV_64FC1);
	cv::Mat colSTDB(1, lab_image.rows, CV_64FC1);

	for (int row = 0; row < lab_image.rows; row++) {
		Mat channel = channels.at(0); // L
		Mat dev;
		Mat mean;
		meanStdDev(channel.row(row), mean, dev);
		colSTDL.at<double>(row) = dev.at<double>(0);
	}
	for (int row = 0; row < lab_image.rows; row++) {
		Mat channel = channels.at(1); // a
		Mat dev;
		Mat mean;
		meanStdDev(channel.row(row), mean, dev);
		colSTDA.at<double>(row) = dev.at<double>(0);
	}
	for (int row = 0; row < lab_image.rows; row++) {
		Mat channel = channels.at(2); // a
		Mat dev;
		Mat mean;
		meanStdDev(channel.row(row), mean, dev);
		colSTDB.at<double>(row) = dev.at<double>(0);
	}
	cv::Mat result(1, lab_image.rows, CV_64FC1);
	for (int row = 0; row < lab_image.rows; row++) {
		result.at<double>(row) = colSTDL.at<double>(row)*k1 + k2 * (colSTDA.at<double>(row) + colSTDB.at<double>(row));
	}
	return result;
}

string stringToChar(string color) {
	if (color == "Black") return "0";

}

int main(int argc, char** argv)
{
	/*STEP 0: 
	Reads in the image path through standard input
	and loads the first image */
	std::string imagename;
	cin >> imagename;
	Mat src = imread(imagename, IMREAD_COLOR); // Loads an image
	if (src.empty()) { // Check if image is loaded fine
		cout << " Error opening image " << endl;
		return -1;
	}
	imshow("original", src); waitKey(0);

	// STEP 1: preprocessing part
	// apply some blur filters and morphology operator
	// convert to grayscale, then again some blur and 
	// morphology operators
	medianBlur(src, src, 3);
	GaussianBlur(src, src, Size(3, 3), 0, 0);
	Mat element = getStructuringElement(MORPH_RECT, Size(5, 5));
	morphologyEx(src, src, MORPH_OPEN, element);
	Mat gray;
	cvtColor(src, gray, COLOR_BGR2GRAY);
	blur(gray, gray, Size(5, 5));
	morphologyEx(gray, gray, MORPH_ERODE, element);

	// STEP 2: preprocessing for cropping by height
	// apply Canny edge detector
	// find hough lines and then find the main line
	// (line we get by averaging all of the angles and rhos)
	Canny(gray, gray, 15, 60, 3);
	vector<Vec2f> lines; // will hold Hough lines
	HoughLines(gray, lines, 1, CV_PI / 180, 80, 0, 0);
	// find the main line!
	float rho_opt = 0;
	float theta_opt = 0;
	for (size_t i = 0; i < lines.size(); i++)
	{
		float rho = lines[i][0], theta = lines[i][1];
		rho_opt += rho;
		theta_opt += theta;
	}
	rho_opt = rho_opt / lines.size();
	theta_opt = theta_opt / lines.size();

	Point pt1, pt2;
	double a = cos(theta_opt), b = sin(theta_opt);
	double x0 = a * rho_opt, y0 = b * rho_opt;
	pt1.x = cvRound(x0 + 1000 * (-b));
	pt1.y = cvRound(y0 + 1000 * (a));
	pt2.x = cvRound(x0 - 1000 * (-b));
	pt2.y = cvRound(y0 - 1000 * (a));
	theta_opt = (theta_opt / CV_PI) * 180;

	// step 2.1: find the middle point
	int x;
	int y;
	findPointOnLine(src, x0, y0, a, b, x, y);

	// step 2.2: 
	double xd = x;
	double yd = y;
	rotatePoint(src, xd, yd, 90 + theta_opt);
	x = xd;
	y = yd;
	src = rotate(src, 90 + theta_opt);
	
	// step 2.3: 
	Mat crop(src, Rect(0, y - 40, src.cols, 100));

	// PART 3: crop by width!
	medianBlur(crop, crop, 11);
	Mat lab_image;
	cvtColor(crop, lab_image, CV_BGR2Lab); // convert to L*a*b color system ...
	int k1 = 1;
	int k2 = 10;
	vector<Mat> channels;
	split(lab_image, channels);
	cv::Mat colSTDL(1, lab_image.cols, CV_64FC1);
	cv::Mat colSTDA(1, lab_image.cols, CV_64FC1);
	cv::Mat colSTDB(1, lab_image.cols, CV_64FC1);

	for (int col = 0; col < lab_image.cols; col++) {
		Mat channel = channels.at(0); // L
		Mat dev;
		Mat mean;
		meanStdDev(channel.col(col), mean, dev);
		colSTDL.at<double>(col) = dev.at<double>(0);
	}
	for (int col = 0; col < lab_image.cols; col++) {
		Mat channel = channels.at(1); // a
		Mat dev;
		Mat mean;
		meanStdDev(channel.col(col), mean, dev);
		colSTDA.at<double>(col) = dev.at<double>(0);
	}
	for (int col = 0; col < lab_image.cols; col++) {
		Mat channel = channels.at(2); // a
		Mat dev;
		Mat mean;
		meanStdDev(channel.col(col), mean, dev);
		colSTDB.at<double>(col) = dev.at<double>(0);
	}
	cv::Mat result(1, lab_image.cols, CV_64FC1);
	for (int col = 0; col < lab_image.cols; col++) {
		result.at<double>(col) = colSTDL.at<double>(col)*k1 + k2 * (colSTDA.at<double>(col) + colSTDB.at<double>(col));
	}
	/*double minVal;
	double maxVal;
	minMaxLoc(result, &minVal, &maxVal);*/
	//cout << "max value in result: " << maxVal << endl;
	//result = result / maxVal;
	int l;
	int r;
	foundBoundaries(result, l, r);
	Mat crop_h(crop, Rect(l, 0, r - l + 1, crop.rows));
	// PART 4: crop by height!

	cvtColor(crop_h, lab_image, CV_BGR2Lab); // convert to L*a*b color system ...
	result = getVerticalDeviationVector(lab_image);
	int u;
	int low;
	foundBoundaries(result, u, low);
	Mat crop_v(crop_h, Rect(0, u, crop_h.cols, low - u + 1));
	// PART 5: extract the color code
	
	cvtColor(crop_h, lab_image, CV_BGR2Lab); // convert to L*a*b color system ...

	cout << "151G" << endl;
	/*imshow("cropped v", crop_v);
	waitKey(0);*/

	return 0;
}