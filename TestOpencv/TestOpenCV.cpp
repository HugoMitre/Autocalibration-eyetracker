#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <iterator>
#include <vector>
#include <fstream>
#include <Windows.h>
#include "MyGaze.h"

using namespace cv;
using namespace std;

int display_img(string path)
{
	Mat image;
	image = imread(path, IMREAD_COLOR); // Read the file

	if (image.empty()) // Check for invalid input
	{
		cout << "Could not open or find the image" << std::endl;
		return -1;
	}

	namedWindow("Display window", CV_WINDOW_NORMAL); // Create a window for display.
	setWindowProperty("Display window", CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);
	imshow("Display window", image); // Show our image inside it.

	waitKey(0); // Wait for a keystroke in the window
	return 0;
}

int display_point(float x, float y)
{
	Mat matPoint = Mat::zeros(1000, 1000, CV_8UC3);
	circle(matPoint,
		Point(x, y),
		5.0,
		Scalar(0, 0, 255),
		-1,
		8);
	namedWindow("Display window", CV_WINDOW_NORMAL); // Create a window for display.
	setWindowProperty("Display window", CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);
	// moveWindow("Display window", 0, 0);
	imshow("Display window", matPoint); // Show our image inside it.
	waitKey(1000); // Wait for a keystroke in the window
	return 0;
}

int calibration_points(Mat &a, float x, float y)
{
	MyGaze *m = new MyGaze();
	display_point(x, y);
	vector<Point2f> tmp = m->get_readings();
	vector<float> arrtmp = { 1, tmp[1].x, tmp[1].y, tmp[1].x * tmp[1].x, tmp[1].y * tmp[1].y, tmp[1].x * tmp[1].y };
	Mat row(arrtmp);
	a.push_back(row.t());
	arrtmp = { 1, tmp[4].x, tmp[4].y, tmp[4].x * tmp[4].x, tmp[4].y * tmp[4].y, tmp[4].x * tmp[4].y };
	row = Mat(arrtmp);
	a.push_back(row.t());
	arrtmp = { 1, tmp[8].x, tmp[8].y, tmp[8].x * tmp[8].x, tmp[8].y * tmp[8].y, tmp[8].x * tmp[8].y };
	row = Mat(arrtmp);
	a.push_back(row.t());
	m->~MyGaze();
	return 0;
}

int main(int argc, char **argv)
{
	ofstream outputFile("output.csv");
	string img_1 = "Slide 1-prima.jpg", img_2 = "Slide 2 -prima.jpg", img_3 = "Slide 3 -conteo.jpg";
	list<Point2f> fix_1, fix_2, fix_3;

	/* Calibration */
	vector<Point2f> sp = { Point2f(100, 100), Point2f(100, 100), Point2f(100, 100),
		Point2f(500, 100), Point2f(500, 100), Point2f(500, 100),
		Point2f(900, 100), Point2f(900, 100), Point2f(900, 100),
		Point2f(100, 500), Point2f(100, 500), Point2f(100, 500),
		Point2f(500, 500), Point2f(500, 500), Point2f(500, 500),
		Point2f(900, 500), Point2f(900, 500), Point2f(900, 500),
		Point2f(100, 900), Point2f(100, 900), Point2f(100, 900),
		Point2f(500, 900), Point2f(500, 900), Point2f(500, 900),
		Point2f(900, 900), Point2f(900, 900), Point2f(900, 900) };
	vector<float> sx, sy, ex, ey;
	transform(sp.begin(), sp.end(), back_inserter(sx), [](Point2f const &pnt) { return pnt.x; });
	transform(sp.begin(), sp.end(), back_inserter(sy), [](Point2f const &pnt) { return pnt.y; });
	Mat a;/*(sx.size(), 6, CV_32FC2)*/
	float y = 100;
	for (int i = 0; i < 3; i++)
	{
		float x = 100;
		for (int j = 0; j < 3; j++)
		{
			calibration_points(a, x, y);
			x = x + 400;
		}
		y = y + 400;
	}

	//cout << a << endl;
	Mat matsx = Mat(sx);
	Mat matsy = Mat(sy);
	Mat tehtax = (a.t() * a).inv() * a.t() * matsx;
	Mat tehtay = (a.t() * a).inv() * a.t() * matsy;
	//cout << tehtax << endl;

	/* Estimate coordinates */
	for (int i = 0; i < a.rows; i++)
	{
		Mat tmpx = a.row(i) * tehtax;
		ex.push_back(tmpx.at<float>(0, 0));
		Mat tmpy = a.row(i) * tehtay;
		ey.push_back(tmpy.at<float>(0, 0));
	}

	/* Output to file */
	if (outputFile.is_open())
	{
		outputFile << "Screen coordinates X, Screen coordinates Y, Estimated coordinates X, Estimated coordinates Y" << endl;
		for (int i = 0; i < sx.size(); i++)
		{
			outputFile << sx[i] << ", " << sy[i] << ", " << ex[i] << ", " << ey[i] << endl;
		}
	}
	else cout << "Unable to open output file";

	// Imagen 1
	MyGaze *m = new MyGaze();
	display_img(img_1);
	//fix_1 = m->get_fixations();
	m->~MyGaze();

	// Imagen 2
	m = new MyGaze();
	display_img(img_2);
	//fix_1 = m->get_fixations();
	m->~MyGaze();

	// Imagen 3
	m = new MyGaze();
	display_img(img_3);
	//fix_1 = m->get_fixations();
	m->~MyGaze();

	return 0;
}