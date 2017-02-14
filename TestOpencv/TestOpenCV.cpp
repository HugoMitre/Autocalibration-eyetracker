#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <iterator>
#include <vector>
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
	waitKey(1000);			// Wait for a keystroke in the window
	return 0;
}

int main(int argc, char **argv)
{
	string img_1 = "Slide 1-prima.jpg", img_2 = "Slide 2 -prima.jpg", img_3 = "Slide 3 -conteo.jpg";
	list<Point2f> fix_1, fix_2, fix_3;

	/* Calibration */
	vector<Point2f> sx = { Point2f(100, 100), Point2f(100, 100), Point2f(100, 100),
		Point2f(100, 300), Point2f(100, 300), Point2f(100, 300),
		Point2f(100, 600), Point2f(100, 600), Point2f(100, 600),
		Point2f(300, 100), Point2f(300, 100), Point2f(300, 100),
		Point2f(300, 300), Point2f(300, 300), Point2f(300, 300),
		Point2f(300, 600), Point2f(300, 600), Point2f(300, 600),
		Point2f(600, 100), Point2f(600, 100), Point2f(600, 100),
		Point2f(600, 300), Point2f(600, 300), Point2f(600, 300),
		Point2f(600, 600), Point2f(600, 600), Point2f(600, 600) };
	Mat a /*(sx.size(), 6, CV_32FC2)*/;

	/* Punto 1 */
	MyGaze *m = new MyGaze();
	display_point(100, 100);
	vector<Point2f> tmp = m->get_readings();
	vector<float> arrtmp = { 1, tmp[2].x, tmp[2].y, tmp[2].x * tmp[2].x, tmp[2].y * tmp[2].y, tmp[2].x * tmp[2].y };
	Mat row(arrtmp);
	a.push_back(row.t());
	m->~MyGaze();
	/* Punto 2 */
	m = new MyGaze();
	display_point(500, 100);
	tmp = m->get_readings();
	arrtmp = { 1, tmp[2].x, tmp[2].y, tmp[2].x * tmp[2].x, tmp[2].y * tmp[2].y, tmp[2].x * tmp[2].y };
	row = Mat(arrtmp);
	a.push_back(row.t());
	m->~MyGaze();
	/* Punto 3 */
	m = new MyGaze();
	display_point(900, 100);
	tmp = m->get_readings();
	arrtmp = { 1, tmp[2].x, tmp[2].y, tmp[2].x * tmp[2].x, tmp[2].y * tmp[2].y, tmp[2].x * tmp[2].y };
	row = Mat(arrtmp);
	a.push_back(row.t());
	m->~MyGaze();
	/* Punto 4 */
	m = new MyGaze();
	display_point(100, 500);
	tmp = m->get_readings();
	arrtmp = { 1, tmp[2].x, tmp[2].y, tmp[2].x * tmp[2].x, tmp[2].y * tmp[2].y, tmp[2].x * tmp[2].y };
	row = Mat(arrtmp);
	a.push_back(row.t());
	m->~MyGaze();
	/* Punto 5 */
	m = new MyGaze();
	display_point(500, 500);
	tmp = m->get_readings();
	arrtmp = { 1, tmp[2].x, tmp[2].y, tmp[2].x * tmp[2].x, tmp[2].y * tmp[2].y, tmp[2].x * tmp[2].y };
	row = Mat(arrtmp);
	a.push_back(row.t());
	m->~MyGaze();
	/* Punto 6 */
	m = new MyGaze();
	display_point(900, 500);
	tmp = m->get_readings();
	arrtmp = { 1, tmp[2].x, tmp[2].y, tmp[2].x * tmp[2].x, tmp[2].y * tmp[2].y, tmp[2].x * tmp[2].y };
	row = Mat(arrtmp);
	a.push_back(row.t());
	m->~MyGaze();
	/* Punto 7 */
	m = new MyGaze();
	display_point(100, 900);
	tmp = m->get_readings();
	arrtmp = { 1, tmp[2].x, tmp[2].y, tmp[2].x * tmp[2].x, tmp[2].y * tmp[2].y, tmp[2].x * tmp[2].y };
	row = Mat(arrtmp);
	a.push_back(row.t());
	m->~MyGaze();
	/* Punto 8 */
	m = new MyGaze();
	display_point(500, 900);
	tmp = m->get_readings();
	arrtmp = { 1, tmp[2].x, tmp[2].y, tmp[2].x * tmp[2].x, tmp[2].y * tmp[2].y, tmp[2].x * tmp[2].y };
	row = Mat(arrtmp);
	a.push_back(row.t());
	m->~MyGaze();
	/* Punto 9 */
	m = new MyGaze();
	display_point(900, 900);
	tmp = m->get_readings();
	arrtmp = { 1, tmp[2].x, tmp[2].y, tmp[2].x * tmp[2].x, tmp[2].y * tmp[2].y, tmp[2].x * tmp[2].y };
	row = Mat(arrtmp);
	a.push_back(row.t());
	m->~MyGaze();

	cout << a << endl;
	/*Mat matsx = Mat(sx);
	Mat tehta = (a.t() * a).inv() * a;
	cout << tehta << endl;*/

	// Imagen 1
	m = new MyGaze();
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