#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
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

	namedWindow("Display window", WINDOW_AUTOSIZE); // Create a window for display.
	setWindowProperty("Display window", WINDOW_FULLSCREEN, WINDOW_FULLSCREEN);
	imshow("Display window", image);		    // Show our image inside it.

	waitKey(0); // Wait for a keystroke in the window
	return 0;
}

int main(int argc, char **argv)
{
	string img_1 = "Slide 1-prima.jpg", img_2 = "Slide 2 -prima.jpg", img_3 = "Slide 3 -conteo.jpg";
	list<Point2f> fix_1, fix_2, fix_3;

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

	return 0;
}