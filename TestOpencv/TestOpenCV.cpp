#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <iterator>
#include <vector>
#include <Eigen/Dense>
#include <fstream>
#include <Windows.h>
#include "MyGaze.h"

using namespace cv;
using namespace std;

vector<Point2f> coords;
int width = GetSystemMetrics(SM_CXSCREEN), height = GetSystemMetrics(SM_CYSCREEN);

void on_mouse(int event, int x, int y, int, void* imgptr)
{
	Mat &img = (*(Mat*)imgptr);
	if (event == EVENT_LBUTTONDOWN)
	{
		cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
		circle(img,
			Point(x, y),
			5.0,
			Scalar(0, 0, 255),
			-1,
			CV_AA);
		imshow("Display window", img);
		coords.push_back(Point2f(x, y));
	}
	else if (event == EVENT_LBUTTONUP)
	{
		cout << "Right button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
		circle(img,
			Point(x, y),
			5.0,
			Scalar(0, 0, 255),
			-1,
			CV_AA);
		coords.push_back(Point2f(x, y));
		for (auto n : coords)
		{
			cout << n.x << " " << n.y << endl;
		}
		rectangle(img, coords[0], coords[1], Scalar(0, 255, 0));
		coords.clear();
		imshow("Display window", img);
	}
	else if (event == EVENT_MBUTTONDOWN)
	{
		cout << "Middle button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
	}
	else if (event == EVENT_MOUSEMOVE)
	{
		cout << "Mouse move over the window - position (" << x << ", " << y << ")" << endl;
	}
}

int display_line(Mat &line_ds, Mat &img)
{
	for (int i = 0; i < line_ds.rows; i++)
	{
		const double* Mi = line_ds.ptr<double>(i);
		//circle(img, Point(Mi[0], Mi[1]), 2.0, Scalar(0, 0, 0), -1, CV_AA);
		if (i + 1 < line_ds.rows)
		{
			double* p2 = line_ds.ptr<double>(i + 1);
			line(img, Point(Mi[0], Mi[1]), Point(p2[0], p2[1]), Scalar(0, 0, 0), 1, CV_AA);
		}
	}
	return 0;
}

Mat adjust_line(Mat &centroids)
{
	Mat y = centroids.col(1), x = centroids.col(0), one = Mat::ones(x.size(), x.type());
	hconcat(x, one, one);
	Mat theta = (one.t() * one).inv() * one.t() * y;
	Mat yp = (theta.at<double>(0) * x) + theta.at<double>(1);
	return yp;
}

int get_centroids(Mat &img)
{
	Mat grey, binimg, labels, stats, centroids, best_labels, attemps, centers, centroids32f;
	cvtColor(img, grey, COLOR_BGR2GRAY);
	threshold(grey, binimg, 80, 255, THRESH_BINARY_INV | THRESH_OTSU);
	connectedComponentsWithStats(binimg, labels, stats, centroids);
	//cout << centroids << endl;
	/*for (int i = 0; i < centroids.rows; i++)
	{
		const double* Mi = centroids.ptr<double>(i);
		circle(img,
			Point(Mi[0], Mi[1]),
			2.0,
			Scalar(0, 0, 255),
			-1,
			8);
	}*/
	//imshow("Display window", img);
	centroids.convertTo(centroids32f, CV_32F);
	kmeans(centroids32f.col(1), 3, best_labels, TermCriteria(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 10000, 0.0001), 5, KMEANS_PP_CENTERS, centers);
	Mat red, green, blue;
	for (int i = 0; i < centroids.rows; i++)
	{
		const double* Mi = centroids.ptr<double>(i);
		int tmp = best_labels.at<int>(i);
		switch (tmp)
		{
		case 0:
			//cout << "Red" << endl;
			circle(img, Point(Mi[0], Mi[1]), 2.0, Scalar(0, 0, 255), -1, CV_AA);
			red.push_back(centroids.row(i));
			break;
		case 1:
			//cout << "Green" << endl;
			circle(img, Point(Mi[0], Mi[1]), 2.0, Scalar(0, 255, 0), -1, CV_AA);
			green.push_back(centroids.row(i));
			break;
		case 2:
			//cout << "Blue" << endl;
			circle(img, Point(Mi[0], Mi[1]), 2.0, Scalar(255, 0, 0), -1, CV_AA);
			blue.push_back(centroids.row(i));
			break;
		default:
			break;
		}
	}
	Mat yp_red = adjust_line(red), yp_blue = adjust_line(blue), yp_green = adjust_line(green), line_red, line_blue, line_green;
	hconcat(red.col(0), yp_red, line_red);
	display_line(line_red, img);
	hconcat(blue.col(0), yp_blue, line_blue);
	display_line(line_blue, img);
	hconcat(green.col(0), yp_green, line_green);
	display_line(line_green, img);
	imshow("Display window", img);
	return 0;
}

int display_img(string path)
{
	Mat image;
	image = imread(path, IMREAD_COLOR); // Read the file
	if (image.empty()) // Check for invalid input
	{
		cout << "Could not open or find the image" << std::endl;
		return -1;
	}
	resize(image, image, Size(width, height), 0, 0, CV_INTER_AREA);
	namedWindow("Display window", CV_WINDOW_NORMAL); // Create a window for display.
	setWindowProperty("Display window", CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);
	setMouseCallback("Display window", on_mouse, &image);
	imshow("Display window", image); // Show our image inside it.
	get_centroids(image);
	/*cout << image.rows << " " << image.cols << endl;
	cout << image.size << endl << image.type() << endl;*/
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
		CV_AA);
	namedWindow("Display window", CV_WINDOW_NORMAL); // Create a window for display.
	setWindowProperty("Display window", CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);
	imshow("Display window", matPoint); // Show our image inside it.
	waitKey(1/*000*/); // Wait for a keystroke in the window
	return 0;
}

int calibration_points(Mat &a, float x, float y, vector<Point2f> tmp)
{
	//MyGaze *m = new MyGaze();
	display_point(x, y);
	//vector<Point2f> tmp = m->get_readings();
	std::sort(tmp.begin(), tmp.end(), [=](const cv::Point2f &a, const cv::Point2f &b) {
		return (norm(a - Point2f(x, y)) < norm(b - Point2f(x, y)));
	});
	int median = floorf(tmp.size() / 2) - 1;
	/*ofstream outfile;
	outfile.open("test.csv", std::ios_base::app);
	outfile << "X, Y" << endl;*/
	/*for (auto n : tmp)
	{
		//outfile << n.x << ", " << n.y << endl;
		double dist = norm(n - Point2f(x, y));
		cout << "Dist " << dist << endl;
	}*/
	//cout << endl;
	vector<double> arrtmp = { 1, tmp[median - 2].x, tmp[median - 2].y, tmp[median - 2].x * tmp[median - 2].x, tmp[median - 2].y * tmp[median - 2].y, tmp[median - 2].x * tmp[median - 2].y };
	Mat row(arrtmp);
	a.push_back(row.t());
	arrtmp = { 1, tmp[median].x, tmp[median].y, tmp[median].x * tmp[median].x, tmp[median].y * tmp[median].y, tmp[median].x * tmp[median].y };
	row = Mat(arrtmp);
	a.push_back(row.t());
	arrtmp = { 1, tmp[median + 2].x, tmp[median + 2].y, tmp[median + 2].x * tmp[median + 2].x, tmp[median + 2].y * tmp[median + 2].y, tmp[median + 2].x * tmp[median + 2].y };
	row = Mat(arrtmp);
	a.push_back(row.t());
	//m->~MyGaze();
	return 0;
}

int auto_calibrate()
{
	vector<Point2f> fix_1, fix_2, fix_3;
	MyGaze *m = new MyGaze();
	display_img("Diapositiva1.BMP");
	fix_1 = m->get_fixations_vec();
	m->~MyGaze();
	ofstream outfile;
	outfile.open("auto_calib1.csv", std::ios_base::app);
	outfile << "X, Y" << endl;
	for (auto n : fix_1)
	{
		outfile << n.x << ", " << n.y << endl;
	}
	cout << endl;
	outfile.close();

	m = new MyGaze();
	display_img("Diapositiva2.BMP");
	fix_2 = m->get_fixations_vec();
	m->~MyGaze();
	outfile.open("auto_calib2.csv", std::ios_base::app);
	outfile << "X, Y" << endl;
	for (auto n : fix_2)
	{
		outfile << n.x << ", " << n.y << endl;
	}
	cout << endl;
	outfile.close();

	m = new MyGaze();
	display_img("Diapositiva3.BMP");
	fix_3 = m->get_fixations_vec();
	m->~MyGaze();
	outfile.open("auto_calib3.csv", std::ios_base::app);
	outfile << "X, Y" << endl;
	for (auto n : fix_3)
	{
		outfile << n.x << ", " << n.y << endl;
	}
	cout << endl;
	outfile.close();

	return 0;
}

int main(int argc, char **argv)
{
	ofstream outputFile("output.csv");
	string img_1 = "Slide 1-prima.jpg", img_2 = "Slide 2 -prima.jpg", img_3 = "Slide 3 -conteo.jpg";
	vector<Point2f> fix_1, fix_2, fix_3;

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
	vector<double> sx, sy, ex, ey;
	transform(sp.begin(), sp.end(), back_inserter(sx), [](Point2f const &pnt) { return pnt.x; });
	transform(sp.begin(), sp.end(), back_inserter(sy), [](Point2f const &pnt) { return pnt.y; });
	Mat a;/*(sx.size(), 6, CV_32FC2)*/
	///////////////////////////////////////////////////////////////
	/*Data set reading*/
	ifstream inputfile("test.csv");
	vector<vector<double> > values;
	vector<Point2f> tmp;
	for (string line; getline(inputfile, line); )
	{
		replace(line.begin(), line.end(), ',', ' ');
		istringstream in(line);
		values.push_back(
			vector<double>(istream_iterator<double>(in),
				istream_iterator<double>()));
	}
	for (auto n : values)
	{
		//cout << n[0] << " " << n[1] << endl;
		tmp.push_back(Point2f(n[0], n[1]));
	}
	tmp.push_back(Point2f(0, 0));
	int count = 0;
	/////////////////////////////////////////////////////////////
	float y = 100;
	for (int i = 0; i < 3; i++)
	{
		float x = 100;
		for (int j = 0; j < 3; j++)
		{
			vector<Point2f> sub(&tmp[count], &tmp[count + 9]);
			count = count + 9;
			calibration_points(a, x, y, sub);
			x = x + 400;
		}
		y = y + 400;
	}

	//cout << a << endl;
	Mat matsx = Mat(sx);
	Mat matsy = Mat(sy);
	Mat thetax = (a.t() * a).inv() * a.t() * matsx;
	Mat thetay = (a.t() * a).inv() * a.t() * matsy;
	cout << thetax << endl;
	cout << thetay << endl;
	/* Eigen */
	if (a.isContinuous())
	{
		/*Eigen::MatrixXf A(4, 2);
		Eigen::MatrixXf Q, R;
		A << 1, 2,
			3, 4,
			5, 6,
			7, 8;
		Eigen::HouseholderQR<Eigen::MatrixXf> qr(A);
		Q = qr.householderQ();
		R = qr.matrixQR().triangularView<Eigen::Upper>();
		cout << (Q * R) - A << endl;*/
		/*tets*/
		Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> A_Eigen(a.ptr<double>(), a.rows, a.cols);
		/*Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> X_Eigen(matsx.ptr<float>(), matsx.rows, matsx.cols);
		Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> Y_Eigen(matsy.ptr<float>(), matsy.rows, matsy.cols);*/
		Eigen::Map<Eigen::VectorXd> X_Eigen(&sx[0], sx.size());
		Eigen::Map<Eigen::VectorXd> Y_Eigen(&sy[0], sy.size());
		Eigen::MatrixXd Q, R;
		Eigen::VectorXd thetaX_Eigen, thetaY_Eigen;
		thetaX_Eigen = Eigen::VectorXd::Zero(6);
		thetaY_Eigen = Eigen::VectorXd::Zero(6);
		Eigen::HouseholderQR<Eigen::MatrixXd> qr(A_Eigen);
		Q = qr.householderQ();
		R = qr.matrixQR().triangularView<Eigen::Upper>();
		//cout << (Q * R) - A_Eigen << endl;
		//X_Eigen.transposeInPlace();
		/*thetaX_Eigen = R*X_Eigen;
		cout << thetaX_Eigen << endl;*/
		//cout << R << endl << X_Eigen << endl;
		//cout << R.rows() << "x" << R.cols() << endl << X_Eigen.rows() << "x" << X_Eigen.cols() << endl;
		Eigen::MatrixXd QtX = Q.transpose() * X_Eigen;
		Eigen::MatrixXd QtY = Q.transpose() * Y_Eigen;
		for (int i = (R.cols() - 1); i > -1; i--)
		{
			thetaX_Eigen(i) = (QtX(i, 0) - R.row(i).dot(thetaX_Eigen)) / R(i, i);
			thetaY_Eigen(i) = (QtY(i, 0) - R.row(i).dot(thetaY_Eigen)) / R(i, i);
		}
		cout << thetaX_Eigen << endl << endl << thetaY_Eigen << endl;
	}
	else
	{
		cout << "Opencv mat is not continuous" << endl;
	}
	/* Eigen */

	/* Estimate coordinates */
	for (int i = 0; i < a.rows; i++)
	{
		Mat tmpx = a.row(i) * thetax;
		ex.push_back(tmpx.at<double>(0, 0));
		Mat tmpy = a.row(i) * thetay;
		ey.push_back(tmpy.at<double>(0, 0));
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

	//waitKey(0);
	//MyGaze *m = new MyGaze();
	//display_point(300, 300);
	//vector<Point2f> tmp = m->get_readings();
	//std::sort(tmp.begin(), tmp.end(), [=](const cv::Point2f &a, const cv::Point2f &b) {
	//	return (norm(a - Point2f(300, 300)) < norm(b - Point2f(300, 300)));
	//});
	//int median = floorf(tmp.size() / 2) - 1;
	//vector<double> arrtmp = { 1, tmp[median].x, tmp[median].y, tmp[median].x * tmp[median].x, tmp[median].y * tmp[median].y, tmp[median].x * tmp[median].y };
	//Mat row(arrtmp);
	//cout << "here" << endl;
	//cout << row.t() * thetax << endl << row.t() * thetay << endl;
	////fix_1 = m->get_fixations();
	//m->~MyGaze();
	//waitKey(0);

	// Fixation point
	cout << "X: " << width << endl;
	cout << "Y: " << height << endl;
	Mat matPoint(Size(width, height), CV_8UC3);
	matPoint.setTo(Scalar(255, 255, 255));
	circle(matPoint,
		Point(width / 2, 100),
		10.0,
		Scalar(0, 0, 0),
		-1,
		CV_AA);
	imshow("Display window", matPoint);
	waitKey(0);
	// Imagen 1
	MyGaze *m = new MyGaze();
	display_img("Diapositiva4.PNG");
	//fix_1 = m->get_fixations();
	m->~MyGaze();

	//// Imagen 2
	//m = new MyGaze();
	//display_img(img_2);
	////fix_1 = m->get_fixations();
	//m->~MyGaze();

	//// Imagen 3
	//m = new MyGaze();
	//display_img(img_3);
	////fix_1 = m->get_fixations();
	//m->~MyGaze();

	//auto_calibrate();

	return 0;
}