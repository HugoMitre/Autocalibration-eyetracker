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
	imshow("Display window", matPoint); // Show our image inside it.
	waitKey(1000); // Wait for a keystroke in the window
	return 0;
}

int calibration_points(Mat &a, float x, float y, vector<Point2f> tmp)
{
	//MyGaze *m = new MyGaze();
	display_point(x, y);
	//vector<Point2f> tmp = m->get_readings();
	/*vector<float> xs, ys;
	transform(tmp.begin(), tmp.end(), back_inserter(xs), [](Point2f const &pnt) { return pnt.x; });
	transform(tmp.begin(), tmp.end(), back_inserter(ys), [](Point2f const &pnt) { return pnt.y; });
	sort(xs.begin(), xs.end());
	sort(ys.begin(), ys.end());*/
	std::sort(tmp.begin(), tmp.end(), [=](const cv::Point2f &a, const cv::Point2f &b) {
		return (norm(a - Point2f(x, y)) < norm(b - Point2f(x, y)));
	});
	int median = floorf(tmp.size() / 2) - 1;
	/*ofstream outfile;
	outfile.open("test.csv", std::ios_base::app);
	outfile << "X, Y" << endl;*/
	for (auto n : tmp)
	{
		//outfile << n.x << ", " << n.y << endl;
		double dist = norm(n - Point2f(x, y));
		cout << "Dist " << dist << endl;
	}
	cout << endl;
	vector<double> arrtmp = { 1, tmp[median - 2].x, tmp[median - 2].y/*, tmp[median - 2].x * tmp[median - 2].x, tmp[median - 2].y * tmp[median - 2].y, tmp[median - 2].x * tmp[median - 2].y */};
	Mat row(arrtmp);
	a.push_back(row.t());
	arrtmp = { 1, tmp[median].x, tmp[median].y/*, tmp[median].x * tmp[median].x, tmp[median].y * tmp[median].y, tmp[median].x * tmp[median].y */};
	row = Mat(arrtmp);
	a.push_back(row.t());
	arrtmp = { 1, tmp[median + 2].x, tmp[median + 2].y/*, tmp[median + 2].x * tmp[median + 2].x, tmp[median + 2].y * tmp[median + 2].y, tmp[median + 2].x * tmp[median + 2].y */};
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
		Point2f(900, 100), Point2f(900, 100), Point2f(900, 100)//,
		/*Point2f(100, 500), Point2f(100, 500), Point2f(100, 500),
		Point2f(500, 500), Point2f(500, 500), Point2f(500, 500),
		Point2f(900, 500), Point2f(900, 500), Point2f(900, 500),
		Point2f(100, 900), Point2f(100, 900), Point2f(100, 900),
		Point2f(500, 900), Point2f(500, 900), Point2f(500, 900),
		Point2f(900, 900), Point2f(900, 900), Point2f(900, 900) */};
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
	//for (int i = 0; i < 3; i++)
	//{
		float x = 100;
		for (int j = 0; j < 3; j++)
		{
			vector<Point2f> sub(&tmp[count], &tmp[count + 9]);
			count = count + 9;
			calibration_points(a, x, y, sub);
			x = x + 400;
		}
	//	y = y + 400;
	//}

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
		thetaX_Eigen = Eigen::VectorXd::Zero(3);
		thetaY_Eigen = Eigen::VectorXd::Zero(3);
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

	auto_calibrate();

	return 0;
}