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
Point2f scr_pnt_up((width / 5) * 4, 100), scr_pnt_down(width / 5, height - 100);

Mat read_file(string path)
{
	/*Data set reading*/
	ifstream inputfile(path);
	vector<vector<double>> values;
	//vector<Point2f> tmp;
	vector<double> x, y;
	for (string line; getline(inputfile, line);)
	{
		replace(line.begin(), line.end(), ',', ' ');
		istringstream in(line);
		values.push_back(
			vector<double>(istream_iterator<double>(in),
				istream_iterator<double>()));
	}
	int count = 0;
	for (auto n : values)
	{
		//cout << n[0] << " " << n[1] << endl;
		//tmp.push_back(Point2f(n[0], n[1]));
		if (count != 1 || count != 2)
		{
			x.push_back(n[0]);
			y.push_back(n[1]);
		}
		else
		{
			count++;
		}
	}
	//tmp.push_back(Point2f(0, 0));
	Mat my(y), mx(x), resoult;
	hconcat(mx, my, resoult);
	return resoult;
}

void on_mouse(int event, int x, int y, int, void *imgptr)
{
	Mat &img = (*(Mat *)imgptr);
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
		//rectangle(img, coords[0], coords[1], Scalar(0, 255, 0));
		line(img, coords[0], coords[1], Scalar(0, 255, 0));
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
		const double *p1 = line_ds.ptr<double>(i);
		//circle(img, Point(Mi[0], Mi[1]), 2.0, Scalar(0, 0, 0), -1, CV_AA);
		if (i + 1 < line_ds.rows)
		{
			double *p2 = line_ds.ptr<double>(i + 1);
			line(img, Point(p1[0], p1[1]), Point(p2[0], p2[1]), Scalar(0, 0, 0), 1, CV_AA);
		}
	}
	return 0;
}

Point2f intersect(Eigen::Vector3d line_h, vector<Point2f> pnt_in)
{
	Eigen::Vector3d pnt1, pnt2, line_intersect, pnt_intersect;
	pnt1 << pnt_in[0].x, pnt_in[0].y, 1;
	pnt2 << pnt_in[1].x, pnt_in[1].y, 1;
	line_intersect = pnt1.cross(pnt2);
	cout << line_intersect << "linea de interserccion" << endl;
	cout << pnt1 << endl << pnt2 << "Puntos de fijacion" << endl;
	pnt_intersect = line_h.cross(line_intersect);
	pnt_intersect = pnt_intersect * (1 / pnt_intersect(2));
	return Point2f(pnt_intersect(0), pnt_intersect(1));
}

Point2f proyect_line(Mat &theta, Mat &img, Point2f pnt)
{
	double a = theta.at<double>(0), c = theta.at<double>(1), a2 = a * a;
	int b = -1, b2 = b * b;
	int x = (b * (b * (pnt.x) - a * pnt.y) - (a * c)) / (a2 + b2);
	int y = (a * (-b * (pnt.x) + a * pnt.y) - (b * c)) / (a2 + b2);
	line(img, Point(pnt.x, pnt.y), Point(x, y), Scalar(0, 0, 0), 1, CV_AA);
	return Point2f(x, y);
}

Mat adjust_line(Mat &centroids, Mat &img, vector<Point2f> &pnt, vector<Point2f> pnt_in)
{
	Mat y = centroids.col(1), x = centroids.col(0), one = Mat::ones(x.size(), x.type());
	hconcat(x, one, one);
	Mat theta = (one.t() * one).inv() * one.t() * y;
	Mat yp = (theta.at<double>(0) * x) + theta.at<double>(1);
	Eigen::Vector3d line_h(theta.at<double>(0), -1, theta.at<double>(1));
	vector<Point2f> vec_line, vec_proyect;
	pnt.push_back(intersect(line_h, pnt_in));
	for (auto i : pnt_in)
	{
		pnt.push_back(proyect_line(theta, img, i));
	}
	return yp;
}

int get_centroids(Mat &img, vector<Point2f> &pnts)
{
	Mat grey, binimg, labels, stats, centroids, best_labels, attemps, centers, centroids32f;
	cvtColor(img, grey, COLOR_BGR2GRAY);
	threshold(grey, binimg, 80, 255, THRESH_BINARY_INV | THRESH_OTSU);
	connectedComponentsWithStats(binimg, labels, stats, centroids);
	centroids.convertTo(centroids32f, CV_32F);
	kmeans(centroids32f.col(1), 3, best_labels, TermCriteria(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 10000, 0.0001), 5, KMEANS_PP_CENTERS, centers);
	Mat red, green, blue;
	for (int i = 0; i < centroids.rows; i++)
	{
		const double *Mi = centroids.ptr<double>(i);
		int tmp = best_labels.at<int>(i);
		switch (tmp)
		{
		case 0:
			circle(img, Point(Mi[0], Mi[1]), 2.0, Scalar(0, 0, 255), -1, CV_AA);
			red.push_back(centroids.row(i));
			break;
		case 1:
			circle(img, Point(Mi[0], Mi[1]), 2.0, Scalar(0, 255, 0), -1, CV_AA);
			green.push_back(centroids.row(i));
			break;
		case 2:
			circle(img, Point(Mi[0], Mi[1]), 2.0, Scalar(255, 0, 0), -1, CV_AA);
			blue.push_back(centroids.row(i));
			break;
		default:
			break;
		}
	}
	vector<Point2f> pnt_in, pnt_red, pnt_blue, pnt_green;
	pnt_in.push_back(scr_pnt_up);
	pnt_in.push_back(scr_pnt_down);
	Mat yp_red = adjust_line(red, img, pnt_red, pnt_in), yp_blue = adjust_line(blue, img, pnt_blue, pnt_in), yp_green = adjust_line(green, img, pnt_green, pnt_in), line_red, line_blue, line_green;
	pnts.reserve(pnt_red.size() + pnt_blue.size() + pnt_green.size());
	pnts.insert(pnts.end(), pnt_red.begin(), pnt_red.end());
	pnts.insert(pnts.end(), pnt_blue.begin(), pnt_blue.end());
	pnts.insert(pnts.end(), pnt_green.begin(), pnt_green.end());
	hconcat(red.col(0), yp_red, line_red);
	display_line(line_red, img);
	hconcat(blue.col(0), yp_blue, line_blue);
	display_line(line_blue, img);
	hconcat(green.col(0), yp_green, line_green);
	display_line(line_green, img);
	imshow("Display window", img);
	return 0;
}

Mat display_img(string path)
{
	Mat image;
	image = imread(path, IMREAD_COLOR); // Read the file
	if (image.empty())			// Check for invalid input
	{
		cout << "Could not open or find the image" << std::endl;
	}
	resize(image, image, Size(width, height), 0, 0, CV_INTER_AREA);
	namedWindow("Display window", CV_WINDOW_NORMAL); // Create a window for display.
	setWindowProperty("Display window", CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);
	setMouseCallback("Display window", on_mouse, &image);
	imshow("Display window", image); // Show our image inside it.
	//get_centroids(image);
	/*cout << image.rows << " " << image.cols << endl;
	cout << image.size << endl << image.type() << endl;*/
	waitKey(0); // Wait for a keystroke in the window
	return image;
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
	waitKey(1 /*000*/);			// Wait for a keystroke in the window
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

int factorize_givens(Eigen::MatrixXd &Q, Eigen::MatrixXd &R)
{
	Eigen::MatrixXd idnt = Eigen::MatrixXd::Identity(Q.rows(), Q.cols());
	for (int i = 0; i < R.cols(); i++)
	{
		Eigen::MatrixXd tmp = idnt;
		Eigen::Matrix2d rotation;
		double a = R(i, i), b = R(i + 1, i);
		double hipotenuse = sqrt((a * a) + (b * b));
		rotation << a / hipotenuse, b / hipotenuse,
			-b / hipotenuse, a / hipotenuse;
		tmp.block(i, i, 2, 2) = rotation;
		R = tmp * R;
		cout << R << endl;
		Q = Q * tmp.transpose();
	}
	return 0;
}

int auto_calibrate()
{
	ofstream outfile;
	vector<Point2f> fix_1, fix_2, fix_3;
	Mat fix1_mat, fix2_mat;
	// Fixation points
	//MyGaze* m = new MyGaze();
	Mat matPoint(Size(width, height), CV_8UC3);
	matPoint.setTo(Scalar(255, 255, 255));
	circle(matPoint,
		scr_pnt_up,
		10.0,
		Scalar(0, 0, 0),
		-1,
		CV_AA);
	namedWindow("Display window", CV_WINDOW_NORMAL); // Create a window for display.
	setWindowProperty("Display window", CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);
	imshow("Display window", matPoint); // Show our image inside it.
	waitKey(1000);
	//fix_1 = m->get_fixations_vec();
	//outfile.open("fix1.csv");
	////outfile << "X, Y" << endl;
	//for (auto n : fix_1)
	//{
	//	outfile << n.x << ", " << n.y << endl;
	//}
	//outfile.close();
	//m->~MyGaze();
	//m = new MyGaze();
	matPoint = Mat(Size(width, height), CV_8UC3);
	matPoint.setTo(Scalar(255, 255, 255));
	circle(matPoint,
		scr_pnt_down,
		10.0,
		Scalar(0, 0, 0),
		-1,
		CV_AA);
	//namedWindow("Display window", CV_WINDOW_NORMAL); // Create a window for display.
	//setWindowProperty("Display window", CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);
	imshow("Display window", matPoint); // Show our image inside it.
	waitKey(1000);
	//fix_2 = m->get_fixations_vec();
	//outfile.open("fix2.csv");
	////outfile << "X, Y" << endl;
	//for (auto n : fix_2)
	//{
	//	outfile << n.x << ", " << n.y << endl;
	//}
	//outfile.close();
	//m->~MyGaze();
	// Imagen 1
	//m = new MyGaze();
	Mat img = display_img("Diapositiva4.PNG");
	//waitKey(0);
	vector<Point2f> pnts_img;
	get_centroids(img, pnts_img);
	Mat centroids, best_labels, centers, centroids32f;
	//fix_3 = m->get_fixations_vec();
	//m->~MyGaze();
	//ofstream cali("cali.csv");
	//if (cali.is_open())
	//{
	//	//cali << "X, Y" << endl;
	//	for (auto n : fix_3)
	//	{
	//		cali << n.x << ", " << n.y << endl;
	//	}
	//}
	//else cout << "Unable to open output file";
	//cali.close();
	//Mat fix = read_file("fix.csv");
	//double* fix_in = fix.ptr<double>(0);
	centroids = read_file("cali.csv");
	fix1_mat = read_file("fix1.csv");
	fix2_mat = read_file("fix2.csv");
	fix_1.clear();
	fix_2.clear();
	for (int i = 0; i < fix1_mat.rows; i++)
	{
		double *ptr = fix1_mat.ptr<double>(i);
		fix_1.push_back(Point2f(ptr[0], ptr[1]));
	}
	for (int i = 0; i < fix2_mat.rows; i++)
	{
		double *ptr = fix2_mat.ptr<double>(i);
		fix_2.push_back(Point2f(ptr[0], ptr[1]));
	}
	std::sort(fix_1.begin(), fix_1.end(), [=](const cv::Point2f &a, const cv::Point2f &b) {
		return (norm(a - scr_pnt_up) < norm(b - scr_pnt_up));
	});
	std::sort(fix_2.begin(), fix_2.end(), [=](const cv::Point2f &a, const cv::Point2f &b) {
		return (norm(a - scr_pnt_down) < norm(b - scr_pnt_down));
	});
	centroids.convertTo(centroids32f, CV_32F);
	kmeans(centroids32f.col(1), 3, best_labels, TermCriteria(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 10000, 0.0001), 5, KMEANS_PP_CENTERS, centers);
	Mat red, green, blue;
	for (int i = 0; i < centroids.rows; i++)
	{
		double *Mi = centroids.ptr<double>(i);
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
	vector<Point2f> pnt_in, pnt_red, pnt_blue, pnt_green;
	pnt_in.push_back(fix_1[0]); //floorf(fix_1.size() / 2)]);
	pnt_in.push_back(fix_2[0]); //floorf(fix_2.size() / 2)]); //floorf(fix_2.size() / 2) - 1
	cout << endl;
	for (auto n : pnt_in)
	{
		cout << "punto est " << n.x << ", " << n.y << endl;
	}
	cout << endl;
	Mat yp_red = adjust_line(red, img, pnt_red, pnt_in), yp_blue = adjust_line(blue, img, pnt_blue, pnt_in), yp_green = adjust_line(green, img, pnt_green, pnt_in), line_red, line_blue, line_green;
	//cout << "declaration" << endl;
	hconcat(red.col(0), yp_red, line_red);
	display_line(line_red, img);
	hconcat(blue.col(0), yp_blue, line_blue);
	display_line(line_blue, img);
	hconcat(green.col(0), yp_green, line_green);
	display_line(line_green, img);
	imshow("Display window", img);
	vector<Point2f> pnts_sensor;
	pnts_sensor.reserve(pnt_red.size() + pnt_blue.size() + pnt_green.size());
	pnts_sensor.insert(pnts_sensor.end(), pnt_red.begin(), pnt_red.end());
	pnts_sensor.insert(pnts_sensor.end(), pnt_blue.begin(), pnt_blue.end());
	pnts_sensor.insert(pnts_sensor.end(), pnt_green.begin(), pnt_green.end());
	std::sort(pnts_img.begin(), pnts_img.end(), [=](const cv::Point2f &p1, const cv::Point2f &p2) {
		return (norm(p1 - scr_pnt_up) < norm(p2 - scr_pnt_up));
	});
	std::sort(pnts_sensor.begin(), pnts_sensor.end(), [=](const cv::Point2f &p1, const cv::Point2f &p2) {
		return (norm(p1 - pnt_in[0]) < norm(p2 - pnt_in[0]));
	});
	Eigen::Matrix<double, 11, 6> a;
	a << 1, pnts_sensor[0].x, pnts_sensor[0].y, pnts_sensor[0].x * pnts_sensor[0].x, pnts_sensor[0].y * pnts_sensor[0].y, pnts_sensor[0].x * pnts_sensor[0].y,
		1, pnts_sensor[1].x, pnts_sensor[1].y, pnts_sensor[1].x * pnts_sensor[1].x, pnts_sensor[1].y * pnts_sensor[1].y, pnts_sensor[1].x * pnts_sensor[1].y,
		1, pnts_sensor[2].x, pnts_sensor[2].y, pnts_sensor[2].x * pnts_sensor[2].x, pnts_sensor[2].y * pnts_sensor[2].y, pnts_sensor[2].x * pnts_sensor[2].y,
		1, pnts_sensor[3].x, pnts_sensor[3].y, pnts_sensor[3].x * pnts_sensor[3].x, pnts_sensor[3].y * pnts_sensor[3].y, pnts_sensor[3].x * pnts_sensor[3].y,
		1, pnts_sensor[4].x, pnts_sensor[4].y, pnts_sensor[4].x * pnts_sensor[4].x, pnts_sensor[4].y * pnts_sensor[4].y, pnts_sensor[4].x * pnts_sensor[4].y,
		1, pnts_sensor[5].x, pnts_sensor[5].y, pnts_sensor[5].x * pnts_sensor[5].x, pnts_sensor[5].y * pnts_sensor[5].y, pnts_sensor[5].x * pnts_sensor[5].y,
		1, pnts_sensor[6].x, pnts_sensor[6].y, pnts_sensor[6].x * pnts_sensor[6].x, pnts_sensor[6].y * pnts_sensor[6].y, pnts_sensor[6].x * pnts_sensor[6].y,
		1, pnts_sensor[7].x, pnts_sensor[7].y, pnts_sensor[7].x * pnts_sensor[7].x, pnts_sensor[7].y * pnts_sensor[7].y, pnts_sensor[7].x * pnts_sensor[7].y,
		1, pnts_sensor[8].x, pnts_sensor[8].y, pnts_sensor[8].x * pnts_sensor[8].x, pnts_sensor[8].y * pnts_sensor[8].y, pnts_sensor[8].x * pnts_sensor[8].y,
		1, pnt_in[0].x, pnt_in[0].y, pnt_in[0].x * pnt_in[0].x, pnt_in[0].y * pnt_in[0].y, pnt_in[0].x * pnt_in[0].y,
		1, pnt_in[1].x, pnt_in[1].y, pnt_in[1].x * pnt_in[1].x, pnt_in[1].y * pnt_in[1].y, pnt_in[1].x * pnt_in[1].y;
	Eigen::VectorXd x(11);
	x << pnts_img[0].x, pnts_img[1].x, pnts_img[2].x, pnts_img[3].x, pnts_img[4].x, pnts_img[5].x, pnts_img[6].x, pnts_img[7].x, pnts_img[8].x, scr_pnt_up.x, scr_pnt_down.x;
	Eigen::VectorXd y(11);
	y << pnts_img[0].y, pnts_img[1].y, pnts_img[2].y, pnts_img[3].y, pnts_img[4].y, pnts_img[5].y, pnts_img[6].y, pnts_img[7].y, pnts_img[8].y, scr_pnt_up.y, scr_pnt_down.y;
	Eigen::MatrixXd Q, R;
	Eigen::VectorXd thetaX_Eigen, thetaY_Eigen;
	thetaX_Eigen = Eigen::VectorXd::Zero(6);
	thetaY_Eigen = Eigen::VectorXd::Zero(6);
	Eigen::HouseholderQR<Eigen::MatrixXd> qr(a);
	Q = qr.householderQ();
	R = qr.matrixQR().triangularView<Eigen::Upper>();
	Eigen::MatrixXd QtX = Q.transpose() * x;
	Eigen::MatrixXd QtY = Q.transpose() * y;
	for (int i = (R.cols() - 1); i > -1; i--)
	{
		thetaX_Eigen(i) = (QtX(i, 0) - R.row(i).dot(thetaX_Eigen)) / R(i, i);
		thetaY_Eigen(i) = (QtY(i, 0) - R.row(i).dot(thetaY_Eigen)) / R(i, i);
	}
	cout << thetaX_Eigen << endl
		<< endl
		<< thetaY_Eigen << endl;
	vector<double> ex, ey;
	for (int i = 0; i < a.rows(); i++)
	{
		Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> tmpx = a.row(i) * thetaX_Eigen;
		ex.push_back(tmpx(0, 0));
		Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> tmpy = a.row(i) * thetaY_Eigen;
		ey.push_back(tmpy(0, 0));
	}
	for (auto n : pnts_img)
	{
		cout << "Punto imagen: " << n.x << ", " << n.y << endl;
		circle(img, n, 5, Scalar(0, 0, 100), -1, CV_AA);
	}
	for (auto n : pnts_sensor)
	{
		cout << "Punto sensor: " << n.x << ", " << n.y << endl;
	}
	for (int i = 0; i < ex.size(); i++)
	{
		circle(img, Point(ex[i], ey[i]), 5, Scalar(255, 0, 255), -1, CV_AA);
		cout << "Punto estimado: " << ex[i] << ", " << ey[i] << endl;
	}
	circle(img, scr_pnt_up, 5, Scalar(0, 0, 100), -1, CV_AA);
	circle(img, scr_pnt_down, 5, Scalar(0, 0, 100), -1, CV_AA);

	for (int i = 0; i < centroids.rows; i++)
	{
		Eigen::VectorXd a_red(6);
		double *pred = centroids.ptr<double>(i);
		a_red << 1, pred[0], pred[1], pred[0] * pred[0], pred[1] * pred[1], pred[0] * pred[1];
		double redx = a_red.dot(thetaX_Eigen);
		double redy = a_red.dot(thetaY_Eigen);
		circle(img, Point(redx, redy), 5, Scalar(0, 0, 0), -1, CV_AA);
		//circle(img, Point(pointer[0], pointer[1]), 10, Scalar(0, 0, 0), -1, CV_AA);
		cout << "\nEstimated point " << redx << ", " << redy << endl;
	}

	//cout << "Test point: " << pointer[0] << ", " << pointer[1] << "\nEstimated point " << redx << ", " << redy << endl;
	imshow("Display window", img);
	waitKey(0);

	//m = new MyGaze();
	Point2f pntex1((width / 4) * 3, height / 2);
	matPoint = Mat(Size(width, height), CV_8UC3);
	matPoint.setTo(Scalar(255, 255, 255));
	circle(matPoint,
		pntex1,
		10.0,
		Scalar(0, 0, 0),
		-1,
		CV_AA);
	namedWindow("Display window", CV_WINDOW_NORMAL); // Create a window for display.
	setWindowProperty("Display window", CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);
	imshow("Display window", matPoint); // Show our image inside it.
	waitKey(0);
	vector<Point2f> fix_4;
	//fix_4 = m->get_fixations_vec();
	//outfile.open("fix4.csv");
	////outfile << "X, Y" << endl;
	//for (auto n : fix_4)
	//{
	//	outfile << n.x << ", " << n.y << endl;
	//}
	//outfile.close();
	//m->~MyGaze();
	Mat fix4_mat = read_file("fix4.csv");
	fix_4.clear();
	for (int i = 0; i < fix4_mat.rows; i++)
	{
		double *ptr = fix4_mat.ptr<double>(i);
		fix_4.push_back(Point2f(ptr[0], ptr[1]));
	}
	std::sort(fix_4.begin(), fix_4.end(), [=](const cv::Point2f &a, const cv::Point2f &b) {
		return (norm(a - Point2f((width / 4) * 3, height / 2)) < norm(b - Point2f((width / 4) * 3, height / 2)));
	});
	Point2f pnt_4 = fix_4[floorf(fix_4.size() / 2)];
	Eigen::VectorXd vec_4(6);
	vec_4 << 1, pnt_4.x, pnt_4.y, pnt_4.x * pnt_4.x, pnt_4.y * pnt_4.y, pnt_4.x * pnt_4.y;
	double x_4 = vec_4.dot(thetaX_Eigen), y_4 = vec_4.dot(thetaY_Eigen);
	cout << "x: " << x_4 << endl
		<< "y: " << y_4 << endl;
	circle(matPoint, Point(x_4, y_4), 15, Scalar(0, 0, 100), -1, CV_AA);
	cout << "points up and down :" << endl << scr_pnt_up.x << ", " << scr_pnt_up.y << endl << scr_pnt_down.x << ", " << scr_pnt_down.y << endl;
	imshow("Display window", matPoint); // Show our image inside it.
	waitKey(0);

	/* Add point to calibration */
	Eigen::MatrixXd idnt = Eigen::MatrixXd::Identity(Q.rows() + 1, Q.cols() + 1);
	Eigen::MatrixXd tmp = idnt;
	tmp.block(1, 1, Q.rows(), Q.cols()) = Q;
	Q = tmp;
	/*tmp = Eigen::MatrixXd(a.rows() + 1, a.cols());
	tmp << vec_4.transpose(),
		a;
	cout << Q * tmp << endl;*/
	cout << R << endl;
	tmp = Eigen::MatrixXd(R.rows() + 1, R.cols());
	tmp << vec_4.transpose(),
		R;
	R = tmp;
	cout << endl << R << endl;
	cout << "doing givens..." << endl;
	factorize_givens(Q, R);
	thetaX_Eigen = Eigen::VectorXd::Zero(6);
	thetaY_Eigen = Eigen::VectorXd::Zero(6);
	Eigen::VectorXd vtmpx(x.size() + 1), vtmpy(y.size() + 1);
	vtmpx << (width / 4) * 3, x;
	vtmpy << height / 2, y;
	QtX = Q.transpose() * vtmpx;
	QtY = Q.transpose() * vtmpy;
	for (int i = (R.cols() - 1); i > -1; i--)
	{
		thetaX_Eigen(i) = (QtX(i, 0) - R.row(i).dot(thetaX_Eigen)) / R(i, i);
		thetaY_Eigen(i) = (QtY(i, 0) - R.row(i).dot(thetaY_Eigen)) / R(i, i);
	}
	cout << thetaX_Eigen << endl
		<< endl
		<< thetaY_Eigen << endl;
	x_4 = vec_4.dot(thetaX_Eigen);
	y_4 = vec_4.dot(thetaY_Eigen);
	cout << "x: " << x_4 << endl
		<< "y: " << y_4 << endl;
	circle(matPoint, Point(x_4, y_4), 15, Scalar(0, 100, 100), -1, CV_AA);
	cout << "points up and down :" << endl << scr_pnt_up.x << ", " << scr_pnt_up.y << endl << scr_pnt_down.x << ", " << scr_pnt_down.y << endl;
	imshow("Display window", matPoint); // Show our image inside it.

	/* Minimos cuadrados */
	tmp = Eigen::MatrixXd(a.rows() + 1, a.cols());
	cout << "A size R*C" << a.rows() << " * " << a.cols() << endl;
	tmp << vec_4.transpose(),
		a;
	cout << "A size R*C" << a.rows() << " * " << a.cols() << endl;
	Eigen::VectorXd thetaXMC(6), thetaYMC(6);
	thetaXMC = (tmp.transpose() * tmp).inverse() * tmp.transpose() * vtmpx;
	thetaYMC = (tmp.transpose() * tmp).inverse() * tmp.transpose() * vtmpy;
	cout << "Minimos cuadrados" << endl << thetaXMC << endl << thetaYMC << endl;
	waitKey(0);
	return 0;
}

int main(int argc, char **argv)
{
	//ofstream outputFile("output.csv");
	//string img_1 = "Slide 1-prima.jpg", img_2 = "Slide 2 -prima.jpg", img_3 = "Slide 3 -conteo.jpg";
	//vector<Point2f> fix_1, fix_2, fix_3;

	///* Calibration */
	//vector<Point2f> sp = { Point2f(100, 100), Point2f(100, 100), Point2f(100, 100),
	//	Point2f(500, 100), Point2f(500, 100), Point2f(500, 100),
	//	Point2f(900, 100), Point2f(900, 100), Point2f(900, 100),
	//	Point2f(100, 500), Point2f(100, 500), Point2f(100, 500),
	//	Point2f(500, 500), Point2f(500, 500), Point2f(500, 500),
	//	Point2f(900, 500), Point2f(900, 500), Point2f(900, 500),
	//	Point2f(100, 900), Point2f(100, 900), Point2f(100, 900),
	//	Point2f(500, 900), Point2f(500, 900), Point2f(500, 900),
	//	Point2f(900, 900), Point2f(900, 900), Point2f(900, 900) };
	//vector<double> sx, sy, ex, ey;
	//transform(sp.begin(), sp.end(), back_inserter(sx), [](Point2f const &pnt) { return pnt.x; });
	//transform(sp.begin(), sp.end(), back_inserter(sy), [](Point2f const &pnt) { return pnt.y; });
	//Mat a;/*(sx.size(), 6, CV_32FC2)*/
	/////////////////////////////////////////////////////////////////
	///*Data set reading*/
	//ifstream inputfile("test.csv");
	//vector<vector<double> > values;
	//vector<Point2f> tmp;
	//for (string line; getline(inputfile, line); )
	//{
	//	replace(line.begin(), line.end(), ',', ' ');
	//	istringstream in(line);
	//	values.push_back(
	//		vector<double>(istream_iterator<double>(in),
	//			istream_iterator<double>()));
	//}
	//for (auto n : values)
	//{
	//	//cout << n[0] << " " << n[1] << endl;
	//	tmp.push_back(Point2f(n[0], n[1]));
	//}
	//tmp.push_back(Point2f(0, 0));
	//int count = 0;
	///////////////////////////////////////////////////////////////
	//float y = 100;
	//for (int i = 0; i < 3; i++)
	//{
	//	float x = 100;
	//	for (int j = 0; j < 3; j++)
	//	{
	//		vector<Point2f> sub(&tmp[count], &tmp[count + 9]);
	//		count = count + 9;
	//		calibration_points(a, x, y, sub);
	//		x = x + 400;
	//	}
	//	y = y + 400;
	//}

	////cout << a << endl;
	//Mat matsx = Mat(sx);
	//Mat matsy = Mat(sy);
	//Mat thetax = (a.t() * a).inv() * a.t() * matsx;
	//Mat thetay = (a.t() * a).inv() * a.t() * matsy;
	//cout << thetax << endl;
	//cout << thetay << endl;
	///* Eigen */
	//if (a.isContinuous())
	//{
	//	/*Eigen::MatrixXf A(4, 2);
	//	Eigen::MatrixXf Q, R;
	//	A << 1, 2,
	//		3, 4,
	//		5, 6,
	//		7, 8;
	//	Eigen::HouseholderQR<Eigen::MatrixXf> qr(A);
	//	Q = qr.householderQ();
	//	R = qr.matrixQR().triangularView<Eigen::Upper>();
	//	cout << (Q * R) - A << endl;*/
	//	/*tets*/
	//	Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> A_Eigen(a.ptr<double>(), a.rows, a.cols);
	//	/*Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> X_Eigen(matsx.ptr<float>(), matsx.rows, matsx.cols);
	//	Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> Y_Eigen(matsy.ptr<float>(), matsy.rows, matsy.cols);*/
	//	Eigen::Map<Eigen::VectorXd> X_Eigen(&sx[0], sx.size());
	//	Eigen::Map<Eigen::VectorXd> Y_Eigen(&sy[0], sy.size());
	//	Eigen::MatrixXd Q, R;
	//	Eigen::VectorXd thetaX_Eigen, thetaY_Eigen;
	//	thetaX_Eigen = Eigen::VectorXd::Zero(6);
	//	thetaY_Eigen = Eigen::VectorXd::Zero(6);
	//	Eigen::HouseholderQR<Eigen::MatrixXd> qr(A_Eigen);
	//	Q = qr.householderQ();
	//	R = qr.matrixQR().triangularView<Eigen::Upper>();
	//	//cout << (Q * R) - A_Eigen << endl;
	//	//X_Eigen.transposeInPlace();
	//	/*thetaX_Eigen = R*X_Eigen;
	//	cout << thetaX_Eigen << endl;*/
	//	//cout << R << endl << X_Eigen << endl;
	//	//cout << R.rows() << "x" << R.cols() << endl << X_Eigen.rows() << "x" << X_Eigen.cols() << endl;
	//	Eigen::MatrixXd QtX = Q.transpose() * X_Eigen;
	//	Eigen::MatrixXd QtY = Q.transpose() * Y_Eigen;
	//	for (int i = (R.cols() - 1); i > -1; i--)
	//	{
	//		thetaX_Eigen(i) = (QtX(i, 0) - R.row(i).dot(thetaX_Eigen)) / R(i, i);
	//		thetaY_Eigen(i) = (QtY(i, 0) - R.row(i).dot(thetaY_Eigen)) / R(i, i);
	//	}
	//	cout << thetaX_Eigen << endl << endl << thetaY_Eigen << endl;
	//}
	//else
	//{
	//	cout << "Opencv mat is not continuous" << endl;
	//}
	///* Eigen */

	///* Estimate coordinates */
	//for (int i = 0; i < a.rows; i++)
	//{
	//	Mat tmpx = a.row(i) * thetax;
	//	ex.push_back(tmpx.at<double>(0, 0));
	//	Mat tmpy = a.row(i) * thetay;
	//	ey.push_back(tmpy.at<double>(0, 0));
	//}

	///* Output to file */
	//if (outputFile.is_open())
	//{
	//	outputFile << "Screen coordinates X, Screen coordinates Y, Estimated coordinates X, Estimated coordinates Y" << endl;
	//	for (int i = 0; i < sx.size(); i++)
	//	{
	//		outputFile << sx[i] << ", " << sy[i] << ", " << ex[i] << ", " << ey[i] << endl;
	//	}
	//}
	//else cout << "Unable to open output file";

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

	auto_calibrate();
	return 0;
}