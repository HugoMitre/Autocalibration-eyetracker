#include "MyGaze.h"
#include <iostream>
#include <numeric>
#include <list>
#include <iterator>
#include <opencv2/core.hpp>
using namespace std;
using namespace cv;
// --- MyGaze implementation
MyGaze::MyGaze()
	: m_api(1), // verbose_level 0 (disabled)
	prev_fix(-1000, -1000)
{
	// Connect to the server on the default TCP port (6555)
	if (m_api.connect())
	{
		/*m_api.calibration_clear();
		m_api.calibration_start(9);
		m_api.calibration_point_start(100, 100);*/
		// Enable GazeData notifications
		m_api.add_listener(*this);
	}
}

MyGaze::~MyGaze()
{
	m_api.remove_listener(*this);
	m_api.disconnect();
	fixations.clear();
}

void MyGaze::on_gaze_data(gtl::GazeData const &gaze_data)
{
	if (gaze_data.state & gtl::GazeData::GD_STATE_TRACKING_GAZE)
	{
		gtl::Point2D const &smoothedCoordinates = gaze_data.avg;
		// Move GUI point, do hit-testing, log coordinates, etc.
		//cout << "time: " << gaze_data.time << "; X coordinates: " << gaze_data.avg.x << "; Y coordinates: " << gaze_data.avg.y << endl;
		gd.push_back(Point2f(gaze_data.avg.x, gaze_data.avg.y));
		if (gd.size() == 10)
		{
			analyze(gd);
		}
	}
}

void MyGaze::analyze(list<Point2f> &points)
{
	vector<Point2f> p{ begin(points), end(points) };
	vector<float> x, y;
	transform(p.begin(), p.end(), back_inserter(x), [](Point2f const &pnt) { return pnt.x; });
	transform(p.begin(), p.end(), back_inserter(y), [](Point2f const &pnt) { return pnt.y; });
	sort(x.begin(), x.end());
	sort(y.begin(), y.end());
	/*std::sort(p.begin(), p.end(), [](const cv::Point2f &a, const cv::Point2f &b) {
		return (a.x * a.x + a.y * a.y < b.x * b.x + b.y * b.y);
	});*/
	Point2f median(x[5], y[5]);
	int counter = 0;
	for (auto n : p)
	{
		//cout << n.x << ", " << n.y << endl;
		//cout << "Median: " << median.x << ", " << median.y << endl;
		double dist = norm(n - median);
		//cout << "Distancia: " << dist << endl;
		if (dist <= 80)
		{
			counter++;
			if (counter == 5)
			{
				double dist_prev = norm(median - prev_fix);
				if (dist_prev >= 80)
				{
					fixations.push_back(median);
					prev_fix = median;
					/*cout << endl
						<< "Fixation!" << endl
						<< endl;*/
				}
			}
		}
	}
	points.pop_front();
}

list<Point2f> MyGaze::get_fixations()
{
	return fixations;
}

vector<Point2f> MyGaze::get_readings()
{
	return{ begin(gd), end(gd) };
}

vector<Point2f> MyGaze::get_fixations_vec()
{
	return{ begin(fixations), end(fixations) };
}