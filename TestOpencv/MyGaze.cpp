#include "MyGaze.h"
#include <iostream>
#include <numeric>
#include <list>
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
		// Enable GazeData notifications
		m_api.add_listener(*this);
	}
}

MyGaze::~MyGaze()
{
	m_api.remove_listener(*this);
	m_api.disconnect();
}

void MyGaze::on_gaze_data(gtl::GazeData const &gaze_data)
{
	if (gaze_data.state & gtl::GazeData::GD_STATE_TRACKING_GAZE)
	{
		gtl::Point2D const &smoothedCoordinates = gaze_data.avg;
		// Move GUI point, do hit-testing, log coordinates, etc.
		cout << "time: " << gaze_data.time << "; X coordinates: " << gaze_data.avg.x << "; Y coordinates: " << gaze_data.avg.y << endl;
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
	std::sort(p.begin(), p.end(), [](const cv::Point2f &a, const cv::Point2f &b) {
		return (a.x * a.x + a.y * a.y < b.x * b.x + b.y * b.y);
	});
	Point2f median = p[5];
	int counter = 0;
	for (auto n : p)
	{
		cout << n.x << ", " << n.y << endl;
		cout << "Mean: " << median.x << ", " << median.y << endl;
		double dist = norm(n - median);
		cout << "Distancia: " << dist << endl;
		if (dist <= 100)
		{
			counter++;
			if (counter == 5)
			{
				double dist_prev = norm(median - prev_fix);
				if (dist_prev >= 100)
				{
					fixations.push_back(median);
					prev_fix = median;
					cout << endl
						<< "Fixation!" << endl;
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