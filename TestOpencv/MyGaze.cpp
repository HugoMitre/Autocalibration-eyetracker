#include "MyGaze.h"
#include <iostream>
#include <list>
using namespace std;
// --- MyGaze implementation
MyGaze::MyGaze()
	: m_api(1) // verbose_level 0 (disabled)
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

void MyGaze::on_gaze_data(gtl::GazeData const & gaze_data)
{
	if (gaze_data.state & gtl::GazeData::GD_STATE_TRACKING_GAZE)
	{
		gtl::Point2D const & smoothedCoordinates = gaze_data.avg;
		// Move GUI point, do hit-testing, log coordinates, etc.
		cout << "time: " << gaze_data.time << "; X coordinates: " << gaze_data.avg.x << "; Y coordinates: " << gaze_data.avg.y << endl;
		gd.push_back(gaze_data);
	}
}