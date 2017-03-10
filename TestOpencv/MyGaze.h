#include <gazeapi.h>
#include <list>
#include <opencv2/core.hpp>
using namespace std;
using namespace cv;
// --- MyGaze definition
class MyGaze : public gtl::IGazeListener
{
public:
	MyGaze();
	~MyGaze();
	list<Point2f> get_fixations();
	vector<Point2f> get_readings();
	vector<Point2f> get_fixations_vec();

private:
	// IGazeListener
	void on_gaze_data(gtl::GazeData const &gaze_data);
	void analyze(list<Point2f> &points);

private:
	gtl::GazeApi m_api;
	list<Point2f> gd;
	list<Point2f> fixations;
	Point2f prev_fix;
};