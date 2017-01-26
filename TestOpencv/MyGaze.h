#include <gazeapi.h>
#include <list>
using namespace std;
// --- MyGaze definition
class MyGaze : public gtl::IGazeListener
{
public:
	MyGaze();
	~MyGaze();
private:
	// IGazeListener
	void on_gaze_data(gtl::GazeData const & gaze_data);
private:
	gtl::GazeApi m_api;
	list<gtl::GazeData> gd;
};