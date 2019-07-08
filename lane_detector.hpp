#include "opencv2/opencv.hpp"
#include <cmath>

using namespace cv;
using namespace std;

class LaneDetector {
private:
	Mat frame;
	Mat resized_frame;
	Mat warped_frame;
	Mat gaussian_frame;
	// Mat gray_frame;
	// Mat sobel_frame;
	Mat l_frame;

	// frame size
	//Size size;

	// threshold param
	int max_val;
	int l_thresh;
	
	// former base point
	int ex_leftx_base;
	int ex_rightx_base;
public:
	void setThresParam(void);
	void getInitialFrame(VideoCapture cap);
	//void setSize(int rows, int cols);
	void resizeFrame(void);
	void getWarpedFrame(void);
	// void getSobelFrame(void);
	void getLthreshFrame(void);
	void slidingWindow(void);
	double gaussianFilter(double x, double mu, double sig);
	void cvPolyfit(vector<Point2f> &points, Mat &dst, int order);
	void showFrame(void);
};
