#include "lane_detector.hpp"

void LaneDetector::setThresParam(void) {
	max_val = 255;
	l_thresh = 120;
}

void LaneDetector::getInitialFrame(VideoCapture cap) {
	cap>>frame;
}

/*void LaneDetector::setSize(int cols, int rows) {
	size.width = cols;
	size.height = rows;
}*/	

void LaneDetector::resizeFrame(void) {
	resize(frame, frame, Size(1280, 720));
}

void LaneDetector::getWarpedFrame(void) {
	Point2f src[4];
	src[0] = Point(490, 630);	
	src[1] = Point(810, 630);	
	src[2] = Point(300, 720);	
	src[3] = Point(1000, 720);

	Point2f dst[4];
	dst[0] = Point(180, 0);	
	dst[1] = Point(frame.cols - 180, 0);	
	dst[2] = Point(250, frame.rows);	
	dst[3] = Point(frame.cols - 250, frame.rows);
	
	Mat M = getPerspectiveTransform(src, dst);
	warpPerspective(frame, warped_frame, M, warped_frame.size(), INTER_LINEAR, BORDER_CONSTANT);
}

/*void LaneDetector::getSobelFrame(void) {
	int scale = 1;
	int delta = 0;
	int ddepth = CV_16S;
	
	GaussianBlur(warped_frame, gaussian_frame, Size(3, 3), 0, 0, BORDER_DEFAULT);
	cvtColor(gaussian_frame, gray_frame, CV_RGB2GRAY);
	
	Mat grad_x, grad_y, abs_grad_x, abs_grad_y;
	// gradient x
	Scharr(gray_frame, grad_x, ddepth, 1, 0, scale, delta, BORDER_DEFAULT);
	convertScaleAbs(grad_x, abs_grad_x);
	// gradient y
	Scharr(gray_frame, grad_y, ddepth, 0, 1, scale, delta, BORDER_DEFAULT);
	convertScaleAbs(grad_y, abs_grad_y);
	// total gradient
	addWeighted(abs_grad_x, 0.5, abs_grad_y, 0.5, 0, sobel_frame);
}*/

void LaneDetector::getLthreshFrame(void) {
	Mat hls_frame;

	cvtColor(warped_frame, hls_frame, CV_BGR2HLS);
	vector<Mat> hls_images(3);
	split(hls_frame, hls_images);

	GaussianBlur(hls_images[1], gaussian_frame, Size(3, 3), 0, 0, BORDER_DEFAULT);
	
	threshold(gaussian_frame, l_frame, l_thresh, max_val, THRESH_BINARY);
}

void LaneDetector::slidingWindow(void) {
	int width = l_frame.cols / 10;
	int height = l_frame.rows / 10;
	int hist[l_frame.cols] = {0, };
	int lhist[l_frame.cols];
	int rhist[l_frame.cols];
	int mid_point = l_frame.cols / 2;
	int one_eighth_point = mid_point / 4;
	int hist_thread = 300;
	vector<Point2f> lline_points;
	vector<Point2f> rline_points;
	// make histogram about entire image
	for(int j=0;j<l_frame.rows;j++) {
		for(int i=0;i<l_frame.cols;i++) {
			if(l_frame.at<uchar>(j, i)==255) hist[i]++;
		}
	}

	if(ex_leftx_base != 0) {
		int distrib_width = 400;
		double sigma = distrib_width / 12;
		
		double weight_distrib[l_frame.cols] = {0, };
		int start_idx = ex_leftx_base - distrib_width / 2;
		for(int i=start_idx;i<start_idx+distrib_width;i++) {
			weight_distrib[i] = gaussianFilter(i, ex_leftx_base, sigma);
		}
		for(int i=0;i<l_frame.cols;i++) {
			lhist[i] = hist[i] * weight_distrib[i];
		}
	}
	
	if(ex_rightx_base != 0) {
		int distrib_width = 400;
		double sigma = distrib_width / 12;
		
		double weight_distrib[l_frame.cols] = {0, };
		int start_idx = ex_rightx_base - distrib_width / 2;
		for(int i=start_idx;i<start_idx+distrib_width;i++) {
			weight_distrib[i] = gaussianFilter(i, ex_rightx_base, sigma);
		}
		for(int i=0;i<l_frame.cols;i++) {
			rhist[i] = hist[i] * weight_distrib[i];
		}
	}

	int leftx_base = 0;
	int left_max = 0;
	int rightx_base = 0;
	int right_max = 0;
	// find left line point
	for(int i=one_eighth_point;i<mid_point;i++) {
		if(hist[i] > left_max) {
			leftx_base = i;
			left_max = hist[i];
		}
	}
	if(abs(ex_leftx_base - leftx_base) > 100 && leftx_base != 0 && ex_leftx_base != 0) {
		leftx_base = ex_leftx_base;
	}
	else ex_leftx_base = leftx_base;
	// find right line point	
	for(int i=mid_point;i<l_frame.cols-one_eighth_point;i++) {
		if(hist[i] > right_max) {
			rightx_base = i;
			right_max = hist[i];
		}
	}
	if(abs(ex_rightx_base - rightx_base) > 100 && rightx_base != 0 && ex_rightx_base != 0) {
		rightx_base = ex_rightx_base;
	}
	else ex_rightx_base = rightx_base;
	
	cout<<"----------"<<endl;
	cout<<"LEFT LANE AT "<<leftx_base<<endl;
	cout<<"RIGHT LANE AT "<<rightx_base<<endl;
	
	// for more accurate line histogram points
	int prev_leftx_base = leftx_base;
	int prev_rightx_base = rightx_base;

	// find left line points
	for(int k=0;k<10;k++) {
		int lhist_max = 0;
		int lhist_base = 0;
		int lhist_count = 0;
		int lhist[width] = {0, };
		int offset = prev_leftx_base-width/2;
		for(int j=l_frame.rows-height*(k+1);j<l_frame.rows-height*k;j++) {
			for(int i=prev_leftx_base-width/2;i<prev_leftx_base+width/2;i++) {
				if(l_frame.at<uchar>(j, i) == 255) {
					lhist[i-offset]++;
					lhist_count++;
				}		
			}
		}
		for(int i=0;i<width;i++) {
			if(lhist[i] > lhist_max) {
				lhist_max = lhist[i];
				lhist_base = i+offset;
			}
		}
		if(lhist_base != 0 && lhist_count > hist_thread && abs(prev_leftx_base - lhist_base) < 50) {
			lline_points.push_back(Point(lhist_base, l_frame.rows-height*(2*k+1)/2));
			prev_leftx_base = lhist_base;
			//rectangle(warped_frame, Point(lhist_base-64, l_frame.rows-height*(k+1)), Point(lhist_base+64, l_frame.rows-height*k), Scalar(255, 0, 0), 2);
		}
		else {
			lline_points.push_back(Point(prev_leftx_base, l_frame.rows-height*(2*k+1)/2));
		}
	}

	for(int i=0;i<lline_points.size();i++) {
		cout<<i<<" LEFT POINT X : "<<lline_points[i].x<<" POINT Y: "<<lline_points[i].y<<endl;
	}
	
	for(int k=0;k<10;k++) {
		int rhist_max = 0;
		int rhist_base = 0;
		int rhist_count = 0;
		int rhist[width] = {0, };
		int offset = prev_rightx_base-width/2;
		for(int j=l_frame.rows-height*(k+1);j<l_frame.rows-height*k;j++) {
			for(int i=prev_rightx_base-width/2;i<prev_rightx_base+width/2;i++) {
				if(l_frame.at<uchar>(j, i) == 255) {
					rhist[i-offset]++;
					rhist_count++;
				}		
			}
		}
		for(int i=0;i<width;i++) {
			if(rhist[i] > rhist_max) {
				rhist_max = rhist[i];
				rhist_base = i+offset;
			}
		}
		if(rhist_base != 0 && rhist_count > hist_thread && abs(prev_rightx_base - rhist_base) < 50) {
			rline_points.push_back(Point(rhist_base, l_frame.rows-height*(2*k+1)/2));
			prev_rightx_base = rhist_base;
			//rectangle(warped_frame, Point(rhist_base-64, l_frame.rows-height*(k+1)), Point(rhist_base+64, l_frame.rows-height*k), Scalar(255, 0, 0), 2);
		}
		else {
			rline_points.push_back(Point(prev_rightx_base, l_frame.rows-height*(2*k+1)/2));
		}
	}
	
	for(int i=0;i<rline_points.size();i++) {
		cout<<i<<" RIGHT POINT X : "<<rline_points[i].x<<" POINT Y: "<<rline_points[i].y<<endl;
	}
	
	// calculate left line coefficients
	Mat lline_coef(3, 1, CV_32F);
	cvPolyfit(lline_points, lline_coef, 2);
	cout<<lline_coef.at<float>(0, 0)<<endl; 
	cout<<lline_coef.at<float>(1, 0)<<endl; 
	cout<<lline_coef.at<float>(2, 0)<<endl;
	
	for(int i=0;i<l_frame.rows;i++) {
		int cols = lline_coef.at<float>(2, 0) * pow(i, 2) + lline_coef.at<float>(1, 0) * i + lline_coef.at<float>(0, 0);
		line(warped_frame, Point(cols, i), Point(cols, i), Scalar(0, 0, 255), 5);
	}
	 
	// find right line points
	Mat rline_coef(3, 1, CV_32F);
	cvPolyfit(rline_points, rline_coef, 2);
	cout<<rline_coef.at<float>(0, 0)<<endl; 
	cout<<rline_coef.at<float>(1, 0)<<endl; 
	cout<<rline_coef.at<float>(2, 0)<<endl; 

	for(int i=0;i<l_frame.rows;i++) {
		int cols = rline_coef.at<float>(2, 0) * pow(i, 2) + rline_coef.at<float>(1, 0) * i + rline_coef.at<float>(0, 0);
		line(warped_frame, Point(cols, i), Point(cols, i), Scalar(0, 0, 255), 5);
	}
}

double LaneDetector::gaussianFilter(double x, double mu, double sig) {
	return exp((-1)*pow(x-mu, 2.0)/(2*pow(sig, 2.0)));
}

void LaneDetector::cvPolyfit(vector<Point2f>& points, Mat &dst, int order) {	
	int N = points.size();
	Mat src_x(N, 1, CV_32F);
	Mat src_y(N, 1, CV_32F);
	for(int i=0;i<N;i++) {
		src_x.at<float>(i,0) = points[i].y;
		src_y.at<float>(i,0) = points[i].x;
	}
	Mat X, copy;
	X = Mat::zeros(src_x.rows, order+1, CV_32FC1);
	for(int i=0; i<=order; i++) {
		copy = src_x.clone();
		pow(copy, i, copy);
		Mat M1 = X.col(i);
		copy.col(0).copyTo(M1);
	}
	Mat X_t, X_inv;
	transpose(X, X_t);
	Mat temp = X_t * X;
	Mat temp2;
	invert(temp, temp2);
	Mat temp3 = temp2 * X_t;
	Mat W = temp3 * src_y;
	dst = W.clone();
}

void LaneDetector::showFrame(void) {
	// imshow("FRAME", frame);
	imshow("WARPED_FRAME", warped_frame);
	imshow("L_FRAME", l_frame);
	waitKey(1);
}

int main(void) {
	VideoCapture cap;
	cap.open("Demo1.mp4");
	LaneDetector ld;;
	for(;;) {
		ld.setThresParam();
		ld.getInitialFrame(cap);
		//ld.setSize(1280, 720);
		ld.resizeFrame();
		ld.getWarpedFrame();
		//ld.getSobelFrame();
		ld.getLthreshFrame();
		ld.slidingWindow();
		ld.showFrame();
	}
	return 0;
}
