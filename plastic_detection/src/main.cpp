#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>

using namespace cv;
using namespace std;

int main( int argc, char** argv )
{

	Mat image;
	image = cv::imread("/home/xiongyi/Pictures/plastic.jpeg");
	if(! image.data )                              // Check for invalid input
	{
		cout <<  "Could not open or find the image" << std::endl ;
		return -1;
	}
	cv::namedWindow( "Display window");
	cv::imshow( "Display window", image );                   // Show our image inside it.
	cv::waitKey(0);                                          // Wait for a keystroke in the window
	return 0;
}
