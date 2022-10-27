#include "opencv2/opencv.hpp"

using namespace cv;

int main(int, char**)
{
    VideoCapture cap("http://192.168.2.2:8080/?action=stream"); // open the default camera
    if(!cap.isOpened()) {
        printf("can't open video stream!\n");
        return -1;
    }

    Mat edges;
    namedWindow("frame",1);
    for(;;)
    {
        Mat frame;
        cap >> frame; 
        imshow("frame", frame);
        if(waitKey(30) >= 0) break;
    }
    // the camera will be deinitialized automatically in VideoCapture destructor
    return 0;
}