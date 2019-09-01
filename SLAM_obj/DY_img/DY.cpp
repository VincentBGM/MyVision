
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <iostream>
#include <limits>
#include <numeric>

using namespace cv;
using namespace std;


struct userdata{
    Mat im;
    vector<Point2f> points;
};


void mouseHandler(int event, int x, int y, int flags, void* data_ptr)
{
    if  ( event == EVENT_LBUTTONDOWN )
    {
        userdata *data = ((userdata *) data_ptr);
        circle(data->im, Point(x, y), 3, Scalar(0, 255, 255), 5, CV_8U);
        imshow("Image", data->im);
        if (data->points.size() < 4)
        {
            data->points.emplace_back(Point2f(x, y));
        }
    }

}

void showFinal(const cv::Mat& src1, const cv::Mat& src2)
{
    Mat gray, gray_inv, src1final, src2final;
    cvtColor(src2, gray, COLOR_BGR2GRAY);
    threshold(gray, gray, 0, 255, THRESH_BINARY);

    //adaptiveThreshold(gray,gray,255,ADAPTIVE_THRESH_MEAN_C,THRESH_BINARY,5,4);
    bitwise_not(gray, gray_inv);
    src1.copyTo(src1final, gray_inv);
    src2.copyTo(src2final, gray);
    Mat finalImage = src1final + src2final;
    namedWindow("output", WINDOW_AUTOSIZE);
    imshow("output", finalImage);
    waitKey(0);
}


int main( int argc, char** argv)
{

    // Read in the image.
    //Mat im_src = imread("first-image.jpg");
    cv::Mat im_src = imread("./cvlife.jpg");
    cv::Size size = im_src.size();

    // Create a vector of points.
    vector<Point2f> pts_src;
    pts_src.emplace_back(Point2f(0,0));
    pts_src.emplace_back(Point2f(size.width - 1, 0));
    pts_src.emplace_back(Point2f(size.width - 1, size.height -1));
    pts_src.emplace_back(Point2f(0, size.height - 1 ));



    // Destination image
    cv::Mat im_dst = imread("./ad.jpg");


    // Set data for mouse handler
    cv::Mat im_temp = im_dst.clone();
    userdata data;
    data.im = im_temp;


    //show the image
    cv::imshow("Image", im_temp);


    cout << "Click on four corners of a billboard and then press ENTER............." << endl;
    //set the callback function for any mouse event
    setMouseCallback("Image", mouseHandler, &data);
    waitKey(0);


    //vector<Point2f> pts_dst;

    cv::Mat m1 = Mat(pts_src);
    cv::Mat m2 = Mat(data.points);//data->points报错
    cv::Mat status;
    cv::Mat h = cv::findHomography(m2, m1, status, 0, 3);
    cv::Mat M = cv::getPerspectiveTransform(pts_src, data.points);
    cv::warpPerspective(im_src, im_temp, M, im_temp.size());
    cv::imshow("Image", im_temp);
    cv::waitKey(0);

    // im_dst=im_src+im_dst;
    // Mat im_dst0 = imread("ad.jpg");
    // showFinal(im_dst0,im_dst);

    //填充
    Point PointArray[4];
    PointArray[0] = data.points[0];
    PointArray[1] = data.points[1];
    PointArray[2] = data.points[2];
    PointArray[3] = data.points[3];
    cv::fillConvexPoly(im_dst, PointArray, 4, Scalar(0), CV_8U);
    cv::imshow("Image", im_dst);
    cv::waitKey(0);
    im_dst = im_dst + im_temp;


    // Display image.
    cv::imshow("Image", im_dst);
    cv::waitKey(0);

    cout << "hello slam....." << endl;
    return 0;
}