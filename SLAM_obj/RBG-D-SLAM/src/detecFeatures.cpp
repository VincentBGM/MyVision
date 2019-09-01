
#include <iostream>
#include "slamBase.hpp"

#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>

using namespace std;
using namespace cv;

int main(int argc, char** argv)
{
    // 读取深度图 和 rgb 图像
    Mat rgb1 = imread("../data/rgb/rgb1.png");
    Mat rgb2 = imread("../data/rgb/rgb2.png");

    Mat depth1 = imread("../data/depth/depth1.png");
    Mat depth2 = imread("../data/depth/depth2.png");

    cout << "reading image and depth ...." << endl;

    // 特征提取 描述子 对象
    cv::Ptr<cv::FeatureDetector> detector;
    cv::Ptr<cv::DescriptorExtractor> descriptor;

    detector = cv::ORB::create();
    descriptor = cv::ORB::create();

    cout << "reading image and depth ...." << endl;

    vector<cv::KeyPoint> keypoint1, keypoint2;
    detector->detect(rgb1, keypoint2);
    detector->detect(rgb2, keypoint2);


    cv::Mat keyPointImg;
    drawKeypoints(rgb1, keypoint1, keyPointImg, Scalar::all(-1), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    cv::imshow("KeyPoingImg", keyPointImg);
//    imwrite("../data/keyPointImg.png", keyPointImg);
    waitKey(0);

    // 计算描述子
    Mat descript1, descript2;
    descriptor->compute(rgb1, keypoint1, descript1);
    descriptor->compute(rgb2, keypoint2, descript2);

    // 匹配描述子
    vector<DMatch> matchers;
    BFMatcher matcher;
    matcher.match(descript1, descript2, matchers);

    // 保存特征匹配
    Mat MatcherImg;
    cv::drawMatches(rgb1, keypoint1, rgb2, keypoint2, matchers, MatcherImg);
    cv::imshow("MatcherImg", MatcherImg);
//    cv::imwrite("../data/MatherImg.png", MatcherImg);
    waitKey(0);

    // 筛选匹配 选择最小距离匹配
    vector<DMatch> goodMatchers;
    double minDis = 9999;
    for(size_t i = 0; i < matchers.size(); i++)
    {
        if(matchers[i].distance < minDis)
        {
            minDis = matchers[i].distance;
        }
    }

    for(size_t i = 0; i < matchers.size(); i++)
    {
        if(matchers[i].distance < 4 * minDis)
        {
            goodMatchers.push_back(matchers[i]);
        }
    }

    // 保存最小匹配
    Mat goodMatcherImg;
    drawMatches(rgb1, keypoint1, rgb2, keypoint2, goodMatchers, goodMatcherImg);
    imshow("goodMatcherImg", goodMatcherImg);
//    imwrite("../data/goodMatcherImg.png", goodMatcherImg);
    waitKey(0);



    // 计算图像之间的运动
    vector<Point3f> point3D;
    vector<Point2f> point2D;

    CAMERA_INTRINSIC_PARAMETERS camera;
    camera.cx = 325.5;
    camera.cy = 253.5;
    camera.fx = 518.0;
    camera.fy = 519.0;
    camera.scale = 1000.0;

    for(size_t i = 0; i < goodMatchers.size(); i++)
    {

        // 第一张的 图像点坐标
        Point2f  p = keypoint1[goodMatchers[i].queryIdx].pt;

        ushort d = depth1.ptr<ushort>(int(p.y))[int(p.x)];

        if(d == 0)
        {
            continue;
        }

        // 第二张的 图像点坐标
        point2D.push_back( Point2f ( keypoint2[goodMatchers[i].trainIdx].pt ) );

        // 将第一张的图像点坐标转换成3D 坐标

        Point3f point_2f (p.x, p.y, d);
        Point3f point_3f = point2DTo3D(point_2f, camera);

        point3D.push_back(point_3f);

    }

    double camera_matris_data[3][3] = {
            {camera.fx, 0, camera.cx},
            {0, camera.fy, camera.cy},
            {0, 0, 1}
    };

    // 构建相机矩阵
    Mat cameraMatrix(3, 3, CV_64F, camera_matris_data);

    Mat rvec, tvec, inliers;

    solvePnPRansac(point3D, point2D, cameraMatrix, cv::Mat(), rvec, tvec, false, 100, 1.0, 100, inliers);

    vector<DMatch> matchersShow;
    for(size_t i = 0; i < inliers.rows; i++)
    {
        matchersShow.push_back(goodMatchers[inliers.ptr<int>(i)[0]]);
    }

    drawMatches(rgb1, keypoint1, rgb2, keypoint2, matchersShow, MatcherImg);
    imshow("MatcherImg", MatcherImg);
//    imwrite("../data/inliers.png", MatcherImg);
    waitKey(0);


    return 0;
}