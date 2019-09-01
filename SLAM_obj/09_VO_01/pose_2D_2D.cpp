
#include <iostream>
#include <string>
#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>


using namespace std;

/****************************************************
 * 使用2D-2D的特征匹配估计相机运动
 * **************************************************/

// 特征匹配
void find_feature_extraction(const cv::Mat& img1, const cv::Mat& img2,
                            vector<cv::KeyPoint>& KeyPoints1, vector<cv::KeyPoint>& KeyPoints2,
                            vector<cv::DMatch>& goodMatchers)
{
    // 特征点 和 描述子对象
    cv::Ptr<cv::FeatureDetector> detector;
    cv::Ptr<cv::DescriptorExtractor> descriptor;

    detector = cv::ORB::create();
    descriptor = cv::ORB::create();

    // 特征点 容器
    detector->detect(img1, KeyPoints1);
    detector->detect(img2, KeyPoints2);

    // 计算描述子
    cv::Mat descriptor1, descriptor2;
    descriptor->compute(img1, KeyPoints1, descriptor1);
    descriptor->compute(img2, KeyPoints2, descriptor2);

    // 匹配特征点
    vector<cv::DMatch> Matchers;
    cv::BFMatcher matcher;
    matcher.match(descriptor1, descriptor2, Matchers);

    // 筛选匹配点
    double minMatches = 9999;
    for (auto i = 0; i < Matchers.size(); i++)
    {
        if (Matchers[i].distance < minMatches)
        {
            minMatches = Matchers[i].distance;
        }
    }

    cout << "minMatches = " << minMatches << endl;

    for (auto i = 0; i < Matchers.size(); i++)
    {
        if( Matchers[i].distance < max (2 * minMatches, 30.0) )
        {
            goodMatchers.push_back(Matchers[i]);
        }
    }
}

// 相机位姿估计
void estimation_pose_2D_2D(const vector<cv::KeyPoint>& KeyPoints1, const vector<cv::KeyPoint>& KeyPoints2,
                            const vector<cv::DMatch>& goodMatchers, cv::Mat R, cv::Mat t)
{
    // 相机内参
    cv::Mat K = ( cv::Mat_<double> ( 3,3 ) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1 );

    // 转换特征点为向量
    vector<cv::Point2f> points1;
    vector<cv::Point2f> points2;

    for (auto i = 0; i < (int)goodMatchers.size(); i++)
    {
        points1.push_back(KeyPoints1[goodMatchers[i].queryIdx].pt);     // 第一张图的匹配点
        points2.push_back(KeyPoints2[goodMatchers[i].trainIdx].pt);     // 第二张图的匹配点
    }

    // 计算基础矩阵
    cv::Mat fundamental_matrix;
    fundamental_matrix = cv::findFundamentalMat(points1, points2, cv::FM_8POINT);
    cout << "基础矩阵 fundamentalMatrix = " << endl << fundamental_matrix << endl;

    // 计算本质矩阵 E = t^R
    cv::Mat essential_matrix;
    cv::Point2d principal_point ( 325.1, 249.7 );	//相机光心, TUM dataset标定值
    double focal_length = 521;			//相机焦距, TUM dataset标定值
    essential_matrix = cv::findEssentialMat(points1, points2, focal_length, principal_point, cv::RANSAC);
    cout << "本质矩阵 essentialMatrix = \n" << essential_matrix << endl;

    // 计算单应矩阵
    cv::Mat homography_matrix;
    homography_matrix = cv::findHomography(points1, points2, cv::RANSAC, 3, cv::noArray(), 2000, 0.99);
    cout << "但应矩阵 homography = \n" << homography_matrix << endl;

    // 从本质矩阵估计相机位姿
    cv::recoverPose(essential_matrix, points1, points2, R, t, focal_length, principal_point);
    cout << "旋转矩阵 R = " << endl << R << endl;
    cout << "平移向量 t = " << endl << t << endl;

}


// 像素坐标系 转换为 归一化坐标系
cv::Point2d pixel2cam ( const cv::Point2d& p, const cv::Mat& K )
{
    cv::Point2f newP;
    newP = cv::Point2f( (p.x - K.at<double>(0, 2)) / K.at<double>(0, 0),
                        (p.y - K.at<double>(1, 2)) / K.at<double>(1, 1));

    return newP;
}


int main(int argc, char** argv)
{
    cv::Mat rgb1 = cv::imread("./rgb1.png");
    cv::Mat rgb2 = cv::imread("./rgb2.png");

    if(rgb1.empty())
    {
        cerr << "Do not find rgb1 image....." << endl;
        return -1;
    }
    if(rgb2.empty())
    {
        cerr << "DO not find rgb2 image....." << endl;
        return -1;
    }


    // 特征匹配
    vector<cv::DMatch> goodMatchers;
    vector<cv::KeyPoint> keyPoints1, keyPoints2;
    find_feature_extraction(rgb1, rgb2, keyPoints1, keyPoints2, goodMatchers);

    cout << "匹配的数量为： " << goodMatchers.size() << endl;


    // 相机位姿估计
    cv::Mat R, t;
    estimation_pose_2D_2D(keyPoints1, keyPoints2, goodMatchers, R, t);

    // 验证 本质矩阵E = t ^ R
    // t 的反对称矩阵
    cv::Mat t_x = ( cv::Mat_<double> ( 3,3 ) <<
            0, -t.at<double> ( 2,0 ), t.at<double>(1, 0),
            t.at<double> ( 2,0 ), 0, -t.at<double>(0, 0),
            -t.at<double>(1, 0), t.at<double>(0, 0), 0);

    cout << "t_x: " << endl << t_x << endl;

//    cv::Mat t_x = ( cv::Mat_<double> (3, 3) <<
//                                             0, 0.1586811098560321,  -0.9516089866836537,
//                            -0.1586811098560321,                   0, -0.1586811098560321,
//                            0.9516089866836537,  0.1586811098560321,                  0);

    cout << "本质矩阵E: t^R = " << endl << t_x * R << endl;


    // 验证堆积约束
//    k = fx,  0, cx,
//         0, fy, cy,
//         0,  0,  1,
    cv::Mat K = ( cv::Mat_<double> (3, 3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1 );
    for( auto i = 0; i < goodMatchers.size(); i++)
    {
        // 第一张图的 归一后的相机坐标系点
        cv::Point2d point1_new = pixel2cam(keyPoints1[goodMatchers[i].queryIdx].pt, K);
        cout << "point1_new = " << point1_new << endl;

        // 第一张图的 归一后的相机坐标系点
        cv::Point2d point2_new = pixel2cam(keyPoints2[goodMatchers[i].trainIdx].pt, K);
        cout << "point2_new = " << point2_new << endl;

        // 第一张图 归一化的坐标向量
        cv::Mat y1 = ( cv::Mat_<double >(3, 1) << point1_new.x, point1_new.y, 1);
        cout << "y1 = " << y1 << endl;

        // 第一张图 归一化的坐标向量
        cv::Mat y2 = ( cv::Mat_<double >(3, 1) << point2_new.x, point2_new.y, 1);
        cout << "y2 = " << y2 << endl;

        //
        cv::Mat d = y2.t() * t_x * R * y1;

        cout << "堆积约束  = " << d << endl;
    }


    cout << "hello slam VO ............." << endl;
    return 0;
}