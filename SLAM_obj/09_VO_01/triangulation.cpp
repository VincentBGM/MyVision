
#include <iostream>
#include <string>
#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>


using namespace std;

string rgb1_path = "./rgb1.png";
string rgb2_path = "./rgb2.png";


// 特征提取 特征匹配
void find_feature_matches(cv::Mat& img1, cv::Mat& img2,
                          vector<cv::KeyPoint>& KeyPoint1, vector<cv::KeyPoint>& KeyPoint2,
                          vector<cv::DMatch>& goodMatchers
                         )
{
    // 特征点 描述子对象
    cv::Ptr<cv::FeatureDetector> detector;
    cv::Ptr<cv::DescriptorExtractor> descriptor;

    detector = cv::ORB::create();
    descriptor = cv::ORB::create();

    // 特征提取
    detector->detect(img1, KeyPoint1);
    detector->detect(img2, KeyPoint2);

    // 特征描述子
    cv::Mat descriptor_1, descriptor_2;

    // 提取特征描述子
    descriptor->compute(img1, KeyPoint1, descriptor_1);
    descriptor->compute(img2, KeyPoint2, descriptor_2);

    // 特征匹配
    vector<cv::DMatch> matchers;
    cv::BFMatcher matcher;
    matcher.match(descriptor_1, descriptor_2, matchers);

    // 筛选匹配
    double minDistance = 1000, maxDistance = 0;


    cout << "matchers  size = " << matchers.size() << endl;
    cout << "decriptor_1 size = " << descriptor_1.rows << endl;


    for(auto i = 0; i < descriptor_1.rows; i++)
    {
        double distance = matchers[i].distance;
        if( distance < minDistance )
        {
            minDistance = distance;
        }

        if( distance > maxDistance )
        {
            maxDistance = distance;
        }
    }

    for(auto i = 0 ; i < descriptor_1.rows; i++)
    {
        if( matchers[i].distance < max( 2 * minDistance, 30.0))
        {
            goodMatchers.emplace_back(matchers[i]);
        }
    }
}

// 计算基础、本质、但应矩阵 计算R t
void estimation_pose_2D_2D(const vector<cv::KeyPoint>& KeyPoint1, const vector<cv::KeyPoint>& KeyPoint2,
                           const vector<cv::DMatch>& goodMatchers, cv::Mat& R, cv::Mat& t
                          )
{
    // 相机的内存参
    cv::Mat K = (cv::Mat_ <double> (3, 3)<< 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);

    // 提取匹配的每一组特征点
    vector<cv::Point2f> point1;
    vector<cv::Point2f> point2;

    for(auto i = 0; i < goodMatchers.size(); i++)
    {
        point1.emplace_back(KeyPoint1[goodMatchers[i].trainIdx].pt);
        point2.emplace_back(KeyPoint2[goodMatchers[i].trainIdx].pt);
    }

    // 计算基础矩阵
    cv::Mat fundamental_matrix;
    fundamental_matrix = cv::findFundamentalMat(point1, point2, cv::FM_8POINT);
    cout << "基础矩阵 fundamental matrix = \n" << fundamental_matrix << endl;

    // 计算本质矩阵 E = t^R
    cv::Mat essential_matrix;
    cv::Point2d principal_point ( 325.1, 249.7 );	//相机光心, TUM dataset标定值
    double focal_length = 521;			//相机焦距, TUM dataset标定值
    essential_matrix = cv::findEssentialMat(point1, point2, focal_length, principal_point, cv::RANSAC);
    cout << "本质矩阵 essential matrix = \n" << essential_matrix << endl;

    // 据算但应矩阵
    cv::Mat homography_matrix;
    homography_matrix = cv::findHomography(point1, point2, cv::RANSAC, 3, cv::noArray(), 2000, 0.99);
    cout << "单应矩阵 homography matrix = \n" << homography_matrix << endl;

    // 计算 变换矩阵 R t
    cv::recoverPose(essential_matrix, point1, point2, R, t, focal_length, principal_point);
    cout << "旋转矩阵 R = \n" << R << endl;
    cout << "平移向量 t = \n" << t << endl;
}


// 将像素坐标 转换为相机坐标
cv::Point2d pixel2cam ( const cv::Point2d& p, const cv::Mat& K )
{
    cv::Point2f newP;
    newP = cv::Point2f( (p.x - K.at<double>(0, 2)) / K.at<double>(0, 0),
                        (p.y - K.at<double>(1, 2)) / K.at<double>(1, 1));

    return newP;
}


// 三角化
void triangulation(const vector<cv::KeyPoint>& KeyPoint1, const vector<cv::KeyPoint>& KeyPoint2,
                   const vector<cv::DMatch>& goodMatchers, const cv::Mat& R, const cv::Mat& t,
                   vector<cv::Point3d>& Points)
{
    cv::Mat T1 = (cv::Mat_<float> (3, 4) <<
                                1, 0, 0, 0,
                                0, 1, 0, 0,
                                0, 0, 1, 0);

    cv::Mat T2 = (cv::Mat_<float> (3, 4) <<
                                R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2), t.at<double>(0, 0),
                                R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2), t.at<double>(1, 0),
                                R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2), t.at<double>(2, 0));

    cv::Mat K = ( cv::Mat_<double> (3, 3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1 );

    // 将像素坐标转换为相机坐标
    vector<cv::Point2f> points_1, points_2;
    for(auto i = 0; i < goodMatchers.size(); i++)
    {
        points_1.emplace_back( pixel2cam( KeyPoint1[goodMatchers[i].queryIdx].pt, K) );
        points_2.emplace_back( pixel2cam( KeyPoint2[goodMatchers[i].trainIdx].pt, K) );
    }

    //
    cv::Mat points_4d;
    cv::triangulatePoints(T1, T2, points_1, points_2, points_4d);

    // 转换为 齐次坐标
    for(auto i = 0; i < points_4d.cols; i++)
    {
        cv::Mat x = points_4d.col(i);

        x /= x.at<float>(3, 0); // 归一化

        cv::Point3d p (
                x.at<float>(0, 0),
                x.at<float>(1, 0),
                x.at<float>(2, 0)
        );

        Points.emplace_back( p );
    }
}


int main(int argc, char** argv)
{

    // 读取图片
    cv::Mat rgb1 = cv::imread(rgb1_path);
    cv::Mat rgb2 = cv::imread(rgb2_path);

    if(rgb1.empty())
    {
        cerr << "Do not find this rgb1 img..." << endl;
        return -1;
    }
    if(rgb2.empty())
    {
        cerr << "Do not find this rgb2.png img..." << endl;
    }

    // 找特征点 匹配特征点
    vector<cv::KeyPoint> keypoint1, keypoint2;
    vector<cv::DMatch> goodMatchers;
    find_feature_matches(rgb1, rgb2, keypoint1, keypoint2, goodMatchers);

    // 估计2张图像的 相机运动
    cv::Mat R, t;
    estimation_pose_2D_2D(keypoint1, keypoint2, goodMatchers, R, t);

    // 三角化
    vector<cv::Point3d> points;
    triangulation(keypoint1, keypoint2, goodMatchers, R, t, points);


    // 验证三角化
    //-- 验证三角化点与特征点的重投影关系
    cv::Mat K = ( cv::Mat_<double> ( 3,3 ) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1 );
    for ( int i=0; i< goodMatchers.size(); i++ )
    {
        cv::Point2d pt1_cam = pixel2cam( keypoint1[ goodMatchers[i].queryIdx ].pt, K );
        cv::Point2d pt1_cam_3d(
                points[i].x / points[i].z,
                points[i].y / points[i].z
        );

        cout<<"point in the first camera frame: "<< pt1_cam <<endl;
        cout<<"point projected from 3D "<<pt1_cam_3d<<", d="<<points[i].z<<endl;

        // 第二个图
        cv::Point2f pt2_cam = pixel2cam( keypoint2[ goodMatchers[i].trainIdx ].pt, K );
        cv::Mat pt2_trans = R*( cv::Mat_<double>(3,1) << points[i].x, points[i].y, points[i].z ) + t;
        pt2_trans /= pt2_trans.at<double>(2,0);
        cout<<"point in the second camera frame: "<<pt2_cam<<endl;
        cout<<"point reprojected from second frame: "<<pt2_trans.t()<<endl;
        cout<<endl;
    }





    cout << "hello slam VO .........." << endl;
    return 0;
}