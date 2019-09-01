

/****************************************************
 *提取特征点
 *匹配特征点
 *筛选特征点
 */

#include <iostream>
#include <string>
#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;



int main(int argc, char** argv)
{
    // 读取图片
    cv::Mat rgb1 = cv::imread("./rgb1.png");
    cv::Mat rgb2 = cv::imread("./rgb2.png");


    cv::Mat depth1 = cv::imread("./depth1.png");
    cv::Mat depth2 = cv::imread("./depth2.png");

    // 特征点 和 描述子对象
    cv::Ptr<cv::FeatureDetector> detector;
    cv::Ptr<cv::DescriptorExtractor> descriptor;

    detector = cv::ORB::create();
    descriptor = cv::ORB::create();

    // 特征点 容器
    vector<cv::KeyPoint> keyPoint1, keyPoint2;
    detector->detect(rgb1, keyPoint1);
    detector->detect(rgb2, keyPoint2);

    // 计算描述子
    cv::Mat descriptor1, descriptor2;
    descriptor->compute(rgb1, keyPoint1, descriptor1);
    descriptor->compute(rgb2, keyPoint2, descriptor2);

    cv::Mat feature_img;
    cv::drawKeypoints(rgb1, keyPoint1, feature_img, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    cv::imshow("feature img", feature_img);
    cv::waitKey(0);

    // 匹配特征点
    vector<cv::DMatch> matchers;
    cv::BFMatcher matcher;
    matcher.match(descriptor1, descriptor2, matchers);

    cv::Mat matcher_img;
    cv::drawMatches(rgb1, keyPoint1, rgb2, keyPoint2, matchers, matcher_img);
    cv::imshow("descriptor img", matcher_img);
    cv::waitKey(0);

    // 筛选匹配点
    double minMatches = 9999;
    vector<cv::DMatch> goodMathers;
    for (auto i = 0; i < matchers.size(); i++)
    {
        if (matchers[i].distance < minMatches)
        {
            minMatches = matchers[i].distance;
        }
    }

    cout << "minMatches = " << minMatches << endl;

    for (auto i = 0; i < matchers.size(); i++)
    {
        if( matchers[i].distance < max (2 * minMatches, 30.0) )
        {
            goodMathers.push_back(matchers[i]);
        }
    }


    // 绘制匹配点
    cv::Mat goodMathcer_img;
    cv::drawMatches(rgb1, keyPoint1, rgb2, keyPoint2, goodMathers, goodMathcer_img);
    cv::imshow("goodMather img", goodMathcer_img);
    cv::waitKey(0);

    cout << "hello slam VO......" << endl;
    return 0;
}