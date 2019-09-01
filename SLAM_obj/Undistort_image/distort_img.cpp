#include <iostream>
#include <opencv2/opencv.hpp>
//#include <opencv4/opencv2/opencv.hpp>
#include <QString>

using namespace std;
using namespace cv;

// 图像去畸变过程
// 提供相机的内参 相机的畸变参数
string img_path = "./test.png";


int main(int argc, char** argv)
{
    // 畸变参数
    double K1 = -0.28340811;
    double K2 = 0.07395907,
           p1 = 0.00019359,
           p2 = 1.76187114e-05;

    // 相机的内参
    double fx = 458.654,
           fy = 457.296,
           cx = 367.215,
           cy = 248.375;

    // 将图像转换为灰度图像
    Mat img = imread(img_path, CV_8UC1);
    int row = img.rows,
        cols = img.cols;

    // 去畸变后的图像
    Mat img_distort = Mat(row, cols, CV_8UC1);
    imshow("Image", img);

    // 计算图像畸变后的内容
    for(int v = 0; v < row; v++)
    {
        for(int u = 0; u < cols; u++)
        {
            double u_distored = 0,
                   v_distored = 0;

            // 1、归一化坐标
            //将得到的归一化坐标系进行畸变处理
            //将畸变处理后的坐标通过内参转换为图像坐标系下的坐标
            //这样就相当于是在非畸变图像的图像坐标和畸变图像的图像坐标之间建立了一个对应关系
            //相当于是非畸变图像坐标在畸变图像中找到了映射
            //对畸变图像进行遍历之后，然后赋值（一般需要线性插值，因为畸变后图像的坐标不一定是整数的），即可得到矫正之后的图像
            double x1,
                   y1,
                   x2,
                   y2;

            x1 = (u - cx) / fx;
            y1 = (v - cy) / fy;

            double r2;
            r2 = pow(x1, 2) + pow(y1, 2);

            x2 = x1*(1 + K1*r2 + K2*pow(r2, 2));
            y2 = y1*(1 + K1*r2 + K2*pow(r2, 2));

            u_distored = fx * x2 + cx;
            v_distored = fy * y2 + cy;


            if (u_distored >= 0 && v_distored >= 0 && u_distored < cols && v_distored < row)
            {
                img_distort.at<uchar>(v, u) = img.at<uchar>((int) v_distored, (int) u_distored);
            }
            else
            {
                img_distort.at<uchar>(v, u) = 0;
            }

        }
    }


    imshow("Img Distort", img_distort);
    waitKey();

    cout << "Hello SLAM!" << endl;
    return 0;
}
