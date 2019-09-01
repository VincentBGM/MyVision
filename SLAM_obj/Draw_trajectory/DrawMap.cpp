#include <iostream>
#include <sophus/se3.h>
#include <pangolin/pangolin.h>

#include <string>
#include <fstream>
#include "unistd.h"

using namespace std;

// 旋转矩阵：3×3 matrix3d     旋转向量：3*1 AngleAxisd  四元数：4*1 Quaterniond  欧拉角：3*1 Vector3d
// 欧式变换矩阵: 4*4 Isometry3d: R | t
// 仿射变换: 4*4 Affine3d A | t
// 映射变换: 4*4 Projective3d


static string map_path = "./map.txt";

void DrawMap(vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>>);


void EigenDemo()
{
    // 旋转矩阵3*3
    Eigen::Matrix3d rotation_matrix = Eigen::Matrix3d::Identity();

    // 旋转向量 沿z轴旋转45度
    Eigen::AngleAxisd rotation_vector(M_PI / 4, Eigen::Vector3d(0, 0, 1));

    // 欧拉角 ( ZYX 顺序 yaw pitch, roll) (3*1)
    //Eigen::Vector3d eular_angle = rotation_matrix.eu(2, 1, 0);

    // 变换矩阵 T 4*4
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.rotate(rotation_vector);
    T.pretranslate(Eigen::Vector3d(1, 3, 4));
    cout << T.matrix() << endl;

    // 四元数 4×4
    Eigen::Quaterniond q;
    //q = Quaterniond(rotation_matrix);
    //q = Quaterniond(rotation_vector);
    cout << q.coeffs() << endl;

}


// pangolin 绘制地图
void DrawMap(vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> poses)
{
    if(poses.empty())
    {
        cerr << "轨迹是空。。。" << endl;
        return;
    }

    // 创建轨迹窗口
    pangolin::CreateWindowAndBind("Map Viewer", 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
            pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
    );

    pangolin::View &d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));


    while (pangolin::ShouldQuit() == false)
    {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        d_cam.Activate(s_cam);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

        glLineWidth(2);
        for (size_t i = 0; i < poses.size() - 1; i++)
        {
            glColor3f(1 - (float) i / poses.size(), 0.0f, (float) i / poses.size());
            glBegin(GL_LINES);
            auto p1 = poses[i], p2 = poses[i + 1];
            glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
            glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
            glEnd();
        }
        pangolin::FinishFrame();
        usleep(5000);
    }
}


int main(int argc, char** argv)
{

    vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> poses;

    vector<float > lines;
    string data_line;
    float data;

    Eigen::Quaterniond q;           // 四元数
    Eigen::Vector3d t;              // 平移向量


    ifstream map_file(map_path);
    if(!map_file)
    {
        cerr << "No find this file ....." << endl;
        return -1;
    }

    while (!map_file.eof())
    {
        getline(map_file, data_line);
        stringstream input_str(data_line);
        lines.clear();
        while (input_str >> data)
        {
            lines.push_back(data);
        }

        for(auto i = 0; i < lines.size(); ++i)
        {
            cout << "data[1]: "<< lines[1] << endl;

            // 四元数
            q = Eigen::Quaterniond(lines[7], lines[4], lines[5], lines[6]);

            // 四元数转换成旋转矩阵
            Eigen::Matrix3d R = q.matrix();

            // 初始化平移向量
            t << lines[1], lines[2], lines[3];

        }


        Sophus::SE3 se3_qt(q, t);
        poses.push_back(se3_qt);

    }

    cout << "lines size = " << lines.size() << endl;


    map_file.close();

    DrawMap(poses);
    return 0;
}
