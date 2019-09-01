
//#include <string>

#include "myslam/common_include.h"
#include "myslam/config.h"
#include "myslam/visual_odometry.h"


int main(int argc, char** argv)
{

    if ( argc != 2 )
    {
        cout << "usage: run_vo parameter_file ...." << endl;
        return 1;
    }

    myslam::Config::setParameterFile( argv[1] );

    myslam::VisualOdometry::Ptr vo ( new myslam::VisualOdometry );

//    std::string dataset_dir = myslam::Config::get<std::string>("dataset_dir");
    std::string dataset_dir = "/home/vincent/slam_data";

    cout << "dataset: " << dataset_dir << endl;

    ifstream fin( dataset_dir + "/associate.txt");


    if (!fin)
    {
        cout << "please generate the associate file called associate.txt!" << endl;
        return 1;
    }

    vector<string > rgb_files, depth_files;
    vector<double > rgb_times, depth_times;
    while ( !fin.eof() )
    {
        string rgb_file, depth_file, rgb_time, depth_time;

        fin >> rgb_time >> rgb_file >> depth_time >> depth_file;

        rgb_times.push_back( atof( rgb_time.c_str() ) );
        depth_times.push_back( atof( depth_time.c_str() ) );
        rgb_files.push_back( dataset_dir + "/" + rgb_file );
        depth_files.push_back( dataset_dir + "/" + depth_file );

        if (!fin.good())
        {
            break;
        }
    }

    myslam::Camera::Ptr camera ( new myslam::Camera );

    // visual
    cv::viz::Viz3d vis("Visual Odometrt ");
    cv::viz::WCoordinateSystem world_coor(1.0), camera_coor(0.5);                       // 创建坐标系
    cv::Point3d cam_pos(0, -1.0, -1.0), cam_focal_point(0, 0, 0), cam_y_dir(0, 1, 0);       // 相机的位置 焦点
    cv::Affine3d cam_pose = cv::viz::makeCameraPose( cam_pos, cam_focal_point, cam_y_dir);
    vis.setViewerPose( cam_pose );

    world_coor.setRenderingProperty(cv::viz::LINE_WIDTH, 2.0);
    camera_coor.setRenderingProperty(cv::viz::LINE_WIDTH, 1.0);
    vis.showWidget( "World", world_coor);
    vis.showWidget( "Camera", camera_coor);

    cout << "read total " << rgb_files.size() << " entrise " << endl;

    for (auto i = 0; i < rgb_files.size(); i++)
    {

        cv::Mat color = cv::imread( rgb_files[i] );
        cv::Mat depth = cv::imread( depth_files[i] );
        if ( color.data == nullptr || depth.data == nullptr )
        {
            cout << "Do not find color or depth data...." << endl;
            break;
        }

        myslam::Frame::Ptr pFrame = myslam::Frame::creatFrame();
        pFrame->camera_ = camera;
        pFrame->color_ = color;
        pFrame->depth_ = depth;
        pFrame->time_stamp_ = rgb_times[i];


        boost::timer timer;

        vo->addFrame( pFrame );
        cout << "VO costs time: " << timer.elapsed() << endl;

        if ( vo->state_ == myslam::VisualOdometry::LOST )
        {
            cout << "VO state is LOST......please checking....." << endl;
            break;
        }

        Sophus::SE3 Tcw = pFrame->T_w_c_.inverse();


        // show map  and camera pose
        cv::Affine3d M (
                cv::Affine3d::Mat3 (
                        Tcw.rotation_matrix()(0, 0), Tcw.rotation_matrix()(0, 1), Tcw.rotation_matrix()(0, 2),
                        Tcw.rotation_matrix()(1, 0), Tcw.rotation_matrix()(1, 1), Tcw.rotation_matrix()(1, 2),
                        Tcw.rotation_matrix()(2, 0), Tcw.rotation_matrix()(2, 1), Tcw.rotation_matrix()(2, 2)

                        ),
                cv::Affine3d::Vec3 (
                        Tcw.translation()(0, 0), Tcw.translation()(1, 0), Tcw.translation()(2, 0)
                        )
                );




        cv::imshow("image", color);
        cv::waitKey(1);
        vis.setWidgetPose("Camera", M);

        vis.spinOnce(1, false);
    }


    return 0;
}

