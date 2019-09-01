
#ifndef POINTCLOUD_CONFIG_H
#define POINTCLOUD_CONFIG_H


#include "myslam/common_include.h"
//#include "myslam/camera.h"
//#include "myslam/frame.h"
//#include "myslam/map.h"
//#include "myslam/mappoint.h"


namespace myslam
{
    class Config
    {
    private:
        static std::shared_ptr<Config> config_;
        cv::FileStorage file_;

        Config() {}

    public:

        ~Config();

        static void setParameterFile(const std::string& fileName);

        template <typename T>
        static T get(const std::string& key)
        {
            cout << "key :" << key << endl;
            return T(Config::config_->file_[key]);
        }

    };
}


#endif //POINTCLOUD_CONFIG_H
