

#include "myslam/config.h"


namespace myslam
{

    shared_ptr<Config> Config::config_ = nullptr;

    void Config::setParameterFile(const std::string& fileName)
    {
        if ( config_ == nullptr )
        {
            config_ = shared_ptr<Config> (new Config );
            cout << "单例。。。。。。。。。。" << endl;
        }


        config_->file_ = cv::FileStorage( fileName.c_str(), cv::FileStorage::READ );


        if (!config_->file_.isOpened())
        {
            cout <<  " parameter file " << fileName << " not exise ..." << endl;

            config_->file_.release();
            return;
        }

        cout << "fileName: " << fileName << endl;
    }

    Config::~Config()
    {
        if( file_.isOpened() )
        {
            cout << "文件关闭。。。。。" << endl;
            file_.release();
        }
    }


}

