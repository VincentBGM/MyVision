

#include "myslam/visual_odometry.h"


namespace myslam
{
    VisualOdometry::VisualOdometry() : state_( INITIALIZING ), ref_(nullptr ), cur_(nullptr), map_( new Map ), num_lost_(0), num_inliers_(0), matcher_flann_ ( new cv::flann::LshIndexParams ( 5,10,2 ) )
    {

//        num_of_features_ = Config::get<int> ( "number_of_features" );
//        scale_factor_ = Config::get<double> ( "scale_factor" );
//        level_pyramid_ = Config::get<int> ( "level_pyramid" );
//        match_ration_ = Config::get<float> ( "match_ratio" );
//        max_number_lost = Config::get<float> ( "max_num_lost" );
//        min_inliers_ = Config::get<int> ( "min_inliers" );
//        key_frame_min_rot = Config::get<double> ( "keyframe_rotation" );
//        key_frame_min_trans = Config::get<double> ( "keyframe_translation" );
//        map_point_erase_ratio_ = Config::get<double> ( "map_point_erase_ratio" );

        num_of_features_ = 500;
        scale_factor_ = 1.2;
        level_pyramid_ = 4;
        match_ration_ = 2.0;
        max_number_lost = 10;
        min_inliers_ = 30;
        key_frame_min_rot = 0.1;
        key_frame_min_trans = 0.1;

        map_point_erase_ratio_ = 0.1;

        orb_ = cv::ORB::create ( num_of_features_, scale_factor_, level_pyramid_ );
    }

    VisualOdometry::~VisualOdometry() = default;


    bool VisualOdometry::addFrame( const Frame::Ptr frame)
    {
        boost::timer timer;
        switch ( state_ )
        {
            case INITIALIZING :
            {
                // 初始化， 如果为初始话，以当前枕为参考帧
                state_ = OK;
                cur_ = ref_ = frame;
                cout << "map stat : " << map_ << endl;
//                map_->insertKeyFrame( frame );

                extractKeyPoints();
                computeDescriptors();
                addKeyFrame();

                cout << " addFram initializing cost time = " << timer.elapsed() << endl;
                break;
            }

            case OK :
            {
                cur_ = frame;
                cur_->T_w_c_ = ref_->T_w_c_;

                extractKeyPoints();
                computeDescriptors();
                featureMatching();
//                poseEstimationPnP();
                poseEstimationPnP_BA();    // 使用BA 优化


                if (checkEstimatedPose())
                {
                    cur_->T_w_c_ = T_c_r_estimated_ * ref_->T_w_c_;         // Tcw = Tcr * Trw

                    optimizeMap();

                    num_lost_ = 0;

                    if (checkKeyFrame())
                    {
                        addKeyFrame();
                    }
                }
                else
                {
                    num_lost_++;
                    if ( num_lost_ > max_number_lost )
                    {
                        state_ = LOST;
                    }
                    return false;
                }

                cout << " addFram OK cost time = " << timer.elapsed() << endl;
                break;
            }

            case LOST :
            {
                cout << " VO is lost ..............." << endl;
                cout << " addFram LOST cost time = " << timer.elapsed() << endl;
                break;
            }

        }

        return true;
    }

    void VisualOdometry::extractKeyPoints()
    {
        boost::timer timer;
        orb_->detect(cur_->color_, keypoints_curr_);

        cout << "extract KeyPoints cost time: " << timer.elapsed() << endl;
    }

    void VisualOdometry::computeDescriptors()
    {
        boost::timer timer;
        orb_->compute(cur_->color_, keypoints_curr_, descriptor_cur_);

        cout << "Descriptor Compute cost time: " << timer.elapsed() << endl;
    }

    void VisualOdometry::featureMatching() {
        // 1、没有局部地图更新 方法
//        boost::timer timer;
//        vector<cv::DMatch> matchers;
//        cv::BFMatcher matcher ( cv::NORM_HAMMING );
//        matcher.match(descriptor_ref_, descriptor_cur_, matchers);

        // 2、使用局部地图更新时
        boost::timer timer;
        vector<cv::DMatch> matchers;

        cv::Mat desp_map;
        vector<MapPoint::Ptr> candidata;
        for (auto& allpoints : map_->map_points_ )
        {
            MapPoint::Ptr& p = allpoints.second;

            // check if p in curr frame imgae
            if( cur_->isInFrame( p->pos_) )
            {
                p->visible_times_++;
                candidata.emplace_back( p );
                desp_map.push_back( p->descriptor_ );
            }
        }
        matcher_flann_.match( desp_map, descriptor_cur_, matchers);


        float min_distance = std::min_element( matchers.begin(), matchers.end(), [](const cv::DMatch& m1, const cv::DMatch& m2){

            return m1.distance < m2.distance;
        })->distance;

        // 1、没有局部地图更新 方法
//        feature_matchers_.clear();


        // 2、使用局部地图更新时
        match_3dpts_.clear();
        match_2dkp_index_.clear();


        for (auto i = 0; i < matchers.size(); i++)
        {
            if ( matchers[i].distance < max<float > (min_distance * match_ration_, 30.0) )
            {
                // 1、没有局部地图更新 方法
//                feature_matchers_.emplace_back(matchers[i]);

                // 2、使用局部地图更新时
                match_3dpts_.push_back( candidata[matchers[i].queryIdx] );
                match_2dkp_index_.push_back( matchers[i].trainIdx );
            }
        }

        cout << "good matchers  match_3dpts_  size = "  << match_3dpts_.size() << endl;

        cout << "Match cost time: " << timer.elapsed() << endl;
    }

    void VisualOdometry::poseEstimationPnP()
    {
        boost::timer timer;
        vector<cv::Point3f> points_3D;
        vector<cv::Point2f> points_2D;

//        for (auto i = 0; i < feature_matchers_.size(); i++)
//        {
//            points_3D.emplace_back(points_3d_ref_[feature_matchers_[i].queryIdx]);
//            points_2D.emplace_back(keypoints_curr_[feature_matchers_[i].trainIdx].pt);
//        }


        for ( int index: match_2dkp_index_)
        {
            points_2D.emplace_back( keypoints_curr_[index].pt );
        }

        for ( MapPoint::Ptr pt: match_3dpts_ )
        {
            points_3D.emplace_back( pt->getPositionCV() );
        }



        cv::Mat K = ( cv::Mat_ <double >(3, 3) <<
                                                    ref_->camera_->fx_, 0, ref_->camera_->cx_,
                                                    0, ref_->camera_->fy_, ref_->camera_->cy_,
                                                    0, 0, 1
                );

        cv::Mat rvec, tvec, inliers;

        cv::solvePnPRansac(points_3D, points_2D, K, cv::Mat(), rvec, tvec, false, 100, 8.0, 0.99, inliers);

        num_inliers_ = inliers.rows;

        cout << "PnP inliers = " << num_inliers_ << endl;

        // 旋转矩阵， 平移向量 转换成SE3形式的
        T_c_r_estimated_ = Sophus::SE3(
                Sophus::SO3(rvec.at<double>(0, 0), rvec.at<double>(1, 0), rvec.at<double>(2, 0)),
                Eigen::Vector3d( tvec.at<double>(0, 0), tvec.at<double>(1, 0), tvec.at<double>(2, 0))
                );

        cout << " Pose PnP cost time : " << timer.elapsed() << endl;
    }

    // g2o 非现线性优化
    void VisualOdometry::poseEstimationPnP_BA()
    {
        boost::timer timer;
        vector<cv::Point3f> points_3D;
        vector<cv::Point2f> points_2D;

//        for (auto i = 0; i < feature_matchers_.size(); i++)
//        {
//            points_3D.emplace_back(points_3d_ref_[feature_matchers_[i].queryIdx]);
//            points_2D.emplace_back(keypoints_curr_[feature_matchers_[i].trainIdx].pt);
//        }


        for ( int index: match_2dkp_index_)
        {
            points_2D.emplace_back( keypoints_curr_[index].pt );
        }

        for ( MapPoint::Ptr pt: match_3dpts_ )
        {
            points_3D.emplace_back( pt->getPositionCV() );
        }



        cv::Mat K = ( cv::Mat_ <double >(3, 3) <<
                                               ref_->camera_->fx_, 0, ref_->camera_->cx_,
                0, ref_->camera_->fy_, ref_->camera_->cy_,
                0, 0, 1
        );

        cv::Mat rvec, tvec, inliers;

        cv::solvePnPRansac(points_3D, points_2D, K, cv::Mat(), rvec, tvec, false, 100, 8.0, 0.99, inliers);

        num_inliers_ = inliers.rows;

        cout << "PnP inliers = " << num_inliers_ << endl;

        // 旋转矩阵， 平移向量 转换成SE3形式的
        T_c_r_estimated_ = Sophus::SE3(
                Sophus::SO3(rvec.at<double>(0, 0), rvec.at<double>(1, 0), rvec.at<double>(2, 0)),
                Eigen::Vector3d( tvec.at<double>(0, 0), tvec.at<double>(1, 0), tvec.at<double>(2, 0))
        );


        // BA optimize the pose

        typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 2> > Block;

        Block::LinearSolverType* linearSolver = new g2o::LinearSolverDense<Block::PoseMatrixType>();
//        std::unique_ptr<Block::LinearSolverType> linearSolver ( new g2o::LinearSolverDense<Block::PoseMatrixType>());

//        Block* solver_ptr = new Block ( std::unique_ptr<Block::LinearSolverType>( linearSolver) );
//        std::unique_ptr<Block> solver_ptr ( new Block ( std::move(linearSolver)));

        cout << "=================error ===============" << endl;
//        g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg( std::unique_ptr<Block>(solver_ptr ) );
//        g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg ( std::move(solver_ptr));

//        g2o::SparseOptimizer optimizer;
//        optimizer.setAlgorithm( solver );
//        optimizer.setVerbose( true );


//        g2o::VertexSE3Expmap* pose = new g2o::VertexSE3Expmap();
//        pose->setId(0);
//        pose->setEstimate( g2o::SE3Quat(
//                T_c_r_estimated_.rotation_matrix(),
//                T_c_r_estimated_.translation()
//                ));
//        optimizer.addVertex( pose );
//
//        // 添加边
//        for (auto i = 0; i < inliers.rows; i++)
//        {
//            int index = inliers.at<int >(i, 0);
//
//            //
//            EdgeProjectXYZ2UVPoseOnly* edge = new EdgeProjectXYZ2UVPoseOnly();
//            edge->setId(i);
//            edge->setVertex(0, pose);
//            edge->camera_ = cur_->camera_.get();
//            edge->point_ = Eigen::Vector3d( points_3D[index].x, points_3D[index].y, points_3D[index].z );
//            edge->setMeasurement( Eigen::Vector2d(points_2D[index].x, points_2D[index].y));
//            edge->setInformation( Eigen::Matrix2d::Identity());
//
//            optimizer.addEdge( edge );
//
//        }
//
//        optimizer.initializeOptimization();
//        optimizer.optimize( 10 );
//
//        T_c_r_estimated_ = Sophus::SE3 (
//                pose->estimate().rotation(),
//                pose->estimate().translation()
//                );
//
        cout << "PnP and BA cost time: " << timer.elapsed() << endl;

    }



    void VisualOdometry::setRef3DPoints()
    {
        // 把当前帧的3D点 特征点 特征描述子 转换到下一个姿态（观测帧）的 特征点， 特征描述子
        points_3d_ref_.clear();
        descriptor_ref_ = cv::Mat();


        for (auto i = 0; i < keypoints_curr_.size(); i++)
        {
            double d = ref_->findDepth(keypoints_curr_[i]);

            if( d > 0 )
            {
                // 把当前点 转换为 相机坐标系
                Eigen::Vector3d point_cam = ref_->camera_->pixel2Camera(Eigen::Vector2d(keypoints_curr_[i].pt.x, keypoints_curr_[i].pt.y), d);

                points_3d_ref_.emplace_back( cv::Point3f(point_cam(0, 0), point_cam(1, 0), point_cam(2, 0) ) );

                descriptor_ref_.push_back( descriptor_cur_.row(i));
            }
        }

    }

    // ================================================重点
    void VisualOdometry::addKeyFrame()
    {
        boost::timer timer;
        cout << " add a key-frame ............" << endl;

        if(map_->key_frame_.empty())
        {

            cout << "keyPoints_curr_ size : " << keypoints_curr_.size() << endl;
            // 第一个关键帧 和 所有的3D 点 加入到局部地图中
            for (auto i = 0; i < keypoints_curr_.size(); i++)
            {
                double d = cur_->findDepth( keypoints_curr_[i]);

                if( d < 0 )
                {
                    continue;
                }

                Eigen::Vector3d p_world = ref_->camera_->pixel2World(Eigen::Vector2d(keypoints_curr_[i].pt.x, keypoints_curr_[i].pt.y), cur_->T_w_c_, d);

                Eigen::Vector3d n = p_world - ref_->getCameraCenter();

                n.normalize();

                MapPoint::Ptr map_point = MapPoint::createMapPoint(p_world, n, descriptor_cur_.row(i).clone(), cur_.get() );

                map_->insertMapPoint( map_point );
            }
        }

        map_->insertKeyFrame( cur_);
        ref_ = cur_;

        cout << " addKeyFrame cost time = " << timer.elapsed() << endl;
    }

    void VisualOdometry::addMapPoints()
    {
        boost::timer timer;
        cout << " addMapPoints function used ...." << endl;
        // 添加新的地图点
        vector<bool> matched(keypoints_curr_.size(), false);
        for(int index : match_2dkp_index_ )
        {
            matched[index] = true;
        }

        for (auto i = 0; i < keypoints_curr_.size(); i++)
        {
            if ( matched[i] == true )
            {
                continue;
            }

            double d = ref_->findDepth( keypoints_curr_[i]);
            if ( d < 0 )
            {
                continue;
            }

            Eigen::Vector3d p_world = ref_->camera_->pixel2World( Eigen::Vector2d(keypoints_curr_[i].pt.x, keypoints_curr_[i].pt.y), cur_->T_w_c_, d);

            Eigen::Vector3d n = p_world - ref_->getCameraCenter();

            n.normalize();

            MapPoint::Ptr map_point = MapPoint::createMapPoint(p_world, n, descriptor_cur_.row(i).clone(), cur_.get());

            map_->insertMapPoint( map_point );
        }

        cout << " addMapPoints cost time : " << timer.elapsed() << endl;
    }

    void VisualOdometry::optimizeMap()
    {
        boost::timer timer;
        cout << "Optimizer function used ..." << endl;
        // 移除 不在视野 删除不在视野里的点
        for (auto it = map_->map_points_.begin(); it != map_->map_points_.end();)
        {
            if (!cur_->isInFrame(it->second->pos_))
            {
                it = map_->map_points_.erase( it );
                continue;
            }

            float match_ration = float ( it->second->matched_times_) / it->second->visible_times_;

//            cout << "optimizeMap match_ration = " << match_ration << endl;
            if ( match_ration < map_point_erase_ratio_)
            {
                it = map_->map_points_.erase( it );
                continue;
            }

            double angle = getViewAngle(cur_, it->second );
            if( angle > M_PI / 6 )
            {
                it = map_->map_points_.erase( it );
                continue;
            }

            if (!it->second->good_)
            {
                // triangulate this map point 三角化更新特征点的世界坐标系===============================

                cout << "good ========================" << endl;

            }

            it++;
        }

        if ( match_2dkp_index_.size() < 100 )
        {
            addMapPoints();
        }

        if ( map_->map_points_.size() > 1000)
        {
            // 地图太大 删除一个
            map_point_erase_ratio_ += 0.05;
        }
        else
        {
            map_point_erase_ratio_ = 0.1;
        }

        cout << "map points size: " << map_->map_points_.size() << endl;
        cout << "Opimizer Map cost time : " << timer.elapsed() << endl;
    }

    double VisualOdometry::getViewAngle(Frame::Ptr frame, MapPoint::Ptr point)
    {
        boost::timer timer;
        Eigen::Vector3d n = point->pos_ - frame->getCameraCenter();

        n.normalize();

//        cout << "getViewAngle cost time = " << timer.elapsed() << endl;
        return acos( n.transpose() * point->norm_ );
    }

    bool VisualOdometry::checkEstimatedPose()
    {
        boost::timer timer;
        // 位姿检测
        if ( num_inliers_ < min_inliers_ )
        {
            cout << " reject because inliers is too small : " << num_inliers_ << endl;
            return false;
        }


        // 运动过大，会出错 ？？？？？？？/
        Sophus::SE3 T_r_c = ref_->T_w_c_ * T_c_r_estimated_.inverse();

        Sophus::Vector6d d = T_c_r_estimated_.log();
        if ( d.norm() > 5.0 )
        {
            cout << " reject because motion is too large: " << d.norm() << endl;
            return false;
        }

        cout << " checkEstimatedPose cost time = " << timer.elapsed() << endl;
        return true;

    }

    bool VisualOdometry::checkKeyFrame()
    {
        boost::timer timer;
        Sophus::SE3 T_r_c = ref_->T_w_c_ * T_c_r_estimated_.inverse();

        Sophus::Vector6d d = T_c_r_estimated_.log();

        Eigen::Vector3d trans = d.head<3>();
        Eigen::Vector3d rot = d.tail<3>();

        if ( rot.norm() >key_frame_min_rot || trans.norm() >key_frame_min_trans)
        {
            return true;
        }

        cout << " checkKeyFrame cost time = " << timer.elapsed() << endl;
        return false;
    }

}

