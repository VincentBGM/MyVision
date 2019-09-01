

#ifndef POINTCLOUD_G2O_TYPES_H
#define POINTCLOUD_G2O_TYPES_H


#include "myslam/common_include.h"
#include "myslam/camera.h"


namespace myslam {


    // 优化位姿
//    class EdgeProjectXYZRGBDPoseOnly : public g2o::BaseUnaryEdge<3, Eigen::Vector3d, g2o::VertexSE3Expmap >
//    {
//
//    public:
//
//        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
//
//        // 误差 = R × point + t
//
//        virtual void computeError();
//
//        virtual void linearizeOplus();
//
//
//        virtual bool read( std::istream& in ) {}
//
//        virtual bool write( std::ostream& out ) const {}
//
//        Eigen::Vector3d point_;
//
//    };



    class EdgeProjectXYZ2UVPoseOnly : public g2o::BaseUnaryEdge<2, Eigen::Vector2d, g2o::VertexSE3Expmap >
    {
    public:

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        // 误差 = R × point + t

        void computeError() override;

        virtual void linearizeOplus();



        virtual bool read( std::istream& in ) {}

        virtual bool write( std::ostream& os ) const {}


        Eigen::Vector3d point_;
        Camera* camera_;

    };


}
#endif //POINTCLOUD_G2O_TYPES_H
