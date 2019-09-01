
#include <iostream>
#include <vector>
#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>

#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/opencv.hpp>

#include <sophus/se3.h>

#include <pangolin/pangolin.h>
#include <boost/format.hpp>


#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_binary_edge.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/types/sba/types_six_dof_expmap.h>

using namespace std;
using namespace cv;


typedef vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> Vector_SE3;
typedef vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> Vector_3d;

static string pose_file = "./poses.txt";
static string points_file = "./points.txt";

float fx = 277.34;
float fy = 291.402;
float cx = 312.234;
float cy = 239.777;

//inline bool inImage(float u, float v, int w, int h)
//{
//    if ( u >= 0 && u < w && v >= 0 && v < h)
//    {
//        return true;
//    }
//    else
//    {
//        return false;
//    }
//}

inline float GetPixelValue(const cv::Mat &img, float x, float y)
{
    uchar *data = &img.data[int(y) * img.step + int(x)];
    float xx = x - floor(x);
    float yy = y - floor(y);
    return float(
            (1 - xx) * (1 - yy) * data[0] +
            xx * (1 - yy) * data[1] +
            (1 - xx) * yy * data[img.step] +
            xx * yy * data[img.step + 1]
    );
}


// 顶点的李代数做为 位姿 pose
class VertexSophus : public g2o::BaseVertex<6, Sophus::SE3>
{
public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    VertexSophus() {}

    ~VertexSophus() {}

    bool read( std::istream &is) {}

    bool write( std::ostream &os) const{}

    virtual void setToOriginImpl()
    {
        _estimate = Sophus::SE3();
    }

    virtual void oplusImpl(const double *update_)
    {
        Eigen::Map<const Eigen::Matrix<double, 6, 1>> update(update_);
        setEstimate(Sophus::SE3::exp(update) * estimate());
    }
};



// 自定义边
typedef Eigen::Matrix<double, 16, 1> Vector16d;
class EdgeDirectorProjection : public g2o::BaseBinaryEdge<16, Vector16d, g2o::VertexSBAPointXYZ, VertexSophus>
{
public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    EdgeDirectorProjection (float *color, cv::Mat &target)
    {
        this->origColor = color;
        this->targetImg = target;
        this->w = targetImg.cols;
        this->h = targetImg.rows;
    }

    ~EdgeDirectorProjection() override
    {

    }

    void computeError() override
    {
        // ==============================================================================
        // 李代数 相机位姿 v1
//        const g2o::VertexSE3Expmap* v1 = static_cast<const g2o::VertexSE3Expmap*> (_vertices[0]);

        // 顶点 v2
        const g2o::VertexSBAPointXYZ* vertexPw = static_cast<const g2o::VertexSBAPointXYZ*> (vertex(0));

        //
        const VertexSophus* vertexTcw = static_cast<const VertexSophus*>(vertex(1));

        Eigen::Vector3d Pc = vertexTcw->estimate()*vertexPw->estimate();

        float u = Pc[0] / Pc[2] * fx + cx;
        float v = Pc[1] / Pc[2] * fy + cy;

        if( (u-3) < 0 || (u+2) > w || (v-3) < 0 || (v+2) > h)
        {
            _error(0,0) = 0.0;
            this->setLevel(1);
        }
        else
        {
            for(int i=-2;i<2;i++)
            {
                for(int j=-2;j<2;j++)
                {
                    int num=4*i+j+10;//0-15
                    _error[num] = origColor[num] - GetPixelValue(targetImg, h + i, v + j);//??
                }
            }
        }


//
//        const g2o::VertexSBAPointXYZ* vertexPw = static_cast<const g2o::VertexSBAPointXYZ* >(_vertices[1]);
//
//        const VertexSophus* vertexTcw = static_cast<const VertexSophus* >(vertex(1));
//
//        Eigen::Vector3d Pc = vertexTcw->estimate() * vertexPw->estimate();
//
//        float u = Pc[0] / Pc[2] * fx + cx;
//        float v = Pc[1] / Pc[2] * fy + cy;
//
//        if(!inImage(u-3,v-3,w,h) || !inImage(u+2,v+2,w,h))
//        {
//            this->setLevel(1);
//            for(int n=0;n<16;n++)
//                _error[n] = 0;
//        }
//        else
//        {
//            for(int i = -2; i<2; i++)
//            {
//                for(int j = -2; j<2; j++)
//                {
//                    int num = 4 * i + j + 10;
//                    _error[num] = origColor[num] - GetPixelValue(targetImg, u + i, v + j);
//                }
//            }
//        }


        // =============================================================================================
    }

    virtual void linearizeOplus() override
    {
        if(level()==1)
        {
            _jacobianOplusXi = Eigen::Matrix<double,16,3>::Zero();      // Eigen
            _jacobianOplusXj = Eigen::Matrix<double,16,6>::Zero();      // Eigen
            return;
        }
        const g2o::VertexSBAPointXYZ* vertexPw = static_cast<const g2o::VertexSBAPointXYZ* >(vertex(0));
        const VertexSophus* vertexTcw = static_cast<const VertexSophus* >(vertex(1));
        Eigen::Vector3d Pc = vertexTcw->estimate() * vertexPw->estimate();      // Eigen
        float x = Pc[0];
        float y = Pc[1];
        float z = Pc[2];
        float inv_z = 1.0/z;
        float inv_z2 = inv_z * inv_z;
        float u = x * inv_z * fx + cx;
        float v = y * inv_z * fy + cy;

        Eigen::Matrix<double, 2, 3> J_Puv_Pc;                 // Eigen
        J_Puv_Pc(0,0) = fx * inv_z;
        J_Puv_Pc(0,1) = 0;
        J_Puv_Pc(0,2) = -fx * x * inv_z2;
        J_Puv_Pc(1,0) = 0;
        J_Puv_Pc(1,1) = fy * inv_z;
        J_Puv_Pc(1,2) = -fy * y * inv_z2;


        Eigen::Matrix<double,3,6> J_Pc_kesi = Eigen::Matrix<double,3,6>::Zero();           // Eigen
        J_Pc_kesi(0,0) = 1;
        J_Pc_kesi(0,4) = z;
        J_Pc_kesi(0,5) = -y;
        J_Pc_kesi(1,1) = 1;
        J_Pc_kesi(1,3) = -z;
        J_Pc_kesi(1,5) = x;
        J_Pc_kesi(2,2) = 1;
        J_Pc_kesi(2,3) = y;
        J_Pc_kesi(2,4) = -x;

        Eigen::Matrix<double,1,2> J_I_Puv;                          // Eigen
        for(int i = -2; i<2; i++)
            for(int j = -2; j<2; j++) {
                int num = 4 * i + j + 10;
                J_I_Puv(0,0) = (GetPixelValue(targetImg,u+i+1,v+j) - GetPixelValue(targetImg,u+i-1,v+j))/2;
                J_I_Puv(0,1) = (GetPixelValue(targetImg,u+i,v+j+1) - GetPixelValue(targetImg,u+i,v+j-1))/2;
                _jacobianOplusXi.block<1,3>(num,0) = -J_I_Puv * J_Puv_Pc * vertexTcw->estimate().rotation_matrix();
                _jacobianOplusXj.block<1,6>(num,0) = -J_I_Puv * J_Puv_Pc * J_Pc_kesi;
            }
    }
    virtual bool read(std::istream &in)
    {

    }

    virtual bool write(std::ostream &out) const
    {

    }


private:

    cv::Mat targetImg;
    float  *origColor = nullptr;
    int w;
    int h;

};



void Draw(const Vector_SE3 &poses, const Vector_3d &points) {
    if (poses.empty() || points.empty()) {
        cerr << "parameter is empty!" << endl;
        return;
    }
    cout<<"Draw poses: "<<poses.size()<<", points: "<<points.size()<<endl;
    // create pangolin window and plot the trajectory
    pangolin::CreateWindowAndBind("Trajectory Viewer", 1024, 768);
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


    while (pangolin::ShouldQuit() == false) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        d_cam.Activate(s_cam);
        glClearColor(0.0f, 0.0f, 0.0f, 0.0f);

        // draw poses
        float sz = 0.1;
        int width = 640, height = 480;
        for (auto &Tcw: poses) {
            glPushMatrix();
            Sophus::Matrix4f m = Tcw.inverse().matrix().cast<float>();
            glMultMatrixf((GLfloat *) m.data());
            glColor3f(1, 0, 0);
            glLineWidth(2);
            glBegin(GL_LINES);
            glVertex3f(0, 0, 0);
            glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);
            glVertex3f(0, 0, 0);
            glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
            glVertex3f(0, 0, 0);
            glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
            glVertex3f(0, 0, 0);
            glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);
            glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);
            glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
            glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
            glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
            glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
            glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);
            glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);
            glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);
            glEnd();
            glPopMatrix();
        }

        // points
        glPointSize(2);
        glBegin(GL_POINTS);
        for (size_t i = 0; i < points.size(); i++)
        {
            glColor3f(0.0, points[i][2]/4, 1.0-points[i][2]/4);
            glVertex3d(points[i][0], points[i][1], points[i][2]);
        }
        glEnd();

        pangolin::FinishFrame();
        usleep(5000);   // sleep 5 ms
    }
}


int main(int argc, char** argv)
{
    // 读取位姿 和 points

    Vector_SE3 poses;
    Vector_3d points;

    // 读取位姿点
    ifstream poses_data(pose_file);
    if (!poses_data)
    {
        cerr << " Poses file not find............." << endl;
        return -1;
    }

    while(!poses_data.eof())
    {
        double data[7] = {0};
        for (auto & d : data )
        {
            poses_data >> d;
        }

        poses.push_back( Sophus::SE3(Eigen::Quaterniond(data[6], data[3], data[4], data[5]),
                                     Eigen::Vector3d(data[0], data[1], data[2])
                ));

        if( !poses_data.good() )
        {
            break;
        }
    }
    poses_data.close();

    // 读取points
    vector<float *> color;
    ifstream points_xyz(points_file);
    if(!points_xyz)
    {
        cerr << "Points file not find .............." << endl;
        return -1;
    }

    while (!points_xyz.eof()) {
        double xyz[3] = {0};
        for (auto &d : xyz) {
            points_xyz >> d;
        }

        points.push_back(Eigen::Vector3d(xyz[0], xyz[1], xyz[2]));

        float  *c = new float[16];
        for(int i = 0; i < 16; i++)
        {
            points_xyz >> c[i];
        }

        color.push_back(c);

        if ( !points_xyz.good() )
        {
            break;
        }
    }
    points_xyz.close();

    cout << "poses: " << poses.size() << endl;
    cout << "points:" << points.size() << endl;

    // 读取图片
    vector<Mat> images;
    boost::format fmt("./%d.png");
    for(int i = 0; i < 3; i++)
    {
        images.push_back(cv::imread((fmt % i).str(), 0));
    }

    cout << "Images: " << images.size() << endl;


    // 创建g2o 优化器
    typedef g2o::BlockSolver< g2o::BlockSolverTraits<6, 3>> Block;      // pose 维度6  ，landmark 维度 3

    // 1、创建线性求解器
    Block::LinearSolverType* linearSolver = new g2o::LinearSolverDense<Block::PoseMatrixType>();

    // 2、创建 blocksolver.
    Block* solver_ptr = new Block ( std::unique_ptr<Block::LinearSolverType>(linearSolver));

    // 3、创建总的求解器
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(std::unique_ptr<Block>(solver_ptr));

    // 4、创建稀疏优化器
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm( solver );

    // 5、添加顶点和边
    // ============================================================================================
    //初始化pose,即添加第一个点
//    g2o::VertexSE3Expmap* pose = new g2o::VertexSE3Expmap();
//    pose->setId(0);
//    pose->setEstimate(g2o::SE3Quat());
//    optimizer.addVertex(pose);


//    //添加点
//    int index = 1;
//    for(const Point3f p : points)
//    {
//        g2o::VertexSBAPointXYZ* point=new g2o::VertexSBAPointXYZ();
//        point->setId(index++);
//        point->setEstimate(Eigen::Vector3d(p.x, p.y, p.z));
//        point->setMarginalized(true);
//        optimizer.addVertex(point);
//    }
//
//    //添加边
//    index = 1;
//    for(int i = 0; i< int(poses.size() / 7); i++ )
//    {
//        EdgeSE3* edge = new EdgeSE3();
//        //edge->vertices()[0] = optimizer.vertex( 0 );
//        //edge->vertices()[1] = optimizer.vertex(index);
//        edge->setId(index);
//        edge->setVertex(0, dynamic_cast<g2o::VertexSBAPointXYZ*> (optimizer.vertex(index)));
//        edge->setVertex(1, pose);
//
//        edge->setMeasurement( poses[i].inverse() );
//        edge->setParameterId(0,0);
//        edge->setInformation( Eigen::Matrix3d::Identity() );
//
//        optimizer.addEdge( edge );
//        index++;
//    }

    //添加点
    for (int i = 0; i < points.size(); i++)
    {
        g2o::VertexSBAPointXYZ* vertexPw = new g2o::VertexSBAPointXYZ();
        vertexPw->setEstimate(points[i]);
        vertexPw->setId(i);
        vertexPw->setMarginalized(true);
        optimizer.addVertex(vertexPw);
    }

    //添加边
    for (int j = 0; j < poses.size(); j++)
    {
        auto *vertexTcw = new VertexSophus();
        vertexTcw->setEstimate(poses[j]);
        vertexTcw->setId(j + points.size());
        optimizer.addVertex(vertexTcw);
    }

    for (int c = 0;c < poses.size(); c++)
    {
        for (int p = 0; p < points.size(); p++) {
            EdgeDirectorProjection *edge = new EdgeDirectorProjection(color[p], images[c]);
            edge->setVertex(0, dynamic_cast<g2o::VertexSBAPointXYZ *>(optimizer.vertex(p)));
            edge->setVertex(1, dynamic_cast<VertexSophus *>(optimizer.vertex(c + points.size())));
            edge->setInformation(Eigen::Matrix<double, 16, 16>::Identity());
            g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber();
            rk->setDelta(1.0);
            edge->setRobustKernel(rk);
            optimizer.addEdge(edge);
        }
    }


    // =========================================================================================================


    // 6、优化执行
    optimizer.initializeOptimization(0);
    optimizer.optimize(200);

    // 7、 从optimizer 获取结果
    for( int m = 0; m < poses.size(); m++)
    {
        for (int n = 0; n < points.size(); n++)
        {
            Eigen::Vector3d Pw = dynamic_cast<g2o::VertexSBAPointXYZ* > (optimizer.vertex(n))->estimate();
            points[n] = Pw;

            Sophus::SE3 Tcw = dynamic_cast<VertexSophus* > (optimizer.vertex( m + points.size()))->estimate();
            poses[m] = Tcw;
        }
    }

    Draw(poses, points);

    for (auto & c : color)
    {
        delete [] c;
        return 0;
    }



    cout << "hello g2o................" << endl;
    return 0;
}








































































































































































































