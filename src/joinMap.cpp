/*************************************************************************
	> File Name: src/joinMap.cpp
	> Author: Gao Xiang
	> Mail: gaoxiang12@mails.tsinghua.edu.cn
	> Created Time: 2015年12月13日 星期日 14时37分05秒
 ************************************************************************/

#include <iostream>
#include <vector>

// octomap 
#include <octomap/octomap.h>
#include <octomap/ColorOcTree.h>
#include <octomap/math/Pose6D.h>

// opencv 用于图像数据读取与处理
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// 使用Eigen的Geometry模块处理3d运动
#include <Eigen/Core>
#include <Eigen/Geometry> 

// pcl
#include <pcl/common/transforms.h>
#include <pcl/point_types.h>

// boost.format 字符串处理
#include <boost/format.hpp>

using namespace std;

// 全局变量：相机矩阵
// 更好的写法是存到参数文件中，但为方便起见我就直接这样做了
float camera_scale  = 1000;
float camera_cx     = 325.5;
float camera_cy     = 253.5;
float camera_fx     = 518.0;
float camera_fy     = 519.0;

int main( int argc, char** argv )
{
    // 读关键帧编号
    ifstream fin( "./data/keyframe.txt" );
    vector<int> keyframes;
    vector< Eigen::Isometry3d > poses;
    // 把文件 ./data/keyframe.txt 里的数据读取到vector中
    while( fin.peek() != EOF )
    {
        int index_keyframe;
        fin>>index_keyframe;
        if (fin.fail()) break;
        keyframes.push_back( index_keyframe );
    }
    fin.close();

    cout<<"load total "<<keyframes.size()<<" keyframes. "<<endl;

    // 读关键帧姿态
    // 我的代码中使用了Eigen来存储姿态，类似的，也可以用octomath::Pose6D来做这件事
    fin.open( "./data/trajectory.txt" );
    while( fin.peek() != EOF )
    {
        int index_keyframe;
        float data[7]; // 三个位置加一个姿态四元数x,y,z, w,ux,uy,uz
        fin>>index_keyframe;
        for ( int i=0; i<7; i++ )
        {
            fin>>data[i];
            cout<<data[i]<<" ";
        }
        cout<<endl;
        if (fin.fail()) break;
        // 注意这里的顺序。g2o文件四元数按 qx, qy, qz, qw来存
        // 但Eigen初始化按照qw, qx, qy, qz来做
        Eigen::Quaterniond q( data[6], data[3], data[4], data[5] );
        Eigen::Isometry3d t(q);
        t(0,3) = data[0]; t(1,3) = data[1]; t(2,3) = data[2];
        poses.push_back( t );
    }
    fin.close();

    // 拼合全局地图
    octomap::ColorOcTree tree( 0.05 ); //全局map

    // 注意我们的做法是先把图像转换至pcl的点云，进行姿态变换，最后存储成octomap
    // 因为octomap的颜色信息不是特别方便处理，所以采用了这种迂回的方式
    // 所以，如果不考虑颜色，那不必转成pcl点云，而可以直接使用octomap::Pointcloud结构
    
    for ( size_t i=0; i<keyframes.size(); i++ )
    {
        pcl::PointCloud<pcl::PointXYZRGBA> cloud; 
        cout<<"converting "<<i<<"th keyframe ..." <<endl;
        int k = keyframes[i];
        Eigen::Isometry3d& pose = poses[i];

        // 生成第k帧的点云，拼接至全局octomap上
        boost::format fmt ("./data/rgb_index/%d.ppm" );
        cv::Mat rgb = cv::imread( (fmt % k).str().c_str() );
        fmt = boost::format("./data/dep_index/%d.pgm" );
        cv::Mat depth = cv::imread( (fmt % k).str().c_str(), -1 );

        // 从rgb, depth生成点云，运算方法见《一起做》第二讲
        // 第一次遍历用于生成空间点云
        for ( int m=0; m<depth.rows; m++ )
            for ( int n=0; n<depth.cols; n++ )
            {
                ushort d = depth.ptr<ushort> (m) [n];
                if (d == 0)
                    continue;
                float z = float(d) / camera_scale;
                float x = (n - camera_cx) * z / camera_fx;
                float y = (m - camera_cy) * z / camera_fy;
                pcl::PointXYZRGBA p;
                p.x = x; p.y = y; p.z = z;

                uchar* rgbdata = &rgb.ptr<uchar>(m)[n*3];
                uchar b = rgbdata[0];
                uchar g = rgbdata[1];
                uchar r = rgbdata[2];

                p.r = r; p.g = g; p.b = b;
                cloud.points.push_back( p ); 
            }
        // 将cloud旋转之后插入全局地图
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr temp( new pcl::PointCloud<pcl::PointXYZRGBA>() );
        pcl::transformPointCloud( cloud, *temp, pose.matrix() );

        octomap::Pointcloud cloud_octo;
        for (auto p:temp->points)
            cloud_octo.push_back( p.x, p.y, p.z );
        
        tree.insertPointCloud( cloud_octo, 
                octomap::point3d( pose(0,3), pose(1,3), pose(2,3) ) );

        for (auto p:temp->points)
            tree.integrateNodeColor( p.x, p.y, p.z, p.r, p.g, p.b );
    }
    
    tree.updateInnerOccupancy();
    tree.write( "./data/map.ot" );

    cout<<"done."<<endl;
    
    return 0;

}
