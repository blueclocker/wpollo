#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>  //RANSAC相关头文件
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/project_inliers.h>  // 投影滤波 
#include <pcl/surface/concave_hull.h>

int
main(int argc, char** argv)
{
    //--------------------------加载点云数据----------------------------
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
          
    pcl::PCDReader reader;
    reader.read("/home/lli/catkin_ws/src/chapter10_tutorials/data/table_scene_lms4001.pcd", *cloud);
    std::cerr << "原始点云点的个数: " << cloud->points.size() << std::endl;
    //-------------------------RANSAC拟合平面---------------------------
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZ> seg; 
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.01);
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);
    //-----------------------点云投影到平面----------------------------
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ProjectInliers<pcl::PointXYZ> proj;
    proj.setModelType(pcl::SACMODEL_PLANE);
    proj.setInputCloud(cloud);
    proj.setModelCoefficients(coefficients);
    proj.filter(*cloud_projected);
    std::cerr << "投影后点的个数: "<< cloud_projected->points.size() << std::endl;
    pcl::PCDWriter writer;
    writer.write("投影点云.pcd", *cloud_projected, true);
    //---------------提取投影平面点云的凸多边形边界-------------------
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_vex_hull(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ConvexHull<pcl::PointXYZ> hull;  //创建凸包对象
    hull.setInputCloud(cloud_projected);            //设置输入点云
    hull.setDimension(2);                 //设置输入数据的维度(2D)
    hull.reconstruct(*cloud_vex_hull);//计算2D凸包结果

    std::cerr << "凸多边形的点数: " << cloud_vex_hull->points.size() << std::endl;

    writer.write("凸多边形.pcd", *cloud_vex_hull, true);


    return (0);
}
