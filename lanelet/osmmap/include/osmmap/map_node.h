/*
 * @Author: your name
 * @Date: 2022-03-03 21:29:43
 * @LastEditTime: 2022-10-03 15:34:00
 * @LastEditors: blueclocker 1456055290@hnu.edu.cn
 * @Description: 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 * @FilePath: /wpollo/src/lanelet/osmmap/include/osmmap/map_node.h
 */
#ifndef MAP_NODE_H_
#define MAP_NODE_H_

#include "map_base.h"
//#include "GeographicLib/LocalCartesian.hpp"

/*node类型元素
* 把读入数据存储在Point3D中
*
*/

namespace map
{
namespace node
{
struct Point3D
{
    int ID_;
    double longitude_;
    double latitude_;
    double local_x_;
    double local_y_;
    double elevation_;
    Point3D() {}
    Point3D(const double lat, const double lon, const double ele)
    {
        ID_ = -1;
        longitude_ = lon;
        latitude_ = lat;
        elevation_ = ele;
        local_x_ = 0;
        local_y_ = 0;
    }
    Point3D(const double x, const double y)
    {
        ID_ = -1;
        local_x_ = x;
        local_y_ = y;
        elevation_ = 0;
        longitude_ = 0;
        latitude_ = 0;
    }
    bool operator==(const Point3D &a) const
    {
        return (this->ID_ == a.ID_);
    }
    Point3D& operator=(const Point3D &a)
    {
        //std::cout << "operator point3d =" << std::endl;
        this->ID_ = a.ID_;
        this->local_x_ = a.local_x_;
        this->local_y_ = a.local_y_;
        this->elevation_ = a.elevation_;
        return *this;
    }
    Point3D& operator*(const double &a)
    {
        //std::cout << "operator point3d *" << std::endl;
        this->local_x_ *= a;
        this->local_y_ *= a;
        this->elevation_ *= a;
        this->ID_ = -1;
        return *this;
    }
    Point3D& operator/(const double &a)
    {
        //std::cout << "operator point3d /" << std::endl;
        this->local_x_ /= a;
        this->local_y_ /= a;
        this->elevation_ /= a;
        this->ID_ = -1;
        return *this;
    }
    Point3D& operator+(const Point3D &a)
    {
        //std::cout << "operator point3d +" << std::endl;
        this->local_x_ += a.local_x_;
        this->local_y_ += a.local_y_;
        this->elevation_ += a.elevation_;
        this->ID_ = -1;
        return *this;
    }
    Point3D& operator-(const Point3D &a)
    {
        //std::cout << "operator point3d -" << std::endl;
        this->local_x_ -= a.local_x_;
        this->local_y_ -= a.local_y_;
        this->elevation_ -= a.elevation_;
        this->ID_ = -1;
        return *this;
    }
};

class Node: public map_base<Point3D>
{
private:
    //Point3D *Data;
    //std::unordered_map<int, Point3D*> Data;
    //int numbers;
    //TiXmlElement *node_root;
    static constexpr double RMajor_ = 6378137.0;
    static constexpr double RMinor_ = 6356752.3142;
    static constexpr double Eccent_ = 0.081819190928906924;  //=std::sqrt(1.0 - RMinor*RMinor/RMajor/RMajor)
public:
    Node();
    Node(TiXmlElement *root);
    virtual void CreateOneObject(TiXmlElement *head);
    //void CreateObjects(TiXmlElement *tail);
    //int Size() const;
    //Point3D* Find(int id_);
    //取消MercatorGPS2xy投影
    //void MercatorGPS2xy(Point3D *pin);
    //void Mercatorxy2GPS(Point3D *pin);
    virtual ~Node();
};


};//namespace node
};//namespace map

#endif