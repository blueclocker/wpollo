/*
 * @Author: your name
 * @Date: 2022-03-03 21:29:43
 * @LastEditTime: 2022-03-19 16:28:03
 * @LastEditors: Please set LastEditors
 * @Description: 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 * @FilePath: /wpollo/src/lanelet/osmmap/include/osmmap/map_node.h
 */
#ifndef MAP_NODE_H_
#define MAP_NODE_H_

#include "map_base.h"

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
    int ID;
    double longitude;
    double latitude;
    double local_x;
    double local_y;
    double elevation;
    Point3D() {}
    Point3D(double lat_, double lon_, double ele_)
    {
        ID = -1;
        longitude = lon_;
        latitude = lat_;
        elevation = ele_;
        local_x = 0;
        local_y = 0;
    }
    Point3D(double x_, double y_)
    {
        ID = -1;
        local_x = x_;
        local_y = y_;
        elevation = 0;
        longitude = 0;
        latitude = 0;
    }
    bool operator==(const Point3D &a)
    {
        return (this->ID == a.ID);
    }
    Point3D& operator=(const Point3D &a)
    {
        //std::cout << "operator point3d =" << std::endl;
        this->ID = a.ID;
        this->local_x = a.local_x;
        this->local_y = a.local_y;
        this->elevation = a.elevation;
        return *this;
    }
    Point3D& operator*(const double &a)
    {
        //std::cout << "operator point3d *" << std::endl;
        this->local_x *= a;
        this->local_y *= a;
        this->elevation *= a;
        this->ID = -1;
        return *this;
    }
    Point3D& operator/(const double &a)
    {
        //std::cout << "operator point3d /" << std::endl;
        this->local_x /= a;
        this->local_y /= a;
        this->elevation /= a;
        this->ID = -1;
        return *this;
    }
    Point3D& operator+(const Point3D &a)
    {
        //std::cout << "operator point3d +" << std::endl;
        this->local_x += a.local_x;
        this->local_y += a.local_y;
        this->elevation += a.elevation;
        this->ID = -1;
        return *this;
    }
    Point3D& operator-(const Point3D &a)
    {
        //std::cout << "operator point3d -" << std::endl;
        this->local_x -= a.local_x;
        this->local_y -= a.local_y;
        this->elevation -= a.elevation;
        this->ID = -1;
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
    static constexpr double RMajor = 6378137.0;
    static constexpr double RMinor = 6356752.3142;
    static constexpr double Eccent = 0.081819190928906924;  //=std::sqrt(1.0 - RMinor*RMinor/RMajor/RMajor)
public:
    Node();
    Node(TiXmlElement *root);
    virtual void CreateOneObject(TiXmlElement *head);
    //void CreateObjects(TiXmlElement *tail);
    //int Size() const;
    //Point3D* Find(int id_);
    void MercatorGPS2xy(Point3D *pin);
    void Mercatorxy2GPS(Point3D *pin);
    virtual ~Node();
};


};//namespace node
};//namespace map

#endif