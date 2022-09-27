/*
 * @Author: your name
 * @Date: 2022-03-06 15:43:53
 * @LastEditTime: 2022-09-25 15:18:39
 * @LastEditors: blueclocker 1456055290@hnu.edu.cn
 * @Description: 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 * @FilePath: /wpollo/src/lanelet/osmmap/include/osmmap/centerway.h
 */
#ifndef CENTER_WAY_H_
#define CENTER_WAY_H_

#include "map_node.h"
#include "map_way.h"
#include "map_relation.h"
#include <numeric>  // std::partial_sum
#include <utility>      // std::pair
#include <cfloat>

/*根据node, way, relation计算得到的道路中心线
* 计算得到的道路中心点存储在CenterPoint3D中，并全部存储在类CenterWay的centerpointmap中
* 计算得到的道路中心点连线存储在CenterWay3D中
*/

namespace map
{
namespace centerway
{
struct CenterPoint3D
{
    int ID;
    double x;
    double y;
    double ele;
    CenterPoint3D() {}
    CenterPoint3D(const double xx, const double yy)
    {
        ID = -1;
        x = xx;
        y = yy;
        ele = 0;
    }
    CenterPoint3D(const node::Point3D &a)
    {
        ID = a.ID;
        x = a.local_x;
        y = a.local_y;
        ele = a.elevation;
    }
    bool operator==(const CenterPoint3D &a) const
    {
        return (this->ID == a.ID);
    }
    CenterPoint3D& operator=(const CenterPoint3D &a)
    {
        //std::cout << "operator centerpoint =" << std::endl;
        this->ID = a.ID;
        this->x = a.x;
        this->y = a.y;
        this->ele = a.ele;
        return *this;
    }
    CenterPoint3D& operator*(const double a)
    {
        //std::cout << "operator centerpoint *" << std::endl;
        this->x *= a;
        this->y *= a;
        this->ele *= a;
        this->ID = -1;
        return *this;
    }
    CenterPoint3D& operator/(const double a)
    {
        //std::cout << "operator centerpoint /" << std::endl;
        this->x /= a;
        this->y /= a;
        this->ele /= a;
        this->ID = -1;
        return *this;
    }
    CenterPoint3D& operator+(const CenterPoint3D &a)
    {
        //std::cout << "operator centerpoint +" << std::endl;
        this->x += a.x;
        this->y += a.y;
        this->ele += a.ele;
        this->ID = -1;
        return *this;
    }
};

struct CenterWay3D
{
    int ID;
    int size;//最大容量
    int length;//实际元素个数
    double speed_limit;//限速
    //bool isturn;//是否转弯,直行false
    relation::WayDirection direction;//道路方向
    int source;//起点CenterPoint3D -> ID
    int target;//终点CenterPoint3D -> ID
    int *centernodeline;//CenterPoint3D的id
    std::pair<int, int> neighbours;//value = 左侧车道id，右侧车道id，若没有则为-1
    bool operator==(const CenterWay3D &a) const
    {
        return (this->ID == a.ID);
    }
    CenterWay3D()
    {
        size = 5;
        length = 0;
        centernodeline = new int[size];
        speed_limit = -1;
        source = -1;
        target = -1;
        //isturn = true;
        direction = relation::WayDirection::unknown;
        // neighbours.first = -1;
        // neighbours.second = -1;
        neighbours = std::pair<int, int>(-1, -1);
    }
    void Changesize()
    {
        int *centernodelinenew = new int[size*2];
        for(int i = 0; i < length; ++i)
        {
            centernodelinenew[i] = centernodeline[i];
        }
        size *= 2;
        delete centernodeline;
        centernodeline = centernodelinenew;
    }
    int Length() const
    {
        return length;
    }
    void Reverse()
    {
        //std::cout << "centernodeline reverse!" << std::endl;
        int head = 0;
        int tail = length-1;
        while(head < tail)
        {
            int temp = centernodeline[head];
            centernodeline[head] = centernodeline[tail];
            centernodeline[tail] = temp;
            ++head;
            --tail;
        }
    }
};

class CenterWay:public map_base<CenterWay3D>
{
private:
    std::unordered_map<int, CenterPoint3D*> centerpointmap;
    void findNeighborleft(const int centerwayid_, std::vector<int> &neighbors_) const;
    void findNeighborright(const int centerwayid_, std::vector<int> &neighbors_) const;
public:
    CenterWay();
    CenterWay(TiXmlElement *root);
    //CenterWay(const CenterWay&) = delete;
    //CenterWay& operator=(const CenterWay&) = delete;
    //void Init();
    //返回centerpointmap的首元素
    std::unordered_map<int, CenterPoint3D*>::const_iterator centerpointBegin() const {return centerpointmap.cbegin();}
    //返回centerpointmap的尾元素
    std::unordered_map<int, CenterPoint3D*>::const_iterator centerpointEnd() const {return centerpointmap.cend();}
    std::unordered_map<int, CenterPoint3D*> getCenterpointmapHashMap() const {return centerpointmap;}
    //根据索引ID查找centerpoint, 使用前需验证是否存在
    CenterPoint3D* Findcenterpoint(const int id_) const {return centerpointmap.at(id_);}
    //计算两点的二维距离
    double NodeDistance2D(const CenterPoint3D *a, const CenterPoint3D *b) const;
    //计算两点的三维距离
    double NodeDistance(const node::Point3D *a, const node::Point3D *b) const;
    double NodeDistance(const CenterPoint3D *a, const CenterPoint3D *b) const;
    //计算莫way的长度
    double EdgeLength(const node::Node *nodes_, const way::Line *line) const;
    //计算某way的累计长度，返回从0依次到最大长度的vector
    std::vector<double> CalculateAccumulatedLengths(const node::Node *nodes_, const way::Line *line) const;
    //寻找最近索引
    std::pair<size_t, size_t> FindNearestIndexPair(const std::vector<double> &accumulated_lengths, 
                                                   const double target_length) const;
    //对某way重采样
    std::vector<CenterPoint3D> ResamplePoints(const node::Node *nodes_, const way::Line *line, const int num_segments) const;
    //生成道路中心线----非const
    std::vector<int> GenerateCenterline(const node::Node *nodes_, const way::Way *ways_, const relation::relationship *relationship_, const double resolution = 5.0);
    //交通标志关联到centerpoint3d
    void Matchregulatoryelement(const node::Node *nodes_, const way::Line *line, relation::regulatoryelement* sign_) const;
    //当前点到路口的距离(当前点最佳中心线的id)
    double length2intersection(const int centerpointid_, const std::vector<int> &pathid_, const relation::Relation *relations_) const;
    //绑定相邻车道
    std::pair<int, int> createNeighbor(const int centerwayid_, const relation::Relation *relations_, const way::Way *ways_) const;
    //获取相邻车道
    void findNeighbor(const int centerwayid_, std::vector<int> &neighbors_) const;
    //判断a、b车道是否相邻
    bool isNeighbor(const int a, const int b) const;
    //遍历寻找最近点的lanelet
    int findNearestCenterwaypointid(const node::Point3D *atnowpoint_) const;
    //寻找某centerwaypoint的下一点id
    void nextCenterwayPointid(const int now_centerway_point_id, CenterPoint3D &targetpoint) const;
    //超过地图时找到最近的同向lanelet
    int findNearestLanelet(const CenterPoint3D *atnowpoint_, const double &heading) const;
    //run()----非const
    void run(const node::Node *nodes_, const way::Way *ways_, const relation::Relation *relations_);
    virtual void CreateOneObject(TiXmlElement *head);
    virtual ~CenterWay();
};





};//namespace centerway
};//namespace map




#endif