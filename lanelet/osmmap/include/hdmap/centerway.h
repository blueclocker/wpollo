/*
 * @Author: your name
 * @Date: 2022-03-06 15:43:53
 * @LastEditTime: 2022-10-03 16:07:18
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
    int ID_;
    double x_;
    double y_;
    double ele_;
    CenterPoint3D() {}
    CenterPoint3D(const double xx, const double yy)
    {
        ID_ = -1;
        x_ = xx;
        y_ = yy;
        ele_ = 0;
    }
    CenterPoint3D(const node::Point3D &a)
    {
        ID_ = a.ID_;
        x_ = a.local_x_;
        y_ = a.local_y_;
        ele_ = a.elevation_;
    }
    bool operator==(const CenterPoint3D &a) const
    {
        return (this->ID_ == a.ID_);
    }
    CenterPoint3D& operator=(const CenterPoint3D &a)
    {
        //std::cout << "operator centerpoint =" << std::endl;
        this->ID_ = a.ID_;
        this->x_ = a.x_;
        this->y_ = a.y_;
        this->ele_ = a.ele_;
        return *this;
    }
    CenterPoint3D& operator*(const double a)
    {
        //std::cout << "operator centerpoint *" << std::endl;
        this->x_ *= a;
        this->y_ *= a;
        this->ele_ *= a;
        this->ID_ = -1;
        return *this;
    }
    CenterPoint3D& operator/(const double a)
    {
        //std::cout << "operator centerpoint /" << std::endl;
        this->x_ /= a;
        this->y_ /= a;
        this->ele_ /= a;
        this->ID_ = -1;
        return *this;
    }
    CenterPoint3D& operator+(const CenterPoint3D &a)
    {
        //std::cout << "operator centerpoint +" << std::endl;
        this->x_ += a.x_;
        this->y_ += a.y_;
        this->ele_ += a.ele_;
        this->ID_ = -1;
        return *this;
    }
};

struct CenterWay3D
{
    int ID_;
    int size_;//最大容量
    int length_;//实际元素个数
    double speed_limit_;//限速
    //bool isturn_;//是否转弯,直行false
    relation::WayDirection direction_;//道路方向
    int source_;//起点CenterPoint3D -> ID
    int target_;//终点CenterPoint3D -> ID
    int *centernodeline_;//CenterPoint3D的id
    std::pair<int, int> neighbours_;//value = 左侧车道id，右侧车道id，若没有则为-1
    bool operator==(const CenterWay3D &a) const
    {
        return (this->ID_ == a.ID_);
    }
    CenterWay3D()
    {
        size_ = 5;
        length_ = 0;
        centernodeline_ = new int[size_];
        speed_limit_ = -1;
        source_ = -1;
        target_ = -1;
        //isturn_ = true;
        direction_ = relation::WayDirection::unknown;
        // neighbours.first = -1;
        // neighbours.second = -1;
        neighbours_ = std::pair<int, int>(-1, -1);
    }
    void Changesize()
    {
        int *centernodelinenew = new int[size_*2];
        for(int i = 0; i < length_; ++i)
        {
            centernodelinenew[i] = centernodeline_[i];
        }
        size_ *= 2;
        delete centernodeline_;
        centernodeline_ = centernodelinenew;
    }
    int Length() const
    {
        return length_;
    }
    void Reverse()
    {
        //std::cout << "centernodeline reverse!" << std::endl;
        int head = 0;
        int tail = length_-1;
        while(head < tail)
        {
            int temp = centernodeline_[head];
            centernodeline_[head] = centernodeline_[tail];
            centernodeline_[tail] = temp;
            ++head;
            --tail;
        }
    }
};

class CenterWay:public map_base<CenterWay3D>
{
private:
    std::unordered_map<int, CenterPoint3D*> centerpointmap_;
    void FindNeighborleft(const int centerwayid, std::vector<int> &neighbors) const;
    void FindNeighborright(const int centerwayid, std::vector<int> &neighbors) const;
public:
    CenterWay();
    CenterWay(TiXmlElement *root);
    //CenterWay(const CenterWay&) = delete;
    //CenterWay& operator=(const CenterWay&) = delete;
    //void Init();
    //返回centerpointmap的首元素
    std::unordered_map<int, CenterPoint3D*>::const_iterator CenterpointBegin() const {return centerpointmap_.cbegin();}
    //返回centerpointmap的尾元素
    std::unordered_map<int, CenterPoint3D*>::const_iterator CenterpointEnd() const {return centerpointmap_.cend();}
    std::unordered_map<int, CenterPoint3D*> GetCenterpointmapHashMap() const {return centerpointmap_;}
    //根据索引ID查找centerpoint, 使用前需验证是否存在
    CenterPoint3D* FindCenterPoint(const int id) const {return centerpointmap_.at(id);}
    //计算两点的二维距离
    double NodeDistance2D(const CenterPoint3D *a, const CenterPoint3D *b) const;
    //计算两点的三维距离
    double NodeDistance(const node::Point3D *a, const node::Point3D *b) const;
    double NodeDistance(const CenterPoint3D *a, const CenterPoint3D *b) const;
    //计算莫way的长度
    double EdgeLength(const node::Node *nodes, const way::Line *line) const;
    //计算某way的累计长度，返回从0依次到最大长度的vector
    std::vector<double> CalculateAccumulatedLengths(const node::Node *nodes_, const way::Line *line) const;
    //寻找最近索引
    std::pair<size_t, size_t> FindNearestIndexPair(const std::vector<double> &accumulated_lengths, 
                                                   const double target_length) const;
    //对某way重采样
    std::vector<CenterPoint3D> ResamplePoints(const node::Node *nodes, const way::Line *line, const int num_segments) const;
    //生成道路中心线----非const
    std::vector<int> GenerateCenterline(const node::Node *nodes, const way::Way *ways, const relation::relationship *relationship, const double resolution = 5.0);
    //交通标志关联到centerpoint3d
    void MatchRegulatoryElement(const node::Node *nodes, const way::Line *line, relation::regulatoryelement* sign) const;
    //当前点到路口的距离(当前点最佳中心线的id)
    double Length2Intersection(const int centerpointid, const std::vector<int> &pathid, const relation::Relation *relations) const;
    //绑定相邻车道
    std::pair<int, int> CreateNeighbor(const int centerwayid, const relation::Relation *relations, const way::Way *ways) const;
    //获取相邻车道
    void FindNeighbor(const int centerwayid, std::vector<int> &neighbors) const;
    //判断a、b车道是否相邻
    bool IsNeighbor(const int a, const int b) const;
    //遍历寻找最近点的lanelet
    int FindNearestCenterwayPointid(const node::Point3D *atnowpoint) const;
    //寻找某centerwaypoint的下一点id
    void NextCenterwayPointid(const int now_centerway_point_id, CenterPoint3D &targetpoint) const;
    //超过地图时找到最近的同向lanelet
    int FindNearestLanelet(const CenterPoint3D *atnowpoint, const double &heading) const;
    //run()----非const
    void Run(const node::Node *nodes, const way::Way *ways, const relation::Relation *relations);
    virtual void CreateOneObject(TiXmlElement *head);
    virtual ~CenterWay();
};





};//namespace centerway
};//namespace map




#endif