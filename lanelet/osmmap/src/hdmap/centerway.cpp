/*
 * @Author: your name
 * @Date: 2022-03-06 15:44:08
 * @LastEditTime: 2022-11-03 22:32:14
 * @LastEditors: blueclocker 1456055290@hnu.edu.cn
 * @Description: 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 * @FilePath: /wpollo/src/lanelet/osmmap/src/hdmap/centerway.cpp
 */
#include "../include/osmmap/centerway.h"

namespace map
{
namespace centerway
{

CenterWay::CenterWay()
{
}

CenterWay::CenterWay(TiXmlElement *root)
{
    Setroot(root);
}

void CenterWay::CreateOneObject(TiXmlElement *head)
{
    return;
}

double CenterWay::NodeDistance2D(const CenterPoint3D *a, const CenterPoint3D *b) const
{
    double x = a->x_ - b->x_;
    double y = a->y_ - b->y_;
    return std::sqrt(x*x + y*y);
}

double CenterWay::NodeDistance(const node::Point3D *a, const node::Point3D *b) const
{
    double x = a->local_x_ - b->local_x_;
    double y = a->local_y_ - b->local_y_;
    double z = a->elevation_ - b->elevation_;
    return std::sqrt(x*x + y*y + z*z);
}

double CenterWay::NodeDistance(const CenterPoint3D *a, const CenterPoint3D *b) const
{
    double x = a->x_ - b->x_;
    double y = a->y_ - b->y_;
    double z = a->ele_ - b->ele_;
    return std::sqrt(x*x + y*y + z*z);
}
 
double CenterWay::EdgeLength(const node::Node *nodes, const way::Line *line) const 
{
    double res = 0;
    for(int i = 0; i < line->Length() - 1; ++i)
    {
        res += NodeDistance(nodes->Find(line->nodeline_[i]), nodes->Find(line->nodeline_[i+1]));
    }
    return res;
}

std::vector<double> CenterWay::CalculateAccumulatedLengths(const node::Node *nodes, const way::Line *line) const 
{
    //1st
    /*
    std::vector<double> accumulated_lengths;
    double res = 0;
    accumulated_lengths.push_back(0);
    for(int i = 0; i < line->Length() - 1; ++i)
    {
        res += NodeDistance(nodes_->Find(line->nodeline[i]), nodes_->Find(line->nodeline[i+1]));
        accumulated_lengths.push_back(res);
    }
    return accumulated_lengths;*/

    //2nd
    std::vector<double> segment_distances;
    segment_distances.reserve(line->length_ - 1);

    for (int i = 1; i < line->length_; ++i) 
    {
        const auto distance = NodeDistance(nodes->Find(line->nodeline_[i]), nodes->Find(line->nodeline_[i-1]));
        segment_distances.push_back(distance);
    }

    std::vector<double> accumulated_lengths{0};
    accumulated_lengths.reserve(segment_distances.size() + 1);
    std::partial_sum(
        std::begin(segment_distances), std::end(segment_distances),
        std::back_inserter(accumulated_lengths));

    return accumulated_lengths;
}

std::pair<size_t, size_t> CenterWay::FindNearestIndexPair(const std::vector<double> & accumulated_lengths, 
                                                          const double target_length) const 
{
    // 累加道路长度的数组的元素个数
    int N = accumulated_lengths.size();

    // 第一个
    if (target_length < accumulated_lengths.at(1)) 
    {
        return std::make_pair(0, 1);
    }

    // 最后一个
    if (target_length > accumulated_lengths.at(N - 2)) 
    {
        return std::make_pair(N - 2, N - 1);
    }

    // 中间
    for (auto i = 1; i < N; ++i) 
    {
        if (accumulated_lengths.at(i - 1) <= target_length && target_length <= accumulated_lengths.at(i)) 
        {
            return std::make_pair(i - 1, i);
        }
    }
}

std::vector<CenterPoint3D> CenterWay::ResamplePoints(const node::Node *nodes, const way::Line *line, const int num_segments) const 
{
    double line_length = EdgeLength(nodes, line);
    std::vector<double> accumulated_lengths = CalculateAccumulatedLengths(nodes, line);
    std::vector<CenterPoint3D> resampled_points;
    for(int i = 0; i <= num_segments; ++i)
    {
        // 找到最近的两个点
        double target_length = (static_cast<double>(i) / num_segments) * line_length;
        auto index_pair = FindNearestIndexPair(accumulated_lengths, target_length);

        // 线性插值
        node::Point3D back_point = *nodes->Find(line->nodeline_[index_pair.first]);
        node::Point3D front_point = *nodes->Find(line->nodeline_[index_pair.second]);
        node::Point3D direction_vector = (front_point - back_point);

        double back_length = accumulated_lengths.at(index_pair.first);
        double front_length = accumulated_lengths.at(index_pair.second);
        double segment_length = front_length - back_length;
        node::Point3D target_point_ = back_point + 
                                    (direction_vector * (target_length - back_length) / segment_length);
        
        //保存插值结果
        CenterPoint3D target_point;
        target_point.x_ = target_point_.local_x_;
        target_point.y_ = target_point_.local_y_;
        target_point.ele_ = target_point_.elevation_;
        target_point.ID_ = i;
        resampled_points.push_back(target_point);
    }
    return resampled_points;
    
}

std::vector<int> CenterWay::GenerateCenterline(const node::Node *nodes, const way::Way *ways, const relation::relationship *relationship, const double resolution)
{
    //if(relationship_->subtype == relation::RelationSubType::crosswalk) return{};
    // 寻找左右边界更长的一个
    double left_length = EdgeLength(nodes, ways->Find(relationship->leftedge_.ID_));
    double right_length = EdgeLength(nodes, ways->Find(relationship->rightedge_.ID_));
    double longer_distance = (left_length > right_length) ? left_length : right_length;
    int num_segments = std::max(static_cast<int>(ceil(longer_distance / resolution)), 1);

    // 重采样，将左右边界的点的数量统一
    std::vector<CenterPoint3D> left_points = ResamplePoints(nodes, ways->Find(relationship->leftedge_.ID_), num_segments);
    std::vector<CenterPoint3D> right_points = ResamplePoints(nodes, ways->Find(relationship->rightedge_.ID_), num_segments);
    
    // 左右边界点一一对应，计算中点
    std::vector<int> centerpointnumbers;
    for(int i = 0; i < num_segments + 1; i++)
    {
        CenterPoint3D *onecenterpoint = new CenterPoint3D;
        *onecenterpoint = (right_points.at(i) + left_points.at(i)) / 2;
        onecenterpoint->ID_ = relationship->ID_ * 100 + i;//产生的新的中心点ID = 原来道路(relation)编号*100+点的顺序
        centerpointmap_[onecenterpoint->ID_] = onecenterpoint;
        centerpointnumbers.push_back(onecenterpoint->ID_);
    }
    return centerpointnumbers;
}

void CenterWay::MatchRegulatoryElement(const node::Node *nodes, const way::Line *line, relation::regulatoryelement* sign) const 
{
    node::Point3D nodea = *nodes->Find(line->nodeline_[0]);
    node::Point3D nodeb = *nodes->Find(line->nodeline_[1]);

    node::Point3D nodeab;
    nodeab.local_x_ = (nodea.local_x_ + nodeb.local_x_) / 2.0;
    nodeab.local_y_ = (nodea.local_y_ + nodeb.local_y_) / 2.0;
    nodeab.elevation_ = (nodea.elevation_ + nodeb.elevation_) / 2.0;
    
    //计算停止线中点与各lanelet道路中心线的终点的距离，取最小距离对应匹配
    CenterPoint3D centerab = CenterPoint3D(nodeab);
    double distance = 10000.0;
    int laneletid = -1;
    for(auto it = Begin(); it != End(); ++it)
    {
        CenterPoint3D centertarget = *centerpointmap_.at(it->second->target_);
        double distemp = NodeDistance(&centerab, &centertarget);
        //std::cout << "distance: " << distemp << std::endl;
        if(distemp < distance)
        {
            distance = distemp;
            laneletid = it->second->ID_;
        }
    }
    sign->laneletid_ = laneletid;
    //std::cout << "min distance: " << distance_ << std::endl;
    //匹配失败
    if(laneletid != -1)
    {
        sign->centerpoint3did_ = Find(laneletid)->target_;
    }
}

double CenterWay::Length2Intersection(const int centerpointid, const std::vector<int> &pathid, const relation::Relation *relations) const
{
    if(pathid.empty()) return -1;
    std::vector<int> centerwayid_before_turning;
    int i = 0;
    //如果当前在弯道，则保留当前弯道，直到下一次转向出现
    if(Find(pathid[i])->direction_ == relation::WayDirection::left || 
       Find(pathid[i])->direction_ == relation::WayDirection::right)
    {
        centerwayid_before_turning.push_back(pathid[i++]);
    }
    for(; i < pathid.size(); ++i)
    {
        if(Find(pathid[i])->direction_ == relation::WayDirection::straight)
        {
            centerwayid_before_turning.push_back(pathid[i]);
            if(relations->IsStopLine(pathid[i])) break;
        }else{
            break;
        }
    }
    
    double length_ = 0;
    for(i = 0; i < centerwayid_before_turning.size(); ++i)
    {
        CenterWay3D *centerway_pin = Find(centerwayid_before_turning[i]);
        bool flag_ = false;
        for(int j = 0; j < centerway_pin->length_ - 1; ++j)
        {
            //第一段路
            if(i == 0)
            {
                if(centerway_pin->centernodeline_[j] == centerpointid) flag_ = true;
                if(!flag_) continue;
                length_ += NodeDistance(centerpointmap_.at(centerway_pin->centernodeline_[j]), centerpointmap_.at(centerway_pin->centernodeline_[j+1]));
            }else{
                length_ += NodeDistance(centerpointmap_.at(centerway_pin->centernodeline_[j]), centerpointmap_.at(centerway_pin->centernodeline_[j+1]));
            }
        }
    }
    return length_;
}

std::pair<int, int> CenterWay::CreateNeighbor(const int centerwayid, const relation::Relation *relations, const way::Way *ways) const
{
    std::pair<int, int> neighbor;
    int thisleftid = relations->Find(centerwayid)->leftedge_.ID_;
    int thisrightid = relations->Find(centerwayid)->rightedge_.ID_;
    bool findleft = false;
    bool findright = false;
    //如果左右边线是road_border，则不存在相邻车道
    if(ways->Find(thisleftid)->type_ == way::WayType::road_border)
    {
        neighbor.first = -1;
        findleft = true;
    }
    if(ways->Find(thisrightid)->type_ == way::WayType::road_border)
    {
        neighbor.second = -1;
        findright = true;
    }

    for(auto it = relations->Begin(); it != relations->End(); ++it)
    {
        if(!findleft)
        {
            if(thisleftid == it->second->rightedge_.ID_)
            {
                neighbor.first = it->second->ID_;
                findleft = true;
            }
        }
        if(!findright)
        {
            if(thisrightid == it->second->leftedge_.ID_)
            {
                neighbor.second = it->second->ID_;
                findright = true;
            }
        }
        if(findleft && findright)
        {
            break;
        }
    }

    return neighbor;
}

void CenterWay::FindNeighborleft(const int centerwayid, std::vector<int> &neighbors) const
{
    if(centerwayid != -1)
    {
        FindNeighborleft(Find(centerwayid)->neighbours_.first, neighbors);
        neighbors.push_back(centerwayid);
        //std::cout << centerwayid_ << std::endl;
    }
}

void CenterWay::FindNeighborright(const int centerwayid, std::vector<int> &neighbors) const
{
    if(centerwayid != -1)
    {
        neighbors.push_back(centerwayid);
        //std::cout << centerwayid_ << std::endl;
        FindNeighborright(Find(centerwayid)->neighbours_.second, neighbors);
    }
}

void CenterWay::FindNeighbor(const int centerwayid, std::vector<int> &neighbors) const
{
    if(!IsExist(centerwayid))
    {
        std::cout << "out of map" << std::endl;
        return;
    }
    FindNeighborleft(centerwayid, neighbors);
    if(!neighbors.empty()) neighbors.pop_back();
    FindNeighborright(centerwayid, neighbors);
}

bool CenterWay::IsNeighbor(const int a, const int b) const
{
    if(IsExist(a) || !IsExist(b))
    {
        return false;
    }
    if(Find(a)->neighbours_.first == b || Find(a)->neighbours_.second == b)
    {
        return true;
    }
    return false;
}

int CenterWay::FindNearestCenterwayPointid(const node::Point3D *atnowpoint) const
{
    int nearest_center_point_id = -1;
    double distemp = DBL_MAX;
    CenterPoint3D startpoint(*atnowpoint);
    for(auto it = centerpointmap_.begin(); it != centerpointmap_.end(); ++it)
    {
        // 判定车与路同向的下一点, 暂不判定
        // double bx = it->second->x - atnowpoint_->local_x;
        // double by = it->second->y - atnowpoint_->local_y;
        // double a_b = bx * cos(heading) + by * sin(heading);

        double temp = NodeDistance(&startpoint, it->second);
        if(temp < distemp)
        {
            nearest_center_point_id = it->first;
            distemp = temp;
        }
    }
    return nearest_center_point_id;
}

void CenterWay::NextCenterwayPointid(const int now_centerway_point_id, CenterPoint3D &targetpoint) const
{
    CenterWay3D* candicate_centerway = Find(now_centerway_point_id / 100);
    CenterPoint3D *a, *b;
    if(now_centerway_point_id % 100 < candicate_centerway->length_ - 1)
    {
        a = centerpointmap_.at(now_centerway_point_id);
        b = centerpointmap_.at(now_centerway_point_id + 1);
    }else{
        a = centerpointmap_.at(now_centerway_point_id - 1);
        b = centerpointmap_.at(now_centerway_point_id);
    }
    if((b->x_ - a->x_ < 1e-5))
    {
        targetpoint.x_ = b->x_;
        targetpoint.y_ = b->y_ > a->y_ ? b->y_ + 5 : b->y_ - 5;
    }else{
        targetpoint.x_ = b->x_ + 5;
        targetpoint.y_ = b->y_ + 5 * (b->y_ - a->y_) / (b->x_ - a->x_);
    }
}

int CenterWay::FindNearestLanelet(const CenterPoint3D *atnowpoint, const double &heading) const
{
    double distemp = DBL_MAX;
    int lanelet_id = -1;
    for(auto it = Begin(); it != End(); ++it)
    {
        CenterWay3D *acenterway = it->second;
        for(int i = 0; i < acenterway->length_-1; ++i)
        {
            double bx = centerpointmap_.at(acenterway->centernodeline_[i+1])->x_ - centerpointmap_.at(acenterway->centernodeline_[i])->x_;
            double by = centerpointmap_.at(acenterway->centernodeline_[i+1])->y_ - centerpointmap_.at(acenterway->centernodeline_[i])->y_;
            double a_b = bx * cos(heading) + by * sin(heading);
            if(a_b < 0) continue;

            double temp = NodeDistance(atnowpoint, centerpointmap_.at(acenterway->centernodeline_[i+1]));
            if(temp < distemp)
            {
                lanelet_id = acenterway->centernodeline_[i+1];
                distemp = temp;
            }
        }
    }
    return lanelet_id == -1 ? -1 : lanelet_id/100;
}

void CenterWay::Run(const node::Node *nodes, const way::Way *ways, const relation::Relation *relations)
{
    for(auto it = relations->Begin(); it != relations->End(); ++it)
    {
        if((it->second)->type_ == relation::RelationType::lanelet && (it->second)->subtype_ != relation::RelationSubType::crosswalk)
        {
            CenterWay3D *oneobject = new CenterWay3D;
            oneobject->ID_ = it->second->ID_;//中心线ID沿用原始realtion的ID号
            oneobject->speed_limit_ = it->second->speed_limit_;
            /*if((it->second)->turn_direction == relation::WayDirection::straight)
            {
                oneobject->isturn = false;
            }*/
            oneobject->direction_ = it->second->turn_direction_;
            std::vector<int> onecenterline = GenerateCenterline(nodes, ways, it->second);
            
            //判断道路走向,确定source/target
            //取第一个道路中心线点
            CenterPoint3D c0 = *centerpointmap_[*onecenterline.begin()];
            //取第二个道路中心线点
            CenterPoint3D c1 = *centerpointmap_[*(onecenterline.begin() + 1)];
            //取道路左边界第一个点
            node::Point3D l0 = *nodes->Find(ways->Find(it->second->leftedge_.ID_)->nodeline_[0]);
            //道路中心线第二个到第一个的连线向量----#1
            double c01x = c1.x_ - c0.x_;
            double c01y = c1.y_ - c0.y_;
            //对C01向量顺时针旋转90度，作为基准判定方向
            double xx = c01y;
            double xy = -c01x;
            //左边界第一点到道路中心线第一点的连线向量----#2
            double l0c0x = l0.local_x_ - c0.x_;
            double l0c0y = l0.local_y_ - c0.y_;
            //若向量#1与向量#2的点积 < 0, 说明道路方向与向量#1同向
            if(xx*l0c0x + xy*l0c0y < 0)
            {
                //std::cout << "correct order" << std::endl;
                oneobject->source_ = *onecenterline.begin();
                oneobject->target_ = *(onecenterline.end() - 1);
            }else{
                //否则，道路方向与向量#1反向
                //std::cout << "uncorrect order" << std::endl;
                oneobject->target_ = *onecenterline.begin();
                oneobject->source_ = *(onecenterline.end() - 1);
            }
            
            //如果元素过多则倍增数组容量
            for(int i = 0; i < onecenterline.size(); ++i)
            {
                oneobject->centernodeline_[oneobject->length_++] = onecenterline[i];
                if(oneobject->length_ == oneobject->size_)
                {
                    oneobject->Changesize();
                }
            } 
            //反转oneobject->centernodeline顺序，使其首个元素与source保持一致
            if(oneobject->source_ != *onecenterline.begin()) 
            {
                oneobject->Reverse();
            }
            //test
            /*std::cout << "-------------" << std::endl;
            std::cout << "centerline: " << it->second->ID << std::endl;
            for(int i = 0; i < oneobject->length; ++i)
            {
                std::cout << oneobject->centernodeline[i] << " ";
            }
            std::cout << std::endl;
            std::cout << "-------------" << std::endl;*/

            //寻找相邻车道
            std::pair<int, int> neighbor_temp = CreateNeighbor(oneobject->ID_, relations, ways);
            oneobject->neighbours_.first = neighbor_temp.first;
            oneobject->neighbours_.second = neighbor_temp.second;
            //std::cout << "path " << oneobject->ID << ", left: " << neighbor_temp.first << ", right: " << neighbor_temp.second << std::endl;

            Insert(it->second->ID_, oneobject);
            AddNumbers();
            //std::cout << it->second->ID << ": " << Find(it->second->ID)->length << std::endl;
            //std::cout << "centerpointmap " << it->first << " size: " << centerpointmap.size() << std::endl;
        }
    }
    for(auto it = relations->RegulatoryelementBegin(); it != relations->RegulatoryelementEnd(); ++it)
    {
        relation::relationship *relationship = relations->Find(it->second->ID_);
        it->second->subtype_ = relationship->subtype_;
        it->second->stoplineid_ = relationship->ref_line_;
        //交通信号标志在map_relation中CreateOneObject()函数中声明并开辟存储空间
        MatchRegulatoryElement(nodes, ways->Find(it->second->stoplineid_), it->second);
        // std::cout << "traffic id " << it->second->ID << std::endl;
        // std::cout << "traffic sign in " << it->second->laneletid << std::endl;
        // std::cout << "traffic sign at " << it->second->centerpoint3did << std::endl;
    }

    //test
    // if(relations_->isRegulatoryelement(182))
    // {
    //     std::cout << "have" << std::endl;
    //     std::cout << relations_->getRegulatoryelement(182).size() << std::endl;
    // }
}

CenterWay::~CenterWay()
{
    // std::cout << "~centerway" << std::endl;
    for(auto it = centerpointmap_.begin(); it != centerpointmap_.end(); ++it)
    {
        delete it->second;
    }
    centerpointmap_.clear();

}


};//namespace centerway
};//namespace map