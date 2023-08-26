#ifndef _PATHGRAPH_H_
#define _PATHGRAPH_H_
#include "areagraph.h"
#include <limits>
#include <cmath>
#include <unordered_map>
#include <iostream>
#include <queue>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>


namespace osm_ag{

enum Distance_Type {L1, L2, ASTAR};

using PathedgesTrjMap = std::map<std::pair<PassageId, PassageId>, std::vector<Eigen::Vector3d>>;


struct PathNode{//或者只需要passage的id就行   : public Passage ????
    PathNode():previous(nullptr) {}
    // PathNode(PassageId id):passage_id(id){}

    PassageId passage_id = 0;//我们只需要用到graph当中的passage的一些信息，通过id进行查询就行了;
    Eigen::Vector3d position;

    // PathNode(PassageId id): passage_id(id) {};


    //for plan
    PathNode* previous = nullptr;


    double h_value = 0.0;//h=0，那么和dijkstra一样
    double g_value = std::numeric_limits<double>::max();
    double f_value = h_value + g_value;
    bool is_open = false;
    bool is_closed = false;

    void InitForPlanning(double hv=0., double gv = std::numeric_limits<double>::max(), bool open = false, bool close = false){//a*和dij
        h_value = hv;
        g_value = gv;
        f_value = h_value + g_value;
        is_open = open;
        is_closed = close;
    }

    void update_f(){
        f_value = h_value + g_value;
    }

    bool operator < (const PathNode& cmp) const {
        //为了优先队列排序，默认大的在前面， 所以写小于的时候用大于
        return f_value > cmp.f_value;
    }

};

// struct PathNode_Plan{
//     PathNode* vertex;
//     PathNode_Plan* previous = nullptr;

//     double h_value = 0.0;//h=0，那么和dijkstra一样
//     double g_value = std::numeric_limits<double>::max();
//     double f_value = h_value + g_value;
//     bool is_open = false;
//     bool is_closed = false;

//     void Init(PathNode* node){//Todo 写成构造函数
//         vertex = node;
//     }

//     void InitForPlanning(double hv, double gv, bool open, bool close){//a*和dij
//         h_value = hv;
//         g_value = gv;
//         is_open = open;
//         is_closed = close;
//     }

//     bool operator < (const PathNode_Plan& cmp) const {
//         //为了优先队列排序，默认大的在前面， 所以写小于的时候用大于
//         return f_value > cmp.f_value;
//     }


// };

struct PathEdge{//建立边
    PathEdge(){}
    PathEdge(PassageId from, PassageId to):from(from),to(to) {}
    
    PassageId from;
    PassageId to;
    double dist_L2;//用最简单的两条边的欧式距离或者用a*的距离（因为有node（points）的值）
    double dist_L1;
    double dist_astar;
    AreaId area_id_belongto;
    std::vector<Eigen::Vector3d> trj_astar;//from->to, pts' 3d cordinate.
};

struct Path{
    Path(){}
    Path(PassageId from, PassageId to):from(from),to(to) {}
    PassageId from;
    PassageId to;
    std::vector<PassageId> path;
    double dist_L2;//用最简单的两条边的欧式距离或者用a*的距离（因为有node（points）的值）
    double dist_L1;
    double dist_astar;
};

//需要对所有最底层的每个区域都计算PathEdge
// void Find_PathInArea(AreaGraph& graph, AreaId area_id, std::vector<PathEdge*>& pathedges);

/**
*@brief To construct the base passage graph for planning. 通过遍历一个内区域所包含的所有边。
*@param pathnodes passages as pathnode of the lowest level area
*@param pathedges 属于同一个area的不同的passage间组成的edge
*/
class PassageGraph{
public:
    std::map<PassageId, PathNode*> pathnodes;//就是passageid以及其对应的value
    std::vector<PathEdge*> pathedges;//属于同一个area的不同的passage间组成的edge
    PathedgesTrjMap pathedges_trj;
};


class PathBase{
public:
    //包含整张图的
    std::map<PassageId, PathNode*> pathnodes;//就是passageid以及其对应的value
    std::vector<PathEdge*> pathedges;//属于同一个area的不同的passage间组成的edge
    std::vector<Path*> paths;//属于不同area的不同的passage间组成的edge，必须用a*的dist，因为是一条路径
    //是否需要建立搜索表 now we build a map
    PathedgesTrjMap pathedges_trj;// 只是同一个area的不同的passage


    Distance_Type dis_type = ASTAR;//The default distance is Euclidean.

    PathBase() = default;

    virtual ~PathBase() = default;
    // void MinDist(AreaGraph& graph,PassageId start_id, std::unordered_set<PassageId> end_ids);//计算所有的pathnode间的最短距离 Dijkstra

    // void AddEdge_AStar(PassageId start_id, PassageId end_id);//rewrite in the next line
    void AddEdge_AStar(PassageId start_id, PassageId end_id, Distance_Type type = L1);

    void FindNeighbors(std::vector<std::pair<PassageId,double>>& subs, PassageId current_id);

    void PrintEdgesDistance(AreaGraph& graph, std::ofstream& outfile);

};


/**
*@brief PathLayer和PathBase的不同在于只需要计算并保存出口通道之间的路径，而PathBase并不需要计算path，只需要为预计算建立passage graph
*/
class PathLayer : public PathBase{
public:

    //初始化，较高一层的areaid
    PathLayer(AreaGraph& graph,AreaId parent_areaid);
    // PathLayer(const AreaGraph& graph,AreaId parent_areaid);

    AreaId parent_area_id;

    virtual ~PathLayer() = default;
    // void MinDist(AreaGraph& graph,PassageId start_id, std::unordered_set<PassageId> end_ids);//计算所有的pathnode间的最短距离 Dijkstra

};

class PathGraph : public PathBase{
public:
    //包含整张图的

    //初始化，traverse all the lowest level passagess. 当前版本的话可以直接用该函数构建整张地图的passage_graph.
    PathGraph(AreaGraph& graph, PassageGraph& passage_graph);
    // PathLayer(const AreaGraph& graph,AreaId parent_areaid);

    virtual ~PathGraph() = default;
    // void MinDist(AreaGraph& graph,PassageId start_id, std::unordered_set<PassageId> end_ids);//计算所有的pathnode间的最短距离 Dijkstra

};



}

#endif