#ifndef _AREAGRAPH_H_
#define _AREAGRAPH_H_

#include "passage.h"
#include <map>
#include <vector>
#include <memory>
#include <set>
#include <unordered_set>

namespace osm_ag{

/**
 * @brief 对于一个areagraph来说，最基本的元素就是nodes，
 * 以及构成的way
 * @param initial parsed the root node and get the origin pose
 * 
 */

class AreaGraph{

  public:
    using Ptr = std::shared_ptr<AreaGraph>;
    //! Static node reference
    //using NodeRef = std::reference_wrapper<const Node>;

    using NodeVertices = std::map<NodeId, Node::Ptr>;

    using Areas = std::map<AreaId, Area::Ptr>;

    using Passages = std::map<PassageId, Passage::Ptr>;//是否需要把所有passage都加到graph的邻接表当中？？？ 可能不用boost graph lib 但是要用邻接矩阵 用Eigen

    //dsg includes some layers like places rooms buildings and mesh. every dsg has a huge mesh
    // now our grid map just for every area, and then maybe merge them

    friend class FindPath;//then the a* algorithm can use the private value

    // friend class PathLayer;
    friend class PathBase;


    AreaGraph(){printf("Area Graph initialized!!!\n");}

    AreaGraph(NodeVertices nodes, Areas areas, Passages passages) {;}

    virtual ~AreaGraph() = default;


    NodeVertices nodes_;
    Areas areas_;
    Passages passages_;

    NodeId max_node_id = INT64_MIN;
    NodeId min_node_id = INT64_MAX;


    std::map<AreaId, std::unordered_set<AreaId>> area_trees;
    std::map<AreaId, std::unordered_set<PassageId>> passage_trees;

    bool initial = false;

    std::pair<NodeId, OriginNode::Ptr> origin_;

    //ToDo 对图的一些操作！！！
    //对map是直接覆盖

    void AddNodeVertex(Node::Ptr&& node_ptr){
        NodeId idx_ = node_ptr->id;
        nodes_[idx_] = std::move(node_ptr);
    }

    void AddArea(Area::Ptr&& area_ptr){
        AreaId idx_ = area_ptr->id;
        areas_[idx_] = std::move(area_ptr);
    }
    
    void AddPassage(Passage::Ptr&& passage_ptr){
        PassageId idx_ = passage_ptr->id;
        passages_[idx_] = std::move(passage_ptr);
    }


    int NodeNums(){
        return nodes_.size();
    }

};



}

#endif