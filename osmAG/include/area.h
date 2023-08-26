#ifndef _AREA_H_
#define _AREA_H_

#include "base_area.h"
#include <map>
#include <vector>
#include <memory>
#include <set>
#include <unordered_set>
#include <grid_map_core/grid_map_core.hpp>
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GridCells.h>
#include<opencv2/opencv.hpp>


namespace osm_ag{

using AreaId = int64_t;

struct GridMat{
    cv::Mat grid_mat;
    int origin_grid_x;
    int origin_grid_y;
    int offset;
    double resolution;
};


class Area : public BaseArea {
  public:
    using Ptr = std::unique_ptr<Area>;
    //! 需要判断一下
    //node container for the layer
    using Nodes = std::map<NodeId, Node::Ptr>; // every area include some  nodes 注意对于xml来说，nodes的连接是有顺序的
    using NodesInOrder = std::vector<NodeId>; // 考虑一下需不需要ptr
    using NodesCheckup = std::map<NodeId, NodeStatus>; //这个是用来对Node进行一些合并之类的操作，目前还不准备用
    using Attributes = WayAttributes;
    using AttributesPtr = std::unique_ptr<Attributes>;

    friend class AreaGraph;

    const AreaId id; // This is fixed id.
    LayerId layer_id; //now not be used 其实可能是树所在的层数，但是这个需要遍历整个树

    const std::string type; //"room" or "structure" 

    std::set<int64_t> passageids; 

    std::set<std::pair<NodeId,NodeId>> passage_nodes;

    grid_map::GridMap area_grid;

    GridMat grid_mat_;//just for temp

    nav_msgs::OccupancyGrid area_occupancy;

    Eigen::Vector3d center_position;

    /**
     * @brief Make a osmag node (usually not necessary)
     *
     * It's usually not suggested to use this directly (it's typically
     * less book-keeping to just emplace directly into the graph)
     * 
     * In an OsmAG, for static map, we can add nodes into graph use this function 
     * 需要确定一下哪些是Area必须的性质
     *
     * @param id the id of the node to create
     * @param attrs attributes for the node
     */
    explicit Area(AreaId id, AttributesPtr&& attrs, std::string type);

    inline bool hasParent() const {return has_parent_;}

    inline bool hasSiblings() const { return not siblings_.empty(); }
    inline bool hasChildren() const { return not children_.empty(); }

    // inline std::optional<AreaId> getParent() const {
    //     if (!has_parent_) {
    //         return std::nullopt;
    //     }
    //     return parent_;  //表示或处理一个“可能为空”的变量
    // }

    Attributes* getAttributesPtr() const { return attributes_.get(); }

    // NodeStatus checkNode(NodeId node_id) const override;


    // Nodes nodes_;//考虑一下是直接放在public里面，还是写个函数来读取修改

    NodesInOrder nodes_inorder_;

    NodesCheckup nodes_status_;

    AreaId parent_; // this will be parsed

    bool has_parent_ = false;

    bool use_grid_map = false;

    bool use_occupancy_map = false;

    bool init_center = false;

    bool is_leaf = true;



    inline void setparent(AreaId parent_id){
        has_parent_ = true;
        parent_ = parent_id;
    }


    virtual ~Area() = default;




//   protected:

    AttributesPtr attributes_;

    std::set<AreaId> siblings_;//兄弟和孩子不一定需要有
    std::set<AreaId> children_;


    virtual std::ostream& fill_ostream(std::ostream& out) const; //this is for printing area info

    // void update_grid_map(AreaGraph& graph, const double length = 10.0, const double width = 10.0, const double resolution = 0.2);
    
};

}


#endif