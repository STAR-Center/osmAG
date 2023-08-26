#ifndef _NODE_H_
#define _NODE_H_
#include <map>
#include <vector>
#include <memory>
#include <string>
#include <unordered_set>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include<fstream>

namespace osm_ag{

using NodeId = int64_t;

struct GeoCoordinate{
    double latitude;
    double longtitude;
    double altitude = 0.0;
};


/**
 * @brief For an osmag, a node just a point with some attributes
 */
struct NodeAttributes{
  public:
    using Ptr = std::unique_ptr<NodeAttributes>;

    //! Make a default set of attributes
    NodeAttributes();

    //! Set the node position
    //explicit NodeAttributes(const Eigen::Vector3d& position);

    ~NodeAttributes() = default;

    NodeAttributes::Ptr clone() const {
        return std::make_unique<NodeAttributes>(*this);
    }

    //! Position of the node
    Eigen::Vector3d position = {0.0, 0.0, 0.0}; // we should transform geo_position to xyz&angle position using ref origin node

    GeoCoordinate geo_position;

    bool visible;

    std::string action; // "modify" or "real"

    std::map<std::string, uint64_t> osm_otherinfos_number; //these are for "uid"....

    std::map<std::string, std::string> osm_otherinfos_string; //these are for "user"...

    std::map<std::string, std::string> tags; //because nodes are points without so many semantic information


  protected:
    std::ostream& fill_ostream(std::ostream& out) const; //this is for printing node info
    
};

/**
 * @brief Base node status.
 *
 * Mostly for keeping history and status of nodes in a graph
 */
enum class NodeStatus { NEW, VISIBLE, MERGED, DELETED, NONEXISTENT };


class Area;
class Passage;

class Node{
  public:
    NodeId id; // this maybe can be changable so not be const now
    using Attributes = NodeAttributes;
    using AttributesPtr = std::unique_ptr<Attributes>;
    using Ptr = std::unique_ptr<Node>;
    friend class Area;
    friend class Passage;
    friend class AreaGraph;


    /**
     * @brief Make a osmag node (usually not necessary)
     *
     * It's usually not suggested to use this directly (it's typically
     * less book-keeping to just emplace directly into the graph)
     * 
     * In an OsmAG, for static map, we can add nodes into graph use this function 
     *
     * @param id the id of the node to create
     * @param attrs attributes for the node
     */
    Node(NodeId id, AttributesPtr&& attrs);//暂时不写拷贝构造函数

    std::unordered_set<int64_t> area_belongto_;//用于编辑修改node和area 或者用邻接矩阵的边来表示
    std::unordered_set<int64_t> passage_belongto_;//用于编辑修改node和area 或者用邻接矩阵的边来表示


    virtual  ~Node() = default;

    AttributesPtr attributes_;
    
    Attributes* getAttributesPtr() const { return attributes_.get(); } //智能指针 也就相当于新的指针和智能指针共同管理一个对象

  protected:
    // AttributesPtr attributes_;
    virtual std::ostream& fill_ostream(std::ostream& out) const; //this is for printing node info
    
};

class OriginNode : public Node {
  public:
    Eigen::Vector3d rotation;
    OriginNode(NodeId id, Eigen::Vector3d rot,  AttributesPtr&& attrs);
    virtual ~OriginNode() = default;
};

}

#endif