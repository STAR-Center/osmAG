#ifndef _BASEAREA_H_
#define _BASEAREA_H_

#include "node.h"
#include <map>
#include <vector>
#include <memory>
#include <set>
#include <unordered_set>

namespace osm_ag {

using LayerId = uint64_t;

/**
 * @brief For an osmag, a node just a point with some attributes
 */
struct WayAttributes{
  public:
    using Ptr = std::unique_ptr<WayAttributes>;

    //! Make a default set of attributes
    WayAttributes() = default;

    virtual ~WayAttributes() = default;

    virtual WayAttributes::Ptr clone() const {
        return std::make_unique<WayAttributes>(*this);
    }
    bool visible;

    std::string action; // "modify" or "real"

    std::map<std::string, std::string> tags; //some other tags


  protected:
    virtual std::ostream& fill_ostream(std::ostream& out) const; //this is for printing node info
    
};

class BaseArea {
  public:
    using NodeRef = std::reference_wrapper<const Node>;
    using Ptr = std::unique_ptr<BaseArea>;

    friend class AreaGraph;

    virtual ~BaseArea()= default;

    // Eigen::Vector3d center_position;

    // BaseArea::Ptr clone() const {
    //     return std::make_unique<BaseArea>(*this);
    // }

    // virtual NodeStatus checkNode(NodeId node_id) const = 0;
    // /**
    // * @brief Get node ids of newly inserted nodes
    // */
    // virtual void getNewNodes(std::vector<NodeId>& new_nodes, bool clear_new) = 0;

    // /**
    // * @brief Get node id of deleted nodes
    // */
    // virtual void getRemovedNodes(std::vector<NodeId>& removed_nodes,
    //                            bool clear_removed) = 0;
    
  // protected:
  //   virtual bool removeNode(NodeId node_id) = 0;

};
}

#endif