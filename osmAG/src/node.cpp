#include "node.h"

namespace osm_ag{
    NodeAttributes::NodeAttributes() : position(Eigen::Vector3d::Zero()) {}

    Node::Node(NodeId node_id, Node::AttributesPtr&& attrs)
        : id(node_id), attributes_(std::move(attrs)) {}

    std::ostream& Node::fill_ostream(std::ostream& out) const {
        out << "Node<id=" << id << ">";
        return out;
    }

    OriginNode::OriginNode(NodeId id, Eigen::Vector3d rot,  AttributesPtr&& attrs)
              : Node(id, std::move(attrs)), rotation(rot) {}
}