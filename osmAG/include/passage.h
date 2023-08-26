#ifndef _PASSAGE_H_
#define _PASSAGE_H_

#include "area.h"
#include <map>
#include <vector>
#include <memory>
#include <set>
#include <unordered_set>

namespace osm_ag{

using PassageId = int64_t;


struct PassageNodes{
    NodeId source;
    NodeId target;
};

enum class PassageStatus { NEW, VISIBLE, DELETED, MERGED, NONEXISTENT };

/**
 * @brief Collection of information for an passage(edge).
 * This is the same part of two areas.我们还要考虑这条通道是否是可以通行的，对于
 * 哪种机器人是可以通行的，或者我们用权重去表示
 */
class Passage : public BaseArea{
  public:

    using Ptr = std::unique_ptr<Passage>;
    //! 需要判断一下  一般来说 passage是两个节点构成的线，所以是边，是无向的
    //
    //node container for the layer
    using NodesCheckup = std::map<NodeId, NodeStatus>; //这个是用来对Node进行一些合并之类的操作，目前还不准备用
    using Attributes = WayAttributes;
    using AttributesPtr = std::unique_ptr<Attributes>;


    friend class AreaGraph;

    const PassageId id; // This is fixed id.

    PassageNodes passage_nodes;
    
    AttributesPtr info;

    AreaId area_from;//考虑一下这是初始化时必须的还是通过遍历后赋值？？？？

    AreaId area_to;

    Eigen::Vector3d center_position;

    //ToDo: if we need consider different levels areas' connection 注意之后是否需要考虑对显式的表示大区域和小区域的passage

    /**
     * @brief Make a osmag passage (usually not necessary)
     * @param id the id of the node to create
     * @param attrs attributes for the node
     */
    explicit Passage(PassageId id, PassageNodes passage_nodes, AttributesPtr&& attrs);



    Attributes* getAttributesPtr() const { return info.get(); }

    // NodeStatus checkNode(NodeId node_id) const override;

    virtual ~Passage() = default;


};

struct PassageKey {//这个key是根据node来找的，那么根据from和to需要再建一个key吗
  PassageKey(NodeId k1, NodeId k2) : k1(std::min(k1, k2)), k2(std::max(k1, k2)) {}

  inline bool operator==(const PassageKey& other) const {
    return k1 == other.k1 && k2 == other.k2;
  }

  inline bool operator<(const PassageKey& other) const {
    if (k1 == other.k1) {
      return k2 < other.k2;
    }

    return k1 < other.k1;
  }

  NodeId k1;
  NodeId k2;
};

/*
因为需要对节点之类的进行合并增加删除的操作，是否需要 一个容器对passage也进行增减，可能需要？因为需要检查passage是否符合两个area的共同部分，
否则需要删除
ToDo !!!!!!!
*/
struct PassageContainer {
  using Passages = std::map<PassageKey, Passage>;
  using PassageStatusMap = std::map<PassageKey, PassageStatus>;

  Passages passages;
  PassageStatusMap passage_status;

  // void insert(NodeId source, NodeId target, WayAttributes::Ptr&& passage_info);

  // void remove(NodeId source, NodeId target);

  // void rewire(NodeId source, NodeId target, NodeId new_source, NodeId new_target);

  // bool contains(NodeId source, NodeId target) const;

  // inline size_t size() const { return passages.size(); }

  // void reset();

  // inline const Passage& get(NodeId source, NodeId target) const {
  //   return passages.at(PassageKey(source, target));
  // }

  // inline Passage& get(NodeId source, NodeId target) {
  //   return passages.at(PassageKey(source, target));
  // }

  // PassageStatus getStatus(NodeId source, NodeId target) const;

  // void getRemoved(std::vector<PassageKey>& removed_Passages, bool clear_removed);

  // void getNew(std::vector<PassageKey>& new_Passages, bool clear_new);

};
}

#endif