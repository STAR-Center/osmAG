#include "passage.h"


namespace osm_ag{
    Area::Area(AreaId Area_id, Area::AttributesPtr&& attrs, std::string area_type)
        : id(Area_id), attributes_(std::move(attrs)), type(area_type) {}


    Passage::Passage(PassageId passage_id, PassageNodes passage_vertices, AttributesPtr&& attrs)
        : id(passage_id), passage_nodes(passage_vertices), info(std::move(attrs)) {}


    std::ostream& Area::fill_ostream(std::ostream& out) const {
        out << "Way<id=" << id << ">";
        return out;
    }

    std::ostream& WayAttributes::fill_ostream(std::ostream& out) const {
        if(!WayAttributes::tags.empty()){
            out << "<tag=" << "osmAG:areaType" << "/>" ;
        }
        
        return out;
    }

    // NodeStatus Area::checkNode(NodeId node_id) const {
    // if (nodes_status_.count(node_id) == 0) {
    //     return NodeStatus::NONEXISTENT;
    // }
    // return nodes_status_.at(node_id);
    // }





    // void Area::update_grid_map(AreaGraph& graph, const double length, const double width, const double resolution){
    //     area_grid.setGeometry(grid_map::Length(length,width), resolution, grid_map::Position(0.0,0.0));
    //     for(auto it = nodes_inorder_.begin(); it != nodes_inorder_.end()-1; it++){
            
    //         grid_map::Position start(nodes_[*it]->attributes_->position[0],nodes_[*it]->attributes_->position[1]);
    //         grid_map::Position end(nodes_[*(it+1)]->attributes_->position[0],nodes_[*(it+1)]->attributes_->position[1]);
    //         grid_map::LineIterator iter_line(area_grid, start, end);
    //     }
    // }

    //
}