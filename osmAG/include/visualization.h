#ifndef _VISUALIZATION_H_
#define _VISUALIZATION_H_
#include "pathgraph.h"
#include<opencv2/opencv.hpp>
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>

using namespace osm_ag;

#define IMAGE_W 2000
#define IMAGE_H 1200

/**
*@brief Plot all nodes(points) in an image to check if the coordinate is correct.
*@param resolution The default is 0.1, is meter per pixel (m/pixel)
*@param parameter-name description
*/
void PlotNodePoints(AreaGraph& graph, cv::Mat& image, double resolution = 0.1){
    // image的尺寸宏定义，需要用来算分辨率,需要根据规模进行缩放 TODO
    for(auto node_it = graph.nodes_.begin(); node_it!=graph.nodes_.end(); node_it++){
        int x = node_it->second->attributes_->position[0]/resolution + 9 * IMAGE_W/10;
        int y = node_it->second->attributes_->position[1]/resolution + IMAGE_H/4;
        cv::Point2i points_2d(x,y);
        cv::circle(image, points_2d, 2, cv::Scalar(255,0,0));
    }
    // cv::imwrite("show.png",image);
}

void PlotArea(AreaGraph& graph, cv::Mat& image, AreaId id, double resolution = 0.1){
    cv::Point2i points_2d_last;
    int i = 0;
    for(auto node_id : graph.areas_[id]->nodes_inorder_){
        int x = graph.nodes_[node_id]->attributes_->position[0]/resolution + 9 * IMAGE_W/10;
        int y = graph.nodes_[node_id]->attributes_->position[1]/resolution + IMAGE_H/4;
        cv::Point2i points_2d(x,y);
        if(i>0){
            cv::line(image, points_2d_last, points_2d, cv::Scalar(0,255,0));
        }
        points_2d_last = points_2d;
        i++;
    }
    // cv::imwrite("show_area.png",image);
}

void PlotAreas(AreaGraph& graph, cv::Mat& image, double resolution = 0.1){
    for(auto area_it = graph.areas_.begin(); area_it!=graph.areas_.end(); area_it++){
        PlotArea(graph, image, area_it->first, resolution);
    }
    cv::imwrite("show_areas.png",image);
}

void PlotPassage(AreaGraph& graph, cv::Mat& image, PassageId id, double resolution = 0.1){
    auto source_id = graph.passages_[id]->passage_nodes.source;
    auto target_id = graph.passages_[id]->passage_nodes.target;

    int x_s = graph.nodes_[source_id]->attributes_->position[0]/resolution + 9 * IMAGE_W/10;
    int y_s = graph.nodes_[source_id]->attributes_->position[1]/resolution + IMAGE_H/4;
    cv::Point2i points_s(x_s,y_s);

    int x_t = graph.nodes_[target_id]->attributes_->position[0]/resolution + 9 * IMAGE_W/10;
    int y_t = graph.nodes_[target_id]->attributes_->position[1]/resolution + IMAGE_H/4;
    cv::Point2i points_t(x_t,y_t);

    cv::line(image, points_s, points_t, cv::Scalar(0,0,255), 1.5);
}

void PlotPassages(AreaGraph& graph, cv::Mat& image, double resolution = 0.1){
    for(auto passage_it = graph.passages_.begin(); passage_it!=graph.passages_.end(); passage_it++){
        PlotPassage(graph, image, passage_it->first, resolution);
    }
    // cv::imwrite("show_passages.png",image);
}


void PlotPath(std::vector<Eigen::Vector3d> path, cv::Mat& image, double resolution = 0.1){
    cv::Point2i points_2d_last;
    int i = 0;
    for(auto pt:path){
        int x = pt[0]/resolution + 9 * IMAGE_W/10;
        int y = pt[1]/resolution + IMAGE_H/4;
        cv::Point2i points_2d(x,y);
        if(i>0){
            // cv::line(image, points_2d_last, points_2d, cv::Scalar(0,0,0));
            cv::line(image, points_2d_last, points_2d, cv::Scalar(0,255,0),2);
        }else{
            // cv::circle(image, points_2d, 3, cv::Scalar(100,100,50));
        }
        points_2d_last = points_2d;
        i++;
    }
}


void PlotAreaInGrid(AreaGraph& graph, cv::Mat& image, AreaId id, double resolution = 0.1){
    std::vector<std::vector<cv::Point2i>> contour_pts(1,std::vector<cv::Point2i> ());
    cv::Point2i points_2d_last;
    int i = 0;
    for(auto node_id : graph.areas_[id]->nodes_inorder_){
        int x = graph.nodes_[node_id]->attributes_->position[0]/resolution + 9 * IMAGE_W/10;
        int y = graph.nodes_[node_id]->attributes_->position[1]/resolution + IMAGE_H/4;
        cv::Point2i points_2d(x,y);
        if(i>0){
            contour_pts[0].push_back(points_2d);
            cv::line(image, points_2d_last, points_2d, cv::Scalar(0));
        }
        points_2d_last = points_2d;
        i++;
    }
    cv::drawContours(image,contour_pts,-1,cv::Scalar(255),CV_FILLED);//空闲区域是白色的

    i=0;
    NodeId node_id_last;
    for(auto node_id : graph.areas_[id]->nodes_inorder_){
        int x = graph.nodes_[node_id]->attributes_->position[0]/resolution + 9 * IMAGE_W/10;
        int y = graph.nodes_[node_id]->attributes_->position[1]/resolution + IMAGE_H/4;
        cv::Point2i points_2d(x,y);
        if(i>0){
            std::pair<NodeId,NodeId> p(node_id,node_id_last);
            std::pair<NodeId,NodeId> p_i(node_id_last,node_id);
            if( !graph.areas_[id]->passage_nodes.count(p) && !graph.areas_[id]->passage_nodes.count(p_i)){ //not belong the passage
                cv::line(image, points_2d_last, points_2d, cv::Scalar(0), 1);//TODO: DEBUG!!!!有的passage被错误地添加为了墙壁
            }
        }
            points_2d_last = points_2d;
            node_id_last = node_id;
            i++;
        }

    // cv::imwrite("show_area.png",image);
}

void PlotHighAreaInGrid(AreaGraph& graph, cv::Mat& image, AreaId id, double resolution = 0.1){
    std::vector<std::vector<cv::Point2i>> contour_pts(1,std::vector<cv::Point2i> ());
    cv::Point2i points_2d_last;
    int i = 0;
    NodeId node_id_last;
    for(auto node_id : graph.areas_[id]->nodes_inorder_){
        int x = graph.nodes_[node_id]->attributes_->position[0]/resolution + 9 * IMAGE_W/10;
        int y = graph.nodes_[node_id]->attributes_->position[1]/resolution + IMAGE_H/4;
        cv::Point2i points_2d(x,y);
        if(i>0){
            std::pair<NodeId,NodeId> p(node_id,node_id_last);
            std::pair<NodeId,NodeId> p_i(node_id_last,node_id);
            if( !graph.areas_[id]->passage_nodes.count(p) && !graph.areas_[id]->passage_nodes.count(p_i)){ //not belong the passage
                cv::line(image, points_2d_last, points_2d, cv::Scalar(0), 1);//TODO: DEBUG!!!!有的passage被错误地添加为了墙壁
            }
        }
            points_2d_last = points_2d;
            node_id_last = node_id;
            i++;
        }

    // cv::imwrite("show_area.png",image);
}

void PlotAreasInGrid(AreaGraph& graph, cv::Mat& image, double resolution = 0.1){
    for(auto area_it = graph.areas_.begin(); area_it!=graph.areas_.end(); area_it++){
        if(area_it->second->is_leaf){//前提：：：在pathgraph那里进行叶子的判断！！！需要改进
            PlotAreaInGrid(graph, image, area_it->first, resolution);
        }else{
            PlotHighAreaInGrid(graph, image, area_it->first, resolution);
        }
    }
    cv::imwrite("show_areas.png",image);
}

void PlotPassageInGrid(AreaGraph& graph, cv::Mat& image, PassageId id, double resolution = 0.1){
    auto source_id = graph.passages_[id]->passage_nodes.source;
    auto target_id = graph.passages_[id]->passage_nodes.target;

    int x_s = graph.nodes_[source_id]->attributes_->position[0]/resolution + 9 * IMAGE_W/10;
    int y_s = graph.nodes_[source_id]->attributes_->position[1]/resolution + IMAGE_H/4;
    cv::Point2i points_s(x_s,y_s);

    int x_t = graph.nodes_[target_id]->attributes_->position[0]/resolution + 9 * IMAGE_W/10;
    int y_t = graph.nodes_[target_id]->attributes_->position[1]/resolution + IMAGE_H/4;
    cv::Point2i points_t(x_t,y_t);

    int x_m = (x_s + x_t)/2 ;
    int y_m = (y_s + y_t)/2 ;
    cv::Point2i points_m(x_m,y_m);

    cv::line(image, points_s, points_t, cv::Scalar(255), 1);
    // cv::circle(image,points_m,3,cv::Scalar(255),CV_FILLED);
}

void PlotPassagesInGrid(AreaGraph& graph, cv::Mat& image, double resolution = 0.1){
    for(auto passage_it = graph.passages_.begin(); passage_it!=graph.passages_.end(); passage_it++){
        PlotPassageInGrid(graph, image, passage_it->first, resolution);
    }
    // cv::imwrite("show_passages.png",image);
}

cv::Point2i XYZ2Grid(Eigen::Vector3d point, double resolution = 0.1){
    int x_grid = point[0]/resolution + 9 * IMAGE_W/10;
    int y_grid = point[1]/resolution +  IMAGE_H/4;
    cv::Point2i result(x_grid,y_grid);
    return result;
}

Eigen::Vector3d Grid2XYZ_vis(cv::Point pt_grid, double resolution = 0.1){
    Eigen::Vector3d result;
    result[0] = (pt_grid.x - 9 * IMAGE_W/10) * resolution;
    result[1] = (pt_grid.y - IMAGE_H/4) * resolution;
    result[2] = 0;
    return result;
}
#endif