#ifndef _AREAGRIDMAP_H_
#define _AREAGRIDMAP_H_
#include "areagraph.h"
#include <cmath>
#include <unordered_map>
#include <iostream>
#include <queue>
#include<opencv2/opencv.hpp>
#include<opencv2/core/core.hpp>
#include "astarcal.h"


namespace osm_ag{
    void ComputeCenter_Area(AreaGraph& graph, AreaId area_id){
        // printf("start to compute\n");
        Eigen::Vector3d center_position(0.0, 0.0, 0.0);
        int num = 0;
        std::vector<NodeId> nodeids = graph.areas_[area_id]->nodes_inorder_;

        for(auto it = nodeids.begin(); it != nodeids.end()-1; it++){
            center_position += graph.nodes_[*it]->attributes_->position;
            num++;
        }
        if(num==0){
            printf("ERR:This area has no node!");
            graph.areas_[area_id]->init_center = false;
        }else{
            center_position /= num;
            graph.areas_[area_id]->init_center = true;
            graph.areas_[area_id]->center_position = center_position;
            // printf("the center x: %f", center_position[0]);
        }
        graph.areas_[area_id]->init_center = true;
    }

    //TODO : The transfomation????
    void InitOccupancyMap_Area(AreaGraph& graph, AreaId area_id, const double resolution = 0.2, const int offset = 4){
        // printf("start to initial grid map for every area\n");
        graph.areas_[area_id]->area_occupancy.info.resolution = resolution;
        if(!graph.areas_[area_id]->init_center){
            ComputeCenter_Area(graph, area_id);
        }
        Eigen::Vector3d center_position = graph.areas_[area_id]->center_position;
        graph.areas_[area_id]->area_occupancy.info.origin.position.x = center_position[0];
        graph.areas_[area_id]->area_occupancy.info.origin.position.y = center_position[1];
        graph.areas_[area_id]->area_occupancy.info.origin.position.z = center_position[2];
        graph.areas_[area_id]->area_occupancy.info.origin.orientation.y = 0.0;
        graph.areas_[area_id]->area_occupancy.info.origin.orientation.z = 0.0;
        graph.areas_[area_id]->area_occupancy.info.origin.orientation.w = 1.0;

        //size of area occupancy map
        // std::vector<Eigen::Vector3d> nodes_inorder_positions;
        double min_x = center_position[0];
        double max_x = center_position[0];
        double min_y = center_position[1];
        double max_y = center_position[1];
        std::vector<NodeId> nodeids = graph.areas_[area_id]->nodes_inorder_;
        for(auto it = nodeids.begin(); it != nodeids.end()-1; it++){
            // nodes_inorder_positions.push_back(graph.nodes_[*it]->attributes_->position);
            if(graph.nodes_[*it]->attributes_->position[0]>=max_x){
                max_x = graph.nodes_[*it]->attributes_->position[0];
            }
            if(graph.nodes_[*it]->attributes_->position[0]<=min_x){
                min_x = graph.nodes_[*it]->attributes_->position[0];
            }
            if(graph.nodes_[*it]->attributes_->position[1]>=max_y){
                max_y = graph.nodes_[*it]->attributes_->position[1];
            }
            if(graph.nodes_[*it]->attributes_->position[1]<=min_y){
                min_y = graph.nodes_[*it]->attributes_->position[1];
            }
        }
        int rows = std::ceil((center_position[1]-min_y)/resolution) + std::ceil((max_y - center_position[1])/resolution) + offset*2;
        int columns = std::ceil((center_position[0]-min_x)/resolution) + std::ceil((max_x - center_position[0])/resolution) + offset*2;

        int origin_row = std::ceil((center_position[1]-min_y)/resolution);
        int origin_column = std::ceil((center_position[0]-min_x)/resolution);

        cv::Mat area_mat(rows, columns, CV_8UC1, cv::Scalar(125));
        //首先画空闲区域
        int i = 0;
        cv::Point2i points_2d_last;
        // std::vector< cv::Point2i> contour_pts;
        std::vector<std::vector<cv::Point2i>> contour_pts(1,std::vector<cv::Point2i> ());
        NodeId node_id_last;
        for(auto node_id : nodeids){
            int x = (graph.nodes_[node_id]->attributes_->position[0] - center_position[0])/resolution + origin_column + offset;
            int y = (graph.nodes_[node_id]->attributes_->position[1] - center_position[1])/resolution + origin_row + offset;
            cv::Point2i points_2d(x,y);
            // printf("x: %d , y: %d ;  ", x,y);
            if(i>0){
                contour_pts[0].push_back(points_2d);
            }
            points_2d_last = points_2d;
            // node_id_last = node_id;
            i++;
        }
        // cv::fillPoly(area_mat, contour_pts, cv::Scalar(255));//可能area的边刚好不在白色区域中?
        cv::drawContours(area_mat,contour_pts,-1,cv::Scalar(255),CV_FILLED);//空闲区域是白色的


        i=0;
        for(auto node_id : nodeids){
            int x = (graph.nodes_[node_id]->attributes_->position[0] - center_position[0])/resolution + origin_column + offset;
            int y = (graph.nodes_[node_id]->attributes_->position[1] - center_position[1])/resolution + origin_row + offset;
            cv::Point2i points_2d(x,y);
            if(i>0){
                std::pair<NodeId,NodeId> p(node_id,node_id_last);
                std::pair<NodeId,NodeId> p_i(node_id_last,node_id);
                if( !graph.areas_[area_id]->passage_nodes.count(p) && !graph.areas_[area_id]->passage_nodes.count(p_i)){ //not belong the passage
                    cv::line(area_mat, points_2d_last, points_2d, cv::Scalar(0), 2);//TODO: DEBUG!!!!有的passage被错误地添加为了墙壁
                }
            }
            points_2d_last = points_2d;
            node_id_last = node_id;
            i++;
        }

        // i=0;
        // for(auto node_id : nodeids){
        //     int x = (graph.nodes_[node_id]->attributes_->position[0] - center_position[0])/resolution + origin_column + offset;
        //     int y = (graph.nodes_[node_id]->attributes_->position[1] - center_position[1])/resolution + origin_row + offset;
        //     cv::Point2i points_2d(x,y);
        //     if(i>0){
        //         std::pair<NodeId,NodeId> p(node_id,node_id_last);
        //         std::pair<NodeId,NodeId> p_i(node_id_last,node_id);
        //         if(graph.areas_[area_id]->passage_nodes.count(p) || graph.areas_[area_id]->passage_nodes.count(p_i)){ //not belong the passage
        //             cv::line(area_mat, points_2d_last, points_2d, cv::Scalar(255), 2);
        //         }
        //     }
        //     points_2d_last = points_2d;
        //     node_id_last = node_id;
        //     i++;
        // }

        for(auto passage : graph.areas_[area_id]->passageids){
            NodeId id_i = graph.passages_[passage]->passage_nodes.source;
            NodeId id_j = graph.passages_[passage]->passage_nodes.target;
            Eigen::Vector3d pi = graph.nodes_[id_i]->attributes_->position;
            Eigen::Vector3d pj = graph.nodes_[id_j]->attributes_->position;
            Eigen::Vector3d pm = (pi+pj)/2.0;
            int x_i = (pi[0] - center_position[0])/resolution + origin_column + offset;
            int y_i = (pi[1] - center_position[1])/resolution + origin_row + offset;
            int x_j = (pj[0] - center_position[0])/resolution + origin_column + offset;
            int y_j = (pj[1] - center_position[1])/resolution + origin_row + offset;
            int x_m = (pm[0] - center_position[0])/resolution + origin_column + offset;
            int y_m = (pm[1] - center_position[1])/resolution + origin_row + offset;
            cv::Point2i point_i(x_i,y_i);
            cv::Point2i point_j(x_j,y_j);
            cv::Point2i point_m(x_m,y_m);

            cv::line(area_mat, point_i, point_j, cv::Scalar(255), 2);
            cv::circle(area_mat,point_m,4,cv::Scalar(255),CV_FILLED);

        }


        graph.areas_[area_id]->grid_mat_.grid_mat = area_mat.clone();//just for temp
        graph.areas_[area_id]->grid_mat_.origin_grid_x = origin_column;
        graph.areas_[area_id]->grid_mat_.origin_grid_y = origin_row;
        graph.areas_[area_id]->grid_mat_.offset = offset;
        graph.areas_[area_id]->grid_mat_.resolution = resolution;
        std::string mat_name = "show_area_" + std::to_string(area_id) + ".png";
        // cv::imwrite(mat_name,area_mat);//TODO:!!! DEBUG 为什么注释掉这个会影响距离的选取？？？？

        graph.areas_[area_id]->area_occupancy.info.width = columns;
        graph.areas_[area_id]->area_occupancy.info.height = rows;
        graph.areas_[area_id]->area_occupancy.data.resize(columns * rows);

        for(int i=0; i< columns; i++){
            for(int j=0; j<rows; j++){
                if(area_mat.at<uchar>(j,i) == 125){
                    graph.areas_[area_id]->area_occupancy.data[j*columns+i] = -1;//unkown
                }else if(area_mat.at<uchar>(j,i) == 255){
                    graph.areas_[area_id]->area_occupancy.data[j*columns+i] = 0;//free white
                }else if(area_mat.at<uchar>(j,i) == 0){
                    graph.areas_[area_id]->area_occupancy.data[j*columns+i] = 100;//occupied black
                }
            }
        }

        graph.areas_[area_id]->use_occupancy_map=true;

    }
}

#endif