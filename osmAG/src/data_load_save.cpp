#include <iostream>
#include <string.h>
#include <vector>
#include <map>
#include <math.h>
#include <array>
#include <ros/ros.h>

// #include "node.h"
// #include "areagraph.h"
// #include "../include/pathgraph.h"
#include "../include/data_load_save.h"


#include "nav_msgs/GetMap.h"
#include <time.h>


using namespace tinyxml2;
using namespace osm_ag;

static double EARTH_RADIUS = 6378.137;
static double EARTH_SHORT_RADIUS = 6356.75231424518;

double Deg2Rad(double d){
    return d * M_PI / 180.0;
}

/**
* @brief form WGS84(GPS) coordinate to XYZ coordinate
*
**/
Eigen::Vector3d CoordinateTransform(GeoCoordinate init, GeoCoordinate cur){
    std::array<double, 2> reference{init.latitude, init.longtitude};
    std::array<double, 2> currrent{cur.latitude, cur.longtitude};
    std::array<double, 2> cur_xy;

    cur_xy = wgs84::toCartesian(reference, currrent);

    double z = cur.altitude - init.altitude;

    return {cur_xy[0],cur_xy[1],z};
}

double ComputPathDistance(vector<Eigen::Vector3d>& path_pts){
    Eigen::Vector3d point_last;
    int i = 0;
    double result_dist = 0;
    for(Eigen::Vector3d pt:path_pts){
        if(i>0){
            result_dist += (pt - point_last).norm();
        }
        point_last = pt;
        i++;
    }
    return result_dist;
}
// void TraverseArea(AreaGraph& graph){
//     for(auto it = graph.areas_.begin(); it!= graph.areas_.end(); it++){
//         for(auto itt = it->second->nodes_inorder_.begin(); itt!=it->second->nodes_inorder_.end(); itt++){
//             if(graph.nodes_.find(*itt)!= graph.nodes_.end()){
//                 if(it->second->nodes_.find(*itt)==it->second->nodes_.end()){
//                     it->second->nodes_[*itt] = graph.nodes_[*itt].get();
//                 }
//                 printf("node_id test:%ld\n",it->second->nodes_[*itt]->id);
//             }else{
//                 printf("Graph is not complete!!!!!!");
//             }
//         }
//     }
// }

// void TraversePassage(AreaGraph& graph){
//     for(auto it = graph.passages_.begin(); it!= graph.passages_.end(); it++){
//         auto idx1 = it->second->passage_nodes.source;
//         auto idx2 = it->second->passage_nodes.target;
//         if(graph.nodes_.find(idx1)!= graph.nodes_.end()){
//             if(graph.nodes_[idx1]->passage_belongto_.find(it->first)!=graph.nodes_[idx1]->passage_belongto_.end()){
//                 graph.nodes_[idx1]->passage_belongto_[it->first] = std::move(it->second);
//             }
//         }else{
//             printf("Graph is not complete!!!!!!");
//         }

//         if(graph.nodes_.find(idx2)!= graph.nodes_.end()){
//             if(graph.nodes_[idx2]->passage_belongto_.find(it->first)!=graph.nodes_[idx2]->passage_belongto_.end()){
//                 graph.nodes_[idx2]->passage_belongto_[it->first] = std::move(it->second);
//             }
//         }else{
//             printf("Graph is not complete!!!!!!");
//         }

//     }
// }


void TraverseArea(AreaGraph& graph){
    printf("areas number: %ld\n", graph.areas_.size());
    for(auto it = graph.areas_.begin(); it!= graph.areas_.end(); it++){
        //  printf("nodes number of this area %ld;    \n", it->second->nodes_inorder_.size());//area_id
        //  //test passage_trees
        //  if(it->second->type == "structure"){
        //     printf("the passage children numbers:%d\n", graph.passage_trees[it->first].size());
        //  }
        for(auto itt = it->second->nodes_inorder_.begin(); itt != (it->second->nodes_inorder_.end()-1); itt++){
            if(graph.nodes_.find(*itt)!= graph.nodes_.end()){
                // it->second->nodes_[*itt] = graph.nodes_[*itt];
                if(graph.nodes_[*itt]->area_belongto_.count(it->first)==0){
                    // printf("belong %ld;    ", it->first);//area_id
                    graph.nodes_[*itt]->area_belongto_.insert(it->first);
                }
                // if(it->second->nodes_.find(*itt)==it->second->nodes_.end()){
                //    it->second->nodes_[*itt] = std::move(graph.nodes_[*itt]);//node 的unique_ptr 不能转移所有权给area里的nodes 全部放在了graph当中
                // }
                // printf("node_id test:%ld\n",it->second->nodes_[*itt]->id);

                // if(it->second->nodes_.find(*itt)==it->second->nodes_.end()){
                //     it->second->nodes_[*itt] = graph.nodes_[*itt].get();//但是尽量不要get，这是个裸指针，不然为什么要用unique
                // }
            }else{
                printf("Graph is not complete!!!!!!");
            }
        }
    }
}

void TraversePassage(AreaGraph& graph){
    printf("Taversing Passage\n");
    for(auto it = graph.passages_.begin(); it!= graph.passages_.end(); it++){
        auto from = it->second->area_from;
        auto to = it->second->area_to;
        if(graph.areas_.find(from)!= graph.areas_.end()){
            graph.areas_[from]->passageids.insert(it->first);
            graph.areas_[from]->passage_nodes.insert(std::make_pair(it->second->passage_nodes.source,it->second->passage_nodes.target));
            if(graph.areas_[from]->hasParent()){
                AreaId parent_id_from = graph.areas_[from]->parent_;
                graph.passage_trees[parent_id_from].insert(it->first);
            }
                
        }else{
            printf("Graph Parsing is not complete!!!!!!");
            printf("areaid is %ld\n", from);
        }

        if(graph.areas_.find(to)!= graph.areas_.end()){
            graph.areas_[to]->passageids.insert(it->first);
            graph.areas_[to]->passage_nodes.insert(std::make_pair(it->second->passage_nodes.source,it->second->passage_nodes.target));
            if(graph.areas_[to]->hasParent()){
                AreaId parent_id_to = graph.areas_[to]->parent_;
                graph.passage_trees[parent_id_to].insert(it->first);
            }
                
        }else{
            printf("Graph Parsing is not complete!!!!!!");
            printf("areaid is %ld\n", to);

        }

        auto idx1 = it->second->passage_nodes.source;
        auto idx2 = it->second->passage_nodes.target;
        Eigen::Vector3d center;
        if(graph.nodes_.find(idx1)!= graph.nodes_.end()){
            center=graph.nodes_[idx1]->attributes_->position;
            if(graph.nodes_[idx1]->passage_belongto_.find(it->first)==graph.nodes_[idx1]->passage_belongto_.end()){
                graph.nodes_[idx1]->passage_belongto_.insert(it->first);
            }
        }else{
            printf("Graph is not complete!!!!!!");
            printf("nodeid is %ld\n", idx1);

        }

        if(graph.nodes_.find(idx2)!= graph.nodes_.end()){
            center += graph.nodes_[idx2]->attributes_->position;

            if(graph.nodes_[idx2]->passage_belongto_.find(it->first)==graph.nodes_[idx2]->passage_belongto_.end()){
                graph.nodes_[idx2]->passage_belongto_.insert(it->first);
            }
        }else{
            printf("Graph is not complete!!!!!!");
            printf("nodeid is %ld\n", idx2);
        }
        // printf("center of this passage %f\n",center[0]);

        center = center/2.0;
        it->second->center_position = center;
    }
    printf("Travering pass success!!!!\n");
}

void PlanInStructure(AreaGraph& graph, AreaId parent_areaid, PassageId start_id, PassageId end_id){
    printf("Start to build the PathGraph for each layer!!!\n");
    if(graph.area_trees.find(parent_areaid)==graph.area_trees.end()){
        printf("This is not an correct Structure!!!!!!!\n");
        return;
    }else{
        printf("parent_area is %ld\n",graph.areas_[parent_areaid]->id);
    }

    // if(graph.areas_.find(-125356)==graph.areas_.end()){
    //     printf("4without this areaid\n");
    // }else{
    //     printf("4we have this id\n");
    // }

    PathLayer layer1(graph,parent_areaid);
    printf("Success Build the PathGraph for each layer!!!\n");

    // if(graph.areas_.find(-125356)==graph.areas_.end()){
    //     printf("without this areaid\n");
    // }else{
    //     printf("we have this id\n");
    // }

    if(layer1.pathnodes.find(start_id)==layer1.pathnodes.end()){
        printf("This layer doesn't include this start passage!!!\n");
        return;
    }
    if(layer1.pathnodes.find(end_id)==layer1.pathnodes.end()){
        printf("This layer doesn't include this end passage!!!\n");
        return;
    }
    layer1.AddEdge_AStar(start_id,end_id);

    printf("The shortest path from %ld to %ld is : ", start_id, end_id);
    auto p = layer1.paths[0]->path;   
    // for(auto pp=p.rbegin(); pp !=p.rend(); ++pp){
    //     printf("%ld -> ",*pp);
    // }
    for(auto pp=p.rbegin(); pp !=p.rend(); ++pp){
        printf("%s -> ",graph.passages_[*pp]->info->tags["name"].c_str());
    }
    printf("\n");
}

void PlanInPathGraph(AreaGraph& graph, PassageId start_id, PassageId end_id, std::vector<Eigen::Vector3d>& path_result){
    //First we need to judge if the start and end points exist.
    if(graph.passages_.find(start_id) == graph.passages_.end()){
        printf("ERROR: The start point id is not in this graph!!!");
        return;
    }
    if(graph.passages_.find(end_id) == graph.passages_.end()){
        printf("ERROR: The end point id is not in this graph!!!");
        return;
    }
    PassageGraph passage_graph;
    PathGraph path_graphs(graph,passage_graph);

    timeval start_time, stop_time;//us级别
    gettimeofday(&start_time, NULL);

    path_graphs.AddEdge_AStar(start_id, end_id, ASTAR);

    gettimeofday(&stop_time, NULL);
    long elapsed_us = (stop_time.tv_sec - start_time.tv_sec)*1000000 + (stop_time.tv_usec - start_time.tv_usec);
    long elapaed_ms = (stop_time.tv_sec - start_time.tv_sec)*1000 + (stop_time.tv_usec - start_time.tv_usec)/1000;
    printf("The time using OSM-AG: %ld us.\n", elapsed_us);

    printf("The shortest path from %ld to %ld is : ", start_id, end_id);
    auto p = path_graphs.paths[0]->path;
    for(auto pp=p.rbegin(); pp !=p.rend(); ++pp){
        printf("%s -> ",graph.passages_[*pp]->info->tags["name"].c_str());
    }
    printf("\n");

    // std::vector<Eigen::Vector3d> path_result;
    std::vector<Eigen::Vector3d> path_temp;

    // geometry_msgs::PoseStamped path_res; 
    // path_res.header.frame_id

    //Transform  topo to metric path
    if(p.size()>1){
        std::reverse(p.begin(), p.end());
        PassageId from = p[0];
        for(auto it = p.begin()+1; it != p.end(); it++){
            std::pair<PassageId, PassageId> edge_(std::make_pair(from, *it));
            if(passage_graph.pathedges_trj.find(edge_) != passage_graph.pathedges_trj.end()){
                path_temp = passage_graph.pathedges_trj[edge_];
                // printf("The size of current path edge: %d;    ", path_temp.size());
                path_result.insert(path_result.end(),path_temp.begin(),path_temp.end());
            }else{
                printf("ERROR!!!!!!Cannot output the metric path!!!\n");
                break;
            }
            from = *it;
        }

        double dist_path = ComputPathDistance(path_result);
        printf("The path distance using OSM-AG: %f m.\n", dist_path);
    }

    /*如果是用L2或者L1进行的计算，这里需要拼接A*的； 当前只是为了计算耗时，因为计算path_graph的时候就已经算过了这一部分
      之后需要进行修改
    */
    if(p.size()>1){
        //前面的p已经反转过了
        timeval start_time_test, stop_time_test;//us级别
        long elapsed_us_test = 0;
        int count = 1;

        PassageId last = p[0];
        for(auto it = p.begin()+1; it != p.end(); it++){
            //首先去找这两个passage在哪一个area当中
            AreaId common_area = 0;
            AreaId last_area_1 = graph.passages_[last]->area_from;
            AreaId last_area_2 = graph.passages_[last]->area_to;
            AreaId current_area_1 = graph.passages_[*it]->area_from;
            AreaId current_area_2 = graph.passages_[*it]->area_to;

            if(last_area_1 == current_area_1 | last_area_1 == current_area_2){
                common_area = last_area_1;
            }
            if(last_area_2 == current_area_1 | last_area_2 == current_area_2){
                common_area = last_area_2;
            }
            if(common_area==0){
                printf("CANNOT find the common area!!!\n");//一般不会出现这种情况。
                return;
            }
            Eigen::Vector3d center_position = graph.areas_[common_area]->center_position;

            GridMat grid_mat_ = graph.areas_[common_area]->grid_mat_;

            Eigen::Vector3d pi = graph.passages_[last]->center_position;
            Eigen::Vector3d pj = graph.passages_[*it]->center_position;
            int x_i = (pi[0] - center_position[0])/grid_mat_.resolution + grid_mat_.origin_grid_x + grid_mat_.offset;
            int y_i = (pi[1] - center_position[1])/grid_mat_.resolution + grid_mat_.origin_grid_y + grid_mat_.offset;
            int x_j = (pj[0] - center_position[0])/grid_mat_.resolution + grid_mat_.origin_grid_x + grid_mat_.offset;
            int y_j = (pj[1] - center_position[1])/grid_mat_.resolution + grid_mat_.origin_grid_y + grid_mat_.offset;
            cv::Point2i point_i(x_i,y_i);
            cv::Point2i point_j(x_j,y_j);
            double res_dis;
            vector<int> dis_result;
	        std::vector<std::vector<int>> sites;
            for (size_t col = 0; col < grid_mat_.grid_mat.cols; col++) {
		        std::vector<int> rows;
		        for (size_t row = 0; row < grid_mat_.grid_mat.rows; row++) {
			        int color = grid_mat_.grid_mat.at<uchar>(row, col);
			        rows.push_back(color == 255 ? 0 : 1);//白色的点是障碍点？？？
		        }
		        sites.push_back(rows);
	        }
            AStarCalc calc = AStarCalc();
            calc.InitSites(sites);
            CalcPt startpt = CalcPt(point_i);
	        CalcPt endpt = CalcPt(point_j);

            gettimeofday(&start_time_test, NULL);

	        list<CalcPt* > reslist = calc.GetPath(startpt, endpt, dis_result);

            gettimeofday(&stop_time_test, NULL);

            elapsed_us_test += (stop_time_test.tv_sec - start_time_test.tv_sec)*1000000 + (stop_time_test.tv_usec - start_time_test.tv_usec);

            last = *it;
            printf("%d...", count);
            count++;
        }
        printf("\nThe A* duration in OSM-AG: %ld us.\n", elapsed_us_test);
    }
}


//for debug
void PlanInPathGraph(AreaGraph& graph, PassageId start_id, PassageId end_id, std::vector<std::vector<Eigen::Vector3d>>& path_result){
    //First we need to judge if the start and end points exist.
    if(graph.passages_.find(start_id) == graph.passages_.end()){
        printf("ERROR: The start point id is not in this graph!!!");
        return;
    }
    if(graph.passages_.find(end_id) == graph.passages_.end()){
        printf("ERROR: The end point id is not in this graph!!!");
        return;
    }
    PassageGraph passage_graph;
    PathGraph path_graphs(graph,passage_graph);
    timeval start_time, stop_time;//us级别
    gettimeofday(&start_time, NULL);
    path_graphs.AddEdge_AStar(start_id,end_id);
    gettimeofday(&stop_time, NULL);
    long elapsed_us = (stop_time.tv_sec - start_time.tv_sec)*1000000 + (stop_time.tv_usec - start_time.tv_usec);
    long elapaed_ms = (stop_time.tv_sec - start_time.tv_sec)*1000 + (stop_time.tv_usec - start_time.tv_usec)/1000;
    printf("The time using OSM-AG: %ld us.\n", elapsed_us);
    printf("The shortest path from %ld to %ld is : ", start_id, end_id);
    auto p = path_graphs.paths[0]->path;
    for(auto pp=p.rbegin(); pp !=p.rend(); ++pp){
        printf("%s -> ",graph.passages_[*pp]->info->tags["name"].c_str());
    }
    printf("\n");

    // std::vector<Eigen::Vector3d> path_result;
    std::vector<Eigen::Vector3d> path_temp;

    // geometry_msgs::PoseStamped path_res; 
    // path_res.header.frame_id

    //Transform  topo to metric path
    if(p.size()>1){
        std::reverse(p.begin(), p.end());
        PassageId from = p[0];
        for(auto it = p.begin()+1; it != p.end(); it++){
            std::pair<PassageId, PassageId> edge_(std::make_pair(from, *it));
            if(passage_graph.pathedges_trj.find(edge_) != passage_graph.pathedges_trj.end()){
                path_temp = passage_graph.pathedges_trj[edge_];
                // printf("The size of current path edge: %d;    ", path_temp.size());
                path_result.push_back(path_temp);
            }else{
                printf("ERROR!!!!!!Cannot output the metric path!!!\n");
                break;
            }
            from = *it;
        }

    }
}

/**
 * @brief just compare to 2d Grid map using A*
 * @param graph which the file parse to
 **/
void PlanInGridMap(AreaGraph& graph, PassageId start_id, PassageId end_id, cv::Mat& grid_map_show, double resolution, bool if_show){
    //built gird map for one layer. 这里暂时不考虑level，不对多层做处理
    //注意只能对底层叶节点进行！！！
    cv::Mat gridmap_(cv::Size(IMAGE_W,IMAGE_H), CV_8UC1, cv::Scalar(125));
    cv::Mat map_color_show(cv::Size(IMAGE_W,IMAGE_H), CV_8UC3);
    cv::cvtColor(gridmap_,map_color_show,cv::COLOR_GRAY2BGR);



    PlotAreasInGrid(graph, gridmap_, resolution);
    PlotPassagesInGrid(graph, gridmap_, resolution);

    cv::imwrite("show_gridmap_layer_ori.png",gridmap_);

    //First we need to judge if the start and end points exist.
    if(graph.passages_.find(start_id) == graph.passages_.end()){
        printf("ERROR: The start point id is not in this graph!!!");
        return;
    }
    if(graph.passages_.find(end_id) == graph.passages_.end()){
        printf("ERROR: The end point id is not in this graph!!!");
        return;
    }
    Eigen::Vector3d pos_s, pos_e;
    pos_s = graph.passages_[start_id]->center_position;
    pos_e = graph.passages_[end_id]->center_position;
    cv::Point2i pt_s, pt_e;
    pt_s = XYZ2Grid(pos_s);
    pt_e = XYZ2Grid(pos_e);
    // printf("the start point in grid:%d, %d\n",pt_s.x, pt_s.y);


    // cv::line(temp,pt_s,pt_e,cv::Scalar(0,0,255),4);
    // cv::circle(temp, cv::Point(pt_s.x, pt_s.y),30,cv::Scalar(0,0,255));
    // cv::imshow("temp",temp);
    // cv::waitKey();
    // cv::destroyAllWindows();
    // cv::imwrite("show_gridmap_layer_ori2.png",temp);


    double res_dis = 0.0;
    std::vector<Eigen::Vector3d> res_xyz;
    std::vector<cv::Point2i> resPoints;//注意这里是倒序的点，终点到起点。
    std::vector<int> dis_result;
	std::vector<std::vector<int>> sites;
    for (size_t col = 0; col < gridmap_.cols; col++) {
		std::vector<int> rows;
		for (size_t row = 0; row < gridmap_.rows; row++) {
			int color = gridmap_.at<uchar>(row, col);
			rows.push_back(color == 255 ? 0 : 1);
		}
		sites.push_back(rows);
	}
    AStarCalc calc = AStarCalc();
    calc.InitSites(sites);
    CalcPt startpt = CalcPt(pt_s);
	CalcPt endpt = CalcPt(pt_e);

    timeval start_time, stop_time;//us级别
    gettimeofday(&start_time, NULL);

	list<CalcPt* > reslist = calc.GetPath(startpt, endpt, dis_result);
    gettimeofday(&stop_time, NULL);
    long elapsed_us = (stop_time.tv_sec - start_time.tv_sec)*1000000 + (stop_time.tv_usec - start_time.tv_usec);
    long elapaed_ms = (stop_time.tv_sec - start_time.tv_sec)*1000 + (stop_time.tv_usec - start_time.tv_usec)/1000;
    printf("The time using grid_map: %ld us.\n", elapsed_us);
	for (auto p : reslist) {
		resPoints.push_back(p->pt);//注意这里是倒序的点，终点到起点。
	}
    cv::Mat srccpy = gridmap_.clone();
    // cv::namedWindow("path_in_one_area");
    // cv::resizeWindow("path_in_one_area",400,300);
    cv::circle(srccpy, pt_s, 4, cv::Scalar(75));
    cv::circle(srccpy, pt_e, 4, cv::Scalar(75));

    //test the performance of a* in grid map
    if(resPoints.size()>0){
        reverse(resPoints.begin(), resPoints.end());
        if(if_show){
            // cv::Mat srccpy = grids.grid_mat.clone();
            res_xyz.push_back(Grid2XYZ_vis(resPoints[0]));

            for(int j=0; j<resPoints.size()-1; j++){
                int k = j+1;
                res_xyz.push_back(Grid2XYZ_vis(resPoints[k]));
                cv::line(srccpy, resPoints[j], resPoints[k], cv::Scalar(200),1);
                cv::line(grid_map_show, resPoints[j], resPoints[k], cv::Scalar(255,0,0),2);
            }
            // cv::imwrite("path_in_one_area.png", srccpy);
            // // cv::imshow("path_in_one_area.png", srccpy);
	        // // cv::waitKey(0);

        }
        for(auto dis_: dis_result){
            res_dis += dis_;
        }
        res_dis /= 10.0 ;
        double dist_path = ComputPathDistance(res_xyz);
        printf("The path distance using grid_map: %f m; the appro dist: %f m.\n", dist_path, res_dis);
             
    }else{
        printf("ERR: CANNOT ARRIVE!!!\n");
        // cv::imshow("path_in_one_area.png", srccpy);
        // cv::imwrite("path_in_one_area_wrong.png", srccpy);

        // cv::waitKey(0);
        // cv::destroyAllWindows();
        res_dis = std::numeric_limits<double>::max();
    }

    cv::imwrite("show_gridmap_layer.png",srccpy);
    cv::imwrite("show_gridmap_layer_compare.png",grid_map_show);


}




/**
 * @brief This function parses .osm(xml format) file to our area graph.
 * @param graph which the file parse to
 * please attention!!!!当前版本的osm要求把area放在passage的前面？？？需要保证所有的areaid已经出现了。因为在parse的阶段有查询graph.areas_[from]->hasParent()
 * 但是这一部分可以放在traverse当中进行？？？TODO!!!!
 **/
bool Parsing_Osm2AreaGraph(AreaGraph& graph, const char* file_path){

    bool sucess_parse = true;

    /*read and load .osm file using the path*/
    XMLDocument doc;

    XMLError eResult = doc.LoadFile(file_path);
    if(eResult != XML_SUCCESS){
        printf("Cannot loda the OSMAG file!!!\n");
    }else{
        printf("Open the OSM file SUCCESSFULLY!\n");
    }
    XMLNode * osm = doc.FirstChildElement("osm");

    // for each xml element child in node :
    //   check if it is one that we 
    //   ELSE:
    //   put the xml text of that node in the string "xml_leftovers"
    std::vector<XMLElement*> xml_left_elements;//like relations

    int counts_area = 0;
    int counts_passage = 0;




    //Please attention : FirstChildElement("node") or FirstChildElement("way") will not find the previous siblingElement 所以要选第一个出现的， 需要改进 还要查找之前的 
    for(XMLElement* xml_node = osm-> FirstChildElement(); xml_node!=NULL ; xml_node = xml_node -> NextSiblingElement()){
        if(std::string(xml_node->Value())== "node"){
            NodeId node_id = atol(xml_node->Attribute("id"));
            // printf("nodeid : %ld\n", node_id);
            NodeAttributes attrs;
            attrs.geo_position.latitude = atof(xml_node->Attribute("lat"));
            attrs.geo_position.longtitude = atof(xml_node->Attribute("lon"));
            attrs.action = std::string(xml_node->Attribute("action"));
            attrs.visible = std::string(xml_node->Attribute("visible"))== "true" ?  true: false;

            /*we should confirm origin node is read at first!!!!!!*/
            if(!graph.initial && std::string(xml_node->Attribute("root"))=="true"){
                attrs.geo_position.altitude = atof(xml_node->Attribute("alt"));
                Eigen::Vector3d rot = {atof(xml_node->Attribute("qx")),atof(xml_node->Attribute("qy")),atof(xml_node->Attribute("qz"))};
                OriginNode::Ptr origin_ptr = std::make_unique<OriginNode>(node_id, rot, attrs.clone());
                graph.initial = true;
                graph.origin_ = std::make_pair(node_id, std::move(origin_ptr));
                //TODO: 现在不能把originnode加到Nodes里面，或者不要作为originnode 再加入一次
                printf("Checked if we have read origin node: %d\n", true);
                // continue;
            }

            if(graph.initial){
                GeoCoordinate init = graph.origin_.second->getAttributesPtr()->geo_position;
                attrs.position = CoordinateTransform(init, attrs.geo_position);
                // Node node(node_id, attrs.clone());
                Node::Ptr node_ptr = std::make_unique<Node>(node_id, attrs.clone());
                // printf("nodes:%ld\n", node_ptr->id);
                graph.AddNodeVertex(std::move(node_ptr));
                graph.min_node_id = std::min(graph.min_node_id, node_id);
                graph.max_node_id = std::max(graph.max_node_id, node_id);
            }

        }else if(std::string(xml_node->Value())== "way"){
            //先由areatype判断是area还是passage  <tag k='osmAG:type' v='passage' />
            WayAttributes way_attrs;
            std::map<std::string, std::string> tags_;
            std::vector<NodeId> nodes_inorder_;

            uint64_t way_id = atol(xml_node->Attribute("id"));

            // IF DEBUG : THIS IS IMPORTANT!!!!!!
            // printf("way id:%ld\n", way_id);


            way_attrs.action = std::string(xml_node->Attribute("action"));
            way_attrs.visible = std::string(xml_node->Attribute("visible"))== "true" ?  true: false;


            //Please attention : FirstChildElement("nd") or FirstChildElement("tag") will not find the previous siblingElement 所以要选第一个出现的， 需要改进 还要查找之前的 

            // xml_node->PreviousSiblingElement()

            //所以默认用0


            for(XMLElement* way_tags = xml_node -> FirstChildElement(); way_tags!= NULL; way_tags = way_tags -> NextSiblingElement()){
                /*
                    At first, save all tags and ref nodes of the way and then check
                */
                if(std::string(way_tags->Value())=="tag"){
                    std::string key_(way_tags->Attribute("k"));
                    std::string value_(way_tags->Attribute("v"));
                    tags_[key_] = value_;
                }else if(std::string(way_tags->Value())=="nd"){
                    //对于area来说，并不一定node全部读取完了，所以nodeptr可能还有点问题，可以用nodeid
                    //而且还要考虑如果nodeid产生了变化的话 因为area是有顺序的，所以读取的时候需要注意
                    nodes_inorder_.emplace_back(atol(way_tags->Attribute("ref")));
                }
            }

            if(tags_["osmAG:type"]=="area"){
                std::string areatype = tags_["osmAG:areaType"];
                // printf("areatype:%s\n",tags_["osmAG:areaType"].c_str());
                AreaId parent_id;
                bool has_parent = false; 
                if(tags_.find("osmAG:parent") != tags_.end()){
                    has_parent = true;
                    parent_id = stol(tags_["osmAG:parent"]);
                    tags_.erase("osmAG:parent");
                }
                tags_.erase("osmAG:areaType");
                tags_.erase("osmAG:type");//erase the explicit tags, the left are semantic tags like "name" etc.

                way_attrs.tags = tags_;
                // printf("area left tags size:%d\n", tags_.size());
                
                // if(way_id == -125356){//for test
                //     printf("It has this area!!!!!!\n");
                // }

                // Area area(way_id, way_attrs.clone(),areatype);
                Area::Ptr area_ptr = std::make_unique<Area>(way_id, way_attrs.clone(), areatype);

                if(nodes_inorder_.back()!=nodes_inorder_.front()){
                    printf("This is not an area!! area id is:%ld", way_id);
                    sucess_parse = false;
                    return sucess_parse;
                }

                //we need check the order of area nodes(points)
                assert(nodes_inorder_.back()==nodes_inorder_.front());
                area_ptr->nodes_inorder_ = nodes_inorder_;

                if(has_parent){
                    area_ptr->setparent(parent_id);
                    // if(graph.area_trees.find(parent_id)!= graph.area_trees.end()){
                        graph.area_trees[parent_id].insert(way_id);
                }
                // printf("if area has parent:%d\n", area_ptr->has_parent_);
                graph.AddArea(std::move(area_ptr));

            }else if(tags_["osmAG:type"]=="passage"){
                PassageNodes nodes_id;
                if(nodes_inorder_.size()!=2){
                    printf("This is not a passage!! way id is:%ld", way_id);
                    sucess_parse = false;
                    return sucess_parse;
                }
                assert(nodes_inorder_.size()==2);
                nodes_id.source = nodes_inorder_[0];
                nodes_id.target = nodes_inorder_[1];
                AreaId from = stol(tags_["osmAG:from"]);
                AreaId to = stol(tags_["osmAG:to"]);//check the node if is exist eg. wayid=0;
                if(from < 0 && to < 0){//测试阶段有不正确的area的id！！！！！！
                if(graph.areas_.find(from)==graph.areas_.end()){
                    printf("This areaID which passage FROM is wrong!! way id is:%ld, from id is: %ld. ", way_id, from);
                    sucess_parse = false;
                    return sucess_parse;
                }
                if(graph.areas_[from]->hasParent()){
                    AreaId parent_id_from = graph.areas_[from]->parent_;
                    graph.passage_trees[parent_id_from].insert(way_id);
                }
                if(graph.areas_.find(to)==graph.areas_.end()){
                    printf("This areaID which passage TO is wrong!! way id is:%ld, from id is: %ld. ", way_id, to);
                    sucess_parse = false;
                    return sucess_parse;
                }
                if(graph.areas_[to]->hasParent()){
                    AreaId parent_id_to = graph.areas_[to]->parent_;
                    graph.passage_trees[parent_id_to].insert(way_id);
                }
                }

                tags_.erase("osmAG:from");
                tags_.erase("osmAG:to");
                tags_.erase("osmAG:type");//erase the explicit tags

                // printf("passage left tags size:%d\n", tags_.size());

                way_attrs.tags = tags_;

                Passage::Ptr passage_ptr = std::make_unique<Passage>(way_id, nodes_id, way_attrs.clone());

                passage_ptr->area_from = from;
                passage_ptr->area_to = to;

                // if(!passage_ptr->getAttributesPtr()->tags.empty()){
                //     printf("passage other tags size:%d\n", passage_ptr-> info ->tags.size());
                // }

                graph.AddPassage(std::move(passage_ptr));
            }
            
        }else{
            xml_left_elements.push_back(xml_node);
        }

    }


    return sucess_parse;
}


void Save_AreaGraph2Osm(const AreaGraph& graph, const char* file_path){
    XMLDocument doc;
    /*add a declartion for xml* version="1.0" encoding=UTF-8*/
    XMLDeclaration* declar = doc.NewDeclaration();
    doc.InsertFirstChild(declar);
    /*creat root node*/
    XMLElement* root = doc.NewElement("osm");
    root->SetAttribute("version","0.6");
    root->SetAttribute("generator","AreaGraph");
    doc.InsertEndChild(root);
    
    for(auto it = graph.nodes_.begin(); it!= graph.nodes_.end(); it++){
        XMLElement* xml_node = doc.NewElement("node");
        xml_node->SetAttribute("id", it->first);
        auto attr = it->second->getAttributesPtr();
        xml_node->SetAttribute("action", attr->action.c_str());
        const char * vis = attr->visible ? "true": "false";
        xml_node->SetAttribute("visible", vis);
        xml_node->SetAttribute("lat", attr->geo_position.latitude);
        xml_node->SetAttribute("lon", attr->geo_position.longtitude);

        /*test the xyz transform*/
        xml_node->SetAttribute("x", attr->position[0]);
        xml_node->SetAttribute("y", attr->position[1]);
        
        // printf("node ID = %ld\n", it->first);
        // if(!it->second->passage_belongto_.empty()){
        //     printf("belong passage%ld\n",*(it->second->passage_belongto_.begin()));
        // }

        // if(!it->second->area_belongto_.empty()){
        //     printf("belong area%ld\n",*(it->second->area_belongto_.begin()));
        // }


        root->InsertEndChild(xml_node);
    }
    
    for(auto it = graph.areas_.begin(); it!= graph.areas_.end(); it++){
        XMLElement* xml_node = doc.NewElement("way");
        
        xml_node->SetAttribute("id", it->first);
        auto attr = it->second->getAttributesPtr();
        xml_node->SetAttribute("action", attr->action.c_str());
        const char * vis = attr->visible ? "true": "false";
        xml_node->SetAttribute("visible", vis);

        for(auto ref_it = it->second->nodes_inorder_.begin(); ref_it != it->second->nodes_inorder_.end(); ref_it++){
            XMLElement* xml_ref = doc.NewElement("nd");
            xml_ref->SetAttribute("ref", *ref_it);/*attention int inversion!!!*/
            xml_node->InsertEndChild(xml_ref);
        }
        XMLElement* xml_tag = doc.NewElement("tag");
        xml_tag->SetAttribute("k", "osmAG:type");
        xml_tag->SetAttribute("v", "area");
        xml_node->InsertEndChild(xml_tag);
        xml_tag = doc.NewElement("tag");
        xml_tag->SetAttribute("k", "osmAG:areaType");
        xml_tag->SetAttribute("v", it->second->type.c_str());
        xml_node->InsertEndChild(xml_tag);
        if(it->second->has_parent_){
            xml_tag = doc.NewElement("tag");
            xml_tag->SetAttribute("k", "osmAG:parent");
            xml_tag->SetAttribute("v", it->second->parent_);
            xml_node->InsertEndChild(xml_tag);
        }
        for(auto tag_left_it = attr->tags.begin(); tag_left_it != attr->tags.end(); tag_left_it++){
            xml_tag = doc.NewElement("tag");
            xml_tag->SetAttribute("k", (tag_left_it->first).c_str());
            xml_tag->SetAttribute("v", (tag_left_it->second).c_str());
            xml_node->InsertEndChild(xml_tag);
        }
        root->InsertEndChild(xml_node);
    }

    for(auto it = graph.passages_.begin(); it!= graph.passages_.end(); it++){
        XMLElement* xml_node = doc.NewElement("way");
        
        xml_node->SetAttribute("id", it->first);
        auto attr = it->second->getAttributesPtr();
        xml_node->SetAttribute("action", attr->action.c_str());
        const char * vis = attr->visible ? "true": "false";
        xml_node->SetAttribute("visible", vis);

        
        XMLElement* xml_ref = doc.NewElement("nd");
        xml_ref->SetAttribute("ref", it->second->passage_nodes.source);
        xml_node->InsertEndChild(xml_ref);

        xml_ref = doc.NewElement("nd");
        xml_ref->SetAttribute("ref", it->second->passage_nodes.target);
        xml_node->InsertEndChild(xml_ref);

        XMLElement* xml_tag = doc.NewElement("tag");
        xml_tag->SetAttribute("k", "osmAG:type");
        xml_tag->SetAttribute("v", "passage");
        xml_node->InsertEndChild(xml_tag);
        xml_tag = doc.NewElement("tag");
        xml_tag->SetAttribute("k", "osmAG:from");
        xml_tag->SetAttribute("v", it->second->area_from);
        xml_node->InsertEndChild(xml_tag);
        xml_tag = doc.NewElement("tag");
        xml_tag->SetAttribute("k", "osmAG:to");
        xml_tag->SetAttribute("v", it->second->area_to);
        xml_node->InsertEndChild(xml_tag);

        for(auto tag_left_it = attr->tags.begin(); tag_left_it != attr->tags.end(); tag_left_it++){
            xml_tag = doc.NewElement("tag");
            xml_tag->SetAttribute("k", (tag_left_it->first).c_str());
            xml_tag->SetAttribute("v", (tag_left_it->second).c_str());
            xml_node->InsertEndChild(xml_tag);
        }
        root->InsertEndChild(xml_node);
    }


    doc.SaveFile(file_path);


    
}

// /**
// * @brief merge two graphs which maybe have same nodes.
// 改变另一个graph的node id
// **/

// void MergeTwoGraphs(AreaGraph& graph1, AreaGraph& graph2){
//     for(auto it = graph2.nodes_.begin(); it!=graph2.nodes_.end(); it++){
//         for(auto itt = graph1.nodes_.begin(); itt!= graph1.nodes_.end(); itt++){
            
//         }
//     }

// }
// bool ServiceCallBack(nav_msgs::GetMap::Request &req,nav_msgs::GetMap::Response &res, AreaGraph& graph, AreaId area_id)
// {
//     if(graph.areas_[area_id]->use_occupancy_map){
//         res.map = graph.areas_[area_id]->area_occupancy;
//         return true;
//     }else{
//         return false;
//     }
// }


int main(int argc, char * argv[]){
    AreaGraph graph;
    // Parsing_Osm2AreaGraph(graph, "../data/fix_id/shanghaitech_merge_v1.osm");//fix_id/shanghaitech_
    bool sucess_read = true;
    sucess_read = Parsing_Osm2AreaGraph(graph, "../data/fix_id/SIST_F2.osm");//fix_id/shanghaitech_

    if(!sucess_read){
        return 0;
    }else{
        printf("We parse OSM-AG from XML!!!\n");
    }

    // Parsing_Osm2AreaGraph(graph);
    TraverseArea(graph);
    TraversePassage(graph);
    cv::Mat img(cv::Size(IMAGE_W,IMAGE_H),CV_8UC3,cv::Scalar(255, 255, 255));
    PlotNodePoints(graph, img);
    PlotAreas(graph,img);
    PlotPassages(graph,img);
    //PlanInStructure(graph,-125355,-125412,-125406);
    PathNode* p = new PathNode;

    // PathNode *p = (PathNode*)std::malloc(sizeof(PathNode));
    if(!p->previous)
    {
        printf("Err p initial pathnode's pre is not NULL!!!!! The pre passage id is %ld\n", p->passage_id);
    }else{
        printf("Ok\n");
    }

    if(p->previous != nullptr)
    {
        printf("Err p initial pathnode's pre is not NULL!!!!! The pre passage id is %ld\n", p->passage_id);
    }else{
        printf("Ok2\n");
    }
    
    // ros::init(argc, argv, "area_occupany_map_node");
	// ros::NodeHandle nh;
    // ros::Publisher pub = nh.advertise<nav_msgs::OccupancyGrid>("/map", 1);

    // // // ros::ServiceServer serv=nh.advertiseService("/static_map",ServiceCallBack);

    AreaId test_area_id = -125349;
    if(graph.areas_.find(test_area_id) != graph.areas_.end()){
        printf("We have this area");
        // ComputeCenter_Area(graph,-125349);
        InitOccupancyMap_Area(graph,test_area_id,0.1);
    }

     
	// while (ros::ok())
	// {
	// 	pub.publish(graph.areas_[test_area_id]->area_occupancy);
	// }
 
	// ros::shutdown();

    cv::Mat temp(img);
    cv::Mat map_compare = cv::imread("show_gridmap_layer_ori.png");
    if(map_compare.type() == CV_8UC1){
        cv::cvtColor(map_compare, map_compare, cv::COLOR_GRAY2BGR);
    }

    std::vector<Eigen::Vector3d> path_result;
    PlanInPathGraph(graph,-147602,-147580,path_result);//-125412,-125406,  -147705,-147706
    PlotPath(path_result, map_compare);
    cv::imwrite("show_path.png",map_compare);

    // std::vector<std::vector <Eigen::Vector3d>> path_temp;
    // PlanInPathGraph(graph,-125432,-125399, path_temp);
    // for(auto p:path_temp){
    //     PlotPath(p, temp);
    // }
    // cv::imwrite("show_path.png",temp);



    //For compare with 2d grid map using A*
    PlanInGridMap(graph,-147602,-147580, map_compare, 0.1);


    Save_AreaGraph2Osm(graph);
    return 0;
}