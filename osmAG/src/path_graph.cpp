#include "../include/pathgraph.h"
#include "../include/area_grid_map.h"

using namespace osm_ag;
double ComputeAstarDis(GridMat& grids, cv::Point2i pt_s, cv::Point2i pt_e, std::vector<cv::Point>& resPoints, bool if_show = false){
	// std::vector<Point> resPoints;
    double res_dis;
    vector<int> dis_result;
	std::vector<std::vector<int>> sites;
    for (size_t col = 0; col < grids.grid_mat.cols; col++) {
		std::vector<int> rows;
		for (size_t row = 0; row < grids.grid_mat.rows; row++) {
			int color = grids.grid_mat.at<uchar>(row, col);
			rows.push_back(color == 255 ? 0 : 1);//白色的点是障碍点？？？
		}
		sites.push_back(rows);
	}
    AStarCalc calc = AStarCalc();
    calc.InitSites(sites);
    CalcPt startpt = CalcPt(pt_s);
	CalcPt endpt = CalcPt(pt_e);

	list<CalcPt* > reslist = calc.GetPath(startpt, endpt, dis_result);
	for (auto p : reslist) {
		resPoints.push_back(p->pt);
	}
    cv::Mat srccpy = grids.grid_mat.clone();
    // cv::namedWindow("path_in_one_area");
    // cv::resizeWindow("path_in_one_area",400,300);
    cv::circle(srccpy,pt_s, 2, cv::Scalar(75));
    cv::circle(srccpy,pt_e, 2, cv::Scalar(75));


    //test the performance of a* in grid map
    if(resPoints.size()>0){
        if(if_show){
            // cv::Mat srccpy = grids.grid_mat.clone();
            for(int j=0; j<resPoints.size()-1; j++){
                int k = j+1;
                cv::line(srccpy, resPoints[j], resPoints[k], cv::Scalar(200),1);
            }
            // cv::imwrite("path_in_one_area.png", srccpy);
            // // cv::imshow("path_in_one_area.png", srccpy);
	        // // cv::waitKey(0);

        }
        res_dis = dis_result[0]/10.0 ;
        return res_dis;
             
    }else{
        printf("ERR: CANNOT ARRIVE!!!\n");
        // cv::imshow("path_in_one_area.png", srccpy);
        // cv::imwrite("path_in_one_area_wrong.png", srccpy);

        // cv::waitKey(0);
        // cv::destroyAllWindows();
        return std::numeric_limits<double>::max();
    }
    
}

Eigen::Vector3d Grid2XYZ(int x_grid, int y_grid, Eigen::Vector3d center_position, double resolution, int origin_x, int origin_y, int offset){
    Eigen::Vector3d result;
    result[0] = (x_grid - offset - origin_x) * resolution + center_position[0];
    result[1] = (y_grid - offset - origin_y) * resolution + center_position[1];
    result[2] = center_position[2];
    return result;
}

void Find_PathInArea(AreaGraph& graph, AreaId area_id, std::vector<PathEdge*>& pathedges){
    if(graph.areas_.find(area_id)==graph.areas_.end()){
            printf("The wrong area id is %ld\n", area_id);
            return;
    }

    auto path_in_area = graph.areas_[area_id]->passageids;//同一个房间的passages
    auto path_in_area_temp = graph.areas_[area_id]->passageids;

    Eigen::Vector3d center_position = graph.areas_[area_id]->center_position;

    if(path_in_area.size()>=2){
        printf("The current area id is %ld\n",area_id);
        //并且 from和to 应该至少有一个不同
        for(auto pathit_i = path_in_area.begin(); pathit_i != path_in_area.end(); pathit_i++){
            std::unordered_set<AreaId> i;
            path_in_area_temp.erase(*pathit_i);
            AreaId idi_from = graph.passages_[*pathit_i]->area_from;
            AreaId idi_to = graph.passages_[*pathit_i]->area_to;
            i.insert(idi_from);
            i.insert(idi_to);
            for(auto pathit_j = path_in_area_temp.begin(); pathit_j!= path_in_area_temp.end(); pathit_j++){
            AreaId idj_from = graph.passages_[*pathit_j]->area_from;
            AreaId idj_to = graph.passages_[*pathit_j]->area_to;
            bool diff = i.count(idj_from) && i.count(idj_to); //是false的话 说明有不一样的元素
                if(diff){
                printf("This two passages %ld and %ld  are similar!!!\n",*pathit_i,*pathit_j);
                    continue;
                }else{
                    //说明有一个区域是不同的
                    // PathEdge* edge = (PathEdge*)std::malloc(sizeof(PathEdge));
                    PathEdge* edge = new PathEdge();
                    Eigen::Vector3d pi = graph.passages_[*pathit_i]->center_position;
                    Eigen::Vector3d pj = graph.passages_[*pathit_j]->center_position;
                    //for elevator, don't build the grid map, so replace dist_astar using dist_L2. 
                    edge->from = *pathit_i;
                    edge->to = *pathit_j;
                    edge->dist_L2 = (pi-pj).norm();
                    edge->dist_L1 = (pi-pj).lpNorm<1>();
                    edge->area_id_belongto = area_id;
                    pathedges.push_back(edge);
                }
            }
        }
    }
}

void Find_PathInArea(AreaGraph& graph, AreaId area_id, std::vector<PathEdge*>& pathedges, PathedgesTrjMap& edges_trj_map){
    if(graph.areas_.find(area_id)==graph.areas_.end()){
            printf("The wrong area id is %ld\n", area_id);
            return;
    }

    if(!graph.areas_[area_id]->use_occupancy_map){
        InitOccupancyMap_Area(graph, area_id, 0.1);
    }

    auto path_in_area = graph.areas_[area_id]->passageids;//同一个房间的passages
    auto path_in_area_temp = graph.areas_[area_id]->passageids;

    Eigen::Vector3d center_position = graph.areas_[area_id]->center_position;

    GridMat grid_mat_ = graph.areas_[area_id]->grid_mat_;
    bool if_show = false;
    // if(area_id == -125431){
    //     if_show = true;
    // }
    if(path_in_area.size()>=2){
        printf("The current area id is %ld\n",area_id);
        //并且 from和to 应该至少有一个不同
        for(auto pathit_i = path_in_area.begin(); pathit_i != path_in_area.end(); pathit_i++){
            std::unordered_set<AreaId> i;
            path_in_area_temp.erase(*pathit_i);
            AreaId idi_from = graph.passages_[*pathit_i]->area_from;
            AreaId idi_to = graph.passages_[*pathit_i]->area_to;
            i.insert(idi_from);
            i.insert(idi_to);
            for(auto pathit_j = path_in_area_temp.begin(); pathit_j!= path_in_area_temp.end(); pathit_j++){
            AreaId idj_from = graph.passages_[*pathit_j]->area_from;
            AreaId idj_to = graph.passages_[*pathit_j]->area_to;
            bool diff = i.count(idj_from) && i.count(idj_to); //是false的话 说明有不一样的元素
                if(diff){
                printf("This two passages %ld and %ld  are similar!!!\n",*pathit_i,*pathit_j);
                    continue;
                }else{
                    //说明有一个区域是不同的
                    // PathEdge* edge = (PathEdge*)std::malloc(sizeof(PathEdge));
                    PathEdge* edge = new PathEdge();
                    Eigen::Vector3d pi = graph.passages_[*pathit_i]->center_position;
                    Eigen::Vector3d pj = graph.passages_[*pathit_j]->center_position;
                    int x_i = (pi[0] - center_position[0])/grid_mat_.resolution + grid_mat_.origin_grid_x + grid_mat_.offset;
                    int y_i = (pi[1] - center_position[1])/grid_mat_.resolution + grid_mat_.origin_grid_y + grid_mat_.offset;
                    int x_j = (pj[0] - center_position[0])/grid_mat_.resolution + grid_mat_.origin_grid_x + grid_mat_.offset;
                    int y_j = (pj[1] - center_position[1])/grid_mat_.resolution + grid_mat_.origin_grid_y + grid_mat_.offset;
                    cv::Point2i point_i(x_i,y_i);
                    cv::Point2i point_j(x_j,y_j);
                    std::vector<cv::Point2i> grid_path; 
                    std::vector<Eigen::Vector3d> trj_astar_inv;
                    // cv::Mat cpy = grid_mat_.grid_mat.clone();
                    edge->dist_astar = ComputeAstarDis(grid_mat_, point_i, point_j, grid_path, if_show);
                    //for elevator, don't build the grid map, so replace dist_astar using dist_L2. 
                    if(grid_path.size()>0){                    
                        Eigen::Vector3d trj_pt;
                        for(auto pt:grid_path){//这个grid_path的输出也是倒序。
                            trj_pt = Grid2XYZ(pt.x, pt.y, center_position, grid_mat_.resolution, grid_mat_.origin_grid_x, grid_mat_.origin_grid_y, grid_mat_.offset);
                            trj_astar_inv.emplace_back(trj_pt);
                        }
                    }else{//TODO: now we just consider situation about elevator but not stair
                        edge->dist_astar = (pi-pj).norm();//考虑一下Z轴上的距离
                        trj_astar_inv.emplace_back(pi);
                        trj_astar_inv.emplace_back(pj);//TODO:当前没有考虑z的数值！！需要对parent进行操作。
                    }
                    //for elevator, don't build the grid map, so replace dist_astar using dist_L2. 
                    edge->from = *pathit_i;
                    edge->to = *pathit_j;
                    edge->dist_L2 = (pi-pj).norm();
                    edge->dist_L1 = (pi-pj).lpNorm<1>();
                    edge->area_id_belongto = area_id;
                    pathedges.push_back(edge);

                    edges_trj_map[make_pair(*pathit_j, *pathit_i)] = trj_astar_inv;
                                        
                    std::reverse(trj_astar_inv.begin(), trj_astar_inv.end());
                    edge->trj_astar = trj_astar_inv;
                    edges_trj_map[make_pair(*pathit_i, *pathit_j)] = edge->trj_astar;

                }
            }
        }
    }
}

/**
*@brief Traverse the area graph and construct the passage graph for areas in the lowest level. 
需要搜索找到叶节点！！不是某个区域的父亲（现在只有两个map能够证明是否是高级的区域），直接遍历所有区域，看其是否是tree的key
*/
PathGraph::PathGraph(AreaGraph& graph, PassageGraph& passage_graph){
    for(auto it = graph.areas_.begin(); it!=graph.areas_.end();it++){
        if(graph.area_trees.find(it->first) ==graph.area_trees.end()){//说明该区域不是任意一个parent
            Find_PathInArea(graph, it->first, pathedges, pathedges_trj);
            passage_graph.pathedges.insert(passage_graph.pathedges.end(),pathedges.begin(),pathedges.end());
            passage_graph.pathedges_trj.insert(pathedges_trj.begin(), pathedges_trj.end());
            for(auto itt = it->second->passageids.begin(); itt != it->second->passageids.end(); itt++){
                // PathNode *path = (PathNode*)std::malloc(sizeof(PathNode));
                PathNode *path = new PathNode();
                if(path->previous!= nullptr){
                    printf("Err initial pathnode's pre is not NULL!!!!! The pre passage id is %ld\n", path->passage_id);
                }
                // else{
                //     printf("OK ");
                // }
                path->passage_id = *itt;
                path->position = graph.passages_[*itt]->center_position;
                pathnodes[*itt] = path;
                passage_graph.pathnodes[*itt] = path;
            }
        }else{
            it->second->is_leaf = false;//TODO 在这里判断很奇怪，traverse的时候应该判断
        }
    }
    printf("The Number of edges of the whole passage graph:%ld\n", pathedges.size());
    std::ofstream out;
    PrintEdgesDistance(graph, out);
}



//TODO::::因为不想修改areagraph当中的东西，所以尽量用GET!!!!!!!
//其他的类是友元，但是plan这个不是友元，把一些私有的东西都弄成get

PathLayer::PathLayer(AreaGraph& graph, AreaId parent_area_id):parent_area_id(parent_area_id){
    //提取该层所有的passage
    std::unordered_set<PassageId> passages_layer = graph.passage_trees[parent_area_id];
    printf("size of passagers_layer %d\n", passages_layer.size());

    for(auto it = passages_layer.begin(); it!=passages_layer.end(); it++){
        // PathNode *path = (PathNode*)std::malloc(sizeof(PathNode));
        PathNode *path = new PathNode();
        if(path->previous!= nullptr){
            printf("Err initial pathnode's pre is not NULL!!!!! The pre passage id is %ld\n", path->passage_id);
        }
        // else{
        //     printf("OK ");
        // }
        path->passage_id = *it;
        path->position = graph.passages_[*it]->center_position;
        pathnodes[*it] = path;
    }


    // printf("STEP2\n");

    //add edge
    //只预先算同一个area(最低级）的不同的edge
    //我们必须先初始化了pathedges_in_onearea,才方便计算pathedges
    std::unordered_set<AreaId> areas_layer = graph.area_trees[parent_area_id];
    for(auto itt = areas_layer.begin(); itt!=areas_layer.end(); itt++){
        Find_PathInArea(graph, *itt, pathedges);
    }
    printf("pathedges counts %d\n",pathedges.size());

}



// PathLayer::PathLayer(AreaGraph& graph, AreaId parent_area_id):parent_area_id(parent_area_id){

//     //提取该层所有的passage
//     std::unordered_set<PassageId> passages_layer = graph.passage_trees[parent_area_id];
//     int test = 0;

//     for(auto it = passages_layer.begin(); it!=passages_layer.end(); it++){
//         PassageId current_id = *it;
//         PathNode path(current_id);
//         path.position = graph.passages_[*it]->center_position;
//         pathnodes[*it] = &path;
//         test++;
//     }


//     //add edge
//     //只预先算同一个area(最低级）的不同的edge
//     //我们必须先初始化了pathedges_in_onearea,才方便计算pathedges
//     std::unordered_set<AreaId> areas_layer = graph.area_trees[parent_area_id];
//     for(auto itt = areas_layer.begin(); itt!=areas_layer.end(); itt++){
//         if(graph.areas_.find(*itt)==graph.areas_.end()){
//             printf("The wrong area id is %ld\n", *itt);
//             continue;
//         }

//         auto path_in_area = graph.areas_[*itt]->passageids;//同一个房间的passages
//         auto path_in_area_temp = graph.areas_[*itt]->passageids;
        

//         if(path_in_area.size()>=2){
//             //并且 from和to 应该至少有一个不同
//             for(auto pathit_i = path_in_area.begin(); pathit_i != path_in_area.end(); pathit_i++){
//                 std::unordered_set<AreaId> i;
//                 path_in_area_temp.erase(*pathit_i);
//                 AreaId idi_from = graph.passages_[*pathit_i]->area_from;
//                 AreaId idi_to = graph.passages_[*pathit_i]->area_to;
//                 i.insert(idi_from);
//                 i.insert(idi_to);
//                 for(auto pathit_j = path_in_area_temp.begin(); pathit_j!= path_in_area_temp.end(); pathit_j++){
//                     AreaId idj_from = graph.passages_[*pathit_j]->area_from;
//                     AreaId idj_to = graph.passages_[*pathit_j]->area_to;
//                     bool diff = i.count(idj_from) && i.count(idj_to); //是false的话 说明有不一样的元素
//                     if(diff){
//                         printf("This two passages %ld and %ld  are similar!!!\n",*pathit_i,*pathit_j);
//                         continue;
//                     }else{
//                         //说明有一个区域是不同的
//                         PathEdge* edge;
//                         Eigen::Vector3d pi = graph.passages_[*pathit_i]->center_position;
//                         Eigen::Vector3d pj = graph.passages_[*pathit_j]->center_position;
//                         edge->from = *pathit_i;
//                         edge->to = *pathit_j;
//                         edge->dist_L2 = (pi-pj).norm();
//                         edge->area_id_belongto = *itt;
//                         pathedges.push_back(edge);
//                     }
//                 }
//             }
//         }
//     }
// }

//计算一个structure中的 不在同一个area间的passage间的距离
void PathBase::AddEdge_AStar(PassageId start_id, PassageId end_id, Distance_Type type){
    dis_type = type;
    printf("Start to compute the paths, we use DISTANCE TYPE : %d\n", dis_type);
    //初始化所有Pathnode的plan值
    printf("pathnodes size: %d\n",pathnodes.size());
    for(auto it=pathnodes.begin(); it!=pathnodes.end(); it++){
        it->second->InitForPlanning();//为了避免之前算过没有恢复初始值
        // printf("pathnode id(passage) is %ld\n", it->second->passage_id);
    }
    // printf("TASK0\n");

    //创建要记录的最短路径
    // Path* result_path = (Path*)std::malloc(sizeof(Path)); //这个是反向的路径
    Path* result_path = new Path(start_id,end_id);


    //创建openset
    std::priority_queue<PathNode*> open_set;//或者用passageid一直查找就行
    //创建closedset
    std::unordered_set<PathNode*> closed_set;
    //把源点加入openset中
    PathNode* start_vertex = pathnodes[start_id];
    start_vertex->g_value = 0;
    start_vertex->update_f();
    open_set.push(start_vertex);
    start_vertex->is_open = true;
    if(start_vertex->previous != nullptr){
      printf("start previous is not null!!!!!!!The passage id is %ld\n", start_vertex->previous->passage_id);
    }


    while(!open_set.empty()){
        //选出openset中f最小的节点
        PathNode* best = open_set.top();
        printf("best id is %ld\n", best->passage_id);
        closed_set.insert(best);
        open_set.pop();
        best->is_closed = true;
        //是否到达终点
        if(best->passage_id == end_id){
            printf("maybe end\n");
            if(dis_type == L2){
                result_path->dist_L2 = best->g_value;
            }else if(dis_type == L1){
                result_path->dist_L1 = best->g_value;
            }else{
                result_path->dist_astar = best->g_value;
            }
            result_path->path.push_back(end_id);
            PathNode* pre = best->previous;
            //反向搜索前序节点
            while(pre != nullptr){//当前序没有节点时，说明已经到达起点 ///TODO: bug 当前的pre居然不为空
                if(pre->passage_id!=0)
                {   
                    result_path->path.push_back(pre->passage_id);
                }else{
                    printf("The start vertex's pre is not null!!!\n");
                }
                printf("previous id %d\n",pre->passage_id);
                pre = pre->previous;
            }
            if(result_path->path.back()==start_id){
                printf("Find the shortest path successfully!!!\n");
                paths.push_back(result_path);
            }else{
                printf("Failed to find path!!!!!!!!\n");
            }
            break;
        }else{
            //为当前的节点创建邻居集 不用set，不可能有重复的两条edge
            std::vector<std::pair<PassageId,double>> subs;
            FindNeighbors(subs,best->passage_id);
            if(subs.size()==0){
                continue;
            }else{
                for(auto sub:subs){
                    PathNode* neigbor = pathnodes[sub.first];
                    double new_g_value = best->g_value + sub.second;
                    //判断neibor是否在openset中， is_open为true同时is_closed为false的话就在
                    if(neigbor->is_open && !neigbor->is_closed){
                        if(new_g_value < neigbor->g_value){
                            //删除以前的历史sub
                            neigbor->g_value = new_g_value;
                            neigbor->previous = best;
                        }else{
                            continue;//处理下一个sub
                        }
                    }else if(neigbor->is_closed){//判断是否在closed集中，都为ture
                        // 在closed集中时，不应该直接跳过吗？？？？！！！！！

                        // if(new_g_value < neigbor->g_value){
                        //     //删除以前的历史sub
                        //     neigbor->g_value = new_g_value;
                        //     neigbor->previous = best;
                        // }else{
                            continue;//处理下一个sub
                        // }
                    }else{
                        neigbor->previous = best;
                        neigbor->is_open = true;
                        open_set.push(neigbor);
                    }
                    neigbor->update_f();
                }
            }
        }
    }


    
}


// //计算所有的pathnode间的最短距离 Dijkstra
// //但是不包括具体的路径！！！
// void PathLayer::MinDist(AreaGraph& graph, PassageId start_id, std::unordered_set<PassageId> end_ids){
//     ;//Todo 好像不需要
// }


void PathBase::FindNeighbors(std::vector<std::pair<PassageId,double>>& subs, PassageId current_id){
    //获取所有邻居，通过edges找是否有from和to中有这个node的
    for(auto edge: pathedges){
        if(current_id==edge->from){
            if(dis_type == L2){
                subs.push_back(std::make_pair(edge->to,edge->dist_L2));
            }else if(dis_type == L1){
                subs.push_back(std::make_pair(edge->to,edge->dist_L1));
            }else{
                subs.push_back(std::make_pair(edge->to,edge->dist_astar));
            }
        }else if(current_id==edge->to){
            if(dis_type == L2){
                subs.push_back(std::make_pair(edge->from,edge->dist_L2));
            }else if(dis_type == L1){
                subs.push_back(std::make_pair(edge->from,edge->dist_L1));
            }else{
                subs.push_back(std::make_pair(edge->from,edge->dist_astar));
            }
        }
    }
    printf("neighbors counts %d\n",subs.size());
}

void PathBase::PrintEdgesDistance(AreaGraph& graph, std::ofstream& outfile){
    if(!outfile.is_open()){
        // outfile.open("distance_check.txt", std::ios::app);
        outfile.open("distance_check.txt", std::ios::in);
    }

    time_t t = time(nullptr);
	struct tm* now = localtime(&t);
 
	std::stringstream time;
 
	time << now->tm_year + 1900 << "年";
	time << now->tm_mon + 1 << "月";
	time << now->tm_mday << "日 ";
	time << now->tm_hour << ":";
	time << now->tm_min << ":";
	time << now->tm_sec;
    outfile << "This file is recorded at " << time.str() << "!!!!" << std::endl;

    for(auto edge: pathedges){
        outfile << "Edge!!! from id: " << edge->from << "(name: " << graph.passages_[edge->from]->info->tags["name"].c_str() << "); ";
        outfile << "to id: " << edge->to << "(name: " << graph.passages_[edge->to]->info->tags["name"].c_str() << "); ";
        outfile << "Distance L2 : " << edge->dist_L2;
        outfile << " and Distance L2 : " << edge->dist_L1 << std::endl;
        // printf("Edge!!! from id: %ld (name: %s); ", edge->from, graph.passages_[edge->from]->info->tags["name"].c_str());
        // printf("to id: %ld (name: %s); ", edge->to, graph.passages_[edge->to]->info->tags["name"].c_str());
        // printf("Distance L2 : %lf \n", edge->dist_L2);
    }
}


