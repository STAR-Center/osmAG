#ifndef _DATALOADSAVE_H_
#define _DATALOADSAVE_H_
#include <tinyxml2.h>
#include "../include/area_grid_map.h"
#include "../include/visualization.h"
#include "../include/WGS84toCartesian.h"

using namespace tinyxml2;
using namespace osm_ag;

double Deg2Rad(double d);

Eigen::Vector3d CoordinateTransform(GeoCoordinate init, GeoCoordinate cur);

double ComputPathDistance(vector<Eigen::Vector3d>& path_pts);

void TraverseArea(AreaGraph& graph);

void TraversePassage(AreaGraph& graph);

void PlanInStructure(AreaGraph& graph, AreaId parent_areaid, PassageId start_id, PassageId end_id);

void PlanInPathGraph(AreaGraph& graph, PassageId start_id, PassageId end_id, std::vector<Eigen::Vector3d>& path_result);

void PlanInPathGraph(AreaGraph& graph, PassageId start_id, PassageId end_id, std::vector<std::vector<Eigen::Vector3d>>& path_result);

void PlanInGridMap(AreaGraph& graph, PassageId start_id, PassageId end_id, cv::Mat& grid_map_show, double resolution = 0.1, bool if_show = true);

bool Parsing_Osm2AreaGraph(AreaGraph& graph, const char* file_path = "../data/f1-d(v1).osm");

void Save_AreaGraph2Osm(const AreaGraph& graph, const char* file_path = "../data/fix_id/new.osm");

bool Init_OSMAG(AreaGraph& graph, const char* file_path = "../data/fix_id/SIST_F2.osm"){
    bool initialized = false;

    bool sucess_read = true;
    sucess_read = Parsing_Osm2AreaGraph(graph, file_path);
    if(!sucess_read){
        return false;
    }else{
        printf("We parse OSM-AG from XML!!!\n");
    }

    TraverseArea(graph);
    TraversePassage(graph);
    initialized = true;
    return initialized;
}

#endif
