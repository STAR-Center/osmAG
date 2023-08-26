//
// Created by houjw on 18-11-26.
//

#include <string>
#include <iostream>
#include <fstream>

#include <cstdio>
#include <cstdlib>
#include "AreaGenerate.h"
#include "Area_osm.h"
#include <cgal/TransformationComputation.h>
#include "TablePrint.h"
#include "csvwriter.h"
#include <time.h>

using namespace std;

VoriConfig *sConfig;

template<typename T>
std::string NumberToString(T Number) {
    std::ostringstream ss;
    ss << Number;
    return ss.str();
}


int main(int argc, char *argv[]) {

    if (argc < 2) {
        cout << "Usage " << argv[0] << " image1.png" << "resolution " << endl;
        return 255;
    }
    QApplication a(argc, argv);
    double reso1 = 1.0, reso2 = 1.0;
    uint sw = 0;
    if (argc >= 3) {
        reso1 = atof(argv[2]);
        if (reso1 == 0 ) {
#define RST  "\x1B[0m"
#define KRED  "\x1B[31m"
#define FRED(x) KRED x RST
            cerr << FRED("ERROR!!! Failed to convert resolutions: at least one resolution is 0! Possible reason: ")
                 << endl
                 << FRED(
                            "atof() is locale-dependant. Your locale uses , to signify a decimal, while the input string uses . to specify a decimal point. Please try to input resolutions with comma as decimal point.")
                 << endl;
            return -1;
        }
    }

    sConfig = new VoriConfig();
    sConfig->doubleConfigVars["alphaShapeRemovalSquaredSize"] = 250;
    sConfig->doubleConfigVars["firstDeadEndRemovalDistance"] = 100000;
    sConfig->doubleConfigVars["secondDeadEndRemovalDistance"] = -100000;
    sConfig->doubleConfigVars["thirdDeadEndRemovalDistance"] = 16;
    sConfig->doubleConfigVars["fourthDeadEndRemovalDistance"] = 8;
    sConfig->doubleConfigVars["topoGraphAngleCalcEndDistance"] = 10;
    sConfig->doubleConfigVars["topoGraphAngleCalcStartDistance"] = 3;
    sConfig->doubleConfigVars["topoGraphAngleCalcStepSize"] = 0.1;
    sConfig->doubleConfigVars["topoGraphDistanceToJoinVertices"] = 10;
    sConfig->doubleConfigVars["topoGraphMarkAsFeatureEdgeLength"] = 16;
    sConfig->doubleConfigVars["voronoiMinimumDistanceToObstacle_"] = 0.20;// 0.25;//0.18;//0.36;
    sConfig->doubleConfigVars["alphaShapeRemoval_R_Meter"] = 1.4;
    1.2 * 0.5;
    sConfig->doubleConfigVars["firstDeadEndRemovalDistance"] = 100000;
    sConfig->doubleConfigVars["secondDeadEndRemovalDistance"] = -100000;
    sConfig->doubleConfigVars["topoGraphDistanceToJoinVertices"] = 4;

    if (argc > 3) {
        sConfig->doubleConfigVars["alphaShapeRemoval_R_Meter"] = atof(argv[3]) * 0.5;
    }

    double ref_lat, ref_lon, cart_orig_x, cart_orig_y;
    if (argc > 7) {
        ref_lat=atof(argv[4]), ref_lon=atof(argv[5]), cart_orig_x=atof(argv[6]), cart_orig_y=atof(argv[7]);
    }else{
        cout<<"Please input ref_lat, ref_lon, cart_orig_x, cart_orig_y: (split with space)"<<endl;
        cin>>ref_lat>>ref_lon>>cart_orig_x>>cart_orig_y;
    }
    double neighbors=8, percentage=0;
    if (argc > 9) {
    neighbors=atof(argv[8]), percentage=atof(argv[9]);
    }

    VoriGraph voriGraph1;

//-----------------Compute properties for the first image---------------
    sConfig->doubleConfigVars["voronoiMinimumDistanceToObstacle"] =
            sConfig->voronoiMinimumDistanceToObstacle_() / reso1;
    sConfig->doubleConfigVars["alphaShapeRemovalSquaredSize"] =
            (sConfig->alphaShapeRemoval_R_Meter() / reso1) * (sConfig->alphaShapeRemoval_R_Meter() / reso1);
    
    bool de = DenoiseImg(argv[1], "clean.png", neighbors, percentage);
    if (de)
        cout << "Denoise run successed!!" << endl;
    generateAreas(argv[1], "detect.png", sConfig, voriGraph1);
    bool if_use_keypts = true ;//If you want to reduces the number of points, please make it true!!!! else false.
    RMG::GraphExtract testosm(voriGraph1, ref_lat, ref_lon, reso1, cart_orig_x, cart_orig_y,"../maps/test.osm", if_use_keypts);

     QImage dectRoom;
     dectRoom.load("clean.png");
     paintVori_onlyArea(dectRoom, voriGraph1);
     dectRoom.save( "dectRoom.png" );

         
    

    delete sConfig;
    return 0;
}

