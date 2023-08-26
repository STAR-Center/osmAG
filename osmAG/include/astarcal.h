#ifndef _ASTARCAL_H_
#define _ASTARCAL_H_
#include<opencv2/opencv.hpp>
#include<iostream>
using namespace std;
 
 
const int kCost1 = 10; //直移一个点消耗G 
const int kCost2 = 14; //斜移一个点消耗G 
 
 
struct CalcPt
{
  cv::Point pt; //OpenCV中点坐标 
  int F, G, H; //F=G+H 
  CalcPt* parent; //parent的坐标，这里没有用指针，从而简化代码 
  CalcPt(cv::Point _pt) :pt(_pt), F(0), G(0), H(0), parent(NULL)  //变量初始化 
  {
  }
};
 
 
class AStarCalc
{
private:
 
 
  CalcPt* findPath(CalcPt& startPoint, CalcPt& endPoint, bool isIgnoreCorner);
  //计算临近八个点
  vector<CalcPt*> getSurroundPoints(const CalcPt* point, bool isIgnoreCorner) const;
  //判断某点是否可以用于下一步判断 
  bool isCanreach(const CalcPt* point, const CalcPt* target, bool isIgnoreCorner) const;
  //判断开启/关闭列表中是否包含某点
  CalcPt* isInList(const std::list<CalcPt*>& list, const CalcPt* point) const;
  //检测障碍点
  bool isInSites(const cv::Point* point) const;
  //从开启列表中返回F值最小的节点 
  CalcPt* getLeastFpoint();
  //计算FGH值 
  int calcG(CalcPt* temp_start, CalcPt* point);
  int calcH(CalcPt* point, CalcPt* end);
  int calcF(CalcPt* point);
 
 
  vector<vector<int>> sites;  //图片点 0-可通行，1-障碍点
  list<CalcPt*> openList;  //开启列表 
  list<CalcPt*> closeList; //关闭列表 
 
 
public:
  void InitSites(vector<vector<int>> _sites); //初始化地图
  //获取到路径
  list<CalcPt*> GetPath(CalcPt& startPoint, CalcPt& endPoint, vector<int>& resDistance, bool isIgnoreCorner = false);
};
 
#endif
 