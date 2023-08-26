#include "../include/astarcal.h"
 
CalcPt* AStarCalc::findPath(CalcPt& startPoint, CalcPt& endPoint, bool isIgnoreCorner)
{
  bool isNearEndPoint = false;
  //首先写入起点,拷贝开启一个节点，内外隔离
  CalcPt* firstpt = new CalcPt(cv::Point(startPoint.pt.x, startPoint.pt.y));
  firstpt->H = calcH(firstpt, &endPoint);
  firstpt->F = calcF(firstpt);
  openList.push_back(firstpt);
  while (!openList.empty()) {
    //找到F值最小的点
    auto curPoint = getLeastFpoint();
    //如果计算的点在关闭列表中，说明无路可走了
    if (!isInList(closeList, curPoint)) {
      //找到后从开启列表中删除
      openList.remove(curPoint);
      //存放到关闭列表中
      closeList.push_back(curPoint);
    }
    else
    {
      return curPoint;
    }
 
 
    //距离小于100时开启终点检测,
    //用此方法测试计算比原来减少了800毫秒

    if (curPoint->H < 100) isNearEndPoint = true;
 
 
    //1.找到当前点周围八个点中可以通过的点
    auto surroundPoints = getSurroundPoints(curPoint, isIgnoreCorner);
    for (auto& target : surroundPoints) {
      //2.对某一个点，如果不在开启列表中，加入到开启列表中，设置当前格为父节点，计算F,G,H的值
      CalcPt* targetpt = isInList(openList, target);
      if (!targetpt) {
        //计算F,G,H的值
        target->G = calcG(curPoint, target);
        target->H = calcH(target, &endPoint);
        target->F = calcF(target);
 
 
        target->parent = curPoint;
 
 
        //插入到开启列表中
        openList.push_back(target);
      }
      //3.对某个点在开启列表中计算G值，如果比原来的大，就什么都不做
      //否则设置它的父节点为当前点，并更新G和F
      else {
        int tempG = calcG(curPoint, targetpt);
        if (tempG < targetpt->G) {
          targetpt->parent = curPoint;
 
 
          targetpt->G = tempG;
          targetpt->F = calcF(targetpt);
        }
      }
 
 
      //判断快到终点后才开始计算
      if (isNearEndPoint) {
        CalcPt* resPoint = isInList(openList, &endPoint);
        //返回列表里的节点指针，不要用原来传入的endpoint指针，因为发生了深拷贝 
        if (resPoint)
          return resPoint;
      }
    }
  }
  return NULL;
}
 
 
 
 
//计算当前点周围八个点
vector<CalcPt*> AStarCalc::getSurroundPoints(const CalcPt* point, bool isIgnoreCorner) const
{
  vector<CalcPt*> surroundPoints;
 
 
  for (int x = point->pt.x - 1; x <= point->pt.x + 1; x++) {
    for (int y = point->pt.y - 1; y <= point->pt.y + 1; y++) {
      if (isCanreach(point, new CalcPt(cv::Point(x, y)), isIgnoreCorner)) {
        surroundPoints.push_back(new CalcPt(cv::Point(x, y)));
      }
    }
  }
 
 
  return surroundPoints;
}
 
 
 
 
//判断是否可能进行规划
bool AStarCalc::isCanreach(const CalcPt* point, const CalcPt* target, bool isIgnoreCorner) const
{
  //坐标小于0直接不计算了
  if (target->pt.x < 0 || target->pt.y < 0
    //计算的点与当前一致也不计算
    || target->pt.x == point->pt.x && target->pt.y == point->pt.y
    //判断点在障碍点中不计算
    || isInSites(&target->pt)
    //如果点在关闭列表中也不计算
    || isInList(closeList, target))
    return false;
  else {
    //如果是直线可以计算
    if (abs(point->pt.x - target->pt.x) + abs(point->pt.y - target->pt.y) == 1)
      return true;
    else {
      //斜线要判断当前直线会否被绊住，也是是说直线中是否在障碍点中
      cv::Point tmppt1 = cv::Point(point->pt.x, target->pt.y);
      cv::Point tmppt2 = cv::Point(target->pt.x, point->pt.y);
      if (!isInSites(&tmppt1) && !isInSites(&tmppt2))
        return true;
      else
        return isIgnoreCorner;
    }
  }
}
 
 
//判断开启/关闭列表中是否包含某点
CalcPt* AStarCalc::isInList(const std::list<CalcPt*>& list, const CalcPt* point) const
{
  //判断某个节点是否在列表中，这里不能比较指针，因为每次加入列表是新开辟的节点，只能比较坐标
  for (auto p : list)
    if (p->pt.x == point->pt.x && p->pt.y == point->pt.y)
      return p;
  return NULL;
}
 
 
//判断是否是障碍点
bool AStarCalc::isInSites(const cv::Point* point) const
{
  return sites[point->x][point->y] == 1;
}
 
 
//从开启列表中返回F值最小的节点 
CalcPt* AStarCalc::getLeastFpoint()
{
  if (!openList.empty())
  {
    auto resPoint = openList.front();
    for (auto& point : openList)
      if (point->F < resPoint->F)
        resPoint = point;
    return resPoint;
  }
  return NULL;
}
 
 
//计算G值
int AStarCalc::calcG(CalcPt* temp_start, CalcPt* point)
{
  //判断当前点与计算的原点是直线还是斜线，获取不同的消耗值
  //上下左右的点移动得到的值肯定是1，不为1的是斜线移动
  int tempG = abs(point->pt.x - temp_start->pt.x) + abs(point->pt.y - temp_start->pt.y) == 1 ? kCost1 : kCost2;
  //判断是不是初始的节点，如果是初始节约，则其父节点为空
  int parentG = point->parent == NULL ? 0 : point->parent->G;
  //两个G相加用于判断是走直线和斜线所消耗的总G
  return parentG + tempG;
}
 
 
//计算H值
int AStarCalc::calcH(CalcPt* point, CalcPt* end)
{
  //计算终点到当前点的Point差值
  cv::Point tmppoint = end->pt - point->pt;
  //利用欧几里德距离计算H
  return sqrt(pow(tmppoint.x, 2) + pow(tmppoint.y, 2)) * 10;
}
 
 
//计算F值
int AStarCalc::calcF(CalcPt* point)
{
  //公式 F=G+H
  return point->F = point->G + point->H;
}
 
 
//初始化地图
void AStarCalc::InitSites(vector<vector<int>> _sites)
{
  sites = _sites;
}
 
 
 
 
list<CalcPt*> AStarCalc::GetPath(CalcPt& startPoint, CalcPt& endPoint, vector<int>& resDistance, bool isIgnoreCorner)
{
  CalcPt* result = findPath(startPoint, endPoint, isIgnoreCorner);
  list<CalcPt*> path;
  //返回路径，如果没有找到路径，返回空链表
  if(result){
    resDistance.push_back(result->G);
  }
  while (result) {
    path.push_back(result);
    result = result->parent;
  }
  return path;
}