#pragma once

#include <vector>
#include <list>
#include <string>
#include <unordered_map>

const float COST1 = 1; // Cost of moving a single step horizontally or vertically
const float COST2 = 1.4; // COst of moving a single step diagnally 

// Construct the node
struct Point 
{
    int x, y; // point coordinates (states of the robot)

    float h, k; // h represents the path cost and k represents the smallest value of h since this certain point was placed on the open list 

    std::string tag; // tag of this point wihch is classified into "NEW", "OPEN", "CLOSED"

    Point* parent; // backpointer pointing to the parent node

    std::unordered_map<std::vector<int, int>, float> costDict; // store the arc cost of a path between the point and its neighbors, where key is coordinate, value is cost
 
    Point(float _x, float _y)
        : x(_x), y(_y), h(0), k(0), tag("NEW"), parent(NULL) {}   
    
};

// Construct Dstar Class
class Dstar
{
    public:
        void InitDstar(std::vector <std::vector<char> > &_maze);
        void PrepareRepair(std::list<Point*> &openList, std::list<Point*> &L, Point* Xc);
        std::list<Point*> InitPlan(std::list<Point*> &openList, std::list<Point*> &L, Point* Xc, Point* G); // Xc stands for currrent state, G stands for goal state
        std::list<Point*> RepairReplan(std::list<Point*> &openList, std::list<Point*> &L, Point* Xc, Point* G);
    
    private:
        void Insert(std::list<Point*> &openList, Point* X, float &h_new);
        void ModifyCost(std::list<Point*> &openList, Point* X, Point* Y, float &r_XY);
        float ProcessState(std::list<Point*> &openList, std::list<Point*> &L); 
        float SensorCost(Point* X, Point* Y);
        bool IsInRange(const Point* Xc, const Point* Y);
        Point* GetKMin(std::list<Point*> &openList);
        std::list<Point*> GetBackPointerList(const std::list<Point*> &L, Point* Xc, Point* G);
        std::list<Point*> SensedState(std::list<Point*> &L, Point* Xc);
        std::list<Point*> GetNeighbor(std::list<Point*> &L, Point* Xc);
        

    private:
        Point* Xc; // current state
        std::vector<std::vector<char> > maze;
        std::list<Point*> L; // list of all states
        std::list<Point*> openList;
        std::list<Point*> closedList; 
};