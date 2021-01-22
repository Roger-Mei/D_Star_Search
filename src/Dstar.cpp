#include "Dstar.h"
#include <algorithm>

void Dstar::InitDstar(std::vector <std::vector<char> > &_maze){

    // Initialize maze
    maze = _maze;

    // Extract maze size
    int row = maze.size();
    int col = maze[0].size();

    // Initialize list of all states L
    for (int i = 0; i < row; ++i){
        for (int j = 0; j < col; ++j){
            L.push_back(new Point(i, j));
        }
    }
}

 void Dstar::PrepareRepair(std::list<Point*> &openList, std::list<Point*> &L, Point* Xc){

     // Get the states within the sensing range of current state Xc
     std::list<Point*> sGroup1 = SensedState(L, Xc);

     for (auto X : sGroup1){
         // Get the neighbor of X
         std::list<Point*> neighbor = GetNeighbor(L, X);
         for (auto Y : neighbor){
            float r1 = SensorCost(Y, X);
            if (r1 != Y->costDict[{X->x, X->y}]){
                ModifyCost(openList, Y, X, r1);
            }
         }

         for (auto Y : neighbor){
            float r2 = SensorCost(X, Y);
            if (r2 != X->costDict[{Y->x, Y->y}]){
                ModifyCost(openList, Y, X, r2);
            }
         }
         
     }
 }

 std::list<Point*> Dstar::InitPlan(std::list<Point*> &openList, std::list<Point*> &L, Point* Xc, Point* G){

     while (true){
         float k_min = ProcessState(openList, L);
         if (k_min == -1 || Xc->tag == "CLOSED") break;
     }

     std::list<Point*> P = GetBackPointerList(L, Xc, G);

     return P;
 }

std::list<Point*> Dstar::RepairReplan(std::list<Point*> &openList, std::list<Point*> &L, Point* Xc, Point* G){

    while(true){
        float k_min = ProcessState(openList, L);
        if (k_min >= Xc->h || k_min == -1) break;
    }

    std::list<Point*> P = GetBackPointerList(L, Xc, G);

    return P;
}

void Dstar::Insert(std::list<Point*> &openList, Point* X, float &h_new){

    if (X->tag == "NEW"){
        X->k = h_new;
    }
    else if (X->tag == "OPEN")
    {
        X->k = std::min(X->k, h_new);
    }
    else if (X->tag == "CLOSED"){
        X->k = std::min(X->h, h_new);
    }

    X->h = h_new;
    X->tag = "OPEN";

    openList.push_back(X);

    // Sort openlist based on increasing k values
    openList.sort([&](Point* A, Point* B){return A->k < B->k;});
}

void Dstar::ModifyCost(std::list<Point*> &openList, Point* X, Point* Y, float &cval){

    X->costDict[{Y->x, Y->y}] = cval;

    if (X->tag == "CLOSED") Insert(openList, X, X->h);
}

float Dstar::ProcessState(std::list<Point*> &openList, std::list<Point*> &L){

    // retrieve the state with minimum k value
    Point* X = GetKMin(openList);
    if (!X) return -1;
    float k_old = X->k;

    // delete X from open List
    openList.pop_front();

}