#include <iostream>
#include <vector>
#include <map>
#include <utility>
#include <math.h>
#include <queue>
#include <limits.h>

using namespace std;

struct Node{
  int x;
  int y;
  int z;
  double g = INT_MAX; //infinity
  double rhs = INT_MAX; //infinity
  double f;
  vector<Node*> successors;
  Node* parent;
  double k1;
  double k2;
};

//Implementing key comparision logic defined in page 3 of paper
class NodeComparator {
  public:
      bool operator()(const Node* a, const Node* b)
      {
          if(a->k1 != b->k1){
            return (a->k1 > b->k1);
          }

          return (a->k2 > b->k2);
      }
};

class Planner{
  public:
    Node* currentStart; //prefix current used because start will move based on where robot is
    map<vector<int>, Node*> loc2Node; //maps a given x,y,z to its corresponding node. This is a container of all of the nodes allocated on the graph
    priority_queue<Node*, vector<Node*>, NodeComparator> PQ; //Aliused to captial U (the priority queue) in paper
    vector<vector<vector<double>>> costMap; //stores cost of traversing to cell x,y,z in costMap[x][y][z]
    vector<vector<vector<double>>> allCostChanges; //stores changes to made to costMap at each time t:
      //Set of all changes made at time t: costChanges[t]
      //Location and cost change for a state s at a given t: costChanges[t][s]
      //CostChanges[t][s] takes form: [x,y,z,newCost]
    int t = 0; //Will be iterated each time we want the map to change costs

    pair<double,double> CalculateKey(Node* state);
    double GetH(Node* state);
    void UpdateCostMap();
    vector<Node*> GetSuccessors(Node* state);

};

//As performed in paper
pair<double,double> Planner::CalculateKey(Node* state){
  double k1 = min(state->g, state->rhs) + GetH(state);
  double k2 = min(state->g, state->rhs);
  return {k1,k2};
}

//For now just basic euclidean distance
double Planner::GetH(Node* state){
  double xDiff = state->x - currentStart->x;
  double yDiff = state->y - currentStart->y;
  double zDiff = state->z - currentStart->z;
  return sqrt(pow(xDiff,2) + pow(yDiff,2) + pow(zDiff,2));
}

//through cost changes at new timestep and update map accordingly
void Planner::UpdateCostMap(){
  if(t < allCostChanges.size()-1){
    t++;
  }
  else t = 0; //just restart cost change trajectory

  auto currentCostChanges = allCostChanges[t];
  for(int s = 0; s<currentCostChanges.size(); s++){
    auto costChange = currentCostChanges[s]; //Takes form: [x,y,z,newCost]
    int x = costChange[0];
    int y = costChange[1];
    int z = costChange[2];
    double newCost = costChange[3];
    costMap[x][y][z] = newCost;
  }
}

//Right now only set up for 2D case. Thought it would be best to finalize how we want to define how we know if a state can change z or not
vector<Node*> Planner::GetSuccessors(Node* state){

  double collision_flag = -1; //just here for now
  int x_size = costMap.size();
  int y_size = costMap[0].size();
  int z_size = costMap[0][0].size();

  //this node has been allocated and had its successors already generated as well. Just return them
  if(state->successors.size() != 0){

    return state->successors;
  }

  const int NUMOFDIRS = 8;
  int dX[NUMOFDIRS] = {-1, -1, -1,  0,  0,  1, 1, 1};
  int dY[NUMOFDIRS] = {-1,  0,  1, -1,  1, -1, 0, 1};

  for(int i = 0; i<NUMOFDIRS; i++){
    int x2 = state->x+dX[i];
    int y2 = state->y+dY[i];
    int z2 = state->z; //not moving z for now...

    //If we've already allocated a node for this location just add a pointer to that
    if(loc2Node.count({x2,y2,z2}) > 0){
      state->successors.push_back(loc2Node[{x2,y2,z2}]);
    }
    //Check if location is a valid successor, if so allocate a new node for it and add it to the overall loc2Node map
    else if(x2 >= 0 && x2<=x_size && y2 >= 0 && y2<=y_size && z2 >= 0 && z2 <= z_size && costMap[x2][y2][z2] != collision_flag){
      Node* newNode = new Node;
      newNode->x = x2;
      newNode->y = y2;
      newNode->g = INT_MAX;
      newNode->rhs = INT_MAX;
      state->successors.push_back(newNode);
      loc2Node[{x2,y2,z2}] = newNode;
    }

  }

  return state->successors;
}


int main(){

  Planner planner;

  return 0;
}
