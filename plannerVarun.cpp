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
  vector<Node*> predecessors;
  Node* parent;
  pair<double, double> key;
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

class Planner
{
  private:
    // variables
    Node* currState; //the current position of the robot, in paper -> s_start 
    Node* lastState; //the last state robot was at
    unordered_map<vector<int>, Node*> graph; //maps a given x,y,z to its corresponding node. This is a container of all of the nodes allocated on the graph
    priority_queue<Node*, vector<Node*>, NodeComparator> U; //priority queue from paper
    km; // the update variable for the priorities which will constantly get updated
    u; // popped vertex from PQ

    // functions
    pair<double,double> CalculateKey(Node* state);
    double GetH(Node* state);
    vector<Node*> GetSuccessors(Node* state);
    vector<Node*> GetPredecessors(Node* state);
    void ComputeShortestPath();
    void Clear(); 
    void Main();

};

//As performed in paper
pair<double,double> Planner::CalculateKey(Node* state)
{
  double k1 = min(state->g, state->rhs) + GetH(state) + km;
  double k2 = min(state->g, state->rhs);
  return make_pair(k1,k2);
}

//For now just basic euclidean distance
double Planner::GetH(Node* state)
{
  double xDiff = state->x - currState->x;
  double yDiff = state->y - currState->y;
  double zDiff = state->z - currState->z;
  return sqrt(pow(xDiff,2) + pow(yDiff,2) + pow(zDiff,2));
}


void Planner::Initialize()
{
  km = 0;
  goalState -> rhs = 0;
  vector<int> pos { 10, 20, 30 };
  graph.insert(make_pair(pos, goalState));
  U.insert(goalState, calculateKey(goalState));
}


void Planner::Clear()
{
  //clear all variables in here at the end of search in order to prepare for next search
}

//Right now only set up for 2D case. Thought it would be best to finalize how we want to define how we know if a state can change z or not
vector<Node*> Planner::getSuccessors(Node* state)
{

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


vector<Node*> Planner::GetPredecesors(Node* state)
{
  //get predecessors within this function
}



void Planner::UpdateVertex(Node* state)
{
  //update vertex within this function
}


void Planner::ComputeShortestPath()
{
  while (U.top->key < CalculateKey(Node* currState || currState -> rhs != currState -> g))
  {
    pair<double, double> k_old = U.top -> key;
    u = U.top;
    U.pop;

    pair<double, double> k_new = CalculateKey(u);

    
    if (k_old < k_new) // condition 1 -> not a lower bound yet
    {
      U.insert(u, k_new);
    }
    else if (u -> g > u -> rhs) // condition 2 -> already a lower bound, just update cost and expand
    {
      u -> g = u -> rhs;
      vector<Node*> pred = GetPredecesors(u); // update the predecessors parameter of struct
      for (auto x: u -> predecessors)
      {
        UpdateVertex(x);
      }
    }
    else // condition 3 -> already a lower bound, just update cost and expand
    {
      u -> g = INT_MAX;
      vector<Node*> pred = GetPredecesors(u); // update the predecessors parameter of struct
      for (auto x: u -> predecessors)
      {
        UpdateVertex(x);
      }
      UpdateVertex(u) // for this condition, need to update the actual vertex itself
    }
  }
}



// main loop for the planner
void Planner::Main()
{

  // 1. Set up node pointer for the starting position
  currState = new Node();
  currState -> x = 0; //temporary
  currState -> y = 0; //temporary
  currState -> z = 0; //temporary
  currState -> parent = nullptr;
  currState -> key = calculateKey(currState);


  // 2. Set up s_last = s_start -> update the last starting point of robot to be the new starting point of robot to rerun the planning algo
  lastState = currState;

  // 3. Initialize goal and km -> insert into pq
  Initialize();


  // 4. Call on ComputeShortestPath to get generic A* path at the beginning from goal to start
  ComputeShortestPath();


  // 5. Main while loop for search


    // 6. Check if currState -> g = INT_MAX, if so then no solution

    // 7. Update currState to be newest state from successors of previous start state (now there will be a difference between last state and curr state)

    // 8. Make new currState the new starting point for robot

    // 9. Check all around graph for edge costs (easier in our case with knowledge of when cost map will change)

    // 10. If cost change detected...

      // 11. Add on to km -> constantly adding on to km to make priorities lower bounds of LPA*

      // 12. Update lastState = currState since robot is going to have a new start state soon

      // 13. Iterate over all edges that had a change in edge costs (again easier in our case since we know what edges will change)

        // 14. Update edge cost c(u,v) (in our case update cell cost)

        // 15. Update the vertex u

      // 16. Compute shortest path from the goal state to the start state
}


int main()
{

  Planner planner;

  return 0;
}



