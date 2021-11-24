#include <iostream>
#include <vector>
#include <map>
#include <utility>
#include <math.h>
#include <queue>
#include <limits.h>

using namespace std;

struct Node{
  vector<int> position; //contains x,y,z position of the state
  double g = INT_MAX; //infinity
  double rhs = INT_MAX; //infinity
  double f;
  vector<Node*> neighbors;
  pair<double, double> key;
};



//Implementing key comparision logic defined in page 3 of paper
class NodeComparator {
  public:
      bool operator()(const Node* a, const Node* b)
      {
          if((a->key).first != (b->key).first){
            return ((a->key).first > (b->key).first);
          }

          return ((a->key).second > (b->key).second);
      }
};

class Planner
{
  private:
    // variables
    Node* currState = new Node; //the current position of the robot, in paper -> s_start 
    Node* lastState = new Node; //the last state robot was at
    Node* goalState = new Node; //the user defined goal state
    Node* dummyState = new Node; //the dummy state used for easy deletion from PQ
    map<vector<int>, Node*> graph; //maps a given x,y,z to its corresponding node. This is a container of all of the nodes allocated on the graph
    priority_queue<Node*, vector<Node*>, NodeComparator> U; //priority queue from paper
    map<vector<int>, Node*> U_copy; //copy of PQ used to check if element exists in PQ
    km; //the update variable for the priorities which will constantly get updated
    u; //popped vertex from PQ

    // functions
    void Planner(vector<int> goalPosition);
    pair<double,double> CalculateKey(Node* state);
    double GetH(Node* state);
    double GetCostOfTravel(Node* state, Node* succ);
    void GetNeighbors(Node* state);
    void ComputeShortestPath();
    void UpdateVertex(Node* state);
    void DeleteFromPQ(Node* state);
    void Clear(); 
    void Main();

};


void Planner::Planner(vector<int> goalPosition)
{
  goalState -> position = goalPosition;
  graph.insert(make_pair(goalPosition, goalState));
}


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
  stateX = (state->position)[0];
  stateY = (state->position)[1];
  stateZ = (state->position)[2];

  currStateX = (currState->position)[0];
  currStateY = (currState->position)[1];
  currStateZ = (currState->position)[2];

  double xDiff = stateX - currStateX;
  double yDiff = stateY - currStateY;
  double zDiff = stateZ - currStateZ;
  return sqrt(pow(xDiff,2) + pow(yDiff,2) + pow(zDiff,2));
}


double Planner::GetCostOfTravel(Node* state, Node* succ)
{
  //for now leave as static cost -> change later based on cost map addition
  return 1;
}


void Planner::Initialize()
{
  //U = 0 : emptying of PQ will occur at end of D* complete run, so this will be taken care of in Clear()
  km = 0;
  goalState -> rhs = 0;
  goalState -> position = {1000, 200, 3}; //temporary position for goal state, will be user entered eventually
  goalState-> key = calculateKey(goalState);
  graph.insert(make_pair(goalState -> position, goalState));
  U.push(goalState); //insert goal into PQ -> will be overconsistent
  U_copy.insert(make_pair(goalState->position, goalState));

  //initialize dummy node
  dummyState->key = make_pair(-1,-1);
}


void Planner::Clear()
{
  //clear all variables in here at the end of full D* search right before while loop breaks once goal is reached

  // 1. Clear out priority queue
  while(!U.empty())
  {
    U.pop();
  }

  // 2. Clear out graph pointers
  for (auto x: graph)
  {
    delete x->second; //delete pointer to node
    x = graph.erase(x); //clears iterator to first element
  }


}

//Right now only set up for 2D case. Thought it would be best to finalize how we want to define how we know if a state can change z or not
void Planner::GetNeighbors(Node* state)
{
  //check all 8 2d successors
  //some struct that can check z in a txt file


  double collision_flag = -1; //just here for now
  int x_size = costMap.size();
  int y_size = costMap[0].size();
  int z_size = costMap[0][0].size();

  //this node has been allocated and had its successors already generated as well. Just return them
  if(state->neighbors.size() != 0){

    return state->neighbors;
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
      state->neighbors.push_back(loc2Node[{x2,y2,z2}]);
    }
    //Check if location is a valid successor, if so allocate a new node for it and add it to the overall loc2Node map
    else if(x2 >= 0 && x2<=x_size && y2 >= 0 && y2<=y_size && z2 >= 0 && z2 <= z_size && costMap[x2][y2][z2] != collision_flag){
      Node* newNode = new Node;
      newNode->x = x2;
      newNode->y = y2;
      newNode->g = INT_MAX;
      newNode->rhs = INT_MAX;
      state->neighbors.push_back(newNode);
      loc2Node[{x2,y2,z2}] = newNode;
    }

  }
}


void Planner::DeleteFromPQ(Node* state)
{
  //Purely remove a state from PQ without affecting state parameters elsewhere

  auto k_old = state->key;
  state->key = make_pair(0,0); //in order to push to top of PQ (right after dummy State) 
  U.push(dummyState);
  U.pop(); //pop dummy state AND reorder PQ -> makes state to get deleted top now
  U.pop(); //pop desired state to get deleted
  U_copy.erase(state->position); //erase from U_copy;
  state -> key = k_old; //reinsert old key into state so as to not affect it elsewhere

}



void Planner::UpdateVertex(Node* state)
{
  //update vertex within this function

  //condition that this state is not goal state
  if (state->position != goalState->position)
  {
    //update rhs(u) to be min cost of travel + g among successors of state
    double minSucc = 10000; //set to something really large
    double costSucc = 0;
    for (auto x: state->neighbors)
    {
      costSucc = x->g + GetCostOfTravel(state, x); 

      if (costSucc < minSucc)
      {
        minSucc = costSucc;
      }
    }

    state->rhs = minSucc;

  }

  //condition that state exists in PQ -> remove u from PQ
  if (U_copy.find(state->position) != U_copy.end()) //state exists in PQ
  {
    //remove state from PQ
    DeleteFromPQ(state);
  }

  //condition that state is overconsistent
  if (state->g != state->rhs)
  {
    state->key = CalculateKey(state);
    U.push(state);
    U_copy.insert(make_pair(state->position, state));
  }

}




void Planner::ComputeShortestPath()
{
  while (U.top->key < CalculateKey(Node* currState) || currState -> rhs != currState -> g)
  {
    pair<double, double> k_old = U.top -> key;
    u = U.top();
    U.pop();
    U_copy.erase(u->position);

    pair<double, double> k_new = CalculateKey(u); //the first time this is run, should be the same as popped since km = 0

    
    if (k_old < k_new) // condition 1 -> not a lower bound yet
    {
      u->key = k_new; // update the key of specific state within node -> change reflected in graph
      U.push(u); // reinsert into PQ with new key priority
      U_copy.insert(make_pair(u->position, u))
    }
    else if (u -> g > u -> rhs) // condition 2 -> already a lower bound, just update cost and expand
    {
      u -> g = u -> rhs; // make consistent
      GetNeighbors(u); // update the neighbors parameter of struct
      for (auto x: u -> neighbors)
      {
        UpdateVertex(x);
      }
    }
    else // condition 3 -> already a lower bound, just update cost and expand
    {
      u -> g = INT_MAX;
      GetNeighbors(u); // update the predecessors parameter of struct
      for (auto x: u -> neighbors)
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
  currState -> position = {0,0,0}; //temporary x,y,z starting position
  currState -> key = calculateKey(currState);

  // insert location and node within graph
  graph.insert(make_pair(currState -> position, currState));


  // 2. Set up s_last = s_start -> update the last starting point of robot to be the new starting point of robot to rerun the planning algo
  lastState->position = currState->position;
  lastState->g = currState->g;
  lastState->rhs = currState->rhs;
  lastState->f = currState->f;
  lastState->neighbors = currState->neighbors;
  lastState->key = currState->key;


  // 3. Initialize goal and km -> insert into pq
  Initialize();


  // 4. Call on ComputeShortestPath to get generic A* path at the beginning from goal to start
  ComputeShortestPath();


  // 5. Main while loop for search
  while(currState->position != goalState->position)
  {
    // 6. Check if currState -> g = INT_MAX, if so then no solution
    if (currState -> g == INT_MAX)
    {
      cout << "No solution found!" << endl;
      break;
    }

    // 7. Update currState to be newest state from successors of previous start state (now there will be a difference between last state and curr state)

    double minCostSucc = 10000; //set to something really large
    double costSucc = 0;
    for (auto x: currState->neighbors)
    {
      costSucc = x->g + GetCostOfTravel(currState, x); 

      if (costSucc < minCostSucc)
      {
        minCostSucc = costSucc;
        currState = x;
      }
    }

    // 8. Make new currState the new starting point for robot -> visualization of robot moving here



    // 9. Check all around graph for edge costs (easier in our case with knowledge of when cost map will change)

    // 10. If cost change detected...

      // 11. Add on to km -> constantly adding on to km to make priorities lower bounds of LPA*
      km += GetH(lastState, currState);

      // 12. Update lastState = currState since robot is going to have a new start state soon
      lastState->position = currState->position;
      lastState->g = currState->g;
      lastState->rhs = currState->rhs;
      lastState->f = currState->f;
      lastState->neighbors = currState->neighbors;
      lastState->key = currState->key;



      // 13. Iterate over all edges that had a change in edge costs (again easier in our case since we know what edges will change)

        // 14. Update edge cost c(u,v) (in our case update cell cost)

        // 15. Update the vertex u

      // 16. Compute shortest path from the goal state to the start state
      ComputeShortestPath();
  }

}


int main()
{

  Planner planner;

  return 0;
}



