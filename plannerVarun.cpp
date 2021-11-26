#include <iostream>
#include <vector>
#include <map>
#include <utility>
#include <math.h>
#include <queue>
#include <limits.h>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <set>

using namespace std;

template <typename T>
std::string to_string_with_precision(const T a_value, const int n = 0)
{
    std::ostringstream out;
    out.precision(n);
    out << std::fixed << a_value;
    return out.str();
}

class CostMap{
private:
  vector<vector<vector<string>>> symbolMap;
  map<int,vector<vector<double>> > costChangeLookUp; //stores the location and valuechange to made to each cell in the costMap at each time t:
  int t = 0;
  bool isCostChange = false;

public:
  vector<vector<vector<double>>> valueMap;

  void Move(vector<int> position){
    int x = position[0];
    int y = position[1];
    int z = position[2];
    symbolMap[z][x][y] = "*";//will indicate the actual taken path
  }

  double GetValueAtCell(double x, double y, double z){
    return valueMap[z][x][y];
  }

  vector<vector<double>> GetCurrentCostChanges(){
    return costChangeLookUp[t];
  }

  //through cost changes at new timestep and update map accordingly
  void printSymbolMap(){
    for(int i = 0; i<symbolMap[0].size(); i++){

      for(int j = 0; j<symbolMap[0][0].size(); j++){
        cout << symbolMap[0][i][j] << ", ";
      }
      cout<<endl;
    }
  }
  bool checkCostChange(){
    return isCostChange;
  }

  void UpdateCostMap(int t){

    auto currentCostChanges = costChangeLookUp[t];
    for(int s = 0; s<currentCostChanges.size(); s++){
      auto costChangeInfo = currentCostChanges[s]; //Takes form: [x,y,z,newCost]
      int i = costChangeInfo[0];
      int j = costChangeInfo[1];
      int k = costChangeInfo[2];
      double newCost = costChangeInfo[3];
      valueMap[i][j][k] = newCost;
      if(symbolMap[i][j][k] != "*") symbolMap[i][j][k] = to_string_with_precision(newCost);
    }
  }

  void InitializeCostChangeLookUp(string fileName){

    ifstream file(fileName);
    string line;
    getline(file, line);
    while(file.good()){

      getline(file, line);
      stringstream ss(line);

      int t_local;
      double i,j,k,costChange;

      ss >> t_local;
      if(ss.peek() == ',') ss.ignore();
      ss >> i;
      if(ss.peek() == ',') ss.ignore();
      ss >> j;
      if(ss.peek() == ',') ss.ignore();
      ss >> k;
      if(ss.peek() == ',') ss.ignore();
      ss >> costChange;

      vector<double> costChangeInfo = {i,j,k,costChange};
      costChangeLookUp[t_local].push_back(costChangeInfo);
    }
  }

  void InitializeFloor(string fileName, int floorNumber){
      vector<vector<double>> floorCosts;
      vector<vector<string>> floorSymbols;
      ifstream file(fileName);
      string line;
      getline(file, line);
      while(file.good())
      {
          getline(file, line);
          const auto pos = line.find(',');
          if(pos != string::npos)
              line.erase(0, pos+1);
          stringstream ss(line);

          double val;
          vector<double> row;
          vector<string> rowSymbols;
          while(ss >> val){
              row.push_back(val);
              rowSymbols.push_back(to_string_with_precision(val));
              // If the next token is a comma, ignore it and move on
              if(ss.peek() == ',') ss.ignore();
          }
          floorCosts.push_back(row);
          floorSymbols.push_back(rowSymbols);
      }
      valueMap.push_back(floorCosts);
      symbolMap.push_back(floorSymbols);

  }

  void Initialize(){
    string mapFileName = "Maps/example_map/example_map_initial.csv";
    InitializeFloor(mapFileName, 0);
    string costChangeFileName = "Maps/example_map/example_costChanges_t1.csv";
    InitializeCostChangeLookUp(costChangeFileName);
  }

  void TickTime(){
    // std::cout << __FUNCTION__ << __LINE__ << std::endl;

    t++;
    if(costChangeLookUp.count(t)>0){
      isCostChange = true;
      UpdateCostMap(t);
    }
    else{
      isCostChange = false;
    }
  }
};


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
      bool operator()(const Node* a, const Node* b) const
      {
          if((a->key).first != (b->key).first){
            return ((a->key).first > (b->key).first);
          }

          return ((a->key).second > (b->key).second);
      }
};

void PQ_Pop(set<Node*, NodeComparator> &U){
    auto to_pop = *U.rbegin();
    U.erase(to_pop);
}

Node* PQ_Top(set<Node*, NodeComparator> &U){
    auto at_top = *U.rbegin();
    return at_top;
}

class Planner
{
  public:
    Planner(vector<int> goalPosition);
    void Main();

  private:
    // variables
    Node* currState = new Node; //the current position of the robot, in paper -> s_start
    Node* lastState = new Node; //the last state robot was at
    Node* goalState = new Node; //the user defined goal state
    map<vector<int>, Node*> graph; //maps a given x,y,z to its corresponding node. This is a container of all of the nodes allocated on the graph
    set<Node*, NodeComparator> U;
    //priority_queue<Node*, vector<Node*>, NodeComparator> U; //priority queue from paper
    // map<vector<int>, Node*> U_copy; //copy of PQ used to check if element exists in PQ
    double km; //the update variable for the priorities which will constantly get updated
    Node* u; //popped vertex from PQ
    CostMap costMap;

    // functions
    pair<double,double> CalculateKey(Node* state);
    double GetH(Node* state);
    double GetCostOfTravel(Node* state, Node* succ);
    vector<Node*> GetNeighbors(Node* state);
    void ComputeShortestPath();
    void UpdateVertex(Node* state);
    void Initialize();
    void Clear();

};


Planner::Planner(vector<int> goalPosition)
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

  double stateX = (state->position)[0];
  double stateY = (state->position)[1];
  double stateZ = (state->position)[2];


  double currStateX = (currState->position)[0];
  double currStateY = (currState->position)[1];
  double currStateZ = (currState->position)[2];

  double xDiff = stateX - currStateX;
  double yDiff = stateY - currStateY;
  double zDiff = stateZ - currStateZ;

  return sqrt(pow(xDiff,2) + pow(yDiff,2) + pow(zDiff,2));
}


double Planner::GetCostOfTravel(Node* state, Node* succ)
{
  //for now leave as static cost -> change later based on cost map addition
  auto x_succ = succ->position[0];
  auto y_succ = succ->position[1];
  auto z_succ = succ->position[2];
  return costMap.GetValueAtCell(x_succ,y_succ,z_succ);
}


void Planner::Initialize()
{
  //U = 0 : emptying of PQ will occur at end of D* complete run, so this will be taken care of in Clear()
  km = 0;
  goalState -> rhs = 0;
  goalState -> position = {6,14,0}; //temporary position for goal state, will be user entered eventually
  goalState-> key = CalculateKey(goalState);
  graph.insert(make_pair(goalState -> position, goalState));
  U.insert(goalState); //insert goal into PQ -> will be overconsistent
  // U_copy.insert(make_pair(goalState->position, goalState));

  //costMap
  costMap.Initialize();
}


void Planner::Clear()
{
  //clear all variables in here at the end of full D* search right before while loop breaks once goal is reached

  // 1. Clear out priority queue
  while(!U.empty())
  {
    PQ_Pop(U);
  }

  // 2. Clear out graph pointers
  for (auto x: graph)
  {
    delete x.second; //delete pointer to node (not sure if this is correct way of deleting)
    graph.erase(x.first); //clears iterator to first element
  }


}

//Right now only set up for 2D case. Thought it would be best to finalize how we want to define how we know if a state can change z or not
vector<Node*> Planner::GetNeighbors(Node* state)
{
  //check all 8 2d successors
  //some struct that can check z in a txt file
  //cout<<"x : " << state->position[0] << " y: " << state->position[1] << " z: " << state->position[2] << endl;

  double collision_flag = -1; //just here for now
  int z_size = costMap.valueMap.size();
  int x_size = costMap.valueMap[0].size();
  int y_size = costMap.valueMap[0][0].size();

  //this node has been allocated and had its successors already generated as well. Just return them
  if(state->neighbors.size() != 0){
    return state->neighbors;
  }

  const int NUMOFDIRS = 8;
  int dX[NUMOFDIRS] = {-1, -1, -1,  0,  0,  1, 1, 1};
  int dY[NUMOFDIRS] = {-1,  0,  1, -1,  1, -1, 0, 1};

  for(int i = 0; i<NUMOFDIRS; i++){

    int x2 = state->position[0]+dX[i];
    int y2 = state->position[1]+dY[i];
    int z2 = state->position[2]; //not moving z for now...
    //If we've already allocated a node for this location just add a pointer to that

    //cout<<"x2 : " << x2 << " y2: " << y2 << " z2: " << z2 << endl;
    if(graph.count({x2,y2,z2}) > 0){

      state->neighbors.push_back(graph[{x2,y2,z2}]);
    }
    //Check if location is a valid successor, if so allocate a new node for it and add it to the overall graph
    else if(x2 >= 0 && x2 < x_size && y2 >= 0 && y2< y_size && z2 >= 0 && z2 <= z_size && costMap.valueMap[z2][x2][y2] != collision_flag){


      Node* newNode = new Node;
      newNode->position = {x2,y2,z2};
      newNode->g = INT_MAX;
      newNode->rhs = INT_MAX;
      state->neighbors.push_back(newNode);
      graph[{x2,y2,z2}] = newNode;

    }

  }

  return state->neighbors;
}



void Planner::UpdateVertex(Node* state)
{
  //update vertex within this function
  // std::cout << __FUNCTION__ << __LINE__ << std::endl;

  // cout << (state->g != state->rhs);

  // std::cout << __FUNCTION__ << __LINE__ << std::endl;


  //condition that this state is not goal state
  if (state->position != goalState->position)
  {
    // std::cout << __FUNCTION__ << __LINE__ << std::endl;

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
  if (U.find(state) != U.end()) //state exists in PQ
  {
    // std::cout << __FUNCTION__ << __LINE__ << std::endl;

    //remove state from PQ
    U.erase(state);
  }

  //condition that state is overconsistent
  if (state->g != state->rhs)
  {
    // std::cout << __FUNCTION__ << __LINE__ << std::endl;

    state->key = CalculateKey(state);
    U.insert(state);
  }

}




void Planner::ComputeShortestPath()
{
  while (PQ_Top(U)->key < CalculateKey(currState) || currState -> rhs != currState -> g)
  {


    pair<double, double> k_old = PQ_Top(U) -> key;
    u = PQ_Top(U);
    PQ_Pop(U);
    // U_copy.erase(u->position);
    pair<double, double> k_new = CalculateKey(u); //the first time this is run, should be the same as popped since km = 0

    if (k_old < k_new) // condition 1 -> not a lower bound yet
    {

      u->key = k_new; // update the key of specific state within node -> change reflected in graph
      U.insert(u); // reinsert into PQ with new key priority
      // U_copy.insert(make_pair(u->position, u));

    }
    else if (u -> g > u -> rhs) // condition 2 -> already a lower bound, just update cost and expand
    {

      u -> g = u -> rhs; // make consistent
      auto neighbors = GetNeighbors(u); // update the neighbors parameter of struct ***LOCATION OF SEG FAULT

      for (auto x: neighbors)
      {

        UpdateVertex(x);

      }
    }
    else // condition 3 -> already a lower bound, just update cost and expand
    {


      u -> g = INT_MAX;
      auto neighbors = GetNeighbors(u); // update the predecessors parameter of struct
      for (auto x: neighbors)
      {
        UpdateVertex(x);
      }

      UpdateVertex(u); // for this condition, need to update the actual vertex itself
    }


  }

}



// main loop for the planner
void Planner::Main()
{

  // 1. Set up node pointer for the starting position
  currState -> position = {14,1,0}; //starting position from example map
  currState -> key = CalculateKey(currState);

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
  costMap.Move(currState->position);
  costMap.printSymbolMap();
  costMap.TickTime();


  // 5. Main while loop for search
  while(currState->position != goalState->position)
  {
    cout << (currState->position)[0] << ", " << (currState->position)[1] << ", " << (currState->position)[2] << endl;
    // 6. Check if currState -> g = INT_MAX, if so then no solution
    if (currState -> g == INT_MAX)
    {
      cout << "No solution found!" << endl;
      break;
    }

    // 7. Update currState to be newest state from successors of previous start state (now there will be a difference between last state and curr state)

    double minCostSucc = 10000; //set to something really large
    double costSucc = 0;
    cout << "Size of Neighbors: " << (currState->neighbors).size() << endl;
    for (auto x: currState->neighbors)
    {
      costSucc = x->g + GetCostOfTravel(currState, x);
      cout << "Neighbor Position: " << endl;
      cout << (x->position)[0] << ", " << (x->position)[1] << ", " << (x->position)[2] << endl;
      cout << "Cost of Neighbor: " << costSucc << endl;

      if (costSucc < minCostSucc)
      {
        minCostSucc = costSucc;
        currState = x;
      }
    }

    cout << "Neighbor Chosen: " << endl;
    cout << (currState->position)[0] << ", " << (currState->position)[1] << ", " << (currState->position)[2] << endl;




    // 8. Make new currState the new starting point for robot -> visualization of robot moving here
    costMap.Move(currState->position);

    // 9. Check all around graph for edge costs (easier in our case with knowledge of when cost map will change)

    // 10. If cost change detected...
    if(costMap.checkCostChange()){

      // 11. Add on to km -> constantly adding on to km to make priorities lower bounds of LPA*
      km += GetH(lastState);

      // 12. Update lastState = currState since robot is going to have a new start state soon
      lastState->position = currState->position;
      lastState->g = currState->g;
      lastState->rhs = currState->rhs;
      lastState->f = currState->f;
      lastState->neighbors = currState->neighbors;
      lastState->key = currState->key;



      // 13. Iterate over all edges that had a change in edge costs (again easier in our case since we know what edges will change)


        // 14. Update edge cost c(u,v) (in our case update cell cost)
        //This is done in TickTime automatically

        // 15. Update the vertex u
        auto costChanges = costMap.GetCurrentCostChanges();

        for(auto costChange : costChanges){

          int x = costChange[1];
          int y = costChange[2];
          int z = costChange[0];
          auto valChange = costChange[3];
          if(graph.count({x,y,z}) > 0){
            auto node2Change = graph[{x,y,z}];
            //cout <<"x : " << x << ", y: " << y << ", z: " << z << ", valChange: " << valChange << endl;
            UpdateVertex(node2Change);
          }

        }

      // 16. Compute shortest path from the goal state to the start state
      ComputeShortestPath();

    }

    costMap.TickTime();
    costMap.printSymbolMap();

  }

  // 17. Clear out graph and PQ

  Clear();


}


int main()
{

  vector<int> goal = {6,14,0}; //goal in example map for now
  Planner planner(goal);
  planner.Main();

  return 0;
}
