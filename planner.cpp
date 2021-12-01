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

//std::cout << __FUNCTION__ << __LINE__ << std::endl;


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

  vector<vector<vector<double>>> valueMap = vector<vector<vector<double>>>(11, vector<vector<double>>(100, vector<double>(100)));
  vector<vector<int>> path;
  void Move(vector<int> position){
    int x = position[0];
    int y = position[1];
    int z = position[2];
    // symbolMap[z][x][y] = "*";//will indicate the actual taken path
    path.push_back({x,y,z});

  }

  double GetValueAtCell(double x, double y, double z){
    std::cout << __FUNCTION__ << __LINE__ << std::endl;
    cout << "POS: " << x << ", " << y << ", " << z << endl;
    auto temp = valueMap[z][x][y];
    std::cout << __FUNCTION__ << __LINE__ << std::endl;
    return valueMap[z][x][y];
  }

  vector<vector<double>> GetCurrentCostChanges(){
    return costChangeLookUp[t];
  }

  //through cost changes at new timestep and update map accordingly
  // void printSymbolMap(){
  //   cout << "Map at t = " << t << endl;
  //   for(int i = 0; i<symbolMap[0].size(); i++){

  //     for(int j = 0; j<symbolMap[0][0].size(); j++){
  //       cout << symbolMap[0][i][j] << ", ";
  //     }
  //     cout<<endl;
  //   }
  //   cout<<endl;
  // }


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
      // if(symbolMap[i][j][k] != "*") symbolMap[i][j][k] = to_string_with_precision(newCost);
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
      //cout<< "tlocal: " << t_local << endl;
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
          //cout << "line :" <<line << endl;
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
      // valueMap.push_back(floorCosts);
      valueMap[floorNumber] = floorCosts;
      symbolMap.push_back(floorSymbols);

  }

  void Initialize(){
    string mapFileName = "Maps/Stadium/costmap.csv";
    InitializeFloor(mapFileName, 0);

    mapFileName = "Maps/Stadium/costmap.csv";
    InitializeFloor(mapFileName, 2);

    mapFileName = "Maps/Stadium/costmapLv2.csv";
    InitializeFloor(mapFileName, 4);

    mapFileName = "Maps/Stadium/costmapLv3.csv";
    InitializeFloor(mapFileName, 6);

    mapFileName = "Maps/Stadium/costmapLv4.csv";
    InitializeFloor(mapFileName, 8);

    mapFileName = "Maps/Stadium/costmapLv5.csv";
    InitializeFloor(mapFileName, 10);

    string costChangeFileName = "Maps/example_map/allCostChangest_test.csv";
    InitializeCostChangeLookUp(costChangeFileName);

  }

  void TickTime(){
    std::cout << __FUNCTION__ << __LINE__ << std::endl;
    t++;
    if(costChangeLookUp.count(t)>0){
      isCostChange = true;
      std::cout << __FUNCTION__ << __LINE__ << std::endl;
      UpdateCostMap(t);
      std::cout << __FUNCTION__ << __LINE__ << std::endl;
    }
    else{
      isCostChange = false;
    }
    std::cout << __FUNCTION__ << __LINE__ << std::endl;
  }
};


struct Node{
  vector<int> position; //contains x,y,z position of the state
  double g = INT_MAX; //infinity
  double rhs = INT_MAX; //infinity
  double h_informed = 0;
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
    map<vector<int>, vector<vector<int>>> stairs;
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

  // if(state->h_informed != 0) return state->h_informed;

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
  std::cout << __FUNCTION__ << __LINE__ << std::endl;
  //for now leave as static cost -> change later based on cost map addition
  auto x_succ = succ->position[0];
  auto y_succ = succ->position[1];
  auto z_succ = succ->position[2];
  costMap.GetValueAtCell(x_succ,y_succ,z_succ);
  std::cout << __FUNCTION__ << __LINE__ << std::endl;

  return costMap.GetValueAtCell(x_succ,y_succ,z_succ);
}


void Planner::Initialize()
{

  //U = 0 : emptying of PQ will occur at end of D* complete run, so this will be taken care of in Clear()
  km = 0;
  goalState -> rhs = 0;
  //goalState -> position = {6,14,0}; //temporary position for goal state, will be user entered eventually
  goalState-> key = CalculateKey(goalState);
  graph.insert(make_pair(goalState -> position, goalState));
  U.insert(goalState); //insert goal into PQ -> will be overconsistent

  //costMap
  costMap.Initialize();


  //stairs mapping 
  vector<int> stairStart;
  vector<vector<int>> stairEnd;

  //Level 0:
  stairStart = {95, 50, 0};
  stairEnd = { {50, 9, 2} };
  stairs.insert(make_pair(stairStart, stairEnd));

  //Level 2:
  stairStart = {50, 9, 2};
  stairEnd = { {95, 50, 0}, {7, 50, 4} };
  stairs.insert(make_pair(stairStart, stairEnd));

  //Level 4:
  stairStart = {7, 50, 4};
  stairEnd = { {50, 9, 2}, {50, 97, 6} };
  stairs.insert(make_pair(stairStart, stairEnd));

  //Level 6:
  stairStart = {50, 97, 6};
  stairEnd = { {7, 50, 4}, {97, 50, 8} };
  stairs.insert(make_pair(stairStart, stairEnd));

  //Level 8:
  stairStart = {97, 50, 8};
  stairEnd = { {50, 97, 6}, {50, 3, 10} };
  stairs.insert(make_pair(stairStart, stairEnd));

  //Level 10:
  stairStart = {50, 3, 10};
  stairEnd = { {97, 50, 8} };
  stairs.insert(make_pair(stairStart, stairEnd));

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

  }
  graph.clear();


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
    if(graph.count({x2,y2,z2}) > 0){

      state->neighbors.push_back(graph[{x2,y2,z2}]);
    }
    //Check if location is a valid successor, if so allocate a new node for it and add it to the overall graph
    else if(x2 >= 0 && x2 < x_size && y2 >= 0 && y2< y_size && z2 >= 0 && z2 < z_size && costMap.GetValueAtCell(x2,y2,z2) != collision_flag){

      Node* newNode = new Node;
      newNode->position = {x2,y2,z2};
      newNode->g = INT_MAX;
      newNode->rhs = INT_MAX;
      state->neighbors.push_back(newNode);
      graph[{x2,y2,z2}] = newNode;

    }

  }
  std::cout << __FUNCTION__ << __LINE__ << std::endl;
  //search for stairways
  if(stairs.find(state->position) != stairs.end()) //state is equal to stairway
  {
    for (auto x: (stairs.find(state->position))->second)
    {
      //If we've already allocated a node for this stairEnd just add a pointer to that
      if(graph.count(x) > 0)
      {

        state->neighbors.push_back(graph[x]);
      }
      //Check if location is a valid successor, if so allocate a new node for it and add it to the overall graph
      else
      {

        Node* newNode = new Node;
        newNode->position = x;
        newNode->g = INT_MAX;
        newNode->rhs = INT_MAX;
        state->neighbors.push_back(newNode);
        graph[x] = newNode;

      }
    }
  }
  std::cout << __FUNCTION__ << __LINE__ << std::endl;
  return state->neighbors;

}


void Planner::UpdateVertex(Node* state)
{
  //update vertex within this function

  //condition that this state is not goal state
  if (state->position != goalState->position)
  {

    //update rhs(u) to be min cost of travel + g among successors of state
    double minSucc = INT_MAX; //set to something really large
    double costSucc = 0;
    // cout << "Neighbor Size1:" << (GetNeighbors(state)).size() << endl;
    for (auto x: GetNeighbors(state))
    {
      std::cout << __FUNCTION__ << __LINE__ << std::endl;
      costSucc = x->g + GetCostOfTravel(state, x);
      std::cout << __FUNCTION__ << __LINE__ << std::endl;

      //cout << "Neighbor Position: " << endl;
      //cout << (x->position)[0] << ", " << (x->position)[1] << ", " << (x->position)[2] << endl;
      //cout << "Cost of Neighbor: " << costSucc << endl;

      if (costSucc < minSucc)
      {
        minSucc = costSucc;
      }
    }
    std::cout << __FUNCTION__ << __LINE__ << std::endl;

    state->rhs = minSucc;

  }

  //condition that state exists in PQ -> remove u from PQ
  if (U.find(state) != U.end()) //state exists in PQ
  {
    //remove state from PQ
    U.erase(state);
  }

  //condition that state is overconsistent
  if (state->g != state->rhs)
  {

    state->key = CalculateKey(state);
    U.insert(state);
  }
  std::cout << __FUNCTION__ << __LINE__ << std::endl;
}


void Planner::ComputeShortestPath()
{
  while (PQ_Top(U)->key < CalculateKey(currState) || currState -> rhs != currState -> g)
  {

    pair<double, double> k_old = PQ_Top(U) -> key;
    u = PQ_Top(U);
    cout << "U Size: " << U.size() << endl;
    PQ_Pop(U);
    //cout<< "currState -> rhs: " << currState -> rhs << ", currState -> g: " << currState -> g << endl;

    //cout << (u->position)[0] << ", " << (u->position)[1] << ", " << (u->position)[2] << endl;

    pair<double, double> k_new = CalculateKey(u); //the first time this is run, should be the same as popped since km = 0

    if (k_old < k_new) // condition 1 -> not a lower bound yet
    {

      u->key = k_new; // update the key of specific state within node -> change reflected in graph
      U.insert(u); // reinsert into PQ with new key priority

    }
    else if (u -> g > u -> rhs) // condition 2 -> already a lower bound, just update cost and expand
    {

      u -> g = u -> rhs; // make consistent
      // std::cout << __FUNCTION__ << __LINE__ << std::endl;
      std::cout << __FUNCTION__ << __LINE__ << std::endl;
      auto neighbors = GetNeighbors(u); // update the neighbors parameter of struct ***LOCATION OF SEG FAULT
      std::cout << __FUNCTION__ << __LINE__ << std::endl;
      cout << "Neighbor Size2:" << neighbors.size() << endl;
      // cout << (u->position)[0] << ", " << (u->position)[1] << ", " << (u->position)[2] << endl;



      for (auto x: neighbors)
      {
        UpdateVertex(x);
      }
    }
    else // condition 3 -> already a lower bound, just update cost and expand
    {

      u -> g = INT_MAX;
      std::cout << __FUNCTION__ << __LINE__ << std::endl;
      auto neighbors = GetNeighbors(u); // update the predecessors parameter of struct
      std::cout << __FUNCTION__ << __LINE__ << std::endl;
      for (auto x: neighbors)
      {
        UpdateVertex(x);
      }
      UpdateVertex(u); // for this condition, need to update the actual vertex itself
    }
    //cout<< "currState -> rhs: " << currState -> rhs << ", currState -> g: " << currState -> g << endl;
    //cout<< "U.size: " << U.size() << endl;
  }
  cout << "U Size2: " << U.size() << endl;
}


// main loop for the planner
void Planner::Main()
{

  // 1. Set up node pointer for the starting position
  currState -> position = {50,1,0}; //starting position from example map
  currState -> key = CalculateKey(currState);

  // insert location and node within graph
  graph.insert(make_pair(currState -> position, currState));

  // 2. Set up s_last = s_start -> update the last starting point of robot to be the new starting point of robot to rerun the planning algo
  lastState = currState;

  // 3. Initialize goal and km -> insert into pq
  Initialize();

  // 4. Call on ComputeShortestPath to get generic A* path at the beginning from goal to start
  ComputeShortestPath();
  costMap.Move(currState->position);

  //costMap.printSymbolMap(); //will print debugging version of map
  costMap.TickTime();
  // 5. Main while loop for search
  int count = 0;
  while(currState->position != goalState->position)
  {
    // std::cout << __FUNCTION__ << __LINE__ << std::endl;
    //count++;
    //if(count == 25) return;
    //cout << "Current position:" << endl;
    //cout << (currState->position)[0] << ", " << (currState->position)[1] << ", " << (currState->position)[2] << endl;
    // 6. Check if currState -> g = INT_MAX, if so then no solution
    if (currState -> g == INT_MAX)
    {
      cout << "No solution found!" << endl;
      break;
    }

    // 7. Update currState to be newest state from successors of previous start state (now there will be a difference between last state and curr state)

    double minCostSucc = INT_MAX; //set to something really large
    double costSucc = 0;
    //cout << "Size of Neighbors: " << (currState->neighbors).size() << endl;
    std::cout << __FUNCTION__ << __LINE__ << std::endl;
    for (auto x: GetNeighbors(currState))
    {
      std::cout << __FUNCTION__ << __LINE__ << std::endl;
      costSucc = x->g + GetCostOfTravel(currState, x); //+ GetH(x);
      //cout << "Neighbor Position: " << endl;
      //cout << (x->position)[0] << ", " << (x->position)[1] << ", " << (x->position)[2] << endl;
      //cout << "Cost of Neighbor: " << costSucc << endl;

      if (costSucc < minCostSucc)
      {
        minCostSucc = costSucc;
        currState->h_informed =minCostSucc;
        currState = x;
      }
    }
    // std::cout << __FUNCTION__ << __LINE__ << std::endl;

    //cout << "Neighbor Chosen: " << endl;
    //cout << (currState->position)[0] << ", " << (currState->position)[1] << ", " << (currState->position)[2] << endl;

    // 8. Make new currState the new starting point for robot -> visualization of robot moving here
    std::cout << __FUNCTION__ << __LINE__ << std::endl;
    costMap.Move(currState->position);
    std::cout << __FUNCTION__ << __LINE__ << std::endl;

    // 9. Check all around graph for edge costs (easier in our case with knowledge of when cost map will change)

    // 10. If cost change detected...
    if(costMap.checkCostChange()){
      std::cout << __FUNCTION__ << __LINE__ << std::endl;

      // 11. Add on to km -> constantly adding on to km to make priorities lower bounds of LPA*
      km += GetH(lastState);

      // 12. Update lastState = currState since robot is going to have a new start state soon
      lastState = currState;
      std::cout << __FUNCTION__ << __LINE__ << std::endl;

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
            UpdateVertex(node2Change);
          }

        }

      // 16. Compute shortest path from the goal state to the start state
      std::cout << __FUNCTION__ << __LINE__ << std::endl;  
      ComputeShortestPath();
      std::cout << __FUNCTION__ << __LINE__ << std::endl;

    }

    costMap.TickTime();
    //costMap.printSymbolMap(); //will print debugging version of map
    std::cout << __FUNCTION__ << __LINE__ << std::endl;

  }

  // 17. Clear out graph and PQ
  //Clear();
  // std::cout << __FUNCTION__ << __LINE__ << std::endl;
  cout<<"[";
  for(int i =0; i<costMap.path.size(); i++){
    auto pose = costMap.path[i];
    cout<< pose[0]+1 <<"," << pose[1]+1 << "," << pose[2];
    if(i<costMap.path.size()-1) cout<< ";" << endl;
    else cout << "];" << endl;
  }

  cout << "Path Size: " << costMap.path.size() << endl;
  // std::cout << __FUNCTION__ << __LINE__ << std::endl;

}


int main()
{

  vector<int> goal = {49,91,4}; //goal in example map for now
  Planner planner(goal);
  planner.Main();

  return 0;
}
