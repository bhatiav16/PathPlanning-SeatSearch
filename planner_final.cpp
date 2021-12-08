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
#include <chrono>
#include <ctime>



//std::cout << __FUNCTION__ << __LINE__ << std::endl;


using namespace std;

vector<int> start = {40,4,0};
vector<int> goal = {67,5,10}; //goal in example map for now

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
  // vector<vector<vector<string>>> symbolMap;
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

    auto temp = valueMap[z][x][y];

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
      //cout<<"costChangeInfo.size(): " << costChangeInfo.size()<<endl;
      //cout << "t = " << t << endl;
      //cout << "costChangeInfo[0]" << costChangeInfo[0] << " costChangeInfo[1]" << costChangeInfo[1] << " costChangeInf0[2]" << costChangeInfo[2] << " costChangeInfo[3]" << costChangeInfo[3]<< endl;

      int i = costChangeInfo[0];
      int j = costChangeInfo[1];
      int k = costChangeInfo[2];
      double newCost = costChangeInfo[3];
      if(i == 200 || j == 200 || k == 200) continue;
      //cout << "i: " << i << ", j: " << j << " k: " << k << "costChange: " << newCost << endl;
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
    for(auto const& x : costChangeLookUp){
      for(auto const& costChangeInfo: x.second){
        //cout << "costChangeInfo[0]" << costChangeInfo[0] << " costChangeInfo[1]" << costChangeInfo[1] << " costChangeInf0[2]" << costChangeInfo[2] << " costChangeInfo[3]" << costChangeInfo[3]<< endl;
      }
    }
  }

  void InitializeFloor(string fileName, int floorNumber){
      vector<vector<double>> floorCosts;
      // vector<vector<string>> floorSymbols;
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
          // vector<string> rowSymbols;
          while(ss >> val){
              row.push_back(val);
              // rowSymbols.push_back(to_string_with_precision(val));
              // If the next token is a comma, ignore it and move on
              if(ss.peek() == ',') ss.ignore();
          }
          floorCosts.push_back(row);
          // floorSymbols.push_back(rowSymbols);
      }
      // valueMap.push_back(floorCosts);
      valueMap[floorNumber] = floorCosts;
      // symbolMap.push_back(floorSymbols);

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

    string costChangeFileName = "Maps/Stadium_mod/allCostChanges_v4.csv";

    InitializeCostChangeLookUp(costChangeFileName);

  }

  void TickTime(){


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
          //if(a->key == b->key) return true;
          if((a->key).first != (b->key).first){
            return ((a->key).first > (b->key).first);
          }

          return ((a->key).second > (b->key).second);
      }


};

/*void PQ_Pop(set<Node*, NodeComparator> &U){
    auto to_pop = *U.rbegin();
    U.erase(to_pop);
}

Node* PQ_Top(set<Node*, NodeComparator> &U){
    auto at_top = *U.rbegin();

    return at_top;
}*/

void PQ_Remove(Node* node2Remove,priority_queue<Node*, vector<Node*>, NodeComparator> &U, set<Node*>  &U_set){

    U_set.erase(node2Remove);
    priority_queue<Node*, vector<Node*>, NodeComparator> U_new;

    while(!U.empty()){
        auto topNode = U.top();
        U.pop();
        if(topNode != node2Remove){
            U_new.push(topNode);
        }
    }
    U = U_new;
}

class Planner
{
  public:
    Planner(vector<int> goalPosition);
    void Main();

  private:
    // variables
    Node* currState; //= new Node; //the current position of the robot, in paper -> s_start
    Node* lastState; //= new Node; //the last state robot was at
    Node* goalState = new Node; //the user defined goal state
    map<vector<int>, Node*> graph; //maps a given x,y,z to its corresponding node. This is a container of all of the nodes allocated on the graph
    //set<Node*, NodeComparator> U;
    priority_queue<Node*, vector<Node*>, NodeComparator> U;
    set<Node*>  U_set;
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


  vector<int> localGoalPosition;
  auto currStateZ = (currState->position)[2];
  if(stateZ == currStateZ){
    localGoalPosition = {(currState->position)[0], (currState->position)[1], (currState->position)[2]};
  }
  else if(stateZ == 0){
    localGoalPosition = {95, 50, 0};
  }
  else if(stateZ == 2){
    localGoalPosition = {50, 9, 2};
  }
  else if(stateZ == 4){
    localGoalPosition = {7, 50, 4};
  }
  else if(stateZ == 6){
    localGoalPosition = {50, 97, 6};
  }
  else if(stateZ == 8){
    localGoalPosition = {97, 50, 8};
  }
  else if(stateZ == 10){
    localGoalPosition = {50, 3, 10};
  }
  else{
    cout<<"invalid input to getH"<<endl;
    return 0;
  }

  double localGoalStateX = localGoalPosition[0];
  double localGoalStateY = localGoalPosition[1];
  double localGoalStateZ = localGoalPosition[2];

  double xDiff = stateX - localGoalStateX;
  double yDiff = stateY - localGoalStateY;
  double zDiff = stateZ - localGoalStateZ;

  return sqrt(pow(xDiff,2) + pow(yDiff,2) + pow(zDiff,2));
}


double Planner::GetCostOfTravel(Node* state, Node* succ)
{

  //for now leave as static cost -> change later based on cost map addition
  auto x_succ = succ->position[0];
  auto y_succ = succ->position[1];
  auto z_succ = succ->position[2];
  costMap.GetValueAtCell(x_succ,y_succ,z_succ);


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
  if(U_set.count(goalState) == 0){
    U.push(goalState);
    U_set.insert(goalState);
  }

  //U.insert(goalState); //insert goal into PQ -> will be overconsistent

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
    U_set.erase(U.top());
    U.pop();
    //PQ_Pop(U);
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
  //std::cout << __FUNCTION__ << __LINE__ << std::endl;

  //check all 8 2d successors
  //some struct that can check z in a txt file
  //cout<<"x : " << state->position[0] << " y: " << state->position[1] << " z: " << state->position[2] << endl;

  double collision_flag = -1; //just here for now
  int z_size = costMap.valueMap.size();
  int x_size = costMap.valueMap[0].size();
  int y_size = costMap.valueMap[0][0].size();

  //std::cout << __FUNCTION__ << __LINE__ << std::endl;


  //this node has been allocated and had its successors already generated as well. Just return them
  if(state->neighbors.size() != 0){
    //std::cout << __FUNCTION__ << __LINE__ << std::endl;

    if(state->neighbors.size() > 10){
      cout<< "neighbors size: " << state->neighbors.size() << endl;
      cout << "position size: " << state->position.size() << endl;
      cout << (state->position)[0] << ", " << (state->position)[1] << ", " << (state->position)[2] << endl;
    }

    return state->neighbors;
  }

  //std::cout << __FUNCTION__ << __LINE__ << std::endl;


  const int NUMOFDIRS = 8;
  int dX[NUMOFDIRS] = {-1, -1, -1,  0,  0,  1, 1, 1};
  int dY[NUMOFDIRS] = {-1,  0,  1, -1,  1, -1, 0, 1};

  for(int i = 0; i<NUMOFDIRS; i++){

    int x2 = state->position[0]+dX[i];
    int y2 = state->position[1]+dY[i];
    int z2 = state->position[2]; //not moving z for now...
    //std::cout << __FUNCTION__ << __LINE__ << std::endl;


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

  //std::cout << __FUNCTION__ << __LINE__ << std::endl;


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
  //std::cout << __FUNCTION__ << __LINE__ << std::endl;

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
    auto neighbors = GetNeighbors(state);
    for (auto x: neighbors)
    {

      costSucc = x->g + GetCostOfTravel(state, x);


      //cout << "Neighbor Position: " << endl;
      //cout << (x->position)[0] << ", " << (x->position)[1] << ", " << (x->position)[2] << endl;
      //cout << "Cost of Neighbor: " << costSucc << endl;

      if (costSucc < minSucc)
      {
        minSucc = costSucc;
      }
    }


    state->rhs = minSucc;
    //if(state->position == start) cout<<"updating start rhs to: " << state->rhs << endl;

  }

  //condition that state exists in PQ -> remove u from PQ
  if (U_set.count(state) > 0) //state exists in PQ
  {
    //remove state from PQ
    PQ_Remove(state, U, U_set);
    //U.erase(state);
  }

  //condition that state is overconsistent
  if (state->g != state->rhs)
  {

    //if(state->position == start && U.find(state) != U.end()) cout << "Start already in PQ1" << endl;
    state->key = CalculateKey(state);
    //if(state->position == start && U.find(state) != U.end()) cout << "Start already in PQ2" << endl;
    //if(state->position == start) cout << "start inconsistent" << endl;
    //if(state->position == start) cout<<"U before inserting:" << U.size() << endl;
    /*
    static bool firstOne = true;

    auto poseA = state->position;
    if( firstOne == true){
      cout<< "node to be inserted: "<< poseA[0] << ", " << poseA[1] << ", " << poseA[2] << endl;
      cout<<"key to be inserted: " << state->key.first << ", " << state->key.second << endl;
    }*/
    U.push(state);
    U_set.insert(state);

    /*
    auto poseB = PQ_Top(U)->position;
    if( firstOne == true){
      cout<< "top node in PQ after: "<< poseB[0] << ", " << poseB[1] << ", " << poseB[2] << endl;
      cout<<"top key in PQ after: " << PQ_Top(U)->key.first << ", " << PQ_Top(U)->key.second << endl;
    }
    firstOne = false;

    if(state->position == start) cout<<"U after inserting:" << U.size() << endl;
    */
  }

}


void Planner::ComputeShortestPath()
{
  //std::cout << __FUNCTION__ << __LINE__ << std::endl;
  while (!U.empty() && (U.top()->key < CalculateKey(currState) || currState -> rhs != currState -> g))
  {



    //pair<double, double> k_old = PQ_Top(U) -> key;
    u = U.top();
    auto k_old = u->key;
    //u = PQ_Top(U);
    //if(u->position == start) cout << "WE HAVE POPPED START!" << endl;
    // cout << "U Size: " << U.size() << endl;
    U.pop();
    U_set.erase(u);
    //PQ_Pop(U);
    //cout<< "currState -> rhs: " << currState -> rhs << ", currState -> g: " << currState -> g << endl;

    //cout << (u->position)[0] << ", " << (u->position)[1] << ", " << (u->position)[2] << endl;

    pair<double, double> k_new = CalculateKey(u); //the first time this is run, should be the same as popped since km = 0

    if (k_old < k_new) // condition 1 -> not a lower bound yet
    {


      u->key = k_new; // update the key of specific state within node -> change reflected in graph
      U.push(u);
      U_set.insert(u);
      //U.insert(u); // reinsert into PQ with new key priority


    }
    else if (u->g > u->rhs) // condition 2 -> already a lower bound, just update cost and expand
    {

      //if(u == graph[start]) cout << "before setting g to rhs..."<<"start->rhs: " << graph[start]->rhs <<  " ,start->g: " << graph[start]->g << endl;
      u->g = u->rhs; // make consistent
      //if(u == graph[start]) cout << "before setting g to rhs..."<<"start->rhs: " << graph[start]->rhs <<  " ,start->g: " << graph[start]->g << endl;


      auto neighbors = GetNeighbors(u); // update the neighbors parameter of struct ***LOCATION OF SEG FAULT

      // cout << "Neighbor Size2:" << neighbors.size() << endl;
      // cout << (u->position)[0] << ", " << (u->position)[1] << ", " << (u->position)[2] << endl;

      for (auto x: neighbors)
      {
        UpdateVertex(x);
      }

    }
    else // condition 3 -> already a lower bound, just update cost and expand
    {
      //cout << "before setting g to max..."<<"start->rhs: " << graph[start]->rhs <<  " ,start->g: " << graph[start]->g << endl;
      u->g = INT_MAX;
      //cout << "after setting g to max..."<< "start->rhs: " << graph[start]->rhs <<  " ,start->g: " << graph[start]->g << endl;


      auto neighbors = GetNeighbors(u); // update the predecessors parameter of struct

      for (auto x: neighbors)
      {
        UpdateVertex(x);
      }
      UpdateVertex(u); // for this condition, need to update the actual vertex itself

    }
    //cout<< "currState -> rhs: " << currState -> rhs << ", currState -> g: " << currState -> g << endl;
    //cout<< "U.size: " << U.size() << endl;

  }
  // cout << "U Size2: " << U.size() << endl;
}


// main loop for the planner


void Planner::Main()
{

  std::cout << __FUNCTION__ << __LINE__ << std::endl;
  // 1. Set up node pointer for the starting position
  currState = new Node;
  currState -> position = start;//{50,1,0}; //starting position from example map
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
    std::cout << __FUNCTION__ << __LINE__ << std::endl;
    //count++;
    //if(count == 25) return;
    //cout << "Current Position:" << endl;
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

    for (auto x: GetNeighbors(currState))
    {

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


    //cout << "Neighbor Chosen: " << endl;
    //cout << (currState->position)[0] << ", " << (currState->position)[1] << ", " << (currState->position)[2] << endl;

    // 8. Make new currState the new starting point for robot -> visualization of robot moving here

    costMap.Move(currState->position);


    // 9. Check all around graph for edge costs (easier in our case with knowledge of when cost map will change)

    // 10. If cost change detected...
    if(costMap.checkCostChange()){

      // 11. Add on to km -> constantly adding on to km to make priorities lower bounds of LPA*
      km = 0; //+= GetH(lastState);

      // 12. Update lastState = currState since robot is going to have a new start state soon
      lastState = currState;

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

      ComputeShortestPath();


    }

    costMap.TickTime();
    //costMap.printSymbolMap(); //will print debugging version of map

    std::cout << __FUNCTION__ << __LINE__ << std::endl;
  }

  // 17. Clear out graph and PQ
  //Clear();

  cout<<"[";
  string outFileName = "path.csv";
  ofstream outFile(outFileName);
  for(int i =0; i<costMap.path.size(); i++){
    auto pose = costMap.path[i];
    cout<< pose[0]+1 <<"," << pose[1]+1 << "," << pose[2];
    outFile << pose[0]+1 <<"," << pose[1]+1 << "," << pose[2] << "\n";
    if(i<costMap.path.size()-1) cout<< ";" << endl;
    else cout << "];" << endl;
  }
  outFile.close();

  cout << "Path Size: " << costMap.path.size() << endl;


}


int main()
{

  //cout<<"Enter goal: "

  //vector<int> goal = {8,25,10}; //goal in example map for now
  //vector<int> goal = {39,6,10}; //goal in example map for now

  auto start = std::chrono::system_clock::now();
  // Some computation here



  Planner planner(goal);
  planner.Main();

  auto end = std::chrono::system_clock::now();
  std::chrono::duration<double> elapsed_seconds = end-start;
  std::time_t end_time = std::chrono::system_clock::to_time_t(end);

  std::cout << "planning time: " << elapsed_seconds.count() << "s\n";

  return 0;
}
