#include <iostream>
#include <vector>
#include <fstream>
#include <string>
#include <sstream>
#include <map>

using namespace std;

class CostMap{
public:
  vector<vector<vector<double>>> costMap;
  map<int,vector<vector<double>> > costChangeLookUp; //stores the location and valuechange to made to each cell in the costMap at each time t:
  int t = 0;

  //through cost changes at new timestep and update map accordingly
  void UpdateCostMap(){
    if(costChangeLookUp.count(t) > 0){
      t++;
    }
    else t = 0; //just restart cost change trajectory

    auto currentCostChanges = costChangeLookUp[t];
    for(int s = 0; s<currentCostChanges.size(); s++){
      auto costChangeInfo = currentCostChanges[s]; //Takes form: [x,y,z,newCost]
      int i = costChangeInfo[0];
      int j = costChangeInfo[1];
      int k = costChangeInfo[2];
      double newCost = costChangeInfo[3];
      costMap[i][j][k] = newCost;
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
          while(ss >> val){
              row.push_back(val);
              // If the next token is a comma, ignore it and move on
              if(ss.peek() == ',') ss.ignore();
          }
          floorCosts.push_back(row);
      }
      costMap.push_back(floorCosts);
  }
};


int main(){

CostMap costMap;

string mapFileName = "Maps/example_map/example_map_initial.csv";
costMap.InitializeFloor(mapFileName, 0);

//printing cost map
cout << "map at t = 0" << endl;
for(int i = 0; i<costMap.costMap[0].size(); i++){

  for(int j = 0; j<costMap.costMap[0][0].size(); j++){
    cout<<costMap.costMap[0][i][j] << ", ";
  }
  cout<<endl;
}

string costChangeFileName = "Maps/example_map/example_costChanges_t1.csv";

costMap.InitializeCostChangeLookUp(costChangeFileName);

/*
for(int i = 0; i<costMap.costChangeLookUp[1].size(); i++){

  for(int j = 0; j<costMap.costChangeLookUp[1][0].size(); j++){
    cout<<costMap.costChangeLookUp[1][i][j] << ", ";
  }
  cout<<endl;
}*/

costMap.UpdateCostMap();

cout << endl << "map at t = 1" << endl;

for(int i = 0; i<costMap.costMap[0].size(); i++){

  for(int j = 0; j<costMap.costMap[0][0].size(); j++){
    cout<<costMap.costMap[0][i][j] << ", ";
  }
  cout<<endl;
}



  return 0;
}
