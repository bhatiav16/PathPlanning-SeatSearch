#include <iostream>
#include <vector>
#include <fstream>
#include <string>
#include <sstream>

using namespace std;

class CostMap{
public:
  vector<vector<vector<double>>> costMap;
  vector<vector<vector<double>>> allCostChanges; //stores changes to made to costMap at each time t:
  int t = 0;

  //through cost changes at new timestep and update map accordingly
  void UpdateCostMap(){
    if(t < allCostChanges.size()-1){
      t++;
    }
    else t = 0; //just restart cost change trajectory

    auto currentCostChanges = allCostChanges[t];
    for(int s = 0; s<currentCostChanges.size(); s++){
      auto costChange = currentCostChanges[s]; //Takes form: [x,y,z,newCost]
      int i = costChange[0];
      int j = costChange[1];
      int k = costChange[2];
      double newCost = costChange[3];
      costMap[i][j][k] = newCost;
    }
  }

  void initializeFloor(string fileName, int floorNumber){

      vector<vector<double>> floorCosts;

      ifstream file(fileName);
      string line;
      vector <string> getInform;
      getline(file, line);
      while(file.good())
      {

          getline(file, line);
          const auto pos = line.find(',');
          if(pos != string::npos)
              line.erase(0, pos+1);
          stringstream ss(line);
          // Extract each integer
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

string fileName = "Maps/example_map/example_map_t1.csv";
costMap.initializeFloor(fileName, 0);

//printing cost map
for(int i = 0; i<costMap.costMap[0].size(); i++){

  for(int j = 0; j<costMap.costMap[0][0].size(); j++){
    cout<<costMap.costMap[0][i][j] << ", ";
  }
  cout<<endl;
}


  return 0;
}
