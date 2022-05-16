#include "trojanmap.h"
#include <fstream>
#include <string>
#include <vector>
#include <unordered_map>
#include <iostream>
#include <sstream>
#include <limits.h>
using namespace std;


// #include <boost/algorithm/string/find.hpp>

// using namespace boost;
// using namespace std;

//-----------------------------------------------------
// TODO: Student should implement the following:
//-----------------------------------------------------
/**
 * GetLat: Get the latitude of a Node given its id. If id does not exist, return -1.
 * 
 * @param  {std::string} id : location id
 * @return {double}         : latitude
 */
double TrojanMap::GetLat(const std::string& id) {
  if (data[id].lat) {
    return data[id].lat;
  }
  return -1;
}

/**
 * GetLon: Get the longitude of a Node given its id. If id does not exist, return -1.
 * 
 * @param  {std::string} id : location id
 * @return {double}         : longitude
 */
double TrojanMap::GetLon(const std::string& id) {
  if (data[id].lon) {
    return data[id].lon;
  }
  return -1;
}

/**
 * GetName: Get the name of a Node given its id. If id does not exist, return "NULL".
 * 
 * @param  {std::string} id : location id
 * @return {std::string}    : name
 */
std::string TrojanMap::GetName(const std::string& id) { 
  return data[id].name;
}

/**
 * GetNeighborIDs: Get the neighbor ids of a Node. If id does not exist, return an empty vector.
 * 
 * @param  {std::string} id            : location id
 * @return {std::vector<std::string>}  : neighbor ids
 */
std::vector<std::string> TrojanMap::GetNeighborIDs(const std::string& id) {
    return {data[id].neighbors};
}

/**
 * GetID: Given a location name, return the id. 
 * If the node does not exist, return an empty string. 
 *
 * @param  {std::string} name          : location name
 * @return {int}  : id
 */
std::string TrojanMap::GetID(const std::string& name) {
  for (auto it = data.begin(); it != data.end(); ++it)
  {
    std::string currStr = it->second.name;
    if (name.compare(currStr) == 0)
    {
      return it->first;
    }
  }
  return "";
}

/**
 * GetPosition: Given a location name, return the position. If id does not exist, return (-1, -1).
 *
 * @param  {std::string} name          : location name
 * @return {std::pair<double,double>}  : (lat, lon)
 */
std::pair<double, double> TrojanMap::GetPosition(std::string name) {
  std::pair<double, double> results(-1, -1);
  if (name == "") {
    return results;
  }
  else {
  transform(name.begin(), name.end(), name.begin(), ::tolower);
  for (auto id : data) {
    transform(id.second.name.begin(), id.second.name.end(), id.second.name.begin(), ::tolower);
    if (name == id.second.name) { 
      results.first = GetLat(id.first);
      results.second = GetLon(id.first);
      break;
    }
  }
  }
  return results; 
}

/**
 * CalculateEditDistance: Calculate edit distance between two location names
 * 
 */
int TrojanMap::CalculateEditDistance(std::string a, std::string b) {
  transform(a.begin(), a.end(), a.begin(), ::tolower);
  transform(b.begin(), b.end(), b.begin(), ::tolower);
  int m = a.size();
  int n = b.size();
  int D[2][m + 1];
  for (int i = 0; i <= m; i++) {
    D[0][i] = i;
  }
  for (int i = 1; i <= n; i++) {
    for (int j = 0; j <= m; j++) {
      if (j == 0) {
      D[i % 2][j] = i;
      } 
    else if(a[j - 1] == b[i - 1]) {
      D[i % 2][j] = D[(i - 1) % 2][j - 1];
    }
    else {
      D[i % 2][j] = 1 + std::min(D[(i - 1) % 2][j],std::min(D[i % 2][j - 1],D[(i - 1) % 2][j - 1]));
      }
    }
  }
  return D[n % 2][m];
}

/**
 * FindClosestName: Given a location name, return the name with smallest edit distance.
 *
 * @param  {std::string} name          : location name
 * @return {std::string} tmp           : similar name
 */
std::string TrojanMap::FindClosestName(std::string name) {
  std::pair <std::string, int> lowest_dist("NULL", INT32_MAX);
  for (auto id : data) {
    std::string tmp = id.second.name;
    int dist = CalculateEditDistance(name, tmp);
    if (dist < lowest_dist.second) {
      lowest_dist.first = id.second.name;
      lowest_dist.second = dist;
    }
  }
  return lowest_dist.first;
}

/**
 * Autocomplete: Given a parital name return all the possible locations with
 * partial name as the prefix. The function should be case-insensitive.
 *
 * @param  {std::string} name          : partial name
 * @return {std::vector<std::string>}  : a vector of full names
 */
std::vector<std::string> TrojanMap::Autocomplete(std::string name){
  std::vector<std::string> results;
  std::string ids; 
  std::transform(name.begin(), name.end(), name.begin(), ::tolower);
  for (auto id : data) { 
  ids = id.second.name;
  std::transform(ids.begin(), ids.end(), ids.begin(), ::tolower);
    if (name == ids.substr(0, name.size())) {
      results.push_back(id.second.name);
    }
  }
  return results;
}

/**
 * CalculateDistance: Get the distance between 2 nodes. 
 * 
 * @param  {std::string} a  : a_id
 * @param  {std::string} b  : b_id
 * @return {double}  : distance in mile
 */
double TrojanMap::CalculateDistance(const std::string &a_id, const std::string &b_id){
  // Do not change this function
  Node a = data[a_id];
  Node b = data[b_id];
  double dlon = (b.lon - a.lon) * M_PI / 180.0;
  double dlat = (b.lat - a.lat) * M_PI / 180.0;
  double p = pow(sin(dlat / 2),2.0) + cos(a.lat * M_PI / 180.0) * cos(b.lat * M_PI / 180.0) * pow(sin(dlon / 2),2.0);
  double c = 2 * asin(std::min(1.0,sqrt(p)));
  return c * 3961;
}

/**
 * CalculatePathLength: Calculates the total path length for the locations inside the vector.
 * 
 * @param  {std::vector<std::string>} path : path
 * @return {double}                        : path length
 */
double TrojanMap::CalculatePathLength(const std::vector<std::string> &path) {
  // Do not change this function
  double sum = 0;
  for (int i = 0;i < int(path.size())-1; i++) {
    sum += CalculateDistance(path[i], path[i+1]);
  }
  return sum;
}

/**
 * CalculateShortestPath_Dijkstra: Given 2 locations, return the shortest path which is a
 * list of id. Hint: Use priority queue.
 *
 * @param  {std::string} location1_name     : start
 * @param  {std::string} location2_name     : goal
 * @return {std::vector<std::string>}       : path
 */

std::vector<std::string> TrojanMap::CalculateShortestPath_Dijkstra(
      std::string location1_name, std::string location2_name) {
  map<string, vector<string>> path; 
  std::map<string, long double> d; 
  std::map<std::string, Node>::iterator it;
  string id1;
  string id2;
  for (auto id : data) { 
  // for(it = data.begin(); it != data.end(); it++){
    d[id.first] = DBL_MAX;
    if(GetName(id.first) == location1_name){
      id1 = id.first;
    }
    else if(GetName(id.first) == location2_name){
      id2 = id.first;
    }
  }
  d[id1] = 0;
  std::priority_queue<pair<double, string>, vector<pair<double, string>>, greater<pair<double, string>>> q;
  q.push(make_pair(0, id1));
  path[id1].push_back(id1);
  while (!q.empty()) {
    string u = q.top().second;
    q.pop();
    for (auto v : data[u].neighbors) {
      double distance_uv = CalculateDistance(u,v);
      if (d[v] > d[u] + distance_uv) {
        d[v] = d[u] + distance_uv;
        path[v] = path[u];
        path[v].push_back(v);
        q.push(make_pair(d[v], v));
      }
    }
  }
  return path[id2];
}
  

/**
 * CalculateShortestPath_Bellman_Ford: Given 2 locations, return the shortest path which is a
 * list of id. Hint: Do the early termination when there is no change on distance.
 *
 * @param  {std::string} location1_name     : start
 * @param  {std::string} location2_name     : goal
 * @return {std::vector<std::string>}       : path
 */
std::vector<std::string> TrojanMap::CalculateShortestPath_Bellman_Ford(
    std::string location1_name, std::string location2_name){
    std::vector<std::string> path;
    std::string start = GetID(location1_name);
    std::string end = GetID(location2_name);
    std::unordered_map <std::string,double> dist;       
    int f = 0;
    for(auto nodes:data){
      dist[nodes.second.id] = DBL_MAX;
    }
    std::unordered_map <std::string,std::string> predecessor; 
    dist[start] = 0;
    //std::cout << data.size();
    if(start!=end){
      for (int i = 0; i < data.size()-1; i++){
        for (auto pair: data){
            std::string current = pair.second.id;
            for(auto neighbour : pair.second.neighbors){
                  double new_dist = dist[current] + CalculateDistance(current,neighbour);
                  if(dist[neighbour]>new_dist){
                    dist[neighbour] = new_dist;
                    predecessor[neighbour] = current;                    
                    f++;
                  }
            }
        }
        if(f==0){
              break;
            }
            f = 0;
      }
    }
    if(dist[end]==DBL_MAX)
      return path;
    for(auto node = end; node!= start; node = predecessor[node])
    {
      path.push_back(node);
    }
    path.push_back(start);
    std::reverse(path.begin(),path.end());
    return path;
}

/**
 * Travelling salesman problem: Given a list of locations, return the shortest
 * path which visit all the places and back to the start point.
 *
 * @param  {std::vector<std::string>} input : a list of locations needs to visit
 * @return {std::pair<double, std::vector<std::vector<std::string>>} : a pair of total distance and the all the progress to get final path
 */
std::pair<double, std::vector<std::vector<std::string>>> TrojanMap::TravellingTrojan_Brute_Force_Iterative(
                                    std::vector<std::string> location_ids) {
  std::pair<double, std::vector<std::vector<std::string>>> records;
  std::vector<std::string> vertex = location_ids;
  vertex.erase(vertex.begin());
  std::sort(vertex.begin(), vertex.end());
  std::vector<std::vector<std::string>> paths;
  std::vector<std::string> currpath;
  double min_path = DBL_MAX;
  do {
    double curr_cost = 0;
    std::string k = location_ids.front();

    for (auto temp: vertex) {
      curr_cost += CalculateDistance(k,temp);
      k = temp;
    }

    curr_cost += CalculateDistance(k,location_ids.front());
    if (min_path > curr_cost) {
      currpath.clear();
      currpath.push_back(location_ids[0]);
      for (auto x: vertex)
        currpath.push_back(x);
      currpath.push_back(location_ids[0]);
      paths.push_back(currpath);
    }
    min_path = std::min(min_path, curr_cost);
  }
  while (next_permutation(vertex.begin(),vertex.end()));
  records.first = min_path;
  records.second = paths;
  return records;
}

std::pair<double, std::vector<std::vector<std::string>>> TrojanMap::TravellingTrojan_Backtracking(
                                    std::vector<std::string> location_ids) {
  std::pair<double, std::vector<std::vector<std::string>>> records;
  std::vector<std::string> vertex = location_ids;
  vertex.erase(vertex.begin());
  std::sort(vertex.begin(), vertex.end());
  std::vector<std::vector<std::string>> paths;
  std::vector<std::string> currpath;
  double min_path = DBL_MAX;
  do {
    double curr_cost = 0;
    std::string k = location_ids.front();

    for(auto temp: vertex) {
      curr_cost += CalculateDistance(k,temp);
      k = temp;
      if (curr_cost > min_path) { continue; }
    }

    curr_cost += CalculateDistance(k,location_ids.front());
    if (min_path > curr_cost) {
      currpath.clear();
      currpath.push_back(location_ids[0]);
      for (auto x: vertex)
        currpath.push_back(x);
      currpath.push_back(location_ids[0]);
      paths.push_back(currpath);
    }
    min_path = std::min(min_path, curr_cost);
  }
  while (next_permutation(vertex.begin(),vertex.end()));
  records.first = min_path;
  records.second = paths;
  return records;
}

std::pair<double, std::vector<std::vector<std::string>>> TrojanMap::TravellingTrojan_Brute_Force_Recursive(std::vector<std::string> location_ids) {
  std::pair<double, std::vector<std::vector<std::string>>> records;
  std::vector<std::vector<std::string>> paths;
  std::string start = location_ids.front();
  std::vector<std::string> curr_path = {start};
  std::vector<std::string> min_path;
  double min_cost = DBL_MAX;
  TravellingTrojan_Brute_Force_Recursive_Aux(start, start, 0, curr_path, min_cost, min_path, paths, location_ids, records);
  return records;
}

void TrojanMap::TravellingTrojan_Brute_Force_Recursive_Aux(std::string start, std::string curr_node, double curr_cost, 
  std::vector<std::string> &curr_path, double &min_cost, std::vector<std::string> &min_path, 
  std::vector<std::vector<std::string>> &paths, std::vector<std::string> &location_ids, std::pair<double, std::vector<std::vector<std::string>>> &records) {
  if (curr_path.size() == location_ids.size()) {
    int final_cost = curr_cost + CalculateDistance(curr_node, start);
    if (final_cost < min_cost) {
      curr_path.push_back(start);
      paths.push_back(curr_path);
      min_cost = final_cost;
      min_path = curr_path;
      records.first = min_cost;
      records.second = paths;
    }
    return;
  }

  for (int i = 0; i < location_ids.size(); i++) {
    if (std::find(curr_path.begin(), curr_path.end(), location_ids[i]) != curr_path.end()) {
      continue;
    }
    curr_path.push_back(location_ids[i]);
    TravellingTrojan_Brute_Force_Recursive_Aux(start, location_ids[i], curr_cost + CalculateDistance(curr_node, location_ids[i]), curr_path, min_cost, min_path, paths, location_ids, records);
    curr_path.pop_back();
  }
}

std::pair<double, std::vector<std::vector<std::string>>> TrojanMap::TravellingTrojan_Brute_Force_Recursive_Backtracking(std::vector<std::string> location_ids) {
  std::pair<double, std::vector<std::vector<std::string>>> records;
  std::vector<std::vector<std::string>> paths;
  std::string start = location_ids.front();
  std::vector<std::string> curr_path = {start};
  std::vector<std::string> min_path;
  double min_cost = DBL_MAX;
  TravellingTrojan_Brute_Force_Recursive_Backtracking_Aux(start, start, 0, curr_path, min_cost, min_path, paths, location_ids, records);
  return records;
}

 void TrojanMap::TravellingTrojan_Brute_Force_Recursive_Backtracking_Aux(
      std::string start, std::string curr_node, double curr_cost, 
  std::vector<std::string> &curr_path, double &min_cost, std::vector<std::string> &min_path, 
  std::vector<std::vector<std::string>> &paths, std::vector<std::string> &location_ids, 
  std::pair<double, std::vector<std::vector<std::string>>> &records) {
    if (curr_path.size() == location_ids.size()) {
    int final_cost = curr_cost + CalculateDistance(curr_node, start);
    if (final_cost < min_cost) {
      curr_path.push_back(start);
      paths.push_back(curr_path);
      min_cost = final_cost;
      min_path = curr_path;
      records.first = min_cost;
      records.second = paths;
    }
    return;
  }
  if (curr_cost >= min_cost) {
    return;
  }
  for (int i = 0; i < location_ids.size(); i++) {
    if (std::find(curr_path.begin(), curr_path.end(), location_ids[i]) != curr_path.end()) {
      continue;
    }
    curr_path.push_back(location_ids[i]);
    TravellingTrojan_Brute_Force_Recursive_Backtracking_Aux(start, location_ids[i], curr_cost + CalculateDistance(curr_node, location_ids[i]), curr_path, min_cost, min_path, paths, location_ids, records);
    curr_path.pop_back();
  }
  }

std::pair<double, std::vector<std::vector<std::string>>> TrojanMap::TravellingTrojan_2opt(
      std::vector<std::string> location_ids){
  
  std::pair<double, std::vector<std::vector<std::string>>> results;
  std::vector<std::string> vertex = location_ids;
  vertex.push_back(location_ids.front());
  int length = vertex.size();
  int improve = 0;
  while(improve<20)
  {
    start_again: double best_dist = CalculatePathLength(vertex);
    for(int i=1; i<length-2; i++)
    {
      for(int j=i+1; j<length-1; j++)
      {
        std::vector<std::string> temp = vertex;
        std::reverse(temp.begin()+i,temp.begin()+j+1);
        double new_dist = CalculatePathLength(temp);
        if(new_dist < best_dist)
        {
          improve = 0;
          vertex.clear();
          vertex = temp;
          best_dist = new_dist;
          results.second.push_back(temp);
          goto start_again;
        }
      }
    }
    improve++;
  }
  results.first = CalculatePathLength(vertex);
  return results; 
}

/**
 * Given CSV filename, it read and parse locations data from CSV file,
 * and return locations vector for topological sort problem.
 *
 * @param  {std::string} locations_filename     : locations_filename
 * @return {std::vector<std::string>}           : locations 
 */
std::vector<std::string> TrojanMap::ReadLocationsFromCSVFile(std::string locations_filename){
  std::vector<std::string> location_names_from_csv;
  std::fstream fin;
  fin.open(locations_filename, std::ios::in);
  std::string location;

  getline(fin, location);
  while (getline(fin, location)) {
    std::stringstream s(location);
    location_names_from_csv.push_back(location);
  }
  return location_names_from_csv;
}

/**
 * Given CSV filenames, it read and parse dependencise data from CSV file,
 * and return dependencies vector for topological sort problem.
 *
 * @param  {std::string} dependencies_filename     : dependencies_filename
 * @return {std::vector<std::vector<std::string>>} : dependencies
 */
std::vector<std::vector<std::string>> TrojanMap::ReadDependenciesFromCSVFile(std::string dependencies_filename){
  std::vector<std::vector<std::string>> dependencies_from_csv;
  std::fstream fin;
  fin.open(dependencies_filename, std::ios::in);
  std::string line;
  getline(fin, line);
  while (getline(fin, line)) {
    std::string firstpos , secondpos;
    auto pos = line.find(',');
    if(pos == -1 || pos == 0 || pos == line.size() - 1) continue;
    firstpos = line.substr(0, pos);
    secondpos = line.substr(pos + 1);
    secondpos.erase(std::remove(secondpos.begin(),secondpos.end(),','), secondpos.end());
    dependencies_from_csv.push_back({firstpos,secondpos});
  }
  fin.close();
  return dependencies_from_csv;
}

void RecursiveTopological(std::string location, std::unordered_map<std::string, std::vector<std::string>> &adjacency_map, std::unordered_map<std::string, bool> &visited, std::vector<std::string> &result) {
  visited[location] = true;
  result.push_back(location);
  for (auto a : adjacency_map[location]) {
    if (visited[a] == false) {
      RecursiveTopological(a, adjacency_map, visited, result);
    }
  }
}

/**
 * DeliveringTrojan: Given a vector of location names, it should return a sorting of nodes
 * that satisfies the given dependencies. If there is no way to do it, return a empty vector.
 *
 * @param  {std::vector<std::string>} locations                     : locations
 * @param  {std::vector<std::vector<std::string>>} dependencies     : prerequisites
 * @return {std::vector<std::string>} results                       : results
 */
std::vector<std::string> TrojanMap::DeliveringTrojan(std::vector<std::string> &locations,
                                                     std::vector<std::vector<std::string>> &dependencies){
  std::vector<std::string> result;
  std::unordered_map<std::string, std::vector<std::string>> adjacency_map;
  std::unordered_map<std::string, bool> visited;
  for (auto i : locations) {
    visited[i] = false;
  }
  for (auto dep : dependencies) {
    adjacency_map[dep[0]].push_back(dep[1]);
  }
  for (int i = 0; i < locations.size(); i++) {
    if (visited[locations[i]] == false) {
      RecursiveTopological(locations[i], adjacency_map, visited, result);
    }
  }
  return result;
}
/**
 * inSquare: Give a id return whether it is in square or not.
 *
 * @param  {std::string} id            : location id
 * @param  {std::vector<double>} square: four vertexes of the square area
 * @return {bool}                      : in square or not
 */
bool TrojanMap::inSquare(std::string id, std::vector<double> &square) {
  return false;
}

/**
 * GetSubgraph: Give four vertexes of the square area, return a list of location ids in the squares
 *
 * @param  {std::vector<double>} square         : four vertexes of the square area
 * @return {std::vector<std::string>} subgraph  : list of location ids in the square
 */
std::vector<std::string> TrojanMap::GetSubgraph(std::vector<double> &square) {
  // include all the nodes in subgraph
  std::vector<std::string> subgraph;
  return subgraph;
}

/**
 * Cycle Detection: Given four points of the square-shape subgraph, return true if there
 * is a cycle path inside the square, false otherwise.
 * 
 * @param {std::vector<std::string>} subgraph: list of location ids in the square
 * @param {std::vector<double>} square: four vertexes of the square area
 * @return {bool}: whether there is a cycle or not
 */

bool TrojanMap::RecursiveCycleDetection(const std::string &currID,std::unordered_map<std::string,int> &visited,const std::string &parentID){
  visited[currID] = 1;
  Node curr_Node = data[currID];
  for(std::string &adj:curr_Node.neighbors){
    if(data.count(adj) == 0 || visited.count(adj)==0 || adj==parentID)
      continue; 
    if(visited[adj]==1 || (visited[adj]==0 && RecursiveCycleDetection(adj,visited,currID)))
      return true;
  }
  visited[currID] = 2;
  return false;
}

bool TrojanMap::CycleDetection(std::vector<std::string> &subgraph, std::vector<double> &square) {
  if(square.size()<4) return false;
  std::vector<std::string> locations;
  for(auto &node:data){
    if(node.second.lon>=square[0] && node.second.lon<=square[1] && node.second.lat<=square[2] && node.second.lat>=square[3]){
      locations.push_back(node.first);
    }
  }
  std::unordered_map<std::string,int> visited;
  for(auto &point:locations){
    visited[point] = 0;
  }
  for (auto &point:visited) {
    if (point.second==0 && RecursiveCycleDetection(point.first,visited,"")) {
      return true;
    }
  }
  return false;
}


/**
 * FindNearby: Given a class name C, a location name L and a number r, 
 * find all locations in class C on the map near L with the range of r and return a vector of string ids
 * 
 * @param {std::string} className: the name of the class
 * @param {std::string} locationName: the name of the location
 * @param {int} r: search radius
 * @param {int} k: search numbers
 * @return {std::vector<std::string>}: location name that meets the requirements
 */
std::vector<std::string> TrojanMap::FindNearby(std::string attributesName, std::string name, double r, int k) {
  std::vector<std::string> res;
  // // std::vector<std::string> places;
  // std::vector<std::pair<double, std::string>> distances;
  // std::string id = GetID(name);
  // for (auto place : data) {
  //   std::pair<double, std::string> st;
  //   // int x = attributesName.std::string::compare(place.second.attributes);
  //   if (attributesName == place.second.attributes) {
  //     double dist = CalculateDistance(place.second.id, id);
  //     if (dist < r) {
  //       st.first = dist;
  //       st.second = place.second.name;
  //       distances.push_back(st);
  //     }
  //   }
  //   std::sort(distances.begin(), distances.end());
  //   for (int i = 0; i < k; i++) {
  //     res.push_back(distances[i].second);
  //   }
  //   std::cout << res[0];
  // }
  return res;
}

/**
 * CreateGraphFromCSVFile: Read the map data from the csv file
 * 
 */
void TrojanMap::CreateGraphFromCSVFile() {
  // Do not change this function
  std::fstream fin;
  fin.open("src/lib/data.csv", std::ios::in);
  std::string line, word;

  getline(fin, line);
  while (getline(fin, line)) {
    std::stringstream s(line);

    Node n;
    int count = 0;
    while (getline(s, word, ',')) {
      word.erase(std::remove(word.begin(), word.end(), '\''), word.end());
      word.erase(std::remove(word.begin(), word.end(), '"'), word.end());
      word.erase(std::remove(word.begin(), word.end(), '{'), word.end());
      word.erase(std::remove(word.begin(), word.end(), '}'), word.end());
      if (count == 0)
        n.id = word;
      else if (count == 1)
        n.lat = stod(word);
      else if (count == 2)
        n.lon = stod(word);
      else if (count == 3)
        n.name = word;
      else {
        word.erase(std::remove(word.begin(), word.end(), ' '), word.end());
        if (isalpha(word[0]))
          n.attributes.insert(word);
        if (isdigit(word[0]))
          n.neighbors.push_back(word);
      }
      count++;
    }
    data[n.id] = n;
  }
  fin.close();
}
