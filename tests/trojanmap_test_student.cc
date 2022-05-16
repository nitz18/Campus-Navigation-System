#include <vector>
#include <unordered_set>
#include "gtest/gtest.h"
#include "src/lib/trojanmap.h"

// // Phase 1
// // Test Autocomplete function
TEST(TrojanMapTest, Autocomplete) {
  TrojanMap m;
  // Test the simple case
  auto names = m.Autocomplete("Chi");
  std::unordered_set<std::string> gt = {"Chick-fil-A", "Chipotle", "Chinese Street Food"}; // groundtruth for "Ch"
  int success = 0;
  for (auto& n: names) {
    EXPECT_EQ(gt.count(n) > 0, true) << n + " is not in gt.";
    if (gt.count(n) > 0){
      success++;
    }
  }
  EXPECT_EQ(success, gt.size());
  // Test the lower case
  names = m.Autocomplete("chi");
  success = 0;
  for (auto& n: names) {
    EXPECT_EQ(gt.count(n) > 0, true) << n + " is not in gt.";
    if (gt.count(n) > 0){
      success++;
    }
  }
  EXPECT_EQ(success, gt.size());
  // Test the lower and upper case 
  names = m.Autocomplete("cHi"); 
  success = 0;
  for (auto& n: names) {
    EXPECT_EQ(gt.count(n) > 0, true) << n + " is not in gt.";
    if (gt.count(n) > 0){
      success++;
    }
  }
  EXPECT_EQ(success, gt.size());
  // Test the upper case 
  names = m.Autocomplete("CHI"); 
  success = 0;
  for (auto& n: names) {
    EXPECT_EQ(gt.count(n) > 0, true) << n + " is not in gt.";
    if (gt.count(n) > 0){
      success++;
    }
  }
  EXPECT_EQ(success, gt.size());
}

TEST(TrojanMapTest, Autocomplete1) {
  TrojanMap m;
  // Test the simple case
  auto names = m.Autocomplete("Po");
  std::unordered_set<std::string> gt = {"Popeyes 1", "Porterhouse Fried Chicken", "Popeyes"}; // groundtruth for "Po"
  int success = 0;
  for (auto& n: names) {
    EXPECT_EQ(gt.count(n) > 0, true) << n + " is not in gt.";
    if (gt.count(n) > 0){
      success++;
    }
  }
  EXPECT_EQ(success, gt.size());
  // Test the lower case
  names = m.Autocomplete("po");
  success = 0;
  for (auto& n: names) {
    EXPECT_EQ(gt.count(n) > 0, true) << n + " is not in gt.";
    if (gt.count(n) > 0){
      success++;
    }
  }
  EXPECT_EQ(success, gt.size());
  // Test the lower and upper case 
  names = m.Autocomplete("pO"); 
  success = 0;
  for (auto& n: names) {
    EXPECT_EQ(gt.count(n) > 0, true) << n + " is not in gt.";
    if (gt.count(n) > 0){
      success++;
    }
  }
  EXPECT_EQ(success, gt.size());
  // Test the upper case 
  names = m.Autocomplete("PO"); 
  success = 0;
  for (auto& n: names) {
    EXPECT_EQ(gt.count(n) > 0, true) << n + " is not in gt.";
    if (gt.count(n) > 0){
      success++;
    }
  }
  EXPECT_EQ(success, gt.size());
}

TEST(TrojanMapTest, Autocomplete2) {
  TrojanMap m;
  // Test the simple case
  auto names = m.Autocomplete("Usc");
  std::unordered_set<std::string> gt = {"USC Roski Eye Institute", "USC Parking", "USC Village Gym", "USC Fisher Museum of Art", "USC Credit Union"}; // groundtruth for "USC"
  int success = 0;
  for (auto& n: names) {
    EXPECT_EQ(gt.count(n) > 0, true) << n + " is not in gt.";
    if (gt.count(n) > 0){
      success++;
    }
  }
  EXPECT_EQ(success, gt.size());
  // Test the lower case
  names = m.Autocomplete("usc");
  success = 0;
  for (auto& n: names) {
    EXPECT_EQ(gt.count(n) > 0, true) << n + " is not in gt.";
    if (gt.count(n) > 0){
      success++;
    }
  }
  EXPECT_EQ(success, gt.size());
  // Test the lower and upper case 
  names = m.Autocomplete("uSc"); 
  success = 0;
  for (auto& n: names) {
    EXPECT_EQ(gt.count(n) > 0, true) << n + " is not in gt.";
    if (gt.count(n) > 0){
      success++;
    }
  }
  EXPECT_EQ(success, gt.size());
  // Test the upper case 
  names = m.Autocomplete("USC"); 
  success = 0;
  for (auto& n: names) {
    EXPECT_EQ(gt.count(n) > 0, true) << n + " is not in gt.";
    if (gt.count(n) > 0){
      success++;
    }
  }
  EXPECT_EQ(success, gt.size());
}

// -------------------------------------------------------------------------------------------------------------------------------------

// Test FindPosition function
TEST(TrojanMapTest, FindPosition) {
  TrojanMap m;
  
  // Test Chick-fil-A
  auto position = m.GetPosition("Chick-fil-A");
  std::pair<double, double> gt1(34.0167334, -118.2825307); // groundtruth for "Chick-fil-A"
  EXPECT_EQ(position, gt1);
  // Test Ralphs
  position = m.GetPosition("Ralphs");
  std::pair<double, double> gt2(34.0317653, -118.2908339); // groundtruth for "Ralphs"
  EXPECT_EQ(position, gt2);
  // Test Target
  position = m.GetPosition("Target");
  std::pair<double, double> gt3(34.0257016, -118.2843512); // groundtruth for "Target"
  EXPECT_EQ(position, gt3);
  // // Test USC Village Gym
  position = m.GetPosition("USC Village Gym");
  std::pair<double, double> gt4(34.0252392,-118.2858186); // groundtruth for "USC Village"
  EXPECT_EQ(position, gt4);
  // Test Unknown
  position = m.GetPosition("XXX");
  std::pair<double, double> gt5(-1, -1);
  EXPECT_EQ(position, gt5);
  // Test KFC
  position = m.GetPosition("KFC");
  std::pair<double, double> gt6(34.0260723,-118.2780288);
  EXPECT_EQ(position, gt6);
  // Test USC Parking
  position = m.GetPosition("USC Parking");
  std::pair<double, double> gt7(34.0238824,-118.2801114);
  EXPECT_EQ(position, gt7);
}

// -------------------------------------------------------------------------------------------------------------------------------------

// Test CalculateEditDistance function
TEST(TrojanMapTest, CalculateEditDistance) {
  TrojanMap m;
  EXPECT_EQ(m.CalculateEditDistance("horse", "ros"), 3);
  EXPECT_EQ(m.CalculateEditDistance("intention", "execution"), 5);
  EXPECT_EQ(m.CalculateEditDistance("apple", "apple"), 0);
  EXPECT_EQ(m.CalculateEditDistance("apple", "APPLY"), 1);
  EXPECT_EQ(m.CalculateEditDistance("apple", "aPPlYY"), 2);
  EXPECT_EQ(m.CalculateEditDistance("apple", ""), 5);
}

// -------------------------------------------------------------------------------------------------------------------------------------

// Test FindClosestName function
TEST(TrojanMapTest, FindClosestName) {
  TrojanMap m;
  EXPECT_EQ(m.FindClosestName("Rolphs"), "Ralphs");
  EXPECT_EQ(m.FindClosestName("Targeety"), "Target");
  EXPECT_EQ(m.FindClosestName("Chick-fil"), "Chick-fil-A");
  EXPECT_EQ(m.FindClosestName("CVC"), "CAVA");
  EXPECT_EQ(m.FindClosestName("USC"), "KFC");
}

// -------------------------------------------------------------------------------------------------------------------------------------

// Phase 2
// Test CalculateShortestPath_Dijkstra function
TEST(TrojanMapTest, CalculateShortestPath_Dijkstra1) {
  TrojanMap m;
  
  // Test from Ralphs to Chick-fil-A
  auto path = m.CalculateShortestPath_Dijkstra("Ralphs", "Chick-fil-A");
  std::vector<std::string> gt{
      "2578244375","4380040154","4380040153","4380040152","4380040148","6818427920","6818427919",
      "6818427918","6818427892","6818427898","6818427917","6818427916","7232024780","6813416145",
      "6813416154","6813416153","6813416152","6813416151","6813416155","6808069740","6816193785",
      "6816193786","123152294","4015203136","4015203134","4015203133","21098539","6389467809",
      "4015203132","3195897587","4015203129","4015203127","6352865690","6813379589","6813379483",
      "3402887081","6814958394","3402887080","602606656","4872897515","4399697589","6814958391",
      "123209598","6787673296","122728406","6807762271","4399697304","4399697302","5231967015",
      "1862347583","3233702827","4540763379","6819179753","6820935900","6820935901","6813379556",
      "6820935898","1781230450","1781230449","4015405542","4015405543","1837212104","1837212107",
      "2753199985","6820935907","1837212100","4015372458","6813411588","1837212101","6814916516",
      "6814916515","6820935910","4547476733"}; // Expected path
  // Print the path lengths
  std::cout << "My path length: "  << m.CalculatePathLength(path) << "miles" << std::endl;
  std::cout << "GT path length: " << m.CalculatePathLength(gt) << "miles" << std::endl;
  EXPECT_EQ(path, gt);
  
  // Reverse the input from Ralphs to Chick-fil-A
  path = m.CalculateShortestPath_Dijkstra("Chick-fil-A", "Ralphs");
  std::reverse(gt.begin(),gt.end()); // Reverse the path

  // Print the path lengths
  std::cout << "My path length: "  << m.CalculatePathLength(path) << "miles" << std::endl;
  std::cout << "GT path length: " << m.CalculatePathLength(gt) << "miles" << std::endl;
  EXPECT_EQ(path, gt);
}

// Test CalculateShortestPath_Dijkstra function
TEST(TrojanMapTest, CalculateShortestPath_Dijkstra2) {
  TrojanMap m;
  
  // Test from Target to Ramen KenJo
  auto path = m.CalculateShortestPath_Dijkstra("Target", "Ramen KenJo");
  std::vector<std::string> gt{"5237417650", "6814769289", "6813379584", "6813360961", "6813379480", "6813360960", "6814620880", "6814820020", "6813379414", "6814820008", "5237417652", "6813379473"}; // Expected path
  // Print the path lengths
  std::cout << "My path length: "  << m.CalculatePathLength(path) << "miles" << std::endl;
  std::cout << "GT path length: " << m.CalculatePathLength(gt) << "miles" << std::endl;
  EXPECT_EQ(path, gt);
  
  // Reverse the input from Target to Ramen KenJo
  path = m.CalculateShortestPath_Dijkstra("Ramen KenJo", "Target");
  std::reverse(gt.begin(),gt.end()); // Reverse the path

  // Print the path lengths
  std::cout << "My path length: "  << m.CalculatePathLength(path) << "miles" << std::endl;
  std::cout << "GT path length: " << m.CalculatePathLength(gt) << "miles" << std::endl;
  EXPECT_EQ(path, gt);
}

// Test CalculateShortestPath_Dijkstra function
TEST(TrojanMapTest, CalculateShortestPath_Dijkstra3) {
  TrojanMap m;
  
  // Test from CVS Pharmacy to USC Parking
  auto path = m.CalculateShortestPath_Dijkstra("CVS Pharmacy", "USC Parking");
  std::vector<std::string> gt{ "3088548446", "6813565328", "6045067410", "6045067407" }; // Expected path
  // Print the path lengths
  std::cout << "My path length: "  << m.CalculatePathLength(path) << "miles" << std::endl;
  std::cout << "GT path length: " << m.CalculatePathLength(gt) << "miles" << std::endl;
  EXPECT_EQ(path, gt);
  
  // Reverse the input from CVS Pharmacy to USC Parking
  path = m.CalculateShortestPath_Dijkstra("USC Parking", "CVS Pharmacy");
  std::reverse(gt.begin(),gt.end()); // Reverse the path

  // Print the path lengths
  std::cout << "My path length: "  << m.CalculatePathLength(path) << "miles" << std::endl;
  std::cout << "GT path length: " << m.CalculatePathLength(gt) << "miles" << std::endl;
  EXPECT_EQ(path, gt);
}

TEST(TrojanMapTest, CalculateShortestPath_Dijkstra_Incorrect) {
  TrojanMap m;
  m.CreateGraphFromCSVFile();
  auto path = m.CalculateShortestPath_Dijkstra("Chipotle Mexican Grill", "Target"); //not reachable 
  std::vector<std::string> gt{}; 
  EXPECT_EQ(path, gt);
  path = m.CalculateShortestPath_Dijkstra("Ralp", "Target");  //wrong first name
  std::vector<std::string> gt2{}; 
  EXPECT_EQ(path, gt2);
  path = m.CalculateShortestPath_Dijkstra("Ralphs", "Tarr"); //wrong second name
  std::vector<std::string> gt3{}; 
  EXPECT_EQ(path, gt3);
}

// -------------------------------------------------------------------------------------------------------------------------------------
// // Test CalculateShortestPath_Bellman_Ford function
// TEST(TrojanMapTest, CalculateShortestPath_Bellman_Ford) {
//   TrojanMap m;
  
//   // Test from Ralphs to Chick-fil-A
//   auto path = m.CalculateShortestPath_Bellman_Ford("Ralphs", "Chick-fil-A");
//   std::vector<std::string> gt{
//       "2578244375","4380040154","4380040153","4380040152","4380040148","6818427920","6818427919",
//       "6818427918","6818427892","6818427898","6818427917","6818427916","7232024780","6813416145",
//       "6813416154","6813416153","6813416152","6813416151","6813416155","6808069740","6816193785",
//       "6816193786","123152294","4015203136","4015203134","4015203133","21098539","6389467809",
//       "4015203132","3195897587","4015203129","4015203127","6352865690","6813379589","6813379483",
//       "3402887081","6814958394","3402887080","602606656","4872897515","4399697589","6814958391",
//       "123209598","6787673296","122728406","6807762271","4399697304","4399697302","5231967015",
//       "1862347583","3233702827","4540763379","6819179753","6820935900","6820935901","6813379556",
//       "6820935898","1781230450","1781230449","4015405542","4015405543","1837212104","1837212107",
//       "2753199985","6820935907","1837212100","4015372458","6813411588","1837212101","6814916516",
//       "6814916515","6820935910","4547476733"}; // Expected path
//   // Print the path lengths
//   std::cout << "My path length: "  << m.CalculatePathLength(path) << "miles" << std::endl;
//   std::cout << "GT path length: " << m.CalculatePathLength(gt) << "miles" << std::endl;
//   EXPECT_EQ(path, gt);
  
//   // Reverse the input from Ralphs to Chick-fil-A
//   path = m.CalculateShortestPath_Bellman_Ford("Chick-fil-A", "Ralphs");
//   std::reverse(gt.begin(),gt.end()); // Reverse the path

//   // Print the path lengths
//   std::cout << "My path length: "  << m.CalculatePathLength(path) << "miles" << std::endl;
//   std::cout << "GT path length: " << m.CalculatePathLength(gt) << "miles" << std::endl;
//   EXPECT_EQ(path, gt);
// }

// // Test CalculateShortestPath_Bellman_Ford function
// TEST(TrojanMapTest, CalculateShortestPath_Bellman_Ford2) {
//   TrojanMap m;
  
//   // Test from Target to Ramen KenJo
//   auto path = m.CalculateShortestPath_Bellman_Ford("Target", "Ramen KenJo");
//   std::vector<std::string> gt{"5237417650", "6814769289", "6813379584", "6813360961", "6813379480", "6813360960", "6814620880", "6814820020", "6813379414", "6814820008", "5237417652", "6813379473"}; // Expected path
//   // Print the path lengths
//   std::cout << "My path length: "  << m.CalculatePathLength(path) << "miles" << std::endl;
//   std::cout << "GT path length: " << m.CalculatePathLength(gt) << "miles" << std::endl;
//   EXPECT_EQ(path, gt);
  
//   // Reverse the input from Target to Ramen KenJo
//   path = m.CalculateShortestPath_Bellman_Ford("Ramen KenJo", "Target");
//   std::reverse(gt.begin(),gt.end()); // Reverse the path

//   // Print the path lengths
//   std::cout << "My path length: "  << m.CalculatePathLength(path) << "miles" << std::endl;
//   std::cout << "GT path length: " << m.CalculatePathLength(gt) << "miles" << std::endl;
//   EXPECT_EQ(path, gt);
// }

// // Test CalculateShortestPath_Bellman_Ford function
// TEST(TrojanMapTest, CalculateShortestPath_Bellman_Ford3) {
//   TrojanMap m;
  
//   // Test from CVS Pharmacy to USC Parking
//   auto path = m.CalculateShortestPath_Bellman_Ford("CVS Pharmacy", "USC Parking");
//   std::vector<std::string> gt{ "3088548446", "6813565328", "6045067410", "6045067407" }; // Expected path
//   // Print the path lengths
//   std::cout << "My path length: "  << m.CalculatePathLength(path) << "miles" << std::endl;
//   std::cout << "GT path length: " << m.CalculatePathLength(gt) << "miles" << std::endl;
//   EXPECT_EQ(path, gt);
  
//   // Reverse the input from CVS Pharmacy to USC Parking
//   path = m.CalculateShortestPath_Bellman_Ford("USC Parking", "CVS Pharmacy");
//   std::reverse(gt.begin(),gt.end()); // Reverse the path

//   // Print the path lengths
//   std::cout << "My path length: "  << m.CalculatePathLength(path) << "miles" << std::endl;
//   std::cout << "GT path length: " << m.CalculatePathLength(gt) << "miles" << std::endl;
//   EXPECT_EQ(path, gt);
// }

// TEST(TrojanMapTest, CalculateShortestPath_Bellman_Ford_Incorrect) {
//   TrojanMap m;
//   m.CreateGraphFromCSVFile();
//   auto path = m.CalculateShortestPath_Bellman_Ford("Chipotle Mexican Grill", "Target"); //not reachable 
//   std::vector<std::string> gt{}; 
//   EXPECT_EQ(path, gt);
//   path = m.CalculateShortestPath_Bellman_Ford("Ralp", "Target");  //wrong first name
//   std::vector<std::string> gt2{}; 
//   EXPECT_EQ(path, gt2);
//   path = m.CalculateShortestPath_Bellman_Ford("Ralphs", "Targ"); //wrong second name
//   std::vector<std::string> gt3{}; 
//   EXPECT_EQ(path, gt3);
// }
// -------------------------------------------------------------------------------------------------------------------------------------

// Test cycle detection function
TEST(TrojanMapTest, CycleDetection) {
  TrojanMap m;
  
  // Test case 1
  std::vector<double> square1 = {-118.299, -118.264, 34.032, 34.011};
  auto sub1 = m.GetSubgraph(square1);
  bool result1 = m.CycleDetection(sub1, square1);
  EXPECT_EQ(result1, true);

  // Test case 2
  std::vector<double> square2 = {-118.290, -118.289, 34.030, 34.020};
  auto sub2 = m.GetSubgraph(square2);
  bool result2 = m.CycleDetection(sub2, square2);
  EXPECT_EQ(result2, false);

  // Test case 3
  std::vector<double> square3 = {-118.280, -118.270, 34.022, 34.015};
  auto sub3 = m.GetSubgraph(square3);
  bool result3 = m.CycleDetection(sub3, square3);
  EXPECT_EQ(result3, true);

  // Test case 4
  std::vector<double> square4 = {-118.291, -118.283, 34.022, 34.020};
  auto sub4 = m.GetSubgraph(square4);
  bool result4 = m.CycleDetection(sub4,square4);
  EXPECT_EQ(result4, true);

  // Test case 5
  std::vector<double> square5 = {-118.286, -118.280, 34.022, 34.021};
  auto sub5 = m.GetSubgraph(square5);
  bool result5 = m.CycleDetection(sub5,square5);
  EXPECT_EQ(result5, true);
}

// -------------------------------------------------------------------------------------------------------------------------------------

// Test cycle detection function

TEST(TrojanMapTest, TopologicalSort) {  
  TrojanMap m;

  std::vector<std::string> location_names = {"Ralphs", "Chick-fil-A", "KFC"};
  std::vector<std::vector<std::string>> dependencies = {{"Ralphs","KFC"}, {"Ralphs","Chick-fil-A"}, {"KFC","Chick-fil-A"}};
  auto result = m.DeliveringTrojan(location_names, dependencies);
  std::vector<std::string> gt ={"Ralphs", "KFC","Chick-fil-A"};
  EXPECT_EQ(result, gt);

  std::vector<std::string> location_names1 = {"Cardinal Gardens", "Coffee Bean1","CVS"};
  std::vector<std::vector<std::string>> dependencies1 = {{"Cardinal Gardens","Coffee Bean1"}, {"Cardinal Gardens","CVS"}, {"Coffee Bean1","CVS"}};
  auto result1 = m.DeliveringTrojan(location_names1, dependencies1);
  std::vector<std::string> gt1 ={"Cardinal Gardens", "Coffee Bean1","CVS"};
  EXPECT_EQ(result1, gt1);

  std::vector<std::string> location_names2 = {"USC Village Gym","USC Village Dining Hall","Department of Motor Vehicles","Leavey Library","CVS"};
  std::vector<std::vector<std::string>> dependencies2 = {{"USC Village Gym","USC Village Dining Hall"},{"Department of Motor Vehicles","USC Village Gym"},{"Leavey Library","USC Village Gym"},{"Department of Motor Vehicles","CVS"}};
  auto result2 = m.DeliveringTrojan(location_names2, dependencies2);
  std::vector<std::string> gt2 ={"USC Village Gym", "USC Village Dining Hall", "Department of Motor Vehicles", "CVS", "Leavey Library"} ;
  EXPECT_EQ(result2, gt2);
}

// -------------------------------------------------------------------------------------------------------------------------------------

// Phase 3
// Test TSP function
TEST(TrojanMapTest, TSP1) {
  TrojanMap m;
  
  std::vector<std::string> input{"6819019976","6820935923","122702233","8566227783","8566227656","6816180153","1873055993","7771782316"}; // Input location ids 
  auto result = m.TravellingTrojan_Brute_Force_Iterative(input);
  std::cout << "My path length: "  << result.first << "miles" << std::endl; // Print the result path lengths
  std::vector<std::string> gt{"6819019976","1873055993","8566227656","122702233","8566227783","6816180153","7771782316","6820935923","6819019976"}; // Expected order
  std::cout << "GT path length: "  << m.CalculatePathLength(gt) << "miles" << std::endl; // Print the gt path lengths
  bool flag = false;
  if (gt == result.second[result.second.size()-1]) // clockwise
    flag = true;
  std::reverse(gt.begin(),gt.end()); // Reverse the expected order because the counterclockwise result is also correct
  if (gt == result.second[result.second.size()-1]) 
    flag = true;
  
  EXPECT_EQ(flag, true);
}

// Test TSP function 2
TEST(TrojanMapTest, TSP2) {
  TrojanMap m;
  m.CreateGraphFromCSVFile();
  std::vector<std::string> input{"4015405538","122817160","4400281325","123209602","6799196824","123009684","4015203136","63068643"}; // Input location ids  
  auto result = m.TravellingTrojan_Brute_Force_Iterative(input);
  std::cout << "My path length: " << result.first << "miles" << std::endl; // Print the result path lengths
  std::vector<std::string> gt{"4015405538","4400281325","6799196824","123209602","4015203136","63068643","122817160","123009684","4015405538"}; // Expected order
  std::cout << "GT path length: " << m.CalculatePathLength(gt) << "miles" << std::endl; // Print the groundtruth path lengths
  bool flag = false;
  if (gt == result.second[result.second.size()-1]) // clockwise
    flag = true;
  std::reverse(gt.begin(),gt.end()); // Reverse the expected order because the counterclockwise result is also correct
  if (gt == result.second[result.second.size()-1]) // counterclockwise
    flag = true;
  EXPECT_EQ(flag, true);
}

TEST(TrojanMapTest, TSP3) {
  TrojanMap m;
  
  std::vector<std::string> input{"6819019976","6820935923","122702233","8566227783","8566227656","6816180153","1873055993","7771782316"}; // Input location ids 
  auto result = m.TravellingTrojan_Backtracking(input);
  std::cout << "My path length: "  << result.first << "miles" << std::endl; // Print the result path lengths
  std::vector<std::string> gt{"6819019976","1873055993","8566227656","122702233","8566227783","6816180153","7771782316","6820935923","6819019976"}; // Expected order
  std::cout << "GT path length: "  << m.CalculatePathLength(gt) << "miles" << std::endl; // Print the gt path lengths
  bool flag = false;
  if (gt == result.second[result.second.size()-1]) 
    flag = true;
  std::reverse(gt.begin(),gt.end()); 
  if (gt == result.second[result.second.size()-1]) 
    flag = true;
  
  EXPECT_EQ(flag, true);
}

TEST(TrojanMapTest, TSP4) {
  TrojanMap m;
  m.CreateGraphFromCSVFile();
  std::vector<std::string> input{"4015405538","122817160","4400281325","123209602","6799196824","123009684","4015203136","63068643"}; // Input location ids  
  auto result = m.TravellingTrojan_Backtracking(input);
  std::cout << "My path length: " << result.first << "miles" << std::endl; // Print the result path lengths
  std::vector<std::string> gt{"4015405538","4400281325","6799196824","123209602","4015203136","63068643","122817160","123009684","4015405538"}; // Expected order
  std::cout << "GT path length: " << m.CalculatePathLength(gt) << "miles" << std::endl; // Print the groundtruth path lengths
  bool flag = false;
  if (gt == result.second[result.second.size()-1]) // clockwise
    flag = true;
  std::reverse(gt.begin(),gt.end()); // Reverse the expected order because the counterclockwise result is also correct
  if (gt == result.second[result.second.size()-1]) // counterclockwise
    flag = true;
  EXPECT_EQ(flag, true);
}

TEST(TrojanMapTest, TSP5) {
  TrojanMap m;
  
  std::vector<std::string> input{"6819019976","6820935923","122702233","8566227783","8566227656","6816180153","1873055993","7771782316"}; // Input location ids 
  auto result = m.TravellingTrojan_2opt(input);
  std::cout << "My path length: "  << result.first << "miles" << std::endl; // Print the result path lengths
  std::vector<std::string> gt{"6819019976","1873055993","8566227656","122702233","8566227783","6816180153","7771782316","6820935923","6819019976"}; // Expected order
  std::cout << "GT path length: "  << m.CalculatePathLength(gt) << "miles" << std::endl; // Print the gt path lengths
  bool flag = false;
  if (gt == result.second[result.second.size()-1]) // clockwise
    flag = true;
  std::reverse(gt.begin(),gt.end()); // Reverse the expected order because the counterclockwise result is also correct
  if (gt == result.second[result.second.size()-1]) 
    flag = true;
  
  EXPECT_EQ(flag, true);
}

TEST(TrojanMapTest, TSP6) {
  TrojanMap m;
  m.CreateGraphFromCSVFile();
  std::vector<std::string> input{"4015405538","122817160","4400281325","123209602","6799196824","123009684","4015203136","63068643"}; // Input location ids  
  auto result = m.TravellingTrojan_2opt(input);
  std::cout << "My path length: " << result.first << "miles" << std::endl; // Print the result path lengths
  std::vector<std::string> gt{"4015405538","4400281325","6799196824","123209602","4015203136","63068643","122817160","123009684","4015405538"}; // Expected order
  std::cout << "GT path length: " << m.CalculatePathLength(gt) << "miles" << std::endl; // Print the groundtruth path lengths
  bool flag = false;
  if (gt == result.second[result.second.size()-1]) // clockwise
    flag = true;
  std::reverse(gt.begin(),gt.end()); // Reverse the expected order because the counterclockwise result is also correct
  if (gt == result.second[result.second.size()-1]) // counterclockwise
    flag = true;
  EXPECT_EQ(flag, false);
}

// -------------------------------------------------------------------------------------------------------------------------------------

// // Test FindNearby points
// TEST(TrojanMapTest, FindNearby) {
//   TrojanMap m;
  
//   auto result = m.FindNearby("supermarket", "Ralphs", 10, 10);
//   std::vector<std::string> ans{"5237417649", "6045067406", "7158034317"};
//   EXPECT_EQ(result, ans);
// }