# **Amulya Kallakuri, Nithyashree Manohar**
# **Project Title: Campus Navigation System**

## **Functions Implemented:** 

### **1. Autocomplete**

std::vector<std::string> Autocomplete(std::string name);

When we run this function we can type the partial name of the location and the function will return a list of possible locations with partial name as prefix. Uppercase and lowercase alphabets are treated as the same character by using std::tolower. To check if the input name matches the name in the database, we call std::substring to compare the two strings' size. 

Time complexity is O(n) where n is the number of nodes.

**Example:**\
Input: "po"\
Output: ["Popeyes 1", "Porterhouse Fried Chicken", "Popeyes"]

**Time spent:**\
30ms for the above example

**Discussion, conclusion, and lessons learned:**\
We learned to use std::transform, std::substring and std::tolower

<img width="516" alt="Screen Shot 2022-05-01 at 4 38 27 PM" src="https://user-images.githubusercontent.com/98286997/166169237-06621fad-9737-43b9-b848-20c83e1f9d57.png">


### **2. Get Position**

std::pair<double, double> GetPosition(std::string name);

This function is used to find the latitude and longitude of a given place. When the user inputs the name of the place, the function returns the latitude and longitude of this place if it is present in the map, if it is not then the function returns (-1,-1). Helper functions GetLat and GetLon are used to extract values of latitude and longitude, respectively.

Time complexity is O(n) where n is the number of nodes. 

**Example:**\
Input: "USC Parking"\
Output: (34.0239, -118.2800)

**Time spent:**\
30 seconds  

**Discussion, conclusion, and lessons learned:**\
To check if the given input is on the map, we loop through all the map elements using a if loop to filter satisfied name as the output. 

<img width="516" alt="Screen Shot 2022-05-01 at 5 12 01 PM" src="https://user-images.githubusercontent.com/98286997/166170541-bc79deb4-2825-443c-9d94-a6cbd831adbc.png">


### **3. Edit Distance**

int CalculateEditDistance(std::string name1, std::string name2);\
std::string FindClosestName(std::string name);

When entering a location name that does not exist in the map, the map will determine whether the input can be replaced with a "similar name" or not. Similar names refer to the names that exist in the map with a smallest distance from the original input. The distance between two names A and B is the minimum number of operations required to convert A to B. There are 3 operations:

Insert a character\
Delete a character\
Replace a character

If the exact match is found, it will be shown on the map. Otherwise, the map will show the most similar name by using FindClosestName and print a warning. For example, if I type Rolphs, I should get a warning like "Did you mean Ralphs instead of Rolphs?"
We can use dynamic programming to calculate edit distance.

Time complexity is O(n) where n is the number of nodes. 

**Example:**\
Input: "Rolphs", "Ralphs"\
Output: 1

**Time spent:**\
30 ms 

**Discussion, conclusion, and lessons learned:**\
Practice in Dynamic Programming 

<img width="516" alt="Screen Shot 2022-05-01 at 4 40 17 PM" src="https://user-images.githubusercontent.com/98286997/166169317-6e6f40a9-6f70-424e-ac77-adf98d944591.png">

### **4. Shortest Path - Dijkstras and Bellman-Ford**

To find the shortest path we use Dijkstra’s algorithm and bellman ford algorithms. In this application, Dijkstra’s algorithm visits each node only once and works better because every edge is greater than one and hence there are no negative edges or negative cycles.  For the same reason, Bellman-Ford is not very helpful in this situation. Bellman-Ford can visit each node more than once and there is a significantly larger number of edges compared to the nodes, so the time taken by Bellman-Ford is greater than the time taken by Dijkstra’s. It works well for sparse graphs but takes a long time for our application

Dijkstra’s Time complexity\
O(mlogn) where, m=nodes and n=edges

Bellman-Ford Time complexity\
O(m*n) where, m=nodes and n=edges


**Example:**\
Start Location: "Ralphs"\
Destination: "Chick-fil-A"

Dijkstra's Output: 
"2578244375","4380040154","4380040153","4380040152","4380040148","6818427920","6818427919","6818427918","6818427892","6818427898","6818427917","6818427916","7232024780","6813416145","6813416154","6813416153","6813416152","6813416151","6813416155","6808069740","6816193785","6816193786","123152294","4015203136","4015203134","4015203133","21098539","6389467809","4015203132","3195897587","4015203129","4015203127","6352865690","6813379589","6813379483","3402887081","6814958394","3402887080","602606656","4872897515","4399697589","6814958391","123209598","6787673296","122728406","6807762271","4399697304","4399697302","5231967015","1862347583","3233702827","4540763379","6819179753","6820935900","6820935901","6813379556","6820935898","1781230450","1781230449","4015405542","4015405543","1837212104","1837212107","2753199985","6820935907","1837212100","4015372458","6813411588","1837212101","6814916516","6814916515","6820935910","4547476733",\
The distance of the path is:1.49919 miles\
Time taken by function: 632 ms

Bellman-Ford Output: 
"2578244375","4380040154","4380040153","4380040152","4380040148","6818427920","6818427919","6818427918","6818427892","6818427898","6818427917","6818427916","7232024780","6813416145","6813416154","6813416153","6813416152","6813416151","6813416155","6808069740","6816193785","6816193786","123152294","4015203136","4015203134","4015203133","21098539","6389467809","4015203132","3195897587","4015203129","4015203127","6352865690","6813379589","6813379483","3402887081","6814958394","3402887080","602606656","4872897515","4399697589","6814958391","123209598","6787673296","122728406","6807762271","4399697304","4399697302","5231967015","1862347583","3233702827","4540763379","6819179753","6820935900","6820935901","6813379556","6820935898","1781230450","1781230449","4015405542","4015405543","1837212104","1837212107","2753199985","6820935907","1837212100","4015372458","6813411588","1837212101","6814916516","6814916515","6820935910","4547476733",\
The distance of the path is:1.49919 miles\
Time taken by function: 9647 ms

**Time spent:**\
Dijkstra's - 632 ms\
Bellman-Ford - 9647 ms

**Discussion, conclusion, and lessons learned:**
In the examples tested with our program, the runtime for Bellman was greater than the runtime for Dijkstra's algorithm in all cases. This problem involved our knowledge of the shortest path algorithms and their implementation and those are the lessons learned.


<img width="516" alt="Screen Shot 2022-05-01 at 4 58 00 PM" src="https://user-images.githubusercontent.com/98286997/166170101-8a607e4b-e5cb-4d64-b8d9-5aa2d355d724.png">

<img width="516" src=https://user-images.githubusercontent.com/98286997/166170385-e885982e-ac33-425e-98dc-4eeb9038b151.jpg>

<img width="516" alt="Screen Shot 2022-05-01 at 5 14 47 PM" src="https://user-images.githubusercontent.com/98286997/166170701-384b9234-0dd3-43ea-b7aa-9d0508c94c23.png">

<img width="516" alt="Screen Shot 2022-05-01 at 6 27 29 PM" src="https://user-images.githubusercontent.com/26215631/166174100-90915897-ccbb-4a06-9cb6-64e908c16be1.png">


### **6. Cycle Detection**

In this section, we use a square-shaped subgraph of the original graph by using four coordinates stored in ```std::vector<double> square```, which follows the order of left, right, upper, and lower bounds. Then try to determine if there is a cycle path in the that subgraph. If it does, return true and report that path on the map. Otherwise return false. Depth First Traversal can be used to detect a cycle in a Graph. DFS for a connected graph produces a tree. There is a cycle in a graph only if there is a back edge present in the graph. A back edge is an edge that is from a node to itself (self-loop) or one of its ancestors in the tree produced by DFS

Time complexity is O(m+n) where n is the number of nodes and m is the number of edges.

**Example:**

Input:
> Please input the left bound longitude(between -118.320 and -118.250):-118.299
Please input the right bound longitude(between -118.320 and -118.250):-118.264
Please input the upper bound latitude(between 34.000 and 34.040):34.032
Please input the lower bound latitude(between 34.000 and 34.040):34.011

Output:
> there exists a cycle in the subgraph

> Time taken by function: 5 ms

**Time spent:**\
5 ms

**Discussion, conclusion, and lessons learned:**
Implementation of cycle detection algorithms and recursive functions.

<img width="516" alt="Screen Shot 2022-05-01 at 5 21 36 PM" src="https://user-images.githubusercontent.com/98286997/166171122-755b4fba-1468-477e-91ba-082de0ceaa91.png">

<img width="516" alt="Screen Shot 2022-05-01 at 5 22 47 PM" src="https://user-images.githubusercontent.com/98286997/166171098-1ccd128b-eb52-4466-819d-0873ff4f5cbf.png">
<img width="516" alt="Screen Shot 2022-05-01 at 6 28 32 PM" src="https://user-images.githubusercontent.com/26215631/166174119-370f0ada-22cf-4063-aafd-acb1033ed2f8.png">


### **7. Topological Sort**

For this function, we added a recursive function to trojanmap.cc called RecursiveTopological which we use in the main Delivering Trojan function.Our recursive function uses the selected node (var: location), the variable visited to indicate a boolean value based on whether the node is visited or not, the adjacency map to figure out which are the neighbouring nodes for the selected nodes, and the result variable to pushback the visited vectors into the result to return from the main function. We call this recursive function for every neighbouring node of the selected node recursively in order to visit all nodes and keep track of which nodes are visited and which are not. The recursive function also pushes back the visited node into the result variable by reference.The main function initialises the visited variable for all nodes as false and the adjacency map relates all nodes to their respective neighbours to use in the recursive function for marking visited based on nodes and their neighbours. Then, we have a for loop that goes thru all the locations and if they have not been visited, the recursive function is called on this location.

Time complexity is O(m+n) where n is the number of nodes and m is the number of edges.

**Example:**

Input:\
USC Village Gym\
USC Village Dining Hall\
Department of Motor Vehicles\
Leavey Library\
CVS

Dependencies:\
USC Village Gym, USC Village Dining Hall\
Department of Motor Vehicles, USC Village Gym\
Leavey Library, USC Village Gym\
Department of Motor Vehicles, CVS

Output:\
Leavey Library\
Department of Motor Vehicles\
CVS\
USC Village Gym\
USC Village Dining Hall


**Time spent:**\
156 ms

**Discussion, conclusion, and lessons learned:**\
Relearnt recursive functions and topological sort

<img width="478" alt="Screen Shot 2022-05-01 at 5 51 53 PM" src="https://user-images.githubusercontent.com/98286997/166172250-fd9c50e6-d3fc-4905-9959-a78f37329fb1.png">
<img width="516" alt="Screen Shot 2022-05-01 at 6 28 20 PM" src="https://user-images.githubusercontent.com/26215631/166174131-dcf7e1e7-106e-45f6-b3f9-745fb66884d1.png">


### **8. Travelling Trojan**

We assume that a complete graph is given to you for this part. That means each node is a neighbor of all other nodes. Given a vector of location ids, assume every location can reach all other locations in the vector (i.e. assume that the vector of location ids is a complete graph). User type the number of nodes N to randomly generate to build the fully connected map. This function find the shortest route that covers all the locations exactly once and goes back to the start point. The program will show the routes on the popup map.

User can choose to use the following algorithms:

Brute Force Method - Recursive and Iterative\
Brute Force with Backtracking\
2-Opt


<img width="516" alt="Screen Shot 2022-05-01 at 6 27 39 PM" src="https://user-images.githubusercontent.com/26215631/166174149-8e6fb192-507d-4010-992d-f05e41cd441e.png">


**Iterative implementation:**

https://user-images.githubusercontent.com/26215631/166173324-18c52f15-7b66-41c5-9dde-6bc204bd13f5.mov


**Recursive implementation:**

https://user-images.githubusercontent.com/26215631/166173322-8cbf02f5-18c7-40e9-bb61-3a1aa86b2e8c.mov


**2-optimal method:**

https://user-images.githubusercontent.com/26215631/166173300-da19b7c8-e1d5-4849-bd26-db0d9e2b7121.mov

