#include<iostream>
#include<vector>
#include<queue>
#include<set>
#include<stack>
#include<algorithm>
#include<numeric>
#include<climits>

// Follow 1-based indexing or numbering for the nodes. Currently not designed for multi-edges or loops or negative weights

/*
    1. Graph() - Create directed, undirected, weighted, unweighted graphs in the form of adjacency list or matrix.
    2. minDistance(*) - Calculate minimum distance between a source node and all other nodes.
    3. minDistance(*, *) - Calculate minimum distance and path between two nodes.
    4. containCycle() - Check whether a graph contains a cycle or not.
    5. connectedComponents() - Returns a list of connected components in an undirected graph.
    6. mstKruskal() - Calculate the size and edges of the MST of an undirected graph.

*/    

using namespace std;

class Graph {

    int numNodes = -1, numEdges = -1;  
    bool directed = false, weighted = false;

   
    // utility for minDistance()
    int minNode(vector<int> unchecked, vector<int> result) { 
        int min = INT_MAX;
        bool p = true;
        
        for(int i = 0; i < unchecked.size(); i++) {
            if ((result[unchecked[i]-1] != -1) && (result[unchecked[i]-1] < min)) {
                min = unchecked[i];
                p = false;
            }
        }
        if (p)
            min = unchecked[0];
        
        return min;    
    }
    
    // utility for containCycle()
    bool containCycle(int node, vector<int>& visited) {
            
        visited[node-1] = 1;
        
        for (auto neighbour : adjList[node-1]) {
            
            if (visited[neighbour[0]-1] == 0) {
                
                if (containCycle(neighbour[0], visited))
                    return true;
            }
            else if (visited[neighbour[0]-1] == 1)
                return true;
        }
        visited[node-1] = 2;
        
        return false;
    }

    // utility for containCycle()
    bool containCycle(int parent, int current, vector<int>& visited) { 

        visited[current-1] = 1;

        for (auto neighbour : adjList[current-1]) {

            if (visited[neighbour[0]-1] == 0) {

                if (containCycle(current, neighbour[0], visited))
                    return true;
            }
            else if (neighbour[0] != parent)
                return true;
        }

        return false;
    }
        
        
    public :

        vector<vector<vector<int>>> adjList;
        vector<vector<int>> adjMatrix;    


        Graph(int numN, vector<vector<int>> edgeList, int listOrMatrix = 0, bool d = false) { 
            
            // listOrMatrix = 0, creates an adjacency list. Otherwise creates an adjacency matrix
            // d = true creates a directed graph
            
            numNodes = numN;
            numEdges = edgeList.size();
            
            if (edgeList[0].size() == 3) 
                weighted = true;
                
            directed = d;
            
            if (listOrMatrix == 0) {
                
                adjList.resize(numNodes);
                
                if (weighted) {
                    
                    if (!directed) {
    
                        for (auto k : edgeList) {
                        
                            int x = k[0], y = k[1], c = k[2];
                            adjList[x-1].push_back({y, c});
                            adjList[y-1].push_back({x, c});
                        }
                    }
                    else {
    
                        for (auto k : edgeList) {
                        
                            int x = k[0], y = k[1], c =k[2];
                            adjList[x-1].push_back({y, c});;
                        }
                    }
                }
                else {

                    if (!directed) {
        
                        for (auto k : edgeList) {
                        
                            int x = k[0], y = k[1];
                            adjList[x-1].push_back({y, 1});
                            adjList[y-1].push_back({x, 1});
                        }
                    }
                    else {
        
                        for (auto k : edgeList) {
                        
                            int x = k[0], y = k[1];
                            adjList[x-1].push_back({y, 1});
                        }
                    }
                }
            }
            else {
                
                adjMatrix.resize(numNodes, vector<int>(numNodes, -1));
                
                if (weighted) {
                    
                    if (!directed) {

                        for (auto k : edgeList) {
                    
                            int x = k[0], y = k[1], c = k[2];
                            adjMatrix[x-1][y-1] = c;
                            adjMatrix[y-1][x-1] = c;
                        }
                    }
                    else {
    
                        for (auto k : edgeList) {
                    
                            int x = k[0], y = k[1], c = k[2];
                            adjMatrix[x-1][y-1] = c;
                        }
                    }
                }
                else {
                    
                    if (!directed) {

                        for (auto k : edgeList) {
                    
                            int x = k[0], y = k[1], c = 1;
                            adjMatrix[x-1][y-1] = c;
                            adjMatrix[y-1][x-1] = c;
                        }
                    }
                    else {
                        
                        for (auto k : edgeList) {
                    
                            int x = k[0], y = k[1], c = 1;
                            adjMatrix[x-1][y-1] = c;
                        }
                    }
                }
            }
            
        }

        vector<int> minDistance(int source) { 
            // returns min distance from source to all other nodes. graph can be weighted or unweighted
            // requires the graph to be in adjacent list form

            vector<int> minDist(numNodes, -1);
            minDist[source-1] = 0;

            if (weighted) {

                vector<int> unchecked(numNodes);
                iota(unchecked.begin(), unchecked.end(), 1);

                while (unchecked.size() > 0) {

                    int min = minNode(unchecked, minDist); 

                    if (minDist[min-1] == -1)
                        break;
                    
                    for (auto neighbour : adjList[min-1]) {
                        
                        int cost = minDist[min-1] + neighbour[1];
                        
                        if (cost < minDist[neighbour[0]-1] || minDist[neighbour[0]-1] == -1) {
                            
                            minDist[neighbour[0]-1] = cost;
                        }
                    }
                    unchecked.erase(find(unchecked.begin(), unchecked.end(), min));
                }              
            }
            else {

                queue<int> q;
                q.push(source);

                while (!q.empty()) {

                    int i = q.front()-1, d = minDist[i]+1;

                    for (auto k : adjList[i]) {

                        if (minDist[k[0]-1] == -1) {
                            minDist[k[0]-1] = d;
                            q.push(k[0]);
                        }
                    }
                    q.pop();
                }                
            }
            return minDist;     
        }

        vector<int> minDistance(int source, int destination) { 
            
            // requires graph to be in adjacent list form
            // returns min path and min distance from source to destination. graph can be weighted or unweighted. return value is a vector whose
            // last value represents min distance and first values represent the min path. if no path exist, then vector will have only one value, -1

            vector<int> minDist(numNodes, -1), parent(numNodes, -1);
            minDist[source-1] = 0, parent[source-1] = source;

            if (weighted) {

                vector<int> unchecked(numNodes);
                iota(unchecked.begin(), unchecked.end(), 1);

                while (unchecked.size() > 0) {

                    int min = minNode(unchecked, minDist); 

                    if (min == destination)
                        break;

                    if (minDist[min-1] == -1)
                        break;
                    
                    for (auto neighbour : adjList[min-1]) {
                        
                        int cost = minDist[min-1] + neighbour[1];
                        
                        if (cost < minDist[neighbour[0]-1] || minDist[neighbour[0]-1] == -1) {
                            
                            minDist[neighbour[0]-1] = cost;
                            parent[neighbour[0]-1] = min;
                        }
                    }
                    unchecked.erase(find(unchecked.begin(), unchecked.end(), min));
                }              
            }
            else {

                queue<int> q;
                q.push(source);

                while (!q.empty()) {

                    int i = q.front()-1, d = minDist[i]+1;

                    for (auto k : adjList[i]) {

                        if (k[0] == destination) {
                            parent[k[0]-1] = i+1;
                            minDist[k[0]-1] = d;
                            break;
                        }

                        if (minDist[k[0]-1] == -1) {
                            minDist[k[0]-1] = d;
                            parent[k[0]-1] = i+1;
                            q.push(k[0]);
                        }
                    }
                    q.pop();

                    if (minDist[destination-1] != -1)
                        break;
                }                
            }

            vector<int> result;
            
            if (minDist[destination-1] == -1)
                result.push_back(-1);
            else {
                
                stack<int> s;
                int p = parent[destination-1];
                s.push(destination);

                while (p != source) {

                    s.push(p);
                    p = parent[p-1];
                }

                result.push_back(source);

                while (!s.empty()) {

                    result.push_back(s.top());
                    s.pop();
                }

                result.push_back(minDist[destination-1]);
            }
            
            return result;     
        }
        
        bool containCycle() { 
            
            // requires graph in adjacency list form

            vector<int> visited(numNodes, 0);
            
            if (directed) {
                
                for (int i = 0; i < numNodes; i++) {
                    
                    if (visited[i] == 0) {
                        
                        if (containCycle(i+1, visited))
                            return true;
                    }
                }
            }
            else {
                
                for (int i = 0; i < numNodes; i++) {

                    if (visited[i] == 0) {
    
                        if (containCycle(-1, i+1, visited))
                            return true;
                    }
                }
            }
            return false;
        }
    
        vector<vector<int>> connectedComponents() {

            // requires the graph in adjacent list form
            // only used for undirected graphs

            vector<int> visited(numNodes, 0);
            vector<vector<int>> connected;

            for (int i = 0; i < numNodes; i++) {

                if (visited[i] == 0) {

                    // do bfs
                    vector<int> group;
                    queue<int> q;
                    q.push(i+1);
                    visited[i] = 1;

                    while (!q.empty()) {

                        int front = q.front();
                        group.push_back(front);
                        q.pop();

                        for (auto neighbour : adjList[front-1]) {

                            if (visited[neighbour[0]-1] != 1) {
                                q.push(neighbour[0]);
                                visited[neighbour[0]-1] = 1;
                            }
                        }
                    }
                    connected.push_back(group);
                }
            }
            return connected;
        }
    
        vector<vector<int>> mstKruskal(int numN, vector<vector<int>> edgeList) {
            
            // returns a vector whose last element is the size of the MST, and first elements are the edges of the MST
            // can be used for undirected graph. if unweighted, create edgeList with weight 1
            
            sort(edgeList.begin(), edgeList.end(), [](vector<int> a, vector<int> b) -> bool {
                if (a[2]<=b[2])
                    return true;
                return false;
            });
            
            int size = 0, e = 0;
            vector<vector<int>> kruskal;
            vector<int> parent(numN);
            iota(parent.begin(), parent.end(), 1);
            vector<int> visited(numN, 0);
            
            for (auto edge : edgeList) {
                
                if (e == numN-1)
                    break;
                
                if (visited[edge[0]-1] == 0 && visited[edge[1]-1] == 0) {
                    
                    kruskal.push_back(edge);
                    size += edge[2];
                    e++;
                    parent[edge[1]-1] = edge[0];
                    visited[edge[0]-1] = 1;
                    visited[edge[1]-1] = 1;
                }
                else {
                    
                    int p1 = parent[edge[0]-1], p2 = parent[edge[1]-1];
                    
                    while (p1 != parent[p1-1] || p2 != parent[p2-1]) {
                        p1 = parent[p1-1];
                        p2 = parent[p2-1];
                    }
                    parent[edge[0]-1] = p1;
                    parent[edge[1]-1] = p2;
                    
                    if (p1 != p2) {
                        
                        kruskal.push_back(edge);
                        size += edge[2];
                        e++;
                        
                        if (visited[edge[0]-1] == 0) {
                            
                            parent[edge[0]-1] = p2;
                            visited[edge[0]-1] = 1;
                        }
                        else {
                            
                            parent[edge[1]-1] = p1;
                            visited[edge[1]-1] = 1;
                        }
                    }
                }
            }
            kruskal.push_back({size});
            return kruskal;
        }
};

int main() {

    vector<vector<int>> edges = {{1,2, 11}, {1,3, 9}, {2,3, 3}, {3,4, 2}, {2, 4, 1}};
    int n = 4;

    // will automatically detect whether weighted or not
    
    Graph a = Graph(n, edges); // undirected, adjacency list
    
    Graph b = Graph(n, edges, 0, 1); // directed  adjacency list
    
    Graph a1 = Graph(n, edges, 1); // undirected adjacency matrix
    
    Graph b1 = Graph(n, edges, 1, 1); // directed adjacency matrix

    vector<int> c = a.minDistance(1); // from node 1 to all other nodes

    vector<int> d = a.minDistance(1, 4); // get min path and distance from node 1 to node 4

    bool t = b.containCycle();  // returns true if cycle exists
    
    if (b.containCycle())
        cout<<"yes";
    else 
        cout<<"no";
    
    return 0;
}
