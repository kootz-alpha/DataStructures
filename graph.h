#include<iostream>
#include<vector>
#include<queue>
#include<set>
#include<stack>
#include<algorithm>
#include<numeric>
#include<climits>

// Follow 1-based indexing or numbering for the nodes. Currently not designed for multi-edges or loops or negative weights

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
            min = -1;
        
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

        vector<int> minDistance(int source) { // returns min distance from source to all other nodes. graph can be weighted or unweighted

            vector<int> minDist(numNodes, -1);
            minDist[source-1] = 0;

            if (weighted) {

                vector<int> unchecked(numNodes);
                iota(unchecked.begin(), unchecked.end(), 1);

                while (unchecked.size() > 0) {

                    int min = minNode(unchecked, minDist); 

                    if (minDist[min-1] == -1)
                        break;
                    
                    int i = 0, cost = minDist[min-1];
                    
                    while(i<numNodes) {
                        
                        if (adjMatrix[min-1][i] != -1) {
                            if (cost+adjMatrix[min-1][i] < minDist[i] || minDist[i] == -1) {
                                minDist[i] = cost+adjMatrix[min-1][i];
                            }
                        }
                        i++;
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

                        if (minDist[k-1] == -1) {
                            minDist[k-1] = d;
                            q.push(k);
                        }
                    }
                    q.pop();
                }                
            }
            return minDist;     
        }

        // returns min path and min distance from source to destination. graph can be weighted or unweighted. return value is a vector whose
        // last value represents min distance and first values represent the min path. if no path exist, then vector will have only one value, -1
        vector<int> minDistance(int source, int destination) { 

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
                    
                    int i = 0, cost = minDist[min-1];
                    
                    while(i<numNodes) {
                        
                        if (adjMatrix[min-1][i] != -1) {
                            if (cost+adjMatrix[min-1][i] < minDist[i] || minDist[i] == -1) {
                                minDist[i] = cost+adjMatrix[min-1][i];
                                parent[i] = min;
                            }
                        }
                        i++;
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

                        if (k == destination) {
                            parent[k-1] = i+1;
                            minDist[k-1] = d;
                            break;
                        }

                        if (minDist[k-1] == -1) {
                            minDist[k-1] = d;
                            parent[k-1] = i+1;
                            q.push(k);
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
};

int main() {

    vector<vector<int>> edges = {{1,2, 11}, {1,3, 9}, {2,3, 3}, {3,4, 2}, {2, 4, 1}};
    int n = 4, e = 4;

    // will automatically detect whether weighted or not
    
    Graph a = Graph(n, edges); // undirected, adjacency list
    
    Graph b = Graph(n, edges, 0, 1); // directed  adjacency list
    
    Graph a1 = Graph(n, edges, 1); // undirected adjacency matrix
    
    Graph b1 = Graph(n, edges, 1, 1); // directed adjacency matrix

    vector<int> c = a.minDistance(1); // from node 1 to all other nodes

    vector<int> d = a.minDistance(1, 4); // get min path and distance from node 1 to node 4

    for (auto i : d) {
            cout<<i<<" ";
    }
    cout<<endl;
    
    if (b.containCycle())
        cout<<"yes";
    else 
        cout<<"no";
    
    return 0;
}
