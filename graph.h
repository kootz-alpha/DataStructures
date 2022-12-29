#include<iostream>
#include<vector>
#include<queue>
#include<set>
#include<stack>
#include<algorithm>
#include<numeric>

// Follow 1-based indexing or numbering for the nodes

using namespace std;

class Graph {

    int numNodes = -1, numEdges = -1, directed = 0, weighted = 0;  // 0 = undirected (unweighted), 1 = directed (weighted). -1 means they aren't yet defined


    // just a utility function for the function minDistance
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

    public :

        vector<set<int>> adjList;
        vector<vector<int>> adjMatrix;    // -1 if there's no edge


        Graph(int numN, int numE, vector<vector<int>> edgeList) { // creates an adjacency list
            
            numNodes = numN;
            numEdges = numE;
            adjList.resize(numNodes);

            for (auto k : edgeList) {
                
                int x = k[0], y = k[1];
                adjList[x-1].insert(y);
                adjList[y-1].insert(x);
            }
        }

        Graph(int numN, int numE, vector<vector<int>> edgeList, int w) { // pass any random integer as 'w'. This one creates an adjacency matrix
            
            numNodes = numN;
            numEdges = numE;
            adjMatrix.resize(numNodes, vector<int>(numNodes, -1));
            weighted = 1;

            for (auto k : edgeList) {
                
                int x = k[0], y = k[1], c = k[2];
                adjMatrix[x-1][y-1] = c;
                adjMatrix[y-1][x-1] = c;
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
    
        bool containCycle() { // initially pass parent and current as same

                cout<<"inside"<<endl;

                vector<bool> visited(numNodes, false);

                for (int i = 0; i < numNodes; i++) {

                    if (!visited[i]) {

                        if (containCycle(-1, i+1, visited))
                            return true;
                    }
                }

                return false;

        }

        bool containCycle(int parent, int current, vector<bool>& visited) { // initially pass parent and current as same

                visited[current-1] = true;

                for (int neighbour : adjList[current-1]) {

                    if (!visited[neighbour-1]) {

                        if (containCycle(current, neighbour, visited))
                            return true;
                    }
                    else if (neighbour != parent)
                        return true;
                }

                return false;
        }


};

int main() {

    vector<vector<int>> edges = {{1,2, 11}, {1,3, 9}, {2,3, 3}, {3,4, 2}, {2, 4, 1}};
    int n = 4, e = 4;

    Graph a = Graph(n, e, edges, 2); // weighted
    
    Graph b = Graph(n, e, edges); // unweighted

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
