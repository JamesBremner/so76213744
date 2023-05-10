// C++ program to solve Traveling Salesman Problem
// using Branch and Bound.
#include <climits>
#include <cstring>
#include <iostream>
#include "GraphTheory.h" //  https://github.com/JamesBremner/PathFinder

// const int N = 5;

int edgeWeight(const raven::graph::cGraph &g, int i, int j)
{
    return atoi(g.rEdgeAttr(g.find(i, j), 0).c_str());
}

// std::vector<std::vector<int>>
// makeAdjacencyMatrix(const raven::graph::cGraph &g)
// {
//     std::vector<std::vector<int>> M;
//     int N = g.vertexCount();
//     for (int n1 = 0; n1 < N; n1++)
//     {
//         std::vector<int> row;
//         for (int n2 = 0; n2 < N; n2++)
//         {
//             if (n1 == n2)
//                 row.push_back(0);
//             else
//                 row.push_back(edgeWeight(g, n1, n2));
//         }
//         M.push_back(row);
//     }
//     return M;
// }

// raven::graph::cGraph
// makeGraph(int adj[N][N])
// {
//     raven::graph::cGraph g;
//     for (int n1 = 0; n1 < N; n1++)
//     {
//         for (int n2 = n1 + 1; n2 < N; n2++)
//         {
//             g.wEdgeAttr(
//                 g.add(n1, n2),
//                 {std::to_string(adj[n1][n2])});
//         }
//     }
//     return g;
// }

using namespace std;

// final_path[] stores the final solution ie, the
// path of the salesman.
std::vector<int> final_path;

std::vector<int> curr_path;

// visited[] keeps track of the already visited nodes
// in a particular path
std::vector<bool> visited;

// Stores the final minimum weight of shortest tour.
int final_res = INT_MAX;

// Function to find the minimum edge cost
// having an end at the vertex i
int firstMin(const raven::graph::cGraph &g, int i)
{
    int min = INT_MAX;
    for (int k : g.adjacentOut(i))
    {
        if (k != i)
        {
            int c = edgeWeight(g, i, k);
            if (c < min)
                min = c;
        }
    }
    return min;
}

// function to find the second minimum edge cost
// having an end at the vertex i
int secondMin(const raven::graph::cGraph &g, int i)
{
    int first = INT_MAX, second = INT_MAX;
    for (int j = 0; j < g.vertexCount(); j++)
    {
        if (i == j)
            continue;

        int c = edgeWeight(g, i, j);
        if (c <= first)
        {
            second = first;
            first = c;
        }
        else if (c <= second &&
                 c != first)
            second = c;
    }
    return second;
}

// recursive function that takes as arguments:
// curr_bound -> lower bound of the root node
// curr_weight-> stores the weight of the path so far
// level-> current level while moving in the search
//		 space tree
// curr_path[] -> where the solution is being stored which
//			 would later be copied to final_path[]
void TSPRec(const raven::graph::cGraph &g, int curr_bound, int curr_weight,
            int level)
{
    // base case is when we have reached level N which
    // means we have covered all the nodes once
    if (level == g.vertexCount())
    {
        // check if there is an edge from last vertex in
        // path back to the first vertex
        if (g.find(curr_path[level - 1], curr_path[0]) >= 0)
        {
            // curr_res has the total weight of the
            // solution we got
            int curr_res = curr_weight +
                           edgeWeight(g, curr_path[level - 1], curr_path[0]);

            // Update final result and final path if
            // current result is better.
            if (curr_res < final_res)
            {
                final_path = curr_path;
                final_path.push_back(curr_path[0]);
                final_res = curr_res;
            }
        }
        return;
    }

    // for any other level iterate for all vertices to
    // build the search space tree recursively
    for (int i = 0; i < g.vertexCount(); i++)
    {
        // Consider next vertex if it is not same (diagonal
        // entry in adjacency matrix and not visited
        // already)
        if (g.find(curr_path[level - 1], i) >= 0 &&
            visited[i] == false)
        {
            int temp = curr_bound;
            curr_weight += edgeWeight(g, curr_path[level - 1], i);

            // different computation of curr_bound for
            // level 2 from the other levels
            if (level == 1)
                curr_bound -= ((firstMin(g, curr_path[level - 1]) +
                                firstMin(g, i)) /
                               2);
            else
                curr_bound -= ((secondMin(g, curr_path[level - 1]) +
                                firstMin(g, i)) /
                               2);

            // curr_bound + curr_weight is the actual lower bound
            // for the node that we have arrived on
            // If current lower bound < final_res, we need to explore
            // the node further
            if (curr_bound + curr_weight < final_res)
            {
                curr_path[level] = i;
                visited[i] = true;

                // call TSPRec for the next level
                TSPRec(g, curr_bound, curr_weight, level + 1);
            }

            // Else we have to prune the node by resetting
            // all changes to curr_weight and curr_bound
            curr_weight -= edgeWeight(g, curr_path[level - 1], i);
            curr_bound = temp;

            // Also reset the visited array
            visited.clear();
            visited.resize(g.vertexCount(), false);

            for (int j = 0; j <= level - 1; j++)
                visited[curr_path[j]] = true;
        }
    }
}

// This function sets up final_path[]
void TSPBB(const raven::graph::cGraph &g)
{

    // Calculate initial lower bound for the root node
    // using the formula 1/2 * (sum of first min +
    // second min) for all edges.
    // Also initialize the curr_path and visited array
    int curr_bound = 0;

    curr_path.clear();
    curr_path.resize(g.vertexCount() + 1, -1);

    // Compute initial bound
    // auto g = makeGraph( adj );
    for (int i = 0; i < g.vertexCount(); i++)
        curr_bound += (firstMin(g, i) +
                       secondMin(g, i));

    // Rounding off the lower bound to an integer
    curr_bound = (curr_bound & 1) ? curr_bound / 2 + 1 : curr_bound / 2;

    // We start at vertex 1 so the first vertex
    // in curr_path[] is 0
    visited[0] = true;
    curr_path[0] = 0;

    // Call to TSPRec for curr_weight equal to
    // 0 and level 1
    TSPRec(g, curr_bound, 0, 1);
}

// Driver code
int main()
{

    raven::graph::cGraph g;
    // readfile( g, "../dat/data.txt" );
    // readfile( g, "../dat/notmetric.txt" );
    readfile(g, "../dat/so76213744.txt");

    visited.resize(g.vertexCount(), false);

    TSPBB(g);

    cout << "Minimum cost : " << final_res << "\nPath Taken : ";
    for (int n : final_path)
        cout << g.userName(n) << " ";
    cout << "\n";

    return 0;
}
