// C++ program to solve Traveling Salesman Problem
// using Branch and Bound.
#include <climits>
#include <cstring>
#include <iostream>
#include <fstream>
#include "GraphTheory.h" //  https://github.com/JamesBremner/PathFinder

class cTourist
{
public:
    void read(const std::string &fname);

    void Optimize();

    void display();

private:
    // final_path[] stores the final solution ie, the
    // path of the salesman.
    std::vector<int> final_path;

    std::vector<int> curr_path;

    // visited[] keeps track of the already visited nodes
    // in a particular path
    std::vector<bool> visited;

    // Stores the final minimum weight of shortest tour.
    int final_res;

    int budget;

    raven::graph::cGraph g;

    void TSPBB();
    void TSPRec(int curr_bound,
                int curr_weight,
                int level);

    int firstMin(int i);
    int secondMin(int i);

    void RemoveLeastInterest();

    int edgeWeight(int i, int j) const
    {
        return atoi(g.rEdgeAttr(g.find(i, j), 0).c_str());
    }
};

void cTourist::Optimize()
{
    TSPBB();
    while( final_res > budget ) {
        RemoveLeastInterest();
        TSPBB();
    }
}

void cTourist::RemoveLeastInterest()
{
    int min = INT_MAX;
    int vmin;
    for( int v = 0; v < g.vertexCount(); v++ )
    {
        int i = atoi(g.rVertexAttr(v,0).c_str());
        if( i < min ) {
            min = i;
            vmin = v;
    } }

    std::cout << "Dropping " << g.userName(vmin) 
        << " ( " << g.rVertexAttr(vmin,0) 
        << " ) from itinerary\n";

    g.remove(vmin);

}
void cTourist::read(const std::string &fname)
{

    g.clear();

    std::ifstream ifs(fname);
    if (!ifs.is_open())
        throw std::runtime_error(
            "Cannot open input file");

    std::string stype, sn1, sn2, scost, directed, same;
    ifs >> stype;

    while (ifs.good())
    {
        switch (stype[0])
        {
        case 'l':
            ifs >> sn1 >> sn2 >> scost;
            g.wEdgeAttr(g.add(sn1, sn2), {scost});
            g.wEdgeAttr(g.find(sn2, sn1), {scost});
            break;
        case 's':
            ifs >> sn1;
            g.startName(sn1);
            break;
        case 'c':
            ifs >> sn1 >> scost;
            g.wVertexAttr(g.find(sn1), {scost});
            break;
        case 'b':
            ifs >> scost;
            budget = atoi(scost.c_str());
            break;
        }

        ifs >> stype;
    }

    visited.resize(g.vertexCount(), false);

}

void cTourist::display()
{
    std::cout << "Minimum cost : " << final_res
              << " Budget : " << budget
              << "\nPath Taken : ";
    for (int n : final_path)
    {
        if( n < 0 )
            break;
        std::cout << g.userName(n) << " ";
    }
    std::cout << "\n";
}

// Function to find the minimum edge cost
// having an end at the vertex i
int cTourist::firstMin(int i)
{
    int min = INT_MAX;
    for (int k : g.adjacentOut(i))
    {
        if (k != i)
        {
            int c = edgeWeight(i, k);
            if (c < min)
                min = c;
        }
    }
    return min;
}

// function to find the second minimum edge cost
// having an end at the vertex i
int cTourist::secondMin(int i)
{
    int first = INT_MAX, second = INT_MAX;
    for (int j = 0; j < g.vertexCount(); j++)
    {
        if (i == j)
            continue;

        int c = edgeWeight(i, j);
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
void cTourist::TSPRec(int curr_bound, int curr_weight,
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
                           edgeWeight(curr_path[level - 1], curr_path[0]);

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
            curr_weight += edgeWeight(curr_path[level - 1], i);

            // different computation of curr_bound for
            // level 2 from the other levels
            if (level == 1)
                curr_bound -= ((firstMin(curr_path[level - 1]) +
                                firstMin(i)) /
                               2);
            else
                curr_bound -= ((secondMin(curr_path[level - 1]) +
                                firstMin(i)) /
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
                TSPRec(curr_bound, curr_weight, level + 1);
            }

            // Else we have to prune the node by resetting
            // all changes to curr_weight and curr_bound
            curr_weight -= edgeWeight(curr_path[level - 1], i);
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
void cTourist::TSPBB()
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
        curr_bound += (firstMin(i) +
                       secondMin(i));

    // Rounding off the lower bound to an integer
    curr_bound = (curr_bound & 1) ? curr_bound / 2 + 1 : curr_bound / 2;

    // We start at vertex 1 so the first vertex
    // in curr_path[] is 0
    visited[0] = true;
    curr_path[0] = 0;

    // Call to TSPRec for curr_weight equal to
    // 0 and level 1
    TSPRec(curr_bound, 0, 1);
}

// Driver code
int main()
{
    cTourist theTourist;

    theTourist.read("../dat/so76213744.txt");

    theTourist.Optimize();

    theTourist.display();

    return 0;
}
