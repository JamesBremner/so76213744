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

    // Stores the final minimum weight of shortest tour.
    int final_res;

    int budget;

    raven::graph::cGraph g;

    void TSPBB();

    void RemoveLeastInterest();
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

void cTourist::TSPBB()
{
    raven::graph::cTSP tsp( g );
    final_path = tsp.calculate();
    final_res = tsp.TotalPathEdgeWeight();
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
