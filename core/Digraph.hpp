// Digraph.hpp
//
// ICS 46 Winter 2019
// Project #4: Rock and Roll Stops the Traffic
//
// This header file declares a class template called Digraph, which is
// intended to implement a generic directed graph. The implementation
// uses the adjacency lists technique, so each vertex stores a linked
// list of its outgoing edges.
//
// Along with the Digraph class template is a class DigraphException
// and a couple of utility structs that aren't generally useful outside
// of this header file.
//
// In general, directed graphs are all the same, except in the sense
// that they store different kinds of information about each vertex and
// about each edge; these two types are the type parameters to the
// Digraph class template.

#ifndef DIGRAPH_HPP
#define DIGRAPH_HPP

#include <exception>
#include <functional>
#include <list>
#include <map>
#include <utility>
#include <vector>


// DigraphExceptions are thrown from some of the member functions in the
// Digraph class template, so that exception is declared here, so it
// will be available to any code that includes this header file.

class DigraphException : public std::runtime_error
{
public:
    DigraphException(const std::string& reason);
};


inline DigraphException::DigraphException(const std::string& reason)
    : std::runtime_error{reason}
{
}



// A DigraphEdge lists a "from vertex" (the number of the vertex from which
// the edge points), a "to vertex" (the number of the vertex to which the
// edge points), and an EdgeInfo object.  Because different kinds of Digraphs
// store different kinds of edge information, DigraphEdge is a struct template.

template <typename EdgeInfo>
struct DigraphEdge
{
    int fromVertex;
    int toVertex;
    EdgeInfo einfo;
};



// A DigraphVertex includes two things: a VertexInfo object and a list of
// its outgoing edges.  Because different kinds of Digraphs store different
// kinds of vertex and edge information, DigraphVertex is a struct template.

template <typename VertexInfo, typename EdgeInfo>
struct DigraphVertex
{
    VertexInfo vinfo;
    std::list<DigraphEdge<EdgeInfo>> edges;
};



// Digraph is a class template that represents a directed graph implemented
// using adjacency lists.  It takes two type parameters:
//
// * VertexInfo, which specifies the kind of object stored for each vertex
// * EdgeInfo, which specifies the kind of object stored for each edge
//
// You'll need to implement the member functions declared here; each has a
// comment detailing how it is intended to work.
//
// Each vertex in a Digraph is identified uniquely by a "vertex number".
// Vertex numbers are not necessarily sequential and they are not necessarily
// zero- or one-based.

template <typename VertexInfo, typename EdgeInfo>
class Digraph
{
public:
    // The default constructor initializes a new, empty Digraph so that
    // contains no vertices and no edges.
    Digraph();

    // The copy constructor initializes a new Digraph to be a deep copy
    // of another one (i.e., any change to the copy will not affect the
    // original).
    Digraph(const Digraph& d);

    // The move constructor initializes a new Digraph from an expiring one.
    Digraph(Digraph&& d) noexcept;

    // The destructor deallocates any memory associated with the Digraph.
    ~Digraph() noexcept;

    // The assignment operator assigns the contents of the given Digraph
    // into "this" Digraph, with "this" Digraph becoming a separate, deep
    // copy of the contents of the given one (i.e., any change made to
    // "this" Digraph afterward will not affect the other).
    Digraph& operator=(const Digraph& d);

    // The move assignment operator assigns the contents of an expiring
    // Digraph into "this" Digraph.
    Digraph& operator=(Digraph&& d) noexcept;

    // vertices() returns a std::vector containing the vertex numbers of
    // every vertex in this Digraph.
    std::vector<int> vertices() const;

    // edges() returns a std::vector of std::pairs, in which each pair
    // contains the "from" and "to" vertex numbers of an edge in this
    // Digraph.  All edges are included in the std::vector.
    std::vector<std::pair<int, int>> edges() const;

    // This overload of edges() returns a std::vector of std::pairs, in
    // which each pair contains the "from" and "to" vertex numbers of an
    // edge in this Digraph.  Only edges outgoing from the given vertex
    // number are included in the std::vector.  If the given vertex does
    // not exist, a DigraphException is thrown instead.
    std::vector<std::pair<int, int>> edges(int vertex) const;

    // vertexInfo() returns the VertexInfo object belonging to the vertex
    // with the given vertex number.  If that vertex does not exist, a
    // DigraphException is thrown instead.
    VertexInfo vertexInfo(int vertex) const;

    // edgeInfo() returns the EdgeInfo object belonging to the edge
    // with the given "from" and "to" vertex numbers.  If either of those
    // vertices does not exist *or* if the edge does not exist, a
    // DigraphException is thrown instead.
    EdgeInfo edgeInfo(int fromVertex, int toVertex) const;

    // addVertex() adds a vertex to the Digraph with the given vertex
    // number and VertexInfo object.  If there is already a vertex in
    // the graph with the given vertex number, a DigraphException is
    // thrown instead.
    void addVertex(int vertex, const VertexInfo& vinfo);

    // addEdge() adds an edge to the Digraph pointing from the given
    // "from" vertex number to the given "to" vertex number, and
    // associates with the given EdgeInfo object with it.  If one
    // of the vertices does not exist *or* if the same edge is already
    // present in the graph, a DigraphException is thrown instead.
    void addEdge(int fromVertex, int toVertex, const EdgeInfo& einfo);

    // removeVertex() removes the vertex (and all of its incoming
    // and outgoing edges) with the given vertex number from the
    // Digraph.  If the vertex does not exist already, a DigraphException
    // is thrown instead.
    void removeVertex(int vertex);

    // removeEdge() removes the edge pointing from the given "from"
    // vertex number to the given "to" vertex number from the Digraph.
    // If either of these vertices does not exist *or* if the edge
    // is not already present in the graph, a DigraphException is
    // thrown instead.
    void removeEdge(int fromVertex, int toVertex);

    // vertexCount() returns the number of vertices in the graph.
    int vertexCount() const noexcept;

    // edgeCount() returns the total number of edges in the graph,
    // counting edges outgoing from all vertices.
    int edgeCount() const noexcept;

    // This overload of edgeCount() returns the number of edges in
    // the graph that are outgoing from the given vertex number.
    // If the given vertex does not exist, a DigraphException is
    // thrown instead.
    int edgeCount(int vertex) const;

    // isStronglyConnected() returns true if the Digraph is strongly
    // connected (i.e., every vertex is reachable from every other),
    // false otherwise.
    bool isStronglyConnected() const;

    // findShortestPaths() takes a start vertex number and a function
    // that takes an EdgeInfo object and determines an edge weight.
    // It uses Dijkstra's Shortest Path Algorithm to determine the
    // shortest paths from the start vertex to every other vertex
    // in the graph.  The result is returned as a std::map<int, int>
    // where the keys are vertex numbers and the value associated
    // with each key k is the precedessor of that vertex chosen by
    // the algorithm.  For any vertex without a predecessor (e.g.,
    // a vertex that was never reached, or the start vertex itself),
    // the value is simply a copy of the key.
    std::map<int, int> findShortestPaths(
        int startVertex,
        std::function<double(const EdgeInfo&)> edgeWeightFunc) const;


private:
    // Add whatever member variables you think you need here.  One
    // possibility is a std::map where the keys are vertex numbers
    // and the values are DigraphVertex<VertexInfo, EdgeInfo> objects.
    std::map<int, DigraphVertex<VertexInfo, EdgeInfo>> m;
    void isStronglyConnected_r(int vertex, std::map<int,bool> &visited) const;
    // You can also feel free to add any additional member functions
    // you'd like (public or private), so long as you don't remove or
    // change the signatures of the ones that already exist.
};



// You'll need to implement the member functions below.  There's enough
// code in place to make them compile, but they'll all need to do the
// correct thing instead.

// The default constructor initializes a new, empty Digraph so that
// contains no vertices and no edges.
template <typename VertexInfo, typename EdgeInfo>
Digraph<VertexInfo, EdgeInfo>::Digraph()
{
    m = {};
}

// The copy constructor initializes a new Digraph to be a deep copy
// of another one (i.e., any change to the copy will not affect the
// original).
template <typename VertexInfo, typename EdgeInfo>
Digraph<VertexInfo, EdgeInfo>::Digraph(const Digraph& d)
{
    std::map<int, DigraphVertex<VertexInfo, EdgeInfo>> m_new;
    for(auto i : d.m) {
        m_new[i.first] = i.second;
    }
    m = m_new;
}


// The move constructor initializes a new Digraph from an expiring one.
template <typename VertexInfo, typename EdgeInfo>
Digraph<VertexInfo, EdgeInfo>::Digraph(Digraph&& d) noexcept
{
    std::map<int, DigraphVertex<VertexInfo, EdgeInfo>> m_new;
    for(auto i : d.m) {
        m_new[i.first] = i.second;
    }
    std::swap(m, m_new);
}

// The destructor deallocates any memory associated with the Digraph.
template <typename VertexInfo, typename EdgeInfo>
Digraph<VertexInfo, EdgeInfo>::~Digraph() noexcept
{
    m.clear();
}

// The assignment operator assigns the contents of the given Digraph
// into "this" Digraph, with "this" Digraph becoming a separate, deep
// copy of the contents of the given one (i.e., any change made to
// "this" Digraph afterward will not affect the other).
template <typename VertexInfo, typename EdgeInfo>
Digraph<VertexInfo, EdgeInfo>& Digraph<VertexInfo, EdgeInfo>::operator=(const Digraph& d)
{
    if(this != &d) {
        m.clear();
        Digraph(d);
    }
    return *this;
}

// The move assignment operator assigns the contents of an expiring
// Digraph into "this" Digraph.
template <typename VertexInfo, typename EdgeInfo>
Digraph<VertexInfo, EdgeInfo>& Digraph<VertexInfo, EdgeInfo>::operator=(Digraph&& d) noexcept
{
    if(this != &d) {
        m.clear();
        std::swap(m, d.m);
    }
    return *this;
}

// vertices() returns a std::vector containing the vertex numbers of
// every vertex in this Digraph.
template <typename VertexInfo, typename EdgeInfo>
std::vector<int> Digraph<VertexInfo, EdgeInfo>::vertices() const
{
    std::vector<int> v_v;
    for(auto i : m) {
        v_v.push_back(i.first);
    }
    return v_v;
}

// edges() returns a std::vector of std::pairs, in which each pair
// contains the "from" and "to" vertex numbers of an edge in this
// Digraph.  All edges are included in the std::vector.
template <typename VertexInfo, typename EdgeInfo>
std::vector<std::pair<int, int>> Digraph<VertexInfo, EdgeInfo>::edges() const
{
    std::vector<std::pair<int, int>> edges_vector;
    for(auto i : m) {
        for(auto j : i.second.edges) {
            edges_vector.push_back(std::pair<int,int>(j.fromVertex, j.toVertex));
        }
    }
    return edges_vector;
}

// This overload of edges() returns a std::vector of std::pairs, in
// which each pair contains the "from" and "to" vertex numbers of an
// edge in this Digraph.  Only edges outgoing from the given vertex
// number are included in the std::vector.  If the given vertex does
// not exist, a DigraphException is thrown instead.
template <typename VertexInfo, typename EdgeInfo>
std::vector<std::pair<int, int>> Digraph<VertexInfo, EdgeInfo>::edges(int vertex) const
{
    std::vector<int> v = vertices();
    if(std::find(v.begin(), v.end(), vertex) == v.end()) {
        throw DigraphException("Vertex not in Digraph");
    }
    else {
        std::vector<std::pair<int, int>> edges_vector;
        for(auto i : m) {
            if(i.first == vertex) {
                for(auto j : i.second.edges) {
                    edges_vector.push_back(std::pair<int,int>(j.fromVertex, j.toVertex));
                }
            }
        }
        return edges_vector;
    }
}

// vertexInfo() returns the VertexInfo object belonging to the vertex
// with the given vertex number.  If that vertex does not exist, a
// DigraphException is thrown instead.
template <typename VertexInfo, typename EdgeInfo>
VertexInfo Digraph<VertexInfo, EdgeInfo>::vertexInfo(int vertex) const
{
    std::vector<int> v = vertices();
    if(std::find(v.begin(), v.end(), vertex) != v.end()) {
        VertexInfo vi = (m.find(vertex)->second).vinfo;
        return vi;
    }
    else {
        throw DigraphException("Vertex not in Digraph");
    }
}

// edgeInfo() returns the EdgeInfo object belonging to the edge
// with the given "from" and "to" vertex numbers.  If either of those
// vertices does not exist *or* if the edge does not exist, a
// DigraphException is thrown instead.
template <typename VertexInfo, typename EdgeInfo>
EdgeInfo Digraph<VertexInfo, EdgeInfo>::edgeInfo(int fromVertex, int toVertex) const
{
    std::vector<int> v = vertices();
    if(std::find(v.begin(), v.end(), fromVertex) == v.end()) {
        throw DigraphException("fromVertex not in Digraph");
    }
    else if(std::find(v.begin(), v.end(), toVertex) == v.end()) {
        throw DigraphException("toVertex not in Digraph");
    }
    else {
        EdgeInfo ei;
        bool found = false;
        for(auto i : m) {
            for(auto j : i.second.edges) {
                if(j.fromVertex == fromVertex && j.toVertex == toVertex) {
                    found = true;
                    ei = j.einfo;
                }
            }
        }
        if(found) {
            return ei;
        }
        else {
            throw DigraphException("Edge does not exist");
        }
    }
}

// addVertex() adds a vertex to the Digraph with the given vertex
// number and VertexInfo object.  If there is already a vertex in
// the graph with the given vertex number, a DigraphException is
// thrown instead.
template <typename VertexInfo, typename EdgeInfo>
void Digraph<VertexInfo, EdgeInfo>::addVertex(int vertex, const VertexInfo& vinfo)
{
    std::vector<int> v = vertices();
    if(std::find(v.begin(), v.end(), vertex) != v.end()) {
        throw DigraphException("Vertex is already in Digraph");
    }
    else {
        DigraphVertex<VertexInfo, EdgeInfo> dv;
        dv.vinfo = vinfo;
        dv.edges = std::list<DigraphEdge<EdgeInfo>>{};
        m.insert(std::pair<int,DigraphVertex<VertexInfo, EdgeInfo>>(vertex,dv));
    }
}

// addEdge() adds an edge to the Digraph pointing from the given
// "from" vertex number to the given "to" vertex number, and
// associates with the given EdgeInfo object with it.  If one
// of the vertices does not exist *or* if the same edge is already
// present in the graph, a DigraphException is thrown instead. 
template <typename VertexInfo, typename EdgeInfo>
void Digraph<VertexInfo, EdgeInfo>::addEdge(int fromVertex, int toVertex, const EdgeInfo& einfo)
{   
    std::vector<int> v = vertices();
    if(std::find(v.begin(), v.end(), fromVertex) == v.end()) {
        throw DigraphException("fromVertex is not in Digraph");
    }
    else if(std::find(v.begin(), v.end(), toVertex) == v.end()) {
        throw DigraphException("toVertex is not in Digraph");
    }
    else {
        for(auto i : m) {
            if(i.first == fromVertex) {
                for(auto j : i.second.edges) {
                    if(j.fromVertex == fromVertex && j.toVertex == toVertex) {
                        throw DigraphException("Edge already exists");
                    }
                }
                DigraphEdge<EdgeInfo> e;
                e.fromVertex = fromVertex;
                e.toVertex = toVertex;
                e.einfo = einfo;
                m[fromVertex].edges.push_back(e);
                break;
            }
        }
    }
    
}

// removeVertex() removes the vertex (and all of its incoming
// and outgoing edges) with the given vertex number from the
// Digraph.  If the vertex does not exist already, a DigraphException
// is thrown instead.
template <typename VertexInfo, typename EdgeInfo>
void Digraph<VertexInfo, EdgeInfo>::removeVertex(int vertex)
{
    std::vector<int> v = vertices();
    if(std::find(v.begin(), v.end(), vertex) == v.end()) {
        throw DigraphException("Vertex not in Digraph");
    }
    else {
        m.erase(vertex);
        for(auto i : m) {
            for(auto j : i.second.edges) {
                if(j.toVertex == vertex) {
                    removeEdge(j.fromVertex, vertex);
                }
            }
        }
    }
}

// removeEdge() removes the edge pointing from the given "from"
// vertex number to the given "to" vertex number from the Digraph.
// If either of these vertices does not exist *or* if the edge
// is not already present in the graph, a DigraphException is
// thrown instead.
template <typename VertexInfo, typename EdgeInfo>
void Digraph<VertexInfo, EdgeInfo>::removeEdge(int fromVertex, int toVertex)
{
    std::vector<int> v = vertices();
    if(std::find(v.begin(), v.end(), fromVertex) == v.end()) {
        throw DigraphException("fromVertex is not in Digraph");
    }
    else if(std::find(v.begin(), v.end(), toVertex) == v.end()) {
        throw DigraphException("toVertex is not in Digraph");
    }
    else {
        bool ffound = false;
        typename std::list<DigraphEdge<EdgeInfo>>::iterator iter = m[fromVertex].edges.begin();
        while(iter != m[fromVertex].edges.end()) {
            if(iter->toVertex == toVertex) {
                m[fromVertex].edges.erase(iter);
                ffound = true;
                break;
            }
            iter++;
        }
        if(ffound == false) {
            throw DigraphException("Edge not in graph");
        }
    }
}

// vertexCount() returns the number of vertices in the graph.
template <typename VertexInfo, typename EdgeInfo>
int Digraph<VertexInfo, EdgeInfo>::vertexCount() const noexcept
{
    std::vector<int> v = vertices();
    return v.size();
}

// edgeCount() returns the total number of edges in the graph,
// counting edges outgoing from all vertices.
template <typename VertexInfo, typename EdgeInfo>
int Digraph<VertexInfo, EdgeInfo>::edgeCount() const noexcept
{
    std::vector<std::pair<int, int>> edges_vector = edges();
    return edges_vector.size();
}

// This overload of edgeCount() returns the number of edges in
// the graph that are outgoing from the given vertex number.
// If the given vertex does not exist, a DigraphException is
// thrown instead.
template <typename VertexInfo, typename EdgeInfo>
int Digraph<VertexInfo, EdgeInfo>::edgeCount(int vertex) const
{
    std::vector<std::pair<int, int>> edges_vector = edges(vertex);
    return edges_vector.size();
}

// isStronglyConnected() returns true if the Digraph is strongly
// connected (i.e., every vertex is reachable from every other),
// false otherwise.
template <typename VertexInfo, typename EdgeInfo>
bool Digraph<VertexInfo, EdgeInfo>::isStronglyConnected() const
{
    std::map<int,bool> visited;
    for(auto i : m) {
        visited[i.first] = false;
    }
    for(auto i : m) {
        isStronglyConnected_r(i.first, visited);
        for(auto j : visited) {
            if(j.second == false){
                return false;
            }
        }
        for(auto j : visited) {
            j.second = false;
        }
    }   
    return true;
}
template <typename VertexInfo, typename EdgeInfo>
void Digraph<VertexInfo, EdgeInfo>::isStronglyConnected_r(int vertex, std::map<int,bool> &visited) const
{   
    visited[vertex] = true;
    for(auto i : m) {
        if(i.first == vertex) {
            for(auto j : i.second.edges) {
                if(visited[j.toVertex] == false) {
                    isStronglyConnected_r(j.toVertex, visited);
                }
            }
        }
    }
}

// findShortestPaths() takes a start vertex number and a function
// that takes an EdgeInfo object and determines an edge weight.
// It uses Dijkstra's Shortest Path Algorithm to determine the
// shortest paths from the start vertex to every other vertex
// in the graph.  The result is returned as a std::map<int, int>
// where the keys are vertex numbers and the value associated
// with each key k is the precedessor of that vertex chosen by
// the algorithm.  For any vertex without a predecessor (e.g.,
// a vertex that was never reached, or the start vertex itself),
// the value is simply a copy of the key.
template <typename VertexInfo, typename EdgeInfo>
std::map<int, int> Digraph<VertexInfo, EdgeInfo>::findShortestPaths(
    int startVertex,
    std::function<double(const EdgeInfo&)> edgeWeightFunc) const
{
    std::map<int,bool> k_flag;
    std::map<int,double> d_length;
    std::map<int,int> predecessor;
    for(auto i : m) {
        k_flag[i.first] = false;
        d_length[i.first] = 9999999;
        predecessor[i.first] = -1;
    }
    d_length[startVertex] = 0;
    predecessor[startVertex] = startVertex;
    //
    std::map<int,int> pq;
    pq.insert(std::pair(startVertex,0));
    while(!pq.empty()) {
        int v;
        int vert;
        bool first = true;
        for(auto it = pq.begin(); it != pq.end(); it++) {
            if(first == true) {
                v = it->first;
                vert = it->second;
                first = false;
            }
            if(it->second < vert) {
                v = it->first;
                vert = it->second;
            }
        }
        if(k_flag[v] == false) {
            k_flag[v] = true;
            for(auto j : m) {
                if(v == j.first) {
                    for(auto i : j.second.edges) {
                        if(d_length[i.toVertex] > d_length[v] + edgeWeightFunc(i.einfo)) {
                            d_length[i.toVertex] = d_length[v] + edgeWeightFunc(i.einfo);
                            predecessor[i.toVertex] = v;
                            pq.insert(std::pair(i.toVertex, d_length[i.toVertex]));
                        }
                    }
                }
            }
            for(auto it = pq.begin(); it != pq.end(); it++) {
                if(it->first == v) {
                    pq.erase(it);
                    break;
                }
            }
        }
    }
    for(auto i : predecessor) {
        if(i.second == -1) {
            predecessor[i.first] = i.first;
        }
    }
    return predecessor;
}



#endif // DIGRAPH_HPP

