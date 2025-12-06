#include <functional>
#include <iostream>
#include <limits>
#include <list>
#include <queue>

const int INF = std::numeric_limits<int>::max();

/**
 * @brief Maps [0-25] to [a-z]
 * 
 * @param num Index of alphabet
 * @return char Alphabet with given index or ?
 */
char int_to_alpha(const int num)
{
  if ((num >= 0) && (num < 26)) {
    return (char)('A' + num);
  }
  return '?';
}

/**
 * @brief Graph with vertices connected by weighted edges.
 * 
 */
class Graph
{
public:
  Graph(int V) : V_{V}, edges_(V){};

  /**
   * @brief Add an edge to connect two vertices
   * 
   * @param u Source vertex 
   * @param v Destination vertex
   * @param w Weight of edge
   */
  void add_edge(int u, int v, int w) { edges_.at(u).emplace_back(v, w); }

  /**
   * @brief Return the shortest path from a source vertex to a destination vertex.
   * 
   * @param src Index of the source vertex.
   * @param dest Index of the destination vertex.
   * @return std::vector<int> Lowest cost (shortest) path from source to destination.
   */
  std::vector<int> shortest_path(int src, int dest)
  {
    std::priority_queue<
      std::pair<int, int>, std::vector<std::pair<int, int>>, std::greater<std::pair<int, int>>>
      pq;
    std::vector<long long> dists(V_, INF);
    std::vector<int> parents(V_);
    std::vector<int> path;
    parents.at(src) = src;

    dists.at(src) = 0;

    pq.emplace(0, src);

    while (!pq.empty()) {
      int u = pq.top().second;

      /* Early exit */
      if (u == dest) {
        break;
      }

      /* Stale entry */
      if (dists.at(u) < pq.top().first) {
        continue;
      }

      /* Unconnected vertex */
      if (dists[u] == INF) {
        continue;
      }

      for (const auto & edge : edges_.at(u)) {
        int v = edge.first;
        int weight = edge.second;
        int new_dist = dists.at(u) + weight;
        if (new_dist < dists.at(v)) {
          parents.at(v) = u;
          dists.at(v) = new_dist;
          pq.emplace(dists.at(v), v);
        }
      }
      pq.pop();
    }
    
    auto parent = dest;
    while (parent != src) {
      path.push_back(parent);
      parent = parents.at(parent);
    }
    path.push_back(src);
    std::reverse(path.begin(), path.end());

    return path;
  }

private:
  std::vector<std::vector<std::pair<int, int>>> edges_;
  int V_;
};

/**
 * @brief 
 * 
 * @param argv 
 * @param argc 
 * @return int 
 */
int main(int argv, char ** argc)
{
  Graph g(6);

  g.add_edge(0, 1, 3);
  g.add_edge(0, 2, 1);
  g.add_edge(1, 0, 3);
  g.add_edge(1, 3, 1);
  g.add_edge(1, 4, 7);
  g.add_edge(2, 0, 1);
  g.add_edge(2, 5, 5);
  g.add_edge(2, 4, 10);
  g.add_edge(3, 1, 1);
  g.add_edge(4, 1, 7);
  g.add_edge(4, 2, 10);
  g.add_edge(4, 5, 5);
  g.add_edge(5, 2, 5);
  g.add_edge(5, 4, 5);

  std::vector<int> path = g.shortest_path(2, 3);

  std::cout << "Path is ";
  for (const auto &u : path) {
    std::cout << int_to_alpha(u) << " ";
  }
  std::cout << std::endl;
}
