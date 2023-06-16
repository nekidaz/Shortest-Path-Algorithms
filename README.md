# Shortest-Path-Algorithms

##Description
This program implements two graph shortest path algorithms: Dijkstra's algorithm and Bellman-Ford algorithm. It provides functionality to create a graph, add vertices and edges to the graph, and find the shortest path between two vertices using the implemented algorithms.

##Algorithms
###Dijkstra's Algorithm
Dijkstra's algorithm is a popular algorithm used to find the shortest path between two vertices in a graph. It works by iteratively selecting the vertex with the minimum distance from a source vertex and relaxing its neighboring vertices to update their distances. This process continues until the destination vertex is reached or all vertices have been visited.

###Bellman-Ford Algorithm
The Bellman-Ford algorithm is another algorithm used to find the shortest path in a graph. It can handle graphs with negative-weight edges, unlike Dijkstra's algorithm. The algorithm iterates through all edges multiple times, relaxing them to update the distances of vertices. After a certain number of iterations, it checks for negative cycles in the graph to determine if a shortest path exists.

##Usage
Create a new instance of the graph using the provided NewGraph() function.
Add vertices to the graph using the AddVertex(name string) method.
Add edges to the graph using the AddEdge(source, destination string, distance, ratio, velocity int) method, specifying the source and destination vertices, as well as the distance, ratio, and velocity of the edge.
Use the FindShortestPath(source, destination string) method to find the shortest path between two vertices using Dijkstra's algorithm.
Use the BellmanFordAlgorithm(source, destination string) method to find the shortest path between two vertices using the Bellman-Ford algorithm.

##Example

```
g := NewGraph()

g.AddVertex("A")
g.AddVertex("B")
g.AddVertex("C")
g.AddVertex("D")

g.AddEdge("A", "B", 5, 1, 10)
g.AddEdge("B", "C", 3, 1, 5)
g.AddEdge("C", "D", 2, 1, 8)
g.AddEdge("A", "D", 10, 1, 2)

g.FindShortestPath("A", "D")
g.BellmanFordAlgorithm("A", "D")

```

In this example, we create a graph with four vertices (A, B, C, D) and add edges between them. We then use the FindShortestPath method with Dijkstra's algorithm and the BellmanFordAlgorithm method to find the shortest path between vertex A and vertex D.
