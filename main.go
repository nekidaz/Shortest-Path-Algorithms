package main

import (
	"container/heap"
	"fmt"
)

// Vertex represents a location in the graph
type Vertex struct {
	name     string
	distance int
	previous *Vertex
	index    int
}

// Edge represents a connection between two vertices
type Edge struct {
	destination *Vertex
	distance    int
	ratio       int
	velocity    int
}

// Graph is the main struct of the graph
type Graph struct {
	vertices map[string]*Vertex
	edges    map[*Vertex][]*Edge
}

// NewGraph creates a new graph
func NewGraph() *Graph {
	return &Graph{
		vertices: make(map[string]*Vertex),
		edges:    make(map[*Vertex][]*Edge),
	}
}

// AddVertex adds a new vertex to the graph
func (g *Graph) AddVertex(name string) {
	g.vertices[name] = &Vertex{
		name:     name,
		distance: -1,
	}
}

// AddEdge adds a new edge to the graph
func (g *Graph) AddEdge(source, destination string, distance, ratio, velocity int) {
	src, ok := g.vertices[source]
	if !ok {
		src = &Vertex{
			name: source,
		}
		g.vertices[source] = src
	}

	dest, ok := g.vertices[destination]
	if !ok {
		dest = &Vertex{
			name: destination,
		}
		g.vertices[destination] = dest
	}

	edge := &Edge{
		destination: dest,
		distance:    distance,
		ratio:       ratio,
		velocity:    velocity,
	}

	g.edges[src] = append(g.edges[src], edge)
}

// VertexHeap is a priority queue of vertices
type VertexHeap []*Vertex

func (h VertexHeap) Len() int {
	return len(h)
}

func (h VertexHeap) Less(i, j int) bool {
	return h[i].distance < h[j].distance
}

func (h VertexHeap) Swap(i, j int) {
	h[i], h[j] = h[j], h[i]
	h[i].index = i
	h[j].index = j
}

func (h *VertexHeap) Push(x interface{}) {
	vertex := x.(*Vertex)
	vertex.index = len(*h)
	*h = append(*h, vertex)
}

func (h *VertexHeap) Pop() interface{} {
	old := *h
	n := len(old)
	vertex := old[n-1]
	vertex.index = -1
	*h = old[0 : n-1]
	return vertex
}

// FindShortestPath finds the shortest path between two vertices
func (g *Graph) FindShortestPath(source, destination string) {
	src, ok := g.vertices[source]
	if !ok {
		fmt.Println("Error: invalid source vertex")
		return
	}

	dest, ok := g.vertices[destination]
	if !ok {
		fmt.Println("Error: invalid destination vertex")
		return
	}

	// Initialize all distances to infinity
	for _, vertex := range g.vertices {
		vertex.distance = -1
		vertex.previous = nil
	}

	// Create a priority queue of vertices
	queue := &VertexHeap{src}
	heap.Init(queue)

	// Set the distance of the source vertex to 0
	src.distance = 0

	for queue.Len() > 0 {
		// Extract the vertex with the smallest distance
		current := heap.Pop(queue).(*Vertex)
		fmt.Println("Visiting vertex", current.name)

		// Stop if we have reached the destination vertex
		if current == dest {
			break
		}

		// Relax the edges going out of the vertex
		for _, edge := range g.edges[current] {
			neighbor := edge.destination
			newDistance := current.distance + edge.distance
			if newDistance < neighbor.distance || neighbor.distance == -1 {
				neighbor.distance = newDistance
				neighbor.previous = current
				heap.Push(queue, neighbor)
				fmt.Println("From vertex ", current.name, " to ", neighbor.name, " distance: ", edge.distance, " ratio: ", edge.ratio)
			}
		}
	}

	// Print the shortest path
	if dest.previous == nil {
		fmt.Println("There is no path from", source, "to", destination)
	} else {
		fmt.Print("The shortest path from", source, "to", destination, "is: ")
		current := dest
		for current != nil {
			fmt.Print(current.name, "-->")
			current = current.previous
		}
	}
}

func main() {
	g := NewGraph()

	g.AddVertex("ENU")
	g.AddVertex("SDU")

	g.AddEdge("ENU", "Airport", 18, 1, 60)
	g.AddEdge("ENU", "Mall A", 8, 1, 6)
	g.AddEdge("Mall A", "Airport", 9, 2, 70)
	g.AddEdge("Airport", "SDU", 25, 1, 30)

	g.FindShortestPath("ENU", "SDU")
	fmt.Println("hwllo ")
}
