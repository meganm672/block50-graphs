// Un directional and un weight graph
class Graph {
    constructor() {
      this.vertices = new Set(); // set of unique vertices
      this.edges = {}; // adjacency list - map of vertices to their neighbors
    }
  
    //function add vertex
    addVertex(v) {
      this.vertices.add(v);
      this.edges[v] = [];
    }
  
    // function add edge
    addEdge(v1, v2) {
      this.edges[v1].push(v2);
      this.edges[v2].push(v1);
    }
  
    // prints out the structutre of the graph
    getGraphStructure() {
      let graphStructure = "";
      for (const vertex of this.vertices) {
        const connections = this.edges[vertex].join(" => ");
        graphStructure += `${vertex} => ${connections}\n`;
      }
      console.log(graphStructure);
    }
}  

// Question 1
// You are a network designer entrusted with the responsibility of designing a computer network for a small office. 
// The office consists of multiple rooms, and your goal is to connect them using the least amount of cable, ensuring that each room is connected to the network. 
// You need to analyze the office layout, identify the rooms, and plan the most efficient way to connect them with cables. 
// The objective is to minimize the required cable length while ensuring every room is connected to the network.

// Your task is to apply Prim's graph-based algorithm, which starts with an initial room and progressively adds neighboring rooms with the shortest cable connections. 
// By iteratively expanding the network, you will create a minimum-cost spanning tree that connects all the rooms in the office. 
// Take on the role of the network designer, employ Prim's algorithm, and determine the minimum cost of connecting all the rooms in the office using the provided scenario.

// Sample Input:- new Edge(0, 1, 4),   new Edge(0, 7, 8),   new Edge(1, 2, 8),   new Edge(1, 7, 11),   new Edge(2, 3, 7),  
//  new Edge(2, 8, 2),   new Edge(2, 5, 4),   new Edge(3, 4, 9),   new Edge(3, 5, 14),   new Edge(4, 5, 10),   new Edge(5, 6, 2),   
// new Edge(6, 7, 1),   new Edge(6, 8, 6),   new Edge(7, 8, 7) in the format of (edge pairs, weights) with a total number of 9 vertices.

// Sample Output: Minimum cost to connect all rooms: 37

//not sure how to do any of these problems .... they are all incorrect couldnt find suggestions or answers online

// Question 2
// You are an aspiring computer scientist tasked with creating a function that can find the shortest path between two locations in a graph. 
// The graph represents various locations and the roads connecting them, with each road having a specific distance associated with it. 
// Your goal is to create a function called bfsShortestPath (graph, source, target) that takes in the graph, the source node 
// (representing the traveler's current location), and the target node (representing the traveler's destination). 
// The function should return an array representing the shortest path from the source to the target.

// The graph is represented using an adjacency list. 
// This means that each location in the graph is a node, and the roads connecting them are represented as edges. 
// The adjacency list stores the neighboring nodes for each node, allowing you to traverse the graph efficiently. 
// Your task is to create a bfsShortestPath function, utilizing the Breadth-First Search (BFS) algorithm to find the shortest path from the source to the target.
//  The function should return an array that represents the shortest path, starting from the source and ending at the target.

// Sample Input: A: ['B', 'C'],   B: ['A', 'D', 'E'],   C: ['A', 'F'],   D: ['B'],   E: ['B', 'F'],   F: ['C', 'E'], 
// in the format of Vertices: (neighboring nodes) and source node will be A and Destination node will be F

// Sample Output: Shortest path from A to F: [ 'A', 'C', 'F' ]

function bfsShortestPath(graph,source,target){
    const visited = new Set();
    const queue = [source];
    visited.add(source);

    while(queue.length >0){
        const vertex = queue.shift();

        for (let neighbor of graph[source]){
            if(!visited.has(neighbor)){
                queue.push(neighbor);
                visited.add(neighbor)
            }
        }
    }
}

const graph={
    A: ['B', 'C'],   
    B: ['A', 'D', 'E'],   
    C: ['A', 'F'],   
    D: ['B'],   
    E: ['B', 'F'],   
    F: ['C', 'E'],
}

// console.log(bfsShortestPath(graph, graph.A, graph.F))


// Question 3
// You are a cab driver in Boston, and you receive a request to pick up a passenger from a specific location. 
// Your task is to find all possible routes to reach the passenger's location using the Depth First Search (DFS) algorithm in JavaScript. 
// You need to implement the Depth First Search algorithm to find all possible routes from your current location (the starting node) 
// to the passenger's location (the target node). 
// Your goal is to provide a list of all possible routes. Implement the dfsAllRoutes(graph, source, target) 
// function in JavaScript that takes the graph, the source node (your current location), and the target node (the passenger's location) as input. 
// The function should return an array of all possible routes from the source to the target.

// Sample Input:  A: ["B", "C"],   B: ["A", "D", "E"],   C: ["A", "F"],   D: ["B"],   E: ["B", "F"],   F: ["C", "E"],  
// in the format of Vertices: (neighboring nodes) and source node will be A and Destination node will be F.

// Sample Output: All possible routes from A to F: [ [ 'A', 'B', 'E', 'F' ], [ 'A', 'C', 'F' ] ]

function dfsAllRoutes(graph,source,target){
    const visited = new Set();
    visited.add(source);

    for(let neighbor of graph[source]){
        if(!visited.has(neighbor)){
            dfs(graph,neighbor)
        }
    }
}

// console.log(dfsAllRoutes(graph, graph.A,graph.F))
// 

// Question 4
// Imagine you are developing a navigation system for a delivery robot that needs to navigate through a city to deliver packages efficiently. 
// The city is represented as a graph, where each point is a location, and the edges between points represent the routes that the robot can take. 
// Each edge has a weight associated with it, representing the distance or time required to travel from one point to another. 
// The goal is to use Dijkstra's algorithm in JavaScript to calculate the shortest path for the robot, optimizing package delivery.

// In this scenario, the graph representing the city is as follows:

// Point A connects to Point B with a weight of 5.

// Point A connects to Point C with a weight of 2.

// Point B connects to Point D with a weight of 4.

// Point B connects to Point E with a weight of 2.

// Point C connects to Point B with a weight of 8.

// Point C connects to Point E with a weight of 7.

// Point D connects to Point E with a weight of 6.

// Point D connects to Point F with a weight of 3.

// Point E connects to Point F with a weight of 1.

 

// Sample Input:  A: { B: 5, C: 2 },   B: { D: 4, E: 2 },   C: { B: 8, E: 7 },   D: { E: 6, F: 3 },   E: { F: 1 },   F: {}, 
const startNode = "A"; 
const endNode = "F";

// Sample Output: Shortest path: A -> B -> E -> F and Distance: 8

// Priority Queue
class PriorityQueue {
    constructor() {
      this.values = [];
    }
  
    enqueue(vertex, priority) {
      this.values.push({ vertex, priority }); // add vertex and priority to values array
      this.sort(); // sort the values array
    }
  
    dequeue() {
      return this.values.shift();
    }
  
    sort() {
      this.values.sort((a, b) => a.priority - b.priority); // sort the values array by priority
    }
  }
    // Dijkstra's algorithm
//    function dijkstra(start, end) {
//       // start and end are the vertices we want the path to start and end at.
//       const distances = {}; // Keep track of the distances from the start vertex to each vertex
//       const previous = {}; // Keep track of the previous vertex in the shortest path to each vertex
//       const priorityQueue = new PriorityQueue(); // Priority queue to keep track of vertices with the shortest distance from the start vertex
//       const visited = new Set(); // Set to keep track of visited vertices
  
//       // initialize distances and previous
//       for (const vertex in this.adjacencyList) {
//         // loop through each vertex in the graph
//         if (vertex === start) {
//           // if the vertex is the starting vertex
//           distances[vertex] = 0; // set the distance to 0
//           priorityQueue.enqueue(vertex, 0); // add the vertex to the priority queue with a distance of 0
//         } else {
//           distances[vertex] = Infinity; // set the distance to infinity
//           priorityQueue.enqueue(vertex, Infinity); // add the vertex to the priority queue with a distance of infinity
//         }
//         previous[vertex] = null; // set the previous vertex to null
//       }
  
//       while (priorityQueue.values.length) {
//         // while the priority queue is not empty
//         const { vertex } = priorityQueue.dequeue(); // dequeue the vertex with the shortest distance from the start vertex
//         visited.add(vertex); // add the vertex to the visited set
  
//         if (vertex == end) {
//           // if the vertex is the end vertex
//           //create path from end to start
//           const path = [];
//           let currentVertex = end;
//           while (currentVertex) {
//             // loop through the path from end to start
//             path.unshift(currentVertex); // add the current vertex to the path
//             currentVertex = previous[currentVertex]; // set the current vertex to the previous vertex in the path
//           }
//           return { distance: distances[end], path }; // return the distance and path
//         }
  
//         if (distances[vertex] !== Infinity) {
//           // if the distance is not infinity
//           for (let neighbor of this.adjacencyList[vertex]) {
//             // loop through each neighbor of the vertex
//             if (!visited.has(neighbor.city)) {
//               // if the neighbor has not been visited
//               const distance = distances[vertex] + neighbor.weight; // calculate the distance to the neighbor
//               if (distance < distances[neighbor.city]) {
//                 // if the distance to the neighbor is less than the current distance
//                 distances[neighbor.city] = distance; // update the distance to the neighbor
//                 previous[neighbor.city] = vertex; // update the previous vertex city
//                 priorityQueue.enqueue(neighbor.city, distance); // add the neighbor to the priority queue
//               }
//             }
//           }
//         }
//       }
//       return { distance: -1, path: [] }; // return an empty object if no path is found
//     }
  
//   console.log(dijkstra(startNode,endNode))