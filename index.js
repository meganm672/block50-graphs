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

class Edge {
    constructor(v, weight) {
        this.v = v;
        this.weight = weight;
    }
}

function prim(graph) {
    const includedVertices = new Set();
    const startVertex = 0;
    includedVertices.add(startVertex);

    const minSpanningTree = [];
    let totalCost = 0;

    while (includedVertices.size < graph.length) {
        let minEdge = null;

        for (const vertex of includedVertices) {
            for (const edge of graph[vertex]) {
                if (!includedVertices.has(edge.v)) {
                    if (!minEdge || edge.weight < minEdge.weight) {
                        minEdge = edge;
                    }
                }
            }
        }
   // Check if a valid edge was found
   if (minEdge === null) {
    console.error("No valid edge found. The graph might not be connected.");
    break;
}
        includedVertices.add(minEdge.v);
        minSpanningTree.push(minEdge);
        totalCost += minEdge.weight;
    }

    return totalCost;
}

// Sample Input with Adjacency List
const adjacencyList = [
    [new Edge(1, 4), new Edge(7, 8)],
    [new Edge(0, 4), new Edge(2, 8), new Edge(7, 11)],
    [new Edge(1, 8), new Edge(3, 7), new Edge(8, 2), new Edge(5, 4)],
    [new Edge(2, 7), new Edge(4, 9), new Edge(5, 14)],
    [new Edge(3, 9), new Edge(5, 10)],
    [new Edge(2, 4), new Edge(3, 14), new Edge(6, 2)],
    [new Edge(5, 2), new Edge(7, 1), new Edge(8, 6)],
    [new Edge(0, 8), new Edge(1, 11), new Edge(6, 1), new Edge(8, 7)],
    [new Edge(2, 2), new Edge(6, 6), new Edge(7, 7)]
];

// Calculate the minimum cost using Prim's algorithm
const minCost = prim(adjacencyList);

// Output the result
console.log("Minimum cost to connect all rooms:", minCost);



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

function bfsShortestPath(graph, source, target) {
    // Initialize a queue with the source node and an empty path
    const queue = [{ node: source, path: [source] }];
    // Initialize a set to keep track of visited nodes
    const visited = new Set();

    // Perform BFS
    while (queue.length > 0) {
        // Dequeue a node and its path from the front of the queue
        const { node, path } = queue.shift();

        // If the node is the target, return the path
        if (node === target) {
            return path;
        }

        // If the node has been visited, skip it
        if (visited.has(node)) {
            continue;
        }

        // Mark the node as visited
        visited.add(node);

        // Enqueue neighboring nodes with their paths
        for (const neighbor of graph[node]) {
            if (!visited.has(neighbor)) {
                const newPath = [...path, neighbor];
                queue.push({ node: neighbor, path: newPath });
            }
        }
    }

    // If no path is found, return an empty array
    return [];
}

// Sample Input
const graph = {
    A: ['B', 'C'],
    B: ['A', 'D', 'E'],
    C: ['A', 'F'],
    D: ['B'],
    E: ['B', 'F'],
    F: ['C', 'E'],
};

const sourceNode = 'A';
const targetNode = 'F';

// Calculate the shortest path using BFS
const shortestPath = bfsShortestPath(graph, sourceNode, targetNode);

// Output the result
console.log(`Shortest path from ${sourceNode} to ${targetNode}:`, shortestPath);


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
function dfsAllRoutes(graph, source, target) {
    const routes = [];
    const visited = new Set();

    function dfsHelper(node, currentRoute) {
        // Add the current node to the current route
        currentRoute.push(node);
        visited.add(node);

        // If the current node is the target, add the current route to the routes array
        if (node === target) {
            routes.push([...currentRoute]);
        }

        // Explore neighbors
        for (const neighbor of graph[node]) {
            // If the neighbor is not visited, recursively call the DFS helper function
            if (!visited.has(neighbor)) {
                dfsHelper(neighbor, currentRoute);
            }
        }

        // Backtrack: Remove the last node from the current route and mark it as unvisited
        currentRoute.pop();
        visited.delete(node);
    }

    // Call the DFS helper function starting from the source node
    dfsHelper(source, []);

    // Return the array of all routes
    return routes;
}

// Sample Input
const DFSgraph = {
    A: ["B", "C"],
    B: ["A", "D", "E"],
    C: ["A", "F"],
    D: ["B"],
    E: ["B", "F"],
    F: ["C", "E"],
};

const sourceN = 'A';
const targetN = 'F';

// Calculate all possible routes using DFS
const allRoutes = dfsAllRoutes(DFSgraph, sourceN, targetN);

// Output the result
console.log(`All possible routes from ${sourceN} to ${targetN}:`, allRoutes);


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
// const startNode = "A"; 
// const endNode = "F";

// Sample Output: Shortest path: A -> B -> E -> F and Distance: 8
class PriorityQueue {
    constructor() {
        this.queue = [];
    }

    enqueue(element, priority) {
        this.queue.push({ element, priority });
        this.sort();
    }

    dequeue() {
        return this.queue.shift();
    }

    isNotEmpty() {
        return this.queue.length > 0;
    }

    sort() {
        this.queue.sort((a, b) => a.priority - b.priority);
    }
}

function dijkstra(graph, startNode, endNode) {
    // Initialize distances with Infinity for all nodes except the start node
    const distances = { [startNode]: 0 };
    // Initialize priority queue to keep track of the nodes with the shortest distances
    const priorityQueue = new PriorityQueue();
    priorityQueue.enqueue(startNode, 0);
    // Initialize previous nodes to reconstruct the shortest path
    const previous = {};
    // Initialize visited set to keep track of visited nodes
    const visited = new Set();

    // Dijkstra's algorithm
    while (priorityQueue.isNotEmpty()) {
        // Dequeue the node with the shortest distance
        const { element: currentNode, priority: currentDistance } = priorityQueue.dequeue();

        // Skip if the node has already been visited
        if (visited.has(currentNode)) {
            continue;
        }

        // Mark the node as visited
        visited.add(currentNode);

        // Explore neighbors
        for (const neighbor in graph[currentNode]) {
            // Calculate the potential distance from the start node to the neighbor
            const potentialDistance = currentDistance + graph[currentNode][neighbor];

            // Update the distance and enqueue the neighbor if it's shorter
            if (potentialDistance < (distances[neighbor] || Infinity)) {
                distances[neighbor] = potentialDistance;
                previous[neighbor] = currentNode;
                priorityQueue.enqueue(neighbor, potentialDistance);
            }
        }
    }

    // Reconstruct the shortest path
    const shortestPath = [];
    let currentNode = endNode;
    while (currentNode !== undefined) {
        shortestPath.unshift(currentNode);
        currentNode = previous[currentNode];
    }

    // Return the result
    return { path: shortestPath, distance: distances[endNode] };
}

// Sample Input
const Dgraph = {
    A: { B: 5, C: 2 },
    B: { D: 4, E: 2 },
    C: { B: 8, E: 7 },
    D: { E: 6, F: 3 },
    E: { F: 1 },
    F: {},
};

const startNode = "A";
const endNode = "F";

// Calculate the shortest path using Dijkstra's algorithm
const result = dijkstra(Dgraph, startNode, endNode);

// Output the result
console.log(`Shortest path: ${result.path.join(" -> ")} and Distance: ${result.distance}`);
