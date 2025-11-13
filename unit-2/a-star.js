/**
 * A* Algorithm - Educational Implementation
 * 
 * A* is a pathfinding algorithm that finds the shortest path between nodes.
 * It combines the benefits of Dijkstra's algorithm and greedy best-first search.
 * 
 * Key Concepts:
 * - g(n): Cost from start node to current node
 * - h(n): Heuristic estimate from current node to goal (never overestimates)
 * - f(n): g(n) + h(n) = total estimated cost of path through this node
 * 
 * The algorithm explores nodes with the lowest f(n) value first.
 */

// Node class represents each position in the grid
class Node {
  constructor(x, y) {
    this.x = x;
    this.y = y;
    this.g = 0;        // Cost from start to this node
    this.h = 0;        // Heuristic estimate to goal
    this.f = 0;        // Total cost (g + h)
    this.parent = null; // Track path back to start
  }
}

// Main A* Algorithm
class AStar {
  constructor(grid) {
    this.grid = grid;
    this.rows = grid.length;
    this.cols = grid[0].length;
  }

  /**
   * Heuristic function: Manhattan Distance
   * Estimates the distance from current node to goal
   * For grids, Manhattan distance = |x1-x2| + |y1-y2|
   */
  heuristic(nodeA, nodeB) {
    return Math.abs(nodeA.x - nodeB.x) + Math.abs(nodeA.y - nodeB.y);
  }

  /**
   * Get neighbors (up, down, left, right)
   * Check if neighbor is within bounds and is walkable (not obstacle)
   */
  getNeighbors(node) {
    const neighbors = [];
    const directions = [
      { x: -1, y: 0 }, // up
      { x: 1, y: 0 },  // down
      { x: 0, y: -1 }, // left
      { x: 0, y: 1 }   // right
    ];

    for (const dir of directions) {
      const newX = node.x + dir.x;
      const newY = node.y + dir.y;

      // Check if within bounds
      if (newX >= 0 && newX < this.rows && newY >= 0 && newY < this.cols) {
        // Check if walkable (0 = walkable, 1 = obstacle)
        if (this.grid[newX][newY] === 0) {
          neighbors.push(new Node(newX, newY));
        }
      }
    }
    return neighbors;
  }

  /**
   * Check if node exists in list (compare by coordinates)
   */
  nodeExists(list, node) {
    return list.some(n => n.x === node.x && n.y === node.y);
  }

  /**
   * Find node in list by coordinates
   */
  findNode(list, node) {
    return list.find(n => n.x === node.x && n.y === node.y);
  }

  /**
   * Main A* search algorithm
   * @param {Node} start - Starting position
   * @param {Node} goal - Goal position
   * @returns {Array} Path from start to goal
   */
  search(start, goal) {
    const openSet = [start];    // Nodes to be evaluated
    const closedSet = [];        // Nodes already evaluated
    
    start.g = 0;
    start.h = this.heuristic(start, goal);
    start.f = start.h;

    while (openSet.length > 0) {
      // Find node in openSet with lowest f value
      let current = openSet[0];
      let currentIndex = 0;
      
      for (let i = 1; i < openSet.length; i++) {
        if (openSet[i].f < current.f) {
          current = openSet[i];
          currentIndex = i;
        }
      }

      // Goal reached!
      if (current.x === goal.x && current.y === goal.y) {
        return this.reconstructPath(current);
      }

      // Move current from open to closed set
      openSet.splice(currentIndex, 1);
      closedSet.push(current);

      // Check all neighbors
      const neighbors = this.getNeighbors(current);
      
      for (const neighbor of neighbors) {
        // Skip if already evaluated
        if (this.nodeExists(closedSet, neighbor)) {
          continue;
        }

        // Calculate costs
        const tentativeG = current.g + 1; // Cost increases by 1 per step
        const existingNode = this.findNode(openSet, neighbor);
        
        // New node or better path found
        if (!existingNode || tentativeG < neighbor.g) {
          neighbor.parent = current;
          neighbor.g = tentativeG;
          neighbor.h = this.heuristic(neighbor, goal);
          neighbor.f = neighbor.g + neighbor.h;

          if (!existingNode) {
            openSet.push(neighbor);
          }
        }
      }
    }

    return []; // No path found
  }

  /**
   * Reconstruct the path from goal back to start
   * by following parent pointers
   */
  reconstructPath(node) {
    const path = [];
    let current = node;
    
    while (current !== null) {
      path.unshift([current.x, current.y]); // Add to beginning
      current = current.parent;
    }
    
    return path;
  }
}

// =====================
// GRAPH VERSION FOR WEIGHTED GRAPHS
// =====================

class GraphNode {
  constructor(name) {
    this.name = name;
    this.g = Infinity;
    this.h = 0;
    this.f = Infinity;
    this.parent = null;
    this.neighbors = []; // Array of {node, cost}
  }

  addNeighbor(node, cost) {
    this.neighbors.push({ node, cost });
  }
}

class AStarGraph {
  constructor(nodes) {
    this.nodes = nodes; // Map of node names to GraphNode objects
  }

  /**
   * Heuristic function: Straight line distance (Euclidean)
   * In real scenarios, you'd use the actual distance between coordinates
   */
  heuristic(current, goal) {
    // For this example, returning 0 makes it similar to Dijkstra
    // You can add coordinates and calculate real distances
    return 0;
  }

  /**
   * A* search for graphs
   */
  search(startName, goalName) {
    const start = this.nodes[startName];
    const goal = this.nodes[goalName];

    const openSet = [start];
    const closedSet = [];

    start.g = 0;
    start.h = this.heuristic(start, goal);
    start.f = start.g + start.h;

    while (openSet.length > 0) {
      // Find node with lowest f value
      let current = openSet[0];
      let currentIndex = 0;

      for (let i = 1; i < openSet.length; i++) {
        if (openSet[i].f < current.f) {
          current = openSet[i];
          currentIndex = i;
        }
      }

      // Goal reached
      if (current.name === goal.name) {
        return this.reconstructPath(current);
      }

      openSet.splice(currentIndex, 1);
      closedSet.push(current);

      // Check all neighbors
      for (const { node: neighbor, cost } of current.neighbors) {
        if (closedSet.includes(neighbor)) {
          continue;
        }

        const tentativeG = current.g + cost;

        if (!openSet.includes(neighbor)) {
          openSet.push(neighbor);
        } else if (tentativeG >= neighbor.g) {
          continue;
        }

        neighbor.parent = current;
        neighbor.g = tentativeG;
        neighbor.h = this.heuristic(neighbor, goal);
        neighbor.f = neighbor.g + neighbor.h;
      }
    }

    return []; // No path found
  }

  reconstructPath(node) {
    const path = [];
    let current = node;

    while (current !== null) {
      path.unshift(current.name);
      current = current.parent;
    }

    return path;
  }
}

// =====================
// EXAMPLE 1: GRID USAGE
// =====================

console.log("=== GRID EXAMPLE ===\n");

// Create a grid: 0 = walkable, 1 = obstacle
const grid = [
  [0, 0, 1, 0, 0],
  [0, 1, 0, 0, 1],
  [0, 0, 0, 1, 0],
  [1, 0, 0, 0, 0],
  [0, 0, 1, 0, 0]
];

// Initialize A* algorithm
const aStar = new AStar(grid);

// Define start and goal positions
const start = new Node(0, 0); // Top-left
const goal = new Node(4, 4);  // Bottom-right

// Find path
const path = aStar.search(start, goal);

console.log("Grid:");
grid.forEach(row => console.log(row));

console.log("\nPath found:");
console.log(path);

// Visualize the path on the grid
console.log("\nVisualized path (. = path, X = obstacle):");
const visualGrid = grid.map(row => [...row]);
for (const [x, y] of path) {
  if (visualGrid[x][y] !== 1) {
    visualGrid[x][y] = '.';
  }
}
visualGrid.forEach(row => 
  console.log(row.map(cell => cell === 0 ? ' ' : cell === 1 ? 'X' : '.').join(' '))
);

// =====================
// EXAMPLE 2: YOUR GRAPH EXAMPLE
// =====================

console.log("\n\n=== YOUR GRAPH EXAMPLE ===\n");

// Create nodes for your graph
const nodeS = new GraphNode("S");
const nodeA = new GraphNode("A");
const nodeB = new GraphNode("B");
const nodeC = new GraphNode("C");
const nodeD = new GraphNode("D");
const nodeE = new GraphNode("E");
const nodeF = new GraphNode("F");
const nodeG = new GraphNode("G");

// Create node map
const graphNodes = {
  S: nodeS,
  A: nodeA,
  B: nodeB,
  C: nodeC,
  D: nodeD,
  E: nodeE,
  F: nodeF,
  G: nodeG
};

// Add BIDIRECTIONAL edges with costs (from your diagram)
// You can travel in BOTH directions on each edge
nodeS.addNeighbor(nodeA, 2);
nodeA.addNeighbor(nodeS, 2);

nodeS.addNeighbor(nodeD, 5);
nodeD.addNeighbor(nodeS, 5);

nodeA.addNeighbor(nodeB, 2);
nodeB.addNeighbor(nodeA, 2);

nodeA.addNeighbor(nodeD, 2);
nodeD.addNeighbor(nodeA, 2);

nodeB.addNeighbor(nodeC, 4);
nodeC.addNeighbor(nodeB, 4);

nodeB.addNeighbor(nodeE, 5);
nodeE.addNeighbor(nodeB, 5);

nodeD.addNeighbor(nodeE, 2);
nodeE.addNeighbor(nodeD, 2);

nodeE.addNeighbor(nodeF, 4);
nodeF.addNeighbor(nodeE, 4);

nodeF.addNeighbor(nodeG, 3);
nodeG.addNeighbor(nodeF, 3);

// Initialize A* for graphs
const aStarGraph = new AStarGraph(graphNodes);

// Find path from S to G
const pathGraph = aStarGraph.search("S", "G");

console.log("Graph structure:");
console.log("S --2--> A");
console.log("S --5--> D");
console.log("A --2--> B");
console.log("A --2--> D");
console.log("B --4--> C");
console.log("B --5--> E");
console.log("C --4--> G");
console.log("D --2--> E");
console.log("E --4--> F");
console.log("F --3--> G");

console.log("\nFinding shortest path from S to G:");
console.log("Path: " + pathGraph.join(" -> "));

// Calculate total cost
let totalCost = 0;
for (let i = 0; i < pathGraph.length - 1; i++) {
  const current = graphNodes[pathGraph[i]];
  const next = pathGraph[i + 1];
  const edge = current.neighbors.find(n => n.node.name === next);
  totalCost += edge.cost;
  console.log(`${pathGraph[i]} -> ${next}: cost ${edge.cost}`);
}
console.log(`Total cost: ${totalCost}`);
