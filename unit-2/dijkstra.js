/**
 * Dijkstra's Algorithm - Interactive Version
 * Find shortest path from start to goal node
 */

class Node {
  constructor(name) {
    this.name = name;
    this.distance = Infinity;
    this.parent = null;
    this.neighbors = [];
  }

  addEdge(node, cost) {
    this.neighbors.push({ node, cost });
  }
}

class Dijkstra {
  search(start, goal) {
    const unvisited = new Set();
    const visited = new Set();
    
    // Collect all nodes reachable from start
    const queue = [start];
    start.distance = 0;
    
    while (queue.length > 0) {
      // Find unvisited node with minimum distance
      let current = null;
      let minDist = Infinity;
      
      for (const node of queue) {
        if (!visited.has(node) && node.distance < minDist) {
          current = node;
          minDist = node.distance;
        }
      }
      
      if (!current || current === goal) break;
      
      visited.add(current);
      
      // Update neighbors
      for (const { node: neighbor, cost } of current.neighbors) {
        if (visited.has(neighbor)) continue;
        
        const newDistance = current.distance + cost;
        if (newDistance < neighbor.distance) {
          neighbor.distance = newDistance;
          neighbor.parent = current;
          if (!queue.includes(neighbor)) {
            queue.push(neighbor);
          }
        }
      }
    }
    
    // Reconstruct path
    if (goal.distance === Infinity) {
      return [];
    }
    
    const path = [];
    let node = goal;
    while (node) {
      path.unshift(node.name);
      node = node.parent;
    }
    return path;
  }
}

const readline = require("readline");

// Interactive mode: collect nodes and edges from user input.
// User flow:
// 1) Enter node names, one per line. Press Enter on an empty line to finish.
// 2) For each node entered, you'll be asked to list its neighbors in the format:
//      B:2, C, D:4
//    (cost is optional; default cost is 1). If you enter a neighbor name that
//    wasn't declared previously it will be auto-created.
// 3) After graph entry you'll be asked for start and goal node names (defaults
//    to the first and last node entered). The program then runs Dijkstra and prints
//    the path and total cost.

async function mainInteractive() {
  const rl = readline.createInterface({ input: process.stdin, output: process.stdout });
  const question = (q) => new Promise(resolve => rl.question(q, resolve));

  console.log("Enter node names one per line. Press Enter on an empty line to finish.");

  const nodes = {};
  const order = [];

  while (true) {
    const name = (await question("> ")).trim();
    if (!name) break;
    if (!nodes[name]) {
      nodes[name] = new Node(name);
      order.push(name);
    } else {
      console.log(`${name} already exists, skipping.`);
    }
  }

  // If user entered no nodes, fall back to the example graph.
  if (order.length === 0) {
    console.log("No nodes entered. Using default example graph.");
    const defaultNodes = {
      S: new Node("S"),
      A: new Node("A"),
      B: new Node("B"),
      C: new Node("C"),
      D: new Node("D"),
      E: new Node("E"),
      F: new Node("F"),
      G: new Node("G")
    };
    const edges = [
      ["S", "A", 2], ["S", "D", 5],
      ["A", "B", 2], ["A", "D", 2],
      ["B", "C", 4], ["B", "E", 5],
      ["D", "E", 2], ["E", "F", 4],
      ["F", "G", 3]
    ];
    Object.assign(nodes, defaultNodes);
    edges.forEach(([a, b, cost]) => {
      nodes[a].addEdge(nodes[b], cost);
      nodes[b].addEdge(nodes[a], cost);
    });
    order.push(...Object.keys(defaultNodes));
  } else {
    // Ask for neighbors for each node entered
    for (const name of order) {
      // Show currently-declared nodes so the user can reference them easily
      console.log(`Declared nodes: ${order.join(", ")}`);
      const line = (await question(`Neighbors for ${name} (format: B:2, C, D:4) > `)).trim();
      if (!line) continue;
      const parts = line.split(",").map(p => p.trim()).filter(Boolean);
      for (const part of parts) {
        let neigh = part;
        let cost = 1;
        if (part.includes(":")) {
          const [n, c] = part.split(":").map(s => s.trim());
          neigh = n;
          const parsed = Number(c);
          if (!Number.isNaN(parsed)) cost = parsed;
        }
        if (!nodes[neigh]) {
          nodes[neigh] = new Node(neigh);
          order.push(neigh);
        }
        nodes[name].addEdge(nodes[neigh], cost);
        // Add reverse edge (make graph undirected)
        nodes[neigh].addEdge(nodes[name], cost);
      }
    }
  }

  // Choose start and goal
  const defaultStart = order[0];
  const defaultGoal = order[order.length - 1];
  let startName = (await question(`Start node (default: ${defaultStart}) > `)).trim();
  if (!startName) startName = defaultStart;
  let goalName = (await question(`Goal node (default: ${defaultGoal}) > `)).trim();
  if (!goalName) goalName = defaultGoal;

  if (!nodes[startName] || !nodes[goalName]) {
    console.log("Start or goal node not found in graph. Exiting.");
    rl.close();
    return;
  }

  const path = new Dijkstra().search(nodes[startName], nodes[goalName]);
  if (path.length === 0) {
    console.log(`No path found from ${startName} to ${goalName}.`);
    rl.close();
    return;
  }

  console.log("Path:", path.join(" -> "));

  // Calculate cost
  let cost = 0;
  for (let i = 0; i < path.length - 1; i++) {
    const edge = nodes[path[i]].neighbors.find(n => n.node.name === path[i + 1]);
    if (edge) cost += edge.cost;
    else cost += 0; // should not happen
  }
  console.log("Cost:", cost);

  rl.close();
}

// If user passed --default, run the built-in example non-interactively.
if (process.argv.includes("--default")) {
  const nodes = {
    S: new Node("S"),
    A: new Node("A"),
    B: new Node("B"),
    C: new Node("C"),
    D: new Node("D"),
    E: new Node("E"),
    F: new Node("F"),
    G: new Node("G")
  };
  const edges = [
    ["S", "A", 2], ["S", "D", 5],
    ["A", "B", 2], ["A", "D", 2],
    ["B", "C", 4], ["B", "E", 5],
    ["D", "E", 2], ["E", "F", 4],
    ["F", "G", 3]
  ];
  edges.forEach(([a, b, cost]) => {
    nodes[a].addEdge(nodes[b], cost);
    nodes[b].addEdge(nodes[a], cost);
  });
  const path = new Dijkstra().search(nodes.S, nodes.G);
  console.log("Path:", path.join(" -> "));
  let cost = 0;
  for (let i = 0; i < path.length - 1; i++) {
    const edge = nodes[path[i]].neighbors.find(n => n.node.name === path[i + 1]);
    cost += edge.cost;
  }
  console.log("Cost:", cost);
  process.exit(0);
} else {
  // Run interactive main
  mainInteractive().catch(err => console.error(err));
}
