/**
 * A* Algorithm - Minimal Version
 * Simple implementation for learning
 */

class Node {
  constructor(name) {
    this.name = name;
    this.g = Infinity;
    this.h = 0;
    this.f = Infinity;
    this.parent = null;
    this.neighbors = [];
  }

  addEdge(node, cost) {
    this.neighbors.push({ node, cost });
  }
}

class AStar {
  search(start, goal) {
    const open = [start];
    const closed = [];
    start.g = 0;

    while (open.length > 0) {
      // Find node with lowest f
      let current = open[0];
      let idx = 0;
      for (let i = 1; i < open.length; i++) {
        if (open[i].f < current.f) {
          current = open[i];
          idx = i;
        }
      }

      if (current === goal) {
        const path = [];
        let node = current;
        while (node) {
          path.unshift(node.name);
          node = node.parent;
        }
        return path;
      }

      open.splice(idx, 1);
      closed.push(current);

      for (const { node: neighbor, cost } of current.neighbors) {
        if (closed.includes(neighbor)) continue;

        const newG = current.g + cost;
        if (!open.includes(neighbor)) open.push(neighbor);
        else if (newG >= neighbor.g) continue;

        neighbor.parent = current;
        neighbor.g = newG;
        neighbor.h = 0; // Simple heuristic
        neighbor.f = neighbor.g + neighbor.h;
      }
    }
    return [];
  }
}

// === EXAMPLE: S to G ===
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

// Add edges (bidirectional)
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

const path = new AStar().search(nodes.S, nodes.G);
console.log("Path:", path.join(" -> "));

// Calculate cost
let cost = 0;
for (let i = 0; i < path.length - 1; i++) {
  const edge = nodes[path[i]].neighbors.find(n => n.node.name === path[i + 1]);
  cost += edge.cost;
}
console.log("Cost:", cost);
