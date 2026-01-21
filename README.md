# 🌌 Erebus: Self-Optimizing Pathfinding Engine

[![C++](https://img.shields.io/badge/C++-17-blue.svg?style=flat-square&logo=c%2B%2B)](https://isocpp.org/)
[![SFML](https://img.shields.io/badge/SFML-3.0-green.svg?style=flat-square)](https://www.sfml-dev.org/)
[![License](https://img.shields.io/badge/license-MIT-brightgreen.svg?style=flat-square)](LICENSE)

Erebus is a high-performance, real-time pathfinding visualizer built with C++ and SFML. Unlike static pathfinding tools, Erebus features a **Self-Optimizing Knowledge Base** and **Optimal Dynamic Relaxation**, allowing it to learn from successful paths and adapt to environmental changes with mathematical precision.

---

## 🚀 Key Features

- **🧠 Self-Optimization (Knowledge Base)**: Remembers previous successful routes in `knowledge.txt`. Future searches receive a heuristic bonus on "tried and true" paths, simulating institutional memory.
- **⚡ Optimal Dynamic Relaxation**: Implemented with Dijkstra-level optimality. If a wall is removed mid-search, the engine propagates cost improvements ($g$-values) to all affected nodes instantly, allowing for real-time optimal re-routing.
- **📱 Responsive UI**: A professional dual-view system using `sf::View`. The grid and sidebar scale independently. Toggle **Fullscreen (F11)** for an immersive experience.
- **🧩 Maze Generation**: Features a randomized DFS (Recursive Backtracking) corridor-carving algorithm. Generates complex, organic mazes while ensuring guaranteed Start-to-End connectivity.
- **🕒 Adjustable Speed Control**: Introduced a timer-based execution model using `sf::Clock`. Users can precisely control the visualization speed via the `stepDelay` parameter.
- **📍 Google Maps Aesthetic**: Professional visuals featuring:
  - Deep Navy Blue route ribbons (Triangle Strips for smooth paths).
  - Custom Start/End pins with smooth animations.
  - Depth-aware gradient coloring for visited nodes.

---

## 🧩 Algorithms

Erebus implements 8 core algorithms for comprehensive analysis:

| Algorithm | Type | Description |
| :--- | :--- | :--- |
| **Dijkstra** | Weighted | Guaranteed shortest path using edge relaxation. |
| **A* Search** | Heuristic | Optimized Dijkstra using Manhattan distance. |
| **Greedy BFS** | Best-First | Prioritizes speed by ignoring path cost. |
| **Swarm** | Hybrid | Weighted A* that explores multiple branches like a hive mind. |
| **Convergent Swarm** | Aggressive | High-heuristic weight for extremely direct pathing. |
| **Bidirectional** | Parallel | Simultaneous search from start and end points. |
| **BFS** | Unweighted | Guaranteed shortest path in hop-count. |
| **DFS** | Exploratory | Deep-branch exploration for maze traversal. |

---

## 🛠️ Installation & Setup

### Prerequisites

- **C++17** compatible compiler (GCC/Clang/MSVC).
- **SFML 3.0** library installed on your system.

### Build

```ps1
# Build with G++ (Windows/Linux)
g++ -std=c++17 src/main.cpp -o main -lsfml-graphics -lsfml-window -lsfml-system
```

### Run

```ps1
./main.exe
```

---

## 🎮 How to Use

1. **Draw Walls**: Click and drag on empty cells to create obstacles.
2. **Erase Walls**: Watch the algorithm adapt to new shortcuts in real-time!
3. **Drag Pins**: Move markers to re-calculate paths instantly.
4. **Generate Maze**: Click to build a randomized DFS-based layout.
5. **Adjust Speed**: Modify `stepDelay` in `main.cpp` for adjustable pacing.
6. **Choose Strategy**: Select one of the 8 algorithms from the sidebar.
7. **Visualize**: Press **Visualize!** to begin the pathfinding simulation.
8. **Fullscreen**: Toggle immersive mode with **F11**.

---

## 🤝 Contributing

Contributions are what make the open-source community such an amazing place to learn, inspire, and create.

1. Fork the Project
2. Create your Feature Branch (`git checkout -b feature/AmazingFeature`)
3. Commit your Changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the Branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

---

## 📄 License

Distributed under the MIT License. See `LICENSE` for more information.

**Authors**: Saif ur Rehman, Huzaifa Imran, Ayesha Tasneem  
**Project Date**: January 2026
