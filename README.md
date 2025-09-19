Pacman AI Search Project

This project demonstrates AI search algorithms in action by navigating Pacman through mazes efficiently. It covers classical search strategies, problem formulations, and heuristic design.

🚀 Features

Search Algorithms implemented:

Depth-First Search (DFS)

Breadth-First Search (BFS)

Uniform Cost Search (UCS)

A* Search (with admissible heuristics)

Custom Search Problems:

CornersProblem: Visit all four corners.

FoodSearchProblem: Eat all food pellets optimally.

AnyFoodSearchProblem: Find the closest food.

Heuristics:

cornersHeuristic → max Manhattan distance to unvisited corners.

foodHeuristic → max maze distance to remaining food with caching for efficiency.

Optimal pathfinding with admissible and consistent heuristics.

Efficient priority queue implementation for UCS and A*.

📂 Project Structure
Pacman/
├─ pacman.py                  # Game engine
├─ search.py                  # DFS, BFS, UCS, A*
├─ searchAgents.py            # Problem definitions & heuristics
├─ util.py                    # Data structures (Stack, Queue, PriorityQueue)
├─ README.md                  # This file
└─ layouts/                   # Maze layouts

🎮 Usage

Run Pacman with your choice of search algorithm and problem:

# DFS on tinyMaze
python pacman.py -l tinyMaze -p SearchAgent -a fn=dfs

# BFS on mediumMaze
python pacman.py -l mediumMaze -p SearchAgent -a fn=bfs

# UCS on bigMaze
python pacman.py -l bigMaze -p SearchAgent -a fn=ucs

# A* with corners heuristic
python pacman.py -l mediumCorners -p SearchAgent -a fn=aStarSearch,prob=CornersProblem,heuristic=cornersHeuristic

# A* with food heuristic
python pacman.py -l trickySearch -p AStarFoodSearchAgent

📌 Key Concepts

State Representation: Efficiently encode only relevant info (e.g., Pacman position + visited corners).

Goal Test: Check if all objectives are achieved (e.g., all corners visited, any food eaten).

Successors: Generate next possible moves and update state accordingly.

Heuristic Design: Ensure admissibility and consistency to guarantee optimal A* solutions.

Priority Queue: Used in UCS and A* to always expand the most promising node.

✅ Learning Outcomes

Applied DFS, BFS, UCS, and A search* in a grid-based environment.

Designed state representations and heuristics for complex search problems.

Understood the difference between uninformed and informed search.

Optimized search performance using caching and priority queues.

📚 References

Artificial Intelligence: A Modern Approach – Russell & Norvig

UC Berkeley CS188 Pacman Projects (inspiration)

Python heapq module for priority queue implementation
