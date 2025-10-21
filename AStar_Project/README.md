This project implements a single A* algorithm that can run in three modes:

1. UCS mode (`h(n)=0`)
2. A* with Euclidean heuristic
3. A* with Manhattan heuristic

### Input Format
Text files with:
- Vertex lines: `<id>,<cell_id>`
- Edge lines: `<u>,<v>,<w>`
- Source/Destination lines: `S,<id>` and `D,<id>`

### Run the program
```bash
python astar_project.py astar_small.txt --mode all
python astar_project.py astar_medium.txt --mode all
