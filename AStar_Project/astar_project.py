import sys
import math
import time
import argparse
from collections import defaultdict
import heapq



def parse_input(filename):
    vertices = set()
    coords = {}
    adj = defaultdict(list)
    source = None
    dest = None

    with open(filename, 'r', encoding='utf-8') as f:
        for raw in f:
            line = raw.strip()
            if not line or line.startswith('#'):
                continue
            parts = [p.strip() for p in line.split(',')]
            if parts[0].upper() == 'S':
                source = int(parts[1])
            elif parts[0].upper() == 'D':
                dest = int(parts[1])
            elif len(parts) == 2:
                node = int(parts[0])
                cell_id = int(parts[1])
                x = cell_id // 10
                y = cell_id % 10
                coords[node] = (x, y)
                vertices.add(node)
            elif len(parts) == 3:
                u, v, w = int(parts[0]), int(parts[1]), float(parts[2])
                adj[u].append((v, w))
                adj[v].append((u, w))
                vertices.update([u, v])
    return vertices, coords, adj, source, dest



def h_zero(n, g, coords): return 0
def h_euclidean(n, g, coords):
    xn, yn = coords[n]; xg, yg = coords[g]
    return math.hypot(xn - xg, yn - yg)
def h_manhattan(n, g, coords):
    xn, yn = coords[n]; xg, yg = coords[g]
    return abs(xn - xg) + abs(yn - yg)



def astar(vertices, coords, adj, source, dest, heuristic):
    start_time = time.perf_counter()
    g_cost = {v: float('inf') for v in vertices}
    parent = {v: None for v in vertices}

    g_cost[source] = 0
    heap = []
    pushes = expanded = 0
    max_frontier = 0

    heapq.heappush(heap, (heuristic(source, dest, coords), source, 0))
    pushes += 1

    while heap:
        f, node, g_val = heapq.heappop(heap)
        if g_val > g_cost[node]:  
            continue
        expanded += 1

        if node == dest:
            path = []
            while node is not None:
                path.append(node)
                node = parent[node]
            path.reverse()
            end = time.perf_counter()
            return g_cost[dest], path, {
                'expanded': expanded, 'pushes': pushes,
                'max_frontier': max_frontier, 'runtime_s': end - start_time
            }

        for v, w in adj[node]:
            new_g = g_cost[node] + w
            if new_g < g_cost[v]:
                g_cost[v] = new_g
                parent[v] = node
                f_n = new_g + heuristic(v, dest, coords)
                heapq.heappush(heap, (f_n, v, new_g))
                pushes += 1
                max_frontier = max(max_frontier, len(heap))

    end = time.perf_counter()
    return None, None, {
        'expanded': expanded, 'pushes': pushes,
        'max_frontier': max_frontier, 'runtime_s': end - start_time
    }



def report(mode, cost, path, stats):
    print(f"MODE: {mode}")
    print(f"Optimal cost: {cost if cost is not None else 'NO PATH'}")
    if path: print("Path: " + " -> ".join(map(str, path)))
    for k, v in stats.items():
        if k == 'runtime_s':
            print(f"{k}: {v:.6f}")
        else:
            print(f"{k}: {v}")
    print()



def run_all(filename):
    vertices, coords, adj, s, d = parse_input(filename)
    if s is None or d is None:
        print("Source/Destination missing!")
        return

    for mode, h_func in [
        ("UCS", h_zero),
        ("A* Euclidean", h_euclidean),
        ("A* Manhattan", h_manhattan)
    ]:
        cost, path, stats = astar(vertices, coords, adj, s, d, h_func)
        report(mode, cost, path, stats)



if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("file", help="input text file")
    parser.add_argument("--mode", choices=["ucs","euclid","manhattan","all"], default="all")
    args = parser.parse_args()

    if args.mode == "all":
        run_all(args.file)
    elif args.mode == "ucs":
        v,c,a,s,d = parse_input(args.file)
        cst,pth,st = astar(v,c,a,s,d,h_zero); report("UCS",cst,pth,st)
    elif args.mode == "euclid":
        v,c,a,s,d = parse_input(args.file)
        cst,pth,st = astar(v,c,a,s,d,h_euclidean); report("A* Euclidean",cst,pth,st)
    elif args.mode == "manhattan":
        v,c,a,s,d = parse_input(args.file)
        cst,pth,st = astar(v,c,a,s,d,h_manhattan); report("A* Manhattan",cst,pth,st)
