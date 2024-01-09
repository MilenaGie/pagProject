import numpy as np
import time
from queue import PriorityQueue

nodes, edges = {}, {}


def path_reconstruction(start_node, end_node, p):
    path = []
    temp_node = end_node
    while temp_node != start_node:
        path.insert(0, p[temp_node][1])
        temp_node = p[temp_node][0]
    return path


def search_Dijkstra(start_node, end_node, path_type="shortest"):    # path_type: shortest/fastest
    start_time = time.process_time()
    S = {}
    Q = nodes.copy()
    d = np.full(len(Q), 999999)
    p = np.full(len(Q), -1)
    d, p = list(d), list(p)
    d[start_node] = 0
    value = 0

    current_node = start_node
    while True:
        if current_node not in Q:
            break
        S[current_node] = Q.pop(current_node)
        # przegladanie wszystkich sasiadow wierzcholka
        for edge in S[current_node].edges_id:
            # ustalanie typu trasy (najkrotsza/najszybsza)
            if path_type == "shortest":
                value = edges[edge].length
            if path_type == "fastest":
                value = edges[edge].cost
            # znalezienie wierzchołka lezacego po przeciwnej stronie krawedzi
            node_to = edges[edge].node_to
            if node_to == S[current_node].id_node:
                node_to = edges[edge].node_from

            if node_to in Q:
                if d[node_to] > d[current_node] + value:
                    d[node_to] = d[current_node] + value
                    p[node_to] = (current_node, edge)
        if len(Q) == 0:
            break

        least_value = min([d[index] for index in list(Q)])
        current_node = d.index(least_value)

    # rekonstrukcja trasy:
    path = path_reconstruction(start_node, end_node, p)
    print(d[end_node])
    print(time.process_time() - start_time, "s")
    return path


def search_A_star(start_node, end_node, path_type="shortest", priority_queue=True):    # path_type: shortest/fastest
    start_time = time.process_time()
    S, Q = {}, {}
    pq = PriorityQueue()
    g = np.full(len(nodes), -1)
    p, f = g.copy(), g.copy()
    g, p, f = list(g), list(p), list(f)
    g[start_node] = 0
    value, heur = 0, 0

    current_node = start_node
    while True:
        if current_node == end_node:
            break
        S[current_node] = nodes[current_node]
        # przegladanie wszystkich sasiadow wierzcholka
        for edge in S[current_node].edges_id:
            # ustalanie typu trasy (najkrotsza/najszybsza)
            if path_type == "shortest":
                value = edges[edge].length
                heur = edges[edge].heuristics
            if path_type == "fastest":
                value = edges[edge].cost
                heur = edges[edge].cost_heur

            node_to = edges[edge].node_to
            if node_to == S[current_node].id_node:
                node_to = edges[edge].node_from

            if node_to not in S:
                if g[node_to] > g[current_node] + value or g[node_to] == -1:
                    Q[node_to] = nodes[node_to]
                    g[node_to] = g[current_node] + value
                    f[node_to] = f[current_node] + value + heur
                    pq.put((f[node_to], node_to))
                    p[node_to] = (current_node, edge)

        if current_node in Q:
            Q.pop(current_node)

        if priority_queue:
            least_value = pq.get()
            current_node = least_value[1]
        else:
            least_value = min([f[index] for index in list(Q)])
            current_node = f.index(least_value)

    # rekonstrukcja trasy:
    path = path_reconstruction(start_node, end_node, p)
    print(g[end_node])
    print(time.process_time() - start_time, "s")
    return path


def get_range(start_node, time_value):
    start_time = time.process_time()
    visited_nodes, nodes_to_visit = {}, {}
    g = np.full(len(nodes), -1)
    g = list(g)
    g[start_node] = 0
    visited_edges = []

    current_node = start_node
    while True:
        visited_nodes[current_node] = nodes[current_node]
        # przegladanie wszystkich sasiadow wierzcholka
        for edge in visited_nodes[current_node].edges_id:
            value = edges[edge].cost

            node_to = edges[edge].node_to
            if node_to == visited_nodes[current_node].id_node:
                node_to = edges[edge].node_from

            if node_to not in visited_nodes:
                g[node_to] = g[current_node] + value
                if g[node_to] <= time_value:
                    nodes_to_visit[node_to] = nodes[node_to]
                    visited_edges.append(edge)

        if current_node in nodes_to_visit:
            nodes_to_visit.pop(current_node)

        if len(nodes_to_visit) == 0:
            break
        else:
            current_node = list(nodes_to_visit.keys())[0]

    print(time.process_time() - start_time, "s")
    return visited_edges
