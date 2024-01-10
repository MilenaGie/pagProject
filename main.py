import node_edge
import path_finding_algorithms as alg
import shapefile
import numpy as np

# kolekcje:
index_nodes = {}    # (x,y) ->  id
nodes = {}          # id    ->  node
edges = {}          # id    ->  edge

generator_node_id = 0

round_prec = 10  # [m]


def get_round_coor(coor):
    return round(coor / round_prec) * round_prec


def edge_length(points):
    length = 0
    for n in range(len(points) - 1):
        pt_0 = points[n]
        pt = points[n + 1]
        length += np.sqrt(np.square(pt_0[0] - pt[0]) + np.square(pt_0[1] - pt[1]))
    return length


def distance_between_nodes(xy1, xy2):
    dist = np.sqrt(np.square(xy1[0] - xy2[0]) + np.square(xy1[1] - xy2[1]))
    return dist


def snap_to_node(x, y):
    buff_size, buff_add = 5000, 1000
    while True:
        selected = dict((k, v) for k, v in index_nodes.items()
                        if get_round_coor(x+buff_size) > k[0] > get_round_coor(x-buff_size)
                        and get_round_coor(y+buff_size) > k[1] > get_round_coor(y-buff_size))
        if len(selected) == 0:
            buff_size += buff_add
        elif len(selected) > 1:
            dist = []
            for node in list(selected):
                dist.append(distance_between_nodes((x, y), node))
            i = dist.index(min(dist))
            return index_nodes[list(selected)[i]]
        else:
            return index_nodes[list(selected)[0]]


def node_path_to_shp(file, path):
    print("saving to shapefile...")
    roads_in_path = {}
    for edge in path:
        roads_in_path[edges[edge].shp_id] = 0

    shp = shapefile.Reader(file, encoding="ANSI")
    w = shapefile.Writer("result", shapeType=3)
    w.field("id", "C")
    w.field("dlugosc", "F")
    w.field("klasaDrogi", "C")
    w.field("oneway", "N")
    i = 0
    for rec in shp.iterShapes():
        if i in roads_in_path:
            w.line([rec.points])
            w.record(edges[i].shp_id, edges[i].length, edges[i].road_type, edges[i].oneway)
        i += 1

    w.close()


def read_file(file):
    print("reading file...")
    shp = shapefile.Reader(file, encoding="ANSI")
    if shp.shapeType != 3:  # 3: polyline
        return

    roads = shp.shapes()
    road_types, oneway = [], []
    shp_id = [i for i in range(len(roads))]
    # uzyskanie atrybutu klasy drogi (zalozenie, ze to dane z BDOTu)
    for rec in shp.iterRecords(fields=["klasaDrogi"]):
        road_types.append(rec[0])
    # proba uzyskania atrybutu kierunkowosci
    try:
        for rec in shp.iterRecords(fields=["oneway"]):  # iterRecords - dla duzych plikow
            oneway.append(bool(rec[0]))
    except:
        oneway.append(0)
    # wyznaczenie najmniejszych wspolrzednych xy w celu dalszego zredukowania dużych liczb w grafie
    bbox = shp.bbox
    min_x, min_y = bbox[0], bbox[1]

    return roads, (min_x, min_y), road_types, oneway, shp_id


def create_edges(roads, min_xy, road_types, oneway, shp_id):
    print("creating edges...")
    edge_id = 0
    min_x, min_y = min_xy
    for i in range(len(roads)):
        n = len(roads[i].points) - 1
        # redukcja duzych wspolrzednych dla wierzcholkow krawedzi
        x1, y1 = roads[i].points[0]
        xn, yn = roads[i].points[n]
        x1 -= min_x
        xn -= min_x
        y1 -= min_y
        yn -= min_y
        # przypisanie wierzcholkow krawedzi
        node_from = get_node_id(x1, y1)
        node_to = get_node_id(xn, yn)
        # przypisanie atrybutow do krawedzi
        edge = node_edge.Edge(edge_id, node_from, node_to)
        edge.shp_id = shp_id[i]
        edge.length = edge_length(roads[i].points)
        edge.road_type = road_types[i]
        edge.heuristics = distance_between_nodes((x1, y1), (xn, yn))
        edge.get_costs()

        if len(oneway) == 1:
            edge.oneway = bool(oneway[0])
        else:
            edge.oneway = bool(oneway[i])
        edges[edge_id] = edge
        # przypisanie krawedzi do wierzcholkow z uwzglednieniem kierunkowosci
        curr_node = nodes[node_from]
        curr_node.edges_id = curr_node.edges_id + [edge_id]
        if not edge.oneway:
            curr_node = nodes[node_to]
            curr_node.edges_id = curr_node.edges_id + [edge_id]

        edge_id += 1


def get_node_id(x, y):
    global generator_node_id
    x_round, y_round = get_round_coor(x), get_round_coor(y)
    if (x_round, y_round) not in index_nodes:
        node = node_edge.Node(generator_node_id, x, y)
        nodes[generator_node_id] = node
        index_nodes[(x_round, y_round)] = generator_node_id
        generator_node_id += 1
    return index_nodes[(x_round, y_round)]


def set_up(roads_file, start_pt, end_pt=None):
    roads, min_xy, road_types, oneway, shp_id = read_file(roads_file)
    create_edges(roads, min_xy, road_types, oneway, shp_id)
    alg.nodes, alg.edges = nodes, edges
    p1 = snap_to_node(start_pt[0]-min_xy[0], start_pt[1]-min_xy[1])
    p2 = None
    if end_pt is not None:
        p2 = snap_to_node(end_pt[0]-min_xy[0], end_pt[1]-min_xy[1])
    return p1, p2


# cost_type:        "shortest" / "fastest"
# algorithm_type:   "Astar" / "Dijkstra"
def get_path(roads_file, start_pt, end_pt, path_type="shortest", algorithm_type="Astar", pq=True):
    p1, p2 = set_up(roads_file, start_pt, end_pt)
    print("algorithm...")
    if algorithm_type == "Astar":
        path = alg.search_A_star(p1, p2, path_type, priority_queue=pq)
    if algorithm_type == "Dijkstra":
        path = alg.search_Dijkstra(p1, p2, path_type)
    node_path_to_shp(roads_file, path)


# time [s]
def get_range(roads_file, start_pt, time):
    p1, p2 = set_up(roads_file, start_pt)
    path = alg.get_range(p1, time)
    node_path_to_shp(roads_file, path)


def main():
    file = "data/L4_1_BDOT10k__OT_SKJZ_L.shp"
    # get_path(file, (470697.22, 568043.58), (474409.77, 576926.38), algorithm_type="Astar", path_type="shortest", pq=True)
    get_range(file, (479641.0,574538.3), 5*60)
    # get_path(file, (471524.1, 571970.6), (473638.1, 574654.5), path_type="shortest")


if __name__ == '__main__':
    main()
