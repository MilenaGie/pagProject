import shapefile

# kolekcje:
index_nodes = {}    # (x,y)     ->  id_node
nodes = {}          # id_node   ->  Node
edges = {}          # id_edge   ->  Edge

generator_id_node = 1


class Edge:
    length, cost = float, float
    road_type = str

    def __init__(self, id_road, node_from, node_to):
        self.id_road, self.node_from, self.node_to = id_road, node_from, node_to


class Node:
    edges_id = []

    def __init__(self, id_node, x, y):
        self.id_node, self.x, self.y = id_node, x, y


def read_file(file):
    shp = shapefile.Reader(file)
    roads = shp.shapes()
    bbox = shp.bbox
    min_x, min_y = bbox[0], bbox[1]
    return roads, (min_x, min_y)


def create_edges(roads, min_xy):
    edge_id = 1
    min_x, min_y = min_xy

    for road in roads:
        n = len(road.points) - 1
        x_1, y_1 = road.points[0]
        x_n, y_n = road.points[n]
        node_from = get_node_id(x_1-min_x, y_1-min_y)
        node_to = get_node_id(x_n-min_x, y_n-min_y)

        edge = Edge(edge_id, node_from, node_to)
        edges[edge_id] = edge

        # nodes[node_from].edges_id.append(edge_id)  # ??
        edge_id += 1


def get_node_id(x, y):
    global generator_id_node
    if (x, y) not in index_nodes:
        node = Node(generator_id_node, x, y)
        nodes[generator_id_node] = node
        index_nodes[(x, y)] = generator_id_node
        generator_id_node += 1
    return index_nodes[(x, y)]


def main():
    # roads, min_xy = read_file("data/L4_1_BDOT10k__OT_SKJZ_L.shp")
    roads, min_xy = read_file("data/test.shp")
    create_edges(roads, min_xy)


if __name__ == '__main__':
    main()
