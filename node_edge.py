# predkosci maksymalne:
# A-autostrada, S-ekspresowa, GP-glowna ruchu przysp., G-glowna, Z-zbiorcza, L-lokalna, D-dojazdowa, I-inna [km/h]
road_costs = {"A": 140, "S": 130, "GP": 110, "G": 100, "Z": 80, "L": 60, "D": 40, "I": 50}

# wspolczynnik wplywajacy na to, o ile wolniej poruszaja sie kierowcy w stosunku do maksymalnych predkosci
factor = 1/4


class Edge:
    shp_id = str
    length = float
    road_type = str
    oneway = bool
    heuristics = float
    cost, cost_heur = float, float  # czas [s]

    def __init__(self, id_road, node_from, node_to):
        self.id_road, self.node_from, self.node_to = id_road, node_from, node_to

    def get_costs(self):
        v = road_costs[self.road_type] * (10/36)
        self.cost = self.length / (v * factor)
        self.cost_heur = self.heuristics / v


class Node:
    edges_id = []

    def __init__(self, id_node, x, y):
        self.id_node, self.x, self.y = id_node, x, y
