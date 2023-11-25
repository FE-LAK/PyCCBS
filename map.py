import networkx as nx
from structs import *
import math

class Map:
    def __init__(self, map_file) -> None:
        self.nodes = {}        
        self.load_map(map_file)

    def load_map(self, map_file):
        self.graph = nx.read_graphml(map_file)
        # Parse node coordinates and assign them to the 'pos' dictionary
        pos = {}
        for node_id_str, node_data in self.graph.nodes(data=True):        
            node_id = int(node_id_str[1:])
            coords = node_data['coords'].split(',')
            x, y = float(coords[0]), float(coords[1])
            self.nodes[node_id] = gNode(x, y)   

        for edge in self.graph.edges(data=True):
            self.nodes[int(edge[0][1:])].neighbours.append(int(edge[1][1:]))

        print(f"Loaded map with {len(self.nodes)} nodes and {len(self.graph.edges(data=True))} edges")
        
    def get_valid_moves(self, id):
        # Create a list of neighbour nodes for SIPP
        return [Node(nn, _x = self.nodes[nn].x, _y = self.nodes[nn].y) for nn in self.nodes[id].neighbours]
    
    def get_dist(self, node1, node2):
        return math.sqrt((node1.x - node2.x)**2 + (node1.y - node2.y)**2)
        
    def get_dist_id(self, id1, id2):
        return math.sqrt((self.nodes[id1].x - self.nodes[id2].x)**2 + (self.nodes[id1].y - self.nodes[id2].y)**2)