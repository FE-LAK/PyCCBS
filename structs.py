class Agent:
    def __init__(self, s_id = -1, g_id = -1, _id = -1, _start_x = -1, _start_y = -1, _goal_x = -1, _goal_y = -1) -> None:
        self.start_id = s_id
        self.goal_id = g_id
        self.id = _id        

class gNode: # Graph node
    def __init__(self, _x = 0, _y = 0) -> None:
        self.x, self.y = _x, _y

        self.neighbours = []

class Node:
    def __init__(self, _id = -1, _f = -1, _g = -1, _x = 0, _y = 0, _parent = None, begin = -1, end = -1) -> None:
        self.id, self.f, self.g, self.x, self.y, self.parent, self.interval = _id, _f, _g, _x, _y, _parent, [begin, end]
        self.interval_id = 0

    def clone(self):
        return Node(self.id, self.f, self.g, self.x, self.y, self.parent, self.interval[0], self.interval[1])
    
    def __repr__(self) -> str:
        return f"Node #{self.id} ({self.x},{self.y}) f={self.f}, g={self.g} interval={self.interval}"

class Path:
    def __init__(self, _nodes = None, _cost = -1, _agentID = -1) -> None:        
        self.nodes, self.cost, self.agentID = _nodes, _cost, _agentID
        self.expanded = 0
        if (self.nodes is None):
            self.nodes = []        
        #print("New Path at " + hex(id(self)))
    
    def __repr__(self) -> str:
        if not self.nodes:
            return "Path empty"
        return f"Path {[f'{node.id}@{int(node.g)}' for node in self.nodes]}"
    

class sNode:
    def __init__(self, _id = -1, _g = -1) -> None:
        self.id, self.g = _id, _g
    
    def clone(self):
        return sNode(self.id, self.g)

class sPath:
    def __init__(self, _nodes, _cost, _agentID) -> None:
        self.nodes, self.cost, self.agentID = _nodes, _cost, _agentID
        self.expanded = 0

class Constraint:
    # prohibited to start moving from id1 to id2 during interval (t1, t2)
    def __init__(self, _agent = -1, _t1 = -1, _t2 = -1, _id1 = -1, _id2 = -1, _positive = False) -> None:
        self.agent, self.t1, self.t2, self.id1, self.id2, self.positive = _agent, _t1, _t2, _id1, _id2, _positive

class Move:
    def __init__(self, _t1 = -1, _t2 = -1, _id1 = -1, _id2 = -1) -> None:
        self.t1, self.t2, self.id1, self.id2 = _t1, _t2, _id1, _id2
    
    @staticmethod
    def fromMove(m):
        return Move(m.t1, m.t2, m.id1, m.id2)
    
    @staticmethod
    def fromConstraint(c):
        return Move(c.t1, c.t2, c.id1, c.id2)
    
    def __repr__(self) -> str:
        return f"Move {self.id1}->{self.id2} @ {self.t1}-{self.t2}"

class Conflict:
    def __init__(self, _agent1 = -1, _agent2 = -1, _move1 = Move(), _move2 = Move(), _t = 1e8) -> None:
        self.agent1, self.agent2, self.move1, self.move2, self.t = _agent1, _agent2, _move1, _move2, _t
        self.overcost = 0

class CBS_Node:
    def __init__(self, _paths = None, _parent = None, _constraint = Constraint(), _cost = 0, _conflicts_num = 0, _total_cons_ = 0) -> None:
        self.paths, self.parent, self.constraint, self.cost, self.conflicts_num, self.total_cons = _paths, _parent, _constraint, _cost, _conflicts_num, _total_cons_
        self.low_level_expanded = 0
        self.h = 0
        self.conflicts = []
        self.semicard_conflicts = []
        self.cardinal_conflicts = []
        if self.paths is None:
            self.paths = []

class Open_Elem:
    def __init__(self, _tree_pointer = None, _id = -1, _cost = -1, _cons_num = 0, _conflicts_num = 0) -> None:
        self.tree_pointer, self.id, self.cost, self.cons_num, self.conflicts_num = _tree_pointer, _id, _cost, _cons_num, _conflicts_num

class CBS_Tree:
    def __init__(self) -> None:
        self.tree = []
        self.container = []
        self.open_size = 0
        self.closed = []

    def get_size(self):
        return len(self.tree)
    
    def get_open_size(self):
        return self.open_size
    
    def add_node(self, node):
        self.tree.append(node)
        #ToDo: container optimization using heap/priority queue structures
        self.container.append(Open_Elem(node, node.id, node.cost, node.total_cons, node.conflicts_num))
        self.container.sort(key=lambda x : x.cost)
        self.open_size += 1

    def get_front(self):
        return self.container.pop(0)
    
    def get_paths(self, node, size):
        paths = [None] * size
        while node.parent != None:
            p = paths[node.paths[0].agentID]
            if p == None or len(p.nodes) == 0:
                paths[node.paths[0].agentID] = node.paths
            node = node.parent

        for i in range(len(node.paths)):
            if paths[i] == None or len(paths[i].nodes) == 0:
                paths[i] = node.paths[i]
        return paths