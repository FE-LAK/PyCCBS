import datetime

class Agent:
    def __init__(self, s_id = -1, g_id = -1, _id = -1) -> None:
        self.start_id = s_id
        self.goal_id = g_id
        self.id = _id        

    def __repr__(self) -> str:
        return f"{self.id}: {self.start_id}->{self.goal_id}"

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
    def __init__(self, _nodes = None, _cost = -1, _agentID = -1) -> None:
        self.nodes, self.cost, self.agentID = _nodes, _cost, _agentID
        if self.nodes is None:
            self.nodes = []
        self.expanded = 0

class Constraint:
    # prohibited to start moving from id1 to id2 during interval (t1, t2)
    def __init__(self, _agent = -1, _t1 = -1, _t2 = -1, _id1 = -1, _id2 = -1, _positive = False) -> None:
        self.agent, self.t1, self.t2, self.id1, self.id2, self.positive = _agent, _t1, _t2, _id1, _id2, _positive
        if self.t2 < self.t1:
            print("Negative time interval")

class Move:
    def __init__(self, _t1 = -1, _t2 = -1, _id1 = -1, _id2 = -1) -> None:
        self.t1, self.t2, self.id1, self.id2 = _t1, _t2, _id1, _id2
    
    @staticmethod
    def fromMove(m):
        return Move(m.t1, m.t2, m.id1, m.id2)
    
    @staticmethod
    def fromConstraint(c):
        return Move(c.t1, c.t2, c.id1, c.id2)
    
    @staticmethod
    def fromNodes(a, b):
        return Move(a.g, b.g, a.id, b.id)
    
    def __repr__(self) -> str:
        return f"Move {self.id1}->{self.id2} @ {self.t1}-{self.t2}"

class Conflict:
    def __init__(self, _agent1 = -1, _agent2 = -1, _move1 = None, _move2 = None, _t = float('inf')) -> None:
        self.agent1, self.agent2, self.move1, self.move2, self.t = _agent1, _agent2, _move1, _move2, _t
        self.overcost = 0
        if self.move1 is None:
            self.move1 = Move()
        if self.move2 is None:
            self.move2 = Move()

    def clone(self):
        return Conflict(self.agent1, self.agent2, self.move1, self.move2, self.t)
    
    def __repr__(self) -> str:
        return f"Conflict {self.agent1}/{self.agent2} {self.move1}/{self.move2} at t={self.t}"

class CBS_Node:
    def __init__(self, _paths = None, _parent = None, _constraint = None, _cost = 0, _conflicts_num = 0, _total_cons_ = 0) -> None:
        self.id = -1
        self.id_str = ""
        self.paths, self.parent, self.constraint, self.cost, self.conflicts_num, self.total_cons = _paths, _parent, _constraint, _cost, _conflicts_num, _total_cons_
        self.low_level_expanded = 0
        self.h = 0
        self.conflicts = []
        self.semicard_conflicts = []
        self.cardinal_conflicts = []
        self.positive_constraint = Constraint()

        if self.constraint is None:
            self.constraint = Constraint()
        if self.paths is None:
            self.paths = []

    def create_node_move_conflicts(self):
        newNode = CBS_Node(self.paths, self, None, self.cost, self.conflicts_num, self.total_cons)
        
        newNode.conflicts.extend(self.conflicts)
        newNode.semicard_conflicts.extend(self.semicard_conflicts)
        newNode.cardinal_conflicts.extend(self.cardinal_conflicts)

        newNode.id = self.id
        newNode.id_str = self.id_str

        newNode.constraint = self.constraint

        self.conflicts.clear()
        self.semicard_conflicts.clear()
        self.cardinal_conflicts.clear()

        return newNode



class Open_Elem:
    def __init__(self, _tree_pointer = None, _id = -1, _cost = -1, _cons_num = 0, _conflicts_num = 0) -> None:
        self.tree_pointer, self.id, self.cost, self.cons_num, self.conflicts_num = _tree_pointer, _id, _cost, _cons_num, _conflicts_num

class CBS_Tree:
    def __init__(self) -> None:
        self.tree = []
        self.container = []
        self.open_size = 0
        self.closed = []

    def get_size(self) -> int:
        return len(self.tree)
    
    def get_open_size(self) -> int:
        return self.open_size
    
    def add_node(self, node : CBS_Node):
        self.tree.append(node)
        #ToDo: container optimization using heap/priority queue structures
        self.container.append(Open_Elem(node, node.id, node.cost, node.total_cons, node.conflicts_num))
        self.container.sort(key=lambda x : x.cost)
        self.open_size += 1

    def get_front(self) -> CBS_Node:
        return self.container.pop(0).tree_pointer
    
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
    
class Solution:
    def __init__(self, found=False, flowtime=-1, makespan=-1, check_time=0, init_cost=-1, constraints_num=0,
                 max_constraints=0, high_level_expanded=0, high_level_generated=0, low_level_expansions=0,
                 low_level_expanded=0, cardinal_solved=0, semicardinal_solved=0, time=None, init_time=None,
                 paths=None):
        self.found = found
        self.flowtime = flowtime
        self.makespan = makespan
        self.check_time = check_time
        self.init_cost = init_cost
        self.constraints_num = constraints_num
        self.max_constraints = max_constraints
        self.high_level_expanded = high_level_expanded
        self.high_level_generated = high_level_generated
        self.low_level_expansions = low_level_expansions
        self.low_level_expanded = low_level_expanded
        self.cardinal_solved = cardinal_solved
        self.semicardinal_solved = semicardinal_solved
        self.task = None
        self.time = time if time is not None else datetime.timedelta()
        self.init_time = init_time if init_time is not None else datetime.timedelta()
        self.paths = paths if paths is not None else []

    def __del__(self):
        self.paths.clear()
        self.found = False

    def __repr__(self) -> str:
        return f"***************************\nSolution found: {self.found}\n\tRuntime: {self.time}\n\tMakespan: {self.makespan}\n\tFlowtime: {self.flowtime}\n\tInitial cost: {self.init_cost}\n\tCollision checking time: {self.check_time}\n\tHL expanded: {self.high_level_expanded}\n\tLL searches: {self.low_level_expansions}\n\tLL expanded(avg): {self.low_level_expanded}"    

class Task:
    def __init__(self) -> None:
        self.agents = []

    def get_agent(self, id) -> Agent:
        return [agent for agent in self.agents if agent.id == id][0]
    
    def load_from_file(self, file):
        from xml.etree import cElementTree as ElementTree

        tree = ElementTree.parse(file)
        root = tree.getroot()
        
        self.agents.clear()
        agent_id = 0
        for agent_task in root:
            ag_attr = agent_task.attrib
            self.agents.append(Agent(int(ag_attr['start_id']), int(ag_attr['goal_id']), agent_id))
            agent_id += 1        

    def __repr__(self) -> str:
        return str(self.agents)
    


class Vector2D:
    def __init__(self, _x=0.0, _y=0.0):
        self.x = _x
        self.y = _y

    def __add__(self, vec):
        return Vector2D(self.x + vec.x, self.y + vec.y)

    def __sub__(self, vec):
        return Vector2D(self.x - vec.x, self.y - vec.y)

    def __neg__(self):
        return Vector2D(-self.x, -self.y)

    def __truediv__(self, num):
        return Vector2D(self.x / num, self.y / num)

    def __mul__(self, num):
        return Vector2D(self.x * num, self.y * num)

    def dot(self, vec):
        return self.x * vec.x + self.y * vec.y

    def __iadd__(self, vec):
        self.x += vec.x
        self.y += vec.y
        return self

    def __isub__(self, vec):
        self.x -= vec.x
        self.y -= vec.y
        return self


class Point:
    def __init__(self, _x=0.0, _y=0.0):
        self.x = _x
        self.y = _y

    def __sub__(self, p):
        return Point(self.x - p.x, self.y - p.y)

    def __eq__(self, p):
        return self.x == p.x and self.y == p.y

    def classify(self, pO, p1):
        p2 = self
        a = p1 - pO
        b = p2 - pO
        sa = a.x * b.y - b.x * a.y

        if sa > 0.0:
            return 1  # LEFT
        if sa < 0.0:
            return 2  # RIGHT
        if (a.x * b.x < 0.0) or (a.y * b.y < 0.0):
            return 3  # BEHIND
        if (a.x**2 + a.y**2) < (b.x**2 + b.y**2):
            return 4  # BEYOND
        if pO == p2:
            return 5  # ORIGIN
        if p1 == p2:
            return 6  # DESTINATION
        return 7  # BETWEEN
    
    