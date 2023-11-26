from structs import *
from map import *
import math

class SIPP:
    CN_INFINITY = float('inf')
    CN_EPSILON = 1e-9

    def __init__(self, map : Map):
        self.open = []
        self.close = {}
        self.collision_intervals = {} # dict: id             -> list(list(start,end))
        self.landmarks = []
        self.constraints = {}         # dict: tuple(id1,id2) -> list(Move)
        self.visited = {}             # dict: tuple(id1,id2) -> list(double, bool)        
        self.agent = None
        self.map = map
        self.verbose = False

    def clear(self):
        self.open.clear()
        self.close.clear()
        self.collision_intervals.clear()
        self.landmarks.clear()
        self.constraints.clear()
        self.visited.clear()        

    def dist(self, a, b):
        return math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2)

    def find_successors(self, cur_node : Node, goal : Node):        
        valid_moves = self.map.get_valid_moves(cur_node.id)
        succs = []

        for move in valid_moves:
            new_node = Node(move.id, _x = move.x, _y = move.y)

            cost = self.map.get_dist(cur_node, new_node)
            new_node.g = cur_node.g + cost

            # Free intervals
            intervals = []

            colls_it = self.collision_intervals.get(new_node.id, [])
            if colls_it:
                # Convert collision intervals into free intervals
                interval = [0, self.CN_INFINITY]
                for coll_single in colls_it:
                    interval[1] = coll_single[0]
                    intervals.append(interval)
                    interval[0] = coll_single[1]
                interval[1] = self.CN_INFINITY
                intervals.append(interval)
            else:
                intervals.append([0, self.CN_INFINITY])
            
            # Get the list of constraints for move to new_node
            cons_it = self.constraints.get((cur_node.id, new_node.id), [])
            id = 0
            for interval in intervals:
                new_node.interval_id = id
                id += 1

                # Has the new node been visited in this interval?
                new_visited_int = (new_node.id, new_node.interval_id)

                it = self.visited.get(new_visited_int)
                if it and it[1]: # if existing and visited...
                    continue
                if interval[1] < new_node.g: # Interval ended before arrival to new_node
                    continue
                if interval[0] > new_node.g: # Interval starts after arrival to new_node
                    new_node.g = interval[0]  # ?? Just set the g to the free interval start?
                
                # Check the constraints
                if cons_it:
                    for new_cons in cons_it:      
                        # If any constraint is valid, apply it              
                        if new_node.g - cost + self.CN_EPSILON > new_cons.t1 and new_node.g - cost < new_cons.t2:
                            new_node.g = new_cons.t2 + cost
                new_node.interval = interval
                if new_node.g - cost > cur_node.interval[1] or new_node.g > new_node.interval[1]:
                    # Move out of valid free interval
                    continue
                
                if it:
                    if it[0] - self.CN_EPSILON < new_node.g:
                        continue
                    else:                        
                        self.visited[new_visited_int][0] = new_node.g
                else:
                    self.visited[new_visited_int] = [new_node.g, False]

                new_node.f = new_node.g + self.map.get_dist(new_node, goal)
                succs.append(new_node)
        return succs
    
    def find_min(self):
        self.open.sort(key=lambda x: x.f)
        return self.open.pop(0)

    def add_open(self, new_node):
        if not self.open or self.open[-1].f - self.CN_EPSILON < new_node.f:
            self.open.append(new_node)
            return
        
        for i, node in enumerate(self.open):
            if node.f > new_node.f + self.CN_EPSILON:  # if new_node.f has a lower f-value
                self.open.append(new_node)
                return
            elif abs(node.f - new_node.f) < self.CN_EPSILON and new_node.g + self.CN_EPSILON > node.g:  # if f-values are equal, compare g-values
                self.open.append(new_node)
                return
        self.open.append(new_node)

    def reconstruct_path(self, cur_node):
        # Reconstruct SIPP path
        nodes = []
        while cur_node.parent:
            nodes.insert(0, cur_node)
            cur_node = cur_node.parent
        nodes.insert(0, cur_node)
        for i in range(len(nodes) - 1):
            j = i + 1
            if abs(nodes[j].g - nodes[i].g - self.map.get_dist(nodes[j], nodes[i])) > self.CN_EPSILON:
                add = nodes[i].clone()
                add.g = nodes[j].g - self.map.get_dist(nodes[j], nodes[i])
                nodes.insert(j, add)
        return nodes
    
    def add_collision_interval(self, id, interval):
        intervals = []
        if id not in self.collision_intervals:
            # Add it to the dict
            self.collision_intervals[id] = [interval]
        else:
            # Add the interval to the list in the dict
            self.collision_intervals[id].append(interval)        
        
        # Filter the intervals
        i = 0
        self.collision_intervals[id].sort(key=lambda x: x[0])
        while i + 1 < len(self.collision_intervals[id]):
            if self.collision_intervals[id][i][1] + self.CN_EPSILON > self.collision_intervals[id][i + 1][0]:
                # Join the intervals i and j
                self.collision_intervals[id][i][1] = self.collision_intervals[id][i + 1][1]
                self.collision_intervals[id].pop(i + 1)
                i -= 1
            i += 1

    def add_move_constraint(self, move):
        m_cons = []
        if (move.id1, move.id2) not in self.constraints:
            # No existing constraint present, create new list
            self.constraints[(move.id1, move.id2)] = [move]
        else:
            # Check the existing constraints before adding the new move            
            m_cons = self.constraints[(move.id1, move.id2)]
            inserted = False
            for i in range(len(m_cons)):
                if inserted:
                    break
                if m_cons[i].t1 > move.t1: # Move starts before the existing constraint
                    if m_cons[i].t1 < move.t2 + self.CN_EPSILON: # Move ends after existing constraint
                        m_cons[i].t1 = move.t1 # Change the constraint to start with the new move start
                        if move.t2 + self.CN_EPSILON > m_cons[i].t2:
                            # Extend the constraint to cover the new move
                            m_cons[i].t2 = move.t2                            
                        inserted = True

                        # Not the first one?
                        if i != 0:
                            # Previous constraint ended after the move start?
                            if m_cons[i - 1].t2 + self.CN_EPSILON > move.t1 and m_cons[i - 1].t2 < move.t2 + self.CN_EPSILON:
                                m_cons[i - 1].t2 = move.t2
                                if m_cons[i - 1].t2 + self.CN_EPSILON > m_cons[i].t1 and m_cons[i - 1].t2 < m_cons[i].t2 + self.CN_EPSILON:
                                    m_cons[i - 1].t2 = m_cons[i].t2
                                    m_cons.pop(i) # We don't need constraint i anymore...
                                inserted = True
                    else:
                        # Not the first one?
                        if i != 0:
                            # Previous constraint ended after the move start?
                            if m_cons[i - 1].t2 + self.CN_EPSILON > move.t1 and m_cons[i - 1].t2 < move.t2 + self.CN_EPSILON:
                                m_cons[i - 1].t2 = move.t2
                                inserted = True
                                break
                        m_cons.insert(i, move)
                        inserted = True
                i += 1

            # Last constraint ends after move start, but move ends later
            if m_cons[-1].t2 + self.CN_EPSILON > move.t1 and m_cons[-1].t2 < move.t2 + self.CN_EPSILON:
                m_cons[-1].t2 = move.t2 # Adjust the constraint
            elif not inserted:
                m_cons.append(move)
            self.constraints[(move.id1, move.id2)] = m_cons

    def make_constraints(self, cons):
        if cons is None:
            return
        
        # Prepare constraints
        for con in cons:
            if not con.positive:
                if con.id1 == con.id2:  # wait constraint
                    self.add_collision_interval(con.id1, [con.t1, con.t2])
                else:
                    self.add_move_constraint(Move(con.t1, con.t2, con.id1, con.id2))
            else:
                inserted = False
                for i in range(len(self.landmarks)):
                    if self.landmarks[i].t1 > con.t1:
                        self.landmarks.insert(i, Move(con.t1, con.t2, con.id1, con.id2))
                        inserted = True
                        break
                if not inserted:
                    self.landmarks.append(Move(con.t1, con.t2, con.id1, con.id2))

    # Combine result and part paths (excluding element 0 of part)
    def add_part(self, result, part):
        part.nodes.pop(0)
        result.nodes.extend(part.nodes)
        return result

    # Find partial path - basic SIPP, account for multiple start and goal intervals
    # starts: list(Node)
    # goals: list(Node)
    # Return: list(Path)
    def find_partial_path(self, starts, goals, max_f = float('inf')):
        self.open.clear()
        self.close.clear()        
        self.visited.clear()

        #print(f"** SIPP: {starts}->{goals} @ max_f={max_f}")

        # Construct a path for each goal
        paths = [Path() for _ in range(len(goals))]

        path_found = 0
        for s in starts:
            s.parent = None
            self.open.append(s)
            self.visited[(s.id, s.interval_id)] = [s.g, False]

        cur_node = None
        while self.open:
            cur_node = self.find_min()
            #print(f" - {cur_node}")
            v = self.visited[(cur_node.id, cur_node.interval_id)]
            if v[1]:
                # Already visited, skip
                continue
            self.visited[(cur_node.id, cur_node.interval_id)][1] = True

            # Add it to the closed list
            parent = self.close[(cur_node.id, cur_node.interval_id)] = cur_node

            # Current node is goal node (all goal nodes have the same id)
            if cur_node.id == goals[0].id:
                for i in range(len(goals)):
                    if cur_node.g - self.CN_EPSILON < goals[i].interval[1] and goals[i].interval[0] - self.CN_EPSILON < cur_node.interval[1]:
                        # Viable path found
                        paths[i].nodes = self.reconstruct_path(cur_node)

                        if paths[i].nodes[-1].g < goals[i].interval[0]:      
                            cur_node = cur_node.clone()                            
                            cur_node.g = goals[i].interval[0]
                            paths[i].nodes.append(cur_node)                            
                        paths[i].cost = cur_node.g
                        paths[i].expanded = len(self.close)
                        path_found += 1
                    if path_found == len(goals):
                        # Finish the search
                        if self.verbose:
                            print("** SIPP found solution")
                        return paths
                                
            # Find successors
            succs = self.find_successors(cur_node, Node(goals[0].id, 0, 0, goals[0].x, goals[0].y))
            for new_node in succs:
                if new_node.f > max_f:
                    # Invalid move
                    continue
                new_node.parent = parent
                self.add_open(new_node)

        if self.verbose:
            print("** Error: SIPP found no solution")
        return paths
    
    # Get the enpoints
    def get_endpoints(self, node_id, t1, t2):
        node_x, node_y = self.map.nodes[node_id].x, self.map.nodes[node_id].y
        nodes = [Node(node_id, 0, 0, node_x, node_y, None, t1, t2)]

        if not self.collision_intervals.get(node_id):
            # No collision interval involved for the specified node
            return nodes
        else:
            k = 0
            while k < len(self.collision_intervals[node_id]):
                i = 0
                while i < len(nodes):
                    n = nodes[i]
                    c = self.collision_intervals[node_id][k]
                    changed = False

                    # Start not possible - whole starting interval blocked
                    if c[0] - self.CN_EPSILON < n.interval[0] and c[1] + self.CN_EPSILON > n.interval[1]:
                        nodes.pop(i)
                        changed = True

                    # Only the beginning of starting interval blocked, adjust it
                    elif c[0] - self.CN_EPSILON < n.interval[0] and c[1] > n.interval[0]:                        
                        nodes[i].interval[0] = c[1]
                        changed = True

                    # Beginning free, gets partially blocked -> create additional interval                    
                    elif c[0] - self.CN_EPSILON > n.interval[0] and c[1] + self.CN_EPSILON < n.interval[1]:
                        nodes[i].interval[1] = c[0]
                        nodes.insert(i + 1, Node(node_id, 0, 0, node_x, node_y, None, c[1], n.interval[1]))
                        changed = True

                    # End blocked, adjust it
                    elif c[0] < n.interval[1] and c[1] + self.CN_EPSILON > n.interval[1]:
                        nodes[i].interval[1] = c[0]
                        changed = True

                    # If changed, recheck it with other intervals again
                    if changed:
                        i = -1
                        k = 0

                    i += 1
                k += 1
        return nodes
    
    def check_endpoint(self, start, goal):
        cost = self.map.get_dist(start, goal)

        if start.g + cost < goal.interval[0]:
            start.g = goal.interval[0] - cost

        if (start.id, goal.id) in self.constraints:
            for move in self.constraints[(start.id, goal.id)]:
                if start.g + self.CN_EPSILON > move.t1 and start.g < move.t2:
                    start.g = move.t2

        if start.g > start.interval[1] or start.g + cost > goal.interval[1]:
            return self.CN_INFINITY
        else:
            return start.g + cost
    
    # SIPP find path
    def find_path(self, agent, cons):
        self.clear()
        self.agent = agent
        self.make_constraints(cons)

        starts, goals = [], []
        parts = []
        results = []
        new_results = []
        expanded = 0        

        # No existing set of landmarks (positive constraints) -> plain SIPP from start to goal
        if not self.landmarks:            
            starts = [self.get_endpoints(agent.start_id, 0, self.CN_INFINITY)[0]]
            goals = [self.get_endpoints(agent.goal_id, 0, self.CN_INFINITY)[-1]]
            
            if self.verbose:
                print(f"SIPP for [{agent.id}]: {agent.start_id} -> {agent.goal_id}  | Constraints: {self.constraints}")
            
            parts = self.find_partial_path(starts, goals)            
            expanded = len(self.close)

            if parts[0].cost < 0:
                return Path()

            result = parts[0]

        else:
            if self.verbose:
                print(f"SIPP for [{agent.id}]: {agent.start_id} -> {[f'{m.id1}-{m.id2}' for m in self.landmarks]} -> {agent.goal_id}   | Constraints: {self.constraints}")

            for i in range(len(self.landmarks) + 1):
                if i == 0:
                    starts = [self.get_endpoints(agent.start_id, 0, self.CN_INFINITY)[0]]
                    goals = self.get_endpoints(self.landmarks[i].id1, self.landmarks[i].t1, self.landmarks[i].t2)
                else:
                    starts = [p.nodes[-1] for p in results] # Starts at previous goals (landmarks)
                    if i == len(self.landmarks):
                        goals = [self.get_endpoints(agent.goal_id, 0, self.CN_INFINITY)[-1]]
                    else:
                        goals = self.get_endpoints(self.landmarks[i].id1, self.landmarks[i].t1, self.landmarks[i].t2)

                # Goal empty - no path possible
                if not goals:
                    if self.verbose:
                        print("No possible goal found")
                    return Path()
                
                parts = self.find_partial_path(starts, goals, goals[-1].interval[1])

                expanded += len(self.close)
                new_results = []                
                
                if i == 0:
                    for k in range(len(parts)):
                        if parts[k].nodes:
                            new_results.append(parts[k])

                for k in range(len(parts)):
                    for j in range(len(results)):
                        if parts[k].nodes and \
                                abs(parts[k].nodes[0].interval[0] - results[j].nodes[-1].interval[0]) < self.CN_EPSILON and \
                                ((math.isinf(parts[k].nodes[0].interval[1]) and math.isinf(results[j].nodes[-1].interval[1])) or \
                                 (abs(parts[k].nodes[0].interval[1] - results[j].nodes[-1].interval[1]) < self.CN_EPSILON)):
                            
                            # If segments match in time, add it as a solution
                            new_results.append(self.add_part(results[j], parts[k]))                            

                results = new_results                

    	        # No results - no path...
                if not results:
                    if self.verbose:
                        print("No path result")
                    return Path()                

                # Finish in a landmark
                if i < len(self.landmarks):
                    starts = [p.nodes[-1] for p in results]
                    offset = self.map.get_dist_id(self.landmarks[i].id1, self.landmarks[i].id2)

                    goals = self.get_endpoints(self.landmarks[i].id2, self.landmarks[i].t1 + offset, self.landmarks[i].t2 + offset)

                    # No goals - no path
                    if not goals:
                        #print("No viable goal found")
                        return Path()

                    new_results = []
                    
                    for k in range(len(goals)):
                        best_g = self.CN_INFINITY
                        best_start_id = -1

                        for j in range(len(starts)):
                            g = self.check_endpoint(starts[j], goals[k])

                            if g < best_g:
                                best_start_id = j
                                best_g = g

                        if best_start_id >= 0:
                            goals[k].g = best_g
                            
                            if not goals[k].id in self.collision_intervals:
                                goals[k].interval[1] = self.CN_INFINITY
                            else:
                                for c in self.collision_intervals[goals[k].id]:
                                    if goals[k].g < c[0]:
                                        goals[k].interval[1] = c[0]
                                        break

                            new_results.append(results[best_start_id])

                            if goals[k].g - starts[best_start_id].g > offset + self.CN_EPSILON:
                                new_results[-1].nodes.append(new_results[-1].nodes[-1])
                                new_results[-1].nodes[-1].g = goals[k].g - offset

                            new_results[-1].nodes.append(goals[k])

                    results = new_results
                    
                    if not results:
                        if self.verbose:
                            print("No resulting path found")
                        return Path()

            result = results[0]
        
        result.cost = result.nodes[-1].g
        result.agentID = agent.id
        result.expanded = expanded

        if self.verbose:
            print(f"Final result: {result}")
            print(f"Total cost: {result.cost}")
            print(f"Expanded nodes: {result.expanded}")

        return result
    
# Test the SIPP
if __name__ == "__main__":
    # Load the map
    print("Loading map...")
    map = Map("map_ccbs_export.xml")

    planner = SIPP(map)
    planner.verbose = True

    # Add constraints
    cs = []
    cs.append(Constraint(0, 250, 260, 240, 547, True)) # Must traverse 240-547 at t = 250-260
    cs.append(Constraint(1, 40, 160, 240, 240, False)) # Collision constraint by agent 1 on node 240 at t = 40-160

    planner.find_path(Agent(35, 85, 0), cs) # Plan path between nodes 35 and 85 for agent 0