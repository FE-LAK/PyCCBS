from structs import *

class Heuristic:
    def __init__(self, nodes, agents) -> None:
        self.h_values = []
        for n in nodes:
            self.h_values.append([-1] * len(agents))

    def count(self, map, agent : Agent):
        # Use Dijskstra to find accurate heuristic values
        curNode = Node(agent.goal_id, 0, 0)
        open = [curNode]

        print(f"Dijskstra for agent {agent.id}")
        while open:
            # Sort the open list
            open.sort(key=lambda x: x.g)
            curNode = open.pop(0)

            self.h_values[curNode.id][agent.id] = curNode.g

            for move in map.get_valid_moves(curNode.id, True):
                newNode = Node(move.id, 0, curNode.g + map.get_dist_id(move.id, curNode.id))
                if self.h_values[move.id][agent.id] < 0:
                    exNode = [x for x in open if x.id == newNode.id]
                    if exNode:
                        if exNode[0].g <= newNode.g:
                            continue
                        open.remove(exNode[0])
                    open.append(newNode)

    def get_dist(self, agent_id: int, source_node_id: int):
        return self.h_values[source_node_id][agent_id]