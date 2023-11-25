
class heuristic:
    def __init__(self, nodes, agents) -> None:
        self.h_values = [[-1] * agents] * nodes

    def count(self, map, agent):
        pass


if __name__ == "__main__":
    h = heuristic(5, 2)

    print(h.h_values)