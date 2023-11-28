class Config:
    def __init__(self) -> None:
        self.use_cardinal = True    # Prioritize cardinal over semicardinal over regular conflicts
        self.agent_size = 0.35      # Agent size in world units
        self.hlh_type = 2           # 0 - no hlh, 1 - solve lpp by simplex, 2 - greedly take disjoint conflicts
        self.precision = 0.1        # Precision for waiting time determination
        self.use_disjoint_splitting = True

        self.timelimit = 30         # Time limit in seconds

        self.use_precalculated_heuristic = False # True: reverse Dijskstra, False: Euclidean distance