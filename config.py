class Config:
    def __init__(self) -> None:
        self.use_cardinal = True
        self.agent_size = 0.35
        self.hlh_type = 2  # 0 - no hlh, 1 - solve lpp by simplex, 2 - greedly take disjoint conflicts

        self.precision = 0.1
        self.use_disjoint_splitting = True

        self.timelimit = 30