# PyCCBS
Continuous Conflict-Based Search (CCBS) implementation in Python

Ported from C++ implementation, available at [https://github.com/PathPlanning/Continuous-CBS](https://github.com/PathPlanning/Continuous-CBS)

## Content

Python port resembles the file structure of the original repository.

- ccbs.py: implementation of the CCBS algorithm
- config.py: configuration of the CCBS
- map.py: object for dealing with maps
- sipp.py: implementation of SIPP algorithm
- structs.py: helping object structures for SIPP and CCBS
- test map *map_ccbs.export.xml*, task list *task_ccbs_export.xml*

## Use

Both SIPP and CCBS can be used directly. SIPP can be used separately.

Map loading code uses libraries `networkx` and `matplotlib`.

The following example loads the map from .xml, adds some example constraints (both positive and negative), then plans the path for one agent
```
    map = Map("map_ccbs_export.xml")

    planner = SIPP(map)
    planner.verbose = True # Opional

    # Add constraints
    cs = []
    cs.append(Constraint(0, 250, 260, 240, 547, True)) # Must traverse 240-547 at t = 250-260
    cs.append(Constraint(1, 40, 160, 240, 240, False)) # Collision constraint by agent 1 on node 240 at t = 40-160

    planner.find_path(Agent(35, 85, 0), cs) # Plan path between nodes 35 and 85 for agent 0
```

CCBS is a multi-agent optimal planner
```
    map = Map("map_ccbs_export.xml")

    ccbs = CCBS(map)

    # Create task
    task = Task()

    # Load tasks
    task.agents.append(Agent(35, 85, 0))
    task.agents.append(Agent(161, 113, 1))
    task.agents.append(Agent(105, 19, 2))
    task.agents.append(Agent(73, 69, 3))
    task.agents.append(Agent(201, 179, 4))
    
    # Run CCBS
    solution = ccbs.find_solution(task)    

    # Render the map graphically
    map.render(solution.paths)

    # Save solution to file
    ccbs.write_to_log_path('test_out.xml')
```

Tasks can also be loaded from file directly
```
    task.load_from_file("task_ccbs_export.xml") 
```

## Changes from original C++ implementation:
- x,y based map
- no simplex-based optimization supported for HL-heuristic
- additional distance trigger for conflicts checking in function check_paths in ccbs.py (check for nodes distance at end of motion interval)

