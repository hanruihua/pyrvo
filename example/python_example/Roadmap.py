import math
import random
from dataclasses import dataclass, field
from typing import List, Tuple

import pyrvo


Vec = Tuple[float, float]


def v_add(a: Vec, b: Vec) -> Vec:
    return (a[0] + b[0], a[1] + b[1])


def v_sub(a: Vec, b: Vec) -> Vec:
    return (a[0] - b[0], a[1] - b[1])


def v_abs(a: Vec) -> float:
    return math.sqrt(a[0] * a[0] + a[1] * a[1])


def v_abs_sq(a: Vec) -> float:
    return a[0] * a[0] + a[1] * a[1]


def v_normalize(a: Vec) -> Vec:
    n = v_abs(a)
    if n == 0.0:
        return (0.0, 0.0)
    return (a[0] / n, a[1] / n)


@dataclass
class RoadmapVertex:
    position: Vec
    neighbors: List[int] = field(default_factory=list)
    dist_to_goal: List[float] = field(default_factory=list)


def setup_scenario(sim: pyrvo.RVOSimulator,
                   roadmap: List[RoadmapVertex],
                   goals: List[int]) -> None:
    random.seed()

    sim.set_time_step(0.25)

    # Obstacles
    obstacle1 = [(-10.0, 40.0), (-40.0, 40.0), (-40.0, 10.0), (-10.0, 10.0)]
    obstacle2 = [(10.0, 40.0), (10.0, 10.0), (40.0, 10.0), (40.0, 40.0)]
    obstacle3 = [(10.0, -40.0), (40.0, -40.0), (40.0, -10.0), (10.0, -10.0)]
    obstacle4 = [(-10.0, -40.0), (-10.0, -10.0), (-40.0, -10.0), (-40.0, -40.0)]
    sim.add_obstacle(obstacle1)
    sim.add_obstacle(obstacle2)
    sim.add_obstacle(obstacle3)
    sim.add_obstacle(obstacle4)
    sim.process_obstacles()

    # Roadmap vertices: first four are goals
    for p in [(-75.0, -75.0), (75.0, -75.0), (-75.0, 75.0), (75.0, 75.0)]:
        roadmap.append(RoadmapVertex(position=p))

    for p in [(-42.0, -42.0), (-42.0, -8.0), (-42.0, 8.0), (-42.0, 42.0),
              (-8.0, -42.0), (-8.0, -8.0), (-8.0, 8.0), (-8.0, 42.0),
              (8.0, -42.0), (8.0, -8.0), (8.0, 8.0), (8.0, 42.0),
              (42.0, -42.0), (42.0, -8.0), (42.0, 8.0), (42.0, 42.0)]:
        roadmap.append(RoadmapVertex(position=p))

    sim.set_agent_defaults(15.0, 10, 5.0, 5.0, 2.0, 2.0)

    # Four groups; goals are indices of first four roadmap vertices
    for i in range(5):
        for j in range(5):
            sim.add_agent((55.0 + i * 10.0, 55.0 + j * 10.0))
            goals.append(0)
            sim.add_agent((-55.0 - i * 10.0, 55.0 + j * 10.0))
            goals.append(1)
            sim.add_agent((55.0 + i * 10.0, -55.0 - j * 10.0))
            goals.append(2)
            sim.add_agent((-55.0 - i * 10.0, -55.0 - j * 10.0))
            goals.append(3)


def build_roadmap(sim: pyrvo.RVOSimulator, roadmap: List[RoadmapVertex]) -> None:
    # Visibility-based neighbors
    for i in range(len(roadmap)):
        for j in range(len(roadmap)):
            if sim.query_visibility(roadmap[i].position, roadmap[j].position, sim.get_agent_radius(0)):
                roadmap[i].neighbors.append(j)
        roadmap[i].dist_to_goal = [float("inf")] * 4

    # Dijkstra from each goal
    for goal_idx in range(4):
        import heapq
        dist = [float("inf")] * len(roadmap)
        dist[goal_idx] = 0.0
        pq = [(0.0, goal_idx)]
        while pq:
            d_u, u = heapq.heappop(pq)
            if d_u != dist[u]:
                continue
            for v in roadmap[u].neighbors:
                w = v_abs((roadmap[v].position[0] - roadmap[u].position[0],
                           roadmap[v].position[1] - roadmap[u].position[1]))
                if dist[v] > d_u + w:
                    dist[v] = d_u + w
                    heapq.heappush(pq, (dist[v], v))
        for i in range(len(roadmap)):
            roadmap[i].dist_to_goal[goal_idx] = dist[i]


def set_preferred_velocities(sim: pyrvo.RVOSimulator,
                              roadmap: List[RoadmapVertex],
                              goals: List[int]) -> None:
    for i in range(sim.get_num_agents()):
        pos = sim.get_agent_position(i).to_tuple()
        min_dist = float("inf")
        min_vertex = -1
        for j, v in enumerate(roadmap):
            d = v_abs((v.position[0] - pos[0], v.position[1] - pos[1])) + v.dist_to_goal[goals[i]]
            if d < min_dist and sim.query_visibility(pos, v.position, sim.get_agent_radius(i)):
                min_dist = d
                min_vertex = j

        if min_vertex == -1:
            pref = (0.0, 0.0)
        else:
            target = roadmap[min_vertex].position
            if v_abs_sq((target[0] - pos[0], target[1] - pos[1])) == 0.0:
                if min_vertex == goals[i]:
                    pref = (0.0, 0.0)
                else:
                    goalp = roadmap[goals[i]].position
                    pref = v_normalize((goalp[0] - pos[0], goalp[1] - pos[1]))
            else:
                pref = v_normalize((target[0] - pos[0], target[1] - pos[1]))

        # Symmetry-breaking jitter
        angle = random.random() * (2.0 * math.pi)
        dist = random.random() * 0.0001
        cur = v_add(pref, (0.0, 0.0))
        jitter = (math.cos(angle), math.sin(angle))
        sim.set_agent_pref_velocity(i, v_add(cur, (jitter[0] * dist, jitter[1] * dist)))


def reached_goal(sim: pyrvo.RVOSimulator,
                 roadmap: List[RoadmapVertex],
                 goals: List[int]) -> bool:
    for i in range(sim.get_num_agents()):
        pos = sim.get_agent_position(i).to_tuple()
        goalp = roadmap[goals[i]].position
        if v_abs_sq((pos[0] - goalp[0], pos[1] - goalp[1])) > 400.0:
            return False
    return True


if __name__ == "__main__":
    sim = pyrvo.RVOSimulator()
    roadmap: List[RoadmapVertex] = []
    goals: List[int] = []
    setup_scenario(sim, roadmap, goals)
    build_roadmap(sim, roadmap)

    while True:
        # Print time and positions
        print(sim.get_global_time(), end="")
        for i in range(sim.get_num_agents()):
            print(" ", sim.get_agent_position(i).to_tuple(), end="")
        print()

        set_preferred_velocities(sim, roadmap, goals)
        sim.do_step()
        if reached_goal(sim, roadmap, goals):
            break

