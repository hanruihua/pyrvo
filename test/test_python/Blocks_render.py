import math
import random

import pyrvo
import irsim

def v_add(a, b):
    return (a[0] + b[0], a[1] + b[1])


def v_sub(a, b):
    return (a[0] - b[0], a[1] - b[1])


def v_scale(a, s):
    return (a[0] * s, a[1] * s)


def v_abs_sq(a):
    return a[0] * a[0] + a[1] * a[1]


def v_norm(a):
    return math.sqrt(v_abs_sq(a))


def v_normalize(a):
    n = v_norm(a)
    if n == 0.0:
        return (0.0, 0.0)
    return (a[0] / n, a[1] / n)


def setup_scenario(sim):
    random.seed()

    # Time step and defaults (match Blocks.cc)
    sim.set_time_step(0.25)
    sim.set_agent_defaults(15.0, 10, 5.0, 5.0, 2.0, 2.0)

    goals = []

    # Four groups of agents on 5x5 grids
    for i in range(5):
        for j in range(5):
            sim.add_agent((55.0 + i * 10.0, 55.0 + j * 10.0))
            goals.append((-75.0, -75.0))

            sim.add_agent((-55.0 - i * 10.0, 55.0 + j * 10.0))
            goals.append((75.0, -75.0))

            sim.add_agent((55.0 + i * 10.0, -55.0 - j * 10.0))
            goals.append((-75.0, 75.0))

            sim.add_agent((-55.0 - i * 10.0, -55.0 - j * 10.0))
            goals.append((75.0, 75.0))

    # Obstacles (rectangles) specified CCW
    obstacle1 = [(-10.0, 40.0), (-40.0, 40.0), (-40.0, 10.0), (-10.0, 10.0)]
    obstacle2 = [(10.0, 40.0), (10.0, 10.0), (40.0, 10.0), (40.0, 40.0)]
    obstacle3 = [(10.0, -40.0), (40.0, -40.0), (40.0, -10.0), (10.0, -10.0)]
    obstacle4 = [(-10.0, -40.0), (-10.0, -10.0), (-40.0, -10.0), (-40.0, -40.0)]

    sim.add_obstacle(obstacle1)
    sim.add_obstacle(obstacle2)
    sim.add_obstacle(obstacle3)
    sim.add_obstacle(obstacle4)
    sim.process_obstacles()

    return goals


def set_preferred_velocities(sim, goals):
    for i in range(sim.get_num_agents()):
        pos = sim.get_agent_position(i).to_tuple()
        goal_vec = v_sub(goals[i], pos)
        if v_abs_sq(goal_vec) > 1.0:
            goal_vec = v_normalize(goal_vec)
        sim.set_agent_pref_velocity(i, goal_vec)

        # Small random perturbation to avoid symmetry deadlocks
        angle = random.random() * (2.0 * math.pi)
        dist = random.random() * 0.0001
        cur = sim.get_agent_pref_velocity(i).to_tuple()
        jitter = (math.cos(angle), math.sin(angle))
        sim.set_agent_pref_velocity(i, v_add(cur, v_scale(jitter, dist)))


def reached_goal(sim, goals):
    for i in range(sim.get_num_agents()):
        pos = sim.get_agent_position(i).to_tuple()
        if v_abs_sq(v_sub(pos, goals[i])) > 400.0:
            return False
    return True


if __name__ == "__main__":
    sim = pyrvo.RVOSimulator()
    goals = setup_scenario(sim)

    env = irsim.make()

    while True:
        # Print time and positions (like C++ demo)
        print(sim.get_global_time(), end="")
        for i in range(sim.get_num_agents()):
            print(" ", sim.get_agent_position(i).to_tuple(), end="")
        print()

        set_preferred_velocities(sim, goals)
        sim.do_step()

        for i in range(env.robot_number):
            env.robot_list[i].set_state([sim.get_agent_position(i).to_tuple()[0], sim.get_agent_position(i).to_tuple()[1], 0])
            env.robot_list[i].set_goal([goals[i][0], goals[i][1]])

        env.render()

        if reached_goal(sim, goals):
            break

