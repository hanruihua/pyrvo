import math
import pyorca
import irsim

def v_add(a, b):
    return (a[0] + b[0], a[1] + b[1])


def v_sub(a, b):
    return (a[0] - b[0], a[1] - b[1])


def v_neg(a):
    return (-a[0], -a[1])


def v_abs_sq(a):
    return a[0] * a[0] + a[1] * a[1]


def v_norm(a):
    return math.sqrt(v_abs_sq(a))


def v_normalize(a):
    n = v_norm(a)
    if n == 0.0:
        return (0.0, 0.0)
    return (a[0] / n, a[1] / n)


def set_preferred_velocities(sim, goals):
    for j in range(sim.get_num_agents()):
        pos = sim.get_agent_position(j).to_tuple()
        goal_vec = v_sub(goals[j], pos)
        if v_abs_sq(goal_vec) > 1.0:
            goal_vec = v_normalize(goal_vec)
        sim.set_agent_pref_velocity(j, goal_vec)


def reached_goal(sim, goals):
    for j in range(sim.get_num_agents()):
        pos = sim.get_agent_position(j).to_tuple()
        if v_abs_sq(v_sub(pos, goals[j])) > sim.get_agent_radius(j) * sim.get_agent_radius(j):
            return False
    return True


if __name__ == "__main__":
    sim = pyorca.RVOSimulator()
    sim.set_time_step(0.25)
    sim.set_agent_defaults(15.0, 10, 10.0, 10.0, 1.5, 2.0)

    env = irsim.make()
    




    goals = []
    two_pi = 2.0 * math.pi
    for i in range(250):
        angle = i * two_pi * 0.004
        pos = (200.0 * math.cos(angle), 200.0 * math.sin(angle))
        sim.add_agent(pos)
        goals.append(v_neg(sim.get_agent_position(i).to_tuple()))

    while True:
        set_preferred_velocities(sim, goals)
        sim.do_step()
        if reached_goal(sim, goals):
            break
        print(sim.get_global_time())
        print(sim.get_agent_position(0).to_tuple())