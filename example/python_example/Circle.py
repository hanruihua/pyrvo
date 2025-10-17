import math
import pyrvo


def v_sub(a, b):
    return (a[0] - b[0], a[1] - b[1])


def v_abs_sq(a):
    return a[0] * a[0] + a[1] * a[1]


def v_normalize(a):
    n2 = v_abs_sq(a)
    if n2 == 0.0:
        return (0.0, 0.0)
    n = math.sqrt(n2)
    return (a[0] / n, a[1] / n)


def setup_scenario(sim):
    sim.set_time_step(0.25)
    sim.set_agent_defaults(15.0, 10, 10.0, 10.0, 1.5, 2.0)

    goals = []
    two_pi = 2.0 * math.pi
    for i in range(250):
        angle = i * two_pi * 0.004
        pos = (200.0 * math.cos(angle), 200.0 * math.sin(angle))
        sim.add_agent(pos)
        # Goal is antipodal
        goals.append((-pos[0], -pos[1]))
    return goals


def set_preferred_velocities(sim, goals):
    for i in range(sim.get_num_agents()):
        pos = sim.get_agent_position(i).to_tuple()
        goal_vec = v_sub(goals[i], pos)
        if v_abs_sq(goal_vec) > 1.0:
            goal_vec = v_normalize(goal_vec)
        sim.set_agent_pref_velocity(i, goal_vec)


def reached_goal(sim, goals):
    for i in range(sim.get_num_agents()):
        pos = sim.get_agent_position(i).to_tuple()
        if v_abs_sq(v_sub(pos, goals[i])) > sim.get_agent_radius(i) * sim.get_agent_radius(i):
            return False
    return True


if __name__ == "__main__":
    sim = pyrvo.RVOSimulator()
    goals = setup_scenario(sim)

    while True:
        print(sim.get_global_time())
        print(sim.get_agent_position(0).to_tuple())
        set_preferred_velocities(sim, goals)
        sim.do_step()
        if reached_goal(sim, goals):
            break

