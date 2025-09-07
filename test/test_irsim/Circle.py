import math
import pyrvo
import irsim
import numpy as np

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

def set_preferred_velocities_irsim(env, goals, sim):
    for i in range(env.robot_number):
        pos = [env.robot_list[i].state[0, 0], env.robot_list[i].state[1, 0]]
        goal_vec = v_sub(goals[i], pos)
        if v_abs_sq(goal_vec) > 1.0:
            goal_vec = v_normalize(goal_vec)
        sim.set_agent_pref_velocity(i, goal_vec)


def set_positions_irsim(env, sim):

    for i in range(env.robot_number):
        pos = [env.robot_list[i].state[0, 0], env.robot_list[i].state[1, 0]]
        sim.set_agent_position(i, pos)

def reached_goal(sim, goals):
    for j in range(sim.get_num_agents()):
        pos = sim.get_agent_position(j).to_tuple()
        if v_abs_sq(v_sub(pos, goals[j])) > sim.get_agent_radius(j) * sim.get_agent_radius(j):
            return False
    return True


if __name__ == "__main__":
    sim = pyrvo.RVOSimulator()
    
    env = irsim.make(save_ani=True)

    sim.set_time_step(env.step_time)
    
    sim.set_agent_defaults(15.0, 10, 20.0, 10.0, 1.5, 2.0)
    # float neighborDist, std::size_t maxNeighbors,float timeHorizon, float timeHorizonObst, float radius, float maxSpeed

    goals = []
    for i in range(env.robot_number):
        # angle = i * two_pi * 0.004
        # pos = (200.0 * math.cos(angle), 200.0 * math.sin(angle))
        pos = [env.robot_list[i].state[0, 0], env.robot_list[i].state[1, 0]]
        sim.add_agent(pos)

        goal_list = env.robot_list[i].goal.flatten().tolist()
        goals.append(goal_list)
        
    while True:
        # set_preferred_velocities(sim, goals)

        set_preferred_velocities_irsim(env, goals, sim)
        set_positions_irsim(env, sim)
        sim.do_step()

        action_list = []
        for i in range(sim.get_num_agents()):
            a = np.array(sim.get_agent_velocity(i).to_tuple()).reshape(2, 1)

            action_list.append(a)

        env.step(action_list)
        env.render()

        if reached_goal(sim, goals):
            break
    
    env.end()
        # print(sim.get_global_time())
        # print(sim.get_agent_position(0).to_tuple())