import pyrvo
import irsim
if __name__ == "__main__":
    sim = pyrvo.RVOSimulator()
    
    env = irsim.make()

    sim.set_time_step(env.step_time)
    sim.set_agent_defaults(15.0, 10, 20.0, 10.0, 1.5, 2.0)

    for _i, robot in enumerate(env.robot_list):
        sim.add_agent(robot.state[:2, 0].tolist())
            
    while True:
        
        for i, robot in enumerate(env.robot_list):
            sim.set_agent_pref_velocity(i, robot.get_desired_omni_vel(normalized=True).flatten().tolist())
            sim.set_agent_position(i, robot.state[:2, 0].tolist())

        sim.do_step()
        action_list = [sim.get_agent_velocity(i).to_tuple() for i in range(sim.get_num_agents())]

        env.step(action_list)
        env.render()

        if env.done():
            break
    
    env.end()