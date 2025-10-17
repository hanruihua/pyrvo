import pyrvo
import irsim


if __name__ == "__main__":
    rvo = pyrvo.RVOSimulator()
    
    env = irsim.make()

    rvo.set_time_step(env.step_time)
    rvo.set_agent_defaults(15.0, 10, 20.0, 10.0, 1.5, 2.0)

    for _i, robot in enumerate(env.robot_list):
        rvo.add_agent(robot.state[:2, 0].tolist())
            
    while True:
        
        for i, robot in enumerate(env.robot_list):
            rvo.set_agent_pref_velocity(i, robot.get_desired_omni_vel(normalized=True).flatten().tolist())
            rvo.set_agent_position(i, robot.state[:2, 0].tolist())

        rvo.do_step()
        action_list = [rvo.get_agent_velocity(i).to_tuple() for i in range(rvo.get_num_agents())]

        env.step(action_list)
        env.render()

        if env.done():
            break
    
    env.end()