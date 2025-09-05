import sys
import os
import math

# Add the build directory to Python path
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'build', 'python'))

import orca

def setup_scenario(sim):
    # Set up the simulation parameters
    sim.set_time_step(0.25)
    sim.set_agent_defaults(15.0, 10, 5.0, 5.0, 2.0, 2.0)
    
    # Add agents
    for i in range(5):
        for j in range(5):
            # Add agents in four corners
            sim.add_agent((55.0 + i * 10.0, 55.0 + j * 10.0))
            sim.add_agent((-55.0 - i * 10.0, 55.0 + j * 10.0))
            sim.add_agent((55.0 + i * 10.0, -55.0 - j * 10.0))
            sim.add_agent((-55.0 - i * 10.0, -55.0 - j * 10.0))
    
    # Add obstacles
    obstacles = [
        [(-10.0, 40.0), (-40.0, 40.0), (-40.0, 10.0), (-10.0, 10.0)],
        [(10.0, 40.0), (10.0, 10.0), (40.0, 10.0), (40.0, 40.0)],
        [(10.0, -40.0), (40.0, -40.0), (40.0, -10.0), (10.0, -10.0)],
        [(-10.0, -40.0), (-10.0, -10.0), (-40.0, -10.0), (-40.0, -40.0)]
    ]
    
    for obstacle in obstacles:
        sim.add_obstacle(obstacle)
    
    # Process obstacles
    sim.process_obstacles()

def set_preferred_velocities(sim, goals):
    for i in range(sim.get_num_agents()):
        pos = sim.get_agent_position(i)
        goal = goals[i]
        pref_vel = (goal[0] - pos[0], goal[1] - pos[1])
        
        # Normalize preferred velocity
        speed = math.sqrt(pref_vel[0] * pref_vel[0] + pref_vel[1] * pref_vel[1])
        if speed > 0.1:
            pref_vel = (pref_vel[0] / speed, pref_vel[1] / speed)
            sim.set_agent_pref_velocity(i, pref_vel)
        else:
            sim.set_agent_pref_velocity(i, (0.0, 0.0))

def reached_goal(sim, goals):
    for i in range(sim.get_num_agents()):
        pos = sim.get_agent_position(i)
        goal = goals[i]
        dist_sq = (pos[0] - goal[0]) * (pos[0] - goal[0]) + (pos[1] - goal[1]) * (pos[1] - goal[1])
        if dist_sq > sim.get_agent_radius(i) * sim.get_agent_radius(i):
            return False
    return True

def main():
    # Create simulator instance
    sim = orca.RVOSimulator()
    
    # Store goals for each agent
    goals = []
    for i in range(5):
        for j in range(5):
            goals.extend([
                (-75.0, -75.0),  # Goal for top-right agents
                (75.0, -75.0),   # Goal for top-left agents
                (-75.0, 75.0),   # Goal for bottom-right agents
                (75.0, 75.0)     # Goal for bottom-left agents
            ])
    
    # Set up the scenario
    setup_scenario(sim)
    
    # Run simulation
    while not reached_goal(sim, goals):
        set_preferred_velocities(sim, goals)
        sim.do_step()
        
        # Print agent positions (optional)
        print(f"Time: {sim.get_global_time():.2f}")
        for i in range(sim.get_num_agents()):
            pos = sim.get_agent_position(i)
            print(f"Agent {i}: ({pos[0]:.2f}, {pos[1]:.2f})")
        print()

if __name__ == "__main__":
    main() 