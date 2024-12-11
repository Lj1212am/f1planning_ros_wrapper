"""
Casadi KMPC waypoint tracker example
"""

import numpy as np
import gymnasium as gym
from f1tenth_gym.envs import F110Env
import time
import os
import sys
sys.path.append(os.path.join(os.path.dirname(__file__), "..", ".."))
from f1tenth_planning.control.nonlinear_mpc.nonlinear_frenet_dmpc import NMPCPlanner
from f1tenth_gym.envs.track import Track

def main():
    """
    KMPC example. This example uses fixed waypoints throughout the 2 laps.
    For an example using dynamic waypoints, see the lane switcher example.
    """
    config_path = "/home/lee/work/f1-fifth/src/trajectory_csv"
    
    csv = "right_slalom_trajectory.csv"
    map_name = os.path.join(config_path, csv)
    waypoints = np.loadtxt(map_name, delimiter=',', skiprows=1) 
    
    
    # x = waypoints[:, 1]
    # y = waypoints[:, 2]
    # v = waypoints[:, 5]
    x = waypoints[:, 0] * 3.0
    x = x[:-1]
    y = waypoints[:, 1] * 3.0
    y = y[:-1]
    end_wp_x = x[0] - 0.1
    end_wp_y = y[0] 
    #add the waypoints to the end of the list
    # x = np.append(x, end_wp_x)
    # y = np.append(y, end_wp_y)
    # have v be the same size as x and y at velocity 10
    v = np.ones_like(x) * 5.0
    # v = 
    # # create track from custom reference line
    track = Track.from_refline(x=x, y=y, velx=v)

    custom_params = F110Env.f1tenth_vehicle_params()
    custom_params["mu"] = 1.0
    # create environment
    env: F110Env = gym.make(
        "f1tenth_gym:f1tenth-v0",
        config={
            "map": "Spielberg_blank",
            "num_agents": 1,
            "params": custom_params,
            "control_input": "accl",
            "observation_config": {"type": "original"},
        },
        render_mode="human",
    )

    # create planner
    planner = NMPCPlanner(track=track, debug=False)
    planner.config.dlk = track.raceline.ss[1] - track.raceline.ss[0]

    env.unwrapped.add_render_callback(planner.render_waypoints)
    env.unwrapped.add_render_callback(planner.render_local_plan)
    env.unwrapped.add_render_callback(planner.render_mpc_sol)

    # reset environment
    poses = np.array(
        [
            [
                track.raceline.xs[0],
                track.raceline.ys[0],
                track.raceline.yaws[0],
            ]
        ]
    )
    obs, info = env.reset(options={"poses": poses})
    done = False
    env.render()

    laptime = 0.0
    start = time.time()
    
    ego_obs = dict()
    # print('dict of state', obs.keys())
    ego_obs["pose_x"] = obs["poses_x"][0]
    ego_obs["pose_y"] = obs["poses_y"][0]
    ego_obs["pose_theta"] = obs["poses_theta"][0]
    ego_obs["linear_vel_x"] = obs["linear_vels_x"][0]
    ego_obs["linear_vel_y"] = obs["linear_vels_y"][0]
    ego_obs["ang_vel_z"] = obs["ang_vels_z"][0]
    ego_obs["delta"] = 0.0 # Starts with 0, then updates from the steerv 
    ego_obs["beta"] = np.arctan2(ego_obs["linear_vel_y"], ego_obs["linear_vel_x"])
    accl, steerv = 0.0, 0.0
    while not done:
        
        ego_obs["pose_x"] = obs["poses_x"][0]
        ego_obs["pose_y"] = obs["poses_y"][0]
        ego_obs["pose_theta"] = obs["poses_theta"][0]
        ego_obs["linear_vel_x"] = obs["linear_vels_x"][0]
        ego_obs["linear_vel_y"] = obs["linear_vels_y"][0]
        ego_obs["ang_vel_z"] = obs["ang_vels_z"][0]
        ego_obs["delta"] = ego_obs["delta"] + steerv * env.unwrapped.timestep
        ego_obs["beta"] = np.arctan2(ego_obs["linear_vel_y"], ego_obs["linear_vel_x"])
        
        
        # if linear velocity < 1 set it greater than 1 else accel is 9 and steerv is 0
        if ego_obs["linear_vel_x"] < 1:
            accl = 9
            steerv = 0
        else:
            accl, steerv = planner.plan(ego_obs)
            
        
        obs, timestep, terminated, truncated, infos = env.step(
            np.array([[steerv, accl]])
        )
        done = terminated or truncated
        laptime += timestep
        env.render()

        print(
            "speed: {}, steer vel: {}, accl: {}".format(
                ego_obs["linear_vel_x"], steerv, accl
            )
        )

    print("Sim elapsed time:", laptime, "Real elapsed time:", time.time() - start)


if __name__ == "__main__":
    main()
