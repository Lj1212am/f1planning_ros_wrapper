# MIT License

# Copyright (c) Hongrui Zheng, Johannes Betz

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

"""
STMPC waypoint tracker example

Author: Hongrui Zheng
Last Modified: 8/1/22
"""

import numpy as np
import gymnasium as gym
from f1tenth_gym.envs import F110Env
import time

import sys, os
sys.path.append(os.path.join(os.path.dirname(__file__), "..", ".."))

from f1tenth_planning.control.dynamic_mpc.dynamic_mpc import STMPCPlanner


def main():
    """
    STMPC example. This example uses fixed waypoints throughout the 2 laps.
    For an example using dynamic waypoints, see the lane switcher example.
    """

    # create environment
    env: F110Env = gym.make(
        "f1tenth_gym:f1tenth-v0",
        config={
            "map": "Spielberg_blank",
            "num_agents": 1,
            "control_input": "accl",
            "observation_config": {"type": "original"},
            "params": F110Env.f1fifth_vehicle_params(),
        },
        render_mode="human",
    )

    # create planner
    planner = STMPCPlanner(track=env.track, debug=False)
    planner.config.dlk = (
        env.track.raceline.ss[1] - env.track.raceline.ss[0]
    )  # waypoint spacing - kinematic
    planner.config.dl = (
        env.track.raceline.ss[1] - env.track.raceline.ss[0]
    )  # waypoint spacing
    env.unwrapped.add_render_callback(planner.render_waypoints)
    env.unwrapped.add_render_callback(planner.render_local_plan)
    env.unwrapped.add_render_callback(planner.render_mpc_sol)

    env.add_render_callback(planner.render_waypoints)

    # reset environment
    poses = np.array(
        [
            [
                env.track.raceline.xs[0],
                env.track.raceline.ys[0],
                env.track.raceline.yaws[0],
            ]
        ]
    )
    obs, info = env.reset(options={"poses": poses})
    done = False
    env.render()
    
    laptime = 0.0
    start = time.time()

    ego_obs = dict()
    ego_obs["pose_x"] = obs["poses_x"][0]
    ego_obs["pose_y"] = obs["poses_y"][0]
    ego_obs["pose_theta"] = obs["poses_theta"][0]
    ego_obs["linear_vel_x"] = obs["linear_vels_x"][0]
    ego_obs["linear_vel_y"] = obs["linear_vels_y"][0]
    ego_obs["ang_vel_z"] = obs["ang_vels_z"][0]
    ego_obs["delta"] = 0.0 # Starts with 0, then updates from the steerv 
    ego_obs["beta"] = np.arctan2(ego_obs["linear_vel_y"], ego_obs["linear_vel_x"])

    accl, steerv = 0.0, 0.0
    done = False
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
            start = time.time()
            steerv, accl = planner.plan(ego_obs)
            print('planning Hz:', 1/(time.time() - start))
            
        obs, timestep, terminated, truncated, info = env.step(
            np.array([[steerv, accl]])
        )
        done = terminated or truncated
        laptime += timestep
        env.render()

    print("Sim elapsed time:", laptime, "Real elapsed time:", time.time() - start)


if __name__ == "__main__":
    main()
