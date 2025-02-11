"""
Casadi KMPC waypoint tracker example
"""

import numpy as np
import gymnasium as gym
from f1tenth_gym.envs import F110Env
from f1tenth_gym.envs.track import Track
import time
import os
import sys
sys.path.append(os.path.join(os.path.dirname(__file__), "..", ".."))
from f1tenth_planning.control.nonlinear_mpc.nonlinear_frenet_dmpc import NMPCPlanner
import matplotlib.pyplot as plt
from scipy.interpolate import CubicSpline


waypoint_first = 0
waypoint_num = -1
def custom_waypoints_to_track(csv_file):
    config_path = os.path.join(os.path.dirname(__file__), "maps")

    # csv = 'wp_20241125_132733.csv'
    # csv = 'interpolated_wp.csv'
    csv = csv_file
    # csv = 'interpolated_wp.csv'
    map_name = os.path.join(config_path, csv)
    waypoints = np.loadtxt(map_name, delimiter=';', skiprows=2) 

    # map_name = os.path.join(config_path, csv)
    # waypoints = np.loadtxt(map_name, delimiter=';', skiprows=15*2) 
    
    # waypoints[:, 3] += math.pi/2
    sin_yaw = np.sin(waypoints[:, 3])
    cos_yaw = np.cos(waypoints[:, 3])
    
    x = waypoints[:, 1]
    y = waypoints[:, 2]
    v = waypoints[:, 5]

    # x = x[waypoint_first:waypoint_num]
    # y = y[waypoint_first:waypoint_num] 
    # v = v[waypoint_first:waypoint_num] * 3.0
    v = 3.0 * v

    #filter topull 1 from every ten waypoints, to make the path smoother
    # x = x[::10]
    # y = y[::10]
    # v = v[::10]
    
    # make sure cubic spline won't fail
    distances = np.sqrt(np.diff(x)**2 + np.diff(y)**2)
    t = np.concatenate(([0], np.cumsum(distances)))  # Parameter t (cumulative distance)

    # Remove points where `t` values are not strictly increasing
    mask = np.diff(t) > 1e-6  # Keep points where the difference in `t` is significant
    mask = np.insert(mask, 0, True)  # Always include the first point
    x = x[mask]
    y = y[mask]
    v = v[mask]
    t = t[mask]

    # INTERPOLATE MANUALLY USING CUBIC SPLINE
    # Create a cubic spline object
    spline = CubicSpline(t, np.c_[x, y, v], bc_type='natural')
    # Interpolate the spline at a higher resolution
    t_interp = np.linspace(t[0], t[-1], num=100)
    x_interp, y_interp, v_interp = spline(t_interp).T
    # Create a new track object
    track = Track.from_refline(x_interp, y_interp, v_interp)
    return track

def main():
    """
    KMPC example. This example uses fixed waypoints throughout the 2 laps.
    For an example using dynamic waypoints, see the lane switcher example.
    """
    wp_track = custom_waypoints_to_track('rotated_safe_slalom.csv')
    # wp_track = custom_waypoints_to_track('Spielberg_blank_raceline.csv')
    # create environment
    env: F110Env = gym.make(
        "f1tenth_gym:f1tenth-v0",
        config={
            "map": "Spielberg_blank",
            "num_agents": 1,
            "control_input": "accl",
            "observation_config": {"type": "original"},
        },
        render_mode="human",
    )

    # Use custom waypoints
    track = wp_track
    # track = env.track
    
    # Convert the yaws to [0, 2pi]
    # track.raceline.yaws = (track.raceline.yaws + 2 * np.pi) % (2 * np.pi)  # Normalize to 0 - 2pi
    # track.raceline.yaws = (track.raceline.yaws + np.pi) % (2 * np.pi)      # Flip directions

    # Extract waypoints
    x = track.raceline.xs  # x-coordinates of waypoints
    y = track.raceline.ys  # y-coordinates of waypoints
    yaw = track.raceline.yaws  # yaws at each waypoint

    # Compute quiver components
    u = np.cos(yaw)  # x-component of arrow (cosine of yaw)
    v = np.sin(yaw)  # y-component of arrow (sine of yaw)

    # Plot waypoints
    plt.figure(figsize=(10, 8))
    plt.plot(x, y, 'o-', label='Waypoints', markersize=5, color='blue')

    # Add quiver plot for yaws
    plt.quiver(x, y, u, v, angles='xy', scale_units='xy', scale=0.5, color='red', label='Yaw')

    # Add labels and legend
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('Track Waypoints with Yaw Directions')
    plt.axis('equal')  # Equal aspect ratio for proper visualization
    plt.grid()
    plt.legend()

    # Show plot
    # plt.show()

    # create planner
    # planner = NMPCPlanner(track=track, debug=False)
    planner = NMPCPlanner(track, debug=False)
    planner.config.dlk = track.centerline.ss[1] - track.centerline.ss[0]
    # planner.config.dlk /= 2.0

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
    print('dict of state', obs.keys())
    ego_obs["pose_x"] = obs["poses_x"][0]
    ego_obs["pose_y"] = obs["poses_y"][0]
    ego_obs["pose_theta"] = obs["poses_theta"][0]
    ego_obs["linear_vel_x"] = obs["linear_vels_x"][0]
    ego_obs["linear_vel_y"] = obs["linear_vels_y"][0]
    ego_obs["ang_vel_z"] = obs["ang_vels_z"][0]
    ego_obs["delta"] = 0.0 # Starts with 0, then updates from the steerv 
    ego_obs["beta"] = np.arctan2(ego_obs["linear_vel_y"], ego_obs["linear_vel_x"])
    accl, steerv = 0.0, 0.0
    iter = 0
    # MAX_ITER = 110000

    x_traj = []
    y_traj = []
    yaw_traj = []

    s_traj = []
    ey_traj = []
    ephi_traj = []
    jump_count = 0
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
            accl, steerv = planner.plan(ego_obs, mu=1.0)
            # print('planning time:', time.time() - start)
            
        x_traj.append(ego_obs["pose_x"])
        y_traj.append(ego_obs["pose_y"])
        yaw_traj.append(ego_obs["pose_theta"])

        (s_curr, ey_curr, ephi_curr) = planner.frenet_state
        # if len(ey_traj) > 0:
        #     if abs(ey_traj[-1] - ey_curr) > 0.1:
        #         jump_count += 1
        #         print(f"Jump in ey: {ey_traj[-1]} -> {ey_curr} | Count: {jump_count}")

        s_traj.append(s_curr)
        ey_traj.append(ey_curr)
        ephi_traj.append(ephi_curr)
        
        obs, timestep, terminated, truncated, infos = env.step(
            np.array([[steerv, accl]])
        )
        done = terminated or truncated
        laptime += timestep
        env.render()

        # iter += 1
        # if(iter == MAX_ITER):
        #     done = True

        # print(
        #     "speed: {}, steer vel: {}, accl: {}".format(
        #         ego_obs["linear_vel_x"], steerv, accl
        #     )
        # )

    print("Sim elapsed time:", laptime, "Real elapsed time:", time.time() - start)

    # Plot the trajectory
    plt.figure(figsize=(10, 8))
    sc = plt.scatter(x_traj, y_traj, c=yaw_traj, cmap='viridis', label='Trajectory')
    plt.colorbar(sc, label='Yaw (radians)')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('Ego Vehicle Trajectory')
    plt.axis('equal')
    plt.grid()
    plt.legend()
    
    plt.figure(figsize=(10, 8))
    sc = plt.scatter(x_traj, y_traj, c=ephi_traj, cmap='viridis', label='Trajectory')
    plt.colorbar(sc, label='EYaw (radians)')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('Frenet State')
    plt.axis('equal')
    plt.grid()
    plt.legend()

    plt.figure(figsize=(10, 8))
    plt.scatter(s_traj, ey_traj, c=ephi_traj, cmap='viridis', label='Frenet State')
    plt.xlabel('s (m)')
    plt.ylabel('ey (m)')
    plt.title('Frenet State')
    plt.axis('equal')
    plt.grid()
    plt.legend()
    plt.show()

if __name__ == "__main__":
    main()
