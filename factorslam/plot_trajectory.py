import matplotlib.pyplot as plt
plt.style.use('ggplot')
from matplotlib.patches import Ellipse

import numpy as np

import gtsam
from gtsam.symbol_shorthand import B, V, X, L

import isam2_graph

def plot_trajectory(isam2graph, landmark_observations, usbl_data, poses, landmarks, dt):
    fig, ax = plt.subplots(figsize=(8,5))
    plt.title("X-Y Trajectory")
    plt.xlabel("X [m]")
    plt.ylabel("Y [m]")

    xT = np.array(isam2graph.x_smooth)
    rT = np.array(isam2graph.r_smooth)
    xT_filter = np.array(isam2graph.x_filter)
    lp_smooth = np.array(isam2graph.landmark_positions)

    for i in landmark_observations.keys():
        for li in landmark_observations[i].keys():
            landmark_obs_est = rT[i]@(landmark_observations[i][li][1]*landmark_observations[i][li][0]) +  xT[i]
            plt.plot([xT[i,0], landmark_obs_est[0]], [xT[i,1], landmark_obs_est[1]], c='gold', zorder=0)
    plt.plot([], [], c='gold', label="Landmark Observation")



    plt.plot(poses[:,0,3], poses[:,1,3], label='True Track', c='blue')
    usbl_data_l = np.array(list(usbl_data.values()))
    plt.scatter(usbl_data_l[:,0], usbl_data_l[:,1], c='purple', label="USBL Position Update")
    plt.plot(xT[:,0], xT[:,1], label='Smoothed Track', c='C0', ls=':', marker='x')
    plt.plot(xT_filter[:,0], xT_filter[:,1], label='Filtered Track', c='green', ls=':', marker='+')

    if len(landmarks):
        plt.scatter(landmarks[:,0], landmarks[:,1], c='orange', label='Landmark')
        if len(lp_smooth):
            plt.scatter(lp_smooth[:,0], lp_smooth[:,1], c='C0', label='Estimated Landmark')        

    for i_ in range(len(xT)):
        cov = isam2graph.isam.marginalCovariance(X(i_))[3:5,3:5]
        eigenvalues, eigenvectors = np.linalg.eig(cov)
        rotation = np.rad2deg(np.arctan(eigenvectors[1,0]/eigenvectors[0,0]))
        w = 2*np.sqrt(5.991 * eigenvalues[0])
        h = 2*np.sqrt(5.991 * eigenvalues[1])
        e = Ellipse(xT[i_, :2], w, h, 
                    angle=rotation, color='C0', alpha=0.7, fill=None, lw=1, ls=':')
        ax.add_artist(e)
    e = Ellipse([0, 0], 1, .5, angle=0, color="C0", ls=':', alpha=0.35,  fill=None, lw=1)
    handles, labels = ax.get_legend_handles_labels()
    handles.append(e)
    labels.append('95% CI')
                                
    plt.gca().set_aspect('equal')
    plt.legend(handles=handles, labels=labels, prop={'size':8}, loc='best')
    plt.show()

    fig = plt.figure(figsize=(8,5))
    plt.plot(np.arange(0, len(poses), 1)*dt, poses[:,2,3], c='blue', label='True Depth')
    plt.plot(np.arange(0, len(xT), 1), xT[:,2], c='C0',  ls=':', marker='x', label='Smoothed Depth')
    plt.plot(np.arange(0, len(xT_filter), 1), xT_filter[:,2], c='green',  ls=':', marker='x', label='Filtered Depth')

    plt.title("Z-Trajectory (Depth)")
    plt.ylabel("Z [m]")
    plt.xlabel("time [s]")
    plt.legend()
    plt.show()