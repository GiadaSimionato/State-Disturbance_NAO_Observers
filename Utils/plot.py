# Script that plots the observers' estimates with respect to the ground truth values
#
# @authors Giada Simionato (1822614), Riccardo Ratini (1656801), Emilian Postolache (1649271).

import matplotlib.pyplot as plt
import sys
import numpy as np
from numpy import linalg as LA
import argparse

parser = argparse.ArgumentParser()
parser.add_argument('--pathESTx', dest='path_estimates_x', type=str, help='Path to estimated data.txt file along x', required=False)
parser.add_argument('--pathESTy', dest='path_estimates_y', type=str, help='Path to estimated data.txt file along y', required=False)
parser.add_argument('--pathESTz', dest='path_estimates_z', type=str, help='Path to estimated data.txt file along z', required=False)
parser.add_argument('--pathGTx', dest='path_gt_x', type=str, help='Path to ground truth data.txt file along x', required=False)
parser.add_argument('--pathGTy', dest='path_gt_y', type=str, help='Path to ground truth data.txt file along y', required=False)
parser.add_argument('--pathGTz', dest='path_gt_z', type=str, help='Path to ground truth data.txt file along z', required=False)
parser.add_argument('--margin', dest = 'margin', type=float, help='Margin for the ground truth plot', default=0.0)
parser.add_argument('--observer', dest='observer', type=str, help='Observer from which the data was produced (e.g. lunemberger, kalman, stephens)', required=True)
parser.add_argument('--quantity', dest='quantity', type=str, help='Quantity to plot (e.g. com_pos, com_vel, com_acc, zmp_pos, cop_pos, force, dforce)', required=False)
parser.add_argument('--pathSavings', dest='path_saving', type=str, help='Path to folder where to save plots', default='./')

args = parser.parse_args()


# --- Function that read and cast data from external data files. ---
# @param path: path to the data.txt file
# @param n: length of the state vector (for check corruption data file)
# @return data: return matrix of floats, whose rows are the estimates and columns are the n estimated values

def readData(path, n):

    print('Reading data from ' + path +'...')
    f = open(path, "r")
    i = 0
    data = []

    while True:
        line = f.readline()
        if line == '':                  # end document reached
            break
        line = line.split()
        if len(line)!=n:                # check consistency data
            print('Data are incomplete in row ' + str(i+1))
            sys.exit()
        data.append(line)
        i += 1
    f.close()
    print('Done. Read ' + str(i) + ' rows.')
    return np.asarray(data).astype(np.float64)


# --- Function that plots the estimated and ground truth data. ---
# @param t: ndarray, timesteps
# @param est: ndarray, estimated data
# @param gt: ndarray, ground truth data
# @param lab: customized label for y axis
# @param a: customized axis of data
# @param u: customized udm
# @return fig: return the figure

def plot(t, est, gt, lab, a, u, i):

    fig = plt.figure(i+1)
    plt.plot(t, est, 'r', linewidth=2.0, label='Estimates')
    plt.plot(t, gt,  'b:', linewidth=2.0, label='Ground truth')
    mar = margin*gt
    plt.fill_between(t, gt-mar, gt+mar, alpha=0.2)
    plt.xlabel('Time [s]')
    plt.ylabel(lab +' along axis ' + a + ' '+u)
    plt.grid(True)
    plt.legend()
    #plt.show()
    return fig


# --- Function that plots the estimated and ground truth data. ---
# @param t: ndarray, timesteps
# @param est: ndarray, estimated data
# @param gt: ndarray, ground truth data
# @return fig: return the figure

def plotNorm(t, est, gt):

    fig = plt.figure(1)
    plt.plot(t, est, 'r', linewidth=2.0, label='Estimates')
    plt.plot(t, gt,  'b:', linewidth=2.0, label='Ground truth')
    plt.fill_between(t, gt-margin, gt+margin, alpha=0.2)
    plt.xlabel('Time [s]')
    plt.ylabel('Norm of External Force [N]')
    plt.grid(True)
    plt.legend()
    plt.show()
    return fig


# --- Function that scales the force. ---
# @param data: np array of data read form the .txt file
# @rerutn data: np array of scaled forces

def scale(data):

    scaleFactor = 5.19/35.1954
    data[:, 3] = scaleFactor*data[:, 3]
    return data


if __name__ == '__main__':

    # Get params
    path_estimates_x = args.path_estimates_x
    path_estimates_y = args.path_estimates_y
    path_estimates_z = args.path_estimates_z
    path_gt_x = args.path_gt_x
    path_gt_y = args.path_gt_y
    path_gt_z = args.path_gt_z
    observer = args.observer.lower()
    quantity = args.quantity
    margin = args.margin
    path_saving = args.path_saving

    # Data observers
    observer_lists = ['luenberger', 'kalman', 'stephens']
    n_obs = [5,5,4]

    state_l = ['com_pos', 'com_vel', 'zmp_pos', 'force', 'dforce']
    state_kf = ['com_pos', 'com_vel', 'com_acc', 'force', 'dforce']
    state_s = ['com_pos', 'com_vel', 'cop_pos', 'force']
    states = [state_l, state_kf, state_s]

    y_labels_l = ['COM Position', 'COM Velocity', 'ZMP Position', 'External Force', 'Derivative of External Force']
    y_labels_kf = ['COM Position', 'COM Velocity', 'COM Acceleration', 'External Force', 'Derivative of External Force']
    y_labels_s = ['COM Position', 'COM Velocity', 'COP Position', 'External Force']
    y_labels = [y_labels_l, y_labels_kf, y_labels_s]

    udm_l = ['[m]', '[m/s]', '[m]', '[N]', '[N/s]']
    udm_kf = ['[m]', '[m/s]', '[m/s^2]', '[N]', '[N/s]']
    udm_s = ['[m]', '[m/s]', '[m]', '[N]']
    udms = [udm_l, udm_kf, udm_s]

    # Filter information
    try:
        o = observer_lists.index(observer)
        n = n_obs[o]
        try:
            if quantity!=None:
                q = [states[o].index(quantity)]
            else:
                q = np.arange(n_obs[o])
        except:
            print('Component '+quantity+" was not estimated in the "+observer+ " observer.")
            sys.exit()
    except:
        print('Observer '+observer+" was not implemented.")
        sys.exit()


    # Extract and Plot data
    dt = 1/100       # sampling rate

    if path_estimates_x != None and path_gt_x != None:

        figs = []
        data_est_x = readData(path_estimates_x, n)
        data_est_x = scale(data_est_x)
        data_gt_x = readData(path_gt_x, n)
        data_gt_x = scale(data_gt_x)
        for i in q:

            y_label = y_labels[o][i]
            udm = udms[o][i]
            timesteps = np.arange(0, dt*len(data_est_x), dt)
            values_est_x = data_est_x[:,i]
            values_gt_x = data_gt_x[:,i]
            fig_i = plot(timesteps, values_est_x, values_gt_x, y_label, 'x', udm, i)
            figs.append(fig_i)
        plt.show()
        print('Do you want to save these plots? [y/n]')
        dec = input()
        if dec == 'y':
            for i, item in enumerate(q):
                path_save = path_saving + observer_lists[o] + '/'+ states[o][item]+'_x'
                figs[i].savefig(path_save, format='svg')
            print('Saved.')
        else:
            print('Discarded.')
            
    
    if path_estimates_y != None and path_gt_y != None:

        figs = []
        data_est_y = readData(path_estimates_y, n)
        data_est_y = scale(data_est_y)
        data_gt_y = readData(path_gt_y, n)
        data_gt_y = scale(data_gt_y)
        for i in q:

            y_label = y_labels[o][i]
            udm = udms[o][i]
            timesteps = np.arange(0, dt*len(data_est_y), dt)
            values_est_y = data_est_y[:,i]
            values_gt_y = data_gt_y[:,i]
            fig_i = plot(timesteps, values_est_y, values_gt_y, y_label, 'y', udm, i)
            figs.append(fig_i)
        plt.show()
        print('Do you want to save these plots? [y/n]')
        dec = input()
        if dec == 'y':
            for i, item in enumerate(q):
                path_save = path_saving + observer_lists[o] + '/'+ states[o][item]+'_y'
                figs[i].savefig(path_save, format='svg')
            print('Saved.')
        else:
            print('Discarded.')
        

    if path_estimates_z != None and path_gt_z != None:

        figs = []
        data_est_z = readData(path_estimates_z, n)
        data_est_z = scale(data_est_z)
        data_gt_z = readData(path_gt_z, n)
        data_gt_z = scale(data_gt_z)
        for i in q:

            y_label = y_labels[o][i]
            udm = udms[o][i]
            timesteps = np.arange(0, dt*len(data_est_z), dt)
            values_est_z = data_est_z[:,i]
            values_gt_z = data_gt_z[:,i]
            fig_i = plot(timesteps, values_est_z, values_gt_z, y_label, 'z', udm, i)
            figs.append(fig_i)
        plt.show()
        print('Do you want to save these plots? [y/n]')
        dec = input()
        if dec == 'y':
            for i, item in enumerate(q):
                path_save = path_saving + observer_lists[o] + '/'+ states[o][item]+'_z'
                figs[i].savefig(path_save, format='svg')
            print('Saved.')
        else:
            print('Discarded.')

    if None not in [path_estimates_x, path_gt_x, path_estimates_y, path_gt_y, path_estimates_z, path_gt_z]:

        i = states[o].index('force')
        values_est_x = data_est_x[:,i]
        values_est_y = data_est_y[:,i]
        values_est_z = data_est_z[:,i]
        values_est_x = values_est_x[:, np.newaxis]
        values_est_y = values_est_y[:, np.newaxis]
        values_est_z = values_est_z[:, np.newaxis]
        tot_est = np.concatenate((values_est_x, values_est_y, values_est_z), axis=1)
        norm_est = LA.norm(tot_est, axis=1)

        values_gt_x = data_gt_x[:,i]
        values_gt_y = data_gt_y[:,i]
        values_gt_z = data_gt_z[:,i]
        values_gt_x = values_gt_x[:, np.newaxis]
        values_gt_y = values_gt_y[:, np.newaxis]
        values_gt_z = values_gt_z[:, np.newaxis]
        tot_gt = np.concatenate((values_gt_x, values_gt_y, values_gt_z), axis=1)
        norm_gt = LA.norm(tot_gt, axis=1)

        figure = plotNorm(timesteps, norm_est, norm_gt)
        print('Do you want to save this plot? [y/n]')
        dec = input()
        if dec == 'y':
            figure.savefig(path_saving + observer_lists[o] + '/force_norm', format='svg')
            print('Saved.')
        else:
            print('Discarded.')
    print('Process terminated.')