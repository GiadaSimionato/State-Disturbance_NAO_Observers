# Script that computes the total RMSE, the total MAE, the partial RMSE and the Time To Convergence (TTC) for all the observers given the log files.
#
# @authors Giada Simionato (1822614), Riccardo Ratini (1656801), Emilian Postolache (1649271).

import sys
import numpy as np
import argparse

parser = argparse.ArgumentParser()
parser.add_argument('--root', dest='path', type=str, help='Path to the .txt files', required=True)
parser.add_argument('--pL', dest='pointL', type=int, help='Point of convergence for Luenberger', required=True)
parser.add_argument('--pK', dest='pointK', type=int, help='Point of convergence for Kalman', required=True)
parser.add_argument('--pS', dest='pointS', type=int, help='Point of convergence for Stephens', required=True)

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


# --- Function that scales the force. ---
# @param data: np array of data read form the .txt file
# @rerutn data: np array of scaled forces

def scale(data):

    scaleFactor = 5.19/35.1954
    data[:, 3] = scaleFactor*data[:, 3]
    return data


# --- Function that computes the RMSE value of the signal. ---
# @param est: np array of the estimates
# @param gt: np array of the truth values
# @return rmse: the corresponding value of the RMSE

def rmse(est, gt):
    e_sq = (np.linalg.norm(gt - est, axis=-1))**2
    return np.sqrt(e_sq.mean())


# --- Function that computes the MAE value of the signal. ---
# @param est: np array of the estimates
# @param gt: np array of the truth values
# @return rmse: the corresponding value of the MAE

def mae(est, gt):
    e_sq = np.linalg.norm(gt - est, axis=-1)
    return e_sq.mean()



if __name__ == '__main__':

    path = args.path + '/'
    pointL = args.pointL
    pointK = args.pointK
    pointS = args.pointS

    dt = 1/100

    force_x_luen = readData(path+'luenberger_x.txt', 5)
    force_x_luen = scale(force_x_luen)
    force_x_luen = force_x_luen[:, 3]
    force_y_luen = readData(path+'luenberger_y.txt', 5)
    force_y_luen = scale(force_y_luen)
    force_y_luen = force_y_luen[:, 3]

    force_luen = np.stack([force_x_luen, force_y_luen], axis=-1)

    force_x_steph = readData(path+'stephens_x.txt', 4)
    force_x_steph = scale(force_x_steph)
    force_x_steph = force_x_steph[:, 3]
    force_y_steph = readData(path+'stephens_y.txt', 4)
    force_y_steph = scale(force_y_steph)
    force_y_steph = force_y_steph[:, 3]

    force_steph = np.stack([force_x_steph, force_y_steph], axis=-1)

    force_x_kalm = readData(path+'kalman_x.txt', 5)
    force_x_kalm = scale(force_x_kalm)
    force_x_kalm = force_x_kalm[:, 3]
    force_y_kalm = readData(path+'kalman_y.txt', 5)
    force_y_kalm = scale(force_y_kalm)
    force_y_kalm = force_y_kalm[:, 3]

    force_kalm = np.stack([force_x_kalm, force_y_kalm], axis=-1)

    gt_x = readData(path+'luenberger_x_gt.txt', 5)
    gt_x = scale(gt_x)
    gt_x = gt_x[:, 3]
    gt_y = readData(path+'luenberger_y_gt.txt', 5)
    gt_y = scale(gt_y)
    gt_y = gt_y[:, 3]

    force_gt = np.stack([gt_x, gt_y], axis=-1)

    rmseL_tot = rmse(force_luen, force_gt)
    rmseK_tot = rmse(force_kalm, force_gt)
    rmseS_tot = rmse(force_steph, force_gt)

    maeL = mae(force_luen, force_gt)
    maeK = mae(force_kalm, force_gt)
    maeS = mae(force_steph, force_gt)

    rmseL = rmse(force_luen[pointL:], force_gt[pointL:])
    rmseK = rmse(force_kalm[pointK:], force_gt[pointK:])
    rmseS = rmse(force_steph[pointS:], force_gt[pointS:])

    tL = pointL*dt
    tK = pointK*dt
    tS = pointS*dt

    print('______________________________________________________________________')
    print()
    print('LUENBERGER data:')
    print('RMSE tot: {}'.format(rmseL_tot))
    print('MAE tot: {}'.format(maeL))
    print('RMSE after convergence: {}'.format(rmseL))
    print('Time to convergence: {} s'.format(tL))
    print('----------------------------------------------------------------------')
    print('KALMAN data:')
    print('RMSE tot: {}'.format(rmseK_tot))
    print('MAE tot: {}'.format(maeK))
    print('RMSE after convergence: {}'.format(rmseK))
    print('Time to convergence: {} s'.format(tK))
    print('----------------------------------------------------------------------')
    print('STEPHENS data:')
    print('RMSE tot: {}'.format(rmseS_tot))
    print('MAE tot: {}'.format(maeS))
    print('RMSE after convergence: {}'.format(rmseS))
    print('Time to convergence: {} s'.format(tS))
    print()
    print('______________________________________________________________________')

