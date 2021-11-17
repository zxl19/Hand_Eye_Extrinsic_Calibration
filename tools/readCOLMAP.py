import configparser
import numpy as np


# *Read config file
config = configparser.ConfigParser()
config.read('config.ini')
input_vo_path = config['DEFAULT']['input_vo_path']
output_vo_path = config['DEFAULT']['output_vo_path']


def inverse(q):
    qw = q[0]
    qx = q[1]
    qy = q[2]
    qz = q[3]
    q_inv = [qw, -qx, -qy, -qz]
    return q_inv


def quat2rotm(q):
    qw = q[0]
    qx = q[1]
    qy = q[2]
    qz = q[3]
    R = np.array([[1-2*qy*qy-2*qz*qz, 2*qx*qy-2*qw*qz, 2*qx*qz+2*qw*qy],
                  [2*qx*qy+2*qw*qz, 1-2*qx*qx-2*qz*qz, 2*qy*qz-2*qw*qx],
                  [2*qx*qz-2*qw*qy, 2*qy*qz+2*qw*qx, 1-2*qx*qx-2*qy*qy]], dtype=np.float64)
    return R


line_num = 0
output_file = open(output_vo_path, 'w')
with open(input_vo_path, 'r') as input_file:
    for line in input_file:
        if line[0] == '#':
            continue
        else:
            line_num = line_num + 1
        if (line_num % 2) == 1:
            data = line.split(' ')
            qw = float(data[1])
            qx = float(data[2])
            qy = float(data[3])
            qz = float(data[4])
            tx = float(data[5])
            ty = float(data[6])
            tz = float(data[7])
            timestamp = data[9].rstrip('.jpg\n')
            q = [qw, qx, qy, qz]
            t = np.array([tx, ty, tz], dtype=np.float64)
            q_inv = inverse(q)
            R_inv = quat2rotm(q_inv)
            t_inv = np.dot(-R_inv, t)
            pose = [timestamp, str(t_inv[0]), str(t_inv[1]), str(t_inv[2]), str(q_inv[0]), str(q_inv[1]), str(q_inv[2]), str(q_inv[3])]
            output_file.write(' '.join(pose))
            output_file.write('\n')
