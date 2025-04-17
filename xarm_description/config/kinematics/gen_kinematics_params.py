import os
import sys
import socket
import struct

try:
    from yaml import dump
except:
    def dump(data, f, indent=0, **kwargs):
        buf = []
        for key, val in data.items():
            if isinstance(val, dict):
                buf.append('{}{}:'.format(' ' * indent, key))
                buf += dump(val, None, indent=indent+2, **kwargs)
            else:
                buf.append('{}{}: {}'.format(' ' * indent, key, val))
        if f is not None:
            f.write('\n'.join(buf))
        return buf

IS_PY3 = sys.version_info.major >= 3

if __name__ == '__main__':
    if len(sys.argv) < 3:
        print('Usage: {} {{robot_ip}} {{kinematics_suffix}}'.format(sys.argv[0]))
        exit(1)
    robot_ip = sys.argv[1]
    kinematics_suffix = sys.argv[2]
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect((robot_ip, 502))

    send_data = [0x00, 0x01, 0x00, 0x02, 0x00, 0x01, 0x08]
    if IS_PY3:
        sock.send(bytes(send_data))
    else:
        sock.send(''.join(map(chr, send_data)))
    recv_data = sock.recv(179)
    if len(recv_data) == 179 and recv_data[8]:
        if IS_PY3:
            robot_dof = recv_data[9]
            robot_type = recv_data[10]
        else:
            robot_dof = ord(recv_data[9])
            robot_type = ord(recv_data[10])
        robot_name = 'uf850' if robot_dof == 6 and robot_type == 12 else 'lite6' if robot_dof == 6 and robot_type == 9 else 'xarm{}'.format(robot_dof)
        ouput_dir = os.path.join(os.path.dirname(__file__), 'user')
        if not os.path.exists(ouput_dir):
            os.makedirs(ouput_dir)
        output_file = os.path.join(ouput_dir, '{}_kinematics_{}.yaml'.format(robot_name, kinematics_suffix))
        params = struct.unpack('<42f', recv_data[11:])
        kinematics = {}
        data = {'kinematics': kinematics}
        for i in range(robot_dof):
            joint_param = {}
            kinematics['joint{}'.format(i + 1)] = joint_param
            joint_param['x'] = params[i * 6]
            joint_param['y'] = params[i * 6 + 1]
            joint_param['z'] = params[i * 6 + 2]
            joint_param['roll'] = params[i * 6 + 3]
            joint_param['pitch'] = params[i * 6 + 4]
            joint_param['yaw'] = params[i * 6 + 5]
        if IS_PY3:
            with open(output_file, 'w', encoding='utf-8') as f:
                try:
                    dump(data, f, default_flow_style=False, allow_unicode=True, sort_keys=False)
                except:
                    dump(data, f, default_flow_style=False, allow_unicode=True)
        else:
            with open(output_file, 'w') as f:
                try:
                    dump(data, f, default_flow_style=False, allow_unicode=True, sort_keys=False)
                except:
                    dump(data, f, default_flow_style=False, allow_unicode=True)
        print('[Success] save to {}'.format(output_file))
    else:
        print('[Failed] recv_len={}, valid={}'.format(len(recv_data), 0 if len(recv_data) < 9 else recv_data[8]))
