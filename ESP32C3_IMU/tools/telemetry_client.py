#!/usr/bin/env python3
"""
Simple telemetry client for Jumper ESP32C3 SoftAP telemetry.
- Connects to 192.168.4.1:3333
- Logs received JSON lines to CSV
- Displays live roll/pitch/yaw plot computed from quaternion
"""
import socket
import json
import csv
import time
import threading
import collections
import math
import sys
import argparse
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D

HOST = '192.168.4.1'
PORT = 3333
CSV_PATH = 'telemetry_log.csv'

def quat_to_euler(w,x,y,z):
    # returns roll, pitch, yaw in degrees
    # roll (x-axis rotation)
    t0 = 2.0*(w*x + y*z)
    t1 = 1.0 - 2.0*(x*x + y*y)
    roll = math.degrees(math.atan2(t0, t1))

    # pitch (y-axis)
    t2 = 2.0*(w*y - z*x)
    t2 = max(-1.0, min(1.0, t2))
    pitch = math.degrees(math.asin(t2))

    # yaw (z-axis)
    t3 = 2.0*(w*z + x*y)
    t4 = 1.0 - 2.0*(y*y + z*z)
    yaw = math.degrees(math.atan2(t3, t4))
    return roll, pitch, yaw

class TelemetryClient:
    def __init__(self):
        self.sock = None
        self.running = True
        self.lock = threading.Lock()
        self.data = collections.deque(maxlen=500)
        self.last_quat = (1.0, 0.0, 0.0, 0.0)
        self.last_vel = (0.0, 0.0, 0.0)
        self.last_velDir = (0.0, 1.0, 0.0)

    def connect(self):
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(5)
            self.sock.connect((HOST, PORT))
            self.f = self.sock.makefile('r')
            print('Connected to telemetry server')
            return True
        except Exception as e:
            print('Connection error:', e)
            return False

    def run(self):
        with open(CSV_PATH, 'w', newline='') as csvf:
            writer = csv.writer(csvf)
            writer.writerow(['t','type','qx','qy','qz','qw','vx','vy','vz','velDir_x','velDir_y','velDir_z'])
            while self.running:
                try:
                    line = self.f.readline()
                    if not line:
                        time.sleep(0.1)
                        continue
                    # Debug: show raw incoming line
                    print('RECV:', line.strip())
                    obj = json.loads(line)
                    t = obj.get('t', int(time.time()*1000))
                    if obj.get('type') == 'ballistic':
                        quat = obj.get('quat',[1,0,0,0])
                        vel = obj.get('vel',[0,0,0])
                        velDir = obj.get('velDir',[0,1,0])
                        self.last_quat = tuple(quat)
                        self.last_vel = tuple(vel)
                        self.last_velDir = tuple(velDir)
                        with self.lock:
                            self.data.append((t, self.last_quat, self.last_vel, self.last_velDir))
                        writer.writerow([t,'ballistic', quat[1], quat[2], quat[3], quat[0], vel[0], vel[1], vel[2], velDir[0], velDir[1], velDir[2]])
                        csvf.flush()
                    else:
                        with self.lock:
                            self.data.append((t, self.last_quat, self.last_vel, self.last_velDir))
                        writer.writerow([t,'waiting', self.last_quat[1], self.last_quat[2], self.last_quat[3], self.last_quat[0], self.last_vel[0], self.last_vel[1], self.last_vel[2], self.last_velDir[0], self.last_velDir[1], self.last_velDir[2]])
                        csvf.flush()
                except Exception as e:
                    print('Read error', e)
                    time.sleep(0.5)

    def stop(self):
        self.running = False
        try:
            self.sock.close()
        except:
            pass


def quat_rot_vec(w, x, y, z, vx, vy, vz):
    tx = 2.0*(y*vz - z*vy)
    ty = 2.0*(z*vx - x*vz)
    tz = 2.0*(x*vy - y*vx)
    return (
        vx + z*tx - y*tz + w*tx,
        vy + x*tz - z*tx + w*ty,
        vz + y*tx - x*ty + w*tz
    )


def vec_angle_error(a, b):
    dot = max(-1.0, min(1.0, a[0]*b[0] + a[1]*b[1] + a[2]*b[2]))
    return math.degrees(math.acos(dot))


def world_dir_to_tilt(vx, vy, vz):
    pitch = math.degrees(math.asin(max(-1.0, min(1.0, vx))))
    roll = math.degrees(math.atan2(-vz, vy))
    return roll, pitch


def main():
    global HOST, PORT

    parser = argparse.ArgumentParser()
    parser.add_argument('--host', help='Telemetry host IP', default=HOST)
    parser.add_argument('--port', help='Telemetry port', type=int, default=PORT)
    args = parser.parse_args()

    HOST = args.host
    PORT = args.port

    client = TelemetryClient()
    if not client.connect():
        print('Failed to connect, exiting.')
        return
    t = threading.Thread(target=client.run, daemon=True)
    t.start()

    fig = plt.figure(figsize=(10,10))
    ax1 = fig.add_subplot(4,1,1)
    ax2 = fig.add_subplot(4,1,2)
    ax3 = fig.add_subplot(4,1,3)
    ax4 = fig.add_subplot(4,1,4, projection='3d')

    lines_actual = [ax1.plot([],[],label=label)[0] for label in ['Roll','Pitch','Yaw']]
    lines_target = [ax2.plot([],[],label=label)[0] for label in ['Target Roll','Target Pitch']]
    line_error = ax3.plot([],[],label='Orientation Error (deg)')[0]

    def init():
        ax1.set_xlim(0,20); ax1.set_ylim(-180,180); ax1.legend(loc='upper right')
        ax2.set_xlim(0,20); ax2.set_ylim(-180,180); ax2.legend(loc='upper right')
        ax3.set_xlim(0,20); ax3.set_ylim(0,180); ax3.set_ylabel('Error (deg)'); ax3.legend(loc='upper right')
        ax4.set_xlim(-1,1); ax4.set_ylim(-1,1); ax4.set_zlim(-1,1)
        ax4.set_xlabel('X'); ax4.set_ylabel('Y'); ax4.set_zlabel('Z')
        return lines_actual + lines_target + [line_error]

    def update(frame):
        with client.lock:
            data = list(client.data)
        if not data:
            return lines_actual + lines_target + [line_error]

        times = [(d[0]-data[0][0])/1000.0 for d in data]
        rolls = []
        pitches = []
        yaws = []
        target_rolls = []
        target_pitches = []
        errors = []
        body_y = (0.0, 1.0, 0.0)
        target_vec = (0.0, 1.0, 0.0)

        for (_, quat, _, velDir) in data:
            w,x,y,z = quat
            r,p,yw = quat_to_euler(w,x,y,z)
            rolls.append(r); pitches.append(p); yaws.append(yw)
            target_roll, target_pitch = world_dir_to_tilt(velDir[0], velDir[1], velDir[2])
            target_rolls.append(target_roll)
            target_pitches.append(target_pitch)
            actual_y = quat_rot_vec(w,x,y,z, 0.0, 1.0, 0.0)
            error = vec_angle_error(actual_y, velDir)
            errors.append(error)
            target_vec = velDir
            body_y = actual_y

        for i,l in enumerate(lines_actual):
            if i==0: l.set_data(times, rolls)
            if i==1: l.set_data(times, pitches)
            if i==2: l.set_data(times, yaws)
        lines_target[0].set_data(times, target_rolls)
        lines_target[1].set_data(times, target_pitches)
        line_error.set_data(times, errors)

        window = 20.0
        if times:
            start = max(0.0, times[-1] - window)
            for ax in (ax1, ax2, ax3):
                ax.set_xlim(start, start + window)

        for ax in (ax1, ax2, ax3):
            ax.relim(); ax.autoscale_view()

        ax4.cla()
        ax4.set_xlim(-1,1); ax4.set_ylim(-1,1); ax4.set_zlim(-1,1)
        ax4.set_xlabel('X'); ax4.set_ylabel('Y'); ax4.set_zlabel('Z')
        ax4.quiver(0,0,0, body_y[0], body_y[1], body_y[2], color='blue', length=1.0, normalize=True)
        ax4.quiver(0,0,0, target_vec[0], target_vec[1], target_vec[2], color='red', length=1.0, normalize=True)
        ax4.text(0.0, -1.2, 0.0, 'Blue: body +Y  Red: target velocity', color='black')

        return lines_actual + lines_target + [line_error]

    ani = FuncAnimation(fig, update, init_func=init, interval=200)
    try:
        plt.show()
    except KeyboardInterrupt:
        pass
    client.stop()

if __name__ == '__main__':
    main()
