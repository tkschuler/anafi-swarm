import socket
import time
import sys
from termcolor import colored
from threading import Thread
import math
import numpy as np
import atexit
from signal import signal, SIGINT

from math import acos, pi, cos, sin


## Olympe Functionality --Added by RN 3/12/19
import olympe
from olympe.messages.ardrone3.Piloting import TakeOff, moveBy, Landing, Emergency, PCMD
from olympe.messages.ardrone3.PilotingState import FlyingStateChanged, PositionChanged
from olympe.enums.ardrone3.PilotingState import FlyingStateChanged_State
from olympe.messages.ardrone3.GPSSettingsState import GPSFixStateChanged
from olympe.messages.ardrone3.PilotingSettings import setAutonomousFlightMaxRotationSpeed
from olympe.messages.ardrone3.SpeedSettings import MaxRotationSpeed
from olympe.messages.ardrone3.SpeedSettingsState import MaxRotationSpeedChanged


'''This file opens a socketed server thread and parses incomming messages to send unique
data packets to multiple clients'''


class Waypoint():
    def __init__(self, x, y, z, roll, pitch, yaw):
        self.x = x
        self.y = y
        self.z = z
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw


class Server:
    def __init__(self,port,host):
        self.port = port
        self.host = host
        self.server = None
        self.ON = True
        self.waypoints = [Waypoint(1.0, 0, 1.0, 0, 0, 0), Waypoint(1., 1 , 1.0, 0, 0, 0), Waypoint(0.0, 1.0, 1.0, 0, 0, 0), Waypoint(0.0, 0.0, 1.0, 0, 0, 0)]
        #self.waypoints = [1,2,3,4,5]
        self.curPose = Waypoint(0, 0, 0, 0, 0, 0) #starting this off big before vicon starts
        self.curWPindex = 0
        self.r = 0.2 # m

    def newClient(self,clientsocket, addr):
        #This listens for the incomming messages that client_rate.py is listening to ROS for
        while True:
            msg = clientsocket.recv(4096)
            msg = msg.decode('utf-8')
            # print( colored(("data received:", msg), "yellow") )
            pose = msg.split(',')
            clientsocket.send(b"ACK!")

            self.curPose.x = float(pose[0])
            self.curPose.y = float(pose[1])
            self.curPose.z = float(pose[2])
            qx  = float(pose[3])
            qy = float(pose[4])
            qz  = float(pose[5])
            qw  = float(pose[6])
            [yaw, pitch, roll] = self.quaternion_to_euler(qx, qy, qz, qw)
            self.curPose.roll = roll
            self.curPose.pitch = pitch
            self.curPose.yaw = yaw

        clientsocket.close()

    def checkWP(self):
        #This is a 1D exaple for checking if the drone has made it to a current waypoint


        while True:

            # Drone Pose
            x_dr   = self.curPose.x
            y_dr   = self.curPose.y
            z_dr   = self.curPose.z
            yaw_dr = self.curPose.yaw

            # Reference
            x_R   = self.waypoints[self.curWPindex].x
            y_R   = self.waypoints[self.curWPindex].y
            z_R   = self.waypoints[self.curWPindex].z

            Thr_vec = np.array([0.1, 0.1, 0.05, 1*pi/180])  # Threshold Vector

            er_mag = math.pow((x_dr - x_R),2) + math.pow((y_dr - y_R),2) + math.pow((z_dr - z_R),2)

            if er_mag <= math.pow(self.r,2): 
                print(colored("MADE IT TO WP!", "green"))
                #send new drone control command
                self.curWPindex +=1
            else:


                gn_mat = [0.4, 0.4, 0.2, 0.3]

                xEr = x_R-x_dr
                yEr = y_R-y_dr
                zEr = z_R-z_dr

                th_req = -New_Sign(yEr)*acos(xEr/math.sqrt(er_mag))

                R_ER_I = np.array([xEr, yEr, zEr, th_req])

                mov_cm_UC =  R3_Mat_Cords(yaw_dr, R_ER_I, gn_mat)

                x_c, y_c, z_c, yaw_c = Check_Move_Sat(mov_cm_UC, Thr_vec)
                yaw_c = 0.0

                print(colored(("Controller commands: ", x_c, -y_c, -z_c, yaw_c),"cyan"))
                print(colored(("Drone position: ", x_dr, y_dr, z_dr),"yellow"))
                print(colored(("Target position: ", x_R, y_R, z_R),"green"))

                drone(moveBy(x_c, -y_c, -z_c, yaw_c)).wait()
                #time.sleep(0.3)


                #check if end of trajectory
                if self.curWPindex == len(self.waypoints):
                    break


    def connect(self):
        self.s = socket.socket()
        print('Server started!')
        print('Waiting for clients...')

        self.s.bind((self.host, self.port))  # Bind to the port
        self.s.listen(5)  #Wait for client connection, up to 5.

    def listen(self):
        # Server must be in while open to stay open and listen
        while self.ON:
            try:
                c,addr = self.s.accept()  # Establish connection with client.
                print(colored(('Got connection from' + str(addr)),"magenta"))
                self.server = Thread(target=self.newClient, args=((c,addr)))
                self.server.daemon = True

                self.wpcheck = Thread(target=self.checkWP)
                self.wpcheck.daemon = True

                self.server.start()
                self.wpcheck.start()
            except:
                print(colored("Socket Error.", "red"))
                sys.exit()

        self.s.close()

    #Stop server manually
    def close(self):
        self.ON = False
        print("Server closed.")

    def quaternion_to_euler(self, x, y, z, w):
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2

        pitch = math.asin(t2)
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)

        return [yaw, pitch, roll]




def R3_Mat_Cords(th_R0, R_ER_I, gn_mat):
    th_R1 = -th_R0
    d_in = np.array([R_ER_I[0], R_ER_I[1], R_ER_I[2]])
    
    R3 = np.array([[cos(th_R1), -sin(th_R1), 0], [sin(th_R1), cos(th_R1), 0], [0, 0, 1]])
    dv2 = R3.dot(d_in)  # Convert to Drone frame
    mat2 = np.array([dv2[0], dv2[1], dv2[2], R_ER_I[3]])
    cm_mat = np.multiply(mat2, gn_mat)

    return cm_mat


def New_Sign(a):
    if a>=0.0:
        r = 1.0
    else:
        r = -1.0 
    return r

def Check_Move_Sat(com_vec, thr_vec):

    l = len(com_vec)
    ve = np.zeros(l)

    for i in range(l):
        if abs(com_vec[i]) <= thr_vec[i]:
            ve[i] = New_Sign(com_vec[i])*thr_vec[i]
        else:
            ve[i] = com_vec[i]

    xc   = ve[0]
    yc   = ve[1]
    zc   = ve[2]
    yawc = ve[3]
    return xc, yc, zc, yawc

def Drone_land(signal_recieved, frame):
    
    drone(Landing()).wait()
    time.sleep(10)
    drone(Emergency()).wait()
    print("Simulation Exited")
    sys.exit(0)


'''Main loop for testing'''

## Initialize Server

signal(SIGINT, Drone_land)
s = Server(50006,"127.0.0.1")
s.connect()

# Initialize Drone and Issue Take Off
global drone
drone = olympe.Drone("192.168.42.1", loglevel = 0)

drone.connection()
drone(
    TakeOff()
    >> FlyingStateChanged(state="hovering", _timeout=5)
).wait()


# Start listening to Vicon on the Server
s.listen()

atexit.register(Drone_land)


