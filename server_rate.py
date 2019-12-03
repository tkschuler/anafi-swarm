import socket
import time
import sys
from termcolor import colored
import thread
from threading import Thread
import math

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
        self.waypoints = [Waypoint(1.,1.,1.,0,0,0),Waypoint(2.,2.,2.,0,0,0),Waypoint(3.,3.,3.,0,0,0)]
        #self.waypoints = [1,2,3,4,5]
        self.curPose = Waypoint(100,100,100,0,0,0) #starting this off big before vicon starts
        self.curWPindex = 0
        self.r = 0.1 # m

    def newClient(self,clientsocket, addr):
        #This listens for the incomming messages that client_rate.py is listening to ROS for
        while True:
            msg = clientsocket.recv(4096)
            print "data received:", msg
            pose = msg.split(',')
            clientsocket.send("ACK!")

            self.curPose.x = float(pose[0])
            self.curPose.y = float(pose[1])
            self.curPose.z = float(pose[2])
            self.curPose.roll  = float(pose[3])
            self.curPose.pitch = float(pose[4])
            self.curPose.yaw   = float(pose[5])

        clientsocket.close()

    def checkWP(self):
        #This is a 1D exaple for checking if the drone has made it to a current waypoint
        while True:
            if math.pow((self.curPose.x - self.waypoints[self.curWPindex].x),2) + math.pow((self.curPose.y - self.waypoints[self.curWPindex].y),2) + math.pow((self.curPose.z - self.waypoints[self.curWPindex].z),2) <= math.pow(self.r,2): #Check if WP is in range of sphere
                print colored("MADE IT TO WP!", "green")
                #send new drone control command
                self.curWPindex +=1

                #check if end of trajectory
                if self.curWPindex == len(self.waypoints):
                    break


    def connect(self):
        self.s = socket.socket()
        print 'Server started!'
        print 'Waiting for clients...'

        self.s.bind((self.host, self.port))  # Bind to the port
        self.s.listen(5)  #Wait for client connection, up to 5.

    def listen(self):
        # Server must be in while open to stay open and listen
        while self.ON:
            try:
                c,addr = self.s.accept()  # Establish connection with client.
                print colored(('Got connection from' + str(addr)),"magenta")
                self.server = Thread(target=self.newClient, args=((c,addr)))
                self.server.daemon = True
                self.wpcheck = Thread(target=self.checkWP)
                self.wpcheck.daemon = True
                self.server.start()
                self.wpcheck.start()
            except:
                print colored("Socket Error.", "red")
                sys.exit()

        self.s.close()

    #Stop server manually
    def close(self):
        self.ON = False
        print "Server closed."

'''Main loop for testing'''
s = Server(50004,"127.0.0.1")
s.connect()
s.listen()
