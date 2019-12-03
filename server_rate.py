import socket
import time
import sys
from termcolor import colored
import thread
from threading import Thread

'''This file opens a socketed server thread and parses incomming messages to send unique
data packets to multiple clients'''

class Server:
    def __init__(self,port,host):
        self.port = port
        self.host = host
        self.server = None
        self.ON = True
        self.waypoints = [1,2,3,4,5]
        self.curWP = self.waypoints[0]
        self.curWPindex = 0

        self.x = 0
        self.y = 0
        self.z = 0
        self.roll = 0
        self.pitch = 0
        self.yaw = 0

    def newClient(self,clientsocket, addr):
        #This listens for the incomming messages that client_rate.py is listening to ROS for
        while True:
            msg = clientsocket.recv(4096)
            print "data received:", msg
            pose = msg.split(',')
            clientsocket.send("ACK!")

            self.x = float(pose[0])
            self.y = float(pose[1])
            self.z = float(pose[2])
            self.roll = float(pose[3])
            self.pitch = float(pose[4])
            self.yaw = float(pose[5])

        clientsocket.close()

    def checkWP(self):
        #This is a 1D exaple for checking if the drone has made it to a current waypoint
        while True:
            if self.x == self.curWP:
                print colored("MADE IT TO WP!", "green")
                #send new drone control command
                self.curWPindex +=1
                self.curWP = self.waypoints[self.curWPindex]


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
s = Server(50001,"127.0.0.1")
s.connect()
s.listen()
