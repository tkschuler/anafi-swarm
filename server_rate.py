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

    def newClient(self,clientsocket, addr):
        while True:
            msg = clientsocket.recv(4096)
            print "data received:", msg
            clientsocket.send("ACK!")

        clientsocket.close()

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
                self.server.start()
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
