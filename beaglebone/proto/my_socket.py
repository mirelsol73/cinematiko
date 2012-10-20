#!/usr/bin/python
# -*- coding: utf-8 -*-

'''
Utility class used for command and core_bb to communicate via a socket
'''

import socket

HOST = ''
PORT = 50000
BACKLOG = 5
SIZE = 1024

class SocketServer():
    def __init__(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.bind((HOST, PORT))
        self.sock.listen(BACKLOG)
    
    def accept(self):
        self.client, self.address = self.sock.accept()
    
    def close_connection(self):
        self.client.close()
    
    def get_data(self):
        data = self.client.recv(SIZE)
        return data
    
    def shutdown(self):
        self.close_connection()
        self.sock.close()

class SocketClient():
    def __init__(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect((HOST, PORT))
    
    def send_data(self, message):
        self.sock.sendall(message)

    def shutdown(self):
        self.sock.close()
