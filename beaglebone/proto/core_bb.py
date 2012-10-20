#!/usr/bin/python
# -*- coding: utf-8 -*-

'''
 Control a brushless motor using a potentiometer
 Works with BeagleBone rev A6
 Authors : Alexandre Massot and Marc Schneider
'''

from bbio import *
import signal, os
from time import sleep

import subprocess as sub
import my_socket

current_cmd = None
sock = None
PID_FILE = "/tmp/core_bb_pid"

# Motion control functions

def stop():
    global current_cmd
    print("Stop!")
    current_cmd = None
    
def record():
    print("Recording...")
    while current_cmd == 'r':
        pass
    print("End record")

def replay_cur_record():
    global current_cmd
    print("Replaying current record...")
    print("Done.")
    current_cmd = None

def erase_cur_record():
    global current_cmd
    print("Erasing current record...")
    print("Done.")
    current_cmd = None

def goto_start_last_record():
    global current_cmd
    print("Going to start point of last record...")
    print("Done.")
    current_cmd = None

def move():
    print("Moving...")
    while current_cmd == 'm':
        pass
    print("End moving")

def set_zero_point():
    global current_cmd
    print("Zero point set")
    current_cmd = None

def goto_zero_point():
    global current_cmd
    print("Going to zero point...")
    print("Done.")
    current_cmd = None

# Command simulation

def ctrlc(signum, frame):
    quit()

def quit():
    print("Quit!")
    os.remove(PID_FILE)
    sys.exit(0)

commands = {
    'er': erase_cur_record,
    'g': goto_start_last_record,
    'h': goto_zero_point,
    'm': move,
    'p': replay_cur_record,
    's': stop,
    'r': record,
    'q': quit,
    'z': set_zero_point,
}

def get_external_cmd(signum, frame):
    '''Called when a command is sent from command.py'''
    
    global current_cmd
    print("Interrupt")
    sock.accept()
    current_cmd = sock.get_data()
    print("Cmd : %s" % current_cmd)
    sock.close_connection()

signal.signal(signal.SIGUSR1, get_external_cmd)
signal.signal(signal.SIGINT, ctrlc) # CTRL-C

def setup():
    global sock
    with open(PID_FILE, "w") as f:
        f.write(str(os.getpid()))
    sock = my_socket.SocketServer()
    print("Setup done")

def loop():
    if current_cmd != None:
        commands[current_cmd]()
    print("Waiting")
    sleep(2)

run(setup, loop)
