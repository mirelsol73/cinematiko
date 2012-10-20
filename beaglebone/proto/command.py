#!/usr/bin/python
# -*- coding: utf-8 -*-

'''
Programm to send command to core_bb
'''

import os, signal, sys
import my_socket

sock = None
core_bb_pid = None
current_cmd = None

valid_commands = {
    'er': {'help': "Erase current record data"},
    'g': {'help': "Go to start point of last record"},
    'h': {'help': "Go to zero point"},
    'i': {'help': "Display infos"},
    'm': {'help': "Move"},
    'p': {'help': "Replay current record"},
    's': {'help': "Stop the system"},
    'r': {'help': "Record a movement"},
    'rd': {'help': "Display current record content"},
    'q': {'help': "Quit the program"},
    'z': {'help': "Set zero point"},
    'help': {'help': "Print help message"},
}

def send_cmd_to_core_bb(cmd):
    s = my_socket.SocketClient()
    s.send_data(cmd)
    print("... command sent.")

def help():
    for cmd in valid_commands:
        print("'%s' : %s" % (cmd, valid_commands[cmd]['help']))

def setup():
    global core_bb_pid
    try:
        with open("/tmp/core_bb_pid", "r") as f:
            core_bb_pid = f.read()
    except IOError:
        print("Can't find pid of core_bb, launch it first!")
        sys.exit()
    print("core_bb pid is %s" % core_bb_pid)

def quit():
    sys.exit()

def ctrlc(signum, frame):
    quit()

def wait_for_cmd():
    global current_cmd
    
    while True:
        input = raw_input("Command : ")
        if input == 'help':
            help()
        elif input in valid_commands:
            current_cmd = input
            send_cmd_to_core_bb(current_cmd)
            os.kill(int(core_bb_pid), signal.SIGUSR1)
            if current_cmd == 'q':
                quit()
        else:
            print("Unknow command : %s" % input)
            help()

signal.signal(signal.SIGINT, ctrlc) # CTRL-C

if __name__ == '__main__':
    setup()
    wait_for_cmd()

