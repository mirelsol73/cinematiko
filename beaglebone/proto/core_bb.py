#!/usr/bin/python
# -*- coding: utf-8 -*-

'''
 Control a brushless motor using a potentiometer
 Works with BeagleBone rev A6
 Authors : Alexandre Massot and Marc Schneider
'''

from __future__ import division # Python 3 facility : 3/4 = 0.75 (and not 0 like in Python 2)
import signal, os
import my_socket

from bbio import *
from time import sleep

# Proto stuff will be removed in the future

sock = None
PID_FILE = "/tmp/core_bb_pid"

# Nb of axis = nb of potentiometers
NB_AXIS = 1

# Pins definition
# --- AIN0 = pin39, AIN2 = pin37 on header P9
POTENT_PINS = [AIN0, AIN2, AIN2, AIN2] # Potentiometer pin for each axis #TODO : change me!
MOTORS_PINS = [ #TODO change me!
                {'dir': USR3, 'enable': USR3, 'step': USR3}, 
                {'dir': USR3, 'enable': USR3, 'step': USR3}, 
                {'dir': USR3, 'enable': USR3, 'step': USR3}, 
                {'dir': USR3, 'enable': USR3, 'step': USR3}
            ]

# Motor settings

MOTOR_MIN_DELAY = 3 # in microsecs
MOTOR_MAX_DELAY = 5000
MOTOR_CRUISE_DELAY = 100

# Define a "dead band" so that the motor doesn't move within a range of values
POTENT_DEAD_BAND_INF = 490
POTENT_DEAD_BAND_SUP = 530
POTENT_MAX = 1023
POTENT_MIN = 0

# TODO : enable me?
MOTOR_NB_OF_IMPULS_PER_STEP = 20 # Nb of impuls that will be sent each time we want to move the motor of one step

# Record settings
MAX_RECORDS = 100000 # For each axis
RECORD_FREQ = 10 # in ms

# Global variables

cur_cmd = None
last_record_time = 0
nb_records = 0

cur_records = [[], [], [], []] # For 4 axis, earch axis will contain a list of dict : {'time':t, 'pos':x}
cur_nb_impuls = [0, 0, 0, 0] # For 4 axis
start_record_nb_impuls = [0, 0, 0, 0] # For 4 axis

# Utility functions
def display_cur_nb_impuls():
    #print("Axis %i : %i impulsions" %(0, nb_impuls))
    for axis in range(NB_AXIS):
        print("Position : Axis %i = %i impulsions" %(axis, cur_nb_impuls[axis]))

def display_cur_infos():
    display_cur_nb_impuls()

# Internal functions

def enable_motors():
    for motor_pin in MOTORS_PINS:
        digitalWrite(motor_pin['enable'], HIGH)

def disable_motors():
    for motor_pin in MOTORS_PINS:
        digitalWrite(motor_pin['enable'], LOW)

def set_zero_point():
    for axis_nb in range(NB_AXIS):
        cur_nb_impuls[axis_nb] = 0

def move_one_axis(axis_nb, potent_value):
    if potent_value <= POTENT_DEAD_BAND_INF: # Move back
        motor_delay = linear_translation(potent_value, POTENT_DEAD_BAND_INF, POTENT_MIN, 
                                                       MOTOR_MAX_DELAY, MOTOR_MIN_DELAY)
        move_one_step(axis_nb, motor_delay, False)
    elif if potent_value >= POTENT_DEAD_BAND_SUP: # Move forward
        motor_delay = linear_translation(potent_value, POTENT_DEAD_BAND_SUP, POTENT_MAX, 
                                         MOTOR_MAX_DELAY, MOTOR_MIN_DELAY)
        motor_delay = 3
        move_one_step(axis_nb, motor_delay, True)

def move():
    # Read value for each potentiometer
    for axis_nb in range(NB_AXIS):  # Use threads?
        potent_value = get_potent_value(axis_nb)
        move_one_axis(axis_nb, potent_value)

def move_to_zero():
    for axis_nb in range(NB_AXIS): # Use threads?
        move_relative(axis_nb, -cur_nb_impuls[axis_nb])

def move_to_last_record_start():
    for axis_nb in range(NB_AXIS): # Use threads?
        move_relative(axis_nb, start_record_nb_impuls[axis_nb] - cur_nb_impuls[axis_nb])

def move_relative(axis_nb, nb_impuls):
    print("Move relative (axis %d) : %d" % (axis_nb, nb_impuls))
    is_forward = nb_impuls >= 0
    nb_impuls = abs(nb_impuls)
    for i in range(nb_impuls):
        move_one_step(axis_nb, MOTOR_CRUISE_DELAY, is_forward)

def record_one_movement():
    global last_record_time
    global nb_records
    # Read value for each potentiometer
    record_value = False
    cur_record_time = millis()
    if (cur_record_time - last_record_time) >= RECORD_FREQ:
        record_value = True
        last_record_time = millis()
        nb_records += 1

    for axis_nb in range(NB_AXIS):  # Use threads?
        potent_value = get_potent_value(axis_nb)
        move_one_axis(axis_nb, potent_value)
        # Do sampling
        if record_value:
            cur_records[axis_nb].append({'time': cur_record_time, 'pos': cur_nb_impuls[axis_nb]})

def replay_movement():
    record_size = len(cur_records[0])
        
    i=0
    while i < (record_size - 1):
        if (cur_records[0][i+1]['time'] - cur_records[0][i]['time']) > 0:
            for axis_nb in range(NB_AXIS): # Use threads?
                nb_impuls_to_move = cur_records[axis_nb][i+1]['pos'] - cur_records[axis_nb][i]['pos']
                is_forward = nb_impuls_to_move >= 0
                nb_impuls_to_move = abs(nb_impuls_to_move)
                delay_ms_between_2_impuls = RECORD_FREQ / nb_impuls_to_move
                #print("%d:%d / %d:%d | Moving nb impuls : %d / delay : %f" % (cur_records[axis_nb][i]['time'], cur_records[axis_nb][i]['pos'], cur_records[axis_nb][i+1]['time'], cur_records[axis_nb][i+1]['pos'], nb_impuls_to_move, delay_ms_between_2_impuls))
                for k in range(nb_impuls_to_move):
                    move_one_step(axis_nb, round(delay_ms_between_2_impuls * 1000), is_forward)
        i += 1

def get_cur_record_length():
    # Use first axis to compute record length
    if len(cur_records[0]) > 0:
        return cur_records[0][-1]['time'] - cur_records[0][0]['time']
    else:
        return 0

def del_cur_records():
    for axis_record in cur_records:
        del axis_record[:]

def move_one_step(axis_nb, motor_delay, is_forward):
    digitalWrite(MOTORS_PINS[axis_nb]['step'], LOW)
    #TODO : add a short delay here?
    digitalWrite(MOTORS_PINS[axis_nb]['step'], HIGH)

    if is_forward:
        cur_nb_impuls[axis_nb] += 1
        digitalWrite(MOTORS_PINS[axis_nb]['dir'], HIGH)
    else:
        cur_nb_impuls[axis_nb] -= 1
        digitalWrite(MOTORS_PINS[axis_nb]['dir'], LOW)
    delayMicroseconds(motor_delay) # In microsecs

def get_potent_value(axis_nb):
    # TODO : return pin value
    val = analogRead(POTENT_PINS[axis_nb])
    #print("get potent val : %d" % val)
    return 1023

# Motion control functions

def cmd_stop():
    global cur_cmd
    disable_motors()
    print("\n*** Stop! ***")
    display_cur_nb_impuls()
    cur_cmd = None

def cmd_record():
    print("\n*** Recording... ****")
    global last_record_time
    global start_record_nb_impuls
    start_record_nb_impuls = cur_nb_impuls[:]
    last_recordTime = 0
    del_cur_records()
    while cur_cmd == 'r' and nb_records < MAX_RECORDS:
        record_one_movement()
    print("%d values recorded" % nb_records)
    print("End record.")

def cmd_replay_cur_record():
    global cur_cmd
    record_size = len(cur_records[0])
    print("\n*** Replaying current record... ***")
    print("Nb of records : %d" % record_size)
    print("Record length : %d ms" % get_cur_record_length())
    if record_size > 1:
        enable_motors()
        replay_movement()
        print("Done.")
        display_cur_nb_impuls()
    else:
        print("Record set must contain at least 2 items!")
    cur_cmd = None

def cmd_display_cur_record():
    global cur_cmd
    print("\n*** Record content ***")
    size = len(cur_records[0])
    for i in range(size):
        for axis_nb in range(NB_AXIS):
            print("%d - Axis %d : time = %d / pos = %d" % (i, axis_nb, cur_records[axis_nb][i]['time'], 
                                                           cur_records[axis_nb][i]['pos']))
    cur_cmd = None

def cmd_erase_cur_record():
    global cur_cmd
    del_cur_records()
    print("\n*** Current records deleted. ***")
    cur_cmd = None

def cmd_goto_last_record_start():
    global cur_cmd
    print("\n*** Going to start point of last record... ***")
    enable_motors()
    move_to_last_record_start()
    print("Done.")
    display_cur_nb_impuls()
    cur_cmd = None

def cmd_move():
    print("\n*** Moving... ***")
    enable_motors()
    start = millis()
    while cur_cmd == 'm':
        move()
    end = millis()
    print("End moving.")

def cmd_set_zero_point():
    global cur_cmd
    set_zero_point()
    print("\n*** Zero point set ***")
    cur_cmd = None

def cmd_goto_zero_point():
    global cur_cmd
    print("\n*** Going to zero point... ***")
    enable_motors()
    move_to_zero()
    print("Done.")
    display_cur_nb_impuls()
    cur_cmd = None

def linear_translation(x, in_min, in_max, out_min, out_max):
    return round((x - in_min) / (in_max - in_min) * (out_max - out_min) + out_min)

# Map from analog value to delay for motor when direction is "high"
def map_analog_to_delay_high(x, in_min, in_max, out_min, out_max):
    #return out_max - (x - in_min) * (abs(out_max - out_min) / abs(in_max - in_min))
    #print("mapAnalogToDelayHigh : %f" % (out_max - (x - in_min) * MAP_HIGH))
    return round(out_max - (x - in_min) * MAP_HIGH)

# Map from analog value to delay for motor when direction is "low"
def map_analog_to_delay_low(x, in_min, in_max, out_min, out_max):
    #return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
    #print("mapAnalogToDelayLow : %f" % ((x - in_min) * MAP_LOW + MOTOR_MIN_DELAY))
    return round((x - in_min) * MAP_LOW + MOTOR_MIN_DELAY)

# Command simulation

def ctrlc(signum, frame):
    print("\n *** CTRL-C! ***")
    os.remove(PID_FILE)
    sys.exit(0)

def cmd_quit():
    print("\n *** Quit! ***")
    os.remove(PID_FILE)
    sys.exit(0)

def cmd_display_cur_infos():
    global cur_cmd
    print("\n *** Current status ***")
    display_cur_infos()
    cur_cmd = None

commands = {
    'g': cmd_goto_last_record_start,
    'h': cmd_goto_zero_point,
    'i': cmd_display_cur_infos,
    'm': cmd_move,
    'p': cmd_replay_cur_record,
    's': cmd_stop,
    'r': cmd_record,
    'rd': cmd_display_cur_record,
    're': cmd_erase_cur_record,    
    'q': cmd_quit,
    'z': cmd_set_zero_point,
}

def on_external_cmd(signum, frame):
    '''Called when a command is sent from command.py'''
    
    global cur_cmd
    old_cmd = cur_cmd
    sock.accept()
    new_cmd = sock.get_data()
    sock.close_connection()
    if old_cmd in ['m', 'r', 'p', 'h', 'g']: # Motor is moving
        if new_cmd not in ['s', 'q']:
            print(">>> ERROR, motor is moving, allowed commands are 's', 'q'")
        else:
            cur_cmd = new_cmd
    else:
        cur_cmd = new_cmd

signal.signal(signal.SIGUSR1, on_external_cmd)
signal.signal(signal.SIGINT, ctrlc) # CTRL-C

def setup():
    global sock
    with open(PID_FILE, "w") as f:
        f.write(str(os.getpid()))
    print("PID : %d" % os.getpid())
    sock = my_socket.SocketServer()
    print("Socket server initialized")
    
    for motor_pins in MOTORS_PINS:
        for cmd in motor_pins:
            pinMode(motor_pins[cmd], OUTPUT)    
    
    print("Setup done, waiting for a command.")

def loop():
    if cur_cmd != None:
        commands[cur_cmd]()

run(setup, loop)
