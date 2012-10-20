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

NB_AXIS = 1

# Pins definition
#TODO : change me!
POTENT_PINS = [0, 1, 2, 3] # Potentiometer pin for each axis
MOTOR_DIR_PIN = 3
MOTOR_ENABLE_PIN = 5
MOTOR_STEP_PIN = 2

# Motor settings

MOTOR_MIN_DELAY = 3 # in microsecs
MOTOR_MAX_DELAY = 5000
MOTOR_CRUISE_DELAY = 100
# Define a "dead band" so that the motor doesn't move within a range of values
MOTOR_DEAD_BAND_INF = 490
MOTOR_DEAD_BAND_SUP = 530

# Constants used when computing mapping between input value (potentiometer) and output value (motor)
MAP_HIGH = abs(MOTOR_MAX_DELAY - MOTOR_MIN_DELAY) / abs(1023 - MOTOR_DEAD_BAND_SUP)
MAP_LOW = (MOTOR_MAX_DELAY - MOTOR_MIN_DELAY) / (1023 - 0)

# Record settings
MAX_RECORDS = 100000 # For each axis
RECORD_FREQ = 10 # in ms

# Global variables

cur_cmd = None
last_record_time = 0
nb_records = 0

cur_records = [[], [], [], []] # For 4 axis
cur_nb_impuls = [0, 0, 0, 0] # For 4 axis
start_record_nb_impuls = [0, 0, 0, 0] # For 4 axis

# Utility functions
def display_cur_nb_impuls():
    #print("Axis %i : %i impulsions" %(0, nb_impuls))
    for axis in range(NB_AXIS):
        print("Axis %i : %i impulsions" %(axis, cur_nb_impuls[axis]))

def display_cur_infos():
    display_cur_nb_impuls()

# Internal functions

def set_zero_point():
    for axis_nb in range(NB_AXIS):
        cur_nb_impuls[axis_nb] = 0

def move_one_axis(axis_nb, potent_value):
    if is_moving_back(potent_value):
        motor_delay = map_analog_to_delay_low(potent_value, 0, 
                                              MOTOR_DEAD_BAND_INF, MOTOR_MIN_DELAY, MOTOR_MAX_DELAY)
        move_one_step(axis_nb, motor_delay, False)
    elif is_moving_forward(potent_value):
        motor_delay = map_analog_to_delay_high(potent_value, MOTOR_DEAD_BAND_SUP, 1023, 
                                               MOTOR_MIN_DELAY, MOTOR_MAX_DELAY)
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
    cur_record_time = millis()
    #print("time : %d : %d" % (last_record_time, cur_record_time))
    for axis_nb in range(NB_AXIS):  # Use threads?
        potent_value = get_potent_value(axis_nb)
        move_one_axis(axis_nb, potent_value)
        # Do sampling
        if (cur_record_time - last_record_time) >= RECORD_FREQ:
            cur_records[axis_nb].append("%d;%d" % (cur_record_time, cur_nb_impuls[axis_nb]))
            last_record_time = millis()
            nb_records += 1

def move_one_step(axis_nb, motor_delay, is_forward):
    #TODO : change me!
    #digitalWrite(STEP_PIN, LOW)
    #digitalWrite(STEP_PIN, HIGH)
    if is_forward:
        cur_nb_impuls[axis_nb] += 1
        #digitalWrite(DIR_PIN, HIGH) #TODO : change me!
    else:
        cur_nb_impuls[axis_nb] -= 1
        #digitalWrite(DIR_PIN, LOW) #TODO : change me!
    delayMicroseconds(motor_delay) # In microsecs

def get_potent_value(axis_nb):
    # TODO : read value from pin
    return 900

def is_moving_forward(potent_value):
    if (potent_value >= MOTOR_DEAD_BAND_SUP):
        return True
    return False

def is_moving_back(potent_value):
    if (potent_value <= MOTOR_DEAD_BAND_INF):
        return True
    return False

# Motion control functions

def cmd_stop():
    global cur_cmd
    print("\n*** Stop! ***")
    display_cur_infos()
    # TODO : set enable pin to low
    
    cur_cmd = None
    
def cmd_record():
    print("\n*** Recording... ****")
    global last_record_time
    global start_record_nb_impuls
    start_record_nb_impuls = cur_nb_impuls[:]
    last_recordTime = 0
    while cur_cmd == 'r' and nb_records < MAX_RECORDS:
        record_one_movement()
    print("%d values recorded" % nb_records)
    print("sr : %d" % start_record_nb_impuls[0])
    print("cr : %d" % cur_nb_impuls[0])
    print("End record.")

def cmd_replay_cur_record():
    global cur_cmd
    print("\n*** Replaying current record... ***")
    print("Done.")
    cur_cmd = None

def cmd_display_cur_record():
    global cur_cmd
    print("\n*** Record content ***")
    size = len(cur_records[0])
    for i in range(size):
        for axis_nb in range(NB_AXIS):
            print("%d - Axis %d : %s" % (i, axis_nb, cur_records[axis_nb][i]))
    cur_cmd = None

def cmd_erase_cur_record():
    global current_cmd
    for record in cur_records:
        record = []
    print("\n*** Current record deleted. ***")
    cur_cmd = None

def cmd_goto_last_record_start():
    global cur_cmd
    print("\n*** Going to start point of last record... ***")
    move_to_last_record_start()
    print("Done.")
    cur_cmd = None

def cmd_move():
    print("\n*** Moving... ***")
    while cur_cmd == 'm':
        move()
    print("End moving.")

def cmd_set_zero_point():
    global cur_cmd
    set_zero_point()
    print("\n*** Zero point set ***")
    cur_cmd = None

def cmd_goto_zero_point():
    global cur_cmd
    print("\n*** Going to zero point... ***")
    move_to_zero()
    print("Done.")
    cur_cmd = None

# Map from analog value to delay for motor when direction is "high"
def map_analog_to_delay_high(x, in_min, in_max, out_min, out_max):
    #return out_max - (x - in_min) * (abs(out_max - out_min) / abs(in_max - in_min))
    #print("mapAnalogToDelayHigh : %f" % (out_max - (x - in_min) * MAP_HIGH))
    return int(round(out_max - (x - in_min) * MAP_HIGH))

# Map from analog value to delay for motor when direction is "low"
def map_analog_to_delay_low(x, in_min, in_max, out_min, out_max):
    #return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
    #print("mapAnalogToDelayLow : %f" % ((x - in_min) * MAP_LOW + MOTOR_MIN_DELAY))
    return int(round((x - in_min) * MAP_LOW + MOTOR_MIN_DELAY))

# Command simulation

def ctrlc(signum, frame):
    quit()

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
    'er': cmd_erase_cur_record,
    'g': cmd_goto_last_record_start,
    'h': cmd_goto_zero_point,
    'i': cmd_display_cur_infos,
    'm': cmd_move,
    'p': cmd_replay_cur_record,
    's': cmd_stop,
    'r': cmd_record,
    'rd': cmd_display_cur_record,
    'q': cmd_quit,
    'z': cmd_set_zero_point,
}

def get_external_cmd(signum, frame):
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

signal.signal(signal.SIGUSR1, get_external_cmd)
signal.signal(signal.SIGINT, ctrlc) # CTRL-C

def setup():
    global sock
    with open(PID_FILE, "w") as f:
        f.write(str(os.getpid()))
    print("PID : %d" % os.getpid())
    sock = my_socket.SocketServer()
    print("Socket server initialized")
    
    # TODO set PIN mode
    
    print("Setup done, waiting for a command.")

def loop():
    if cur_cmd != None:
        commands[cur_cmd]()

run(setup, loop)
