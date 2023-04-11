import math

GRAV = 5
NUM_ITERATIONS = 1000
SLEEP_VAL = (1.0/144.0)
#SLEEP_VAL = 0

SIN_FACTOR = .01

AMPLITUDE1 = math.pi/2
FREQUENCY1 = NUM_ITERATIONS/100
PHASE1 = 0


AMPLITUDE2 = math.pi/2
FREQUENCY2 = NUM_ITERATIONS/100
PHASE2 = math.pi/4

NUM_GENERATIONS = 20
POPULATION_SIZE = 25

num_sensor_neurons = 14
num_motor_neurons = 13
motorJointRange = math.pi/2