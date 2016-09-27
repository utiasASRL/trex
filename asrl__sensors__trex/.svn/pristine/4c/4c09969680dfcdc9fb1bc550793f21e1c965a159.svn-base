#!/usr/bin/env python

# asrl__trex__sensor/lj_* variables
use_new_code_version = True
use_new_code_version_spi = True
# ~150 is the fastest it can go on a single sensor read name
    # ~140 it can go on multi read names
    # ~90 is the SPI call

#~140,120 same as 150
#110, SPI can only go ~95, the rest 110
# It looks like single call reads go to a max of 140 hz
    # The more multi call reads, the lower this becomes, thus imposing an upper limit.
# It looks like the maximum call to SPI is 110 hz
# ~100 looks like to be optimal for not "dropping" msgs (by drop I mean msg out on the correct time interval)
# For >6 sensors, it looked like adding more topics for the register to listen on helps this,
#   but we cannot avoid the speed of reading multiple sensor names from LJ.
# The max hz speed depends on the speed of the slowest LJ api call!
#loop_hz = 110   # This is the hightest we can go, without msg delay
loop_hz = 60

to_register_topic_name_split_1 = '/register_lj_sensor_1'
to_register_log_name_split_1 = "lj_registar_1_log.txt"

to_register_topic_name_split_2 = '/register_lj_sensor_2'
to_register_log_name_split_2 = "lj_registar_2_log.txt"

lji_callers_unregister_mag = 10

print_time = True
print_log = True

# Each sesnor's interface caller service names
angle_lji_caller_sub =  '/angle_lj_response'
force_lji_caller_sub =  '/forceCell_lj_response'
length_lji_caller_sub =  '/length_lj_response'
mag_lji_caller_sub =  '/mag_lj_response'
motor_lji_caller_sub =  '/motor_lj_response'
pitchPot_lji_caller_sub =  '/pitchPot_lj_response'
