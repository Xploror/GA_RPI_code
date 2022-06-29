# -*- coding: utf-8 -*-
"""
Created on Wed Mar 06 16:02:00 2019

@author: Sachchit Vekaria
@Organization: General Aeronautics Pvt Ltd
"""

# import necessary modules
import time
from util.gacommonutil import ScheduleTask, send_remaining_msg, CommonData, CompanionComputer
import logging

class GA3CompanionComputer(CompanionComputer):
    def __init__(self):
        # Initialize super class
        

# Empty data stroage for reference
class Data(CommonData):
    def __init__(self):
        super().__init__()
        self.vx = 0
        self.vy = 0
        self.vz = 0
        self.currentWP = 0
        self.rfndDist = 0

def set_data_stream(statusData):
    # data rate of more than 100 Hz is not possible as recieving loop is set to run at interval of 0.01 sec
    # don't change that as it affects other vehicles as well

    # request data to be sent at the given rate
    # arguments are (target_system, target_component, stream_type, frequency in Hz, stop(0) or start(1))

    with statusData.lock:
        # stop data which are coming automatically to stop recieving unnecessary messeges
        statusData.msgList.append(statusData.mavutil.mavlink.MAVLink_request_data_stream_message(statusData.mavConnection.target_system, statusData.mavConnection.target_component, statusData.mavutil.mavlink.MAV_DATA_STREAM_ALL, 4, 0))


        #statusData.msgList.append(statusData.mavutil.mavlink.MAVLink_request_data_stream_message(statusData.mavConnection.target_system, statusData.mavConnection.target_component, statusData.mavutil.mavlink.MAV_DATA_STREAM_ALL, 1, 1))
        #statusData.msgList.append(statusData.mavutil.mavlink.MAVLink_request_data_stream_message(statusData.mavConnection.target_system, statusData.mavConnection.target_component, statusData.mavutil.mavlink.MAV_DATA_STREAM_RAW_SENSORS, 1, 1))
        statusData.msgList.append(statusData.mavutil.mavlink.MAVLink_request_data_stream_message(statusData.mavConnection.target_system, statusData.mavConnection.target_component, statusData.mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS, 2, 1))
        statusData.msgList.append(statusData.mavutil.mavlink.MAVLink_request_data_stream_message(statusData.mavConnection.target_system, statusData.mavConnection.target_component, statusData.mavutil.mavlink.MAV_DATA_STREAM_RC_CHANNELS, 2, 1))
        #statusData.msgList.append(statusData.mavutil.mavlink.MAVLink_request_data_stream_message(statusData.mavConnection.target_system, statusData.mavConnection.target_component, statusData.mavutil.mavlink.MAV_DATA_STREAM_RAW_CONTROLLER, 1, 1))
        statusData.msgList.append(statusData.mavutil.mavlink.MAVLink_request_data_stream_message(statusData.mavConnection.target_system, statusData.mavConnection.target_component, statusData.mavutil.mavlink.MAV_DATA_STREAM_POSITION, 5, 1))
        #statusData.msgList.append(statusData.mavutil.mavlink.MAVLink_request_data_stream_message(statusData.mavConnection.target_system, statusData.mavConnection.target_component, statusData.mavutil.mavlink.MAV_DATA_STREAM_EXTRA1, 1, 1))
        #statusData.msgList.append(statusData.mavutil.mavlink.MAVLink_request_data_stream_message(statusData.mavConnection.target_system, statusData.mavConnection.target_component, statusData.mavutil.mavlink.MAV_DATA_STREAM_EXTRA2, 1, 1))
        statusData.msgList.append(statusData.mavutil.mavlink.MAVLink_request_data_stream_message(statusData.mavConnection.target_system, statusData.mavConnection.target_component, statusData.mavutil.mavlink.MAV_DATA_STREAM_EXTRA3, 2, 1))

    logging.info("Stream Rate have been set")

def handle_sensor(statusData):
    pass

def handle_messeges(recievedMsg, statusData):
    with statusData.lock:
        if recievedMsg.get_type() == "MISSION_CURRENT":
            statusData.currentWP = recievedMsg.seq
            return
        
        if recievedMsg.get_type() == "DISTANCE_SENSOR":
            statusData.rfndDist = 0.01*recievedMsg.current_distance
            return

def update(statusData):
    ############################### Defining Variables ##############################
    # Frequency of sending the mavlink meesages
    freq = float(100)
    
    #################################################################################
    
    # set data stream rate for this vehicle
    set_data_stream(statusData)

    ############################# Scheduled tasks #####################################
    
    # make sure all scheduled tasks are finite/non-blocking tasks for clean exit
    #
    # ALSO MAKE SURE SCHEDULED TASKS TAKE MUCH LESS TIME THAN THE TIME INTERVAL 
    
    # handle sensors
    handle_sensor(statusData)

    # Schedule message sending tasks
    # schTaskList.append(ScheduleTask(timeInterval, functionName, functionArguments))
    statusData.schTaskList.append(ScheduleTask(1./freq, send_remaining_msg, statusData))

    ####################################################################################

    # Final infinite loop on tasks which requires data generated/recieved
    # from Scheduled task and takes long time to run
    # This loop should wait for all tasks to be finished before going to next iteration
    while True:
        try:
            
            # Prevent unnecessary resource usage by this program
            time.sleep(0.2)

            ############################# Sequential Tasks ###############################
#            logging.info("Main Loop Running")
            ##############################################################################

        except KeyboardInterrupt:
            # stop scheduled tasks
            for task in statusData.schTaskList:
                task.stop()

            # again raise keyboardinterrupt so that outer try except can also handle the clean thread exits
            raise KeyboardInterrupt
        

        

