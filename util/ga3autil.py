# -*- coding: utf-8 -*-
"""
Created on Wed Mar 06 16:02:00 2019

@author: Sachchit Vekaria
@Organization: General Aeronautics Pvt Ltd
"""

# import necessary modules
# import necessary modules
import time
from util.gacommonutil import CompanionComputer, mavutil, path, dist_between_lat_lon, ScheduleTask, State, CountDown
import logging
from util.ga3apayloadutil import AgriPayload, PIBStatus, FlowSensor
import numpy as np
import sys
import threading
import os
from collections import OrderedDict

class GA3ACompanionComputer(CompanionComputer):
    def __init__(self, sitlType, sitlPort):
        # Initialize super class
        super().__init__(sitlType, sitlPort)

        # Threading Lock for TestCompanionComputer Class
        self.handleRecievedMsgThread = None

        # Spraying Mission Related
        self.startWP = 1
        self.endWP = 2
        self.missionYaw = 0
        self.missionAlt = 3       # m
        self.missionOn = False
        self.previousMode = "UNKNOWN"

        # Resume Mission Related
        self.RTLLat = -200
        self.RTLLon = -200
        self.RTLWP = 0
        self.clearanceAlt = 10    # m
        self.resumeOn = False
        self.resumeState = 0
        self.resumeSendingCounter = 0
        self.holdBeforeStartingAutoCountDown = CountDown(5)

        # Payload Related
        self.agriPayload = AgriPayload(self.isSITL)
        self.maxSpeed = 5 # m/s
        self.maxSpeedCount = 0

        # Read Agri Mission File
        self.read_mission_file()
        
        # Define Parameter Dictionary
        # Format {"Parameter_Name": [index, stter_function, getter_function, min, max]}
        self.paramDict = OrderedDict()
        i = 1
        self.paramDict["PAYLOAD"] = [i,
                                     self.agriPayload.set_remaining_payload, 
                                     self.agriPayload.get_remaining_payload, 
                                     0, 
                                     17]
        i = i+1
        self.paramDict["CLEARANCE_ALT"] = [i,
                                           self.set_clearance_altitude, 
                                           self.get_clearance_altitude, 
                                           5, 
                                           35]
        i = i+1
        self.paramDict["PESTI_PER_ACRE"] = [i,
                                            self.agriPayload.set_pestiscide_per_acre, 
                                            self.agriPayload.get_pestiscide_per_acre, 
                                            1, 
                                            20]   
        i = i+1               
        self.paramDict["SWATH"] = [i,
                                   self.agriPayload.set_swath, 
                                   self.agriPayload.get_swath, 
                                   1, 
                                   8]
        i = i+1
        self.paramDict["MAX_FLOW_RATE"] = [i,
                                           self.agriPayload.set_max_flow_rate, 
                                           self.agriPayload.get_max_flow_rate, 
                                           0.3, 
                                           5]
        i = i+1
        self.paramDict["DROPLET_SIZE"] = [i,
                                          self.agriPayload.set_particle_size, 
                                          self.agriPayload.get_particle_size, 
                                          50, 
                                          600]
        i = i+1                  
        self.paramDict["FLO_SEN_CAL_M"] = [i,
                                           self.agriPayload.flowSensor.set_calib_factor_multiplier, 
                                           self.agriPayload.flowSensor.get_calib_factor_multiplier, 
                                           0.5, 
                                           1.5]
        # i = i+1
        # self.paramDict["PIB_ENABLED"] = [i,
        #                                  self.agriPayload.pibStatus.set_pib_enabled,
        #                                  self.agriPayload.pibStatus.get_pib_enabled,
        #                                  0,
        #                                  1]

        i = i+1
        self.paramDict["NOZZ_TYPE"] = [i,
                                           self.agriPayload.set_nozz_type,
                                           self.agriPayload.get_nozz_type,
                                           0,
                                           10]

        i = i+1
        self.paramDict["NOZZ_MIN_PWM"] = [i,
                                           self.agriPayload.set_nozz_min_pwm,
                                           self.agriPayload.get_nozz_min_pwm,
                                           0,
                                           2000]

        i = i+1
        self.paramDict["NOZZ_MAX_PWM"] = [i,
                                           self.agriPayload.set_nozz_max_pwm,
                                           self.agriPayload.get_nozz_max_pwm,
                                           1000,
                                           20000]

        i = i+1
        self.paramDict["NOZZ_NODRIP_PWM"] = [i,
                                           self.agriPayload.set_nozz_nodrip_pwm,
                                           self.agriPayload.get_nozz_nodrip_pwm,
                                           0,
                                           20000]
        i = i+1
        self.paramDict["NOZZ_COUNT"] = [i,
                                           self.agriPayload.set_nozz_count,
                                           self.agriPayload.get_nozz_count,
                                           1,
                                           6]


        self.numParams = len(self.paramDict.keys())
        
        # Read parameter file
        self.load_params_from_file()
        
    def init(self):
        super().init()

        if self.agriPayload.nozzType is 0: #micromiser_with_dcu
            self.agriPayload.pibStatus.pibEnabled = True
            self.agriPayload.pibStatus.init()
            # Schedule the payload sensors readings
            self.scheduledTaskList.append(ScheduleTask(0.15, self.agriPayload.pibStatus.update))

        self.scheduledTaskList.append(ScheduleTask(0.2, self.agriPayload.flowSensor.calc_flow_rate))


        # set data stream rate
        self.set_data_stream()

        # start our recieving message handling loop
        self.handleRecievedMsgThread = threading.Thread(target=self.handle_recieved_message)
        self.handleRecievedMsgThread.start()

        # Start Agri Loop
        self.scheduledTaskList.append(ScheduleTask(0.2, self.update))


        while True:
            time.sleep(1)

    def set_data_stream(self):
        # data rate of more than 100 Hz is not possible as recieving loop is set to run at interval of 0.01 sec
        # don't change that as it affects other vehicles as well

        # request data to be sent at the given rate
        # arguments are (target_system, target_component, stream_type, frequency in Hz, stop(0) or start(1))

        # stop data which are coming automatically to stop recieving unnecessary messeges
        self.add_new_message_to_sending_queue(mavutil.mavlink.MAVLink_request_data_stream_message(self.mavlinkInterface.mavConnection.target_system, self.mavlinkInterface.mavConnection.target_component, mavutil.mavlink.MAV_DATA_STREAM_ALL, 4, 0))


        #self.add_new_message_to_sending_queue(mavutil.mavlink.MAVLink_request_data_stream_message(self.mavlinkInterface.mavConnection.target_system, self.mavlinkInterface.mavConnection.target_component, mavutil.mavlink.MAV_DATA_STREAM_ALL, 1, 1))
        #self.add_new_message_to_sending_queue(mavutil.mavlink.MAVLink_request_data_stream_message(self.mavlinkInterface.mavConnection.target_system, self.mavlinkInterface.mavConnection.target_component, mavutil.mavlink.MAV_DATA_STREAM_RAW_SENSORS, 1, 1))
        self.add_new_message_to_sending_queue(mavutil.mavlink.MAVLink_request_data_stream_message(self.mavlinkInterface.mavConnection.target_system, self.mavlinkInterface.mavConnection.target_component, mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS, 2, 1))
        self.add_new_message_to_sending_queue(mavutil.mavlink.MAVLink_request_data_stream_message(self.mavlinkInterface.mavConnection.target_system, self.mavlinkInterface.mavConnection.target_component, mavutil.mavlink.MAV_DATA_STREAM_RC_CHANNELS, 2, 1))
        #self.add_new_message_to_sending_queue(mavutil.mavlink.MAVLink_request_data_stream_message(self.mavlinkInterface.mavConnection.target_system, self.mavlinkInterface.mavConnection.target_component, mavutil.mavlink.MAV_DATA_STREAM_RAW_CONTROLLER, 1, 1))
        self.add_new_message_to_sending_queue(mavutil.mavlink.MAVLink_request_data_stream_message(self.mavlinkInterface.mavConnection.target_system, self.mavlinkInterface.mavConnection.target_component, mavutil.mavlink.MAV_DATA_STREAM_POSITION, 5, 1))
        self.add_new_message_to_sending_queue(mavutil.mavlink.MAVLink_request_data_stream_message(self.mavlinkInterface.mavConnection.target_system, self.mavlinkInterface.mavConnection.target_component, mavutil.mavlink.MAV_DATA_STREAM_EXTRA1, 1, 1))
        #self.add_new_message_to_sending_queue(mavutil.mavlink.MAVLink_request_data_stream_message(self.mavlinkInterface.mavConnection.target_system, self.mavlinkInterface.mavConnection.target_component, mavutil.mavlink.MAV_DATA_STREAM_EXTRA2, 1, 1))
        self.add_new_message_to_sending_queue(mavutil.mavlink.MAVLink_request_data_stream_message(self.mavlinkInterface.mavConnection.target_system, self.mavlinkInterface.mavConnection.target_component, mavutil.mavlink.MAV_DATA_STREAM_EXTRA3, 2, 1))

        logging.info("Stream Rate have been set")

    def handle_recieved_message(self):
        while True:
            if self.killAllThread.is_set():
                break
            recievedMsg = self.get_new_message_from_recieving_queue()

            if recievedMsg is not None:
                if recievedMsg.get_type() == "RC_CHANNELS":
                    if recievedMsg.chan7_raw > 1700:
                        self.agriPayload.payloadTesting = 2
                    elif recievedMsg.chan7_raw < 1700 and recievedMsg.chan7_raw > 1300:
                        self.agriPayload.payloadTesting = 1
                        self.agriPayload.dripStopCountDown.reset()
                    else:
                        if self.agriPayload.payloadTesting != 0:
                            self.agriPayload.dripStopCountDown.start()
                        self.agriPayload.payloadTesting = 0

                if recievedMsg.get_type() == "PARAM_SET":
                    #paramId = recievedMsg.param_id.strip(b"\x00").decode()
                    paramId = recievedMsg.param_id
                    try:
                        paramId = recievedMsg.param_id.replace(b'\x00',b'')
                        paramId = paramId.decode()
                    except:
                        pass
                    paramValue = recievedMsg.param_value
                    if paramId in self.paramDict.keys():
                        if paramValue >= self.paramDict[paramId][3] and paramValue <= self.paramDict[paramId][4]:
                            self.paramDict[paramId][1](paramValue)
                            self.add_new_message_to_sending_queue(mavutil.mavlink.MAVLink_param_value_message(paramId.encode(),
                                                                                                              self.paramDict[paramId][2](),
                                                                                                              mavutil.mavlink.MAV_PARAM_TYPE_REAL64,
                                                                                                              self.numParams,
                                                                                                              self.paramDict[paramId][0]))
                    
                    self.save_params_to_file()

                if recievedMsg.get_type() == "PARAM_REQUEST_READ":
                    paramId = recievedMsg.param_id
                    try:
                        paramId = recievedMsg.param_id.replace(b'\x00',b'')
                        paramId = paramId.decode()
                    except:
                        pass
                    if paramId in self.paramDict.keys():
                        self.add_new_message_to_sending_queue(mavutil.mavlink.MAVLink_param_value_message(paramId.encode(),
                                                                                                          self.paramDict[paramId][2](),
                                                                                                          mavutil.mavlink.MAV_PARAM_TYPE_REAL64,
                                                                                                          self.numParams,
                                                                                                          self.paramDict[paramId][0]))
                        
                if recievedMsg.get_type() == "GA3A_MISSION_CMD":
                    if recievedMsg.start_wp > 0 and recievedMsg.end_wp > recievedMsg.start_wp:
                        self.startWP = recievedMsg.start_wp
                        self.endWP = recievedMsg.end_wp
                        if recievedMsg.mission_alt > 100:
                            self.missionAlt = recievedMsg.mission_alt/100.
                        self.missionYaw = recievedMsg.mission_yaw
                        self.missionOn = True
                        self.RTLLat = -200
                        self.RTLLon = -200
                        self.RTLWP = 0
                        self.write_mission_file()
                    
                    # This message will be recieved and mission is cleared
                    if recievedMsg.start_wp == -1 and recievedMsg.end_wp == -1:
                        self.startWP = recievedMsg.start_wp
                        self.endWP = recievedMsg.end_wp
                        if recievedMsg.mission_alt > 100:
                            self.missionAlt = recievedMsg.mission_alt/100.
                        self.missionYaw = recievedMsg.mission_yaw
                        self.missionOn = False
                        self.RTLLat = -200
                        self.RTLLon = -200
                        self.RTLWP = 0
                        self.write_mission_file()

                if recievedMsg.get_type() == "GA3A_RESUME_CMD":
                    if recievedMsg.do_resume == 1 and not self.isFlying and not self.resumeOn:
                        self.resumeState = 1
                        self.resumeOn = True

                super().handle_recieved_message(recievedMsg)
            else:
                time.sleep(0.01)
                
    def load_params_from_file(self):
        if not os.path.isfile('agri_param'):
            self.save_params_to_file()
        
        with open('agri_param', 'r') as f:
            for line in f:
                
                # Ignore empty lines
                if len(line) > 0:
                    
                    # Split the line
                    splittedLine = line.split(",")
                    
                    # If only 2 fields are then only accept for reading
                    if len(splittedLine) == 2:
                        
                        # Remove empty spaces if the are
                        key = splittedLine[0].strip()
                        value = float(splittedLine[1].strip())
                        
                        if key in self.paramDict.keys():
                            self.paramDict[key][1](value)
        
        # In case some parameters are removed and some are added, this step will 
        # make sure update is done proeperly
        self.save_params_to_file()
                        
    
    def save_params_to_file(self):
        with open('agri_param', 'w') as f:
            for key in self.paramDict.keys():
                f.write(key + "," + str(self.paramDict[key][2]()) + "\n")

                
    def get_clearance_altitude(self):
        return self.clearanceAlt
    
    def set_clearance_altitude(self,value):
        self.clearanceAlt = value
            
    def resume_mission(self):
        sendCount = 10
        logging.info("Resume, %d, %d, %d"%(self.resumeSendingCounter, self.resumeOn, self.resumeState))
        
        if self.resumeOn:
            logging.info("Resume, " + self.currentMode)
            logging.info("Resume, %f, %f, %f"%(self.terrainAlt, self.missionAlt, self.clearanceAlt))
            logging.info("Resume, %f, %f"%(np.math.degrees(self.yaw), self.missionYaw))
            logging.info("Resume, %.7f, %.7f, %.7f, %.7f"%(self.RTLLat, self.RTLLon, self.lat, self.lon))
            logging.info("Resume, %d, %d"%(self.holdBeforeStartingAutoCountDown.started, self.holdBeforeStartingAutoCountDown.finished))
            # Check vehicle is armed or not
            if self.isArmed:
                # First change to GUIDED mode
                if self.resumeState == 1:
                    if self.currentMode == "GUIDED":
                        self.resumeSendingCounter = 0
                        self.resumeState = 2
                    else:
                        if self.resumeSendingCounter < sendCount:
                            self.add_new_message_to_sending_queue(mavutil.mavlink.MAVLink_set_mode_message(self.mavlinkInterface.mavConnection.target_system,
                                                                                                           mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                                                                                                           4) # GUIDED
                                                                  )
                            self.resumeSendingCounter = self.resumeSendingCounter + 1
                        return
            else:
                # If not armed, Go out of resume state
                self.resumeSendingCounter = 0
                self.resumeOn = False
                self.resumeState = 0


            # TakeOff
            if self.resumeState == 2:
                if self.terrainAlt > 1:
                    self.resumeSendingCounter = 0
                    self.resumeState = 3
                else:
                    if self.resumeSendingCounter < sendCount:
                        self.add_new_message_to_sending_queue(mavutil.mavlink.MAVLink_command_long_message(self.mavlinkInterface.mavConnection.target_system,
                                                                                                           self.mavlinkInterface.mavConnection.target_component,
                                                                                                           mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, # command
                                                                                                           0, # confirmation
                                                                                                           0, # param1
                                                                                                           0, # param2
                                                                                                           0, # param3
                                                                                                           0, # param4
                                                                                                           0, # param5
                                                                                                           0, # param6
                                                                                                           4) # param7 (alt)
                                                              )
                        self.resumeSendingCounter = self.resumeSendingCounter + 1
                    return

            # Goto Clearance Altitude
            if self.resumeState == 3:
                if self.terrainAlt > (self.clearanceAlt-1):
                    self.resumeSendingCounter = 0
                    self.resumeState = 4
                else:
                    if self.resumeSendingCounter < sendCount:
                        self.add_new_message_to_sending_queue(mavutil.mavlink.MAVLink_set_position_target_global_int_message(0, # Time from boot (Irrelevant)
                                                                                                                             self.mavlinkInterface.mavConnection.target_system,
                                                                                                                             self.mavlinkInterface.mavConnection.target_component,
                                                                                                                             mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT, # Frame
                                                                                                                             0b110111111000, # Command Mask, Only position command
                                                                                                                             int(self.lat*1e7), # LAT
                                                                                                                             int(self.lon*1e7), # LON
                                                                                                                             self.clearanceAlt,    # Alt
                                                                                                                             0,     # vx
                                                                                                                             0,     # vy
                                                                                                                             0,     # vz
                                                                                                                             0,     # ax
                                                                                                                             0,     # ay
                                                                                                                             0,     # az
                                                                                                                             0,     # yaw
                                                                                                                             0)     # yaw_rate
                                                              )
                        self.resumeSendingCounter = self.resumeSendingCounter + 1
                    return

            # Align heading to mission heading
            if self.resumeState == 4:
                if abs(np.math.degrees(self.yaw)-self.missionYaw) < 5:
                    self.resumeSendingCounter = 0
                    self.resumeState = 5
                else:
                    if self.resumeSendingCounter < sendCount:
                        self.add_new_message_to_sending_queue(mavutil.mavlink.MAVLink_command_long_message(self.mavlinkInterface.mavConnection.target_system,
                                                                                                           self.mavlinkInterface.mavConnection.target_component,
                                                                                                           mavutil.mavlink.MAV_CMD_CONDITION_YAW, # command
                                                                                                           0,                               # confirmation
                                                                                                           self.missionYaw,   # param1 (Angle respect to North)
                                                                                                           30,                              # param2 (rate deg/s)
                                                                                                           1,                               # param3 (Clockwise)
                                                                                                           0,                               # param4 (Absolute frame. North is 0)
                                                                                                           0,                               # param5
                                                                                                           0,                               # param6
                                                                                                           0)                               # param7
                                                              )
                        self.resumeSendingCounter = self.resumeSendingCounter + 1
                    return

            # Guide to Top of the RTL point
            if self.resumeState == 5:
                if dist_between_lat_lon(self.RTLLat, self.RTLLon, self.lat, self.lon) < 2.5:
                    self.resumeSendingCounter = 0
                    self.resumeState = 6
                else:
                    if self.resumeSendingCounter < sendCount:
                        self.add_new_message_to_sending_queue(mavutil.mavlink.MAVLink_set_position_target_global_int_message(0, # Time from boot (Irrelevant)
                                                                                                                             self.mavlinkInterface.mavConnection.target_system,
                                                                                                                             self.mavlinkInterface.mavConnection.target_component,
                                                                                                                             mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT, # Frame
                                                                                                                             0b110111111000, # Command Mask, Only position command
                                                                                                                             int(self.RTLLat*1e7), # LAT
                                                                                                                             int(self.RTLLon*1e7), # LON
                                                                                                                             self.clearanceAlt,    # Alt
                                                                                                                             0,     # vx
                                                                                                                             0,     # vy
                                                                                                                             0,     # vz
                                                                                                                             0,     # ax
                                                                                                                             0,     # ay
                                                                                                                             0,     # az
                                                                                                                             0,     # yaw
                                                                                                                             0)     # yaw_rate
                                                              )
                        self.resumeSendingCounter = self.resumeSendingCounter + 1
                    return

            # Come down to Actual point
            if self.resumeState == 6:
                if self.terrainAlt < (self.missionAlt+1):
                    self.resumeSendingCounter = 0
                    self.resumeState = 7
                else:
                    if self.resumeSendingCounter < sendCount:
                        self.add_new_message_to_sending_queue(mavutil.mavlink.MAVLink_set_position_target_global_int_message(0, # Time from boot (Irrelevant)
                                                                                                                             self.mavlinkInterface.mavConnection.target_system,
                                                                                                                             self.mavlinkInterface.mavConnection.target_component,
                                                                                                                             mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT, # Frame
                                                                                                                             0b110111111000, # Command Mask, Only position command
                                                                                                                             int(self.RTLLat*1e7), # LAT
                                                                                                                             int(self.RTLLon*1e7), # LON
                                                                                                                             self.missionAlt,    # Alt
                                                                                                                             0,     # vx
                                                                                                                             0,     # vy
                                                                                                                             0,     # vz
                                                                                                                             0,     # ax
                                                                                                                             0,     # ay
                                                                                                                             0,     # az
                                                                                                                             0,     # yaw
                                                                                                                             0)     # yaw_rate
                                                              )
                        self.resumeSendingCounter = self.resumeSendingCounter + 1
                    return
                
            # Wait for 5 s and start pump and nozzle before starting mission
            if self.resumeState == 7:
                if self.holdBeforeStartingAutoCountDown.started and self.holdBeforeStartingAutoCountDown.finished:
                    self.resumeSendingCounter = 0
                    self.resumeState = 8
                    
                    self.holdBeforeStartingAutoCountDown.reset()
                    
                    self.agriPayload.resumeRequestedSpray = False
                else:
                    if not self.holdBeforeStartingAutoCountDown.started:
                        self.holdBeforeStartingAutoCountDown.start()
                    self.agriPayload.resumeRequestedSpray = True
                    return

            # Set Current Waypoint
            if self.resumeState == 8:
                if self.currentWP == self.RTLWP:
                    self.resumeSendingCounter = 0
                    self.resumeState = 9
                else:
                    if self.resumeSendingCounter < sendCount:
                        self.add_new_message_to_sending_queue(mavutil.mavlink.MAVLink_mission_set_current_message(self.mavlinkInterface.mavConnection.target_system,
                                                                                                                  self.mavlinkInterface.mavConnection.target_component,
                                                                                                                  self.RTLWP)
                                                              )
                        self.resumeSendingCounter = self.resumeSendingCounter + 1
                    return

            # Engage AUTO mission
            if self.resumeState == 9:
                if self.currentMode == "AUTO":
                    self.resumeSendingCounter = 0
                    self.resumeOn = False
                    self.resumeState = 0
                else:
                    if self.resumeSendingCounter < sendCount:
                        self.add_new_message_to_sending_queue(mavutil.mavlink.MAVLink_set_mode_message(self.mavlinkInterface.mavConnection.target_system,
                                                                                                       mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                                                                                                       3) # AUTO
                                                              )
                        self.resumeSendingCounter = self.resumeSendingCounter + 1
                    return

    def write_mission_file(self):
        with open('agri_mission_file', 'w') as f:
            f.write('%d %d %d %d %d %d %d %d'%(int(self.missionOn),
                                               self.startWP,
                                               self.endWP,
                                               int(self.missionAlt*100),
                                               self.missionYaw,
                                               int(self.RTLLat*1e7),
                                               int(self.RTLLon*1e7),
                                               self.RTLWP))

    def read_mission_file(self):
        if path.exists('agri_mission_file'):
            with open('agri_mission_file', 'r') as f:
                for line in f:
                    data = line.split()
                    if len(data) == 8:
                        with self.lock:
                            self.missionOn = bool(int(data[0]))
                            self.startWP = int(data[1])
                            self.endWP = int(data[2])
                            self.missionAlt = int(data[3])*0.01
                            self.missionYaw = int(data[4])
                            self.RTLLat = int(data[5])*1e-7
                            self.RTLLon = int(data[6])*1e-7
                            self.RTLWP = int(data[7])
                    return
                
    def check_payload_failsafe(self):
        if not self.agriPayload.payloadRTLEnabled:
            return
        
        if self.agriPayload.payloadRTLEngage and self.currentMode != "RTL":
            logging.info("MSG, Engaging RTL Due to Payload Over")

            # Engage RTL
            self.add_new_message_to_sending_queue(mavutil.mavlink.MAVLink_set_mode_message(self.mavlinkInterface.mavConnection.target_system,
                                                                                           mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                                                                                           6)) # RTL
            self.add_new_message_to_sending_queue(mavutil.mavlink.MAVLink_statustext_message(mavutil.mavlink.MAV_SEVERITY_CRITICAL,
                                                                                             "FAILSAFE: Payload Over".encode()))
        else:
            self.agriPayload.payloadRTLEngage = False

    def update(self):
        currentMode = self.currentMode
        
        # Rewrite mission file in case of mission is over
        if self.currentWP > self.endWP and (self.RTLWP>0):
            if self.missionOn:
                self.RTLLat = -200
                self.RTLLon = -200
                self.RTLWP = 0
                self.write_mission_file()


        # if mode changes to RTL from AUTO then store the current (Lat Lon) as RTL (Lat Lon)
        if self.previousMode == 'AUTO' and currentMode == 'RTL':
            self.RTLLat = self.lat
            self.RTLLon = self.lon
            self.RTLWP = self.currentWP
            self.write_mission_file()

        # Resume Mission Handling
        self.resume_mission()

        if (self.resumeOn and currentMode != 'GUIDED' and self.resumeState > 1) or not self.isArmed:
            self.resumeSendingCounter = 0
            self.resumeOn = False
            self.resumeState = 0
            self.agriPayload.resumeRequestedSpray = False

        # update the required flow rate to the agri payload handlere
        self.agriPayload.update(self.speed, self.startWP, self.endWP, self.currentWP, currentMode)

        # send the data to GCS
        resumeButtonEnable = False
        if not self.isFlying and self.missionOn and self.RTLWP>self.startWP and self.RTLWP<=self.endWP:
            resumeButtonEnable = True
        
        # Check for payload failsafe
        self.check_payload_failsafe()
        
        # Update the Vehicle max speed
        self.update_vehicle_max_speed()

        # Payload Status send to GCS
        self.add_new_message_to_sending_queue(mavutil.mavlink.MAVLink_ga3a_payload_status_message(0,
                                                                                                  0,
                                                                                                  self.agriPayload.remainingPayload,
                                                                                                  int(resumeButtonEnable)))

        # Update Mode
        self.previousMode = currentMode
        
    def update_vehicle_max_speed(self):
        
        if self.maxSpeed != self.agriPayload.maxSpeedSetPoint:
            self.maxSpeedSentCount = 0
            self.maxSpeed = self.agriPayload.maxSpeedSetPoint
            
        # send the message 3 times to be sure message reaches autopilot
        if (self.maxSpeedSentCount < 3):
            if self.maxSpeed >= 8:
                self.maxSpeed = 8

            # create message
            self.add_new_message_to_sending_queue(mavutil.mavlink.MAVLink_command_long_message(self.mavlinkInterface.mavConnection.target_system,
                                                                                               self.mavlinkInterface.mavConnection.target_component,
                                                                                               mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED, # command
                                                                                               0,                                       # confirmation
                                                                                               0,                                       # param1 
                                                                                               self.maxSpeed,                           # param2 
                                                                                               0,                                       # param3
                                                                                               0,                                       # param4
                                                                                               0,                                       # param5
                                                                                               0,                                       # param6
                                                                                               0)                                       # param7
                                                 )
            
            # update counter
            self.maxSpeedSentCount = self.maxSpeedSentCount + 1

    def kill_all_threads(self):
        logging.info("GA3ACompanionComputer killing all threads")
        super().kill_all_threads()
#        self.killAllThread.set()
#
#        for task in self.scheduledTaskList:
#            task.stop()
#
        self.handleRecievedMsgThread.join()
        logging.info("GA3ACompanionComputer joined all threads")
