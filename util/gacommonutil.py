# -*- coding: utf-8 -*-
"""
Created on Wed Mar 06 14:21:00 2019

@author: Sachchit Vekaria
@Organization: General Aeronautics Pvt Ltd
"""

# import necessary modules
import time
from threading import Timer
from datetime import datetime
import pytz
import logging
from util.npnt import NPNT, listdir, path, remove, FlyingBreachedState, State
import threading
from pymavlink import mavutil
import queue
import numpy as np
import struct
import os
from sys import platform

class CompanionComputer(object):
    def __init__(self, sitlType, sitlport):
        # Version Control
        self.version = "v01.11"
        self.sendVersionInfo = False

        # Threading Lock
        self.lock = threading.Lock()

        # Handling for SITL
        self.isSITL = False
        if sitlType is not None:
            self.sitlType = sitlType
            self.isSITL = True
        self.sitlPort = sitlport

        # Instance Mavlink Communication class
        self.mavlinkInterface = MavlinkInterface(sitlType, sitlport)

        # Vehicle Status
        self.rc6 = 0
        self.isFlying = False
        self.isArmed = False
        self.currentWP = 0
        self.currentMode = "UNKNOWN"

        # Vehicle Position
        self.lat = -200
        self.lon = -200
        self.hdop = 100
        self.globalTime = 0
        self.globalAlt = -1000      # m
        self.relativeAlt = -1000    # m
        self.terrainAlt = 0         # m

        # Vehicle Speed
        self.vx = 0 # m/s
        self.vy = 0 # m/s
        self.vz = 0 # m/s
        self.speed = 0 # m/s

        #Position Estimate in meters
        self.px = 0 # m
        self.py = 0 #m

        # Attitude
        self.pitch = 0 # rad (-pi to pi)
        self.roll = 0 # rad (-pi to pi)
        self.yaw = 0 # rad (0 to 2pi)

        # NPNT
        self.npnt = NPNT(sitlType)

        # FTP
        self.ftp = FTP()

        # Variable to kill all threads cleanly
        self.killAllThread = threading.Event()
        self.scheduledTaskList = []

        # Comm loss
        self.commLoss = False
        self.commLossEnabled = False
        self.commLossPersistance = 5 #s
        self.gcsLastHearbeatTime = time.time()

        # Breach RTL engage handling
        self.breached = False
        self.breachedRTLEnabled = False
        self.commRange = 500 # m
        self.lastTakeOffLocation = (-200, -200) # (lat, lon)
        self.takeOffLocationStored = False

    def init(self):
        # Start the main connection to autopilot
        self.mavlinkInterface.create_mavlink_connection()

        # Schedule tasks which needs to be done at particular interval
        self.scheduledTaskList.append(ScheduleTask(0.1, self.check_pause))
        self.scheduledTaskList.append(ScheduleTask(1, self.send_heartbeat))

        # Update NPNT related Variables
        self.scheduledTaskList.append(ScheduleTask(1, self.update_npnt))

        # Check for Comm Loss
        self.scheduledTaskList.append(ScheduleTask(1, self.check_comm_loss))

        # Breach RTL check
        self.scheduledTaskList.append(ScheduleTask(1, self.check_breach))

        # Record Home Location
        self.scheduledTaskList.append(ScheduleTask(1, self.record_home_location))

        # Update Log FileName Every 5 s
        self.scheduledTaskList.append(ScheduleTask(5, self.change_log_file_name))

    # handle common messages
    def handle_recieved_message(self, recievedMsg):
        if recievedMsg.get_type() == "RC_CHANNELS":
            self.rc6 = recievedMsg.chan6_raw
            return

        if recievedMsg.get_type() == "HEARTBEAT":
            # Update Mode
            self.currentMode = self.mavlinkInterface.mavConnection.flightmode
            logging.info("FlightMode, " + self.currentMode)
            if recievedMsg.autopilot == 8 and not self.sendVersionInfo: #GCS
                for i in range(2):
                    self.add_new_message_to_sending_queue(
                        mavutil.mavlink.MAVLink_statustext_message(mavutil.mavlink.MAV_SEVERITY_CRITICAL,
                                                                   ("cc "+self.version).encode()))
                    self.sendVersionInfo = True

            # Handle Message
            if recievedMsg.autopilot == mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA:
                sysStatus = recievedMsg.system_status
                isCurrentFlying = False
                isCurrentFlying = sysStatus == mavutil.mavlink.MAV_STATE_ACTIVE
                if(self.isFlying and not isCurrentFlying):
                    isCurrentFlying = sysStatus == mavutil.mavlink.MAV_STATE_CRITICAL or sysStatus == mavutil.mavlink.MAV_STATE_EMERGENCY
                self.isFlying = isCurrentFlying

                if (recievedMsg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0:
                    self.isArmed = True
                else:
                    self.isArmed = False
                return

            if recievedMsg.type == mavutil.mavlink.MAV_TYPE_GCS:
                self.gcsLastHearbeatTime = time.time()
                return


        if recievedMsg.get_type() == "GPS_RAW_INT":
            self.lat = recievedMsg.lat * 1e-7
            self.lon = recievedMsg.lon * 1e-7
            self.globalAlt = recievedMsg.alt/1000.
            self.hdop = recievedMsg.eph/100.
            return

        if recievedMsg.get_type() == "LOCAL_POSITION_NED":
            self.px = round(recievedMsg.x,2)
            self.py = round(recievedMsg.y,2)
            
        if recievedMsg.get_type() == "GLOBAL_POSITION_INT":
            self.vx = 0.01*recievedMsg.vx
            self.vy = 0.01*recievedMsg.vy
            self.vz = 0.01*recievedMsg.vz
            self.speed = np.sqrt(self.vx**2 + self.vy**2 + self.vz**2)
            self.relativeAlt = recievedMsg.relative_alt/1000.
            return

        if recievedMsg.get_type() == "ATTITUDE":
            self.pitch = recievedMsg.pitch
            self.roll = recievedMsg.roll
            if recievedMsg.yaw < 0:
                self.yaw = recievedMsg.yaw + 2*np.pi
            else:
                self.yaw = recievedMsg.yaw

        if recievedMsg.get_type() == "MISSION_CURRENT":
            self.currentWP = recievedMsg.seq
            return

        if recievedMsg.get_type() == "SYSTEM_TIME":
            self.globalTime = int(recievedMsg.time_unix_usec*1e-6)
            return

        if recievedMsg.get_type() == "RANGEFINDER":
            self.terrainAlt = recievedMsg.distance
            return

        if recievedMsg.get_type() == "NPNT_UIN_REGISTER":
            self.npnt.uinChangeRequested = ''.join(map(chr,recievedMsg.uin[0:recievedMsg.size]))
            return

        if recievedMsg.get_type() == "NPNT_KEY_ROTATION":
            self.npnt.keyRotationRequested = True
            return

        if recievedMsg.get_type() == "NPNT_REQ_LOGS":
            self.npnt.logDownloadRequest = True
            self.npnt.logDownloadDateTime = ''.join(map(chr,recievedMsg.date_time[0:15]))
            return

        if recievedMsg.get_type() == "NPNT_RFM_DETAIL":
            self.add_new_message_to_sending_queue(mavutil.mavlink.MAVLink_npnt_rfm_detail_message(0,
                                                                                                  0,
                                                                                                  self.npnt.firmwareVersion.encode(),
                                                                                                  len(self.npnt.firmwareVersion),
                                                                                                  self.npnt.firmwareHash.encode(),
                                                                                                  len(self.npnt.firmwareHash),
                                                                                                  self.npnt.rpasId.encode(),
                                                                                                  len(self.npnt.rpasId),
                                                                                                  self.npnt.rpasModelId.encode(),
                                                                                                  len(self.npnt.rpasModelId),
                                                                                                  self.npnt.uin.encode(),
                                                                                                  len(self.npnt.uin)))
            return

        if recievedMsg.get_type() == "NPNT_GEOFENCE":
            self.npnt.fenceSentToGCS = True

        if recievedMsg.get_type() == "FILE_TRANSFER_PROTOCOL":
            replyPayload = self.ftp.handle_ftp_message(recievedMsg.payload)
            if replyPayload:
                self.add_new_message_to_sending_queue(mavutil.mavlink.MAVLink_file_transfer_protocol_message(0,
                                                                                                             255,
                                                                                                             0,
                                                                                                             replyPayload))
            return

    def change_log_file_name(self):
        if platform == "win32":
            return

        if self.globalTime > 0 and os.path.exists("temp"):
            currentTime = pytz.utc.localize(datetime.utcfromtimestamp(self.globalTime)).astimezone(self.npnt.timeZone)
            fileName = "companion_comp_log_" + currentTime.strftime("%Y_%m_%d_%H_%M_%d")
            os.rename("temp",fileName)

    def check_pause(self):
        # check if rc 6 is more than 1800
        # if yes change the flag
        if self.rc6 is not None:
            if self.rc6 > 1800:
                self.mavlinkInterface.sendingBlocked = True
                logging.warn("Blocking Message sending")
            else:
                self.mavlinkInterface.sendingBlocked = False

    def send_heartbeat(self):
        msg = mavutil.mavlink.MAVLink_heartbeat_message(18, 8, 0, 0, 0, 3)
        self.add_new_message_to_sending_queue(msg)

    def update_npnt(self):
        self.npnt.update(self.lat, self.lon, self.globalAlt, self.hdop, self.globalTime, self.isArmed, self.ftp.latestUploadedPAFile)
        self.add_new_message_to_sending_queue(mavutil.mavlink.MAVLink_npnt_status_message(self.mavlinkInterface.mavConnection.target_system,
                                                                                          self.mavlinkInterface.mavConnection.target_component,
                                                                                          self.npnt.get_npnt_allowed(),
                                                                                          self.npnt.get_npnt_not_allowed_reason()))

        # Geofence Message
        if not self.npnt.fenceSentToGCS and len(self.npnt.fence.list_points()) < 10:
            lat = [int(-200e7)]*10
            lon = [int(-200e7)]*10
            i=0
            for point in self.npnt.fence.list_points():
                lat[i] = int(point[0]*1e7)
                lon[i] = int(point[1]*1e7)
                i = i+1
            logging.info("NPNT, Sending Geofence data to GCS")
            self.add_new_message_to_sending_queue(mavutil.mavlink.MAVLink_npnt_geofence_message(255,
                                                                                                0,
                                                                                                lat,
                                                                                                lon))
        # Acknoledgement Messages
        if self.npnt.keyRotationRequested:
            self.npnt.keyRotationRequested = False
            confirm = int(self.npnt.key_rotation())
            if confirm:
                logging.info("NPNT, Key Rotation Done")
            self.add_new_message_to_sending_queue(mavutil.mavlink.MAVLink_npnt_key_rotation_message(255,
                                                                                                    0,
                                                                                                    confirm))

        if self.npnt.uinChangeRequested:
            self.npnt.update_uin()
            logging.info("NPNT, UIN Update Done")
            emptyBytes = [0]*30
            self.add_new_message_to_sending_queue(mavutil.mavlink.MAVLink_npnt_uin_register_message(255,
                                                                                                    0,
                                                                                                    1,
                                                                                                    30,
                                                                                                    emptyBytes))
            self.npnt.uinChangeRequested = None

        if self.npnt.logDownloadRequest:
            confirm = 0
            self.npnt.logDownloadRequest = False
            if len(self.npnt.logDownloadDateTime) == 15:
                self.npnt.start_bundling()
                logging.info("NPNT, Log Bundling Done")
                self.npnt.logDownloadDateTime = None
                confirm = 1

            emptyBytes = [0]*15
            self.add_new_message_to_sending_queue(mavutil.mavlink.MAVLink_npnt_req_logs_message(255,
                                                                                                0,
                                                                                                confirm,
                                                                                                emptyBytes))

    def check_comm_loss(self):
        # No need to check if vehicle is not flying
        if not self.isFlying:
            return

        # Check whether comloss action feature is enabled or not
        if not self.commLossEnabled:
            return

        # If no GCS heatbeat for some time
        if (time.time()-self.gcsLastHearbeatTime) > self.commLossPersistance:
            # If comm loss is already reported
            if self.commLoss:
                return

            logging.info("MSG, Engaging RTL Due to Comm Loss")

            # Engage RTL
            self.add_new_message_to_sending_queue(mavutil.mavlink.MAVLink_set_mode_message(self.mavlinkInterface.mavConnection.target_system,
                                                                                           mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                                                                                           6)) # RTL
            self.add_new_message_to_sending_queue(mavutil.mavlink.MAVLink_statustext_message(mavutil.mavlink.MAV_SEVERITY_CRITICAL,
                                                                                             "FAILSAFE: Comm Loss".encode()))
            self.commLoss = True
        else:
            self.commLoss = False

    def check_breach(self):
        # No need to check if vehicle is not flying
        if not self.isFlying:
            self.breached = False
            return

        # Check whether comloss action feature is enabled or not
        if not self.breachedRTLEnabled:
            return

        distFromHome = dist_between_lat_lon(self.lastTakeOffLocation[0],
                                            self.lastTakeOffLocation[1],
                                            self.lat,
                                            self.lon)

        if distFromHome > self.commRange or self.relativeAlt > 30 or str(self.npnt.state) is str(FlyingBreachedState()):
            # If comm loss is already reported
            if self.breached:
                return

            logging.info("MSG, Engaging RTL Due to Breach")
            # Engage RTL
            self.add_new_message_to_sending_queue(mavutil.mavlink.MAVLink_set_mode_message(self.mavlinkInterface.mavConnection.target_system,
                                                                                           mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                                                                                           6)) # RTL
            self.add_new_message_to_sending_queue(mavutil.mavlink.MAVLink_statustext_message(mavutil.mavlink.MAV_SEVERITY_CRITICAL,
                                                                                             "FAILSAFE: GeoFence Breach".encode()))

            self.breached = True
        else:
            self.breached = False


    def record_home_location(self):
        if self.isArmed and not self.takeOffLocationStored:
            logging.info("MSG, Home Location Recorded")
            self.lastTakeOffLocation = (self.lat, self.lon)
            self.takeOffLocationStored = True

        if not self.isArmed and self.takeOffLocationStored:
            self.takeOffLocationStored = False

    def add_new_message_to_sending_queue(self, msg):
        # Only this method to be used to send the messages
        if self.mavlinkInterface.connected:
            # This if loop to make sure no garbage in queue when there is no
            # connection
            self.mavlinkInterface.pendingSendMsgList.put(msg)

    def get_new_message_from_recieving_queue(self):
        # Only this method to be used to recieve messages
        if not self.mavlinkInterface.recievedMsgQueue.empty():
            return self.mavlinkInterface.recievedMsgQueue.get()
        else:
            return None

    def kill_all_threads(self):
        logging.info("CompanionComputer killing all threads")
        self.mavlinkInterface.kill_all_threads()
        self.npnt.kill_all_threads()
        self.killAllThread.set()

        for task in self.scheduledTaskList:
            task.stop()

        logging.info("CompanionComputer joined all threads")

class MavlinkInterface(object):
    # This class handles all the Mavlink Messaging
    def __init__(self, sitlType, sitlPort):
        # Handling for SITL
        self.isSITL = False
        if sitlType is not None:
            self.sitlType = sitlType
            self.isSITL = True
        self.sitlPort = sitlPort

        # Initialize connection variables
        self.mavConnection = None
        self.connected = False

        # Threading Lock for Mavlink Class
        self.lock = threading.Lock()
        self.recievingThread = None
        self.sendingThread = None

        # Message list
        self.recievedMsgQueue = queue.Queue()
        self.pendingSendMsgList = queue.Queue()

        # Blocker for sending message
        self.sendingBlocked = False

        # Start AutoReconnect signal slot mechanism
        self.disconnectEvent = threading.Event()
        self.autoReconnectThread = threading.Thread(target=self.auto_reconnect)
        self.autoReconnectThread.start()

        # Variable to kill all threads cleanly
        self.killAllThread = threading.Event()


    def auto_reconnect(self):
        # If we recieve disconnection signal, set the connected status to
        # False and try restarting the connection
        logging.info("Auto Reconnection Functionality Started")
        while True:
            try:
                self.disconnectEvent.wait()
                if self.killAllThread.is_set():
                    break
                self.connected = False
                self.create_mavlink_connection()
                self.disconnectEvent.clear()

            # During debugging so that we can exit the loop
            except KeyboardInterrupt:
                self.kill_all_threads()
                break

    def create_mavlink_connection(self):
        # Close already existing connection
        if self.mavConnection is not None:
            self.mavConnection.close()

        logging.info("Starting Mavlink Connection")
        while not self.connected:
            if self.killAllThread.is_set():
                break
            try:
                time.sleep(1)
                # Create the connection
                if self.isSITL:
                    if self.sitlType == 'tcp':
                        self.mavConnection = mavutil.mavlink_connection('tcp:127.0.0.1:'+str(self.sitlPort),
                                                                   source_system=1,
                                                                   source_component=10)
                    elif self.sitlType == 'udp':
                        self.mavConnection = mavutil.mavlink_connection('udp:127.0.0.1:'+str(self.sitlPort),
                                                                   source_system=1,
                                                                   source_component=10)
                    elif self.sitlType == 'com':
                        self.mavConnection = mavutil.mavlink_connection('COM28',
                                                                   baud=57600,
                                                                   source_system=1,
                                                                   source_component=10)
                else:
                    # Need to provide the serial port and baudrate
                    self.mavConnection = mavutil.mavlink_connection('/dev/serial0',
                                                               baud=921600,
                                                               source_system=1,
                                                               source_component=10)

            # During debugging so that we can exit the loop
            except KeyboardInterrupt:
                self.kill_all_threads()
                break

            except Exception as e:
                logging.exception(e)
                if self.mavConnection is not None:
                    self.mavConnection.close()
            else:
                # wait for the heartbeat msg to find the system ID
                self.mavConnection.wait_heartbeat()
                logging.info("Mavlink Connection Established")

                self.connected = True
                self.recievingThread = threading.Thread(target=self.recieving_loop)
                self.sendingThread = threading.Thread(target=self.sending_loop)
                self.recievingThread.start()
                self.sendingThread.start()

    # Mavlink recieveng loop
    def recieving_loop(self):
        logging.info("starting recieving loop")
        while self.connected:
            if self.killAllThread.is_set():
                break
            try:
                # Recieve the messages
                recieved = self.mavConnection.recv_match()

                # If empty message ignore
                if recieved is not None:
                    self.recievedMsgQueue.put(recieved)
                else:
                    time.sleep(0.001)

            # During debugging so that we can exit the loop
            except KeyboardInterrupt:
                self.kill_all_threads()
                break

            except Exception as e:
                logging.exception(e)
                if not self.disconnectEvent.is_set():
                    self.disconnectEvent.set()

    # Mavlink Sending Loop
    def sending_loop(self):
        logging.info("starting sending loop")
        while self.connected:
            if self.killAllThread.is_set():
                break
            if not self.pendingSendMsgList.empty():
                try:
                    msg = self.pendingSendMsgList.get()
                    if not self.sendingBlocked:
                        self.mavConnection.mav.send(msg)


                # During debugging so that we can exit the loop
                except KeyboardInterrupt:
                    self.kill_all_threads()
                    break

                except Exception as e:
                    logging.exception(e)
                    if not self.disconnectEvent.is_set():
                        self.disconnectEvent.set()
            else:
                # If there is no pending message to be sent wait for 0.05 seconds
                time.sleep(0.01)

    def kill_all_threads(self):
        logging.info("MavlinkInterface killing all threads")
        self.disconnectEvent.set()
        self.killAllThread.set()
        self.autoReconnectThread.join()
        self.recievingThread.join()
        self.sendingThread.join()
        logging.info("MavlinkInterface joined all threads")

class FTP(object):
    def __init__(self):
        self.recievedFile = None
        self.latestUploadedPAFile = None
        self.receivedFileString = ''

        self.sentFile = None
        self.sentFileSize = -1
        self.sentFileString = ''

    def update_recieved_file_status(self):
        with open(self.recievedFile,'w') as f:
            if(len(self.receivedFileString)>0):
                f.write(self.receivedFileString)
                self.receivedFileString = ''
        if "NPNT/Permission_Artefact" in self.recievedFile:
            self.latestUploadedPAFile = self.recievedFile.split("/")[-1]

        self.recievedFile = None

    def update_sent_file_status(self):
        self.sentFile = None
        self.sentFileSize = -1

    def handle_ftp_message(self, payload):
        sessionid =  payload[2]
        opcode = payload[3]
        size = payload[4]
        reqopcode = payload[5]
        burstComplete = payload[6]
        padding = payload[7]
        contentOffset = struct.unpack('I',bytearray([payload[8],payload[9],payload[10],payload[11]]))[0]
        data = ''.join(map(chr,payload[12:12+size]))

        replyPayload = None

        if(opcode == 6 or opcode == 7):
            replyPayload = self.save_file(opcode, contentOffset, data)
        if(opcode == 1):
            replyPayload = self.terminate_session()
        if(opcode == 3):
            replyPayload = self.send_directory_list(contentOffset, data)
        if(opcode == 4 or opcode == 5):
            replyPayload = self.send_file(opcode, contentOffset, size, data)
        if(opcode == 8):
            replyPayload = self.remove_file(data)

        #replying the same sequence number back
        replyPayload[0]=payload[0]
        replyPayload[1]=payload[1]

        return replyPayload

    def remove_file(self, data):
        payloadVal = [0]*251
        payloadVal[3] = 128
        if path.isfile(data):
            remove(data)
        payloadVal[5] = 8

        return payloadVal

    def save_file(self, opcode, contentOffset, data):
        payloadVal = [0]*251
        payloadVal[3] = 128
        if (opcode == 6):
            self.recievedFile=data
            self.receivedFileString = ''
            payloadVal[5] = 6
        if (opcode == 7):
            if(len(self.receivedFileString)==contentOffset):
                self.receivedFileString += data
            payloadVal[5] = 7

        return payloadVal

    def send_file(self, opcode, contentOffset, size, data):
        payloadVal = [0]*251
        payloadVal[3]=128
        if (opcode==4):
            self.sentFile=data
            if path.isfile(self.sentFile):
                with open(self.sentFile,'r') as f:
                    self.sentFileString = f.read()
                    self.sentFileSize = len(self.sentFileString)
            else:
                self.sentFileSize=0
                payloadVal[3]=129
            payloadVal[4]=4
            payloadData = struct.pack('I',self.sentFileSize)
            for j in range(0,len(payloadData)):
                payloadVal[12+j] = payloadData[j]#struct.unpack('B', payloadData[j])[0]
            payloadVal[5]=4
        if (opcode==5):
            if(contentOffset < self.sentFileSize):
                charCount=size
                fileString = ''
                if(contentOffset+size > self.sentFileSize):
                   charCount = self.sentFileSize - contentOffset
                fileString = self.sentFileString[contentOffset:][:charCount]
                payloadVal[5]=5
                payloadVal[4]=charCount
                payloadData  = list(fileString.encode())
                for j in range(0,charCount):
                    payloadVal[12+j] = payloadData[j]#struct.unpack('B', payloadData[j])[0]


            else:
                payloadVal[3]=129
                payloadVal[5]=5

        return payloadVal

    def terminate_session(self):
        payloadVal = [0]*251
        payloadVal[3] = 128
        payloadVal[5] = 1
        # Update Which file is uploaded/downloaded for other codes to handle
        # the relevant actions
        if self.recievedFile:
            self.update_recieved_file_status()
        if self.sentFile:
            self.update_sent_file_status()

        return payloadVal

    def send_directory_list(self, contentOffset, data):
        # Sending only list of files
        payloadVal = [0]*251
        payloadVal[5]=3
        onlyfiles=[]
        if path.isdir(data):
            onlyfiles = [f for f in listdir(data) if path.isfile(path.join(data, f))]
        dirListString=''
        if(contentOffset<len(onlyfiles)):
            for i in range(contentOffset,len(onlyfiles)):
                if(len(dirListString)==0):
                    if(len(dirListString) + len(onlyfiles[i]) <=200):
                        dirListString = dirListString + onlyfiles[i]
                    else:
                        break
                else:
                    if(len(dirListString) + len(':') + len(onlyfiles[i]) <=200):
                        dirListString = dirListString + ':' + onlyfiles[i]
                    else:
                        break
            payloadVal[3]=128
            payloadData  = list(dirListString.encode())
            size=len(payloadData)
            payloadVal[4]=size
            for j in range(0,size):
                payloadVal[12+j] = payloadData[j]#struct.unpack('B', payloadData[j])[0]
        else:
            payloadVal[3]=129
            size=1
            payloadVal[4]=size
            payloadVal[12]=6

        return payloadVal

# Class for scheduling tasks
class ScheduleTask(object):
    def __init__(self, interval, function, *args, **kwargs):
        self._timer     = None
        self.interval   = interval
        self.function   = function
        self.args       = args
        self.kwargs     = kwargs
        self.is_running = False
        self.start()

    def _run(self):
        self.is_running = False
        self.start()
        self.function(*self.args, **self.kwargs)

    def start(self):
        if not self.is_running:
            self._timer = Timer(self.interval, self._run)
            self._timer.start()
            self.is_running = True

    def stop(self):
        self._timer.cancel()
        self.is_running = False

def dist_between_lat_lon(lat1, lon1, lat2, lon2):
    # Returns distance between two lat lon in meters
    R = 6373000

    lat1 = np.radians(lat1)
    lon1 = np.radians(lon1)
    lat2 = np.radians(lat2)
    lon2 = np.radians(lon2)

    dlon = lon2 - lon1
    dlat = lat2 - lat1

    a = np.sin(dlat / 2)**2 + np.cos(lat1) * np.cos(lat2) * np.sin(dlon / 2)**2
    c = 2 * np.arctan2(np.sqrt(a), np.sqrt(1 - a))

    return R * c

class CountDown(object):
    def __init__(self, interval):
        self.started = False
        self.finished = False
        self.interval = interval
        self._timer = None

    def start(self):
        if not self.started:
            self.started = True
            self.finished = False
            self._timer = Timer(self.interval, self.time_complete)
            self._timer.start()

    def time_complete(self):
        self.finished = True

    def reset(self):
        if self._timer is not None:
            self._timer.cancel()
        self.started = False
        self.finished = False
