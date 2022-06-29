# -*- coding: utf-8 -*-
"""
Created on Fri Mar  8 12:22:57 2019

@author: Sachchit Vekaria
@Organization: General Aeronautics Pvt Ltd
"""

import numpy as np
import time
import struct
import serial
import copy
import logging
from util.gacommonutil import CountDown, ScheduleTask


class AgriPayload:
    def __init__(self, isSITL):  
        self.isSITL = isSITL
        
        self.actualFlowRate = 0     # LPM
        self.pesticidePerAcre = 5   # Litre/Acre
        self.swath = 4              # m
        self.maxFlowRate = 1.2      # Litre/Minute
        
        # Status
        self.remainingPayload = 8                   # in liters
        self.reqFlowRate = 0.                        # in ltr/min
        self.shouldSpraying = False
        self.payloadRTLEngage = False
        self.payloadRTLEnabled = True
        
        # pump parameters
        self.pumpMaxPWM = 2000
        self.pumpMinPWM = 1200
        self.pumpAbsMinPWM = 1000
        self.pumpMaxFlowRate = 2.8                  # in ltr/min
        self.pumpMinFlowRate = 0.4                    # in ltr/min
        self.pumpPWM = 0
        
        # Nozzle parameters
        self.nozzMaxPWM = 2000
        self.nozzMinPWM = 1000
        self.targetPS = 120                 # Particle Size in micrometer
        self.nozzPWM = 0                    # Current PWM of Nozzle
        self.nozzNum = 4                    # number of nozzles
        self.nozzMaxFlowRate = 4            # in ltr/min
        self.nozzMinFlowRate = 0.1          # in ltr/min
        self.nozzType = 0                   # 0->micromiser_with_dcu, 1->micromiser_without_dcu, 2->chinese
        self.nozzP = 0.01
        self.nozzNoDripPWM = 1500
        
        # Counter to send the speed to autopilot
        self.maxSpeedSetPoint = 5                   # m/s

        # GPIO handling
        if not isSITL:
            import pigpio
            self.pi = pigpio.pi()
            
        # Initialize Sensor handling class
        self.pibStatus = PIBStatus(isSITL)
        self.flowSensor = FlowSensor(isSITL)

        self.pumpPin = 18
        self.nozzPin = 19

        # PID Pump
        self.pumpP = 50.
        self.pumpI = 10.
        self.pumpIMax = 3.
        self.pumpD = 20.
        self.pumpOldError = 0.
        self.pumpIntError = 0.
        
        # Time related
        self.time = time.time()
        
        # Payload Over Counter
        self.payloadOverStartTime = time.time()

        # Debug Timer
        self.debugTime = time.time()
        
        # Agri Specific vehicle status
        self.payloadTesting = 0
        
        # Drip Stopping parameters
        self.dripStopCountDown = CountDown(20)
        
        # Resume Function
        self.resumeRequestedSpray = False


    # Parameter Getter and Setter
    def set_remaining_payload(self, value):
        self.remainingPayload = value
    
    def set_pestiscide_per_acre(self, value):
        self.pesticidePerAcre = value
        
    def set_swath(self, value):
        self.swath = value
        
    def set_max_flow_rate(self, value):
        self.maxFlowRate = value
        
    def set_particle_size(self, value):
        self.targetPS = value
        
    def get_remaining_payload(self):
        return self.remainingPayload
    
    def get_pestiscide_per_acre(self):
        return self.pesticidePerAcre
        
    def get_swath(self):
        return self.swath
        
    def get_max_flow_rate(self):
        return self.maxFlowRate
        
    def get_particle_size(self):
        return self.targetPS

    def set_nozz_type(self, value):
        self.nozzType = int(value)

    def get_nozz_type(self):
        return self.nozzType

    def set_nozz_min_pwm(self, value):
        self.nozzMinPWM = int(value)

    def get_nozz_min_pwm(self):
        return self.nozzMinPWM

    def set_nozz_max_pwm(self, value):
        self.nozzMaxPWM = int(value)

    def get_nozz_max_pwm(self):
        return self.nozzMaxPWM

    def set_nozz_nodrip_pwm(self, value):
        self.nozzNoDripPWM = int(value)

    def get_nozz_nodrip_pwm(self):
        return self.nozzNoDripPWM

    def set_nozz_count(self, value):
        self.nozzNum = int(value)

    def get_nozz_count(self):
        return self.nozzNum

    def update_remaining_payload(self, actualFlowRate):
        if self.remainingPayload > 0:
            self.remainingPayload = self.remainingPayload - self.dt * (actualFlowRate/60.)
            
    def update_time(self):
        currTime = time.time() 
        self.dt = currTime - self.time
        self.time = currTime
        
    def update(self, speed, startWP, endWP, currentWP, flightMode):
        # update timer
        self.update_time()
        
        # RPM
        actualRPM = 0 #self.pibStatus.status['ATOMIZER_RPM']
        
        # actual flow rate
        actualFlowRate = self.flowSensor.flowRate
        
        # check whether we should be spraying
        logging.info("WP, %d, %d, %d"%(startWP, currentWP, endWP))
        if (currentWP <= endWP and currentWP > startWP and endWP > 1 and flightMode == 'AUTO') or self.resumeRequestedSpray:
            # Allow sending max speed again to vehicle if the shouldSpraying state have changed
            if not self.shouldSpraying:
                self.maxSpeedSentCount = 0
                
            self.shouldSpraying = True
            # Set correct top speed of vehicle
            sprayDensity = self.pesticidePerAcre/4047.
            self.maxSpeedSetPoint = self.maxFlowRate/60./self.swath/sprayDensity + 0.15
            
            # Payload Over RTL
            if actualFlowRate < 0.1 and self.reqFlowRate > 0.4 and self.pumpPWM > 1800 and self.remainingPayload < 3:
                if (time.time() - self.payloadOverStartTime) > 2:
                    self.payloadRTLEngage = True
            else:
                self.payloadOverStartTime = time.time()
            
        else:
            # If we have just stopped spraying, start for drip stopping run
            if self.shouldSpraying:
                self.dripStopCountDown.start()
            
            self.shouldSpraying = False
            # Change the max speed setpoint
            self.maxSpeedSetPoint = 8
        
        if self.shouldSpraying:
            # calculate flow rate
            self.calc_flow_rate(speed)
            
            # calculate pwm for pump and nozzle
            self.calc_pump_pwm(actualFlowRate)
            self.calc_nozz_pwm(actualRPM)
            
            # Reset Counter
            self.dripStopCountDown.reset()
            
        else:
            if self.payloadTesting == 1:
                self.reqFlowRate = self.maxFlowRate
                    
                self.calc_pump_pwm(actualFlowRate)
                self.calc_nozz_pwm(actualRPM)
                
            elif self.payloadTesting == 2:
                self.nozzPWM = self.nozzMinPWM

                self.reqFlowRate = self.maxFlowRate
                    
                self.calc_pump_pwm(actualFlowRate)
                
            else:     
                if self.dripStopCountDown.started:
                    if not self.dripStopCountDown.finished:
                        self.nozzPWM = self.nozzNoDripPWM
                    else:
                        self.dripStopCountDown.reset()
                else:
                    self.nozzPWM = self.nozzMinPWM
                self.pumpPWM = self.pumpAbsMinPWM
                self.reqFlowRate = 0
        
        # update the pwm in DCU
        self.update_pwm()

        # update the remaining payload
        self.update_remaining_payload(actualFlowRate)
            
        logging.info("FlowRate, %f, %f"%(self.reqFlowRate, actualFlowRate))
        logging.info("FlowPWM, %d, %d"%(self.nozzPWM, self.pumpPWM))
        logging.info("RemPayload, %f"%(self.remainingPayload))
        
    def calc_flow_rate(self, speed):
        # sanity check of speed
        if (speed < 0):
            speed = 0
        if (speed >= self.maxSpeedSetPoint+1):
            speed = self.maxSpeedSetPoint+1
        
        # Calculate flow rate
        sprayDensity = self.pesticidePerAcre/4047.          # Ltr/m^2
        self.reqFlowRate = 60*self.swath*sprayDensity*speed # Ltr/min

        
        # check flow rate limit
        if self.reqFlowRate > self.maxFlowRate:
            self.reqFlowRate = self.maxFlowRate
        if self.reqFlowRate < self.pumpMinFlowRate:
            self.reqFlowRate = self.pumpMinFlowRate
            
    def calc_pump_pwm(self, actualFlowRate):
        # P
        error = (self.reqFlowRate - actualFlowRate)
        pumpPWM = int(self.pumpPWM + self.pumpP * error)

        # D
        dError = (error - self.pumpOldError) / self.dt
        self.pumpOldError = error
        pumpPWM = int(pumpPWM + self.pumpD * dError)

        # I
        self.pumpIntError = self.pumpIntError + error * self.dt
        if (self.pumpI * self.pumpIntError) > self.pumpIMax:
            self.pumpIntError = self.pumpIMax/self.pumpI

        if (self.pumpI * self.pumpIntError) < -self.pumpIMax:
            self.pumpIntError = -self.pumpIMax/self.pumpI
        pumpPWM = int(pumpPWM + self.pumpI * self.pumpIntError)

        logging.debug("Pump PID, %f, %f, %f, %f, %f"%(error, self.pumpIntError, self.pumpP * error, self.pumpI * self.pumpIntError, self.pumpD * dError))
        
        self.pumpPWM = pumpPWM
        if self.pumpPWM > self.pumpMaxPWM:
            self.pumpPWM = self.pumpMaxPWM
        if self.pumpPWM < self.pumpMinPWM:
            self.pumpPWM = self.pumpMinPWM
        
    def calc_nozz_pwm(self, actualRPM):
        
        if self.nozzType is 0: #micromiser_with_dcu
            # self.nozzPWM = int(self.nozzMaxPWM - 500*float(self.nozzMaxFlowRate - self.reqFlowRate/4)/float(self.nozzMaxFlowRate - self.nozzMinFlowRate))
            
            # Calculate Target RPM
            nozzFlow = self.reqFlowRate/self.nozzNum
            if nozzFlow < self.nozzMinFlowRate:
                nozzFlow = self.nozzMinFlowRate
            if nozzFlow > self.nozzMaxFlowRate:
                nozzFlow = self.nozzMaxFlowRate
            logging.info("Nozz flowrate, %f" % (nozzFlow))
            
            # curve fit equation RPM = A * (PS - C) ^ B from Micromiser 10 Datasheet
            equationCoeefsA = 257965*np.exp(nozzFlow * -3.519)
            equationCoeefsB = -0.732 + (nozzFlow - 0.3) / (0.1 - 0.3) * (-0.858 + 0.732)
            equationCoeefsC = 80 + (nozzFlow - 0.3) / (0.1 - 0.3) * (42 - 80)
            
            # assuming Power = A * RPM^2 + C
            # and A = a1 * (flow^2) + b1 * flow + c1
            # C = a2 * (flow^2) + b2 * flow + c2
            A = 2.779850e-07*nozzFlow**2 + 7.171870e-08*nozzFlow + 1.936642e-08
            C = 1.10688083*nozzFlow**2 + 0.04140868*nozzFlow + 0.19592032
            
            maxPossibleRPM = np.sqrt(10/A-C)-500
            
            # Whether Target Particle Size can be achieved or not.
            # If not possible only go for max possible RPM
            if (self.targetPS - equationCoeefsC) > 1:
                nozzTargetRPM = equationCoeefsA * (self.targetPS - equationCoeefsC)**equationCoeefsB
                logging.info("Nozz PS, %f, %s, %f" % (self.targetPS, 'possible' ,nozzTargetRPM))
            else:
                nozzTargetRPM = maxPossibleRPM
                logging.info("Nozz PS, %f, %s, %f" % (self.targetPS, 'not_possible' ,nozzTargetRPM))

            # If Target Nozzle RPM is more than max possible RPM
            # Then limit the target RPM to max possible RPM
            if nozzTargetRPM > maxPossibleRPM:
                nozzTargetRPM = maxPossibleRPM
                logging.info("Nozz RPM, %f, %s" % (nozzTargetRPM, 'exceeded_max'))
            
            # rpm = m*voltage + c
            m = -373.91 * nozzFlow + 599.77
            c = -197.78 * nozzFlow - 129.68
            
            voltage = (nozzTargetRPM-c)/m
            if not 6<=voltage<=24:
                logging.info("Nozz voltage req, %f, %s" % (voltage, 'out_of_bound'))
            else:
                logging.info("Nozz voltage req, %f" % (voltage))
            if voltage < 6:
                voltage = 6
            if voltage > 24:
                voltage = 24

            self.nozzPWM = int(1000 + voltage/24*1000)
                
            # # handle garbage value
            # if actualRPM < 0:
            #     actualRPM = 0
            # if actualRPM > 20000:
            #     actualRPM = 20000
                
            # # P control for maintaing target PWM
            # self.nozzPWM = int(self.nozzPWM + self.nozzP * (nozzTargetRPM - actualRPM))

            if self.nozzPWM > self.nozzMaxPWM:
                self.nozzPWM = self.nozzMaxPWM
            if self.nozzPWM < self.nozzMinPWM:
                self.nozzPWM = self.nozzMinPWM


        if self.nozzType is 1: #micromiser_without_dcu
            self.nozzPWM = self.nozzMaxPWM

        if self.nozzType is 2: #aadyah
            self.nozzPWM = self.nozzMaxPWM

        if self.nozzType is 3: #chinese
            self.nozzPWM = self.nozzMaxPWM

        logging.info("Nozz pwm, %f"%(self.nozzPWM ))


    def update_pwm(self):
        # Redundant check to prevent unrealistic value to go to the FCS
        if self.nozzPWM > self.nozzMaxPWM:
            self.nozzPWM = self.nozzMaxPWM
        if self.nozzPWM < self.nozzMinPWM:
            self.nozzPWM = self.nozzMinPWM
        if self.pumpPWM > self.pumpMaxPWM:
            self.pumpPWM = self.pumpMaxPWM
        if self.pumpPWM < self.pumpAbsMinPWM:
            self.pumpPWM = self.pumpAbsMinPWM

        if not self.isSITL:
            self.pi.hardware_PWM(self.pumpPin, 50, 50*self.pumpPWM)
            self.pi.hardware_PWM(self.nozzPin, 50, 50*self.nozzPWM)
        
###############################################################################
        
class PIBStatus:
    # This class store the current status of the PIB board
    #
    # Also, it will have additional functionalities like storing for future
    # data analysis
    def __init__(self, isSITL):
        self.isSITL = isSITL
        
        self.serial = '/dev/ttyDCU'
        
        # All data recieved from PIB is stored in this dictionary
        self.status = {'ATOMIZER_RPM': np.zeros(6),
                       'ATOMIZER_CURRENT': np.zeros(6),
                       'PUMP_CURRENT': np.zeros(2),
                       'FLOW_METER_READING': np.zeros(2)}
        
        # Status of the error
        # Whether currently in error or not
        self.errorNow =  {'INVALID_START_BIT': False,
                          'INVALID_FUNCTION_CODE': False,
                          'INVALID_LRC_PIB': False,         # PIB side
                          'HEALTH_CHECK_FAIL': False,
                          'WRONG_DATA': False,              # RPi side
                          'INVALID_LRC_RPI': False}        # RPi side
        self.errorList = {'INVALID_START_BIT': 5,
                          'INVALID_FUNCTION_CODE': 6,
                          'INVALID_LRC_PIB': 7,         # PIB side
                          'HEALTH_CHECK_FAIL': 8,
                          'WRONG_DATA': 9,              # RPi side
                          'INVALID_LRC_RPI': 10}        # RPi side
        self.errorStartTime = 0 # start time of error
        self.errorTime = 0  # Time since error without resolvement
        
        # definition of function codes
        self.functionCode = {'MSG1': 1,         # Have data from PIB
                             'MSG2': 2,         # Acknowledgement from PIB for recieving messege from RPi
                             'MSG3': 3}         # Nozzle selection
        
        # data not recieved counter
        self.dataNotRecievedStartTime = 0 # start time after which no data is coming
        self.dataNotRecievedTime = 0      # time till no data is recieved
        
        # Timer to count timing of events
        self.timer = time.time()
        
        # Nozzle configuration
        self.nozzleConfiguration = 0b00111100
        
        # Initialize
        self.pibEnabled = False
        
    def init(self):
        if not self.pibEnabled:
            return
        
        if self.isSITL:
            return
        
        while True:
            # keep trying  to open port unitl succesful
            try:
                time.sleep(1)
                self.ser = serial.Serial(port=self.serial,
                                         baudrate=115200,
                                         parity=serial.PARITY_ODD,
                                         stopbits=serial.STOPBITS_ONE,
                                         bytesize=serial.EIGHTBITS,
                                         timeout=1)
                break
            except KeyboardInterrupt:
                raise KeyboardInterrupt
        self.send_nozzle_config()
    
    def set_pib_enabled(self, value):
        if value == 0 or value == 1:
            self.pibEnabled = bool(value)
            
    def get_pib_enabled(self):
        return int(self.pibEnabled)
                
    def update(self):
        if not self.pibEnabled:
            return
        
        if self.isSITL:
            return
        
        # read the data
        data = self.ser.readline()
        self.ser.reset_input_buffer()

        self.decode_data(data)

        self.send_nozzle_config()

    def decode_data(self, data):
        
        # update the timer
        self.timer = time.time()
        
        # if data is not recieved, store how long it has been without data
        if not data:
            if self.dataNotRecievedStartTime == 0:
                self.dataNotRecievedStartTime = int(self.timer)
            self.dataNotRecievedTime = int(self.timer - self.dataNotRecievedStartTime)
            return
        else:
            # if data is recieved reset the counters
            self.dataNotRecievedStartTime = 0
            self.dataNotRecievedTime = 0
        
        # extract data
        extractedData = self.extract_data(data)
        
        # process data only if something is there in the data
        if extractedData:            
            if (len(extractedData) != 3):
                
                # Check LRC
                lrc = self.calc_lrc(data)
                if (lrc == extractedData[-2]):
                    self.errorNow['INVALID_LRC_RPI'] = False
                    
                    self.update_data(extractedData)
                else:
                    self.errorNow['INVALID_LRC_RPI'] = True
            else:
                if extractedData[1] == 2:
                    logging.info("message reached PIB")
                else:
                    for key in self.errorList.keys():
                        if self.errorList[key] == extractedData[1]:
                            self.errorNow[key] = True


        
    def extract_data(self, data):
        acceptedLengths = [38,3]
        dataLength = len(data)
        
        # Reject data with
        # first character not being $
        # length of data not matching to any of the packet size expected by us
        # second character is out of bound of acceptable function codes
        if (data[0] == ord('$')) and (dataLength in acceptedLengths) and (data[1]<ord('8')):
            self.errorNow['WRONG_DATA'] = False
            
            if dataLength == 3:
                extractedData = struct.unpack('cbc', data)
                return extractedData
            else:
                #extractedData = struct.unpack('cchhhhhhhhhhhhhhhhhBc', data)
                extractedData = struct.unpack('ccHHHHHHHHHHHHHHHHHBc', data)
                return extractedData
                
        else:
            self.errorNow['WRONG_DATA'] = True
            return None
        
    def calc_lrc(self, data):
        lrc = 0
        if type(data) == type('str'):
            for char in data[:-2]:
                lrc  ^= ord(char)
        else:
            # we are assuming integers
            for char in data:
                lrc  ^= char
        
        return lrc
    
    def update_data(self, extractedData):
        # Recheck data length
        if len(extractedData) == 21:
            self.status['ATOMIZER_RPM'] = extractedData[2:8]
            self.status['ATOMIZER_CURRENT'] = np.asarray(extractedData[8:14])/100.
            self.status['PUMP_CURRENT'] = np.asarray(extractedData[14:16])/100.
            self.status['FLOW_METER_READING'] = extractedData[16:18]
            logging.info("Temp, %f"%(extractedData[18]))
            logging.info("RPM, %f, %f, %f, %f, %f, %f"
                         %(self.status['ATOMIZER_RPM'][0], 
                           self.status['ATOMIZER_RPM'][1], 
                           self.status['ATOMIZER_RPM'][2], 
                           self.status['ATOMIZER_RPM'][3], 
                           self.status['ATOMIZER_RPM'][4], 
                           self.status['ATOMIZER_RPM'][5]))
            logging.info("ACURR, %f, %f, %f, %f, %f, %f"
                         %(self.status['ATOMIZER_CURRENT'][0], 
                           self.status['ATOMIZER_CURRENT'][1], 
                           self.status['ATOMIZER_CURRENT'][2], 
                           self.status['ATOMIZER_CURRENT'][3], 
                           self.status['ATOMIZER_CURRENT'][4], 
                           self.status['ATOMIZER_CURRENT'][5]))
            logging.info("PCURR, %f, %f"
                         %(self.status['PUMP_CURRENT'][0], 
                           self.status['PUMP_CURRENT'][1]))
            logging.info("FMReading, %f, %f"
                         %(self.status['FLOW_METER_READING'][0], 
                           self.status['FLOW_METER_READING'][1]))
    
    def send_nozzle_config(self):
        # check serial port is properly open
        if self.ser:
            if self.ser.isOpen():
                data = [ord('$'), self.functionCode['MSG3'], self.nozzleConfiguration]
                lrc = self.calc_lrc(data)
                packedData = struct.pack('cbbbc', '$'.encode(), self.functionCode['MSG3'], self.nozzleConfiguration, lrc, '\n'.encode())
                self.ser.write(packedData)

class FlowSensor:
    # This handles pulse based flow sensors
    def __init__(self, isSITL, pin=11):
        self.isSITL = isSITL
        
        self.pin = pin
        self.count = 0
        self.countList = [0]*10
        
        if not isSITL:
            import pigpio
            self.pi = pigpio.pi()
            self.pi.set_mode(self.pin, pigpio.INPUT)
            self.pi.callback(self.pin, pigpio.RISING_EDGE, self.counter)
            
        self.flowRate = 0
        
        # Calibration Factor
        self.calibFactorMultiPlier = 1.0
        
    def counter(self,g,l,t):
        # Increase count by 1 when pulse arrives
        self.count = self.count + 1
        
    def set_calib_factor_multiplier(self, value):
        self.calibFactorMultiPlier = value
        
    def get_calib_factor_multiplier(self):
        return self.calibFactorMultiPlier
       
    def calc_flow_rate(self):
        # This function will be called at 5 Hz
        count = self.count
        
        # Reset the counter after every 0.2s (5 Hz)
        self.count = 0
        
        actualFlowrate = 0
        
        if self.isSITL:
            count = 5
        
        # Don't record very low flowrate as it can be errorneous
        if count > 4:
            # update the queue with new reading
            self.countList[:-1] = self.countList[1:]
            self.countList[-1] = count
            
            # Log the countlist
            logging.info("FLowSensor Count, %s"%(','.join([str(elem) for elem in self.countList])))
            
            # Use linear model to convert pulse count to flow rate
            a5Hz = 17.195 * 5
            a1Hz = a5Hz/5
            a05Hz = a5Hz/10
            b = -139.97
            flowRate5Hz = a5Hz*self.countList[-1] + b
            flowRate1Hz = a1Hz*(sum(self.countList[5:])) + b
            flowRate05Hz = a05Hz*(sum(self.countList[:])) + b

            # Sort the queue
            sortedCountList1Hz = copy.deepcopy(self.countList[5:])
            sortedCountList05Hz = copy.deepcopy(self.countList[:])
            sortedCountList1Hz.sort()
            sortedCountList05Hz.sort()
            
            # First set the flow rate to the instantaneous flow rate 
            actualFlowrate = flowRate5Hz
            
            # Instantaneous flow will be oscillating quite a lot and may cause 
            # issue in contoller. One way to avoid is to have low pass filter.
            # But low pass filter can cause slow response when fast response is 
            # required. This method ensures that till the certain point low pass
            # average filter is applied but beyond certain point it will be 
            # instantaneous reading
            if abs(sortedCountList1Hz[0]-sortedCountList1Hz[-1]) < 3:
                actualFlowrate = flowRate1Hz
            if abs(sortedCountList05Hz[0]-sortedCountList05Hz[-1]) < 3:
                actualFlowrate = flowRate05Hz
            
            # handling sudden spikes. User previous reading in case of spike
            if abs(flowRate5Hz/1000 - self.flowRate) < 2:
                self.flowRate = self.calibFactorMultiPlier*actualFlowrate/1000.
        else:
            self.flowRate=0
        
class OnlineHealthMonitor:
    def __init__(self):
        pass        
        
