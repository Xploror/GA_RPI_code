"""
@author: Dhruv Parikh
@organisation: General Aeronautics Pvt. Ltd.
@date: 1-03-2022
"""

"""
@Description:
    Sensor driver for lidar/radar
    @Todo: Add UART driver support
           Add CAN driver support
"""


import math
import socket
import math
import logging
import threading
import serial
import time
import struct

class SensorDriver():
    def __init__(self,drivertype):
        """Initialiser for Sensor Driver

        Args:
            drivertype (String): Put in SITL or RPLidar
        """
        self.HOST = None
        self.raw_data = [40]*360
        self.drivername = drivertype
        
        #Connect to local host and port. This connects to the C++ bridge
        if drivertype == 'SITL':
            self.HOST, self.PORT = "localhost", 8080
            self.raw_data = [0]*10
            self.lid = []

        #Connect to actual RPLidar. Warning: Pretty unstable code 
        else:
            self.PORT = '/dev/ttyUSB1'
            self.START_FLAG = b"\xA5"
            self.HEALTH_CMD = b"\x52"
            self.GET_INFO = b"\x50"
            self.RESET = b"\x40"
            
            self.STOP = b"\x25"
            self.START_SCAN = b"\x20"
            self.HEALTH = 1
            self.INFO = 2
            self.SCAN = 3
            self.RESPONSE_FLAG = b"\x5A"
            
            self.master_angle = []
            self.master_distance = []
            
            self.scan_data = None
            

            

    def connect_and_fetch(self):
        """Connects to the sensor and starts the scan request
        """
        #SITl
        if self.HOST != None:
            self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.s.connect((self.HOST, self.PORT))
            logging.info("Bridge initialised")
        
        #RPLidar
        else:
            #Connect to the sensor
            self.lidar_connection = serial.Serial(self.PORT,baudrate=115200,parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE,timeout=1)
            #Sleep for 2 Seconds
            time.sleep(2)
            #Send Stop Scan request
            self.send_stopscan_request()
            print("Stopped")
            #Wait for few ms
            time.sleep(0.002)
            #Reset the lidar
            self.send_reset_request()
            #Stop the motor
            self.set_pwm(0)

            time.sleep(2)
            #Clear the buffer
            self.clear_input_buffer()            
            
            #Start the motor
            self.set_pwm(866)



            time.sleep(1)
            #Send the request to start scanning - by this time, the motor would be at constant rate and hence should output data immediately
            self.start_scan_request()
            #Read the header of the response packet
            self.read_response()
    
    def read_fast(self):
        """
        Read all stack of 4000 bytes from the buffer. 
        This function has to be called only once - create a seperate thread
        """
        while True:
            if self.lidar_connection.inWaiting()>300:
                self.scan_data = self.pass_raw_data(4000)

    def clear_input_buffer(self):
        """Clears the input buffer multiple times
        We will make sure that my fragile and non robust driver won't have any problems with the header

        @Warning:
            Don't Call haphazardly. 
        """
        for i in range(1000):
            self.lidar_connection.reset_input_buffer()

    def give_scan_values(self):
        """
        Parse the readings, append the angle and distance from it. 
        Note that every new scan, the old scan values are discarded. 
        This is okay for memory based sense and stop as it stores the values anyway.
        """
        while True:
            time.sleep(0.0001)
            #If there is data populated
            if self.scan_data is not None:
                #Protect the instant data -> Tip: We need such protectors in every function 
                data = self.scan_data

                #For each 5 byte packet, parse the readings according to datasheet
                for i in range(0,len(data),5):

                    new_scan, angle, distance = self.parse_scan_readings(data[i:i+5])
                    #If we get new scan, remove all the previous data and start appending
                    if new_scan:
                        self.master_angle = []
                        self.master_distance = []
                        self.master_angle.append(angle)
                        self.master_distance.append(distance) 
                    #This flag is output of the parse_scan_readings when things go wrong. 
                    elif new_scan == -1:
                        print("problem")
                    
                    #Keep appending till we get new scan header
                    else:
                        self.master_angle.append(float(angle))
                        self.master_distance.append(float(distance))
                    
    def update_rplidar(self):
        """
        @Description: 
        Populates the lidar data in the raw_data var
        If lidar data outputs more than 10 measurements per scan, then we will consider the output
        The data is downsampled to resolution of 1 degree to be fed to the SAADataHandler
        """

        lid = []
        ang = []
        mag = [40]*360
        #print(mag)

        
        #Saving the values so they dont get updated
        #This is an example of the many length based checks I have done to ensure that data is there before me start dividing stuff by zero

        master_angles_copy = self.master_angle
        master_distance_copy = self.master_distance
        if len(master_angles_copy)>2 and (len(master_distance_copy)==len(master_angles_copy)):
            angles = master_angles_copy
            distance = master_angles_copy
        else:
            #Wait for some time to populate a few more readings
            time.sleep(0.001)
            if((len(self.master_distance)==len(self.master_angle))):
                angles = self.master_angle
                distance = self.master_distance
            else:
                master_angles_copy = self.master_angle
                master_distance_copy = self.master_distance

                min_len = min(len(master_distance_copy),len(master_angles_copy))
                print(min_len)
                angles = master_angles_copy[0:min_len-1]
                distance = master_distance_copy[0:min_len-1]
        
        
        #Reset the vars
        mag = [40]*360

        #Number of measurement in this scan
        no_of_scans = len(angles)
        #If scan is valid
        if no_of_scans>2:
            #Append the data to lidar and magnitude
            for j in range(no_of_scans-1):
                #Sometimes the index error occurs
                    # lid.append(float(scan[j][2]))
                    #Downsampling the angular reading
                    #if ang = 1.2, it will be converted to 1
                    # ang.append(int(math.floor(scan[j][1])))
                ang = (math.floor(angles[j]))
                if ang>359 or ang<0:
                    ang = 0
                try:
                    mag[ang] = float(distance[j])/1000
                except:
                    print(len(angles),j,no_of_scans)

            #Assign to the global var once the data is populated
            self.raw_data = mag

        else:
            #If no of scans are less, then assign the default unusable reading
            self.raw_data = mag
        
    def return_readings(self):
        """Raw data accessor

        Returns:
            1D array of length 360
        """
        return self.raw_data


    def update_sitl_sensor(self):
        """This is for Gazebo SITL. It just decodes the sockets 
        """

        #Empty the array for new fetch
        lid = []

        #empty the data
        data = None

        #Keep fetching until the data is filled with 64 indices
        while len(lid)<=64:
            data = self.s.recv(4).decode("utf-8") 
            #If we recieve a new packet of data
            if data == 'new ':
                if len(lid) == 64:
                    self.raw_data = lid
                    lid = []             
            elif data != None:
                lid.append(float(data))
                
    def send_health_request(self):
        """Sends the health request. It doesn't handle the recieve commands
        """
        if self.lidar_connection.is_open:
            buf = self.START_FLAG + self.HEALTH_CMD
            self.lidar_connection.write(buf)
            self.request = self.HEALTH

    def send_getinfo_request(self):
        self.send_cmd(self.GET_INFO)
        self.request = self.INFO

    def send_reset_request(self):
        """This function doesnt work that well. Need more tests. Kindly don't use
        """
        self.lidar_connection.setDTR(True)
        self.send_cmd(self.RESET)
    
    
    def send_stopscan_request(self):
        """Stops the scan.
        """
        self.lidar_connection.setDTR(True)
        self.send_cmd(self.STOP)
    
    def start_scan_request(self):
        """Starts the scan. 
        """
        self.lidar_connection.setDTR(True)
        self.send_cmd(self.START_SCAN)
        self.request = self.SCAN
        
    def send_cmd(self,cmd):
        """Sends the command to the RPLidar. Since each command is send using the start flag, this common function
        helps.

        Args:
            cmd (Byte String): Declare the vars in the class variables
        """
        if self.lidar_connection.is_open:
            buf = self.START_FLAG + cmd
            self.lidar_connection.write(buf)
        
    def read_response(self):
        """This function reads predefined responses. 
        @TODO: A better implementation is required.

        Returns:
            Your byte data
        """
        if self.lidar_connection.is_open:
            #Based on request, the amount of bytes to read is defined and headers are checked. 
            if self.request == self.HEALTH:
                data_len = 3
                data = self.lidar_connection.read(7)
            if self.request ==self.INFO:
                data_len = 20
                data = self.lidar_connection.read(7)
            if self.request == self.SCAN:
                data_len = 0
                data = self.lidar_connection.read(7)
                
            return self.check_data_header(data,data_len)
            
    def check_data_header(self,data,data_len):
        """This function checks the data header. Without this, no data can be read.

        Args:
            data (Bytesarray): Bytes array of header
            data_len (int): Amount of data to read for that specific request

        Returns:
            bytesarray: Returns the byte array if all checks are passed
        """
        if len(data) != 7:
            print("Descriptor length mismatch")
            return None
        elif not data.startswith(self.START_FLAG + self.RESPONSE_FLAG):
            print("Incorrect descriptor starting bytes")
            return None
        else:
            if data_len!=0:
                data = (self.lidar_connection.read(data_len))
                return data
        
    def interprete_data(self,data):
        """Unused function from initial design

        """
        if len(data) > 0:
            if self.request == self.HEALTH:
                self.device_health = data[0]
            # if self.request == self.motor
        print(data)

    def _send_payload_cmd(self, cmd: bytes, payload: bytes) -> None:
        """Sends the payload data (for motor spin). I have taken it from slamtec library.

        Args:
            cmd (bytes): MOTOR START
            payload (bytes)
        """
        size = struct.pack("B", len(payload))
        req = self.START_FLAG + cmd + size + payload
        
        checksum = 0
        for v in struct.unpack("B" * len(req), req):
            checksum ^= v
        req += struct.pack("B", checksum)
        self.lidar_connection.write(req)
        #Indicator that the command is sent
        print('Command sent: %s' % self._showhex(req))
        
        
        
    def set_pwm(self, pwm: int) -> None:
        """Just call this function to start the motor

        Args:
            pwm (int): Keep it above 800 pwm
        """
        self.lidar_connection.setDTR(False)
        payload = struct.pack("<H", pwm)
        yo = b"\xF0"
        self._send_payload_cmd(yo, payload)
             
    
    def _showhex(self,signal):
        '''Converts string bytes to hex representation (useful for debugging)'''
        return [format((b), '#02x') for b in signal]
    
    
    def get_scan_data(self):
        """Master function for getting the scan data
        """
        self.check_data_header()
        self.lidar_connection.read()
        
    def pass_raw_data(self,bytes):
        """Passes the raw data -> Just a read handler

        Args:
            bytes (int): Number of bytes

        Returns:
            bytesarray
        """
        return self.lidar_connection.read(bytes)
            


    def parse_scan_readings(self,raw:bytes):

        if len(raw)<5:
            print("Length not enough")
            return -1,0,0
        else:
            new_scan = bool(raw[0] & 0b1)
            inversed_new_scan = bool((raw[0] >> 1) & 0b1)
            quality = raw[0] >> 2
            if new_scan == inversed_new_scan:
                print("New scan flags mismatch")
                # self.send_stopscan_request()
                self.lidar_connection.flushInput()
                
                return -1,0,0
            else:
                angle = ((raw[1] >> 1) + (raw[2] << 7)) / 64.0
                distance = (raw[3] + (raw[4] << 8)) / 4.0
                return new_scan,angle,distance


    def _process_scan(self,raw: bytes):
        """Processes input raw data and returns measurement data"""
        if len(raw)==5:
            new_scan = bool(raw[0] & 0b1)
            inversed_new_scan = bool((raw[0] >> 1) & 0b1)
            quality = raw[0] >> 2
            if new_scan == inversed_new_scan:
                print("New scan flags mismatch")
                return 0,0,0,0
            check_bit = raw[1] & 0b1
            if check_bit != 1:
                print("Check bit not equal to 1")
                return 0,0,0,0
            
            angle = ((raw[1] >> 1) + (raw[2] << 7)) / 64.0
            distance = (raw[3] + (raw[4] << 8)) / 4.0
            return new_scan, quality, angle, distance
