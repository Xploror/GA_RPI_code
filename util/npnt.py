# -*- coding: utf-8 -*-
"""
Created on Sat Jun 27 13:40:16 2020

@author: Sachchit Vekaria
@Organization: General Aeronautics Pvt Ltd
"""

from datetime import datetime
import pytz
import cryptography
from Cryptodome.PublicKey import RSA
from Cryptodome.Hash import SHA256
from Cryptodome.Signature import pkcs1_15
from signxml import XMLVerifier
from lxml import etree
import logging
import jks
import OpenSSL
import base64, textwrap
import threading
import json
import socket
from os import listdir, makedirs, path, remove

class NPNT(object):
    def __init__(self,sitlType):
        # Log Folder
        self.flightLogFolder = "./NPNT/Flight_Logs/"
        self.bundledFlightLogFolder = "./NPNT/Flight_Logs/Bundled/"
        # PA Folder
        self.paFolder = "./NPNT/Permission_Artefact/"
        self.verifiedPAFolder = "./NPNT/Permission_Artefact/Verified/"
        # KeyStore File
        self.keyStoreFile = "./NPNT/Key_Store/Keystore.jks"
        self.pubKeyeFile = "./NPNT/Key_Store/rfm_pub_key.pem"
        # PublicKey File
        self.publicKeyFile = "./NPNT/Key_Store/rfm_key_pair.pub"
        # RFM Information File
        self.rfmInfoFile = "./NPNT/rfm_info"
        # Create Folder Structure if doesn't Exist
        self.create_folder_structure()
        
        self.keystore = jks.KeyStore.load(self.keyStoreFile, "GenAero2016")
        
        # NPNT Status
        self.__npntAllowed = False
        self.__npntNotAllowedReason = 'RPAS Tampered'.encode()
        
        # State Machine NPNT
        self.state = VehicleTamperedState()
        
        # Pre Checks
        self.vtds = VTDS(sitlType)
        self.firmwareVersion = None
        self.firmwareHash = None
        self.rpasId = None
        self.rpasModelId = None
        self.uin = None
        self.lastLogHash = ""
        self.parse_rfm_info()
        
        # PA Related
        self.permissionArtefactFileName = ''
        self.pauin = None
        self.permissionArtefactTree = None
        self.permissionArtefactTreeRoot = None
        self.timeZone = pytz.timezone('Asia/Kolkata')
        
        # GeoFence and Time Limit Related
        self.fence = Fence()
        self.fenceSentToGCS = True
        self.flightStartTime = 0
        self.flightEndTime = 0
        
        # Logging
        self.loggingEntryType = []
        self.loggingTimeStamp = []
        self.loggingLon = []
        self.loggingLat = []
        self.loggingGlobalAlt = []
        self.permissionArtefactId = None
        self.fileIndex = 0
        
        # Lock
        self.lock = threading.Lock()
        
        # Variable to kill all threads cleanly
        self.killAllThread = threading.Event()
        
        # Mavlink Handling
        self.keyRotationRequested = False
        self.uinChangeRequested = None
        self.logDownloadRequest = False
        self.logDownloadDateTime = None
    
    def get_npnt_allowed(self):
        return int(self.__npntAllowed)
        
    def get_npnt_not_allowed_reason(self):
        return self.__npntNotAllowedReason
    
    def create_folder_structure(self):
        if not path.exists(self.flightLogFolder):
            makedirs(self.flightLogFolder)
            
        if not path.exists(self.bundledFlightLogFolder):
            makedirs(self.bundledFlightLogFolder)
            
        if not path.exists(self.paFolder):
            makedirs(self.paFolder)
            
        if not path.exists(self.verifiedPAFolder):
            makedirs(self.verifiedPAFolder)
    
    def parse_rfm_info(self):
        with open(self.rfmInfoFile,'r') as rfmInfo:
            for line in rfmInfo:
                data = line.split(",")
                if len(data) == 2:
                    identifier = data[0].strip()
                    value = data[1].strip()
                    
                    if identifier == "VER":
                        self.firmwareVersion = value
                        
                    if identifier == "VER_CRC32":
                        self.firmwareHash = value
                    
                    if identifier == "RPAS_ID":
                        self.rpasId = value
                        
                    if identifier == "RPAS_MODEL_ID":
                        self.rpasModelId = value
                    
                    if identifier == "UIN":
                        self.uin = value
                        
                    if identifier == "LAST_LOG_HASH":
                        self.lastLogHash = value
                        
    def update_uin(self):
        data = []
        with open(self.rfmInfoFile, 'r') as f:
            for line in f:
                if "UIN" in line:
                    data.append("UIN," + str(self.uinChangeRequested))
                else:
                    data.append(line)
        
        with open(self.rfmInfoFile, 'w') as f:
            f.writelines(data)
            
        self.parse_rfm_info()
        
    def update_last_log_hash(self, lastLogHash):
        data = []
        with open(self.rfmInfoFile, 'r') as f:
            for line in f:
                if "LAST_LOG_HASH" in line:
                    data.append("LAST_LOG_HASH," + str(lastLogHash))
                else:
                    data.append(line)
        
        with open(self.rfmInfoFile, 'w') as f:
            f.writelines(data)
            
        self.parse_rfm_info()
        
    def key_rotation(self):
        if len(listdir(self.bundledFlightLogFolder)) > 0:
            return False

        # generate key
        key = OpenSSL.crypto.PKey()
        key.generate_key(OpenSSL.crypto.TYPE_RSA, 2048)
        
        # generate a self signed certificate
        cert = OpenSSL.crypto.X509()
        cert.get_subject().CN = 'generalaeronautics.com'
        cert.set_serial_number(473289472)
        cert.gmtime_adj_notBefore(0)
        cert.gmtime_adj_notAfter(365*24*60*60)
        cert.set_issuer(cert.get_subject())
        cert.set_pubkey(key)
        cert.sign(key, 'sha256')
        
        # dumping the key and cert to ASN1
        dumped_cert = OpenSSL.crypto.dump_certificate(OpenSSL.crypto.FILETYPE_ASN1, cert)
        dumped_key = OpenSSL.crypto.dump_privatekey(OpenSSL.crypto.FILETYPE_ASN1, key)
        
        # creating a private key entry
        pke = jks.PrivateKeyEntry.new('RFM_Key_Pair', [dumped_cert], dumped_key, 'rsa_raw')
        
        # creating a jks keystore with the private key, and saving it
        keystore = jks.KeyStore.new('jks', [pke])
        keystore.save(self.keyStoreFile, 'GenAero2016')
        
        # Export the public key
        with open(self.pubKeyeFile, "w") as f:
            data = OpenSSL.crypto.dump_publickey(OpenSSL.crypto.FILETYPE_PEM,key).decode("utf-8") 
            f.write(data)
        
        return True
    
    def start_bundling(self):
        # Convert Date and time from string to python datetime structure
        currentDateTime = datetime.strptime(self.logDownloadDateTime, '%Y%m%d_%H%M%S')
        currentDateTimeLocalized = self.timeZone.localize(currentDateTime)
        
        paList = listdir(self.verifiedPAFolder)
        
        # Loop over all verified PA
        for pa in paList:
            # Get PA ID
            paId = pa.split("_")[1]
            
            # Get End Time of PA
            paDateTime = datetime.strptime(pa.split("_")[2], '%Y%m%d%H%M%S')
            paDateTimeLocalized = self.timeZone.localize(paDateTime)
            
            bundledLogList = []
            logEntries = []
            
            # If PA is expired
            if currentDateTimeLocalized > paDateTimeLocalized:
                logList = listdir(self.flightLogFolder)
                
                for log in logList:
                    # Check to remove folders like Bundle folder to be parsed as log
                    if path.isfile(path.join(self.flightLogFolder,log)):
                        paIdLog = log.split("_")[1]
                        if paId == paIdLog:
                            bundledLogList.append(log)
                            with open(path.join(self.flightLogFolder,log), 'r') as f:
                                logJson = json.load(f)
                            logEntries = logEntries + logJson["FlightLog"]["logEntries"]
                    
                # Start Bundling
                # Creating dictionary for flight log
                flightLog = {"FlightLog": {"permissionArtefact": paId,
                                           "previousLogHash": self.lastLogHash,
                                           "logEntries":logEntries
                                           }
                            }
                
                # Signing Flight Log
                rsaKey = RSA.import_key(self.read_key())
                hashedLogData = SHA256.new(json.dumps((flightLog["FlightLog"])).encode())
                self.update_last_log_hash(hashedLogData.hexdigest())
                logSignature = pkcs1_15.new(rsaKey).sign(hashedLogData)
                # the signature is encoded in base64 for transport
                enc = base64.b64encode(logSignature)
                # dealing with python's byte string expression
                flightLog['Signature'] = enc.decode('ascii')
                
                # Creating file name
                logFileName = self.bundledFlightLogFolder + "signed_" + paId + "_log.json"
                
                with open(logFileName, "w") as signedLogFile:
                    json.dump(flightLog, signedLogFile, indent=4)
                
                # Remove individual Flight Log
                for log in bundledLogList:
                    remove(self.flightLogFolder + log)
                
                # Remove PA
                remove(self.verifiedPAFolder + pa)
    
    def parse_permission_artefact(self):
        if len(self.permissionArtefactFileName) == 0:
            return False
        
        self.permissionArtefactTree = etree.parse(self.paFolder + self.permissionArtefactFileName)
        self.permissionArtefactTreeRoot = self.permissionArtefactTree.getroot()
        
        # Read Permission Artifact ID
        self.permissionArtefactId = self.permissionArtefactTreeRoot.get("permissionArtifactId")
        
        # Read UIN from PA
        self.pauin = self.permissionArtefactTreeRoot.find(".//UADetails").get("uinNo")
        
        # Empty the fence data
        self.fence.remove_all_points()
        
        #read Coordinates from PA   
        coordinates = self.permissionArtefactTreeRoot.find(".//Coordinates")
        for x in coordinates.findall('Coordinate'):
            self.fence.add_point((float(x.get('latitude')),float(x.get('longitude'))))
            
        #read time from PA
        flightParams = self.permissionArtefactTreeRoot.find(".//FlightParameters")
        startTime = flightParams.get('flightStartTime')[0:19].replace("T", " ")
        startTimeZone = flightParams.get('flightStartTime')[-6:]
        endTime = flightParams.get('flightEndTime')[0:19].replace("T", " ")
        endTimeZone = flightParams.get('flightEndTime')[-6:]
        if startTimeZone != "+05:30" and endTimeZone != "+05:30":
            logging.info('NPNT, Timezone issue')
            return False
        
        self.flightStartTime = self.timeZone.localize(datetime.strptime(startTime, "%Y-%m-%d %H:%M:%S"), is_dst=None)
        self.flightEndTime = self.timeZone.localize(datetime.strptime(endTime, "%Y-%m-%d %H:%M:%S"), is_dst=None)
        
        self.fenceSentToGCS = False

        return True
    
    def verify_xml_signature(self):
        #certificate = self.read_cert()
        cert = self.permissionArtefactTreeRoot.findtext(".//{http://www.w3.org/2000/09/xmldsig#}X509Certificate")
        
        try:
            XMLVerifier().verify(data=self.permissionArtefactTreeRoot, require_x509=True, x509_cert=cert).signed_xml
            # The file signature is authentic
            return True
        except cryptography.exceptions.InvalidSignature:
            # add the type of exception
            return False
        
    def read_cert(self):
        for alias, c in self.keystore.certs.items():
            return ("\r\n".join(textwrap.wrap(base64.b64encode(c.cert).decode('ascii'), 64)))
    
    def read_key(self):
        for alias, pk in self.keystore.private_keys.items():
            s=("-----BEGIN RSA PRIVATE KEY-----\n")
            s+=("\r\n".join(textwrap.wrap(base64.b64encode(pk.pkey).decode('ascii'), 64)))
            s+=("\n-----END RSA PRIVATE KEY-----")
            return ''.join(s)
        
    def save_verified_pa(self):
        outFileName = self.verifiedPAFolder + "verified_" + str(self.permissionArtefactId) + \
                    "_" + self.flightEndTime.strftime("%Y%m%d%H%M%S") + "_PA.xml"
        
        self.permissionArtefactTree.write(outFileName)
        
        # Update the file index in case the same PA is uploaded after reboot
        for fileName in listdir(self.flightLogFolder):
            if len(fileName)>0 and path.isfile(path.join(self.flightLogFolder, fileName)):
                if fileName.split("_")[1] == self.permissionArtefactId and int(fileName.split("_")[2]) >= self.fileIndex:
                    self.fileIndex = int(fileName.split("_")[2])+1

    def within_time(self, timestamp):
        if timestamp > 0:
            currentTime = pytz.utc.localize(datetime.utcfromtimestamp(timestamp)).astimezone(self.timeZone)
            if self.flightStartTime<=currentTime and currentTime<=self.flightEndTime:
                return True
            else:
                return False
        else:
            logging.info("NPNT, Not getting time from autopilot")
            return False
        
    def write_log(self):
        with self.lock:
            # Convert Data to proper format for json export
            logEntries = []
            for i in range(len(self.loggingEntryType)):
                logEntries.append({"entryType":self.loggingEntryType[i],
                                   "timeStamp":self.loggingTimeStamp[i],
                                   "longitude":self.loggingLon[i],
                                   "latitude":self.loggingLat[i],
                                   "altitude":self.loggingGlobalAlt[i]
                                   })
            
            self.loggingEntryType = []
            self.loggingTimeStamp = []
            self.loggingLon = []
            self.loggingLat = []
            self.loggingGlobalAlt = []
            
        # Creating dictionary for flight log
        flightLog = {"FlightLog": {"permissionArtefact": self.permissionArtefactId,
                                   "previousLogHash": "",
                                   "logEntries":logEntries
                                   }
                    }
        
        # Signing Flight Log
        rsaKey = RSA.import_key(self.read_key())
        hashedLogData = SHA256.new(json.dumps((flightLog["FlightLog"])).encode())
        logSignature = pkcs1_15.new(rsaKey).sign(hashedLogData)
        # the signature is encoded in base64 for transport
        enc = base64.b64encode(logSignature)
        # dealing with python's byte string expression
        flightLog['Signature'] = enc.decode('ascii')
        
        # Creating file name
        logFileName = self.flightLogFolder+"signed_" + self.permissionArtefactId + "_" + str(self.fileIndex) + "_log.json"
        self.fileIndex = self.fileIndex + 1
        
        with open(logFileName, "w") as signedLogFile:
            json.dump(flightLog, signedLogFile, indent=4)
        
    def update(self, lat, lon, globalAlt, hdop, globalTime, isArmed, latestUploadedPAFileName):
        if str(self.state) is str(VehicleTamperedState()):
            self.__npntAllowed = False
            self.__npntNotAllowedReason = 'RPAS Tampered'.encode()
            
            if self.vtds.check_code():
                self.state = self.state.on_event("VTDS code recieved")
                logging.info("NPNT, No Tampering")
            else:
                logging.info("NPNT, RPAS Tampered")
                
        if str(self.state) is str(VehicleNotRegisteredState()):
            self.__npntAllowed = False
            self.__npntNotAllowedReason = 'RPAS is not registered'.encode()
            
            if self.uin:
                self.state = self.state.on_event("UIN available")
                logging.info("NPNT, RPAS is registered")
            else:
                logging.info("NPNT, RPAS is not registered")
            
        # Handling New PA in Between
        if latestUploadedPAFileName:
            if self.permissionArtefactFileName != latestUploadedPAFileName:
                self.permissionArtefactFileName = latestUploadedPAFileName
                self.state = self.state.on_event("PA Uploaded")
                logging.info("NPNT, PA Uploaded")
                
        if str(self.state) is str(PANotUploadedState()):
            self.__npntAllowed = False
            self.__npntNotAllowedReason = 'PA Not Uploaded'.encode()
            
            logging.info("NPNT, PA Not Uploaded")
                
        if str(self.state) is str(PANotParsedState()):
            self.__npntAllowed = False
            self.__npntNotAllowedReason = 'PA Not Parsed'.encode()
            
            if self.parse_permission_artefact():
                self.state = self.state.on_event("PA Parsed")
                logging.info("NPNT, PA Parsed")
            else:
                logging.info("NPNT, PA Not Parsed")
                
        if str(self.state) is str(UINNotCorrectState()):
            self.__npntAllowed = False
            self.__npntNotAllowedReason = 'UIN Not Correct'.encode()
            
            if self.uin == self.pauin:
                self.state = self.state.on_event("UIN Matched")
                logging.info("NPNT, UIN Matched")
            else:
                logging.info("NPNT, UIN Not Correct")
                
        if str(self.state) is str(PANotAuthenticState()):
            self.__npntAllowed = False
            self.__npntNotAllowedReason = 'PA not Authentic'.encode()
            
            if self.verify_xml_signature():
                self.state = self.state.on_event("PA Signature Verified")
                logging.info("NPNT, PA Signature Verified")
            else:
                logging.info("NPNT, PA not Authentic")
                
        if str(self.state) is str(PANotStoredState()):
            self.save_verified_pa()
            
            self.state = self.state.on_event("PA Stored")
            
            logging.info("NPNT, PA Stored")
        
        if str(self.state) is str(OutsideTimelimitState()):
            self.__npntAllowed = False
            self.__npntNotAllowedReason = 'Outside Time Limit'.encode()
            
            if lat > -190 and lon > -190 and hdop < 2:
                if self.within_time(globalTime):
                    self.state = self.state.on_event("Within Allowed Time")
                    logging.info("NPNT, Within TimeLimit")
                else:
                    logging.info("NPNT, Outside Time Limit")    
            else:
                self.__npntNotAllowedReason = 'No GPS Lock'.encode()
                logging.info("NPNT, No GPS Lock")
                
        if str(self.state) is str(OutsideGeofenceState()):
            self.__npntAllowed = False
            self.__npntNotAllowedReason = 'Outside GeoFence'.encode()
            
            if lat > -190 and lon > -190 and hdop < 2:
                if self.fence.check_point((lat,lon)):
                    self.state = self.state.on_event("Within Geofence")
                    logging.info("NPNT, Within Geofence")
                else:
                    logging.info("NPNT, Outside GeoFence")    
            else:
                self.__npntNotAllowedReason = 'No GPS Lock'.encode()
                logging.info("NPNT, No GPS Lock")
                
        if str(self.state) is str(ArmAllowedState()):
            self.__npntAllowed = True
            self.__npntNotAllowedReason = 'Go go go'.encode()
            
            if isArmed:
                self.state = self.state.on_event("Armed")
            else:
                self.state = self.state.on_event("Not Armed")
                
            logging.info("NPNT, Allowing to Arm")    
            
        if str(self.state) is str(TakeoffLocationNotRecorededState()):
            self.loggingEntryType.append("TAKEOFF/ARM")
            self.loggingTimeStamp.append(globalTime)
            self.loggingLon.append(lon)
            self.loggingLat.append(lat)
            self.loggingGlobalAlt.append(int(globalAlt))
            
            self.state = self.state.on_event("Takeoff location stored")
            
            logging.info("NPNT, TakeOff Point Recorded")
            
        if str(self.state) is str(FlyingState()):
            if not self.fence.check_point((lat,lon)):
                self.state = self.state.on_event("Breached")
                logging.info("NPNT, Geofence Breach")
                
            if not self.within_time(globalTime):
                self.state = self.state.on_event("Breached")
                logging.info("NPNT, Time Breach")
                
            if not isArmed:
                self.state = self.state.on_event("Landed")
                logging.info("NPNT, Landed")
            else:
                logging.info("NPNT, Flying")
            
        if str(self.state) is str(FlyingBreachedState()):
            self.loggingEntryType.append("BREACHED")
            self.loggingTimeStamp.append(globalTime)
            self.loggingLon.append(lon)
            self.loggingLat.append(lat)
            self.loggingGlobalAlt.append(int(globalAlt))
            
            if not isArmed:
                self.state = self.state.on_event("Landed")
                logging.info("NPNT, Landed")
            else:
                logging.info("NPNT, Breached")
            
        if str(self.state) is str(LandLocationNotRecordedState()):
            self.loggingEntryType.append("LAND/DISARM")
            self.loggingTimeStamp.append(globalTime)
            self.loggingLon.append(lon)
            self.loggingLat.append(lat)
            self.loggingGlobalAlt.append(int(globalAlt))

            self.state = self.state.on_event("Land Location Stored")
                
            logging.info("NPNT, Land Location Stored")
            
        if str(self.state) is str(FlightLogNotCreatedState()):
            logWriteThread = threading.Thread(target=self.write_log)
            logWriteThread.daemon = True
            logWriteThread.start()
            
            self.state = self.state.on_event("Log Created")
                
            logging.info("NPNT, Log Created")

    def kill_all_threads(self):
        logging.info("NPNT killing all threads")
        self.killAllThread.set()
        logging.info("NPNT joined all threads")

class VTDS(object):
    def __init__(self, sitlType):
        # Enabling VTDS
        self.vtdsEnabled = False
        
        # Handling for SITL
        self.isSITL = False
        if sitlType is not None or not self.vtdsEnabled:
            self.isSITL = True
            
        channel = 1     #I2C1 on Pin 3 & 5
        self.addr = 0x04     #I2C address of VTDS Board
        self.correctCode = "ga123"
            
        if not self.isSITL:
            from smbus2 import SMBus
            self.bus=SMBus(channel)
            
    def check_code(self):
        if self.isSITL:
            return True
        else:
            buf = self.bus.read_i2c_block_data(self.addr,0,5) #read 5 bytes with 0 offset
            
            stringBuf = ''
            for character in buf:
                stringBuf = stringBuf + chr(character)
            
            if(stringBuf == self.correctCode):
                logging.info("VTDS, Correct Code Recieved")
                return True
            else:
                logging.info("VTDS, Wrong Code Recived: %s"%(stringBuf))
                return False
            

class State(object):
    """
    We define a state object which provides some utility functions for the
    individual states within the state machine.
    """

    def __init__(self):
        pass
#        logging.info('Processing current state: %s', str(self))

    def on_event(self, event):
        """
        Handle events that are delegated to this State.
        """
        pass

    def __repr__(self):
        """
        Leverages the __str__ method to describe the State.
        """
        return self.__str__()

    def __str__(self):
        """
        Returns the name of the State.
        """
        return self.__class__.__name__

# NPNT States
class VehicleTamperedState(State):
    """
    State in which vehicle is tampered
    This is the state when we power on
    """
    def on_event(self, event):
        # Once we recieve code from VTDS state will move on 
        # to not registered state
        if event == 'VTDS code recieved':
            return VehicleNotRegisteredState()

        return self
    
class VehicleNotRegisteredState(State):
    """
    State in which UIN is not available for RPAS
    """
    def on_event(self, event):
        # IF UIN is avaliable in rfminfo file then move
        # to PA not uploaded Status
        if event == 'UIN available':
            return PANotUploadedState()

        return self

class PANotUploadedState(State):
    """
    State in which PA is not uploaded for RPAS
    """
    def on_event(self, event):
        # If PA is uploaded then move
        # to PA not Parsed state
        if event == 'PA Uploaded':
            return PANotParsedState()

        return self
    
class PANotParsedState(State):
    """
    State in which PA is not parsed
    """
    def on_event(self, event):
        # If PA is parsed then move on to 
        # to PA not Parsed state
        if event == 'PA Parsed':
            return UINNotCorrectState()
        
        if event == 'PA Uploaded':
            return PANotParsedState()

        return self
    
class UINNotCorrectState(State):
    """
    State in which UIN in PA and RPAS PA doesn't matches
    """
    def on_event(self, event):
        # If UIN Matches move on to 
        # to PA not Parsed state
        if event == 'UIN Matched':
            return PANotAuthenticState()
        
        if event == 'PA Uploaded':
            return PANotParsedState()

        return self

class PANotAuthenticState(State):
    """
    State in which PA signature is not verified
    """
    def on_event(self, event):
        # If PA signature is verified then move to
        # PA not stored
        if event == 'PA Signature Verified':
            return PANotStoredState()
        
        if event == 'PA Uploaded':
            return PANotParsedState()

        return self

class PANotStoredState(State):
    """
    State in which PA is not stored in memory
    """
    def on_event(self, event):
        # If PA storage process is complete then move to
        # Time Outside Limit State
        if event == 'PA Stored':
            return OutsideTimelimitState()
        
        if event == 'PA Uploaded':
            return PANotParsedState()

        return self
    
class OutsideTimelimitState(State):
    """
    State in which current time is outside Timelimit allowed in PA
    """
    def on_event(self, event):
        # If current GPS time within PA timelimit then move to
        # Outside geofence state
        if event == 'Within Allowed Time':
            return OutsideGeofenceState()
        
        if event == 'PA Uploaded':
            return PANotParsedState()

        return self
    
class OutsideGeofenceState(State):
    """
    State in which current location is outside geofence allowed in PA
    """
    def on_event(self, event):
        # If current location within PA geofence then move to
        # Allowed to arm
        if event == 'Within Geofence':
            return ArmAllowedState()
        
        if event == 'PA Uploaded':
            return PANotParsedState()

        return self
    
class ArmAllowedState(State):
    """
    State in which Arming Allowed
    """

    def on_event(self, event):
        # If vehicle actually arms
        # Then move to flight log take off not recorded state
        # If it is not armed move to Outside Timelimit state (for 1 Hz Loop)
        if event == 'Armed':
            return TakeoffLocationNotRecorededState()
        
        if event == 'Not Armed':
            return OutsideTimelimitState()
            
        if event == 'PA Uploaded':
            return PANotParsedState()
        
        return self
    
class TakeoffLocationNotRecorededState(State):
    """
    State in which takeoff location is not stored after arming
    """

    def on_event(self, event):
        # If Takeoff Location is stored
        # Move to  Breached check State
        if event == 'Takeoff location stored':
            return FlyingState()
        
        return self
    
class FlyingState(State):
    """
    State in which RPAS is flying
    """

    def on_event(self, event):
        # If GeoFence is breached or time breached
        if event == 'Breached':
            return FlyingBreachedState()
        
        if event == 'Landed':
            return LandLocationNotRecordedState()
        
        return self
    
class FlyingBreachedState(State):
    """
    State in which RPAS is flying after breaching either Time or Geofence
    """

    def on_event(self, event):
        # If Landed        
        if event == 'Landed':
            return LandLocationNotRecordedState()
        
        return self
    
class LandLocationNotRecordedState(State):
    """
    State in which RPAS has landed but yet to record Land location
    """

    def on_event(self, event):
        # If Land Location is recorded then move on to
        # Flight log not created
        if event == 'Land Location Stored':
            return FlightLogNotCreatedState()
        
        return self
    
class FlightLogNotCreatedState(State):
    """
    State in which RFM is yet to store individual log
    """

    def on_event(self, event):
        # If flight log is create move on to
        # Outside Time Limit state in PA loop so that take off can be taken up
        if event == 'Log Created':
            return OutsideTimelimitState()
        
        return self
    
# Class to define a fence.
class Fence:
    """
    A Fence object is primarily a way of containing a list of points that form
    a boundary, and providing the ability to add / change points in the fence.
    """

    points = None

    def __init__(self):
        self.points = []
        
    def remove_all_points(self):
        self.points = []

    def add_point(self, point):
        # Add new point to points list
        self.points.append(point)

    def list_points(self):
        return(self.points)

    def check_point(self, point, debug = False):
        """
        check_point()  checks if a given point lies inside a given fence polygon.
        Parameters are given as an instance of the fence class and a given point as
        a, tuple of (x, y).
        """
        def check_in_bounds(point, line_eqns_index):
            """
            check_in_bounds() checks if a supplied point is within the upper and
            lower x & y bounds of the points that form a given line.

            Takes the point to be checked and the index that points to which line
            is being tested. Knowing the location of the line equation in its list
            we can find the points that it is formed from.

            Returns true if the point is within bound and false if it's outside
            bounds.
            """
            withinX = False;
            withinY = False;

            pointA = self.points[line_eqns_index]
            # If the index is pointing to the last member of the list, the line
            # is made of points that wrap around back to the start of the points
            # list.
            if line_eqns_index + 1 == len(self.points):
                pointB = self.points[0]
            else:
                pointB = self.points[line_eqns_index + 1]

#            print(point)
#            print(pointA, pointB)
            # Check if point[x] is within pointA[x] and pointB[x].
            if (pointA[0] >= pointB[0]):
                # pointA is more positive than pointB so check if point[x] is
                # between these.
                if (point[0] <= pointA[0] and point[0] >= pointB[0]):
                    withinX = True
            elif (pointA[0] <= pointB[0]):
                # pointA is less positive than pointB.
                if (point[0] >= pointA[0] and point[0] <= pointB[0]):
                    withinX = True

            # Check if point[y] is within pointA[y] and pointB[y].
            if (pointA[1] >= pointB[1]):
#                print("pointA[1] >= pointB[1]")
                # pointA is more positive than pointB so check if point[y] is
                # between these.
                if (point[1] <= pointA[1] and point[1] >= pointB[1]):
                    withinY = True
            elif (pointA[1] <= pointB[1]):
#                print("pointA[1] <= pointB[1]")
                # pointA is less positive than pointB.
                if (point[1] >= pointA[1] and point[1] <= pointB[1]):
                    withinY = True

#            print(withinX, withinY)

            if withinX and withinY:
                return True
            else:
                return False

        # Find numbe of intersections of the fence with the point horizon line on
        # either side of the point.
        def find_intersect(line, point_horizon_eqn):
            """
            find_intersect() returns a coordinate if an intersection exists, and
            False if not.

            Uses Cramers rule to work it out.
            """
            # Calculate the determinant.
            D  = line[0] * point_horizon_eqn[1] - line[1] * point_horizon_eqn[0]
            Dx = line[2] * point_horizon_eqn[1] - line[1] * point_horizon_eqn[2]
            Dy = line[0] * point_horizon_eqn[2] - line[2] * point_horizon_eqn[0]
            if D != 0:
                x = Dx / D
                y = Dy / D
                return (x, y)
            else:
                return False

        # First, find the horozontal line equation that passes through the point.
        # Using the Ax + By = C fomat, express as a tuple of (A, B, C).
        point_horizon_eqn = (0, 1, point[1])

        # Next, form equations with the list of points given by the Fence object.
        if len(self.points) < 3:
            raise Exception("The supplied fence has not enough (< 3) points!")
        # Form list of line equations. Last in list will circle back to first point.
        # E.g. self.points = [(A), (B), (C)], line_eqns = [(AB), (BC), (CA)].
        # Where each element in the line_eqns list is a tuple of (m, c).
        line_eqns = []
        for point_index in range(len(self.points)):
            point1 = self.points[point_index]
            # If point1 is the last in the list, point2 is the first element of the
            # list.
            if point_index == (len(self.points) - 1):
                point2 = self.points[0]
            else:
                point2 = self.points[point_index + 1]
            # Check if vertical or horizontal line first.
            if point1[1] == point2[1]:
                # Hoizontal
                a = 0
                b = 1
                c = point1[1]
#                if debug == True:
#                    print("Formed equation", str(a) + "x+" + str(b) + "y=" + str(c), "from points:", point1, "and", point2)
                line_eqns.append((a, b, c))
            elif point1[0] == point2[0]:
                # Vertical
                a = 1
                b = 0
                c = point1[0]
#                if debug == True:
#                    print("Formed equation", str(a) + "x+" + str(b) + "y=" + str(c), "from points:", point1, "and", point2)
                line_eqns.append((a, b, c))
            else:
                # Non vertical or hoizontal line.
                a = (-1 * ((point2[1] - point1[1]) / (point2[0] - point1[0])))
                b = 1
                # c = y - ax
                c = (point1[1] + (a * point1[0]))
#                if debug == True:
#                    print("Formed equation", str(a) + "x+" + str(b) + "y=" + str(c), "from points:", point1, "and", point2)
                line_eqns.append((a, b, c))

#        if debug == True:
#            print("\n")
#            print("All equations formed (in order):", line_eqns)
#            print("Finding intersections...\n")
#            print("\nx bounds are:", str(self.max_x), str(self.min_x), "y bounds:", str(self.max_y), str(self.min_y))
        intersection_points_left = []
        intersection_points_right = []
        for line_index in range(0, len(line_eqns)):
            intersection_point = find_intersect(line_eqns[line_index], point_horizon_eqn)
#            if debug == True:
#                print("Intersection point between lines:", line_eqns[line_index], "&", point_horizon_eqn, "is:", intersection_point)
            if intersection_point != False:
                if intersection_point[0] < point[0]:
                    # Intersection point x value is less than point x value.
                    if (intersection_point not in intersection_points_left) and check_in_bounds(intersection_point, line_index) == True:
                        intersection_points_left.append(intersection_point)
                else:
                    # Intersection point x value is greater than point x value.
                    if (intersection_point not in intersection_points_right) and check_in_bounds(intersection_point, line_index) == True:
                        intersection_points_right.append(intersection_point)

        # Check if the number of intersections to the left and right are odd.
        if len(intersection_points_left) > 0 and len(intersection_points_right) > 0:
#            if debug == True:
#                print("\n")
#                print((str(len(intersection_points_left))), "intersection points to the left.")
#                print((str(len(intersection_points_right))), "intersection points to the right.")
            if ((len(intersection_points_left) % 2) == 1) and ((len(intersection_points_right) % 2) == 1):
                # Both have odd intersection counts, so the point is in the Fence.
                return(True)
            else:
#                print(intersection_points_left, intersection_points_right)
                return(False)
        else:
#            print(intersection_points_left, intersection_points_right)
            return(False)

def convertDMSToDD(degrees, minutes, seconds):
    return(degrees + (minutes / 60) + (seconds / 3600))

   
def fence_coordinates(g_fence):
    n = len(g_fence)
    my_fence = Fence()
    for i in range(0,n):
        my_fence.add_point((g_fence[i][0],g_fence[i][1]))
        
    return my_fence

def drone_inside_fence(lat,lon,my_fence):
    geofencce_output = my_fence.check_point((lat,lon))
    
    return geofencce_output


#npnt=NPNT()
#npnt.parse_permission_artefact()
