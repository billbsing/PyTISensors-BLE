#  This program is free software; you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation; either version 2 of the License, or
#  (at your option) any later version.
#  
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#  
#  You should have received a copy of the GNU General Public License
#  along with this program; if not, write to the Free Software
#  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
#  MA 02110-1301, USA.


"""

	GATTServer
	
	This module provides the GATTServer and connection sessions to the bluetooth LE devices.
	
	
	
	

"""

import serial, struct, logging, time, binascii, re, socket

class Dongle:
	"""
	Information about the connected BLE dongle. 
	"""
	def __init__(self):
		self.address = None
		self.IRK = None
		self.CSRK = None
		self.ready = False
	
class RemoteDevice:
	"""
	Information stored about the remote device after a discovery.
	"""
	
	def __init__(self):
		self.address = None
		self.name = None
	
	def asString(self):
		return "%s '%s'" % (self.address, self.name)
		
class ConnectSession:
	"""
		Returned after as succesfull connect to a remote BLE device.
	"""	

	def __init__(self, gattServer):
		self._gattServer = gattServer
		self.addressType = None
		self.address = None
		self.connectionHandle = None
		self.interval = 0
		self.latency = 0
		self.timeout = 0
		self.clockAccuracy = 0
		self.responseList = []
		self.responseValue = None
		self.errorCode = 0	
		self.OnEventNotification = None

	def discoverByHandle(self, handle, timeout = None):
		"""
		Returns value/s from the remote device based on the handle number 'handle'
		
		If timeout is set then the number of seconds to wait for a reply
		"""
		self.errorCode = 0
		self.responseValue = None
		self._gattServer.sendATTDiscCharsByHandle(self.connectionHandle, handle, timeout)
		return self.responseValue

	def discoverByUUID(self, uuid, timeout = None):
		"""
		Returns value/s from the remote device based on the UUID.

		If timeout is set then the number of seconds to wait for a reply
		"""
		self.errorCode = 0
		self.responseList = []
		self._gattServer.sendATTDiscCharsByUUID(self.connectionHandle, uuid, timeout)
		return self.errorCode

	def writeRequestByte(self, handle, value):
		"""
		Send a byte value to the remote device using the handle number
		"""
		self.errorCode = 0
		self._gattServer.sendATTWriteRequest(self.connectionHandle, handle, struct.pack('<B', value))
		return self.errorCode

	def writeRequestWord(self, handle, value):
		"""
		Send a word value to the remote device using the handle number
		"""
		self.errorCode = 0
		self._gattServer.sendATTWriteRequest(self.connectionHandle, handle, struct.pack('<H', value))
		return self.errorCode
		
	def writeRequest(self, handle, data):
		"""
		Send data to the remote device using the handle number
		"""
		self.errorCode = 0
		self._gattServer.sendATTWriteRequest(self.connectionHandle, handle, data)
		return self.errorCode
		
		
		
class Device:
	"""
	Base class for a BLE usb dongle device
	"""
	def open(self, deviceName):
		"""
		open the device for reading / writing using the deviceName
		"""
		pass
	
	def read(self, byteLength = 0):
		"""
		read bytes of bytelength from the device, if bytelength = 0 then read until timeout, return a bytearray of data
		"""
		pass
	
	def write(self, data):
		"""
		write data to the device , the data is a bytearray of data to write
		"""
		pass
	
	
class SerialDevice(Device):
	"""
	Serial BLE device, this is usually the TI development USB dongle.
	"""
	def open(self, deviceName):
		self._deviceName = deviceName
		self._serial = serial.Serial(self._deviceName, 57600, timeout = 1)
		
	def read(self, byteLength = 0):
		return self._serial.read(size=byteLength)
		
	def write(self, data):
		return self._serial.write(data)
		
		
class SocketDevice(Device):
	"""
	Socket client to communicate with standard BLE USB dongles. 
	
	*** Still working on this not sure if this will work ? ****
	
	"""
	def open(self, deviceName):
		self._deviceName = deviceName
		self._socket = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_RAW, socket.BTPROTO_HCI)
		self._socket.setsockopt(socket.SOL_HCI, socket.HCI_DATA_DIR,1)
		self._socket.setsockopt(socket.SOL_HCI, socket.HCI_TIME_STAMP,1)
		self._socket.setsockopt(socket.SOL_HCI, socket.HCI_FILTER, struct.pack("IIIh2x", 0xffffffff, 0xffffffff, 0xffffffff, 0)) #type mask, event mask, event mask, opcode
		self._socket.settimeout(0.5)
		print("Connecting to device '%s'" % self._deviceName)
		#self._socket.bind((self._deviceName, 1))
		#self._socket.listen(1)
		#self._socketClient, self._address = self._socket.accept()
	
	def read(self, byteLength = 0):
 		#print("Read serial %d" % byteLength) 
		return self._socket.recv(byteLength)

	def write(self, data):
		return self._socket.send(data)


class GATTServer:
	"""
	The General Attribute Server.
	This class is used to communicate with another remote BLE device.
	
	"""		

	def open(self, deviceName, timeout = 5):
		"""
		Open the USB device to start communications.
		deviceName - is the device to use to talk to the dongle ( linux usually /dev/ttyACM0 )
		timeout		- number of seconds to wait before deciding that there is no more data, defaults to 5 seconds
		"""
		if re.search('dev', deviceName):
			self._device = SerialDevice()
		else:
			self._device = SocketDevice()
		self._device.open(deviceName)
		
		self._commandTable = {
								0x067F: self._GAPExtCommandStatus,
								0x0600: self._GAPDeviceInitDone,
								0x0601: self._GAPDeviceDiscoveryDone,
								0x006D: self._GAPDeviceInformationEvent,
								0x060D: self._GAPDeviceInformationEvent2,
								0x0605:	self._GAPEstablishLink,
								0x0606: self._GAPTerminateLink,
								0x051B: self._GAPHandleValueNotification,
								0x0513: self._ATTWriteResponse,
								0x0509: self._ATTReadByTypeRsponse,
								0x050B: self._ATTReadResponse,
								0x0501: self._ATTErrorResponse,


							}
		self._dongle = Dongle()
		self._timeout = timeout
		self._remoteDevices = {}
		self._connectSessions = []
		self._timeoutTime = 0
		self._isWaitingForResponse = False

	def start(self):
		"""
		Start the server, this method will get the initial communication going with the USB dongle.
		"""
		self._initDone = False
		# read until empty
		self._device.read()
		self._device.write(self._GAPDeviceInit())
		self.waitForResponse()
		if self._dongle.ready:
			self._initDone = True
		return self._initDone
		
		
	def discover(self):
		"""
		Discover the available devices, this method will return a list of devices and their addresses that are available.
		"""
		self._remoteDevices = {}
		self._device.write(self._GAPDeviceDiscoveryRequest())
		self.waitForResponse()
		
		return self._remoteDevices.values()
		
	def connect(self, macAddress):
		"""
		Connect to a known device with the addrenn macAddrress.
		If successfull return a ConnectionSession object of the connected device.
		"""
		self._device.write(self._GAPEstablishLinkRequest(macAddress))
		self.waitForResponse()
		link = self._findConnectSessionFromAddress(macAddress)
		if link:
			return link
		return False
		
	def disconnect(self, macAddress = ''):
		"""
		Disconect from a device with the address of macAddress
		"""
		for connectSession in self._connectSessions:
			if connectSession.address == macAddress or macAddress == '':
				self._device.write(self._GAPTermiateLinkRequest(connectSession.connectionHandle))
		self.waitForResponse()
		
		
	def waitForResponse(self, timeout = None):
		"""
		Wait for a response from the remote device. You may need to call this method if you wish to receive 
		notifications from the remote device.
		This method returns when another process/stream calls the responseFinished method or the timeout has occured.
		"""
		if timeout == None:
			timeout = self._timeout
		self._timeoutTime = time.time() + timeout
		self._isWaitingForResponse = True
		while (self._timeoutTime > time.time() or timeout == 0) and self._isWaitingForResponse:
			hciPacketType = self._device.read(1)
			if hciPacketType:
				hciPacketType = struct.unpack('<B', hciPacketType)[0]
				if hciPacketType == 0x04:
					self.readEvent()
				else:
					logging.warn("Unknown packed data %d" % hciPacketType)
		
		
	def responseFinished(self):
		"""
		Method to close the waitForResponse loop. This method is called when the server knows that the 
		message process has come to an end.
		"""
		self._timeoutTime = time.time() + 0.5
		self._isWaitingForResponse = False

	def readEvent(self):
		"""
		read an actual event from the USB dongle
		This methods decodes the data stream for the event byte and then passes the rest of the data onto the 
		internal method _processCommand.
		"""
		eventCode = self._readStruct('<B')[0]
		if eventCode != 0xFF:
			logging.warn("Error in event code received %02X" % eventCode)
			return
		(dataLength, command) = self._readStruct('<BH')
		dataLength -= 2
		logging.debug("Received Command: %04X DataLength: %d bytes" % (command, dataLength))
		self._processCommand(command, dataLength)
				
				
	def sendATTDiscCharsByHandle(self, connectionHandle, handle, timeout = None):	
		"""
		Send out an ATT Discovery chars by handle.
		connectionHandle value returned after a successfull connection.
		handle		Handle number to use to read data from.
		timeout		Optional timeout to wait for data.
		
		This method sends out the request and then waits for a reply uisng the method waitForResponse.
		"""
		self._device.write(self._ATTDiscCharsByHandle(connectionHandle, handle))
		self.waitForResponse(timeout)

			
	def sendATTDiscCharsByUUID(self, connectionHandle, uuid, timeout = None):
		"""
		Send out an ATT Discovery chars by UUID.
		connectionHandle value returned after a successfull connection.
		uuid		UID number to use to read data from.
		timeout		Optional timeout to wait for data.

		This method sends out the request and then waits for a reply uisng the method waitForResponse.
		"""
		self._device.write(self._ATTDiscCharsByUUID(connectionHandle, uuid))
		self.waitForResponse(timeout)

	def sendATTWriteRequest(self, connectionHandle, handle, data, timeout = None):
		"""
		Send out an ATT write request
		connectionHandle value returned after a successfull connection.
		handle		Handle number to use to read data from.
		data		ByteArray to send out to the remote device
		timeout		Optional timeout to wait for confirmation of write.
		
		"""
		self._device.write(self._ATTWriteRequest(connectionHandle, handle, data))
		self.waitForResponse(timeout)
		
		
	def addConnectSession(self, address, connectionHandle):
		"""
		Add a new ConnectionSession, this is usually called by the Connect method after a successfull connection has been
		made.
		You can also call this to create a dummy connection, to try to talk to a device that has already been connected.
		
		"""
		connectSession = ConnectSession(self)
		connectSession.address = address
		connectSession.connectionHandle = connectionHandle
		self._connectSessions.append(connectSession)
		return connectSession
		
	def removeConnectSession(self, connectSession):
		"""
		Remove a ConnectionSession from the lists of connections.
		"""
		self._connectSessions.remove(connectSession)


	def _processCommand(self, command, dataLength):
		if command in self._commandTable:
			methodCall = self._commandTable[command]
			methodCall(dataLength)
		else:
			logging.warn("Cannot find command %04X" % command)
			data = self._readStruct("<%ds" % dataLength)
			logging.warn(data)
		
	def _readStruct(self, packDef):
		data = self._device.read(struct.calcsize(packDef))
		return struct.unpack(packDef, data)
		

	def _binToMACAddress(self, data):
		result = []
		for i in range(len(data)):
			result.append("%02X" % data[i])
		return ":".join(result)

	def _macAddressToBin(self, text):
		values = text.split(":")
		data = binascii.a2b_hex(''.join(values))
		#for value in values:
			#data += struct.pack('<B', value.encode("hex"))
		return data
		

	def _buildCommand(self, code, data):
		return struct.pack('<BHB', 0x01, code, len(data)) + data
		
		
	def _findConnectSessionFromAddress(self, address):
		for connectSession in self._connectSessions:
			if connectSession.address == address:
				return connectSession
		return None		
		
	def _findConnectSessionFromConnectionHandle(self, connectionHandle):
		for connectSession in self._connectSessions:
			if connectSession.connectionHandle == connectionHandle:
				return connectSession
		return None		


	def _GAPDeviceInit(self):
		
		return self._buildCommand(
									0xFE00, 													#GAP_DeviceInit
									struct.pack('<BB16s16sL',8, 3, b"\x00", b"\x00", 1) 		#ProfileRole, MaxScanRsp, IRK, CSRK, SignCounter
								)

	def _GAPDeviceDiscoveryRequest(self):
		logging.debug("Sending GAP_DeviceDiscoveryRequest")
		return self._buildCommand(0xFE04,										# 0xFE GAP_DeviceDiscoveryRequest
									struct.pack('<BBB', 0x03, 0x01, 0x00)		#Mode (all), Enable Name Mode, Disable Whitelist
								)
		

	def _GAPEstablishLinkRequest(self, macAddress):
		logging.debug("Sending GAP_Establishlinkrequest")
		return self._buildCommand(0xFE09,																		#0xfe09 GAP_Establishlinkrequest
								struct.pack('<BBB8s', 0x00, 0x00, 0x00, self._macAddressToBin(macAddress))		# High Duty Cycle, WhiteList, Address Type Peer, Address
								)

	def _GAPTermiateLinkRequest(self, connectionHandle):
		logging.debug("Sending GAP_TerminateLinkRequest")
		return self._buildCommand(0xFE0A,												#0xFE0A GAP_TerminateLinkRequest
								struct.pack('<H', connectionHandle)						# handle
								)



	def _ATTWriteRequest(self, connectionHandle, attributeAddress, attributeData):
		logging.debug("Sending ATT_WriteReq '%04X<=%s'", attributeAddress, attributeData)
		return self._buildCommand(0xFD12,																					#0xFD12 ATT_WriteReq
									struct.pack('<HBBH', connectionHandle, 0x00, 0x00, attributeAddress) + attributeData					# handle, signature off, command off, attribute address,  + attribute data
									)


	def _ATTDiscCharsByUUID(self, connectionHandle, UUID):
		logging.debug("Sending GATT_DiscoveryCharsByUUID '%04X'" % UUID)
		return self._buildCommand(0xFD88,																	#0xFD88 GATT_DiscoveryCharsByUUID
									struct.pack('<HHHH', connectionHandle, 0x01, 0xFFFF, UUID)				# handle, start handle, end handle, UUID
									)
		

	def _ATTDiscCharsByHandle(self, connectionHandle, handle):
		logging.debug("Sending GATT_DisccoveryCharsByHandle '%04X'" % handle)
		return self._buildCommand(0xFD8A,																	#0xFD8A GATT_DiscoveryCharsByHandle
									struct.pack('<HH', connectionHandle, handle)							# connectionHandle, handle
									)
		
	def _GAPExtCommandStatus(self, dataLength):
		
		(status, param) = self._readStruct('<BH')
		
		if status == 0:
			if param == 0xFE00:				# GAP_deviceINIT 
				logging.debug("Dongle recieved GAP_deviveInit command")
			elif param == 0xFE04: 			# GAP Device Discovery Request
				logging.debug("Dongle recieved command and is now searching")
			elif param == 0xFE09: 			# GAP Establish link request
				logging.debug("Dongle recieved estalblish link request")
			elif param == 0xFE0A: 			# GAP terminate linkrequest
				logging.debug("Dongle recieved link term request")
			elif param == 0xFD88: 			# (GATT_DiscCharsByUUID)
				logging.debug("Remote device is searching")
			elif param == 0xFD12: 			# (ATT_WriteReq)
				logging.debug( "Remote device got WriteRequest")
			elif param == 0xFD8A: 			# (ATT_ReadReq)
				logging.debug( "Remote device got ReadRequest")
			else:
				logging.warn("Unknown sub command %04X" % param)

		# read any left over data
		dataLength -= struct.calcsize('<BH')
		if dataLength > 0:
			self._device.read(dataLength)

	def _GAPDeviceInitDone(self, dataLength):
		logging.debug("Device init done")
		
		(status, deviceAddress, dataPacketLength, numberDataPackets, irk, csrk) = self._readStruct('<B6sHB16s16s')
		if status == 0: 
			logging.debug("Device initialized and ready")
			self._dongle.address = self._binToMACAddress(deviceAddress)
			self._dongle.irk = irk
			self._dongle.csrk = csrk
			self._dongle.ready = True
		else:
			logging.warn("Device init failed")
		self.responseFinished()

	def _GAPDeviceInformationEvent(self, dataLength):
		(status, eventType, addressType, address, RSSI, dataLength, data) = self._readStruct('<BBB6sBB')
		print (status, eventType, addressType, address, RSSI, dataLength, data)


		remoteInfo = self._readStruct('<BBBBB')
		print(remoteInfo)
		
	def _GAPDeviceInformationEvent2(self, dataLength):
		(status, eventType, addressType, address, rssi, dataLength) = self._readStruct('<BBB6sBB')
		if status == 0:
			#print (status, eventType, addressType, rssi, dataLength)
			data = self._device.read(dataLength)

			deviceAddress = self._binToMACAddress(address)
			logging.debug("Discovered device %s" % deviceAddress)
			if not deviceAddress in self._remoteDevices:
				remoteDevice = RemoteDevice()
				remoteDevice.address = deviceAddress
				self._remoteDevices[deviceAddress] = remoteDevice
			else:
				remoteDevice = self._remoteDevices[deviceAddress]
				
			remoteDevice.signalStrength = rssi * 100/255.0
			if eventType == 0x04:
				remoteDevice.name = data[2:(data[0] - 1) + 2].decode('utf-8')
				logging.debug("Discovered device named %s" % remoteDevice.name)
			else:
				remoteDevice.data = data

	def _GAPHandleValueNotification(self, dataLength):
		(data, connectionHandle, idTag) = self._readStruct('<BHB')
		#logging.debug("Receive HandleValue notification from connhandle: %02X %04X %02X" % ( data, handleValue, idTag))
		
		(attribute, value) = self._readStruct("<H%ds" % (dataLength - 6))
		logging.debug("Attribute: %04X = %s " % (attribute, value))
		
		connectSession = self._findConnectSessionFromConnectionHandle(connectionHandle)
		if connectSession:
			if connectSession.OnEventNotification:
				connectSession.OnEventNotification(attribute, value)
		

	def _GAPDeviceDiscoveryDone(self, dataLength):
		(status, deviceCount) = self._readStruct('<BB')
		if status == 0:
			if deviceCount > 0:
				for i in range(deviceCount):					
					(eventType, addressType, address) = self._readStruct('<BB6s')
					print (eventType, addressType, address)
			else:
				logging.warn("No devices found")
			
		else:
			logging.error("Connot complete discovery")
		self.responseFinished()
		

	def _GAPEstablishLink(self, dataLength):
		(status, addressType, address, handle, interval, latency, timeout, clockAccuracy) = self._readStruct('<BB6sHHHHB')
		if status == 0:
			connectSession = self.addConnectSession(self._binToMACAddress(address), handle)
			connectSession.addressType = addressType
			connectSession.interval = interval
			connectSession.latency = latency
			connectSession.timeout = timeout
			connectSession.clockAccuracy = clockAccuracy
			logging.debug("Established link to device '%s'" % connectSession.address) 
			self.responseFinished()

	def _GAPTerminateLink(self, dataLength):
		(status, handle, reason) = self._readStruct('<BHB')
		if status == 0:
			connectSession = self._findConnectSessionFromConnectionHandle(handle)
			if connectSession:
				connectSession.connectionHandle = None
				logging.debug("Device '%s' has terminated link" % connectSession.address)
			else:
				logging.error("Cannot find link device in list to terminate")
			self.responseFinished()
		else:
			logging.error("Cannot terminate link")



	def _ATTWriteResponse(self, dataLength):
		(status, handle, reason) = self._readStruct('<BHB')
		if status == 0:
			logging.debug("Write compled")
		self.responseFinished()


	def _ATTReadByTypeRsponse(self, dataLength):
		status = self._readStruct('<B')[0]
		if status == 0x1A:
			# response list has finished
			logging.debug("ATT Read 0x1A")
			data = self._readStruct('<BBB')			
			self.responseFinished()
		elif status == 0x00:
			(connectionHandle, pduLength, dataPacketLength, dataHandle) = self._readStruct('<HBBH')
			
			#print (connectionHandle, pduLength, dataPacketLength, dataHandle, dataLength)
			dataPacketLength -= 2
			data = self._readStruct("<%ds" % dataPacketLength)[0]
			
			connectSession = self._findConnectSessionFromConnectionHandle(connectionHandle)
			if connectSession:
				connectSession.responseList.append(data)
			
		else:
			logging.warn("Error status '%02X' returned from ATTReadyByTypeResponse")
			self.responseFinished()
		

	def _ATTReadResponse(self, dataLength):
		status = self._readStruct('<B')[0]
		if status == 0x00:
			(connectionHandle, pduLength) = self._readStruct('<HB')
			data = self._readStruct("<%dB" % pduLength)
			logging.debug("Read Response value: %s", data)
			connectSession = self._findConnectSessionFromConnectionHandle(connectionHandle)
			if connectSession:
				connectSession.responseValue = data

		self.responseFinished()

	def _ATTErrorResponse(self, dataLength):
		(status, connectionHandle, pduLength, requestOpcode, handle, errorCode) = self._readStruct('<BHBBHB')
		logging.debug("Error Response %02X" % errorCode)
		connectSession = self._findConnectSessionFromConnectionHandle(connectionHandle)
		if connectSession:
			connectSession.errorCode = errorCode
		
		self.responseFinished()
