#!/usr/bin/env python3
# -*- coding: UTF-8 -*-
import spidev
import time

# Internal helper parsing functions.
# These handle input that might be none or null and return none instead of
# throwing errors.
def _parse_degrees(nmea_data):
	# Parse a NMEA lat/long data pair 'dddmm.mmmm' into a pure degrees value.
	# Where ddd is the degrees, mm.mmmm is the minutes.
	if nmea_data is None or len(nmea_data) < 3:
		return None
	raw = float(nmea_data)
	deg = raw // 100
	minutes = raw % 100
	return deg + minutes / 60


def _parse_int(nmea_data):
	if nmea_data is None or nmea_data == "":
		return None
	return int(nmea_data)


def _parse_float(nmea_data):
	if nmea_data is None or nmea_data == "":
		return None
	return float(nmea_data)


def _parse_str(nmea_data):
	if nmea_data is None or nmea_data == "":
		return None
	return str(nmea_data)
	

"""def readSentence(spiport):
	s = ""
	buf = ""
	isstart = False
	while True:
		c = chr(spiport.readbytes(1)[0])
		if isstart == False :
			if c == '$' :
				isstart = True
				buf += c
		else :
			if c !='\n' :
				buf += c
			else :
				buf += c
				time.sleep(0.5)
				isstart = False
				break
	return buf
"""

class GPS:
	"""GPS parsing module.	Can parse simple NMEA data sentences from SPI
	GPS modules to read latitude, longitude, and more.
	"""
	def __init__(self,spiport):
		self.spiport = spiport
		# Initialize null starting values for GPS attributes.
		self.datatype = None
		self.timestamp_utc = None
		self.latitude = None
		self.longitude = None
		self.fix_quality = None
		self.fix_quality_3d = None
		self.satellites = None
		self.satellites_prev = None
		self.horizontal_dilution = None
		self.altitude_m = None
		self.height_geoid = None
		self.speed_knots = None
		self.track_angle_deg = None
		self.sats = None
		self.isactivedata = None
		self.true_track = None
		self.mag_track = None
		self.sat_prns = None
		self.sel_mode = None
		self.pdop = None
		self.hdop = None
		self.vdop = None
		self.total_mess_num = None
		self.mess_num = None
		self._raw_sentence = None
		self.debug = None

	def readSentence(self,spiport):
		s = ""
		buf = ""
		isstart = False
		while True:
			#print('test:------------')
			#print(type(spiport.readbytes(1)[0]))
			#print(type(spiport.readbytes(1)))
			c = chr(spiport.readbytes(1)[0])
			if isstart == False :
				if c == '$' :
					isstart = True
					buf += c
			else :
				if c !='\n' :
					buf += c
				else :
					buf += c
					isstart = False
					break
		return buf
	
	def send_command(self, command, add_checksum=True):
		"""Send a command string to the GPS.  If add_checksum is True (the
		default) a NMEA checksum will automatically be computed and added.
		Note you should NOT add the leading $ and trailing * to the command
		as they will automatically be added!
		"""
		self.write(b"$")
		self.write(command)
		if add_checksum:
			checksum = 0
			for char in command:
				checksum ^= char
			self.write(b"*")
			self.write(bytes("{:02x}".format(checksum).upper(), "ascii"))
		self.write(b"\r\n")
	
	def write(self, bytestr):
		"""Write a bytestring data to the GPS directly, without parsing
		or checksums"""
		return self.spiport.writebytes(bytestr)
	
	def update(self):
		"""Check for updated data from the GPS module and process it
		accordingly.  Returns True if new data was processed, and False if
		nothing new was received.
		"""
		# Grab a sentence and check its data type to call the appropriate
		# parsing function.
		try:
			sentence = self.readSentence(self.spiport)
		except UnicodeError:
			return None
			
		if sentence is None:
			return False
		
		self.datatype = sentence.split(',')[0]
		#print(type(self.datatype))
		#self.datatype[2:3] = '*'
		#print(self.datatype)
		if self.datatype == "$GPGGA" or self.datatype == "$GNGGA":
			self.parse_gpgga(sentence)
		#elif self.datatype == "$GPRMC" or self.datatype == "$GNRMC":
			#self.parse_gprmc(sentence)
		return True
		
	
	def parse_gpgga(self,sentence):
		data = sentence.split(",")
		if data is None or len(data) != 15 or (data[0] == ""):
			return	# Unexpected number of params.
		# Parse fix time.
		time_utc = int(_parse_float(data[1]))
		if time_utc is not None:
			hours = time_utc // 10000 + 8
			mins = (time_utc // 100) % 100
			secs = time_utc % 100
			# Set or update time to a friendly python time struct.
			if self.timestamp_utc is not None:
				self.timestamp_utc = time.struct_time(
					(
						self.timestamp_utc.tm_year,
						self.timestamp_utc.tm_mon,
						self.timestamp_utc.tm_mday,
						hours,
						mins,
						secs,
						0,
						0,
						-1,
					)
				)
			else:
				self.timestamp_utc = time.struct_time(
					(0, 0, 0, hours, mins, secs, 0, 0, -1)
				)
		# Parse latitude and longitude.
		self.latitude = _parse_degrees(data[2])
		if self.latitude is not None and data[3] is not None and data[3].lower() == "s":
			self.latitude *= -1.0
		self.longitude = _parse_degrees(data[4])
		if (
			self.longitude is not None
			and data[5] is not None
			and data[5].lower() == "w"
		):
			self.longitude *= -1.0
		# Parse out fix quality and other simple numeric values.
		# print('check',data[6],'int',int(data[6]))
		self.fix_quality = _parse_int(data[6])
		self.satellites = _parse_int(data[7])
		self.horizontal_dilution = _parse_float(data[8])
		self.altitude_m = _parse_float(data[9])
		self.height_geoid = _parse_float(data[11])
		return 0

	def parse_gprmc(self,sentence):
		# Parse the arguments (everything after data type) for NMEA GPRMC
		# minimum location fix sentence.
		data = sentence.split(",")
		if data is None or len(data) < 11 or data[0] is None or (data[0] == ""):
			return	# Unexpected number of params.
		# Parse fix time.
		time_utc = int(_parse_float(data[1]))
		if time_utc is not None:
			hours = time_utc // 10000 + 8
			mins = (time_utc // 100) % 100
			secs = time_utc % 100
			# Set or update time to a friendly python time struct.
			if self.timestamp_utc is not None:
				self.timestamp_utc = time.struct_time(
					(
						self.timestamp_utc.tm_year,
						self.timestamp_utc.tm_mon,
						self.timestamp_utc.tm_mday,
						hours,
						mins,
						secs,
						0,
						0,
						-1,
					)
				)
			else:
				self.timestamp_utc = time.struct_time(
					(0, 0, 0, hours, mins, secs, 0, 0, -1)
				)
		# Parse status (active/fixed or void).
		status = data[2]
		self.fix_quality = 0
		if status is not None and status.lower() == "a":
			self.fix_quality = 1
		# Parse latitude and longitude.
		self.latitude = _parse_degrees(data[3])
		if self.latitude is not None and data[4] is not None and data[4].lower() == "s":
			self.latitude *= -1.0
		self.longitude = _parse_degrees(data[5])
		if (
			self.longitude is not None
			and data[6] is not None
			and data[6].lower() == "w"
		):
			self.longitude *= -1.0
		# Parse out speed and other simple numeric values.
		self.speed_knots = _parse_float(data[7])
		self.track_angle_deg = _parse_float(data[8])
		# Parse date.
		if data[8] is not None and len(data[9]) == 6:
			day = int(data[9][0:2])
			month = int(data[9][2:4])
			year = 2000 + int(data[9][4:6])	 # Y2k bug, 2 digit year assumption.
			# This is a problem with the NMEA
			# spec and not this code.
			if self.timestamp_utc is not None:
				# Replace the timestamp with an updated one.
				# (struct_time is immutable and can't be changed in place)
				self.timestamp_utc = time.struct_time(
					(
						year,
						month,
						day,
						self.timestamp_utc.tm_hour,
						self.timestamp_utc.tm_min,
						self.timestamp_utc.tm_sec,
						0,
						0,
						-1,
					)
				)
			else:
				# Time hasn't been set so create it.
				self.timestamp_utc = time.struct_time(
					(year, month, day, 0, 0, 0, 0, 0, -1)
				)
		
#r start to run
spiport =spidev.SpiDev()
spiport.open(2,0)#（2,0）for spi1，（0,0）for spi0
spiport.max_speed_hz=1000000 # 7800000,15600000,62400000....
i = 0
gps = GPS(spiport)
#data = b'\xB5\x62\x06\x01\x08\x00\xF0\x04\x01\x01\x00\x01\x00\x00\06\x4D'
#print(b'0000',binascii.b2a_hex(data))
#spiport.
#gps.send_command(b"PMTK220,1000")
print('test-----------')
#data1 = [0x0000,0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x04,0x01,0x01,0x00,0x01,0x01,0x00,0x07,0x4F]
#rate = [0x0000,0xB5,0x62,0x06,0x08,0x06,0x00,0x64,0x00,0x05,0x00,0x01,0x00,0x7E,0x22]
#print(type(data))
#print(chr(data[1]))
#spiport.writebytes(data1)
#spiport.writebytes(rate)

last_print = time.monotonic()
while True:
	gps.update()
	
	current = time.monotonic()
	
	if current - last_print >= 1.0:
		last_print = current
		print(i)
		if gps.timestamp_utc is not None:
			print(
					"Fix timestamp: {}/{}/{} {:02}:{:02}:{:02}".format(
						gps.timestamp_utc.tm_mon,	# Grab parts of the time from the
						gps.timestamp_utc.tm_mday,	# struct_time object that holds
						gps.timestamp_utc.tm_year,	# the fix time.	 Note you might
						gps.timestamp_utc.tm_hour,	# not get all data like year, day,
						gps.timestamp_utc.tm_min,	# month!
						gps.timestamp_utc.tm_sec,
					)
				)
		if gps.latitude is not None:
			print("Latitude: {0:.6f} degrees".format(gps.latitude))
		if gps.longitude is not None:
			print("Longitude: {0:.6f} degrees".format(gps.longitude))
		if gps.fix_quality is not None:
			print("Fix quality: {}".format(gps.fix_quality))
		# Some attributes beyond latitude, longitude and timestamp are optional
		# and might not be present.	 Check if they're None before trying to use!
		if gps.satellites is not None:
			print("# satellites: {}".format(gps.satellites))
		if gps.altitude_m is not None:
			print("Altitude: {} meters".format(gps.altitude_m))
		if gps.speed_knots is not None:
			print("Speed: {} knots".format(gps.speed_knots))
		if gps.track_angle_deg is not None:
			print("Track angle: {} degrees".format(gps.track_angle_deg))
		if gps.horizontal_dilution is not None:
			print("Horizontal dilution: {}".format(gps.horizontal_dilution))
		if gps.height_geoid is not None:
			print("Height geo ID: {} meters".format(gps.height_geoid))
		i+=1


