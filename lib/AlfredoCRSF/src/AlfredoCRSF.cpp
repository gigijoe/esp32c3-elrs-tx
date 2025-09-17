#include <AlfredoCRSF.h>

AlfredoCRSF::AlfredoCRSF() :
	_crc(0xd5),
	_lastReceive(0), _lastChannelsPacket(0), _linkIsUp(false), _updateInterval(0), _correction(0), _device_address(0), _device_name("Unknown"), _currentParamNumber(0), _currentFieldChunk(0), _chunkRemaining(0), _paramDataSize(0), _isParamReading(false), _isParamReadingDone(false), 
	m_packetRateIndex(0), m_maxPowerIndex(0), m_telemRatioIndex(0), m_switchModeIndex(0), m_modelMatchEnabled(false), m_bindState(0)
{
	 
}

void AlfredoCRSF::begin(Stream & port)
{
  this->_port = &port;
}

// Call from main loop to update
void AlfredoCRSF::update()
{
	handleSerialIn();
}

void AlfredoCRSF::handleSerialIn()
{
	while (_port->available())
	{
		uint8_t b = _port->read();
		_lastReceive = millis();

		_rxBuf[_rxBufPos++] = b;
		handleByteReceived();

		if (_rxBufPos == (sizeof(_rxBuf)/sizeof(_rxBuf[0])))
		{
			// Packet buffer filled and no valid packet found, dump the whole thing
			_rxBufPos = 0;
		}
	}

	//checkPacketTimeout();
	checkLinkDown();
}

void AlfredoCRSF::handleByteReceived()
{
	bool reprocess;
	do
	{
		reprocess = false;
		if (_rxBufPos > 1)
		{
			uint8_t len = _rxBuf[1];
			// Sanity check the declared length, can't be shorter than Type, X, CRC
			if (len < 3 || len > CRSF_MAX_PACKET_LEN)
			{
				shiftRxBuffer(1);
				reprocess = true;
			}

			else if (_rxBufPos >= (len + 2))
			{
				uint8_t inCrc = _rxBuf[2 + len - 1];
				uint8_t crc = _crc.calc(&_rxBuf[2], len - 1);
				if (crc == inCrc)
				{
					processPacketIn(len);
					shiftRxBuffer(len + 2);
					reprocess = true;
				}
				else
				{
Serial.printf("CRC error...\r\n");
					shiftRxBuffer(1);
					reprocess = true;
				}
			}  // if complete packet
		} // if pos > 1
	} while (reprocess);
}

void AlfredoCRSF::checkPacketTimeout()
{
	// If we haven't received data in a long time, flush the buffer a byte at a time (to trigger shiftyByte)
	if (_rxBufPos > 0 && millis() - _lastReceive > CRSF_PACKET_TIMEOUT_MS)
		while (_rxBufPos)
			shiftRxBuffer(1);
}

void AlfredoCRSF::checkLinkDown()
{
	if (_linkIsUp && millis() - _lastChannelsPacket > CRSF_FAILSAFE_STAGE1_MS)
	{
		_linkIsUp = false;
	}
}

void AlfredoCRSF::processPacketIn(uint8_t len)
{
	const crsf_header_t *hdr = (crsf_header_t *)_rxBuf;
	const crsf_ext_header_t *extHdr = (crsf_ext_header_t *)_rxBuf;

	//if (hdr->device_addr == CRSF_ADDRESS_FLIGHT_CONTROLLER)
	{
		switch (hdr->type)
		{
		case CRSF_FRAMETYPE_GPS: // 0x02
			packetGps(hdr);
			break;
		case CRSF_FRAMETYPE_BATTERY_SENSOR:
			packetBatterySensor(hdr);
			break;
		case CRSF_FRAMETYPE_RC_CHANNELS_PACKED: // 0x16
			packetChannelsPacked(hdr);
			break;
		case CRSF_FRAMETYPE_LINK_STATISTICS: // 0x14
			packetLinkStatistics(hdr);
			break;
		case CRSF_FRAMETYPE_BARO_ALTITUDE: // 0x09
			packetBaroAltitude(hdr);
			break;
		case CRSF_FRAMETYPE_VARIO: // 0x07
			packetVario(hdr);
			break;
// Used by extended header frames (type in range 0x28 to 0x96)    
		case CRSF_FRAMETYPE_DEVICE_INFO: // 0x29
			packetDeviceInfo(extHdr); 
			break;
		case CRSF_FRAMETYPE_PARAMETER_SETTINGS_ENTRY:
#if 0
			Serial.printf("Device Address 0x%02x, Frame Type 0x%02x\r\n", hdr->device_addr, hdr->type);
			for(int i=0;i<hdr->frame_size;i++) {
				Serial.printf("0x%02x ", _rxBuf[i+2]);
			}
			Serial.printf("\r\n");
#endif
			packetParameterSettingsEntry(extHdr);
			break;
		case CRSF_FRAMETYPE_RADIO_ID: // 0x3A
			packetRadioId(extHdr);
			break;
		default:
			Serial.printf("Device Address 0x%02x, Frame Type 0x%02x\r\n", hdr->device_addr, hdr->type);
			for(int i=0;i<hdr->frame_size;i++) {
				Serial.printf("0x%02x ", _rxBuf[i+2]);
			}
			Serial.printf("\r\n");
			break;
		}
	}
}

// Shift the bytes in the RxBuf down by cnt bytes
void AlfredoCRSF::shiftRxBuffer(uint8_t cnt)
{
	// If removing the whole thing, just set pos to 0
	if (cnt >= _rxBufPos)
	{
		_rxBufPos = 0;
		return;
	}

	// Otherwise do the slow shift down
	uint8_t *src = &_rxBuf[cnt];
	uint8_t *dst = &_rxBuf[0];
	_rxBufPos -= cnt;
	uint8_t left = _rxBufPos;
	while (left--)
		*dst++ = *src++;
}

void AlfredoCRSF::packetChannelsPacked(const crsf_header_t *p)
{
	crsf_channels_t *ch = (crsf_channels_t *)&p->data;
	_channels[0] = ch->ch0;
	_channels[1] = ch->ch1;
	_channels[2] = ch->ch2;
	_channels[3] = ch->ch3;
	_channels[4] = ch->ch4;
	_channels[5] = ch->ch5;
	_channels[6] = ch->ch6;
	_channels[7] = ch->ch7;
	_channels[8] = ch->ch8;
	_channels[9] = ch->ch9;
	_channels[10] = ch->ch10;
	_channels[11] = ch->ch11;
	_channels[12] = ch->ch12;
	_channels[13] = ch->ch13;
	_channels[14] = ch->ch14;
	_channels[15] = ch->ch15;

	for (unsigned int i=0; i<CRSF_NUM_CHANNELS; ++i)
		_channels[i] = map(_channels[i], CRSF_CHANNEL_VALUE_1000, CRSF_CHANNEL_VALUE_2000, 1000, 2000);

	_linkIsUp = true;
	_lastChannelsPacket = millis();

	memcpy(&_channelsPacked, ch, sizeof(_channelsPacked));
}

void AlfredoCRSF::packetBatterySensor(const crsf_header_t *p)
{
	const crsf_sensor_battery_t *bat = (crsf_sensor_battery_t *)p->data;
	memcpy(&_batterySensor, bat, sizeof(_batterySensor));
	
	Serial.printf("[ Battery Sensor ]\r\n");
	Serial.printf("\tVoltage : %.1f V\r\n", be16toh(bat->voltage) / 10.0f);
	Serial.printf("\tCurrent : %.1f A\r\n", be16toh(bat->current) / 10.0f);
	uint32_t c = bat->capacity;
	Serial.printf("\tcapacity : %u mAh\r\n", be32toh(c));
	Serial.printf("\tremaining : %u %\r\n", bat->remaining);
}

/*
 * 0x14 Link statistics
 * Payload:
 *
 * uint8_t Uplink RSSI Ant. 1 ( dBm * -1 )
 * uint8_t Uplink RSSI Ant. 2 ( dBm * -1 )
 * uint8_t Uplink Package success rate / Link quality ( % )
 * int8_t Uplink SNR ( db )
 * uint8_t Diversity active antenna ( enum ant. 1 = 0, ant. 2 )
 * uint8_t RF Mode ( enum 4fps = 0 , 50fps, 150hz)
 * uint8_t Uplink TX Power ( enum 0mW = 0, 10mW, 25 mW, 100 mW, 500 mW, 1000 mW, 2000mW )
 * uint8_t Downlink RSSI ( dBm * -1 )
 * uint8_t Downlink package success rate / Link quality ( % )
 * int8_t Downlink SNR ( db )
 * Uplink is the connection from the ground to the UAV and downlink the opposite direction.
 */

void AlfredoCRSF::packetLinkStatistics(const crsf_header_t *p)
{
	const crsfLinkStatistics_t *link = (crsfLinkStatistics_t *)p->data;
	memcpy(&_linkStatistics, link, sizeof(_linkStatistics));
	
	Serial.printf("[ Link Statistics ]\r\n");
	Serial.printf("\tUp Link ->\r\n");
	Serial.printf("\tRSSI 1 : %u\r\n", link->uplink_RSSI_1);
	Serial.printf("\tRSSI 2 : %u\r\n", link->uplink_RSSI_2);
	Serial.printf("\tLink Quality : %u\r\n", link->uplink_Link_quality);
	Serial.printf("\tSNR : %d\r\n", link->uplink_SNR);
	Serial.printf("\tActive Antenna : %u\r\n", link->active_antenna);
	Serial.printf("\tRF Mode : %u\r\n", link->rf_Mode);
	Serial.printf("\tTX Power : %u\r\n", link->uplink_TX_Power);
	Serial.printf("\tDown Link ->\r\n");
	Serial.printf("\tRSSI : %u\r\n", link->downlink_RSSI);
	Serial.printf("\tLink Quality : %u\r\n", link->downlink_Link_quality);
	Serial.printf("\tSNR : %d\r\n", link->downlink_SNR);
}

void AlfredoCRSF::packetGps(const crsf_header_t *p)
{
	const crsf_sensor_gps_t *gps = (crsf_sensor_gps_t *)p->data;
	_gpsSensor.latitude = be32toh(gps->latitude);
	_gpsSensor.longitude = be32toh(gps->longitude);
	_gpsSensor.groundspeed = be16toh(gps->groundspeed);
	_gpsSensor.heading = be16toh(gps->heading);
	_gpsSensor.altitude = be16toh(gps->altitude);
	_gpsSensor.satellites = gps->satellites;
}

void AlfredoCRSF::packetVario(const crsf_header_t *p)
{
	const crsf_sensor_vario_t *vario = (crsf_sensor_vario_t *)p->data;
	_varioSensor.verticalspd = be16toh(vario->verticalspd);
}

void AlfredoCRSF::packetBaroAltitude(const crsf_header_t *p)
{
	const crsf_sensor_baro_altitude_t *baroAltitude = (crsf_sensor_baro_altitude_t *)p->data;
	_baroAltitudeSensor.altitude = be16toh(baroAltitude->altitude);
	_baroAltitudeSensor.verticalspd = be16toh(baroAltitude->verticalspd);
}

void AlfredoCRSF::packetAttitude(const crsf_header_t *p)
{
	const crsf_sensor_attitude_t *attitude = (crsf_sensor_attitude_t *)p->data;
	_attitudeSensor.pitch = be16toh(attitude->pitch);
	_attitudeSensor.roll = be16toh(attitude->roll);
	_attitudeSensor.yaw = be16toh(attitude->yaw);
}

void AlfredoCRSF::packetDeviceInfo(const crsf_ext_header_t *p)
{
	const uint8_t *data = (const uint8_t *)p->data;
	_device_address = p->orig_addr;
	
	strlcpy(_device_name, (const char *)&p->data[0], CRSF_MAX_NAME_LEN);
	int i = strlen((const char *)_device_name) + 1; // name + '\0'
	
	deviceInformationPacket_t *deviceInfo = (deviceInformationPacket_t *)&p->data[i];
	
	_deviceInfo.serialNo = be32toh(deviceInfo->serialNo); // ['E', 'L', 'R', 'S'], seen [0x00, 0x0a, 0xe7, 0xc6] // "Serial 177-714694" (value is 714694)
	_deviceInfo.hardwareVer = be32toh(deviceInfo->hardwareVer); // unused currently by us, seen [ 0x00, 0x0b, 0x10, 0x01 ] // "Hardware: V 1.01" / "Bootloader: V 3.06"
	_deviceInfo.softwareVer = be32toh(deviceInfo->softwareVer); // seen [ 0x00, 0x00, 0x05, 0x0f ] // "Firmware: V 5.15"
	_deviceInfo.fieldCnt = deviceInfo->fieldCnt;
	_deviceInfo.parameterVersion = deviceInfo->parameterVersion;
	
	Serial.printf("[ Device Information ]\r\n");
	Serial.printf("\tDevice Address 0x%02x\r\n", _device_address);
	Serial.printf("\tDevice Name %s\r\n", _device_name);
	Serial.printf("\tSerial No 0x%08x\r\n", _deviceInfo.serialNo);
	Serial.printf("\tSerial No %c%c%c%c\r\n", _deviceInfo.serialNo >> 24, (_deviceInfo.serialNo >> 16) & 0xff, (_deviceInfo.serialNo >> 8) & 0xff, _deviceInfo.serialNo & 0xff);
	Serial.printf("\tHardware Version 0x%08x\r\n", _deviceInfo.hardwareVer);
	Serial.printf("\tFoftware Version 0x%08x\r\n", _deviceInfo.softwareVer);
	Serial.printf("\tField Count %d\r\n", _deviceInfo.fieldCnt);
	Serial.printf("\tParameter Version %d\r\n", _deviceInfo.parameterVersion);
}

void AlfredoCRSF::packetParameterSettingsEntry(const crsf_ext_header_t *p)
{
	const uint8_t *data = (const uint8_t *)p->data;
	uint8_t number = data[0];
	uint8_t chunkRemaining = data[1];  
	uint8_t dataSize = p->frame_size - 6; // type, dest_addr, orig_addr, number, chunk_remaining, crc
	
	if(_isParamReading == false) {
Serial.printf("Not param reading phase ...\r\n");
Serial.printf("<%u> Chunk remaining : %u\r\n", number, chunkRemaining);
		return;
	}
  
	if(_currentParamNumber != number) {
Serial.printf("Param number miss match...\r\n");
		return;
	}
  
  
	if(_chunkRemaining != 0 && 
			chunkRemaining >= _chunkRemaining) {
Serial.printf("Repeat entry ...\r\n");
		return; // Repeat entry
	} else
		_chunkRemaining = chunkRemaining;

	if(_chunkRemaining == 0xff) {
		_chunkRemaining = 0;
		_currentFieldChunk = 0;
		_isParamReading = false;
Serial.printf("Chunk remaining is 0xff ...\r\n");
		return;
	}
  
	memcpy(&_paramData[_paramDataSize], &data[2], dataSize);
	_paramDataSize += dataSize;

	if(_chunkRemaining > 0) {
		//writeParameterRead(p->device_addr, number, _currentFieldChunk + 1);
		_currentFieldChunk++;
		//delayMicroseconds(1000);
	} else {
		uint8_t parentFolder = _paramData[0];
		uint8_t dataType = _paramData[1];
		int n = 0;
		char name[CRSF_MAX_NAME_LEN];
		if(_paramData[2] == 'H' &&
			_paramData[3] == 'o' &&
			_paramData[4] == 'o' &&
			_paramData[5] == 'J') {
		  memcpy(name, &_paramData[2], 4);
		  name[4] = '\0';
		  n = 6;
		} else {
		  strlcpy(name, (const char *)&_paramData[2], CRSF_MAX_NAME_LEN);
		  n = strlen((const char *)name) + 1 + 2; // folder + type + name + '\0' 
		}

		Serial.printf("[ Parameter Read %u ]\r\n", number);
		Serial.printf("\tParent Folder : %u\r\n", parentFolder);
		Serial.printf("\tDtat Type : %u\r\n", dataType);
		Serial.printf("\tName : %s\r\n", name);

		switch(dataType) {
			case CRSF_TEXT_SELECTION: {
				char options[256] = {0};
				strlcpy(options, (const char *)&_paramData[n], 256);
				n += strlen((const char *)options) + 1; // options + '\0'
				Serial.printf("\tOptions : %s\r\n", options);
				uint8_t value = _paramData[n++];
				Serial.printf("\tValue : %u\r\n", value);
				Serial.printf("\tMin : %u\r\n", _paramData[n++]);
				Serial.printf("\tMax : %u\r\n", _paramData[n++]);
				uint8_t defaultValue = _paramData[n++];
				Serial.printf("\tDefault : %u\r\n", defaultValue);
				char unit[CRSF_MAX_NAME_LEN];
				strlcpy(unit, (const char *)&_paramData[n], CRSF_MAX_NAME_LEN);
				Serial.printf("\tUnit : %s\r\n", unit);

				if(strcmp(name, "Packet Rate") == 0) {
					const char s[] = ";";
					char *token = strtok(options, s);
					while(token != NULL) {
						//Serial.printf("\t%s\r\n", token);
						m_packetRateArray.push_back(token);
						token = strtok(NULL, s);
					}
					m_packetRateIndex = value;
				} else if(strcmp(name, "Telem Ratio") == 0) {
					const char s[] = ";";
					char *token = strtok(options, s);
					while(token != NULL) {
						//Serial.printf("\t%s\r\n", token);
						m_telemRatioArray.push_back(token);
						token = strtok(NULL, s);
					}
					m_telemRatioIndex = value;
					m_telemRatioUnit = unit;
				} else if(strcmp(name, "Switch Mode") == 0) {
					const char s[] = ";";
					char *token = strtok(options, s);
					while(token != NULL) {
						//Serial.printf("\t%s\r\n", token);
						m_switchModeArray.push_back(token);
						token = strtok(NULL, s);
					}
					m_switchModeIndex = value;
				} else if(strcmp(name, "Model Match") == 0) {
					m_modelMatchEnabled = value > 0 ? true : false;
					m_modelMatchId = unit;
				} else if(strcmp(name, "Max Power") == 0) {
					const char s[] = ";";
					char *token = strtok(options, s);
					while(token != NULL) {
						//Serial.printf("\t%s\r\n", token);
						m_maxPowerArray.push_back(token);
						token = strtok(NULL, s);
					}
					m_maxPowerIndex = value;
					m_maxPowerUnit = unit;
				}
			} break;
			case CRSF_COMMAND: {
				uint8_t state = _paramData[n++];
				Serial.printf("\tState : %u\r\n", state);
				if(strcmp(name, "Bind") == 0) {
					m_bindState = state;
				}

				char info[CRSF_MAX_NAME_LEN];
				strlcpy(info, (const char *)&_paramData[n], CRSF_MAX_NAME_LEN);
				n += strlen((const char *)info) + 1; // info + '\0'
				Serial.printf("\tInfo : %s\r\n", info);
			} break;
			case CRSF_INT8: // fallthrough
			case CRSF_UINT8:
				break;
			case CRSF_INT16: // fallthrough
			case CRSF_UINT16:
				break;
			case CRSF_STRING: // fallthough
			case CRSF_INFO: {
				char info[256];
				strlcpy(info, (const char *)&_paramData[n], 256);
				n += strlen((const char *)info) + 1; // info + '\0'
				Serial.printf("\tInfo : %s\r\n", info);				
			} break;
			case CRSF_FOLDER:
				for(int i=n;i<dataSize;++i) {
					if(_paramData[i] == 0xff)
						break;
					Serial.printf("0x%02x ", _paramData[i]);
				}
				Serial.printf("\r\n");

				if(strncmp(name, "TX Power", 8) == 0) {
					m_txPower = name;
				}
				break;
			break;
			case CRSF_FLOAT:
			case CRSF_OUT_OF_RANGE:
			default:
				break;
		}
	
		_currentFieldChunk = 0;
		_isParamReading = false;
		_isParamReadingDone = true;
		_currentParamNumber = 0;
	}
}

void AlfredoCRSF::packetRadioId(const crsf_ext_header_t *p)
{
	const uint8_t *data = (const uint8_t *)p->data;
	if(p->dest_addr == CRSF_ADDRESS_RADIO_TRANSMITTER && // 0xEA - radio address
			data[0] == CRSF_FRAMETYPE_OPENTX_SYNC) { // 0x10 - timing correction frame
		uint32_t updateInterval = be32toh(*(uint32_t *)&data[1]);
		int32_t correction = be32toh(*(int32_t *)&data[5]);
		// values are in 10th of micro-seconds
		updateInterval /= 10;
		correction /= 10;
		if (correction >= 0)
			correction %= updateInterval;
		else
			correction = -((-correction) % updateInterval);

		_updateInterval = updateInterval;
		_correction = correction; // LSB = 100ns, positive values = data came too early,
	
		//Serial.printf("Update Interval = %u, Correction = %d\r\n", updateInterval, correction);
	}
}

void AlfredoCRSF::write(const uint8_t *buf, size_t len)
{
duplex_set_TX();
	_port->write(buf, len);
	_port->flush();
duplex_set_RX();
}

void AlfredoCRSF::queuePacket(uint8_t addr, uint8_t type, const void *payload, uint8_t len)
{
	if (!_linkIsUp)
		return;
	if (len > CRSF_MAX_PACKET_LEN)
		return;
   
	uint8_t buf[CRSF_MAX_PACKET_LEN+4];
	buf[0] = addr;
	buf[1] = len + 2; // type + payload + crc
	buf[2] = type;
	if(len > 0)
		memcpy(&buf[3], payload, len);
	buf[len+3] = _crc.calc(&buf[2], len + 1);
	write(buf, len + 4);
}

void AlfredoCRSF::writePacket(uint8_t addr, uint8_t type, const void *payload, uint8_t len)
{
	uint8_t buf[CRSF_MAX_PACKET_LEN+4];
	buf[0] = addr;
	buf[1] = len + 2; // type + payload + crc
	buf[2] = type;
	if(len > 0)
		memcpy(&buf[3], payload, len);
	buf[len+3] = _crc.calc(&buf[2], len + 1);
	write(buf, len + 4);
}

void AlfredoCRSF::writeExtPacket(uint8_t addr, uint8_t type, uint8_t dest_addr, uint8_t orig_addr, const void *payload, uint8_t len)
{
	uint8_t buf[CRSF_MAX_PACKET_LEN+6];
	buf[0] = addr;
	buf[1] = len + 4; // type + dest_addr + orig_addr + payload + crc
	buf[2] = type;
	buf[3] = dest_addr;
	buf[4] = orig_addr;
	if(len > 0)
		memcpy(&buf[5], payload, len);
	buf[len+5] = _crc.calc(&buf[2], len + 3);
	write(buf, len + 6);
}

bool AlfredoCRSF::waitForRxPacket(uint32_t timeout_ms)
{
	uint32_t timeNow = millis();
	while(1) {
		if(_port->available())
			return true;
		if(millis() - timeNow >= timeout_ms)
		break;
		yield();
	}

	return false;
}

void AlfredoCRSF::writeParameterRead(uint8_t addr, uint8_t number, uint8_t chunk)
{
	_currentParamNumber = number;
	//_currentFieldChunk = chunk;

	//Serial.printf("Parameter Read %u chunk %u\r\n", number, chunk);

	if(chunk == 0) {
		_paramDataSize = 0;
		_currentFieldChunk = 0;
		_chunkRemaining = 0;
		_isParamReading = true;
		_isParamReadingDone = false;
		memset(_paramData, 0, CRSF_MAX_CHUNKS * CRSF_MAX_CHUNK_SIZE);
	}

	uint8_t packetCmd[5];
	packetCmd[0] = CRSF_FRAMETYPE_PARAMETER_READ; // 0x2D
	packetCmd[1] = CRSF_ADDRESS_CRSF_TRANSMITTER; // 0xEE
	packetCmd[2] = CRSF_ADDRESS_RADIO_TRANSMITTER; // 0xEA
	packetCmd[3] = number;
	packetCmd[4] = chunk;
	
	writeExtPacket(addr, CRSF_FRAMETYPE_PARAMETER_READ, CRSF_ADDRESS_CRSF_TRANSMITTER, CRSF_ADDRESS_RADIO_TRANSMITTER, &packetCmd[3], 2);
}

void AlfredoCRSF::writeParameterWrite(uint8_t addr, uint8_t number, uint8_t value)
{
	uint8_t packetCmd[5];
	packetCmd[0] = CRSF_FRAMETYPE_PARAMETER_WRITE; // 0x2D
	packetCmd[1] = CRSF_ADDRESS_CRSF_TRANSMITTER; // 0xEE
	packetCmd[2] = CRSF_ADDRESS_RADIO_TRANSMITTER; // 0xEA
	packetCmd[3] = number;
	packetCmd[4] = value;
	
	writeExtPacket(addr, CRSF_FRAMETYPE_PARAMETER_WRITE, CRSF_ADDRESS_CRSF_TRANSMITTER, CRSF_ADDRESS_RADIO_TRANSMITTER, &packetCmd[3], 2);
}

