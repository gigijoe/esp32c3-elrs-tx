#include <AlfredoCRSF.h>

AlfredoCRSF::AlfredoCRSF() :
    _crc(0xd5),
    _lastReceive(0), _lastChannelsPacket(0), _linkIsUp(false), _updateInterval(0), _correction(0), _device_address(0), _device_name("Unknown")
{
     
}

void AlfredoCRSF::begin(Stream &port)
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

    checkPacketTimeout();
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

    Serial.printf("Device Address 0x%02x, Frame Type 0x%02x\r\n", hdr->device_addr, hdr->type);
    for(int i=0;i<hdr->frame_size;i++) {
        Serial.printf("0x%02x ", _rxBuf[i+2]);
    }
    Serial.printf("\r\n");
    
    //if (hdr->device_addr == CRSF_ADDRESS_FLIGHT_CONTROLLER)
    {
        switch (hdr->type)
        {
        case CRSF_FRAMETYPE_GPS: // 0x02
            packetGps(hdr);
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
        case CRSF_FRAMETYPE_RADIO_ID: // 0x3A
            packetRadioId(extHdr);
            break;
        default:
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

void AlfredoCRSF::packetLinkStatistics(const crsf_header_t *p)
{
    const crsfLinkStatistics_t *link = (crsfLinkStatistics_t *)p->data;
    memcpy(&_linkStatistics, link, sizeof(_linkStatistics));
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

void AlfredoCRSF::packetRadioId(const crsf_ext_header_t *p)
{
    const uint8_t *data = (const uint8_t *)p->data;
    if(p->dest_addr == CRSF_ADDRESS_RADIO_TRANSMITTER && // 0xEA - radio address
        	data[0] == CRSF_FRAMETYPE_OPENTX_SYNC) { // 0x10 - timing correction frame
    	uint32_t updateInterval = be32toh(*(uint32_t *)&data[1]);
    	uint32_t correction = be32toh(*(uint32_t *)&data[5]);
    	// values are in 10th of micro-seconds
        updateInterval /= 10;
        correction /= 10;
        if (correction >= 0)
            correction %= updateInterval;
        else
            correction = -((-correction) % updateInterval);

    	_updateInterval = updateInterval;
    	_correction = correction;
    
        Serial.printf("Update Interval = %u, Correction = %u\r\n", updateInterval, correction);
    }
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
    
    Serial.printf("[ Device Onformation ]\r\n");
    Serial.printf("Device Address 0x%02x\r\n", _device_address);
    Serial.printf("Device Name %s\r\n", _device_name);
    Serial.printf("Serial No 0x%08x\r\n", _deviceInfo.serialNo);
    Serial.printf("Hardware Version 0x%08x\r\n", _deviceInfo.hardwareVer);
    Serial.printf("Foftware Version 0x%08x\r\n", _deviceInfo.softwareVer);
    Serial.printf("Field Count %d\r\n", _deviceInfo.fieldCnt);
    Serial.printf("Parameter Version %d\r\n", _deviceInfo.parameterVersion);
}

void AlfredoCRSF::write(uint8_t b)
{
    _port->write(b);
    _port->flush();
}

void AlfredoCRSF::write(const uint8_t *buf, size_t len)
{
    _port->write(buf, len);
    _port->flush();
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


