#pragma once

#include <Arduino.h>
#include <crc8.h>
#include <crsf_protocol.h>

#include <vector>

#if defined (CONFIG_IDF_TARGET_ESP32C3)
extern void ICACHE_RAM_ATTR duplex_set_RX();
extern void ICACHE_RAM_ATTR duplex_set_TX();
#else
extern void duplex_set_RX();
extern void duplex_set_TX();
#endif

enum eFailsafeAction { fsaNoPulses, fsaHold };

class AlfredoCRSF
{
public:
    // Packet timeout where buffer is flushed if no data is received in this time
    static const unsigned int CRSF_PACKET_TIMEOUT_MS = 100;
    static const unsigned int CRSF_FAILSAFE_STAGE1_MS = 300;

    AlfredoCRSF();
    void begin(Stream & port);
    void update();
    void write(const uint8_t *buf, size_t len);
    void queuePacket(uint8_t addr, uint8_t type, const void *payload, uint8_t len);
    void writePacket(uint8_t addr, uint8_t type, const void *payload, uint8_t len);
    void writeExtPacket(uint8_t addr, uint8_t type, uint8_t dest_addr, uint8_t orig_addr, const void *payload, uint8_t len);
    bool waitForRxPacket(uint32_t timeout_ms); // ms timeout

    // Return current channel value (1-based) in us
    int getChannel(unsigned int ch) const { return _channels[ch - 1]; }
    const crsf_channels_t *getChannelsPacked() const { return &_channelsPacked;}
    const crsfLinkStatistics_t *getLinkStatistics() const { return &_linkStatistics; }
    const crsf_sensor_gps_t *getGpsSensor() const { return &_gpsSensor; }
    const crsf_sensor_vario_t *getVarioSensor() const { return &_varioSensor; }
    const crsf_sensor_baro_altitude_t *getBaroAltitudeSensor() const { return &_baroAltitudeSensor; }
    const crsf_sensor_attitude_t *getAttitudeSensor() const { return &_attitudeSensor; }
    bool isLinkUp() const { return _linkIsUp; }
    uint32_t getUpdateInterval() const { return _updateInterval; }
    int32_t getCorrection() const { return _correction; }

    // Device
    uint8_t getDeviceAddress() { return _device_address; }
    const char *getDeviceName() { return _device_name; }
    uint8_t getDeviceFieldCount() { return _deviceInfo.fieldCnt; };
    
    // Parameter
    void writeParameterRead(uint8_t addr, uint8_t number, uint8_t chunk);
    uint8_t getCurrentParamNumber() { return _currentParamNumber; }
    uint8_t getCurrentFieldChunk() { return _currentFieldChunk; }
    uint8_t getChunkRemaining() { return _chunkRemaining; }
    bool getIsParamReading() { return _isParamReading; }
    bool getIsParamReadingDone() { return _isParamReadingDone; }
    
    void writeParameterWrite(uint8_t addr, uint8_t number, uint8_t value);
    
    uint8_t m_packetRateIndex, m_maxPowerIndex;
    uint8_t m_telemRatioIndex;
    std::string m_telemRatioUnit;
    uint8_t m_switchModeIndex;
    bool m_modelMatchEnabled;
    std::string m_modelMatchId;
    std::string m_txPower;
    std::string m_maxPowerUnit;
    int m_bindState;

    std::vector<std::string> m_packetRateArray;
    std::vector<std::string> m_telemRatioArray;
    std::vector<std::string> m_switchModeArray;
    std::vector<std::string> m_maxPowerArray;

    const std::vector<std::string> & getPacketRateArray() { return m_packetRateArray; }
    const std::vector<std::string> & getTelemRatioArray() { return m_telemRatioArray; }
    const std::vector<std::string> & getSwitchModeArray() { return m_switchModeArray; }
    const std::vector<std::string> & getMaxPowerArray() { return m_maxPowerArray; }

    uint8_t getPacketRateIndex() { return m_packetRateIndex; }
    uint8_t getTelemRatioIndex() { return m_telemRatioIndex; }
    uint8_t getSwitchModeIndex() { return m_switchModeIndex; }
    uint8_t getMaxPowerIndex() { return m_maxPowerIndex; }

    const std::string getPacketRate() { return m_packetRateIndex < m_packetRateArray.size() ? m_packetRateArray[m_packetRateIndex] : "Unknown"; }
    const std::string getTelemRatio() { return m_telemRatioIndex < m_telemRatioArray.size() ? m_telemRatioArray[m_telemRatioIndex] : "Unknown"; }
    const std::string getTelemRatioUnit() { return m_telemRatioUnit; }
    const std::string getSwitchMode() { return m_switchModeIndex < m_switchModeArray.size() ? m_switchModeArray[m_switchModeIndex] : "Unknown"; }
    bool getModelMatchEnabled() { return m_modelMatchEnabled; }
    const std::string getModelMatchId() { return m_modelMatchId; }
    const std::string getTxPower() { return m_txPower; }

    const std::string getMaxPower() { return m_maxPowerIndex < m_maxPowerArray.size() ? m_maxPowerArray[m_maxPowerIndex] : "Unknown"; }
    const std::string getMaxPowerUnit() { return m_maxPowerUnit; }
    int getBindState() { return m_bindState; }

private:
    Stream* _port;
    uint8_t _rxBuf[CRSF_MAX_PACKET_LEN+3];
    uint8_t _rxBufPos;
    Crc8 _crc;
    crsf_channels_t _channelsPacked;
    crsf_sensor_battery_t _batterySensor;
    crsfLinkStatistics_t _linkStatistics;
    crsf_sensor_gps_t _gpsSensor;
    crsf_sensor_vario_t _varioSensor;
    crsf_sensor_baro_altitude_t _baroAltitudeSensor;
    crsf_sensor_attitude_t _attitudeSensor;
    uint32_t _baud;
    uint32_t _lastReceive;
    uint32_t _lastChannelsPacket;
    bool _linkIsUp;
    int _channels[CRSF_NUM_CHANNELS];
    uint32_t _updateInterval;
    int32_t _correction;
    uint8_t _device_address;
    char _device_name[CRSF_MAX_NAME_LEN];
    deviceInformationPacket_t _deviceInfo;
    uint8_t _currentParamNumber;
    uint8_t _currentFieldChunk;
    uint8_t _chunkRemaining;
    uint8_t _paramData[CRSF_MAX_CHUNKS * CRSF_MAX_CHUNK_SIZE];
    uint32_t _paramDataSize;
    bool _isParamReading;
    bool _isParamReadingDone;

    void handleSerialIn();
    void handleByteReceived();
    void shiftRxBuffer(uint8_t cnt);
    void processPacketIn(uint8_t len);
    void checkPacketTimeout();
    void checkLinkDown();

    // Packet RX Handlers
    void packetChannelsPacked(const crsf_header_t *p);
    void packetBatterySensor(const crsf_header_t *p);
    void packetLinkStatistics(const crsf_header_t *p);
    void packetGps(const crsf_header_t *p);
    void packetVario(const crsf_header_t *p);
    void packetBaroAltitude(const crsf_header_t *p);
    void packetAttitude(const crsf_header_t *p);
    void packetDeviceInfo(const crsf_ext_header_t *p);
    void packetParameterSettingsEntry(const crsf_ext_header_t *p);
    void packetRadioId(const crsf_ext_header_t *p);
};
