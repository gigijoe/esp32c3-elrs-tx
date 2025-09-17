/*
  Blink

  Flashes a LED every second, repeatedly.

  The ESP32-C3 SuperMini has an on-board LED you can control. 
  It is attached to digital pin 8. 
  LED_BUILTIN is set to the correct LED pin.

  This example code is in the public domain.

  Adapted from:
  https://www.arduino.cc/en/Tutorial/BuiltInExamples/Blink

  Official CRSF Protocol Specifications:
  https://github.com/crsf-wg/crsf/wiki

*/
#include <Arduino.h>

#include <Preferences.h>
#include <nvs.h>
#include <nvs_flash.h>

#include <AlfredoCRSF.h>
#include <HardwareSerial.h>

#define PIN_RX_OUT 21
#define PIN_TX_OUT 21
#define GPIO_PIN_RCSIGNAL_UART_INV false

HardwareSerial crsfBus(1);
AlfredoCRSF crsfInst;

void ICACHE_RAM_ATTR duplex_set_RX()
{
#if 1
	portDISABLE_INTERRUPTS();
	ESP_ERROR_CHECK(gpio_set_direction((gpio_num_t)PIN_RX_OUT, GPIO_MODE_INPUT));
#if GPIO_PIN_RCSIGNAL_UART_INV
	//gpio_matrix_out((gpio_num_t)PIN_TX_OUT, SIG_GPIO_OUT_IDX, true, true); // Detach TX & invertOut (Map out pin to gpio)
	gpio_matrix_in((gpio_num_t)PIN_RX_OUT, U1RXD_IN_IDX, true);
	gpio_pulldown_en((gpio_num_t)PIN_RX_OUT);
	gpio_pullup_dis((gpio_num_t)PIN_RX_OUT);
#else
	//gpio_matrix_out((gpio_num_t)PIN_TX_OUT, SIG_GPIO_OUT_IDX, false, false); // Detach TX (Map out pin to gpio)
	gpio_matrix_in((gpio_num_t)PIN_RX_OUT, U1RXD_IN_IDX, false);
	gpio_pullup_en((gpio_num_t)PIN_RX_OUT);
	gpio_pulldown_dis((gpio_num_t)PIN_RX_OUT);
#endif
	portENABLE_INTERRUPTS();
#endif
}

void ICACHE_RAM_ATTR duplex_set_TX()
{
#if 1
	portDISABLE_INTERRUPTS();
	ESP_ERROR_CHECK(gpio_set_pull_mode((gpio_num_t)PIN_TX_OUT, GPIO_FLOATING));
	ESP_ERROR_CHECK(gpio_set_pull_mode((gpio_num_t)PIN_RX_OUT, GPIO_FLOATING));
#if GPIO_PIN_RCSIGNAL_UART_INV
	ESP_ERROR_CHECK(gpio_set_level((gpio_num_t)PIN_TX_OUT, 0));
	ESP_ERROR_CHECK(gpio_set_direction((gpio_num_t)PIN_TX_OUT, GPIO_MODE_OUTPUT));
	//constexpr uint8_t MATRIX_DETACH_IN_LOW = 0x30;             // routes 0 to matrix slot
	constexpr uint8_t MATRIX_DETACH_IN_LOW = GPIO_FUNC_IN_LOW;
	gpio_matrix_in(MATRIX_DETACH_IN_LOW, U1RXD_IN_IDX, false); // Disconnect RX from all pads
	gpio_matrix_out((gpio_num_t)PIN_TX_OUT, U1TXD_OUT_IDX, true, false); // Attach TX & invertOut (Map ou pin to U1TXD)
#else
	ESP_ERROR_CHECK(gpio_set_level((gpio_num_t)PIN_TX_OUT, 1));
	ESP_ERROR_CHECK(gpio_set_direction((gpio_num_t)PIN_TX_OUT, GPIO_MODE_OUTPUT));
	//constexpr uint8_t MATRIX_DETACH_IN_HIGH = 0x38;             // routes 1 to matrix slot
	constexpr uint8_t MATRIX_DETACH_IN_HIGH = GPIO_FUNC_IN_HIGH;
	gpio_matrix_in(MATRIX_DETACH_IN_HIGH, U1RXD_IN_IDX, false); // Disconnect RX from all pads
	gpio_matrix_out((gpio_num_t)PIN_TX_OUT, U1TXD_OUT_IDX, false, false); // Attach TX (Map ou pin to U1TXD)
#endif
	portENABLE_INTERRUPTS();
#endif
}

// Fallback method to send default channel values
void sendFallbackChannels(uint8_t addr) {
	crsf_channels_t crsfChannels = { 0 };
	crsfChannels.ch0 = CRSF_CHANNEL_VALUE_1000;
	crsfChannels.ch1 = CRSF_CHANNEL_VALUE_1000;
	crsfChannels.ch2 = CRSF_CHANNEL_VALUE_1000;
	crsfChannels.ch3 = CRSF_CHANNEL_VALUE_1000;
	crsfChannels.ch4 = CRSF_CHANNEL_VALUE_1000;
	crsfChannels.ch5 = CRSF_CHANNEL_VALUE_1000;
	crsfChannels.ch6 = CRSF_CHANNEL_VALUE_1000;
	crsfChannels.ch7 = CRSF_CHANNEL_VALUE_1000;
	crsfChannels.ch8 = CRSF_CHANNEL_VALUE_1000;
	crsfChannels.ch9 = CRSF_CHANNEL_VALUE_1000;
	crsfChannels.ch10 = CRSF_CHANNEL_VALUE_1000;
	crsfChannels.ch11 = CRSF_CHANNEL_VALUE_1000;
	crsfChannels.ch12 = CRSF_CHANNEL_VALUE_1000;
	crsfChannels.ch13 = CRSF_CHANNEL_VALUE_1000;
	crsfChannels.ch14 = CRSF_CHANNEL_VALUE_1000;
	crsfChannels.ch15 = CRSF_CHANNEL_VALUE_1000;

	crsfInst.writePacket(addr, CRSF_FRAMETYPE_RC_CHANNELS_PACKED, &crsfChannels, sizeof(crsfChannels));
}

void sendChannels(uint8_t addr, int chs[CRSF_NUM_CHANNELS]) {
	crsf_channels_t crsfChannels = { 0 };

	crsfChannels.ch0 = map(chs[0], 1000, 2000, CRSF_CHANNEL_VALUE_1000, CRSF_CHANNEL_VALUE_2000);
	crsfChannels.ch1 = map(chs[1], 1000, 2000, CRSF_CHANNEL_VALUE_1000, CRSF_CHANNEL_VALUE_2000);
	crsfChannels.ch2 = map(chs[2], 1000, 2000, CRSF_CHANNEL_VALUE_1000, CRSF_CHANNEL_VALUE_2000);
	crsfChannels.ch3 = map(chs[3], 1000, 2000, CRSF_CHANNEL_VALUE_1000, CRSF_CHANNEL_VALUE_2000);
	crsfChannels.ch4 = map(chs[4], 1000, 2000, CRSF_CHANNEL_VALUE_1000, CRSF_CHANNEL_VALUE_2000);
	crsfChannels.ch5 = map(chs[5], 1000, 2000, CRSF_CHANNEL_VALUE_1000, CRSF_CHANNEL_VALUE_2000);
	crsfChannels.ch6 = map(chs[6], 1000, 2000, CRSF_CHANNEL_VALUE_1000, CRSF_CHANNEL_VALUE_2000);
	crsfChannels.ch7 = map(chs[7], 1000, 2000, CRSF_CHANNEL_VALUE_1000, CRSF_CHANNEL_VALUE_2000);
	crsfChannels.ch8 = map(chs[8], 1000, 2000, CRSF_CHANNEL_VALUE_1000, CRSF_CHANNEL_VALUE_2000);
	crsfChannels.ch9 = map(chs[9], 1000, 2000, CRSF_CHANNEL_VALUE_1000, CRSF_CHANNEL_VALUE_2000);
	crsfChannels.ch10 = map(chs[10], 1000, 2000, CRSF_CHANNEL_VALUE_1000, CRSF_CHANNEL_VALUE_2000);
	crsfChannels.ch11 = map(chs[11], 1000, 2000, CRSF_CHANNEL_VALUE_1000, CRSF_CHANNEL_VALUE_2000);
	crsfChannels.ch12 = map(chs[12], 1000, 2000, CRSF_CHANNEL_VALUE_1000, CRSF_CHANNEL_VALUE_2000);
	crsfChannels.ch13 = map(chs[13], 1000, 2000, CRSF_CHANNEL_VALUE_1000, CRSF_CHANNEL_VALUE_2000);
	crsfChannels.ch14 = map(chs[14], 1000, 2000, CRSF_CHANNEL_VALUE_1000, CRSF_CHANNEL_VALUE_2000);
	crsfChannels.ch15 = map(chs[15], 1000, 2000, CRSF_CHANNEL_VALUE_1000, CRSF_CHANNEL_VALUE_2000);

	crsfInst.writePacket(addr, CRSF_FRAMETYPE_RC_CHANNELS_PACKED, &crsfChannels, sizeof(crsfChannels));
}

// CRC8 implementation with polynom = 0xBA
const unsigned char crc8tab_BA[256] = {
0x00, 0xBA, 0xCE, 0x74, 0x26, 0x9C, 0xE8, 0x52,
0x4C, 0xF6, 0x82, 0x38, 0x6A, 0xD0, 0xA4, 0x1E,
0x98, 0x22, 0x56, 0xEC, 0xBE, 0x04, 0x70, 0xCA,
0xD4, 0x6E, 0x1A, 0xA0, 0xF2, 0x48, 0x3C, 0x86,
0x8A, 0x30, 0x44, 0xFE, 0xAC, 0x16, 0x62, 0xD8,
0xC6, 0x7C, 0x08, 0xB2, 0xE0, 0x5A, 0x2E, 0x94,
0x12, 0xA8, 0xDC, 0x66, 0x34, 0x8E, 0xFA, 0x40,
0x5E, 0xE4, 0x90, 0x2A, 0x78, 0xC2, 0xB6, 0x0C,
0xAE, 0x14, 0x60, 0xDA, 0x88, 0x32, 0x46, 0xFC,
0xE2, 0x58, 0x2C, 0x96, 0xC4, 0x7E, 0x0A, 0xB0,
0x36, 0x8C, 0xF8, 0x42, 0x10, 0xAA, 0xDE, 0x64,
0x7A, 0xC0, 0xB4, 0x0E, 0x5C, 0xE6, 0x92, 0x28,
0x24, 0x9E, 0xEA, 0x50, 0x02, 0xB8, 0xCC, 0x76,
0x68, 0xD2, 0xA6, 0x1C, 0x4E, 0xF4, 0x80, 0x3A,
0xBC, 0x06, 0x72, 0xC8, 0x9A, 0x20, 0x54, 0xEE,
0xF0, 0x4A, 0x3E, 0x84, 0xD6, 0x6C, 0x18, 0xA2,
0xE6, 0x5C, 0x28, 0x92, 0xC0, 0x7A, 0x0E, 0xB4,
0xAA, 0x10, 0x64, 0xDE, 0x8C, 0x36, 0x42, 0xF8,
0x7E, 0xC4, 0xB0, 0x0A, 0x58, 0xE2, 0x96, 0x2C,
0x32, 0x88, 0xFC, 0x46, 0x14, 0xAE, 0xDA, 0x60,
0x6C, 0xD6, 0xA2, 0x18, 0x4A, 0xF0, 0x84, 0x3E,
0x20, 0x9A, 0xEE, 0x54, 0x06, 0xBC, 0xC8, 0x72,
0xF4, 0x4E, 0x3A, 0x80, 0xD2, 0x68, 0x1C, 0xA6,
0xB8, 0x02, 0x76, 0xCC, 0x9E, 0x24, 0x50, 0xEA,
0x48, 0xF2, 0x86, 0x3C, 0x6E, 0xD4, 0xA0, 0x1A,
0x04, 0xBE, 0xCA, 0x70, 0x22, 0x98, 0xEC, 0x56,
0xD0, 0x6A, 0x1E, 0xA4, 0xF6, 0x4C, 0x38, 0x82,
0x9C, 0x26, 0x52, 0xE8, 0xBA, 0x00, 0x74, 0xCE,
0xC2, 0x78, 0x0C, 0xB6, 0xE4, 0x5E, 0x2A, 0x90,
0x8E, 0x34, 0x40, 0xFA, 0xA8, 0x12, 0x66, 0xDC,
0x5A, 0xE0, 0x94, 0x2E, 0x7C, 0xC6, 0xB2, 0x08,
0x16, 0xAC, 0xD8, 0x62, 0x30, 0x8A, 0xFE, 0x44
};

uint8_t crc8_BA(const uint8_t * ptr, uint32_t len)
{
	uint8_t crc = 0;
	for (uint32_t i=0; i<len; i++) {
		crc = crc8tab_BA[crc ^ *ptr++];
	}
	return crc;
}

#define CRSF_COMMAND_SUBCMD_RX         0x10 
#define CRSF_COMMAND_SUBCMD_RX_BIND    0x01
#define CRSF_COMMAND_MODEL_SELECT_ID   0x05

void writeModelId(uint8_t addr, uint8_t modelId)
{
	uint8_t packetCmd[7];
	
	packetCmd[0] = CRSF_FRAMETYPE_COMMAND; // 0x32
	packetCmd[1] = CRSF_ADDRESS_CRSF_TRANSMITTER; // 0xEE
	packetCmd[2] = CRSF_ADDRESS_RADIO_TRANSMITTER; // 0xEA
	packetCmd[3] = CRSF_COMMAND_SUBCMD_RX; // 0x10
	packetCmd[4] = CRSF_COMMAND_MODEL_SELECT_ID; // 0x05
	packetCmd[5] = modelId;
	packetCmd[6] = crc8_BA((const uint8_t *)&packetCmd[0], 6);
	
	crsfInst.writeExtPacket(addr, CRSF_FRAMETYPE_COMMAND, CRSF_ADDRESS_CRSF_TRANSMITTER, CRSF_ADDRESS_RADIO_TRANSMITTER, &packetCmd[3], 4);
}

// ELRS command
//#define ELRS_ADDRESS                    0xEE // CRSF_ADDRESS_CRSF_TRANSMITTER
#define ELRS_BIND_COMMAND               0xFF
#define ELRS_START_COMMAND              0x04
#define ELRS_WIFI_COMMAND               0xFE
#define ELRS_PKT_RATE_COMMAND           0x01
#define ELRS_TELM_RATIO_COMMAND         0x02
#define ELRS_SWITCH_MODE_COMMAND        0x03
#define ELRS_MODEL_MATCH_COMMAND        0x04
#define ELRS_POWER_COMMAND              0x06
#define ELRS_BLE_JOYSTIC_COMMAND        17
#define TYPE_SETTINGS_WRITE             0x2D

void writeElrsStatusRequest(uint8_t addr)
{
	crsfInst.writeParameterWrite(addr, 0, 0); // Parameter is 0 special case for elrs linkstat request
	crsfInst.waitForRxPacket(100);
}

void writeBroadcastPing(uint8_t addr)
{
	crsfInst.writeExtPacket(addr, CRSF_FRAMETYPE_DEVICE_PING, CRSF_ADDRESS_BROADCAST, CRSF_ADDRESS_RADIO_TRANSMITTER, 0, 0);
	crsfInst.waitForRxPacket(100);
}

void writeBinding(uint8_t addr)
{
#if 1
	uint8_t packetCmd[6];
	
	packetCmd[0] = CRSF_FRAMETYPE_COMMAND; // 0x32
	packetCmd[1] = CRSF_ADDRESS_CRSF_TRANSMITTER; // 0xEE
	packetCmd[2] = CRSF_ADDRESS_RADIO_TRANSMITTER; // 0xEA
	packetCmd[3] = CRSF_COMMAND_SUBCMD_RX; // 0x10
	packetCmd[4] = CRSF_COMMAND_SUBCMD_RX_BIND; // 0x01
	packetCmd[5] = crc8_BA((const uint8_t *)&packetCmd[0], 5);

	crsfInst.writeExtPacket(addr, CRSF_FRAMETYPE_COMMAND, CRSF_ADDRESS_CRSF_TRANSMITTER, CRSF_ADDRESS_RADIO_TRANSMITTER, &packetCmd[3], 3);
#endif
#if 0
	crsfInst.writeParameterWrite(addr, ELRS_BIND_COMMAND, ELRS_START_COMMAND);
	//crsfInst.writeParameterRead(addr, 0, 0);
	//crsfInst.waitForRxPacket(100);
#endif
}

#define STR_PACKET_RATE "100Hz"
#define KEY_PACKET_RATE_INDEX "keyPRI"
static int s_packetRateIndex = 0;
static uint32_t s_txInterval = 10000; // us / 100Hz

#define STR_MAX_TXPOWER "100"
#define KEY_MAX_TXPOWER_INDEX "keyMTPI"
static int s_maxTxPowerIndex = 0;

#define STR_TELM_RATIO "1:2"
#define KEY_TELM_RATIO_INDEX "keyTRI"
static int s_telemRatioIndex = 0;

uint8_t s_modelId = 0;
bool s_modelMatch = true;

/*
 *
 */

hw_timer_t *timer1 = NULL;
portMUX_TYPE timerMux1 = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR onTimer1() {
portENTER_CRITICAL_ISR(&timerMux1);
	digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
portEXIT_CRITICAL_ISR(&timerMux1);
}

// the setup function runs once when you press reset or power the board
void setup() {
	// initialize digital pin LED_BUILTIN as an output.
	pinMode(LED_BUILTIN, OUTPUT);
	pinMode(BOOT_BUILTIN, INPUT_PULLUP);      //input pull-up resistor is enabled
	pinMode(A4, INPUT_PULLUP);      //input pull-up resistor is enabled

#if 1 // Model ID by dip switch 	
	pinMode(ID0, INPUT_PULLUP);
	pinMode(ID1, INPUT_PULLUP);
	pinMode(ID2, INPUT_PULLUP);
	pinMode(ID3, INPUT_PULLUP);
	s_modelId = !digitalRead(ID0) | (!digitalRead(ID1) << 1) | (!digitalRead(ID2) << 2) | (!digitalRead(ID3) << 3);
#endif
	Serial.begin(115200);

	Serial.printf("[ Host Information ]\r\n");
	Serial.printf("\tModel ID : %d\r\n", s_modelId);
	Serial.printf("\r\n");

	timer1 = timerBegin(1, 80, true);               // timer 1, prescaler 80 --> 80MHz/80 = 1 MHz = 1us, count up
	timerAttachInterrupt(timer1, &onTimer1, true);   // attach handler, trigger on edge 
	timerAlarmWrite(timer1, 1000000L, true);            // 1000000 * 1us = 1000ms, single shot
	timerAlarmEnable(timer1); // start

	crsfBus.begin(CRSF_BAUDRATE, SERIAL_8N1, PIN_RX_OUT, PIN_TX_OUT, false, 500); // None invert, 500ms timeout  
	crsfInst.begin(crsfBus);
	
	duplex_set_RX();

	Preferences preferences;
	preferences.begin("elrs.cfg", false);
	s_packetRateIndex = preferences.getInt(KEY_PACKET_RATE_INDEX, 0);
	s_maxTxPowerIndex = preferences.getInt(KEY_MAX_TXPOWER_INDEX, 0);
	preferences.end();
}

int channels[CRSF_NUM_CHANNELS] = { 0 };

// the loop function runs over and over again forever
void loop() {
	uint32_t timeNow = micros();
	static unsigned long lastUpdate = 0;
	static uint32_t loopCount = 0;
	//static uint32_t txInterval = TxInterval[s_packetRate];
	//static uint32_t txInterval = 10000; // us / 100Hz

	static uint8_t paramNumber = 0; 
	static bool paramReadingOn = false;

	crsfInst.update();

	if(timeNow - lastUpdate >= s_txInterval) {
		if(digitalRead(BOOT_BUILTIN) == LOW) {
			writeBinding(CRSF_ADDRESS_CRSF_TRANSMITTER);
			if(paramNumber == 0) {
				paramReadingOn = true;
			}
		} else if(digitalRead(A4) == LOW) {
			//Serial.printf("A4\r\n");
			for(int i=0;i<CRSF_NUM_CHANNELS;++i) {
				if(i!=4)
					channels[i] = 2000;
			}
			sendChannels(CRSF_ADDRESS_CRSF_TRANSMITTER, channels);
		} else {
			if(loopCount <= 500) { // repeat 1000 packets to build connection to TX module
				sendFallbackChannels(CRSF_ADDRESS_CRSF_TRANSMITTER);
				loopCount++;
			} else if(loopCount > 500 && loopCount <= 505) {
				if(crsfInst.getDeviceAddress() == 0) { // Wait for device response
					writeBroadcastPing(CRSF_ADDRESS_CRSF_TRANSMITTER);
					loopCount--;
				} else
					loopCount++;
			} else if(loopCount > 505 && loopCount <= 510) {
				crsfInst.writeParameterWrite(CRSF_ADDRESS_CRSF_TRANSMITTER, ELRS_PKT_RATE_COMMAND, s_packetRateIndex);
				loopCount++;
			} else if(loopCount > 510 && loopCount <= 515) {
				crsfInst.writeParameterWrite(CRSF_ADDRESS_CRSF_TRANSMITTER, ELRS_POWER_COMMAND, s_maxTxPowerIndex);
				loopCount++;
			} else if(loopCount > 515 && loopCount <= 520) {
				crsfInst.writeParameterWrite(CRSF_ADDRESS_CRSF_TRANSMITTER, ELRS_TELM_RATIO_COMMAND, s_telemRatioIndex);
				loopCount++;
			} else if(loopCount > 520 && loopCount <= 525) {
				crsfInst.writeParameterWrite(CRSF_ADDRESS_CRSF_TRANSMITTER, ELRS_MODEL_MATCH_COMMAND, s_modelMatch);
				loopCount++;
			} else if(loopCount > 525 && loopCount <= 530) {
				writeModelId(CRSF_ADDRESS_CRSF_TRANSMITTER, s_modelId);
				loopCount++;
			} else {
				if(paramReadingOn && paramNumber < crsfInst.getDeviceFieldCount()) {
					if(crsfInst.getIsParamReadingDone()) {
						paramNumber++;
						if(paramNumber < crsfInst.getDeviceFieldCount()) {
							crsfInst.writeParameterRead(CRSF_ADDRESS_CRSF_TRANSMITTER, paramNumber, 0);
							//crsfInst.waitForRxPacket(100);
						} else { // All field read
							Serial.printf("Packet Rate is %s\r\n", crsfInst.getPacketRate().c_str());
							Serial.printf("Telemetry Ratio is %s unit %s\r\n", crsfInst.getTelemRatio().c_str(), crsfInst.getTelemRatioUnit().c_str());
							Serial.printf("Switch Mode is %s\r\n", crsfInst.getSwitchMode().c_str());
							Serial.printf("Model Match is %s ID is %s\r\n", crsfInst.getModelMatchEnabled() ? "Enabled" : "Disabled", crsfInst.getModelMatchId().c_str());
							Serial.printf("TX Power is %s\r\n", crsfInst.getTxPower().c_str());
							Serial.printf("Max Power is %s %s\r\n", crsfInst.getMaxPower().c_str(),crsfInst.getMaxPowerUnit().c_str());

							Serial.printf("Supported Packet Rates\r\n");

							std::size_t found = std::string::npos;
							int i = 0;
							for(i = 0; i < crsfInst.getPacketRateArray().size(); ++i) {
								auto & s = crsfInst.getPacketRateArray()[i];
								found = s.find(STR_PACKET_RATE);
								if(found != std::string::npos) {
									break;
								}
							}
							if(found != std::string::npos) {
								if(s_packetRateIndex != i) {
									s_packetRateIndex = i;
									Preferences preferences;
									preferences.begin("elrs.cfg", false);
									preferences.putInt(KEY_PACKET_RATE_INDEX, s_packetRateIndex);
									preferences.end();
								}
							}
							for(i = 0; i < crsfInst.getPacketRateArray().size(); ++i) {
								Serial.printf("%c\t%s\r\n", i == s_packetRateIndex ? '*' : ' ', crsfInst.getPacketRateArray()[i].c_str());
							}

							Serial.printf("Supported Max Power\r\n");

							found = std::string::npos;
							i = 0;
							for(i = 0; i < crsfInst.getMaxPowerArray().size(); ++i) {
								auto & s = crsfInst.getMaxPowerArray()[i];
								found = s.find(STR_MAX_TXPOWER);
								if(found != std::string::npos) {
									break;
								}
							}
							if(found != std::string::npos) {
								if(s_maxTxPowerIndex != i) {
									s_maxTxPowerIndex = i;
									Preferences preferences;
									preferences.begin("elrs.cfg", false);
									preferences.putInt(KEY_MAX_TXPOWER_INDEX, s_maxTxPowerIndex);
									preferences.end();
								}
							}
							for(i = 0; i < crsfInst.getMaxPowerArray().size(); ++i) {
								Serial.printf("%c\t%s\r\n", i == s_maxTxPowerIndex ? '*' : ' ', crsfInst.getMaxPowerArray()[i].c_str());
							}

							Serial.printf("Supported Telemetry Ratio\r\n");

							found = std::string::npos;
							i = 0;
							for(i = 0; i < crsfInst.getTelemRatioArray().size(); ++i) {
								auto & s = crsfInst.getTelemRatioArray()[i];
								found = s.find(STR_TELM_RATIO);
								if(found != std::string::npos) {
									break;
								}
							}
							if(found != std::string::npos) {
								if(s_telemRatioIndex != i) {
									s_telemRatioIndex = i;
									Preferences preferences;
									preferences.begin("elrs.cfg", false);
									preferences.putInt(KEY_TELM_RATIO_INDEX, s_telemRatioIndex);
									preferences.end();
								}
							}
							for(i = 0; i < crsfInst.getTelemRatioArray().size(); ++i) {
								Serial.printf("%c\t%s\r\n", i == s_telemRatioIndex ? '*' : ' ', crsfInst.getTelemRatioArray()[i].c_str());
							}

							loopCount = 0;
							paramReadingOn = false;
							paramNumber = 0;
						}
					} else if(crsfInst.getIsParamReading()) {
						if(crsfInst.getChunkRemaining() > 0) {
							crsfInst.writeParameterRead(CRSF_ADDRESS_CRSF_TRANSMITTER, paramNumber, crsfInst.getCurrentFieldChunk());
							//delayMicroseconds(1000);
							//crsfInst.waitForRxPacket(100);
						}
					} else if(paramNumber == 0 && 
							crsfInst.getIsParamReading() == false) {
						crsfInst.writeParameterRead(CRSF_ADDRESS_CRSF_TRANSMITTER, paramNumber, 0);
					}  
				}
				sendFallbackChannels(CRSF_ADDRESS_CRSF_TRANSMITTER);
			}      
		}
		lastUpdate = timeNow;
	}
}

