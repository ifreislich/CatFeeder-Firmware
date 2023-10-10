/*
 * Copyright (c) 2012-2023 Ian Freislich
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * $Id$
 */

#include <WiFi.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <FRAM.h>
#include <FastCRC.h>
#include <HX711.h>
#include <esp_sntp.h>
#include <RTClib.h>
#include <SNMP_Agent.h>
#include <time.h>
#include <sys/time.h>
#include <stdarg.h>
#include <EEPROM.h>
#include <esp_http_client.h>
#include <NimBLEDevice.h>

#define MAGIC			0xd41d8cd5
#define CFG_NCATS		7
#define CFG_NSCHEDULES	10
#define MAX_CERT		6144

struct schedule {
	uint8_t	hour;
	uint8_t	minute;
	uint8_t	weight;
	uint8_t	flags;
} __attribute__((__packed__));

struct cfg {
	uint32_t	magic;
	float		scale;
	long		offset;
	// end of callibration data. Do not add configuration items above.
	char		hostname[33];
	char		ssid[64];
	char		wpakey[64];
	uint32_t	flags;
	char		ntpserver[64];
	struct {
		char		name[20];
		uint8_t		facility;
		uint16_t	id;
		uint8_t		flags;
	} cat[CFG_NCATS];
	char		pad;
	uint8_t		perFeed;
	uint8_t		quota;
	uint8_t		cooloff;
	struct schedule schedule[CFG_NSCHEDULES];
	struct {
		uint8_t	pad;
		char	url[64];
		char	topic[64];
		char	username[16];
		char	password[16];
	} ntfy;
	char		snmpro[64];
	char		snmprw[64];
	char		timezone[32];
	char		cacrt[MAX_CERT + 1];
	uint8_t		BLEtimeout;
	uint16_t	crc;
} __attribute__((__packed__));

struct nvdata {
	uint32_t	magic;
	uint32_t	state;
	float		dispensedTotal;
	time_t		lastDispense;
	uint16_t	crc;
} __attribute__((__packed__));

#define HISTORY_NDAYS	365
struct nvhistory {
	uint32_t	magic;
	struct {
		float	dispensed;
		float	start;
		float	end;
	} day[HISTORY_NDAYS];
	uint16_t	crc;
} __attribute__((__packed__));

// 32 Bits, Bitmap Configuration Flags
#define CFG_NTFY_ENABLE		0x0001
#define CFG_DISPENSE_CLOSED	0x0002
#define CFG_ENGINEERING		0x0004
#define CFG_SNMP_ENABLE		0x0008
#define CFG_HX711_FAST		0x0010
#define CFG_NTFY_PROBLEM	0x0020
#define CFG_NTFY_DISPENSE	0x0040
#define CFG_NTFY_VISIT		0x0080
#define CFG_NTFY_INTRUDE	0x0100
#define CFG_HX711_GAIN_HIGH	0x0200
#define CFG_DOOR_MOTOR_SLOW	0x0400
#define CFG_NTFY_AUTH		0x0800

#define CFG_CAT_ACCESS		0x01
#define CFG_CAT_DISPENSE	0x02

#define CFG_SCHED_ENABLE	0x01
#define CFG_SCHED_SKIP		0x02

#define SCALE_FACTOR		247.408408f
#define SCALE_OFFSET		62236

#define PIN_CLOSE_LIMIT		17
#define PIN_OPEN_LIMIT		5

#define PIN_AUGER_EN		13
#define PIN_AUGER_DIR		14
#define PIN_AUGER_STEP		12
#define PIN_AUGER_SLEEP		23
#define PIN_AUGER_MICROSTEP	25
#define AUGER_DIR_CW		1
#define AUGER_DIR_CCW		0

#define PIN_DOOR_EN			16
#define PIN_DOOR_DIR		15
#define PIN_DOOR_STEP		2
#define PIN_DOOR_SLEEP		18
#define PIN_DOOR_MICROSTEP	4
#define DOOR_DIR_CLOSE		0
#define DOOR_DIR_OPEN		1

#define PIN_DATA0			27
#define PIN_DATA1			26
#define WEIGAND_TIMEOUT		20	// timeout in ms on Wiegand sequence 
#define DOOR_TIMEOUT		60	// Door stays locked for max X seconds

#define PIN_RTC_INTR		19

// 32 Bits, Bitmap States.
#define STATE_OPEN			0x00000001
#define STATE_WEIGHTREPORT	0x00000002
#define STATE_RTC_PRESENT	0x00000004
#define STATE_RTC_INTR		0x00000008
#define STATE_GOT_TIME		0x00000010
#define STATE_CHECK_HOPPER	0x00000020
#define STATE_NO_SCHEDULE	0x00000040
#define STATE_SKIP_DISPENSE	0x00000080
#define STATE_WEIGAND_DONE	0x00000100
#define STATE_UNUSED_0		0x00000200
#define STATE_MENU_SELECT	0x00000400
#define STATE_MENU_SSID		0x00000800
#define STATE_MENU_PSK		0x00001000
#define STATE_GOT_IP_ADDR	0x00002000
#define STATE_BOOTUP_NTFY	0x00004000
#define STATE_BLE_CONNECTED	0x00008000
#define STATE_BLE_DATA		0x00010000

#define OFFSET_CONFIG		0
#define OFFSET_NVDATA		OFFSET_CONFIG + sizeof(struct cfg)
#define OFFSET_NVHISTORY	(32767 - sizeof(struct nvhistory))

extern const char ca_pem_start[] asm("_binary_src_ca_pem_start");
extern const char ca_pem_end[] asm("_binary_src_ca_pem_end");
extern const char graph_start[] asm("_binary_src_dygraph_min_js_start");
extern const char graph_end[] asm("_binary_src_dygraph_min_js_end");
extern const char css_start[] asm("_binary_src_dygraph_css_start");
extern const char css_end[] asm("_binary_src_dygraph_css_end");
extern const char favicon_start[] asm("_binary_src_favicon_ico_start");
extern const char favicon_end[] asm("_binary_src_favicon_ico_end");

volatile uint64_t		databits;              // stores all of the data bits
volatile unsigned char	bitCount;              // number of bits currently captured

time_t					 closeTime = 0, lastAuth = 0, bootTime;
int						 haveFRAM = 0;
volatile struct nvdata	 nvdata;
struct cfg				 conf;
struct nvhistory		 history;
hw_timer_t				*timer;

NimBLEServer			*pServer = NULL;
NimBLECharacteristic	*pTxCharacteristic;
NimBLECharacteristic	*pRxCharacteristic;
String					 BleData;
#define UUID_UART		"6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define UUID_RX			"6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define UUID_TX			"6E400003-B5A3-F393-E0A9-E50E24DCCA9E"
class MyServerCallbacks: public BLEServerCallbacks {
	void onConnect(BLEServer* pServer) {
		nvdata.state |= STATE_BLE_CONNECTED;
	}

	void onDisconnect(BLEServer* pServer) {
		pServer->getAdvertising()->start();
		nvdata.state &= ~STATE_BLE_CONNECTED;
	}
};

class MyCallbacks: public BLECharacteristicCallbacks {
	void onWrite(BLECharacteristic *pCharacteristic) {
		BleData = pCharacteristic->getValue();
		if (BleData.length() > 0)
			nvdata.state |= STATE_BLE_DATA;
	}
};

void wshandleRoot(void);
void wshandleConfig(void);
void wshandle404(void);
void wshandleDoFeed(void);
void wshandleSave(void);
void wshandleReboot(void);
void wshandleCalibrate(void);
void wshandleDoCalibrate(void);
void wshandleTare(void);
void wshandleSchedule(void);
void wshandleScheduleSave(void);
void wshandleEngineering(void);
void wshandleDoEngineering(void);
void wshandleSSL(void);
void wshandleSaveSSL(void);
void wshandleDygraphJS(void);
void wshandleDygraphCSS(void);
void wshandleGraphData(void);
void wshandleFavIcon(void);

int checkCard(uint8_t, uint16_t);
const char *catName(uint8_t, uint16_t);
const char *getResetReason(void);
int comparSchedule(const void *, const void *);
float weigh(bool);
int snmpGetWeight(void);
int snmpGetTemp(void);
int snmpGetUptime(void);
void debug(bool, const char *, ...);
void ntpCallBack(struct timeval *);
void wifiEvent(WiFiEvent_t);
float dispense(int);
void defaultSettings(void);
void saveSettings(void);
void initConfig(void);
void closeDoor(void);
void openDoor(void);
void shutdownDoor(void);
void shutdownAuger(void);
void enableDoor(void);
void enableAuger(void);
void saveNvData(void);
void initNvData(void);
void saveNvHistory(void);
void initNvHistory(void);
uint8_t getNextAlarm(void);
uint8_t getCurrentAlarm(void);
void setAlarm(uint8_t);
TimeSpan getTz(void);
void ntfy(const char *, const char *, const uint8_t, const char *, ...);

void IRAM_ATTR
ISR_weigandTimer0(void)
{
	nvdata.state |= STATE_WEIGAND_DONE;
}

// interrupt that happens when INTO goes low (0 bit)
void IRAM_ATTR
ISR_D0(void)
{
	if (~nvdata.state & STATE_WEIGAND_DONE) {
		timerRestart(timer);
		if (!timerAlarmEnabled(timer))
		  timerAlarmEnable(timer);
		bitCount++;
		databits <<= 1;
	}
}

// interrupt that happens when INT1 goes low (1 bit)
void IRAM_ATTR
ISR_D1(void)
{
	if (~nvdata.state & STATE_WEIGAND_DONE) {
		timerRestart(timer);
		if (!timerAlarmEnabled(timer))
		  timerAlarmEnable(timer);
		bitCount++;
		databits <<= 1;
		databits |= 1;
	}
}

void IRAM_ATTR
ISR_RTC(void)
{
	nvdata.state |= STATE_RTC_INTR;
}

#define PIN_HX711_CLK	32
#define PIN_HX711_DATA	34  // This is an input only pin without pullup.
#define PIN_HX711_RATE	33
HX711		scale;
WebServer	webserver(80);
RTC_DS3231	rtc;
WiFiUDP		udp;
SNMPAgent	snmp;
FRAM		fram;

void
setup()
{
	NimBLEService	*pService;
	NimBLESecurity	*pSecurity;
	char			 hostname[28];
	uint16_t		 mid, pid;
	timeval			 tv;

	Serial.begin(115200);
	debug(false, "ESP chip: %s, %d Cores", ESP.getChipModel(), ESP.getChipCores());
	debug(false, "Reboot reason: %s", getResetReason());
	if (fram.begin(0x50) == FRAM_OK) {
		debug(false, "FRAM present");
		haveFRAM = 1;
	}
	else {
		debug(false, "FRAM not present, Fallback to flash EEPROM emulation");
		haveFRAM = 0;
		EEPROM.begin(8192);
	}
	if (Wire.setClock(400000))
		debug(false, "i2c clock=400kHz");
	else
		debug(false, "i2c clock=100kHz");
	initConfig();
	initNvData();
	initNvHistory();
	if (rtc.begin()) {
		debug(false, "RTC present");
		nvdata.state |= STATE_RTC_PRESENT;
		rtc.disable32K();
		rtc.writeSqwPinMode(DS3231_OFF);
		rtc.clearAlarm(1);
		rtc.clearAlarm(2);
		if (rtc.lostPower()) {
			debug(false, "RTC lost power, time is invalid");
		}
		else {
			debug(false, "Setting time from RTC");
			DateTime now = rtc.now();
			tv.tv_sec = now.unixtime();
			tv.tv_usec = 0;
			settimeofday(&tv, NULL);
			nvdata.state |= STATE_GOT_TIME;
			bootTime = now.unixtime();
		}
	}
	else {
		nvdata.state &= ~STATE_RTC_PRESENT;
		debug(false, "No RTC found");
	}
	configTzTime(conf.timezone, conf.ntpserver);
	sntp_set_time_sync_notification_cb(ntpCallBack);

	//RTC is in UTC
	DateTime alarm2(__DATE__, "00:00:00");
	rtc.setAlarm2(alarm2 - getTz(), DS3231_A2_Hour);
	if (nvdata.state & STATE_GOT_TIME)
		setAlarm(getNextAlarm());
	else {
		nvdata.state |= STATE_NO_SCHEDULE;
		rtc.disableAlarm(1);
	}
	pinMode(PIN_RTC_INTR, INPUT_PULLUP);

	pinMode(PIN_AUGER_EN, OUTPUT);
	digitalWrite(PIN_AUGER_EN, HIGH);
	pinMode(PIN_AUGER_SLEEP, OUTPUT);
	digitalWrite(PIN_AUGER_SLEEP, LOW);
	pinMode(PIN_AUGER_DIR, OUTPUT);
	pinMode(PIN_AUGER_STEP, OUTPUT);
	pinMode(PIN_AUGER_MICROSTEP, OUTPUT);
	digitalWrite(PIN_AUGER_MICROSTEP, HIGH);  // HIGH = 3200, LOW = 200 steps per revolution

	pinMode(PIN_DOOR_EN, OUTPUT);
	digitalWrite(PIN_DOOR_EN, HIGH);
	pinMode(PIN_DOOR_SLEEP, OUTPUT);
	digitalWrite(PIN_DOOR_SLEEP, LOW);
	pinMode(PIN_DOOR_DIR, OUTPUT);
	pinMode(PIN_DOOR_STEP, OUTPUT);
	pinMode(PIN_DOOR_MICROSTEP, OUTPUT);
	digitalWrite(PIN_DOOR_MICROSTEP, LOW);

	pinMode(PIN_DATA0, INPUT);
	pinMode(PIN_DATA1, INPUT);
	pinMode(PIN_CLOSE_LIMIT, INPUT);
	pinMode(PIN_OPEN_LIMIT, INPUT);
	if (!digitalRead(PIN_CLOSE_LIMIT))
		nvdata.state |= STATE_OPEN;

	//connect to WiFi
	WiFi.onEvent(wifiEvent, ARDUINO_EVENT_WIFI_STA_GOT_IP);
	WiFi.onEvent(wifiEvent, ARDUINO_EVENT_WIFI_STA_CONNECTED);
	WiFi.onEvent(wifiEvent, ARDUINO_EVENT_WIFI_STA_DISCONNECTED);
	WiFi.onEvent(wifiEvent, ARDUINO_EVENT_WIFI_STA_START);
	WiFi.mode(WIFI_STA);
	
	WiFi.disconnect(true);
	delay(100);
	snprintf(hostname, 28, "Feeder-%s", conf.hostname);
	if (WiFi.setHostname(hostname))
		debug(true, "Set hostname to '%s'", hostname);
	else
		debug(true, "Hostname failed");
	
	if (*conf.ssid && *conf.wpakey)
		WiFi.begin(conf.ssid, conf.wpakey);
	MDNS.begin(hostname);
	MDNS.addService("http", "tcp", 80);

	NimBLEDevice::init(hostname);
	NimBLEDevice::setPower(ESP_PWR_LVL_P1);
	NimBLEDevice::setSecurityAuth(true, true, true);
	NimBLEDevice::setSecurityPasskey(524316);
	NimBLEDevice::setSecurityIOCap(BLE_HS_IO_DISPLAY_ONLY);
	NimBLEDevice::setMTU(517);
	pServer = NimBLEDevice::createServer();
	pServer->setCallbacks(new MyServerCallbacks());
	pService = pServer->createService(UUID_UART);
	pTxCharacteristic = pService->createCharacteristic(UUID_TX,
	  NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::READ_ENC | NIMBLE_PROPERTY::READ_AUTHEN | NIMBLE_PROPERTY::NOTIFY);
	pRxCharacteristic = pService->createCharacteristic(UUID_RX,
	  NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_ENC | NIMBLE_PROPERTY::WRITE_AUTHEN);
	pRxCharacteristic->setCallbacks(new MyCallbacks());
	pService->start();
	pServer->getAdvertising()->start();

	pinMode(PIN_HX711_RATE, OUTPUT);
	digitalWrite(PIN_HX711_RATE, conf.flags & CFG_HX711_FAST ? 1 : 0); // 0: 10Hz, 1: 80Hz
	scale.begin(PIN_HX711_DATA, PIN_HX711_CLK, conf.flags & CFG_HX711_GAIN_HIGH ? 128 : 64);
	scale.set_scale(conf.scale);
	scale.set_offset(conf.offset);
	nvdata.state &= ~STATE_WEIGHTREPORT;

	snmp = SNMPAgent(conf.snmpro, conf.snmprw);
	snmp.setUDP(&udp);
	snmp.begin();
	snmp.addDynamicIntegerHandler(".1.3.6.1.4.1.5.0", snmpGetWeight);
	snmp.addDynamicIntegerHandler(".1.3.6.1.4.1.5.1", snmpGetUptime);
	snmp.addDynamicIntegerHandler(".1.3.6.1.4.1.5.2", snmpGetTemp);

	timer = timerBegin(0, 80, true);
	timerAttachInterrupt(timer, &ISR_weigandTimer0, true);
	timerAlarmWrite(timer, WEIGAND_TIMEOUT * 1000, false);

	ArduinoOTA.setHostname(hostname);
	ArduinoOTA
	.onStart([]() {
		String type;

		shutdownAuger();
		shutdownDoor();
		if (ArduinoOTA.getCommand() == U_FLASH)
		  type = "firmware";
		else // U_SPIFFS
		  type = "filesystem";

		// NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
		debug(true, "Updating: %s", type);
		ntfy(WiFi.getHostname(), "floppy_disk", 3, "Updating: %s", type);
	})
	.onEnd([]() {
		debug(true, "Rebooting");
		NimBLEDevice::deinit();
		delay(100);
	})
	.onProgress([](unsigned int progress, unsigned int total) {
		if (!(progress % 73728))
			debug(false, "Received: %7d of %7d", progress, total);
	})
	.onError([](ota_error_t error) {
		const char	*reason;
		switch (error) {
		  case OTA_AUTH_ERROR:
			reason = "Auth Failed";
		  case OTA_BEGIN_ERROR:
			reason = "Begin Failed";
		  case OTA_CONNECT_ERROR:
			reason = "Connect Failed";
		  case OTA_RECEIVE_ERROR:
			reason = "Receive Failed";
		  case OTA_END_ERROR:
			reason = "End Failed";
		  default:
			reason = "Undefined";
		}
		debug(true, "OTA failed Error[%u]: %s", error, reason);
		ntfy(WiFi.getHostname(), "warning", 3, "OTA failed: %s", reason);
	});

	webserver.on("/", wshandleRoot);
	webserver.on("/config", wshandleConfig);
	webserver.on("/save", wshandleSave);
	webserver.on("/reboot", wshandleReboot);
	webserver.on("/feed", wshandleDoFeed);
	webserver.on("/calibrate", wshandleCalibrate);
	webserver.on("/docalibrate", wshandleDoCalibrate);
	webserver.on("/engineering", wshandleEngineering);
	webserver.on("/doengineering", wshandleDoEngineering);
	webserver.on("/tare", wshandleTare);
	webserver.on("/schedule", wshandleSchedule);
	webserver.on("/schedulesave", wshandleScheduleSave);
	webserver.on("/ssl", wshandleSSL);
	webserver.on("/sslsave", wshandleSaveSSL);
	webserver.on("/dygraph.min.js", wshandleDygraphJS);
	webserver.on("/dygraph.css", wshandleDygraphCSS);
	webserver.on("/data.txt", wshandleGraphData);
	webserver.on("/favicon.ico", wshandleFavIcon);
	webserver.onNotFound(wshandle404);
	webserver.begin();
	ArduinoOTA.begin();
	closeDoor();
	openDoor();
	attachInterrupt(PIN_DATA0, ISR_D0, FALLING);
	attachInterrupt(PIN_DATA1, ISR_D1, FALLING);
	attachInterrupt(PIN_RTC_INTR, ISR_RTC, FALLING);
	debug(true, "Ready");
	ntfy(WiFi.getHostname(), "facepalm", 3, "Reset reason: %s\\nFirmware: %s %s", getResetReason(), __DATE__, __TIME__);
}

void
loop()
{
	static char	 menudata[128];
	static int	 menupos = 0;
	int			 BleDataPos = 0;
	const char	*text;
	uint8_t		 facilityCode = 0, alarmIdx, alarmNextIdx;      // decoded facility code
	uint16_t	 cardCode = 0;          // decoded card code
	float		 dispensed;
	
	ArduinoOTA.handle();
	webserver.handleClient();
	if (conf.flags & CFG_SNMP_ENABLE)
		snmp.loop();
	
	while (Serial.available() || nvdata.state & STATE_BLE_DATA) {
		if (Serial.available())
			menudata[menupos] = Serial.read();
		if (nvdata.state & STATE_BLE_DATA) {
			menudata[menupos] = BleData.charAt(BleDataPos++);
			if (!(nvdata.state & (STATE_MENU_PSK | STATE_MENU_SSID)) || BleDataPos >= BleData.length())
				nvdata.state &= ~STATE_BLE_DATA;
		}
		if (nvdata.state & (STATE_MENU_PSK | STATE_MENU_SSID)) {
			if (menudata[menupos] == 8) { // chr(8) is backspace
				if (menupos) {
					Serial.printf("%c %c", menudata[menupos], menudata[menupos]);
				}
				menudata[menupos] = '\0';
				menupos -= 2;
			}
			else if (menudata[menupos] != '\r' && menudata[menupos] != '\n'){
				Serial.print(menudata[menupos]);
			}
		}

        if (menupos < 0 || ++menupos == 64)
            menupos = 0;
        menudata[menupos] = '\0';
		if (nvdata.state & STATE_MENU_SELECT && menupos == 1) {
			switch (menudata[menupos - 1]) {
			  case 'c': {
				char	mac[18];
				WiFi.macAddress().toCharArray(mac, 18);
				snprintf(menudata, 128, "SSID: %s\r\nPSK: %s\r\nIP address: %s\r\nMAC Address: %s\r\nMDNS Name: feeder-%s.local\r\n",
					conf.ssid, *conf.wpakey ? "Set" : "Not set", WiFi.localIP().toString().c_str(), mac, conf.hostname);
				if (nvdata.state & STATE_BLE_CONNECTED) {
					pTxCharacteristic->setValue((uint8_t *)menudata, strlen(menudata));
					pTxCharacteristic->notify();
				}
				Serial.printf(menudata);
				break;
			  }
			  case 'd':
				snprintf(menudata, 128, "Removing %d paired devices\r\n", NimBLEDevice::getNumBonds());
				if (nvdata.state & STATE_BLE_CONNECTED) {
					pTxCharacteristic->setValue((uint8_t *)menudata, strlen(menudata));
					pTxCharacteristic->notify();
				}
				Serial.printf(menudata);
				NimBLEDevice::deleteAllBonds();
				break;
			  case 'e':
				conf.flags |= CFG_ENGINEERING;
				text = "Engieering settings enabled. Write to NVRAM to save.\r\n";
				if (nvdata.state & STATE_BLE_CONNECTED) {
					pTxCharacteristic->setValue((uint8_t *)text, strlen(text));
					pTxCharacteristic->notify();
				}
				Serial.print(text);
				break;
			  case 'f':
				nvdata.dispensedTotal = 0;
				nvdata.state &= STATE_OPEN | STATE_MENU_SELECT;
				nvdata.lastDispense = time(NULL) - conf.cooloff * 60;
				nvdata.magic = MAGIC;
				defaultSettings();
				text = "Config and persistent data cleared. Rebooting...\r\n";
				if (nvdata.state & STATE_BLE_CONNECTED) {
					pTxCharacteristic->setValue((uint8_t *)text, strlen(text));
					pTxCharacteristic->notify();
				}
				Serial.print(text);
				saveSettings();
				saveNvData();
				NimBLEDevice::deinit();
				ESP.restart();
				break;
			  case 'r':
				text = "Rebooting\r\n";
				if (nvdata.state & STATE_BLE_CONNECTED) {
					pTxCharacteristic->setValue((uint8_t *)text, strlen(text));
					pTxCharacteristic->notify();
				}
				Serial.print(text);
				NimBLEDevice::deinit();
				ESP.restart();
				break;
			  case 's':
				text = "Enter SSID:";
				if (nvdata.state & STATE_BLE_CONNECTED) {
					pTxCharacteristic->setValue((uint8_t *)text, strlen(text));
					pTxCharacteristic->notify();
				}
				Serial.print(text);
				nvdata.state |= STATE_MENU_SSID;
				nvdata.state &= ~STATE_MENU_SELECT;
				break;
			  case 'p':
			  	text = "Enter PSK:";
				if (nvdata.state & STATE_BLE_CONNECTED) {
					pTxCharacteristic->setValue((uint8_t *)text, strlen(text));
					pTxCharacteristic->notify();
				}
				Serial.print(text);
				nvdata.state |= STATE_MENU_PSK;
				nvdata.state &= ~STATE_MENU_SELECT;
				break;
			  case 'w':
				saveSettings();
				saveNvData();
				text = "Configuration and NVDATA saved\r\n";
				if (nvdata.state & STATE_BLE_CONNECTED) {
					pTxCharacteristic->setValue((uint8_t *)text, strlen(text));
					pTxCharacteristic->notify();
				}
				Serial.print(text);
				break;
			  case '\r':
			  case '\n':
			  	break;
			  default:
				text = "Menu\r\n"
				"c: Show WiFi config\r\n"
				"d: Delete BlueTooth pairings\r\n"
				"e: Enable Engineering settings\r\n"
				"f: Factory default\r\n"
				"s: Set SSID\r\n"
				"p: Set WPA2 PSK\r\n"
				"w: Write config to NVRAM\r\n"
				"r: Reboot\r\n";
				if (nvdata.state & STATE_BLE_CONNECTED) {
					pTxCharacteristic->setValue((uint8_t *)text, strlen(text));
					pTxCharacteristic->notify();
				}
				Serial.print(text);
				break;
			}
			menupos = 0;
		}
		if (menupos && (menudata[menupos-1] == '\r' || menudata[menupos-1] == '\n')) {
			if (nvdata.state & STATE_MENU_SSID) {
				Serial.println();
				menudata[menupos-1] = '\0';
				strncpy(conf.ssid, menudata, 64);
				Serial.printf("SSID set to '%s'.  Write to NVRAM to save.\r\n", conf.ssid);
				if (*conf.ssid && *conf.wpakey)
					WiFi.begin(conf.ssid, conf.wpakey);
			}
			if (nvdata.state & STATE_MENU_PSK) {
				Serial.println();
				menudata[menupos-1] = '\0';
				strncpy(conf.wpakey, menudata, 64);
				Serial.printf("WPA2 PSK set to '%s'.  Write to NVRAM to save.\r\n", conf.wpakey);
				if (*conf.ssid && *conf.wpakey)
					WiFi.begin(conf.ssid, conf.wpakey);
			}
			menupos = 0;
			nvdata.state &= ~(STATE_MENU_SSID | STATE_MENU_PSK);
			nvdata.state |= STATE_MENU_SELECT;
		}

    }

	if (NimBLEDevice::getInitialized() && conf.BLEtimeout && !(nvdata.state & STATE_BLE_CONNECTED) &&
	  nvdata.state & STATE_GOT_TIME && time(NULL) - bootTime > conf.BLEtimeout * 60) {
		debug(true, "Disabling BlueTooth");
		NimBLEDevice::deinit();
	}

	if (nvdata.state & STATE_RTC_INTR) {
		nvdata.state &= ~STATE_RTC_INTR;
		if (rtc.alarmFired(1)) {
			uint8_t feedWeight;
			float	curWeight;
			const char *reason1;
			const char *reason2;

			debug(true, "RTC Alarm 1");
			rtc.clearAlarm(1);
			alarmIdx = getCurrentAlarm();
			alarmNextIdx = getNextAlarm();
			if (conf.schedule[alarmIdx].weight)
			    feedWeight = conf.schedule[alarmIdx].weight;
			else
			    feedWeight = conf.perFeed;

			// We may already have dispensed the quota but that will 
			// be caught later.
			if (nvdata.dispensedTotal + feedWeight > conf.quota)
				feedWeight = abs(conf.quota - nvdata.dispensedTotal);
			
			curWeight = weigh(true);
			reason1 = weigh(true) > feedWeight / 2.0 ? "High residual weight" : "";
			reason2 = nvdata.state & STATE_SKIP_DISPENSE ? "No visit after dispense" : "";

			if (!((conf.schedule[alarmIdx].flags & CFG_SCHED_SKIP) && (~nvdata.state & STATE_CHECK_HOPPER) && (*reason1 || *reason2))) {
				dispensed = dispense(feedWeight);
				nvdata.dispensedTotal += dispensed;
				if (~nvdata.state & STATE_CHECK_HOPPER && conf.cooloff)
					nvdata.state |= STATE_SKIP_DISPENSE;
				debug(true, "Auto-dispense: %3.0fg of %dg Dispensed total: %3.0fg of %dg Next dispense: %02d:%02d",
					dispensed, feedWeight, nvdata.dispensedTotal, conf.quota, conf.schedule[alarmNextIdx].hour, conf.schedule[alarmNextIdx].minute);
				if (conf.flags & CFG_NTFY_DISPENSE)
					ntfy(WiFi.getHostname(), "alarm_clock", 3, "Auto-dispense: %3.0fg of %dg\\nDispensed total: %3.0fg of %dg\\nNext dispense: %02d:%02d",
					  dispensed, feedWeight, nvdata.dispensedTotal, conf.quota, conf.schedule[alarmNextIdx].hour, conf.schedule[alarmNextIdx].minute);
			}
			else {
				debug(true, "Auto-dispense: Skipped. Dispensed total: %3.0fg of %dg Next dispense: %02d:%02d",
					nvdata.dispensedTotal, conf.quota, conf.schedule[alarmNextIdx].hour, conf.schedule[alarmNextIdx].minute);
				if (conf.flags & CFG_NTFY_DISPENSE)
					ntfy(WiFi.getHostname(), "alarm_clock", 3, "Auto-dispense: Skipped (%s%s%s)\\n"
					  "Dispensed total: %3.0fg of %dg\\n"
					  "Next dispense: %02d:%02d",
					  reason1, *reason1 && *reason2 ? ", " : "", reason2, nvdata.dispensedTotal, conf.quota,
					  conf.schedule[alarmNextIdx].hour, conf.schedule[alarmNextIdx].minute);

			}
			setAlarm(alarmNextIdx); 
		}
		if (rtc.alarmFired(2)) {
			debug(true, "RTC Alarm 2");
			rtc.clearAlarm(2);
			history.day[0].end = weigh(true);
			history.day[0].dispensed = nvdata.dispensedTotal;
			for (int i = HISTORY_NDAYS - 1; i > 0; i--) {
				history.day[i].dispensed = history.day[i-1].dispensed;
				history.day[i].start = history.day[i-1].start;
				history.day[i].end = history.day[i-1].end;
			}
			history.day[0].start = weigh(true);
			saveNvHistory();
			nvdata.dispensedTotal = 0;
			debug(true, "Reset dispensed total");
		}
		saveNvData();
	}
	
	if ((~nvdata.state & STATE_OPEN) && digitalRead(PIN_CLOSE_LIMIT) && time(NULL) > DOOR_TIMEOUT + closeTime)
		openDoor();
	// If the door is supposed to be closed but the limit switch doesn't read closed,
	// Attempt to re-close the door
	if ((~nvdata.state & STATE_OPEN) && !digitalRead(PIN_CLOSE_LIMIT)) {
		debug(true, "Door should be closed, reclosing");
		closeDoor();
	}

	weigh(false);
	if (nvdata.state & STATE_WEIGHTREPORT && time(NULL) - lastAuth > 120) {
		if (conf.flags & CFG_NTFY_VISIT)
			ntfy(WiFi.getHostname(), "balance_scale", 3, "End weight: %3.0fg\\nDispensed total: %3.0fg of %dg",
			  weigh(true), nvdata.dispensedTotal, conf.quota);
		nvdata.state &= ~STATE_WEIGHTREPORT;
	}

	// if we have bits and we the weigand transmission timed out
	if (bitCount > 0 && nvdata.state & STATE_WEIGAND_DONE) {
		switch (bitCount) {
		  case 26: // standard 26 bit format
			// Even parity over bits 2-13 = bit 1
			// facility code = bits 2 to 9 (256) mask = 0b01111111100000000000000000
			// card code = bits 10 to 25 (65535) mask = 0b11111111111111110
			// Odd parity over bits 14-25 = bit 26
			facilityCode = (databits & 0x1FE0000) >> 17;
		 	cardCode = (databits & 0x1FFFE) >> 1;
		 	break;
		  default:
			break;
			// you can add other formats if you want!
			// Serial.println("Unable to decode.");
		}
		if (facilityCode && cardCode) {
			debug(true, "Read card: facility %d, card %d", facilityCode, cardCode);
			checkCard(facilityCode, cardCode);
		}
	}

	// cleanup and get ready for the next card
	if (nvdata.state & STATE_WEIGAND_DONE) {
		bitCount = 0;
		databits = 0;
		nvdata.state &= ~STATE_WEIGAND_DONE;
	}
}

int
checkCard(uint8_t facilityCode, uint16_t cardCode)
{
	float	 dispensed;
	int		 i;

	for (i = 0; i < CFG_NCATS; i++)
		if (conf.cat[i].facility == facilityCode && conf.cat[i].id == cardCode)
			break;

	if (i == CFG_NCATS) {
		debug(true, "Intruder: facility %d, card %d", facilityCode, cardCode);
		closeDoor();
		if (conf.flags & CFG_NTFY_INTRUDE)
			ntfy(WiFi.getHostname(), "no_entry", 3, "Intruder: facility %d, card %d", facilityCode, cardCode);
		return(0);
	}
	if (conf.cat[i].flags & CFG_CAT_ACCESS) {
		nvdata.state |= STATE_WEIGHTREPORT;
		nvdata.state &= ~STATE_SKIP_DISPENSE;
		if (~nvdata.state & STATE_OPEN)
			openDoor();
		if (time(NULL) - lastAuth > 120) {
			debug(true, "Authorized: %s", catName(facilityCode, cardCode));
			if (conf.flags & CFG_NTFY_VISIT)
				ntfy(WiFi.getHostname(), "plate_with_cutlery", 3, "Authorized: %s\\nStart weight: %3.0fg", catName(facilityCode, cardCode), weigh(true));
		}
		lastAuth = time(NULL);
	}
	else if (conf.cat[i].flags & CFG_CAT_DISPENSE) {
		uint8_t feedWeight = conf.perFeed;
		if (nvdata.dispensedTotal + feedWeight > conf.quota)
			feedWeight = abs(conf.quota - nvdata.dispensedTotal);
		dispensed = dispense(feedWeight);
		nvdata.dispensedTotal += dispensed;
		if (~nvdata.state & STATE_CHECK_HOPPER && conf.cooloff)
			nvdata.state |= STATE_SKIP_DISPENSE;
		if (conf.flags & CFG_NTFY_DISPENSE)
			ntfy(WiFi.getHostname(), "cook", 3, "Manual-dispense: %3.0fg of %dg\\nDispensed total: %3.0fg of %dg", dispensed, conf.perFeed, nvdata.dispensedTotal, conf.quota);
	}
	else {
		closeDoor();
		debug(true, "Intruder: %s", catName(facilityCode, cardCode));
		if (conf.flags & CFG_NTFY_INTRUDE)
			ntfy(WiFi.getHostname(), "no_entry", 3, "Intruder: %s", catName(facilityCode, cardCode));
	}
	saveNvData();
	return (1);
}

const char *
catName(uint8_t facilityCode, uint16_t cardCode)
{
	int i;
	for (i = 0; i < CFG_NCATS; i++)
		if (conf.cat[i].facility == facilityCode && conf.cat[i].id == cardCode)
			break;

	if (i == CFG_NCATS)
		return("Unnamed");
		
	return(conf.cat[i].name);
}

void
closeDoor(void)
{
	int	 i;

	closeTime = time(NULL);
	if (digitalRead(PIN_CLOSE_LIMIT) && ~nvdata.state & STATE_OPEN) // Just reset the open timer if the door is closed
		return;
	debug(true, "Closing");
	digitalWrite(PIN_DOOR_DIR, DOOR_DIR_CLOSE);
	enableDoor();
	for (i = 0; i < 3000 && !digitalRead(PIN_CLOSE_LIMIT); i++) {
		digitalWrite(PIN_DOOR_STEP, HIGH);
		digitalWrite(PIN_DOOR_STEP, LOW);
		delayMicroseconds(conf.flags & CFG_DOOR_MOTOR_SLOW ? 2500 : 1200);
	}
	shutdownDoor();
	nvdata.state &= ~STATE_OPEN;
}

void
openDoor(void)
{
	debug(true, "Opening");
	digitalWrite(PIN_DOOR_DIR, DOOR_DIR_OPEN);
	enableDoor();
	for (int i = 0; i < 3000 && !digitalRead(PIN_OPEN_LIMIT); i++) {
		digitalWrite(PIN_DOOR_STEP, HIGH);
		digitalWrite(PIN_DOOR_STEP, LOW);
		delayMicroseconds(conf.flags & CFG_DOOR_MOTOR_SLOW ? 2500 : 1200);
	}
	nvdata.state |= STATE_OPEN;
	shutdownDoor();
}

#define STEP_DELAY 250
float
dispense(int grams)
{
	float	 startWeight = weigh(false), curWeight;

	// Override cooloff period if the last dispense didn't fully succeed.
	if (~nvdata.state & STATE_CHECK_HOPPER && (time(NULL) < nvdata.lastDispense + conf.cooloff * 60 || nvdata.dispensedTotal >= conf.quota))
		return(0);

	debug(true, "Dispense %dg", grams);
	if (conf.flags & CFG_DISPENSE_CLOSED)
		closeDoor();
	enableAuger();
	// This is a bit of a magic number. 5 revolutions with a reversal
	// of 0.25 revolution every revolution dispenses about 10 grams
	// if the kibbles don't get stuck. So, try rotating 4 as much as
	// needed which works out to 2 revolution per gram.
	curWeight = startWeight;
	for(uint8_t i = 0, stop = 0; !stop && i < grams * 2; i++) {
		digitalWrite(PIN_AUGER_DIR, AUGER_DIR_CW);
		for (int i = 0; !stop && i < 3200; i++) {
			digitalWrite(PIN_AUGER_STEP, HIGH);
			digitalWrite(PIN_AUGER_STEP, LOW);
			delayMicroseconds(STEP_DELAY);
			// Weigh every 45 degrees rotation
			if (i % 400 == 0) {
				curWeight = weigh(false);
				if (curWeight - startWeight >= grams)
					stop = 1;
				webserver.handleClient();
			}
		}
		digitalWrite(PIN_AUGER_DIR, AUGER_DIR_CCW);
		for (int i = 0; !stop && i < 800; i++) {
			digitalWrite(PIN_AUGER_STEP, HIGH);
			digitalWrite(PIN_AUGER_STEP, LOW);
			delayMicroseconds(STEP_DELAY);
		}
	}
	shutdownAuger();
	if (conf.flags & CFG_DISPENSE_CLOSED)
		openDoor();
	nvdata.lastDispense = time(NULL);
	grams - (curWeight - startWeight) > 3 ? nvdata.state |= STATE_CHECK_HOPPER : nvdata.state &= ~STATE_CHECK_HOPPER;
	if (nvdata.state & STATE_CHECK_HOPPER && conf.flags & CFG_NTFY_PROBLEM)
		ntfy(WiFi.getHostname(), "warning", 4, "Check Hopper");
	nvdata.state |= STATE_SKIP_DISPENSE;
	return(curWeight - startWeight);
}

void
shutdownDoor(void)
{
	digitalWrite(PIN_DOOR_EN, HIGH);
	digitalWrite(PIN_DOOR_SLEEP, LOW);
}
void
shutdownAuger(void)
{
	digitalWrite(PIN_AUGER_EN, HIGH);
	digitalWrite(PIN_AUGER_SLEEP, LOW);
}
void
enableDoor(void)
{
	digitalWrite(PIN_DOOR_EN, LOW);
	digitalWrite(PIN_DOOR_SLEEP, HIGH);
}
void
enableAuger(void)
{
	digitalWrite(PIN_AUGER_EN, LOW);
	digitalWrite(PIN_AUGER_SLEEP, HIGH);
}

float
weigh(bool doAverage)
{
	static float	 weight[100] = {0};
	static uint8_t	 pos = 0;
	float			 total = 0;

	if (pos == 100)
		pos = 0;
	weight[pos] = scale.get_units(1);

	if (!doAverage)
		return(weight[pos++]);

	pos++;
	for (int i = 0; i < 100; i++)
		total += weight[i];
	return(total / 100);
}

int
snmpGetWeight(void)
{
	return(weigh(true) * 1000);
}

int
snmpGetTemp(void)
{
	return(rtc.getTemperature()*100);
}

int
snmpGetUptime(void)
{
	return(time(NULL) - bootTime);
}

void
ntpCallBack(struct timeval *tv)
{
	DateTime	 now(tv->tv_sec);
	
	if (~nvdata.state & STATE_GOT_TIME) {
		setAlarm(getNextAlarm());
		bootTime = tv->tv_sec - millis() / 1000;
	}
	
	nvdata.state |= STATE_GOT_TIME;

	if (nvdata.state & STATE_RTC_PRESENT)
		rtc.adjust(now);
}

void
wifiEvent(WiFiEvent_t event)
{
	switch (event) {
	case ARDUINO_EVENT_WIFI_STA_GOT_IP:
		debug(true, "WiFi IP address %s", WiFi.localIP().toString().c_str());
		nvdata.state |= STATE_GOT_IP_ADDR;
		break;
	case ARDUINO_EVENT_WIFI_STA_CONNECTED:
		debug(true, "WiFi Connected");
		break;
	case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
		debug(true, "WiFi Disconnected, reconnecting");
		if (*conf.ssid && *conf.wpakey)
			WiFi.begin(conf.ssid, conf.wpakey);
		nvdata.state &= ~STATE_GOT_IP_ADDR;
		break;
	case ARDUINO_EVENT_WIFI_STA_START:
		debug(true, "WiFi Connecting");
		break;
	default:
		break;
	}
}

void
initNvHistory(void)
{
	FastCRC16	 CRC16;
	uint32_t	 clock;

	if (haveFRAM) {
		if ((clock = Wire.getClock()) > 0)
			Wire.setClock(1000000);
		fram.read(OFFSET_NVHISTORY, (uint8_t *)&history, sizeof(struct nvhistory));
		if (clock > 0)
			Wire.setClock(clock);
	}
	if (history.magic != MAGIC || history.crc != CRC16.ccitt((uint8_t *)&history, sizeof(struct nvhistory) - 2)) {
		debug(false, "Resetting history");
		memset(&history, '\0', sizeof(struct nvhistory));
		history.magic = MAGIC;
		history.crc = CRC16.ccitt((uint8_t *)&history, sizeof(struct nvhistory) - 2);
		saveNvHistory();
	}
}

void
saveNvHistory(void)
{
	FastCRC16	 CRC16;
	uint32_t	 clock;

	if (haveFRAM) {
		if ((clock = Wire.getClock()) > 0)
			Wire.setClock(1000000);
		history.crc = CRC16.ccitt((uint8_t *)&history, sizeof(struct nvhistory) - 2);
		fram.write(OFFSET_NVHISTORY, (uint8_t *)&history, sizeof(struct nvhistory));
		if (clock > 0)
			Wire.setClock(clock);
	}
}

void
initNvData(void)
{
	FastCRC16	 CRC16;
	uint32_t	 clock;

	if (haveFRAM) {
		if ((clock = Wire.getClock()) > 0)
			Wire.setClock(1000000);
		fram.read(OFFSET_NVDATA, (uint8_t *)&nvdata, sizeof(nvdata));
		if (clock > 0)
			Wire.setClock(clock);
	}
	if (nvdata.magic != MAGIC || nvdata.crc != CRC16.ccitt((uint8_t *)&nvdata, sizeof(struct nvdata) - 2)) {
		debug(false, "Resetting persistent records");
		nvdata.dispensedTotal = 0;
		nvdata.state = 0;
		nvdata.lastDispense = time(NULL) - conf.cooloff * 60;
		nvdata.magic = MAGIC;
		saveNvData();
	}
	// Clear states that shouldn't really be kept between boots
	// TODO: Split out non-NV states so we don't have to bother with this.
	nvdata.state |= STATE_MENU_SELECT;
	nvdata.state &= ~(STATE_GOT_TIME | STATE_RTC_PRESENT | STATE_WEIGAND_DONE | STATE_GOT_IP_ADDR |
	  STATE_BOOTUP_NTFY | STATE_BLE_CONNECTED | STATE_BLE_DATA);
}

void
saveNvData(void)
{
	FastCRC16	 CRC16;
	uint32_t	 clock;

	if (haveFRAM) {
		if ((clock = Wire.getClock()) > 0)
			Wire.setClock(1000000);
		nvdata.crc = CRC16.ccitt((uint8_t *)&nvdata, sizeof(nvdata) - 2);
		fram.write(OFFSET_NVDATA, (uint8_t *)&nvdata, sizeof(nvdata));
		if (clock > 0)
			Wire.setClock(clock);
	}
}

void
saveSettings(void)
{
	FastCRC16	 CRC16;
	uint32_t	 clock;

	conf.crc = CRC16.ccitt((uint8_t *)&conf, sizeof(cfg) - 2);
	if (haveFRAM) {
		if ((clock = Wire.getClock()) > 0)
			Wire.setClock(1000000);
		fram.write(OFFSET_CONFIG, (uint8_t *)&conf, sizeof(conf));
		if (clock > 0)
			Wire.setClock(clock);
	}
	else {
		unsigned char  *p;
		p = reinterpret_cast<unsigned char *>(&conf);
		for (uint16_t i = 0; i < sizeof(struct cfg); i++)
			EEPROM.write(OFFSET_CONFIG + i, *p++);
		EEPROM.commit();
	}
}

void
defaultSettings(void)
{
	uint8_t		 mac[6];
	uint32_t	 flags;

	// Do not clear calibration data and Engineering configuration unless MAGIC is wrong
	if (conf.magic != MAGIC) {
		conf.offset = SCALE_OFFSET;
		conf.scale = SCALE_FACTOR;
		conf.flags |= CFG_HX711_FAST | CFG_HX711_GAIN_HIGH;
	}
	flags = conf.flags & (CFG_HX711_FAST | CFG_HX711_GAIN_HIGH | CFG_DOOR_MOTOR_SLOW | CFG_ENGINEERING);
	memset(&conf.hostname, '\0', sizeof(struct cfg) - (offsetof(struct cfg, hostname) - offsetof(struct cfg, magic)));
	conf.flags = flags;
	conf.BLEtimeout = 3;
	WiFi.macAddress(mac);
	snprintf(conf.hostname, 32, "%02X%02X%02X", mac[3], mac[4], mac[5]);
	strcpy(conf.cacrt, ca_pem_start);
	strcpy(conf.snmpro, "public");
	strcpy(conf.snmprw, "private");
	strcpy(conf.timezone, "EST5EDT,M3.2.0,M11.1.0");
	strcpy(conf.ntpserver, "pool.ntp.org");
	conf.magic = MAGIC;
}

void
initConfig(void)
{
	FastCRC16	 CRC16;
	uint32_t	 clock;

	if (haveFRAM) {
		if ((clock = Wire.getClock()) > 0)
			Wire.setClock(1000000);
		fram.read(OFFSET_CONFIG, (uint8_t *)&conf, sizeof(conf));
		if (clock > 0)
			Wire.setClock(clock);
	}
	else {
		unsigned char  *p;

		p = reinterpret_cast<unsigned char *>(&conf);
		for (uint16_t i = 0; i < sizeof(struct cfg); i++)
			*p++ = EEPROM.read(OFFSET_CONFIG + i);
	}
	if (conf.magic != MAGIC || conf.crc != CRC16.ccitt((uint8_t *)&conf, sizeof(struct cfg) - 2)) {
		debug(true, "Settings corrupted, defaulting");
		if (conf.magic != MAGIC)
			defaultSettings();
		saveSettings();
	}
	scale.set_scale(conf.scale);
	scale.set_offset(conf.offset);
}

uint8_t
getNextAlarm(void)
{
	time_t		 t = time(NULL);
	struct tm	*tm = localtime(&t);
	int			 i;

	for (i = 0; i < CFG_NSCHEDULES; i++) {
		if (~conf.schedule[i].flags & CFG_SCHED_ENABLE)
			continue;
		if (tm->tm_hour * 60 + tm->tm_min < conf.schedule[i].hour * 60 + conf.schedule[i].minute)
			break;
	}

	if (i == CFG_NSCHEDULES)
		for (i = 0; i < CFG_NSCHEDULES && ~conf.schedule[i].flags & CFG_SCHED_ENABLE; i++);
	debug(true, "Schedule %d", i);
	return(i);
}

uint8_t
getCurrentAlarm(void)
{
	DateTime	 alarm1;
	int			 i;

	if (nvdata.state & STATE_NO_SCHEDULE)
		return(CFG_NSCHEDULES);

	alarm1 = rtc.getAlarm1() + getTz();
	for (i = 0; i < CFG_NSCHEDULES; i++) {
		if (~conf.schedule[i].flags & CFG_SCHED_ENABLE)
			continue;
		if (alarm1.hour() == conf.schedule[i].hour && alarm1.minute() == conf.schedule[i].minute)
			break;
	}
	return(i);
}

void
setAlarm(uint8_t alarm)
{
	if (alarm >= CFG_NSCHEDULES) {
		nvdata.state |= STATE_NO_SCHEDULE;
		rtc.disableAlarm(1);
		return;
	}
	nvdata.state &= ~STATE_NO_SCHEDULE;

	DateTime  alarm1(2000, 1, 1, conf.schedule[alarm].hour, conf.schedule[alarm].minute, 0);
	rtc.setAlarm1(alarm1 - getTz(), DS3231_A1_Hour);
}

TimeSpan
getTz(void)
{
	time_t		 t;
	struct tm	*tm;
	TimeSpan	 tz;

	t = time(NULL);
	tm = gmtime(&t);
	tz = t - mktime(tm) + (tm->tm_isdst ? 3600 : 0);
	return(tz);
}

const char *
getResetReason(void)
{
	const char	*reason;
	switch (esp_reset_reason()) {
	  case ESP_RST_UNKNOWN:    //!< Reset reason can not be determined
		reason = "Unkown";
		break;
	  case ESP_RST_POWERON:    //!< Reset due to power-on event
		reason = "Power-on";
		break;
	  case ESP_RST_EXT:        //!< Reset by external pin (not applicable for ESP32)
		reason = "External reset";
		break;
	  case ESP_RST_SW:         //!< Software reset via esp_restart
		reason = "Software reset";
		break;
	  case ESP_RST_PANIC:      //!< Software reset due to exception/panic
		reason = "Panic";
		break;
	  case ESP_RST_INT_WDT:    //!< Reset (software or hardware) due to interrupt watchdog
		reason = "Interrupt watchdog";
		break;
	  case ESP_RST_TASK_WDT:   //!< Reset due to task watchdog
		reason = "Task watchdog";
		break;
	  case ESP_RST_WDT:        //!< Reset due to other watchdogs
		reason = "Watchdog";
		break;
	  case ESP_RST_DEEPSLEEP:  //!< Reset after exiting deep sleep mode
		reason = "Deep sleep";
		break;
	  case ESP_RST_BROWNOUT:   //!< Brownout reset (software or hardware)
		reason = "Brownout";
		break;
	  case ESP_RST_SDIO:       //!< Reset over SDIO
		reason = "SDIO";
		break;
	}
	return(reason);
}

int
comparSchedule(const void *a, const void *b)
{
	return(((const struct schedule *)a)->hour * 60 + ((const struct schedule *)a)->minute - ((const struct schedule *)b)->hour * 60 - ((const struct schedule *)b)->minute);
}

void
debug(bool logtime, const char *format, ...)
{
	va_list    pvar;
	static char       str[128];
	static char       timestr[20];
	time_t     t = time(NULL);
	struct tm *tm = localtime(&t);
	
	if (logtime && nvdata.state & STATE_GOT_TIME) {
		strftime(timestr, 20, "%F %T", tm);
		Serial.print(timestr);
		Serial.print(": "); 
	}
	va_start(pvar, format);
	vsnprintf(str, 125, format, pvar);
	va_end(pvar);
	Serial.println(str);
	Serial.flush();
	if (nvdata.state & STATE_BLE_CONNECTED) {
		strcat(str, "\r\n");
		pTxCharacteristic->setValue((uint8_t *)str, strlen(str));
		pTxCharacteristic->notify();
	}
}

// tags: https://docs.ntfy.sh/emojis/
// priority: https://docs.ntfy.sh/publish/#message-priority
void
ntfy(const char *title, const char *tags, const uint8_t priority, const char *format, ...)
{
	esp_err_t	 err;
	va_list		 pvar;
	char		*buffer, *message;
	int			 content_length, wlen;

	if (~conf.flags & CFG_NTFY_ENABLE)
		return;

	if ((buffer = (char *)malloc(3072)) == NULL) {
		debug(true, "NTFY failed to allocate memory");
		return;
	}
	if ((message = (char *)malloc(2048)) == NULL) {
		debug(true, "NTFY failed to allocate memory");
		free(buffer);
		return;
	}
	va_start(pvar, format);
	vsnprintf(message, 2048, format, pvar);
	va_end(pvar);

	esp_http_client_config_t config = {
	  .url = conf.ntfy.url,
	  .username = conf.flags & CFG_NTFY_AUTH ? conf.ntfy.username : "",
	  .password = conf.flags & CFG_NTFY_AUTH ?conf.ntfy.password : "",
	  .auth_type = conf.flags & CFG_NTFY_AUTH ? HTTP_AUTH_TYPE_BASIC : HTTP_AUTH_TYPE_NONE,
	  .cert_pem = conf.cacrt
	};
	esp_http_client_handle_t client = esp_http_client_init(&config);
	esp_http_client_set_method(client, HTTP_METHOD_POST);
	esp_http_client_set_header(client, "Content-Type", "application/json");

	const char *post_data = "{"
	  "\"topic\":\"%s\","
	  "\"title\":\"%s\","
	  "\"tags\":[\"%s\"],"
	  "\"priority\":%d,"
	  "\"message\":\"%s\""
	"}";
	content_length = snprintf(buffer, 3072, post_data, conf.ntfy.topic, title, tags, priority, message);
	err = esp_http_client_open(client, content_length);
	if (err != ESP_OK) {
		debug(true, "NTFY connection failed: %s", esp_err_to_name(err));
		esp_http_client_cleanup(client);
		free(message);
		free(buffer);
		return;
	} 
	wlen = esp_http_client_write(client, buffer, content_length);
	if (wlen < 0) {
		debug(true, "Write failed");
	}
	content_length = esp_http_client_fetch_headers(client);
	if (content_length < 0) {
		debug(true, "HTTP client fetch headers failed");
	} else {
		int data_read = esp_http_client_read_response(client, buffer, 2048);
		if (data_read >= 0) {
			debug(true, "HTTP POST Status = %d, content_length = %ld",
			esp_http_client_get_status_code(client),
			esp_http_client_get_content_length(client));
			debug(true, buffer, strlen(buffer));
		} else {
			debug(true, "Failed to read response");
		}
	}
	free(message);
	free(buffer);
	esp_http_client_cleanup(client);
}

/*
 * Web server parts
 */

void
wshandleRoot(void) {
	char		*body;
	char		 timestr[20];
	int			 sec = time(NULL) - bootTime;
	int			 min = sec / 60;
	int			 hr = min / 60;
	char		 alarm1Date[9] = "hh:mm";
	time_t		 t = time(NULL);
	struct tm	*tm = localtime(&t);

	strftime(timestr, 20, "%F %T", tm);

	if ((body = (char *)malloc(2048)) == NULL) {
		debug(true, "WEB / failed to allocate memory");
		return;
	}
	if (~nvdata.state & STATE_NO_SCHEDULE) {
		DateTime alarm1 = rtc.getAlarm1() + getTz();
		alarm1.toString(alarm1Date);
	}
	snprintf(body, 2048,
	  "<!doctype html>"
	  "<html lang='en'>"
	  "<head>"
	  "<meta charset='UTF-8'>"
	  "<title>Feeder [%s]</title>"
	  "<script src='dygraph.min.js'></script>"
	  "<link rel='stylesheet' type='text/css' href='dygraph.css'>"
	  "<style>"
	    "body {background-color: #cccccc; font-family: Arial, Helvetica, Sans-Serif; Color: #000088;}"
		".dygraph-legend {text-align: right;background: none;}</style>"
	  "</head>"
	  "<body>"
	  "<h1>Feeder %s</h1>"
	  "%s<br>"
	  "%s"
	  "<p>Next dispense: %s</p>"
	  "<p>Weight: %3.0f g</p>"
	  "<p>Dispensed: %3.0f of %3d g</p>"
	  "<p><a href='/config'>System Configuration</a></p>"
	  "<p><a href='/schedule'>Feed Dispensing Schedule</a></p>"
	  "<form method='post' action='/feed' name='Feed'>"
		"Dispense grams:&nbsp;<input name='weight' type='number' size='4' value='%d' min='1' max='25'>"
		"<input name='Feed' type='submit' value='Feed'>"
	  "</form>"
	  "<div id='history' style='width:600px; height:300px;'></div>"
	  "<p><font size=1>"
	  "<a href='/tare'>Zero the scale</a><br>"
	  "Uptime: %d days %02d:%02d:%02d<br>"
	  "Firmware: " __DATE__ " " __TIME__ "<br>"
	  "</font>"
	  "<script>Dygraph.onDOMready(function onDOMready() {"
		"new Dygraph(document.getElementById('history'), 'data.txt', "
		"{title: 'Feeding history', legend: 'always', ylabel: 'Weight', showRangeSelector: true});"
	  "});"
	  "</script>"
	  "</body>"
	  "</html>",
	  conf.hostname, conf.hostname,
	  timestr,
	  nvdata.state & STATE_CHECK_HOPPER ? "<p style='color: #AA0000;'>Check Hopper</p>" : "",
	  ~nvdata.state & STATE_NO_SCHEDULE ? alarm1Date : "No Schedule",
	  weigh(true), nvdata.dispensedTotal, conf.quota,
	  conf.perFeed,
	  sec / 86400, hr % 24, min % 60, sec % 60
	);
	webserver.send(200, "text/html", body);
	free(body);
}
		//"{title: 'Feeding history', ylabel: 'Temperature (F)', showRangeSelector: true});"

void
wshandleEngineering(void) {
	char		*body;

	if ((body = (char *)malloc(2048)) == NULL) {
		debug(true, "WEB / failed to allocate memory");
		return;
	}
	snprintf(body, 2048,
	  "<!doctype html>"
	  "<html>"
	  "<head>"
	  "<title>Feeder [%s]</title>"
	  "<style>body { background-color: #cccccc; font-family: Arial, Helvetica, Sans-Serif; Color: #000088; }</style>"
	  "</head>"
	  "<body>"
	  "<h1>Feeder %s</h1>"
	  "<p><a href='/calibrate'>Calibrate the scale</a></p>"
	  "<p><a href='/ssl'>CA Certificate</a></p>"
	  "<form method='post' action='/doengineering' name='save'>\n"
	  "<table border=0 width='520' cellspacing=4 cellpadding=0>\n"
	  "<tr><td width='40%%'>Advanced:</td><td><input name='advanced' type='checkbox' value='true' %s></td>"
	  "<tr><td width='40%%'>HX711 fast sample rate:</td><td><input name='hx711fast' type='checkbox' value='true' %s></td>"
	  "<tr><td width='40%%'>HX711 high gain:</td><td><input name='hx711high' type='checkbox' value='true' %s></td></tr>"
	  "<tr><td width='40%%'>Door stepper slow timing:</td><td><input name='door12v' type='checkbox' value='true' %s></td></tr>"
	  "<tr><td width='40%%'>Scale offset:</td><td><input name='offset' type='number' value='%ld'></td></tr>"
	  "<tr><td width='40%%'>Scale factor:</td><td><input name='factor' type='number' value='%f'></td></tr>"
	  "<tr><td width='40%%'>Dispensed total:</td><td><input name='weight' type='number' value='%5.2f' min='0' max='100' step='0.01'></td></tr>"
	  "</table>"
	  "<input name='Save' type='submit' value='Save'>\n"
	  "</form>"
	  "<p>Config Size: %d<br>"
	  "</body>"
	  "</html>",
	  conf.hostname, conf.hostname,
	  conf.flags & CFG_ENGINEERING ? "checked" : "",
	  conf.flags & CFG_HX711_FAST ? "checked" : "",
	  conf.flags & CFG_HX711_GAIN_HIGH ? "checked" : "",
	  conf.flags & CFG_DOOR_MOTOR_SLOW ? "checked" : "",
	  conf.offset,
	  conf.scale,
	  nvdata.dispensedTotal,
	  sizeof(struct cfg)
	);
	webserver.send(200, "text/html", body);
	free(body);
}

void
wshandleSSL()
{
	char		*body;

	if ((body = (char *)malloc(MAX_CERT+450)) == NULL) {
		debug(true, "WEB / failed to allocate memory");
		return;
	}
	// 437 excluding certificate text
	snprintf(body, MAX_CERT+450,
	  "<!doctype html>"
	  "<html>"
	  "<head>"
	  "<title>Feeder [%s]</title>"
	  "<style>body { background-color: #cccccc; font-family: Arial, Helvetica, Sans-Serif; Color: #000088; }</style>"
	  "</head>"
	  "<body>"
	  "<h1>Feeder %s</h1>"
	  "<form method='post' action='/sslsave' name='save'>\n"
	  "<label for='cacrt'>CA Certificate:</label><p><textarea name='cacrt' type='textarea' maxlength=%d rows=20 cols=80>%s</textarea><br>"
	  "<input name='Save' type='submit' value='Save'>\n"
	  "</form>"
	  "</body>"
	  "</html>",
	  conf.hostname, conf.hostname, MAX_CERT, conf.cacrt
	);

	webserver.send(200, "text/html", body);
	free(body);
}

void
wshandleReboot()
{
	char	*body;

	if ((body = (char *)malloc(400)) == NULL) {
		debug(true, "WEB / failed to allocate memory");
		return;
	}
	snprintf(body, 400,
	  "<!doctype html>"
	  "<html>\n"
	  "<head>\n"
	  "<title>Feeder [%s]</title>\n"
	  "<style>body { background-color: #cccccc; font-family: Arial, Helvetica, Sans-Serif; Color: #000088; }</style>\n"
	  "</head>\n"
	  "<body>\n"
	  "<h1>Feeder %s</h1>"
	  "Rebooting<BR>"
	  "<meta http-equiv='Refresh' content='5; url=/'>"
	  "</body>\n"
	  "</html>", conf.hostname, conf.hostname);
	  
	webserver.send(200, "text/html", body);
	free(body);
	delay(100);
	NimBLEDevice::deinit();
	ESP.restart();
}

void 
wshandle404(void)
{
	String	 message = "File Not Found\n\n";

	message += "URI: ";
	message += webserver.uri();
	message += "\nMethod: ";
	message += (webserver.method() == HTTP_GET) ? "GET" : "POST";
	message += "\nArguments: ";
	message += webserver.args();
	message += "\n";

	for (uint8_t i = 0; i <= 254 & i < webserver.args(); i++) {
		message += " " + webserver.argName(i) + ": " + webserver.arg(i) + "\n";
	}

	webserver.send(404, "text/plain", message);
}

void
wshandleFavIcon(void)
{
	webserver.send_P(200, "image/x-icon", favicon_start, favicon_end - favicon_start);
}

void
wshandleDygraphJS(void)
{
	webserver.send_P(200, "application/javascript", graph_start);
}

void
wshandleDygraphCSS(void)
{
	webserver.send_P(200, "text/css", css_start);
}

void
wshandleGraphData(void)
{
	char	*body, *p;
	char	 timestr[13];
	int		 i, len;
	time_t	 td, t = time(NULL);
	struct tm	*tm;

	if ((body = (char *)malloc(8800)) == NULL) {
		debug(true, "WEB / failed to allocate 8800 bytes");
		return;
	}

	p = body;
	len = sprintf(p, "%s, %4s\n", "Date", "Weight");
	p += len;
	for (i = HISTORY_NDAYS - 1 ; i >= 0; i--) {
		if (i && history.day[i].start == 0 && history.day[i].dispensed == 0 && history.day[i].end == 0)
			continue;
		td = t - i * 86400;
		tm = localtime(&td);
		strftime(timestr, 20, "%F", tm);

		len = sprintf(p, "%s, %5.1f\n", timestr,
		  history.day[i].start + (i == 0 ? nvdata.dispensedTotal : history.day[i].dispensed) -
		  (i == 0 ? weigh(true) : history.day[i].end));
		p += len;
	}

	webserver.send_P(200, "text/plain", body);
	free(body);
}

void
wshandleConfig(void)
{
	char	*body, *temp;

	// 8736 characters max body size.
	if ((body = (char *)malloc(8800)) == NULL) {
		debug(true, "WEB / failed to allocate 8800 bytes");
		return;
	}
	if ((temp = (char *)malloc(690)) == NULL) {
		debug(true, "WEB / failed to allocate 690 bytes");
		free(body);
		return;
	}
	
	// max 3662 characters.
	snprintf(body, 3670,
		"<!doctype html>"
		"<html>\n"
		"<head>\n"
		"<title>Feeder [%s]</title>\n"
		"<style>body { background-color: #cccccc; font-family: Arial, Helvetica, Sans-Serif; Color: #000088; }</style>\n"
		"</head>\n"
		"<body>\n"
		"<h1>Feeder %s</h1>"
		"<form method='post' action='/save' name='Configuration'>\n"
		  "<table border=0 width='520' cellspacing=4 cellpadding=0>\n"
		  "<tr><td width='30%%'>Feeder Name:</td><td><input name='name' type='text' value='%s' size='32' maxlength='32' "
			"pattern='^[A-Za-z0-9_-]+$' title='Letters, numbers, _ and -'></td></tr>\n"
		  "<tr><td width='30%%'>SSID:</td><td><input name='ssid' type='text' value='%s' size='32' maxlength='63'></td></tr>\n"
		  "<tr><td width='30%%'>WPA Key:</td><td><input name='key' type='text' value='%s' size='32' maxlength='63'></td></tr>\n"
		  "<tr><td width='30%%'>BLE enable time:</td><td><input name='ble' type='number' value='%d' size='4' min='0' max='255'>Minutes</td></tr>\n"
		  "<tr><td width='30%%'>NTP Server:</td><td><input name='ntp' type='text' value='%s' size='32' maxlength='63' "
			"pattern='^([a-z0-9]+)(\\.)([_a-z0-9]+)((\\.)([_a-z0-9]+))?$' title='A valid hostname'></td></tr>\n"
		  "<tr><td width='30%%'>Time Zone spec:</td><td><input name='tz' type='text' value='%s' size='32' maxlength='32'></td></tr>\n"
		  "<tr><td>&nbsp</td><td>&nbsp</td></tr>"
		  "<tr><td width='30%%'>Daily quota:</td><td><input name='quota' type='number' size='4' value='%d' min='10' max='120'>grams</td></tr>\n"
		  "<tr><td width='30%%'>Per feed:</td><td><input name='perfeed' type='number' size='4' value='%d' min='1' max='25'>grams</td></tr>\n"
		  "<tr><td width='30%%'>Close to feed:</td><td><input name='ctf' type='checkbox' value='true' %s></td></tr>\n"
		  "<tr><td width='30%%'>Feed cooloff:</td><td><input name='cooloff' type='number' size='4' value='%d' min='0' max='120'> Minutes</td></tr>\n"
		  "<tr><td>&nbsp</td><td>&nbsp</td></tr>"
		  "<tr><td width='30%%'>Notifications:</td><td><input name='ntfy' type='checkbox' value='true' %s></td></tr>\n"
		  "<tr><td width='30%%'>Events:</td><td>"
			"<table border=0 cellspacing=0 cellpadding=0>\n"
		  	"<tr><td><input name='ntfy-prob' type='checkbox' value='true' %s>Problems</td></tr>"
		  	"<tr><td><input name='ntfy-disp' type='checkbox' value='true' %s>Dispense</td></tr>"
		  	"<tr><td><input name='ntfy-vist' type='checkbox' value='true' %s>Visits</td></tr>"
		  	"<tr><td><input name='ntfy-ntrd' type='checkbox' value='true' %s>Intruder</td></tr>"
			"</table>"
		  	"</td></tr>\n"
		  "<tr><td width='30%%'>Service URL:</td><td><input name='url' type='url' value='%s' size='32' maxlength='63'></td></tr>\n"
		  "<tr><td width='30%%'>Topic:</ltd><td><input name='topic' type='text' value='%s' size='32' maxlength='63'></td></tr>\n"
		  "<tr><td width='30%%'>NTFY login:</td><td><input name='ntfy-auth' type='checkbox' value='true' %s></td></tr>\n"
		  "<tr><td width='30%%'>Username:</td><td><input name='user' type='text' value='%s' size='15' maxlength='15'></td></tr>\n"
		  "<tr><td width='30%%'>Password:</td><td><input name='passwd' type='text' value='%s' size='15' maxlength='15'></td></tr>\n"
		  "<tr><td>&nbsp</td><td>&nbsp</td></tr>"
		  "<tr><td width='30%%'>SNMP:</td><td><input name='snmp' type='checkbox' value='true' %s></td></tr>\n"
		  "<tr><td width='30%%'>RO Community:</td><td><input name='snmpro' type='text' value='%s' size='32' maxlength='63'></td></tr>\n"
		  "<tr><td width='30%%'>RW Community:</td><td><input name='snmprw' type='text' value='%s' size='32' maxlength='63'></td></tr>\n"
		  "</table><p>\n",
		  conf.hostname, conf.hostname, conf.hostname, conf.ssid, conf.wpakey, conf.BLEtimeout, conf.ntpserver, conf.timezone, conf.quota, conf.perFeed,
		  conf.flags & CFG_DISPENSE_CLOSED ? "checked" : "", conf.cooloff,
		  conf.flags & CFG_NTFY_ENABLE ? "checked" : "", 
		  conf.flags & CFG_NTFY_PROBLEM ? "checked" : "", 
		  conf.flags & CFG_NTFY_DISPENSE ? "checked" : "", 
		  conf.flags & CFG_NTFY_VISIT ? "checked" : "", 
		  conf.flags & CFG_NTFY_INTRUDE ? "checked" : "", 
		  conf.ntfy.url, conf.ntfy.topic,
		  conf.flags & CFG_NTFY_AUTH ? "checked" : "",
		  conf.ntfy.username, conf.ntfy.password,
		  conf.flags & CFG_SNMP_ENABLE ? "checked" : "", conf.snmpro, conf.snmprw);

	// max 681 characters per cat
	for (int i = 0; i < CFG_NCATS; i++) {
		snprintf(temp, 688,
		  "<table border=0 width='520' cellspacing=4 cellpadding=0>\n"
		  "<tr><td width='30%%'><b>Cat %d:</b></td><td><input name='catname%d' type='text' value='%s' size='19' maxlength='19'></td></tr>\n"
		  "<tr><td width='30%%'>Facility Code:</td><td><input name='facility%d' type='number' size='4' value='%d' min='0' max='255'></td></tr>\n"
		  "<tr><td width='30%%'>Tag ID:</td><td><input name='id%d' type='number' size='4' value='%d' min='0' max='8191'></td></tr>\n"
		  "<tr><td width='30%%'>Access:</td><td><input name='access%d' type='checkbox' value='true' %s></td></tr>\n"
		  "<tr><td width='30%%'>Dispense:</td><td><input name='dispense%d' type='checkbox' value='true' %s></td></tr>\n"
		  "</table><p>\n",
		  i + 1, i, conf.cat[i].name, i, conf.cat[i].facility, i, conf.cat[i].id, i, conf.cat[i].flags & CFG_CAT_ACCESS ? "checked" : "",
		  i, conf.cat[i].flags & CFG_CAT_DISPENSE ? "checked" : ""
		);
		strcat(body, temp);
	}
	// 239 characters
	snprintf(temp, 250,
	  "<input name='Save' type='submit' value='Save'>\n"
	  "</form>"
	  "<br>"
	  "<form method='post' action='/reboot' name='Reboot'>\n"
		"<input name='Reboot' type='submit' value='Reboot'>\n"
	  "</form>\n"
	  "%s"
	  "</body>\n"
	  "</html>",
	  conf.flags & CFG_ENGINEERING ? "<p><a href='/engineering'>Engineering options</a></p>" : ""
	  );
	strcat(body, temp);
	free(temp);
	webserver.send_P(200, "text/html", body);
	free(body);
}

void
wshandleSave(void)
{
	char	*temp, hostname[28];
	int		 wifiChanged = 0;
	String	 value;

	if ((temp = (char *)malloc(400)) == NULL)
		return;
	if (webserver.hasArg("ssid")) {
		value = webserver.arg("ssid");
		if (strncmp(value.c_str(), conf.ssid, 63)) {
			strncpy(conf.ssid, value.c_str(), 63);
			wifiChanged = 1;
		}
	}

	if (webserver.hasArg("key")) {
		value = webserver.arg("key");
		if (strncmp(value.c_str(), conf.wpakey, 63)) {
			strncpy(conf.wpakey, value.c_str(), 63);
			wifiChanged = 1;
		}
	}

	value = webserver.arg("ble");
	if (value.length() && value.toInt() >= 0 && value.toInt() <= 255)
		conf.BLEtimeout = value.toInt();

	if (webserver.hasArg("name")) {
		value = webserver.arg("name");
		strncpy(conf.hostname, value.c_str(), 32);
	}

	if (webserver.hasArg("ntp")) {
		value = webserver.arg("ntp");
		strncpy(conf.ntpserver, value.c_str(), 32);
	}

	if (webserver.hasArg("tz")) {
		value = webserver.arg("tz");
		strncpy(conf.timezone, value.c_str(), 32);
	}

	value = webserver.arg("quota");
	if (value.length() && value.toInt() >= 10 && value.toInt() <= 120)
		conf.quota = value.toInt();

	value = webserver.arg("perfeed");
	if (value.length() && value.toInt() >= 1 && value.toInt() <= 25)
		conf.perFeed = value.toInt();

	if (webserver.hasArg("ctf"))
		conf.flags |= CFG_DISPENSE_CLOSED;
	else
		conf.flags &= ~CFG_DISPENSE_CLOSED;

	value = webserver.arg("cooloff");
	if (value.length() && value.toInt() >= 0 && value.toInt() <= 120)
		conf.cooloff = value.toInt();

	if (webserver.hasArg("ntfy"))
		conf.flags |= CFG_NTFY_ENABLE;
	else
		conf.flags &= ~CFG_NTFY_ENABLE;

	if (webserver.hasArg("ntfy-auth"))
		conf.flags |= CFG_NTFY_AUTH;
	else
		conf.flags &= ~CFG_NTFY_AUTH;

	if (webserver.hasArg("ntfy-prob"))
		conf.flags |= CFG_NTFY_PROBLEM;
	else
		conf.flags &= ~CFG_NTFY_PROBLEM;

	if (webserver.hasArg("ntfy-disp"))
		conf.flags |= CFG_NTFY_DISPENSE;
	else
		conf.flags &= ~CFG_NTFY_DISPENSE;

	if (webserver.hasArg("ntfy-ntrd"))
		conf.flags |= CFG_NTFY_INTRUDE;
	else
		conf.flags &= ~CFG_NTFY_INTRUDE;

	if (webserver.hasArg("ntfy-vist"))
		conf.flags |= CFG_NTFY_VISIT;
	else
		conf.flags &= ~CFG_NTFY_VISIT;

	if (webserver.hasArg("url")) {
		value = webserver.arg("url");
		strncpy(conf.ntfy.url, value.c_str(), 64);
	}

	if (webserver.hasArg("topic")) {
		value = webserver.arg("topic");
		strncpy(conf.ntfy.topic, value.c_str(), 64);
	}

	if (webserver.hasArg("user")) {
		value = webserver.arg("user");
		strncpy(conf.ntfy.username, value.c_str(), 16);
	}

	if (webserver.hasArg("passwd")) {
		value = webserver.arg("passwd");
		strncpy(conf.ntfy.password, value.c_str(), 16);
	}

	if (webserver.hasArg("snmp"))
		conf.flags |= CFG_SNMP_ENABLE;
	else
		conf.flags &= ~CFG_SNMP_ENABLE;

	if (webserver.hasArg("snmpro")) {
		value = webserver.arg("snmpro");
		strncpy(conf.snmpro, value.c_str(), 64);
		snmp.setReadOnlyCommunity(conf.snmpro);
	}

	if (webserver.hasArg("snmprw")) {
		value = webserver.arg("snmprw");
		strncpy(conf.snmprw, value.c_str(), 64);
		snmp.setReadWriteCommunity(conf.snmprw);
	}

	for (int i=0; i < CFG_NCATS; i++) {
		snprintf(temp, 399, "catname%d", i);
		if (webserver.hasArg(temp)) {
			value = webserver.urlDecode(webserver.arg(temp));
			strncpy(conf.cat[i].name, value.c_str(), 20);
		}

		snprintf(temp, 399, "facility%d", i);
		value = webserver.arg(temp);
		if (value.length() && value.toInt() >= 0 && value.toInt() <= 255)
			conf.cat[i].facility = value.toInt();

		snprintf(temp, 399, "id%d", i);
		value = webserver.arg(temp);
		if (value.length() && value.toInt() >= 0 && value.toInt() <= 8191)
			conf.cat[i].id = value.toInt();

		snprintf(temp, 399, "access%d", i);
		if (webserver.hasArg(temp))
			conf.cat[i].flags |= CFG_CAT_ACCESS;
		else
			conf.cat[i].flags &= ~CFG_CAT_ACCESS;

		snprintf(temp, 399, "dispense%d", i);
		if (webserver.hasArg(temp))
			conf.cat[i].flags |= CFG_CAT_DISPENSE;
		else
			conf.cat[i].flags &= ~CFG_CAT_DISPENSE;
	}

	snprintf(hostname, 28, "Feeder-%s", conf.hostname);
	WiFi.hostname(hostname);
	if (wifiChanged && *conf.ssid && *conf.wpakey)
		WiFi.begin(conf.ssid, conf.wpakey);
	MDNS.setInstanceName(hostname);
	configTzTime(conf.timezone, conf.ntpserver);
	
	snprintf(temp, 400,
	  "<html>\n"
	  "<head>\n"
	  "<title>Feeder [%s]</title>\n"
	  "<style>body { background-color: #cccccc; font-family: Arial, Helvetica, Sans-Serif; Color: #000088; }</style>\n"
	  "</head>\n"
	  "<body>\n"
	  "<h1>Feeder %s</h1>"
	  "Updated conguration, %d items<BR>"
	  "<meta http-equiv='Refresh' content='3; url=/'>"
	  "</body>\n"
	  "</html>", conf.hostname, conf.hostname, webserver.args());
	  
	webserver.send(200, "text/html", temp);
	free(temp);
	saveSettings();
}

void
wshandleSaveSSL()
{
	String		 value;
	char		*body;

	if ((body = (char *)malloc(400)) == NULL) {
		debug(true, "WEB / failed to allocate memory");
		return;
	}
	if (webserver.hasArg("cacrt")) {
		value = webserver.arg("cacrt");
		strncpy(conf.cacrt, value.c_str(), 6144);
	}

	snprintf(body, 400,
	  "<html>\n"
	  "<head>\n"
	  "<title>Feeder [%s]</title>\n"
	  "<style>body { background-color: #cccccc; font-family: Arial, Helvetica, Sans-Serif; Color: #000088; }</style>\n"
	  "</head>\n"
	  "<body>\n"
	  "<h1>Feeder %s</h1>"
	  "Updated conguration, %d items<BR>"
	  "<meta http-equiv='Refresh' content='3; url=/'>"
	  "</body>\n"
	  "</html>", conf.hostname, conf.hostname, webserver.args());

	webserver.send(200, "text/html", body);
	free(body);
	saveSettings();
}

void
wshandleDoFeed()
{
	char	*body;
	int		 weight = 0;
	String	 value;

	if ((body = (char *)malloc(400)) == NULL)
		return;
	value = webserver.arg("weight");
	if (value.length()&& value.toInt() > 0 && value.toInt() < 60)
		weight = value.toInt();

	snprintf(body, 400,
	  "<html>\n"
	  "<head>\n"
	  "<title>Feeder [%s]</title>\n"
	  "<style>body { background-color: #cccccc; font-family: Arial, Helvetica, Sans-Serif; Color: #000088; }</style>\n"
	  "</head>\n"
	  "<body>\n"
	  "<h1>Feeder %s</h1>"
	  "Dispensing %d g"
	  "<meta http-equiv='Refresh' content='3; url=/'>"
	  "</body>\n"
	  "</html>", conf.hostname, conf.hostname, weight);
	  
	webserver.send(200, "text/html", body);
	free(body);
	
	if (weight) {
		float dispensed = dispense(weight);
		nvdata.dispensedTotal += dispensed;
		saveNvData();
		ntfy(WiFi.getHostname(), "desktop_computer", 3, "Web-dispense: %3.0fg of %dg\\nDispensed total: %3.0fg of %dg", dispensed, weight, nvdata.dispensedTotal, conf.quota);
	}
}

void
wshandleCalibrate(void)
{
	char	*body;

	if ((body = (char *)malloc(540)) == NULL)
		return;

	scale.set_scale();
	scale.tare(100);
	snprintf(body, 540,
	  "<html>\n"
	  "<head>\n"
	  "<title>Feeder [%s]</title>\n"
	  "<style>body { background-color: #cccccc; font-family: Arial, Helvetica, Sans-Serif; Color: #000088; }</style>\n"
	  "</head>\n"
	  "<body>\n"
	  "<h1>Feeder %s</h1>"
	  "Place a known weight on the empty bowl and enter the weight weight below<br><br>"
	  "<form method='post' action='/docalibrate' name='Calibrate'/>\n"
		"<label for=weight'>Weight:</label><input name='weight' type='number' value='100' min='100' max='2000'><br><br>\n"
	  "<input name='Save' type='submit' value='Calibrate'>\n"
	  "</form>"
	  "</body>\n"
	  "</html>", conf.hostname, conf.hostname);
	  
	webserver.send(200, "text/html", body);
	free(body);
}

void
wshandleDoCalibrate(void)
{
	char	*body;
	String	 value;

	if ((body = (char *)malloc(400)) == NULL)
		return;

	value = webserver.arg("weight");
	if (value.length() && value.toInt() >= 1 && value.toInt() <= 2000) {
		scale.set_scale();
		conf.scale = scale.get_units(100) / value.toInt();
		scale.set_scale(conf.scale);
		scale.set_offset(conf.offset);
		saveSettings();
	}

	snprintf(body, 400,
	  "<html>\n"
	  "<head>\n"
	  "<title>Feeder [%s]</title>\n"
	  "<style>body { background-color: #cccccc; font-family: Arial, Helvetica, Sans-Serif; Color: #000088; }</style>\n"
	  "</head>\n"
	  "<body>\n"
	  "<h1>Feeder %s</h1>"
	  "Calibration complete %f\n"
	  "<meta http-equiv='Refresh' content='3; url=/'>"
	  "</body>\n"
	  "</html>", conf.hostname, conf.hostname, conf.scale);

	webserver.send(200, "text/html", body);
	free(body);
}

void
wshandleTare(void)
{
	char	*body;

	if ((body = (char *)malloc(400)) == NULL)
		return;
	scale.tare(100);
	conf.offset = scale.get_offset();
	saveSettings();
	
	snprintf(body, 400,
	  "<html>\n"
	  "<head>\n"
	  "<title>Feeder [%s]</title>\n"
	  "<style>body { background-color: #cccccc; font-family: Arial, Helvetica, Sans-Serif; Color: #000088; }</style>\n"
	  "</head>\n"
	  "<body>\n"
	  "<h1>Feeder %s</h1>"
	  "Scale offset: %ld"
	  "<meta http-equiv='Refresh' content='3; url=/'>"
	  "</body>\n"
	  "</html>", conf.hostname, conf.hostname, conf.offset);
	  
	webserver.send(200, "text/html", body);
	free(body);
}

void
wshandleSchedule(void)
{
	char	*body, *temp;

	if ((body = (char *)malloc(5520)) == NULL)
		return;
	if ((temp = (char *)malloc(690)) == NULL) {
		free(body);
		return;
	}
	
	snprintf(body, 2000,
	  "<html>\n"
	  "<head>\n"
	  "<title>Feeder [%s]</title>\n"
	  "<style>body { background-color: #cccccc; font-family: Arial, Helvetica, Sans-Serif; Color: #000088; }</style>\n"
	  "</head>\n"
	  "<body>\n"
	  "<h1>Feeder %s</h1>"
	  "<form method='post' action='/schedulesave' name='Schedule'/>\n",
	  conf.hostname, conf.hostname);

	//for (int i = 0; i < CFG_NSCHEDULES; i++) {
	for (int i = 0; i < 10; i++) {
		snprintf(temp, 688,
		  "<table border=0 width='520' cellspacing=4 cellpadding=0>\n"
		  "<tr><td width='20%%'><b>Schedule %d:</b></td><td>Enable:<input name='e%d' type='checkbox' value='true' %s>"
		  "Allow skip:<input name='s%d' type='checkbox' value='true' %s><br></td></tr>\n"
		  "<tr><td width='20%%'>time:</td><td><input name='t%d' type='time' value='%02d:%02d'></td></tr>\n"
		  "<tr><td width='20%%'>Weight:</td><td><input name='w%d' type='number' value=%d size=3 min=0 max=25></td></tr>\n"
		  "</table><p>\n",
		  i + 1, i, conf.schedule[i].flags & CFG_SCHED_ENABLE ? "checked" : "",
		  i, conf.schedule[i].flags & CFG_SCHED_SKIP ? "checked" : "",
		  i, conf.schedule[i].hour, conf.schedule[i].minute,
		  i, conf.schedule[i].weight
		);
		strcat(body, temp);
	}
	strcat(body,
	  "<input name='Save' type='submit' value='Save'>\n"
	  "</form>"
	  "<br>"
	  "</body>\n"
	  "</html>");

	webserver.send(200, "text/html", body);
	free(body);
	free(temp);
}

void
wshandleScheduleSave(void)
{
	char	*temp;
	String	 value;

if ((temp = (char *)malloc(400)) == NULL)
	return;

	for (int i=0; i < CFG_NSCHEDULES; i++) {
		snprintf(temp, 399, "t%d", i);
		value = webserver.arg(temp);
		if (value.length()) {
		  int h, m;
		  if (sscanf(value.c_str(), "%d:%d", &h, &m) == 2 ) {
		    if (h >= 0 && h <= 23)
				conf.schedule[i].hour = h;
		    if (m >= 0 && m <= 59)
				conf.schedule[i].minute = m;
		  }
		}

		snprintf(temp, 399, "w%d", i);
		value = webserver.arg(temp);
		if (value.length() && value.toInt() >= 0 && value.toInt() <= 25)
			conf.schedule[i].weight = value.toInt();

		snprintf(temp, 399, "e%d", i);
		if (webserver.hasArg(temp))
			conf.schedule[i].flags |= CFG_SCHED_ENABLE;
		else
			conf.schedule[i].flags &= ~CFG_SCHED_ENABLE;

		snprintf(temp, 399, "s%d", i);
		if (webserver.hasArg(temp))
			conf.schedule[i].flags |= CFG_SCHED_SKIP;
		else
			conf.schedule[i].flags &= ~CFG_SCHED_SKIP;
	}

	snprintf(temp, 400,
	  "<html>\n"
	  "<head>\n"
	  "<title>Feeder [%s]</title>\n"
	  "<style>body { background-color: #cccccc; font-family: Arial, Helvetica, Sans-Serif; Color: #000088; }</style>\n"
	  "</head>\n"
	  "<body>\n"
	  "<h1>Feeder %s</h1>"
	  "Updated Schedule Conguration, %d items<BR>"
	  "<meta http-equiv='Refresh' content='3; url=/'>"
	  "</body>\n"
	  "</html>", conf.hostname, conf.hostname, webserver.args());

	webserver.send(200, "text/html", temp);
	free(temp);

	qsort(conf.schedule, CFG_NSCHEDULES, sizeof(struct schedule), comparSchedule);
	saveSettings();
	setAlarm(getNextAlarm());
}


void
wshandleDoEngineering(void)
{
	char	*body;
	String	 value;

	if ((body = (char *)malloc(400)) == NULL)
		return;

	if (webserver.hasArg("advanced"))
		conf.flags |= CFG_ENGINEERING;
	else
		conf.flags &= ~CFG_ENGINEERING;

	if (webserver.hasArg("hx711fast"))
		conf.flags |= CFG_HX711_FAST;
	else
		conf.flags &= ~CFG_HX711_FAST;

	if (webserver.hasArg("hx711high"))
		conf.flags |= CFG_HX711_GAIN_HIGH;
	else
		conf.flags &= ~CFG_HX711_GAIN_HIGH;

	if (webserver.hasArg("door12v"))
		conf.flags |= CFG_DOOR_MOTOR_SLOW;
	else
		conf.flags &= ~CFG_DOOR_MOTOR_SLOW;

	if (webserver.hasArg("offset")) {
		value = webserver.arg("offset");
		conf.offset = value.toInt();
	}

	if (webserver.hasArg("factor")) {
		value = webserver.arg("factor");
		conf.scale = value.toFloat();
	}

	if (webserver.hasArg("weight")) {
		value = webserver.arg("weight");
		if (value.toFloat() >= 0 && value.toFloat() <= 100) {
			nvdata.dispensedTotal = value.toFloat();
			saveNvData();
		}
	}

	snprintf(body, 400,
	  "<html>\n"
	  "<head>\n"
	  "<title>Feeder [%s]</title>\n"
	  "<style>body { background-color: #cccccc; font-family: Arial, Helvetica, Sans-Serif; Color: #000088; }</style>\n"
	  "</head>\n"
	  "<body>\n"
	  "<h1>Feeder %s</h1>"
	  "Updated Engineering Conguration, %d items<br>"
	  "<meta http-equiv='Refresh' content='3; url=/'>"
	  "</body>\n"
	  "</html>", conf.hostname, conf.hostname, webserver.args());

	webserver.send(200, "text/html", body);
	free(body);
	digitalWrite(PIN_HX711_RATE, conf.flags & CFG_HX711_FAST ? 1 : 0); // 0: 10Hz, 1: 80Hz
	scale.set_gain(conf.flags & CFG_HX711_GAIN_HIGH ? 128 : 64);
	scale.set_offset(conf.offset);
	scale.set_scale(conf.scale);
	saveSettings();
}
