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
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <FastCRC.h>
#include <HX711.h>
#include <esp_sntp.h>
#include <time.h>
#include <sys/time.h>
#include <stdarg.h>
#include <EEPROM.h>      // EEPROM emulation is deprecated.
#include <Preferences.h> // Convert settings to prefs at some point.

#define MAGIC 0xd41d8cd5
#define CFG_NCATS 7
struct cfg {
  uint32_t  magic;
  float     scale;
  long      offset;
  // end of callibration data. Do not add configuration items above.
  char      hostname[33];
  char      ssid[64];
  char      wpakey[64];
  uint8_t   logServer[4];
  char      ntpserver[64];
  struct {
     char     name[20];
     uint8_t  facility;
     uint16_t id;
     uint8_t  flags;
  } cat[CFG_NCATS];
  char      pad;
  uint8_t   perFeed;
  uint8_t   quota;
  uint16_t  crc;
} __attribute__((__packed__));

// Bitmap Configuration Flags
#define CFG_ACCESS    0x01
#define CFG_DISPENSE  0x02

#define SCALE_FACTOR 247.408408f
#define SCALE_OFFSET 62236

const char* ntpServer = "pool.ntp.org";
const int   gmtOffset_sec = -4 * 3600;
const int   daylightOffset_sec = 0 * 3600;

#define CLOSE_LIMIT 17
#define OPEN_LIMIT  5
#define LOGPORT     5001

#define PIN_AUGER_EN        13
#define PIN_AUGER_DIR       14
#define PIN_AUGER_STEP      12
#define PIN_AUGER_SLEEP     23
#define PIN_AUGER_MICROSTEP 25
#define AUGER_DIR_CW    1
#define AUGER_DIR_CCW   0

#define PIN_DOOR_EN         16
#define PIN_DOOR_DIR        15
#define PIN_DOOR_STEP       2
#define PIN_DOOR_SLEEP      18
#define PIN_DOOR_MICROSTEP  4
#define DOOR_DIR_CLOSE  0
#define DOOR_DIR_OPEN   1

#define PIN_DATA0       27
#define PIN_DATA1       26
#define MAX_BITS        100            // max number of bits 
#define WEIGAND_TIMEOUT 20     // timeout in ms on Wiegand sequence 
#define DOOR_TIMEOUT    60        // Door stays unlocked for max X seconds

// Bitmap States
#define STATE_OPEN          0x01
#define STATE_WEIGHTREPORT  0x02
#define STATE_3             0x04
#define STATE_4             0x08

volatile unsigned char  databits[MAX_BITS];    // stores all of the data bits
volatile unsigned char  bitCount;              // number of bits currently captured
volatile unsigned char  flagDone;              // goes low when data is currently being captured
volatile uint32_t       weigand_counter;       // countdown until we assume there are no more bits

uint8_t       state = 0;
time_t        closeTime = 0, lastFeed = 0;
float         dispensedTotal = 0;
struct cfg    conf;

void wshandleRoot(void);
void wshandleConfig(void);
void wshandle404(void);
void wshandleDoFeed(void);
void wshandleSave(void);
void wshandleReboot(void);
void wshandleCalibrate(void);
void wshandleDoCalibrate(void);
void wshandleTare(void);

int checkCard(uint8_t, uint16_t);
const char *catName(uint8_t, uint16_t);
float weigh(bool);
void debug(byte, const char *, ...);
void ntpCallBack(struct timeval *);
void wifiEvent(WiFiEvent_t);
float dispense(int);
void defaultSettings(void);
void saveSettings(void);
void configInit(void);
void closeDoor(void);
void openDoor(void);
void urlDecode(char *);
void shutdownDoor(void);
void shutdownAuger(void);
void enableDoor(void);
void enableAuger(void);

// interrupt that happens when INTO goes low (0 bit)
void IRAM_ATTR ISR_D0(void)
{
  bitCount++;
  flagDone = 0;
  weigand_counter = millis() + WEIGAND_TIMEOUT;
}

// interrupt that happens when INT1 goes low (1 bit)
void IRAM_ATTR ISR_D1(void)
{
  databits[bitCount++] = 1;
  flagDone = 0;
  weigand_counter = millis() + WEIGAND_TIMEOUT;
}

#define PIN_HX711_CLK   32
#define PIN_HX711_DATA  34  // This is an input only pin without pullup.
#define PIN_HX711_RATE  33
HX711     scale;
WebServer webserver(80);

void
setup()
{
  char  hostname[28];
  timeval tv;
  timezone tz = {0, 0};
  
  delay(100);
  pinMode(PIN_AUGER_EN, OUTPUT);
  digitalWrite(PIN_AUGER_EN, HIGH);
  pinMode(PIN_AUGER_SLEEP, OUTPUT);
  digitalWrite(PIN_AUGER_SLEEP, HIGH);
  pinMode(PIN_AUGER_DIR, OUTPUT);
  pinMode(PIN_AUGER_STEP, OUTPUT);
  pinMode(PIN_AUGER_MICROSTEP, OUTPUT);
  digitalWrite(PIN_AUGER_MICROSTEP, HIGH);  // HIGH = 3200, LOW = 200 steps per revolution

  pinMode(PIN_DOOR_EN, OUTPUT);
  digitalWrite(PIN_DOOR_EN, HIGH);
  pinMode(PIN_DOOR_SLEEP, OUTPUT);
  digitalWrite(PIN_DOOR_SLEEP, HIGH);
  pinMode(PIN_DOOR_DIR, OUTPUT);
  pinMode(PIN_DOOR_STEP, OUTPUT);
  pinMode(PIN_DOOR_MICROSTEP, OUTPUT);
  digitalWrite(PIN_DOOR_MICROSTEP, LOW);

  pinMode(PIN_DATA0, INPUT);
  pinMode(PIN_DATA1, INPUT);
  pinMode(CLOSE_LIMIT, INPUT);
  pinMode(OPEN_LIMIT, INPUT);
  if (!digitalRead(CLOSE_LIMIT))
    state |= STATE_OPEN;

  Serial.begin(115200);
  configInit();
  //connect to WiFi
  WiFi.onEvent(wifiEvent, ARDUINO_EVENT_WIFI_STA_CONNECTED);
  WiFi.onEvent(wifiEvent, ARDUINO_EVENT_WIFI_STA_DISCONNECTED);
  WiFi.onEvent(wifiEvent, ARDUINO_EVENT_WIFI_STA_START);
  WiFi.mode(WIFI_STA);
  
  WiFi.disconnect(true);
  delay(100);
  strcpy(hostname, "CatFeeder-");
  strcat(hostname, conf.hostname);
  if (WiFi.setHostname(hostname))
    debug(false, "Set hostname to '%s'", hostname);
  else
    debug(false, "Hostname failed");
  
  WiFi.begin(conf.ssid, conf.wpakey);
  MDNS.begin(hostname);
  MDNS.addService("http", "tcp", 80);

/*
  if(ds_rtc.lostPower()) {
    Serial.println(F("RTC lost power"));
  }
  else {
    Serial.println(F("RTC OK"));
  }

  DateTime now = rtc.now();
  tv.tv_sec = now.unixtime();
  tv.tv_usec = 0;
  settimeofday(&tv, &tz);
 */

  pinMode(PIN_HX711_RATE, 0); // 0: 10Hz, 1: 80Hz
  scale.begin(PIN_HX711_DATA, PIN_HX711_CLK);
  scale.set_scale(conf.scale);
  scale.set_offset(conf.offset);
  state &= ~STATE_WEIGHTREPORT;
  //init and get the time
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  sntp_set_time_sync_notification_cb(ntpCallBack);
  
  ArduinoOTA.setHostname(hostname);
  ArduinoOTA
  .onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH)
      type = "firmware";
    else // U_SPIFFS
      type = "filesystem";

    // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
    debug(true, "Updating: %s", type);
  })
  .onEnd([]() {
    debug(true, "Rebooting");
  })
  .onProgress([](unsigned int progress, unsigned int total) {
    debug(false, "Received: %7d of %7d", progress, total);
  })
  .onError([](ota_error_t error) {
    debug(true, "Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) debug(false, "Auth Failed");
    else if (error == OTA_BEGIN_ERROR) debug(false, "Begin Faile");
    else if (error == OTA_CONNECT_ERROR) debug(false, "Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) debug(false, "Receive Failed");
    else if (error == OTA_END_ERROR) debug(false, "End Failed");
  });

  webserver.on("/", wshandleRoot);
  webserver.on("/config", wshandleConfig);
  webserver.on("/save", wshandleSave);
  webserver.on("/reboot", wshandleReboot);
  webserver.on("/feed", wshandleDoFeed);
  webserver.on("/calibrate", wshandleCalibrate);
  webserver.on("/docalibrate", wshandleDoCalibrate);
  webserver.on("/tare", wshandleTare);
  webserver.onNotFound(wshandle404);
  webserver.begin();
  ArduinoOTA.begin();
  closeDoor();
  openDoor();
  attachInterrupt(PIN_DATA0, ISR_D0, FALLING);
  attachInterrupt(PIN_DATA1, ISR_D1, FALLING);
  weigand_counter = millis();
}

void
loop()
{
  uint8_t   facilityCode = 0;      // decoded facility code
  uint16_t  cardCode = 0;          // decoded card code
  
  ArduinoOTA.handle();
  webserver.handleClient();
  
  if ((~state & STATE_OPEN) && digitalRead(CLOSE_LIMIT) && time(NULL) > DOOR_TIMEOUT + closeTime)
    openDoor();
  // If the door is supposed to be closed but the limit switch doesn't read closed,
  // Attempt to re-close the door
  if ((~state & STATE_OPEN) && !digitalRead(CLOSE_LIMIT)) {
    debug(true, "Door should be closed, reclosing");
    closeDoor();
  }

  weigh(false);
  if (state & STATE_WEIGHTREPORT && time(NULL) - lastFeed > 30) {
    debug(true, "Weight: %8.2fg", weigh(true));
    state &= ~STATE_WEIGHTREPORT;
  }

  // This waits to make sure that there have been no more data pulses before processing data
  if (!flagDone && weigand_counter < millis())
    flagDone = 1;
  // if we have bits and we the weigand counter went out
  if (bitCount > 0 && flagDone) {
    unsigned char i;

    switch (bitCount) {
      case 35: // 35 bit HID Corporate 1000 format
        // facility code = bits 2 to 14 (4096)
        // card code = bits 15 to 34 (524288)
        // For now, we don't support 35 bit Wiegand.
        break;
      case 26: // standard 26 bit format
        // facility code = bits 2 to 9 (128)
        for (i = 1; i < 9; i++) {
          facilityCode <<= 1;
          facilityCode |= databits[i];
        }

        // card code = bits 10 to 23 (4096)
        for (i = 9; i < 25; i++) {
          cardCode <<= 1;
          cardCode |= databits[i];
        }
        if (facilityCode && cardCode)
          checkCard(facilityCode, cardCode);
        break;
      default:
        break;
        // you can add other formats if you want!
        // Serial.println("Unable to decode.");
    }

    // cleanup and get ready for the next card
    bitCount = 0;
    facilityCode = 0;
    cardCode = 0;
    for (i = 0; i < MAX_BITS; i++) {
      databits[i] = 0;
    }
  }
}

int
checkCard(uint8_t facilityCode, uint16_t cardCode)
{
  float dispensed;
  int   i;

  for (i = 0; i < CFG_NCATS; i++)
    if (conf.cat[i].facility == facilityCode && conf.cat[i].id == cardCode)
      break;

  if (i == CFG_NCATS) {
    debug(true, "Intruder: %s", catName(facilityCode, cardCode));
    closeDoor();
    return(0);
  }
  if (conf.cat[i].flags & CFG_ACCESS) {
    lastFeed = time(NULL);
    state |= STATE_WEIGHTREPORT;
    debug(true, "Authorized: %s", catName(facilityCode, cardCode));
    if (~state & STATE_OPEN)
      openDoor();
  }
  else if (conf.cat[i].flags & CFG_DISPENSE)
    dispensedTotal += dispense(conf.perFeed);
  else
    closeDoor();
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
  int   i;

  closeTime = time(NULL);
  if (digitalRead(CLOSE_LIMIT) && ~state & STATE_OPEN) // Just reset the open timer if the door is closed
    return;
  debug(true, "Closing");
  digitalWrite(PIN_DOOR_DIR, DOOR_DIR_CLOSE);
  enableDoor();
  for (i = 0; !digitalRead(CLOSE_LIMIT); i++) {
    digitalWrite(PIN_DOOR_STEP, HIGH);
    digitalWrite(PIN_DOOR_STEP, LOW);
    delayMicroseconds(1200);
  }
  shutdownDoor();
  state &= ~STATE_OPEN;
}

void
openDoor(void)
{
  debug(true, "Opening");
  digitalWrite(PIN_DOOR_DIR, DOOR_DIR_OPEN);
  enableDoor();
  for (int i = 0; i < 2650 && !digitalRead(OPEN_LIMIT); i++) {
    digitalWrite(PIN_DOOR_STEP, HIGH);
    digitalWrite(PIN_DOOR_STEP, LOW);
    delayMicroseconds(1200);
  }
  state |= STATE_OPEN;
  shutdownDoor();
}

#define STEP_DELAY 250
float
dispense(int grams)
{
  float startWeight = weigh(false), curWeight;

  debug(true, "Dispense %dg", grams);
  enableAuger();
  // This is a bit of a magic number. 5 revolutions with a reversal
  // of 0.25 revolution every revolutions dispenses about 10 grams
  // if the kibbles don't get stuck. So, try rotating 4 as much as
  // needed which works out to 2 revolution per gram.
  for(uint8_t i = 0, stop = 0; !stop && i < grams * 2; i++) {
    digitalWrite(PIN_AUGER_DIR, AUGER_DIR_CW);
    for (int i = 0; !stop && i < 3200; i++) {
      digitalWrite(PIN_AUGER_STEP, HIGH);
      digitalWrite(PIN_AUGER_STEP, LOW);
      delayMicroseconds(STEP_DELAY);
      // Weigh every 45 degrees rotation
      if (i % 400 == 0)
        curWeight = weigh(false);
      if (i % 400 == 0 && curWeight - startWeight > grams)
        stop = 1;
    }
    digitalWrite(PIN_AUGER_DIR, AUGER_DIR_CCW);
    for (int i = 0; !stop && i < 800; i++) {
      digitalWrite(PIN_AUGER_STEP, HIGH);
      digitalWrite(PIN_AUGER_STEP, LOW);
      delayMicroseconds(STEP_DELAY);
    }
  }
  shutdownAuger();
  return(curWeight - startWeight);
}

void
shutdownDoor(void)
{
  digitalWrite(PIN_DOOR_EN, HIGH);
}
void
shutdownAuger(void)
{
  digitalWrite(PIN_AUGER_EN, HIGH);
}
void
enableDoor(void)
{
  digitalWrite(PIN_DOOR_EN, LOW);
}
void
enableAuger(void)
{
  digitalWrite(PIN_AUGER_EN, LOW);
}

float
weigh(bool doAverage)
{
  static float    weight[100];
  static uint8_t  pos=0;
  float           total;

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

void
ntpCallBack(struct timeval *tv)
{
  char timestr[20];
  struct tm *tm = localtime(&tv->tv_sec);
  
  strftime(timestr, 20, "%F %T", tm);
  debug(true, "ntp: %s", timestr);

  //rtc.adjust(now);
}

void
wifiEvent(WiFiEvent_t event)
{
  switch (event) {
    case ARDUINO_EVENT_WIFI_STA_CONNECTED:
      debug(true, "WiFi Connected");
      break;
    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
      debug(true, "WiFi Disconnected, reconnecting");
      WiFi.begin(conf.ssid, conf.wpakey);
      break;
    case ARDUINO_EVENT_WIFI_STA_START:
      debug(true, "WiFi Connecting");
      break;
    default:
      break;
  }
}

void
saveSettings(void)
{
  FastCRC16       CRC16;
  unsigned char  *p;

  conf.crc = CRC16.ccitt((uint8_t *)&conf, sizeof(cfg) - 2);
  p = (unsigned char *)&conf;
  for (uint16_t i = 0; i < sizeof(struct cfg); i++)
    EEPROM.write(i, *p++);
  EEPROM.commit();
}

void
defaultSettings(void)
{
  byte            addr[8];
  // Do not clear calibration data 
  memset(&conf.hostname, '\0', sizeof(struct cfg) - (offsetof(struct cfg, hostname) - offsetof(struct cfg, magic)));
  WiFi.macAddress().toCharArray(conf.hostname, 32);
  conf.magic = MAGIC;
}

void
configInit(void)
{
  FastCRC16        CRC16;
  unsigned char   *p;

  EEPROM.begin(512);
  p = (unsigned char *)&conf;
  for (uint16_t i = 0; i < sizeof(struct cfg); i++)
    *p++ = EEPROM.read(i);
  if (conf.magic != MAGIC || conf.crc != CRC16.ccitt((uint8_t *)&conf, sizeof(struct cfg) - 2)) {
    debug(true, "Settings corrupted, defaulting");
    defaultSettings();
    saveSettings();
  }
  scale.set_scale(conf.scale);
  scale.set_offset(conf.offset);
}

void
debug(byte logtime, const char *format, ...)
{
  va_list    pvar;
  char       str[60];
  char       timestr[20];
  time_t     t = time(NULL);
  struct tm *tm = localtime(&t);
  
  va_start(pvar, format);
    if (logtime) {
      strftime(timestr, 20, "%F %T", tm);
      Serial.print(timestr);
      Serial.print(": "); 
    }
  vsnprintf(str, 60, format, pvar);
  Serial.println(str);
  va_end(pvar);
}

/*
 * Web server parts
 */

void
wshandleRoot(void) {
  char body[1024];
  int sec = millis() / 1000;
  int min = sec / 60;
  int hr = min / 60;

  snprintf(body, 1024,
    "<html>"
    "<head>"
    "<meta http-equiv='refresh' content='20'/>"
    "<title>Feeder [%s]</title>"
    "<style>body { background-color: #cccccc; font-family: Arial, Helvetica, Sans-Serif; Color: #000088; }</style>"
    "</head>"
    "<body>"
      "<h1>Feeder %s</h1>"
      "<p>Uptime: %02d:%02d:%02d</p>"
      "<p>Weight: %6.2f</p>"
      "<p>Dispensed: %6.2f</p>"
      "<p>"
      "<a href='/config'>System Configuration</a>"
      "<form method='post' action='/feed' name='Feed'/>\n"
        "<label for='weight'>Dispense grams:</label><input name='weight' type='number' value='%d' min='1' max='60'>\n"
        "<input name='Feed' type='submit' value='Feed'>\n"
      "</form>"
      "<p><font size=1>Firmware: " __DATE__ " " __TIME__ "<br>Config Size: %d<br>"
      "offset: %ld<br>"
      "factor: %f</font>\n"
      "</body>"
    "</html>",
    conf.hostname, conf.hostname,
    hr, min % 60, sec % 60, weigh(true), dispensedTotal, conf.perFeed, sizeof(struct cfg), conf.offset, conf.scale
  );
  webserver.send(200, "text/html", body);
}

void
wshandleReboot()
{
  char body[400];
  
  snprintf(body, 400,
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
  delay(100);
  ESP.restart();
}

void 
wshandle404(void) {
  String message = "File Not Found\n\n";
  message += "URI: ";
  message += webserver.uri();
  message += "\nMethod: ";
  message += (webserver.method() == HTTP_GET) ? "GET" : "POST";
  message += "\nArguments: ";
  message += webserver.args();
  message += "\n";

  for (uint8_t i = 0; i < webserver.args(); i++) {
    message += " " + webserver.argName(i) + ": " + webserver.arg(i) + "\n";
  }

  webserver.send(404, "text/plain", message);
}

void
wshandleConfig(void)
{
  char  body[5120], temp[600];
  
  snprintf(body, 2000,
    "<html>\n"
    "<head>\n"
    "<title>Feeder [%s]</title>\n"
    "<style>body { background-color: #cccccc; font-family: Arial, Helvetica, Sans-Serif; Color: #000088; }</style>\n"
    "</head>\n"
    "<body>\n"
    "<h1>Feeder %s</h1>"
    "<form method='post' action='/save' name='Configuration'/>\n"
        "<label for='name'>Feeder Name:</label><input name='name' type='text' value='%s' size='32' maxlength='32' "
          "pattern='^[A-Za-z0-9_-]+$' title='Letters, numbers, _ and -'><br>\n"
        "<label for='ssid'>SSID:</label><input name='ssid' type='text' value='%s' size='32' maxlength='63'><br>\n"
        "<label for='key'>WPA Key:</label><input name='key' type='text' value='%s' size='32' maxlength='63'><br>\n"
        "<label for='ntp'>NTP Server:</label><input name='ntp' type='text' value='%s' size='32' maxlength='63' "
          "pattern='^([a-z0-9]+)(\\.)([_a-z0-9]+)((\\.)([_a-z0-9]+))?$' title='A valid hostname'><br>\n"
        "<label for=quota'>Daily quota:</label><input name='quota' type='number' value='%d' min='10' max='120'><br>\n"
        "<label for=perfeed'>Per feed:</label><input name='perfeed' type='number' value='%d' min='1' max='25'><br><br>\n",
        conf.hostname, conf.hostname, conf.hostname, conf.ssid, conf.wpakey, conf.ntpserver, conf.quota, conf.perFeed);

  for (int i = 0; i < CFG_NCATS; i++) {
    snprintf(temp, 600,
      "<label for='catname%d'>Cat %d:</label><input name='catname%d' type='text' value='%s' size='19' maxlength='19'><br>\n"
      "<label for=facility%d'>Facility Code:</label><input name='facility%d' type='number' value='%d' min='0' max='255'><br>\n"
      "<label for=id%d'>Tag ID:</label><input name='id%d' type='number' value='%d' min='0' max='8191'><br>\n"
      "<label for='access%d'>Access:</label><input name='access%d' type='checkbox' value='true' %s><br>\n"
      "<label for='dispense%d'>Dispense:</label><input name='dispense%d' type='checkbox' value='true' %s><br><br>\n",
      i, i + 1, i, conf.cat[i].name, i, i, conf.cat[i].facility, i, i, conf.cat[i].id, i, i, conf.cat[i].flags & CFG_ACCESS ? "checked" : "",
      i, i, conf.cat[i].flags & CFG_DISPENSE ? "checked" : ""
    );
    strcat(body, temp);
  }

  strcat(body,
    "<input name='Save' type='submit' value='Save'>\n"
    "</form>"
    "<br>"
    "<form method='post' action='/reboot' name='Reboot'>\n"
      "<input name='Reboot' type='submit' value='Reboot'>\n"
    "</form>\n"
    "</body>\n"
    "</html>");
  webserver.send(200, "text/html", body);
}

void
wshandleSave(void)
{
  char  temp[400];
  String value;

  if (webserver.hasArg("ssid")) {
    value = webserver.arg("ssid");
    strncpy(temp, value.c_str(), 399);
    urlDecode(temp);
    strncpy(conf.ssid, temp, 63);
  }

  if (webserver.hasArg("key")) {
    value = webserver.arg("key");
    strncpy(temp, value.c_str(), 399);
    urlDecode(temp);
    strncpy(conf.wpakey, temp, 63);
  }

  if (webserver.hasArg("name")) {
    value = webserver.arg("name");
    strncpy(temp, value.c_str(), 399);
    urlDecode(temp);
    strncpy(conf.hostname, temp, 32);
  }

  if (webserver.hasArg("ntp")) {
    value = webserver.arg("ntp");
    strncpy(temp, value.c_str(), 399);
    urlDecode(temp);
    strncpy(conf.ntpserver, temp, 32);
  }

  value = webserver.arg("quota");
  if (value.length() && value.toInt() >= 10 && value.toInt() <= 120)
    conf.quota = value.toInt();

  value = webserver.arg("perfeed");
  if (value.length() && value.toInt() >= 1 && value.toInt() <= 25)
    conf.perFeed = value.toInt();

  for (int i=0; i < CFG_NCATS; i++) {
    snprintf(temp, 399, "catname%d", i);
    if (webserver.hasArg(temp)) {
      value = webserver.arg(temp);
      strncpy(temp, value.c_str(), 399);
      urlDecode(temp);
      strncpy(conf.cat[i].name, temp, 20);
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
      conf.cat[i].flags |= CFG_ACCESS;
    else
      conf.cat[i].flags &= ~CFG_ACCESS;

    snprintf(temp, 399, "dispense%d", i);
    if (webserver.hasArg(temp))
      conf.cat[i].flags |= CFG_DISPENSE;
    else
      conf.cat[i].flags &= ~CFG_DISPENSE;
  }

  WiFi.hostname(conf.hostname);
  
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
  
  saveSettings();
}

void
wshandleDoFeed()
{
  char  temp[400];
  int   weight = 0;
  String value;

  value = webserver.arg("weight");
  if (value.length()&& value.toInt() > 0 && value.toInt() < 60)
    weight = value.toInt();

  snprintf(temp, 400,
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
   
  webserver.send(200, "text/html", temp);
  
  if (weight)
    dispensedTotal += dispense(weight);
}

void
wshandleCalibrate(void)
{
  char  temp[512];

  scale.set_scale();
  scale.tare(20);
  snprintf(temp, 512,
    "<html>\n"
    "<head>\n"
    "<title>Feeder [%s]</title>\n"
    "<style>body { background-color: #cccccc; font-family: Arial, Helvetica, Sans-Serif; Color: #000088; }</style>\n"
    "</head>\n"
    "<body>\n"
    "<h1>Feeder %s</h1>"
    "Place a known weight on the empty bowl and enter the weight weight below<br><br>"
    "<form method='post' action='/docalibrate' name='Configuration'/>\n"
      "<label for=weight'>Weight:</label><input name='weight' type='number' value='100' min='100' max='2000'><br><br>\n"
    "<input name='Save' type='submit' value='Calibrate'>\n"
    "</form>"
    "</body>\n"
    "</html>", conf.hostname, conf.hostname);
   
  webserver.send(200, "text/html", temp);
}

void
wshandleDoCalibrate(void)
{
  char  body[400];
  String value;
  
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
}

void
wshandleTare(void)
{
  char  temp[400];
  
  scale.tare(100);
  conf.offset = scale.get_offset();
  saveSettings();
  
  snprintf(temp, 400,
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
   
  webserver.send(200, "text/html", temp);
}
/*
 * Convert a single hex digit character to its integer value
 */
unsigned char
h2int(char c)
{
    if (c >= '0' && c <='9'){
        return((unsigned char)c - '0');
    }
    if (c >= 'a' && c <='f'){
        return((unsigned char)c - 'a' + 10);
    }
    if (c >= 'A' && c <='F'){
        return((unsigned char)c - 'A' + 10);
    }
    return(0);
}

void
urlDecode(char *urlbuf)
{
  char c;     
  char *dst = urlbuf;     
  while ((c = *urlbuf) != 0) {         
    if (c == '+') c = ' ';         
    if (c == '%') {             
      c = *++urlbuf;            
      c = (h2int(c) << 4) | h2int(*++urlbuf);         
    }        
    *dst++ = c;        
    urlbuf++;     
  }     
  *dst = '\0';
}
