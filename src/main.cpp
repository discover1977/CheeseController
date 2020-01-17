#include <Arduino.h>
#include <WiFi.h>
#include <ESP8266FtpServer.h>
#include <FS.h>
#include <SPIFFS.h>
#include <WebServer.h>
#include <EEPROM.h>
#include <ADS1115.h>
#include <Nextion.h>
#include <NextionText.h>
#include <NextionButton.h>
#include <NextionSlider.h>
#include <NextionCrop.h>

#define DBG_SERIAL  Serial2
#define NEX_SERIAL  Serial

#define PARAM_ADDR  (0)
#define PRG_NUM   10

#define ON  1
#define OFF 0
#define FORWARD		0
#define BACKWARD	1

#define PWML_MOTOR_PIN 32
#define PWMR_MOTOR_PIN 33
#define ENL_MOTOR_PIN 26
#define ENR_MOTOR_PIN 27
#define PWM_RELAY_PIN 25
#define PWM_FREQ  5000
#define PWML_TIMER_CHANN  0
#define PWMR_TIMER_CHANN  1
#define PWM_RES_BIT 8

enum NexPage {
  NexPPast,
  NexPMix,
  NexPSett
};

Nextion nex(NEX_SERIAL);
/* Past page objects */
NextionText nTxtPastState(nex, NexPPast, 1, "tPState");
NextionText nTxtTabMixPP(nex, NexPPast, 20, "tTabMix");

/* Mixer page objects */
NextionText nTxtCycleTime(nex, NexPMix, 14, "tCycleTime");
NextionText nTxtCycleNum(nex, NexPMix, 13, "tCycleNum");
NextionText nTxtPrgTime(nex, NexPMix, 18, "tPtgTime");
NextionText nTxtProgName(nex, NexPMix, 12, "tProgName");
NextionText nTxtMotorPwr(nex, NexPMix, 9, "tMotorPwr");
NextionButton nButMixStartStop(nex, NexPMix, 4, "bMixStartStop");
NextionButton nButCCRot(nex, NexPMix, 6, "bCCRot");
NextionButton nButCRot(nex, NexPMix, 8, "bCRot");
NextionButton nButPrgDec(nex, NexPMix, 3, "bPrgDec");
NextionButton nButPrgInc(nex, NexPMix, 7, "bPrgInc");
NextionSlider nSliMotorPwr(nex, NexPMix, 5, "sMotorPwr");
NextionCrop nCropMPwrDec(nex, NexPMix, 10, "mDecPwr");
NextionCrop nCropMPwrInc(nex, NexPMix, 11, "mIncPwr");

/* Setting page objects */
NextionText nTxtTabMixPS(nex, NexPSett, 4, "tTabMix");

struct Param {
  uint64_t FuseMac;         
  char ssid[20];
  char pass[20];          
} Param;  

struct ProgCycle {
  uint8_t State;
  uint16_t Time;
  uint16_t OWTime;
  uint8_t PBRevers;
  uint8_t Power;
};

enum ProgIndex {
  MixManual = -1
};

ProgCycle prgCycle[PRG_NUM];
String prgList[PRG_NUM];
uint8_t prgCount = 0;
uint8_t cyclesCount = 0;
uint8_t HeatingPower = 0;
uint8_t MotorPower = 25;
char TxtChar[51];
uint8_t CurrCycle = 0;
uint16_t CurrCycleTime = 0;
uint32_t ProgTime = 0;
int8_t CurrProg = -1;
uint8_t Direction = FORWARD;

WebServer server;
FtpServer ftpSrv;
const char* ftpUser = "esp32";
const char* ftpPass = "esp32";
const char* ap_ssid = "esp32_dev";
const char* ap_pass = "esp32_dev";

struct RunningTime {
  uint8_t second = 0; 
  uint8_t minute = 0;
  uint8_t hour = 0;
  uint16_t day = 0;
} RunningTime;

struct Flag {
  bool SecondTimer;
  bool MixProgEn;
  bool MixManualEn;
  bool InitLocalTimerVar;
  bool EndProg;
} Flag;

hw_timer_t *relayTimer = NULL;
void IRAM_ATTR onRelayTimer() {
  static uint8_t pwmCnt = 0;
  if(pwmCnt < HeatingPower) {
    digitalWrite(PWM_RELAY_PIN, HIGH);
  }
  else {
    digitalWrite(PWM_RELAY_PIN, LOW);
  }
  if(++pwmCnt > 100) pwmCnt = 0;
}

String get_rt() {
    char str[30];
    sprintf(str, 
            "%3u / %02u:%02u:%02u",
            RunningTime.day,
            RunningTime.hour,
            RunningTime.minute,
            RunningTime.second);
    return str;
}

void led_ctrl(uint8_t lev) {
  digitalWrite(2, lev);
}

void sblink(uint8_t rep, uint16_t del) {
  for(int i = 0; i < rep; i++) {
    led_ctrl(HIGH);
    delay(del);
    led_ctrl(LOW);
    delay(del);
  }
}

int8_t set_power(uint8_t val) {
	if(val > 100) return -1;
	else {
    ledcWrite(0, map(val, 0, 255, 0, 100));
	}
	return val;
}

enum State{
	Stop,
	Work,
  Pause
};

void motor_ctrl(uint8_t enable, uint8_t direct, uint8_t power) {
	if(enable == ON) {
    digitalWrite(ENL_MOTOR_PIN, HIGH);
    digitalWrite(ENR_MOTOR_PIN, HIGH);
    if(direct == FORWARD) {
      ledcWrite(PWML_TIMER_CHANN, map(power, 0, 100, 0, 255));
      ledcWrite(PWMR_TIMER_CHANN, 0);
      //Serial.print(F("Motor ON, CW, power: ")); Serial.print(power); Serial.println(F("%"));
    }
    else {
      ledcWrite(PWML_TIMER_CHANN, 0);
      ledcWrite(PWMR_TIMER_CHANN, map(power, 0, 100, 0, 255));
      //Serial.print(F("Motor ON, CCW, power: ")); Serial.print(power); Serial.println(F("%"));
    }
	}
	else {
    digitalWrite(ENL_MOTOR_PIN, LOW);
    digitalWrite(ENR_MOTOR_PIN, LOW);
    ledcWrite(PWML_TIMER_CHANN, 0);
    ledcWrite(PWMR_TIMER_CHANN, 0);
    //Serial.println(F("Motor OFF"));
	}
}

uint32_t cycle_time(uint8_t cycle) {
	uint32_t result = 0;
	result = prgCycle[cycle].Time;
	if(prgCycle[cycle].OWTime != 0) {
		result += prgCycle[cycle].Time / prgCycle[cycle].OWTime * prgCycle[cycle].PBRevers;
	}
	return result;
}

uint32_t prog_time(uint8_t prgNum) {
  uint32_t ret = 0;
  for(int i = 0; i < cyclesCount; i++) {
    ret += cycle_time(i);
  }
  return ret;
}

hw_timer_t *secondTimer = NULL;
void IRAM_ATTR onSecondTimer() {
  static uint8_t lCycleCount = 0;
  static uint16_t lCycleTime = 0, lPTime = 0, lRTime = 0, lDirection = FORWARD, lState = Work;

  Flag.SecondTimer = true;

	if(Flag.InitLocalTimerVar) {
		Flag.InitLocalTimerVar = 0;
		lPTime = 0;
		lRTime = 0;
		lDirection = FORWARD;
		lState = Work;
		lCycleCount = 0;
    CurrCycleTime = cycle_time(lCycleCount);
    CurrCycle = 0;
    ProgTime = prog_time(CurrProg);
	}

  /* Mixer motor control ************************************************/
  if(Flag.MixProgEn) {
		if(lCycleCount < cyclesCount) {
			if(prgCycle[lCycleCount].State == Work) {
				if(lState == Work) {
					motor_ctrl(ON, lDirection, prgCycle[lCycleCount].Power);   
					if(++lRTime == prgCycle[lCycleCount].OWTime) {
						lRTime = 0;
						lState = Pause;
					}
				}
				else {
					motor_ctrl(OFF, FORWARD, 0);
					if(++lPTime == prgCycle[lCycleCount].PBRevers) {
						lPTime = 0;
						if(prgCycle[lCycleCount].OWTime > 0) {
							lDirection = !lDirection;
						}
						else {
							lDirection = FORWARD;
						}
						lState = Work;
					}
				}
			}
			else {
				/*set_power(prgCycle[lCycleCount].Power);*/
				motor_ctrl(OFF, FORWARD, prgCycle[lCycleCount].Power);
			}
      CurrCycleTime--;

			if(++lCycleTime == cycle_time(lCycleCount)) {
				lCycleTime = 0;
				lCycleCount++;
				CurrCycle = lCycleCount;
        CurrCycleTime = cycle_time(lCycleCount);
				// beep(30, 1);
			}
			ProgTime--;
		}
		if(lCycleCount == cyclesCount) {
			lPTime = 0;
			lRTime = 0;
			lDirection = FORWARD;
			lState = Work;
			lCycleCount = 0;
			Flag.MixProgEn = false;
      Flag.EndProg = true;
			//Flag.BeepOnStop = 1;
		}
	}
  /************************************************ Mixer motor control */
}

void save_param() {
  led_ctrl(HIGH);
  int eSize = sizeof(Param);
  EEPROM.begin(eSize);
  EEPROM.put(PARAM_ADDR, Param);
  EEPROM.end();
  led_ctrl(LOW);
}

void eeprom_init() {
  DBG_SERIAL.println("EEPROM initialization");
  int eSize = sizeof(Param);
  EEPROM.begin(eSize);
  Param.FuseMac = ESP.getEfuseMac();
  EEPROM.put(PARAM_ADDR, Param);
  EEPROM.end();
  DBG_SERIAL.println("EEPROM initialization completed");
  DBG_SERIAL.println("");
}

/* Функция формирования строки XML данных */
String build_XML() {
  String xmlStr;
  xmlStr = F("<?xml version='1.0'?>");
  xmlStr +=    F("<xml>");
  xmlStr +=      F("<x_rt>");
  xmlStr +=        get_rt();   // for example
  xmlStr +=      F("</x_rt>");
  xmlStr +=     F("</xml>");
  // Serial.println(XML);
  return xmlStr;
}

bool loadFromSpiffs(String path) {
  led_ctrl(HIGH);
  DBG_SERIAL.print(F("Request File: "));
  DBG_SERIAL.println(path);
  String dataType = F("text/plain");
  if(path.endsWith(F("/"))) path += F("index.html");

  if(path.endsWith(F(".src"))) path = path.substring(0, path.lastIndexOf(F(".")));
  else if(path.endsWith(F(".html"))) dataType = F("text/html");
  else if(path.endsWith(F(".htm"))) dataType = F("text/html");
  else if(path.endsWith(F(".css"))) dataType = F("text/css");
  else if(path.endsWith(F(".js"))) dataType = F("application/javascript");
  else if(path.endsWith(F(".png"))) dataType = F("image/png");
  else if(path.endsWith(F(".gif"))) dataType = F("image/gif");
  else if(path.endsWith(F(".jpg"))) dataType = F("image/jpeg");
  else if(path.endsWith(F(".ico"))) dataType = F("image/x-icon");
  else if(path.endsWith(F(".xml"))) dataType = F("text/xml");
  else if(path.endsWith(F(".pdf"))) dataType = F("application/pdf");
  else if(path.endsWith(F(".zip"))) dataType = F("application/zip");
  File dataFile = SPIFFS.open(path.c_str(), "r");
  DBG_SERIAL.print(F("Load File: "));
  DBG_SERIAL.println(dataFile.name());
  if (server.hasArg(F("download"))) dataType = F("application/octet-stream");
  if (server.streamFile(dataFile, dataType) != dataFile.size()) {}

  dataFile.close();
  led_ctrl(LOW);
  return true;
}

/* Обработчик/handler запроса XML данных */
void h_XML() {
  server.send(200, F("text/xml"), build_XML());
}

void h_wifi_param() {
  DBG_SERIAL.println(F("Apply Wi-Fi parameters"));
  String ssid = server.arg(F("wifi_ssid"));
  String pass = server.arg(F("wifi_pass"));
  DBG_SERIAL.print(F("Wi-Fi SSID: ")); DBG_SERIAL.println(ssid);
  DBG_SERIAL.print(F("Wi-Fi pass: ")); DBG_SERIAL.println(pass);
  server.send(200, F("text/html"), F("Wi-Fi setting is updated, module will be rebooting..."));
  // WiFi.begin(ssid.c_str(), pass.c_str());
  delay(100);
  WiFi.softAPdisconnect(true);
  delay(100);
  strcpy(Param.ssid, ssid.c_str());
  strcpy(Param.pass, pass.c_str());
  save_param();
  DBG_SERIAL.print(F("Param SSID: ")); DBG_SERIAL.println(Param.ssid);
  DBG_SERIAL.print(F("Param pass: ")); DBG_SERIAL.println(Param.pass);
  WiFi.mode(WIFI_STA);
  WiFi.begin(Param.ssid, Param.pass);
  delay(500);
  SPIFFS.end();
  DBG_SERIAL.println(F("Resetting ESP..."));
  ESP.restart();
}

void handleWebRequests() {
  if(loadFromSpiffs(server.uri())) return;
  DBG_SERIAL.println(F("File Not Detected"));
  String message = F("File Not Detected\n\n");
  message += F("URI: ");
  message += server.uri();
  message += F("\nMethod: ");
  message += (server.method() == HTTP_GET)?F("GET"):F("POST");
  message += F("\nArguments: ");
  message += server.args();
  message += "\n";
  for (uint8_t i=0; i<server.args(); i++){
    message += " NAME:" + server.argName(i) + "\n VALUE:" + server.arg(i) + "\n";
  }
  server.send(404, F("text/plain"), message);
  DBG_SERIAL.println(message);
}

/* Обработчик/handler запроса HTML страницы */
/* Отправляет клиенту(браузеру) HTML страницу */
void h_Website() {  
  //Serial.println("HTML handler");
  server.sendHeader(F("Location"), F("/index.html"), true);   //Redirect to our html web page
  server.send(302, F("text/plane"), "");
}

void ap_config() {
  sblink(1, 500);
  DBG_SERIAL.println(F("Wi-Fi AP mode"));
  DBG_SERIAL.println(F("AP configuring..."));
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ap_ssid, ap_pass); 
  DBG_SERIAL.println(F("done"));
  IPAddress myIP = WiFi.softAPIP();
  DBG_SERIAL.print(F("AP IP address: "));
  DBG_SERIAL.println(myIP);    
}

uint8_t readProg(uint8_t progNumber, ProgCycle *pc) {
  DBG_SERIAL.print(F("Read prog: ")); DBG_SERIAL.println(prgList[progNumber]);
  String fn = F("/");
  if(prgList[progNumber].indexOf("\r") != -1) {
    prgList[progNumber].setCharAt((prgList[progNumber].length() - 1), ' ');
    prgList[progNumber].trim();
  }
  fn += prgList[progNumber];
  fn += F(".csv");
  uint8_t cycleNumber = 0;
  uint8_t si, ei;
  DBG_SERIAL.print(F("Open file prog: ")); DBG_SERIAL.println(fn);
  if(SPIFFS.exists(fn)) {
    File pf = SPIFFS.open(fn, "r");  
    if (!pf) {
      DBG_SERIAL.println(F("Prog file open failed on read."));
    } else {
      while(pf.available()) {
        String line = pf.readStringUntil('\n');
        //Serial.print(F("Cycle string: ")); Serial.println(line);
        pc[cycleNumber].State = (line.substring(0, 1) == "W")?(1):(0);
        si = 2;
        ei = line.indexOf(";", si);
        pc[cycleNumber].Time = line.substring(si, ei).toInt();
        si = ei + 1;
        ei = line.indexOf(";", si);
        pc[cycleNumber].OWTime = line.substring(si, ei).toInt();
        si = ei + 1;
        ei = line.indexOf(";", si);
        pc[cycleNumber].PBRevers = line.substring(si, ei).toInt();
        si = ei + 1;
        ei = line.indexOf(";", si);
        pc[cycleNumber].Power = line.substring(si, ei).toInt();
        cycleNumber++;
      } 
      pf.close();
    }   
  }
  else {
    DBG_SERIAL.print(F("File: ")); DBG_SERIAL.print(fn); DBG_SERIAL.println(F(" not found!"));
  }
  return cycleNumber;
}

int read_progList(String file) {
  uint8_t cnt = 0;  
  File f = SPIFFS.open("/ProgList.txt", "r");
  if (!f) {
    DBG_SERIAL.println(F("ProgList file open failed."));
  } else {
    while(f.available()) {
      String line = f.readStringUntil('\n');
      prgList[cnt] = line;
      cnt++;      
    } 
    f.close();
  }
  return cnt;
}

char* strToCharArray(String str) {
  str.toCharArray(TxtChar, (str.length() + 1));
  return TxtChar;
}

char* intToCharArray(int val) {
  sprintf(TxtChar, "%i", val);
  return TxtChar;
}

char* getMPwrCharArr(int val) {
  sprintf(TxtChar, "%i%%", val);
  return TxtChar;
}

/*
void cbNex_______(NextionEventType type, INextionTouchable *widget) {
  if (type == NEX_EVENT_PUSH) {}
  else if (type == NEX_EVENT_POP) {}
}
*/

void nSendPrgName(int8_t val) {
  if(val >= 0) {
    nTxtProgName.setText(strToCharArray(prgList[val]));
    DBG_SERIAL.print(F("Prog number: ")); DBG_SERIAL.print(val);
    DBG_SERIAL.print(F(", prg name: ")); DBG_SERIAL.print(prgList[val]);
    DBG_SERIAL.print(F("(")); DBG_SERIAL.print(TxtChar); DBG_SERIAL.println(F(")"));
  }
  else {
    nTxtProgName.setText("Manual");
  }
}

void nSendMotorPwr(uint8_t val) {
  nTxtMotorPwr.setText(getMPwrCharArr(val));
}

void nSendButMixStartStopTxt(String str) {
  nButMixStartStop.setText(strToCharArray(str));
}

void cbButMixStartStop(NextionEventType type, INextionTouchable *widget) {
  if (type == NEX_EVENT_PUSH) {
    if(CurrProg == MixManual) {
      if(!Flag.MixManualEn) {
        Flag.MixManualEn = true;
        motor_ctrl(ON, Direction, MotorPower);
        nSendButMixStartStopTxt("Stop");
      }
      else {
        Flag.MixManualEn = false;
        motor_ctrl(OFF, Direction, MotorPower);
        nSendButMixStartStopTxt("Start");
      }
    }
    else {
      if(!Flag.MixProgEn) {        
        Flag.InitLocalTimerVar = true;
        Flag.MixProgEn = true;
        nSendButMixStartStopTxt("Stop");
      }
      else {
        Flag.MixProgEn = false;
        motor_ctrl(OFF, FORWARD, 0);
        nSendButMixStartStopTxt("Start");
      }
    }
  }
  else if (type == NEX_EVENT_POP) {}
}

void getProgTimeStr(uint32_t t, char* txt) {
  uint8_t h = 0, m = 0, s = 0;
    s = t % 60;
    t /= 60;
    m = t % 60;
    h = t / 60;
    sprintf(TxtChar, "%i:%02i:%02i", h, m, s);
}

void nSendProgTime() {
  getProgTimeStr(prog_time(CurrProg), TxtChar);
  if(CurrProg != MixManual) {
    nTxtPrgTime.setText(TxtChar);
  }
  else nTxtPrgTime.setText("-/--/--");
}

void cbButPrgDec(NextionEventType type, INextionTouchable *widget) {
  if (type == NEX_EVENT_PUSH) {
    if(!Flag.MixProgEn) {
      if(--CurrProg < -1) CurrProg = (prgCount - 1);
      readProg(CurrProg, prgCycle);
      nSendPrgName(CurrProg);
      nSendProgTime();
    }
  }
  else if (type == NEX_EVENT_POP) {}
}

void cbButPrgInc(NextionEventType type, INextionTouchable *widget) {
  if (type == NEX_EVENT_PUSH) {
    if(!Flag.MixProgEn) {
      if(++CurrProg == prgCount) CurrProg = -1;
      readProg(CurrProg, prgCycle);
      nSendPrgName(CurrProg);
      nSendProgTime();
    }
  }
  else if (type == NEX_EVENT_POP) {}
}

void cbButCCRot(NextionEventType type, INextionTouchable *widget) {
  if((!Flag.MixManualEn) && (!Flag.MixProgEn)) {
    if (type == NEX_EVENT_PUSH) {
      motor_ctrl(ON, BACKWARD, MotorPower);  
      Direction = BACKWARD;  
    }
    else if (type == NEX_EVENT_POP) {
      motor_ctrl(OFF, BACKWARD, 0);
    }
  }
}

void cbButCRot(NextionEventType type, INextionTouchable *widget) {
  if((!Flag.MixManualEn) && (!Flag.MixProgEn)) {
    if (type == NEX_EVENT_PUSH) {
      motor_ctrl(ON, FORWARD, MotorPower);
      Direction = FORWARD;
    }
    else if (type == NEX_EVENT_POP) {
      motor_ctrl(OFF, FORWARD, 0);
    }
  }
}

void cbTxtTabMix(NextionEventType type, INextionTouchable *widget) {
  if (type == NEX_EVENT_PUSH) {
    nSendPrgName(CurrProg);
    nSliMotorPwr.setValue(MotorPower);
    nSendMotorPwr(MotorPower);
    nSendProgTime();
  }
  else if (type == NEX_EVENT_POP) {}
}

void cbCropMPwrDec(NextionEventType type, INextionTouchable *widget) {
  if (type == NEX_EVENT_PUSH) {
    if(MotorPower > 0) MotorPower--;
    nSendMotorPwr(MotorPower);
    if(Flag.MixManualEn) {
      motor_ctrl(ON, Direction, MotorPower);
    }
    DBG_SERIAL.print(F("Motor power: ")); DBG_SERIAL.println(MotorPower);
  }
  else if (type == NEX_EVENT_POP) {}
}

void cbCropMPwrInc(NextionEventType type, INextionTouchable *widget) {
  if (type == NEX_EVENT_PUSH) {
    if(MotorPower < 100) MotorPower++;
    nSendMotorPwr(MotorPower);
    if(Flag.MixManualEn) {
      motor_ctrl(ON, Direction, MotorPower);
    }
    DBG_SERIAL.print(F("Motor power: ")); DBG_SERIAL.println(MotorPower);
  }
  else if (type == NEX_EVENT_POP) {}
}

void cbSliMotorPwr(NextionEventType type, INextionTouchable *widget) {
  if (type == NEX_EVENT_PUSH) {}
  else if (type == NEX_EVENT_POP) {
    MotorPower = nSliMotorPwr.getValue();
    if(Flag.MixManualEn) {
      motor_ctrl(ON, Direction, MotorPower);
    }
    //nSendMotorPwr(MotorPower);
    DBG_SERIAL.print(F("Motor power: ")); DBG_SERIAL.println(MotorPower);
  }
}

void setup() {
  // put your setup code here, to run once:
  DBG_SERIAL.begin(921600);
  NEX_SERIAL.begin(921600);
  nex.init();

  pinMode(2, OUTPUT);
  pinMode(ENL_MOTOR_PIN, OUTPUT);
  pinMode(ENR_MOTOR_PIN, OUTPUT);
  pinMode(PWML_MOTOR_PIN, OUTPUT);
  pinMode(PWMR_MOTOR_PIN, OUTPUT);
  pinMode(PWM_RELAY_PIN, OUTPUT);

  sblink(20, 20);

  uint8_t WiFiConnTimeOut = 50;
  delay(1000);
  DBG_SERIAL.println("");
  DBG_SERIAL.println(F("--- Wi-Fi config ---"));
  
  DBG_SERIAL.println("");
  int eSize = sizeof(Param);
  EEPROM.begin(eSize);
  EEPROM.get(PARAM_ADDR, Param);
  if(Param.FuseMac != ESP.getEfuseMac()) {
    eeprom_init();
    ap_config();
  }
  else {
    DBG_SERIAL.println(F("Wi-Fi STA mode"));
    //Serial.print(F("Param SSID: ")); Serial.println(Param.ssid);
    //Serial.print(F("Param pass: ")); Serial.println(Param.pass);
    WiFi.mode(WIFI_STA);
    WiFi.begin(Param.ssid, Param.pass);
    while (WiFi.status() != WL_CONNECTED) {      
      sblink(1, 10);
      delay(190);
      DBG_SERIAL.print(".");
      if(--WiFiConnTimeOut == 0) break;
    }
    if(WiFiConnTimeOut == 0) {
      DBG_SERIAL.println("");
      DBG_SERIAL.println(F("Wi-Fi connected timeout"));
      ap_config();        
    }
    else {
      sblink(2, 500);
      DBG_SERIAL.println("");
      DBG_SERIAL.print(F("STA connected to: ")); 
      DBG_SERIAL.println(WiFi.SSID());
      IPAddress myIP = WiFi.localIP();
      DBG_SERIAL.print(F("Local IP address: ")); DBG_SERIAL.println(myIP);
      WiFi.enableAP(false);
    }
  }

  DBG_SERIAL.println();

  delay(1000);

  if(SPIFFS.begin(true)) {
    DBG_SERIAL.println(F("SPIFFS opened!"));
    ftpSrv.begin(ftpUser, ftpPass); 
    DBG_SERIAL.println(F("FTP server started!"));
    DBG_SERIAL.print(F("user: ")); DBG_SERIAL.println(ftpUser);
    DBG_SERIAL.print(F("pass: ")); DBG_SERIAL.println(ftpPass);

    DBG_SERIAL.print(F("SPIFFS total bytes: ")); DBG_SERIAL.println(SPIFFS.totalBytes());
    DBG_SERIAL.print(F("SPIFFS used bytes: ")); DBG_SERIAL.println(SPIFFS.usedBytes());
  }
  // Регистрация обработчиков
  server.on(F("/"), h_Website);    
  server.on(F("/wifi_param"), h_wifi_param); 
  //server.on(F("/pushButt"), h_pushButt);    
  server.onNotFound(handleWebRequests);       
  server.on(F("/xml"), h_XML);
  server.begin();

  //eeprom_init();
  //save_param();
  prgCount = read_progList(F("ProgList.txt"));
  DBG_SERIAL.print(F("Read "));
  DBG_SERIAL.print(prgCount);
  DBG_SERIAL.println(F(" prog"));

  for(uint i = 0; i < prgCount; i++) {
    DBG_SERIAL.print(i + 1); DBG_SERIAL.print(F(" - ")); DBG_SERIAL.println(prgList[i]); 
  }

  cyclesCount = readProg(0, prgCycle);
  DBG_SERIAL.print(F("Read "));
  DBG_SERIAL.print(cyclesCount);
  DBG_SERIAL.print(F(" cycles in "));
  DBG_SERIAL.print(prgList[0]);
  DBG_SERIAL.println(F(" prog"));

  for(uint i = 0; i < cyclesCount; i++) {
    DBG_SERIAL.print(F("Cycle: ")); DBG_SERIAL.print(i + 1); DBG_SERIAL.print(F(" - "));
    DBG_SERIAL.print(prgCycle[i].State); DBG_SERIAL.print(F(", "));
    DBG_SERIAL.print(prgCycle[i].Time); DBG_SERIAL.print(F(", "));
    DBG_SERIAL.print(prgCycle[i].OWTime); DBG_SERIAL.print(F(", "));
    DBG_SERIAL.print(prgCycle[i].PBRevers); DBG_SERIAL.print(F(", "));
    DBG_SERIAL.println(prgCycle[i].Power);
  }

  ledcSetup(PWML_TIMER_CHANN, PWM_FREQ, PWM_RES_BIT);
  ledcAttachPin(PWML_MOTOR_PIN, PWML_TIMER_CHANN);
  ledcWrite(PWML_TIMER_CHANN, 0);

  ledcSetup(PWMR_TIMER_CHANN, PWM_FREQ, PWM_RES_BIT);
  ledcAttachPin(PWMR_MOTOR_PIN, PWMR_TIMER_CHANN);
  ledcWrite(PWMR_TIMER_CHANN, 0);

  relayTimer = timerBegin(2, 80, true);
  timerAttachInterrupt(relayTimer, &onRelayTimer, true);
  timerAlarmWrite(relayTimer, 10000, true);
  timerAlarmEnable(relayTimer);

  secondTimer = timerBegin(3, 80, true);
  timerAttachInterrupt(secondTimer, &onSecondTimer, true);
  timerAlarmWrite(secondTimer, 1000000, true);
  timerAlarmEnable(secondTimer);

  nTxtTabMixPP.attachCallback(&cbTxtTabMix);
  nTxtTabMixPS.attachCallback(&cbTxtTabMix);
  nButCCRot.attachCallback(&cbButCCRot);
  nButCRot.attachCallback(&cbButCRot);
  nButPrgDec.attachCallback(&cbButPrgDec);
  nButPrgInc.attachCallback(&cbButPrgInc);
  nCropMPwrDec.attachCallback(&cbCropMPwrDec);
  nCropMPwrInc.attachCallback(&cbCropMPwrInc);
  nSliMotorPwr.attachCallback(&cbSliMotorPwr);
  nButMixStartStop.attachCallback(&cbButMixStartStop);
}

void loop() {
  /* Обработка запросов HTML клиента */
  server.handleClient();
  /* Обработка запросов FTP клиента */
  ftpSrv.handleFTP();
  /* Обработка сообщений Nextion дисплея */
  nex.poll();

  if(Flag.SecondTimer) {
    Flag.SecondTimer = false;
    if((Flag.MixProgEn) || (Flag.EndProg)) {      
      sprintf(TxtChar, "%i/%s", (CurrCycle + 1), ((prgCycle[CurrCycle].State == 1)?("W"):("P")));
      nTxtCycleNum.setText(TxtChar);
      if(cycle_time(CurrCycle) < 60) sprintf(TxtChar, "0:%02i", CurrCycleTime);
      else sprintf(TxtChar, "%i:%02i", (CurrCycleTime / 60), (CurrCycleTime % 60));
      nTxtCycleTime.setText(TxtChar);     
      getProgTimeStr(ProgTime, TxtChar);
      nTxtPrgTime.setText(TxtChar);
      nSendMotorPwr(prgCycle[CurrCycle].Power);
      nSliMotorPwr.setValue(prgCycle[CurrCycle].Power);
      if(Flag.EndProg) {
        Flag.EndProg = false;
        nButMixStartStop.setText("Start");
        nTxtCycleNum.setText("--/-");
        nTxtCycleTime.setText("--:--");
        nSendProgTime();
      }
    }
  }
}