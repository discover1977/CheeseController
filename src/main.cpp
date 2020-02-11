#include <Arduino.h>
#include <WiFi.h>
#include <ESP8266FtpServer.h>
#include <ESP32httpUpdate.h>
#include <FS.h>
#include <SPIFFS.h>
#include <WebServer.h>
#include <EEPROM.h>
#include <Nextion.h>
#include <NextionText.h>
#include <NextionButton.h>
#include <NextionRadioButton.h>
#include <NextionSlidingText.h>
#include <NextionSlider.h>
#include <NextionCrop.h>
#include <PID_v1.h>
#include <Adafruit_ADS1015.h>
#include <Wire.h>

#define DBG_SERIAL  Serial
#define DBG_BAUD    921600
#define NEX_SERIAL  Serial2
#define NEX_BAUD    921600

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
  NexPNone = -1,
  NexPHeat,
  NexPMix,
  NexPSett
};

Adafruit_ADS1115  adcTemp;

Nextion nex(NEX_SERIAL);
/* Past page objects */
NextionText nTxtPHeatH(nex, NexPHeat, 17, "tPHeatH");
NextionText nTxtPMixH(nex, NexPHeat, 18, "tPMixH");
NextionText nTxtPSettH(nex, NexPHeat, 19, "tPSettH");

NextionText nTxtHeatState(nex, NexPHeat, 1, "tHeatState");
NextionText nTxtHeatStateA(nex, NexPHeat, 23, "tHeatStateA");
NextionText nTxtPState(nex, NexPHeat, 1, "tPState");
NextionText nTxtMilkT(nex, NexPHeat, 2, "tMilk_t");
NextionText nTxtShirtT(nex, NexPHeat, 3, "tShirt_t");
NextionText nTxtHMode(nex, NexPHeat, 13, "tHMode");
NextionText nTxtUVal(nex, NexPHeat, 15, "tUVal");
NextionText nTxtUnit(nex, NexPHeat, 14, "tUnit");
NextionText nTxtHeatPwr(nex, NexPHeat, 24, "tHeatPwr");
NextionButton nButHeatUp(nex, NexPHeat, 8, "bHeatUp");
NextionButton nButHeatDown(nex, NexPHeat, 9, "bHeatDown");
NextionButton nButHeatStart(nex, NexPHeat, 7, "bHeatStart");
NextionRadioButton nRButPast(nex, NexPHeat, 4, "rbPast");
NextionRadioButton nRButPower(nex, NexPHeat, 5, "rbPower");
NextionRadioButton nRButTemp(nex, NexPHeat, 6, "rbTemp");

/* Mixer page objects */
NextionText nTxtPHeatM(nex, NexPMix, 15, "tPHeatM");
NextionText nTxtPMixM(nex, NexPMix, 16, "tPMixM");
NextionText nTxtPSettM(nex, NexPMix, 17, "tPSettM");

NextionText nTxtCycleTime(nex, NexPMix, 14, "tCycleTime");
NextionText nTxtCycleNum(nex, NexPMix, 13, "tCycleNum");
NextionText nTxtProgTime(nex, NexPMix, 18, "tProgTime");
NextionText nTxtProgName(nex, NexPMix, 12, "tProgName");
NextionText nTxtMotorPwr(nex, NexPMix, 9, "tMotorPwr");
NextionButton nButMixStart(nex, NexPMix, 4, "bMixStart");
NextionButton nButCCRot(nex, NexPMix, 6, "bCCRot");
NextionButton nButCRot(nex, NexPMix, 8, "bCRot");
NextionButton nButPrgDec(nex, NexPMix, 3, "bPrgDec");
NextionButton nButPrgInc(nex, NexPMix, 7, "bPrgInc");
NextionSlider nSliMotorPwr(nex, NexPMix, 5, "sMotorPwr");
NextionCrop nCropMPwrDec(nex, NexPMix, 10, "mDecPwr");
NextionCrop nCropMPwrInc(nex, NexPMix, 11, "mIncPwr");

/* Setting page objects */
NextionText nTxtPHeatS(nex, NexPSett, 3, "tPHeatS");
NextionText nTxtPMixS(nex, NexPSett, 4, "tPMixS");
NextionText nTxtPSettS(nex, NexPSett, 5, "tPSettS");

NextionSlidingText nSTxtAddr(nex, NexPSett, 8, "stAddr");
NextionText nQrAddr(nex, NexPSett, 6, "qrAddr");

int8_t NPage = NexPHeat;

struct Param {
  uint64_t FuseMac;         
  char ssid[20];
  char pass[20];
  uint8_t SetPointValue[3];
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

TaskHandle_t Task1;
IPAddress myIP;
ProgCycle prgCycle[PRG_NUM];
String prgList[PRG_NUM];
uint8_t prgCount = 0;
uint8_t cyclesCount = 0;
uint8_t MotorPower = 25;
char Text[51];
uint8_t CurrCycle = 0;
uint16_t CurrCycleTime = 0;
uint32_t ProgTime = 0;
int8_t CurrProg = MixManual;
uint8_t Direction = FORWARD;
int8_t OPower = -1;

/* Pasteurizer variables */
enum TempEnum {
  Milk,
  Shirt
};
double Temperature[2] = {0, 0};
double PIDSetpoint, PIDInput, PIDOutput;
double aggKp=15, aggKi=10, aggKd=5;
double consKp=1, consKi=0.5, consKd=0.5;
bool consK = false;
uint16_t PastDelayCnt;
PID m_PID(&PIDInput, &PIDOutput, &PIDSetpoint, consKp, consKi, consKd, DIRECT);
enum HeatModeEnum {
  Past,
  Power,
  Temp
};
uint8_t HeatingMode = Past;
enum PastStateEnum {
  PastStateIdle,
  PastStateHeating,
  PastStateDelay
};
uint8_t PastState = PastStateIdle;
uint8_t HeatingPWM = 0;
char degSymbol[3] = {0};
char percentSymbol[2] = {'%', 0x00};

/* WEB & FTP server variables */
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
  bool HeatingEn;
  bool HSecond;
} Flag;

void log(String str) {
  DBG_SERIAL.println(str);
}
void log(char* str) {
  DBG_SERIAL.println(str);
}
void log(String str, String sArg) {
  DBG_SERIAL.print(str);
  DBG_SERIAL.print(": ");
  DBG_SERIAL.println(sArg);
}
void log(char* str, char* sArg) {
  DBG_SERIAL.print(str);
  DBG_SERIAL.print(": ");
  DBG_SERIAL.println(sArg);
}
void log(String str, int nArg) {
  DBG_SERIAL.print(str);
  DBG_SERIAL.print(": ");
  DBG_SERIAL.println(nArg);
}
void log(char* str, int nArg) {
  DBG_SERIAL.print(str);
  DBG_SERIAL.print(": ");
  DBG_SERIAL.println(nArg);
}

hw_timer_t *relayTimer = NULL;
void IRAM_ATTR onRelayTimer() {
  static uint8_t hsCnt = 0;
  static uint8_t pwmCnt = 0;
  if(pwmCnt < HeatingPWM) {
    digitalWrite(PWM_RELAY_PIN, HIGH);
  }
  else {
    digitalWrite(PWM_RELAY_PIN, LOW);
  }
  if(++pwmCnt > 100) pwmCnt = 0;

  if(++hsCnt == 50) {
    hsCnt = 0;
    Flag.HSecond = true;
  }
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

enum State{
	Stop,
	Work,
  Pause
};

void heating_ctrl(uint8_t val) {
  if(val > 100) return;
  HeatingPWM = val;
}

void motor_ctrl(uint8_t enable, uint8_t direct, uint8_t power) {
	if(enable == ON) {
    digitalWrite(ENL_MOTOR_PIN, HIGH);
    digitalWrite(ENR_MOTOR_PIN, HIGH);
    if(direct == FORWARD) {
      ledcWrite(PWML_TIMER_CHANN, map(power, 0, 100, 0, 255));
      ledcWrite(PWMR_TIMER_CHANN, 0);
      DBG_SERIAL.print(F("Motor ON, CW, power: ")); Serial.print(power); Serial.println(F("%"));
    }
    else {
      ledcWrite(PWML_TIMER_CHANN, 0);
      ledcWrite(PWMR_TIMER_CHANN, map(power, 0, 100, 0, 255));
      DBG_SERIAL.print(F("Motor ON, CCW, power: ")); Serial.print(power); Serial.println(F("%"));
    }
	}
	else {
    digitalWrite(ENL_MOTOR_PIN, LOW);
    digitalWrite(ENR_MOTOR_PIN, LOW);
    ledcWrite(PWML_TIMER_CHANN, 0);
    ledcWrite(PWMR_TIMER_CHANN, 0);
    DBG_SERIAL.println(F("Motor OFF"));
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
  //DBG_SERIAL.print("prog_time func, prgNum: "); DBG_SERIAL.println(prgNum);
  //DBG_SERIAL.print("cyclesCount: "); DBG_SERIAL.println(cyclesCount);
  for(int i = 0; i < cyclesCount; i++) {
    ret += cycle_time(i);
    //DBG_SERIAL.print("cycle_time: "); DBG_SERIAL.println(ret);
  }
  return ret;
}

hw_timer_t *secondTimer = NULL;
void IRAM_ATTR onSecondTimer() {
  static uint8_t lCycleCount = 0;
  static uint16_t lCycleTime = 0, lPTime = 0, lRTime = 0, lDirection = FORWARD, lState = Work;

  Flag.SecondTimer = true;
  //static uint16_t cnt = 0;
  //DBG_SERIAL.print("onSecondTimer cnt: "); DBG_SERIAL.println(cnt++);
  //DBG_SERIAL.print("Flag.SecondTimer: "); DBG_SERIAL.println(Flag.SecondTimer);

  if(PastDelayCnt > 0) {
    PastDelayCnt--;
  }

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
  Param.SetPointValue[Past] = 63;
  Param.SetPointValue[Power] = 50;
  Param.SetPointValue[Temp] = 40;
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

void h_pushButt() {
  if(server.arg(F("buttID")) == "updFWBut") {
    t_httpUpdate_return ret = ESPhttpUpdate.update("http://192.168.1.62/firmware.bin");
  }
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
  myIP = WiFi.softAPIP();
  DBG_SERIAL.print(F("AP IP address: "));
  DBG_SERIAL.println(myIP);    
}

uint8_t readProg(uint8_t progNumber, ProgCycle *pc) {
  //DBG_SERIAL.print(F("Read prog: ")); DBG_SERIAL.println(prgList[progNumber]);
  String fn = F("/");
  if(prgList[progNumber].indexOf("\r") != MixManual) {
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
        //DBG_SERIAL.print(F("Cycle string: ")); Serial.println(line);
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

int readProgList(String file) {
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
  str.toCharArray(Text, (str.length() + 1));
  return Text;
}

char* intToCharArray(int val) {
  sprintf(Text, "%i", val);
  return Text;
}

char* getMPwrCharArr(int val) {
  sprintf(Text, "%i%%", val);
  return Text;
}

/*void cbNex_______(NextionEventType type, INextionTouchable *widget) {
  if (type == NEX_EVENT_PUSH) {}
  else if (type == NEX_EVENT_POP) {}
}*/
char* addDeg(char* s) {
  int i = strlen(s);
  s[i++] = '°';
  s[i++] = 'C';
  s[i] = 0x00;
  return s;
}

void showPastPageData() {
  nTxtUnit.setText(degSymbol);
  sprintf(Text, "%i", Param.SetPointValue[HeatingMode]);
  nTxtUVal.setText(Text);
  nTxtMilkT.setText(addDeg(dtostrf(Temperature[Milk], 3, 1, Text)));
  nTxtShirtT.setText(addDeg(dtostrf(Temperature[Shirt], 3, 1, Text)));
}

void nSendPrgName(int8_t val) {
  if(val >= 0) {
    nTxtProgName.setText(strToCharArray(prgList[val]));
  }
  else {
    nTxtProgName.setText("Manual");
  }
}

void nSendMotorPwr(uint8_t val) {
  nTxtMotorPwr.setText(getMPwrCharArr(val));
}

void nSendButMixStartStopTxt(String str) {
  nButMixStart.setText(strToCharArray(str));
}

void nSendIPAddress() {
  sprintf(Text, "%s", myIP.toString());
  String str = "http://" + myIP.toString() + "/index.html";
  str.toCharArray(Text, str.length() + 1);
  nSTxtAddr.setText(Text);
  nQrAddr.setText(Text);
}

void cbButMixStartStop(NextionEventType type, INextionTouchable *widget) {
  log("cbButMixStartStop");
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

void getTimeStrHMS(uint32_t t, char* txt) {
  uint8_t h = 0, m = 0, s = 0;
    s = t % 60;
    t /= 60;
    m = t % 60;
    h = t / 60;
    sprintf(Text, "%i:%02i:%02i", h, m, s);
}

void nSendProgTime() {
  getTimeStrHMS(prog_time(CurrProg), Text);
  if(CurrProg != MixManual) {
    nTxtProgTime.setText(Text);
  }
  else nTxtProgTime.setText("-:--:--");
}

void cbButPrgDec(NextionEventType type, INextionTouchable *widget) {
  if (type == NEX_EVENT_PUSH) {
    if((!Flag.MixProgEn) && (!Flag.MixManualEn)) {
      if(--CurrProg < MixManual) CurrProg = (prgCount - 1);
      if(CurrProg != MixManual) {
        cyclesCount = readProg(CurrProg, prgCycle);
      }
      else {
        nSendMotorPwr(MotorPower);
        if(OPower != MotorPower) {
          nSliMotorPwr.setValue(MotorPower);
          OPower = MotorPower;
        }
      }
      nSendPrgName(CurrProg);
      nSendProgTime();
    }
  }
  else if (type == NEX_EVENT_POP) {}
}

void cbButPrgInc(NextionEventType type, INextionTouchable *widget) {
  if (type == NEX_EVENT_PUSH) {
    if((!Flag.MixProgEn) && (!Flag.MixManualEn)) {
      if(++CurrProg == prgCount) CurrProg = MixManual;
      if(CurrProg != MixManual) {
        cyclesCount = readProg(CurrProg, prgCycle);
      }
      else {
        nSendMotorPwr(MotorPower);
        if(OPower != MotorPower) {
          nSliMotorPwr.setValue(MotorPower);
          OPower = MotorPower;
        }
      }
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

void cbPMixShow(NextionEventType type, INextionTouchable *widget) {
  if (type == NEX_EVENT_PUSH) {
    DBG_SERIAL.print(F("cbPMixShow, CurrProg: ")); DBG_SERIAL.println(CurrProg);
    nSendPrgName(CurrProg);
    nSliMotorPwr.setValue(MotorPower);
    nSendMotorPwr(MotorPower);
    nSendProgTime();
    NPage = NexPMix;
  }
  else if (type == NEX_EVENT_POP) {}
}

int8_t getNum(char* txt) {
  String str = txt;
  str.remove(str.length() - 1);
  return str.toInt();
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
    nTxtMotorPwr.getText(Text, 4);
    MotorPower = getNum(Text);
    if(Flag.MixManualEn) {
      motor_ctrl(ON, Direction, MotorPower);
    }
    DBG_SERIAL.print(F("Motor power: ")); DBG_SERIAL.println(MotorPower);
  }
}

void cbNexRButPast(NextionEventType type, INextionTouchable *widget) {
  if (type == NEX_EVENT_PUSH) {
    if (!Flag.HeatingEn) {
      HeatingMode = Past;
      nTxtHMode.setText("P");
      nTxtUnit.setText(degSymbol);
      sprintf(Text, "%i", Param.SetPointValue[HeatingMode]);
      nTxtUVal.setText(Text);
    }
  }
  else if (type == NEX_EVENT_POP) {}
}

void cbNexRButPower(NextionEventType type, INextionTouchable *widget) {
  if (type == NEX_EVENT_PUSH) {
    if (!Flag.HeatingEn) {
      HeatingMode = Power;
      nTxtHMode.setText("W");
      nTxtUnit.setText(percentSymbol);
      sprintf(Text, "%i", Param.SetPointValue[HeatingMode]);
      nTxtUVal.setText(Text);
    }
  }
  else if (type == NEX_EVENT_POP) {}
}

void cbNexRButTemp(NextionEventType type, INextionTouchable *widget) {
  if (type == NEX_EVENT_PUSH) {
    if (!Flag.HeatingEn) {
      HeatingMode = Temp;
      nTxtHMode.setText("t");
      nTxtUnit.setText(degSymbol);
      sprintf(Text, "%i", Param.SetPointValue[HeatingMode]);
      nTxtUVal.setText(Text);
    }
  }
  else if (type == NEX_EVENT_POP) {}
}

void cbNexButHeatUp(NextionEventType type, INextionTouchable *widget) {
  if (type == NEX_EVENT_PUSH) {
    if(Param.SetPointValue[HeatingMode] < 100) Param.SetPointValue[HeatingMode]++;
    sprintf(Text, "%i", Param.SetPointValue[HeatingMode]);
    nTxtUVal.setText(Text);
  }
  else if (type == NEX_EVENT_POP) {}
}

void cbNexButHeatDown(NextionEventType type, INextionTouchable *widget) {
  if (type == NEX_EVENT_PUSH) {
    if(Param.SetPointValue[HeatingMode] > 0) Param.SetPointValue[HeatingMode]--;
    sprintf(Text, "%i", Param.SetPointValue[HeatingMode]);
    nTxtUVal.setText(Text);
  }
  else if (type == NEX_EVENT_POP) {}
}

void cbHeatStart(NextionEventType type, INextionTouchable *widget) {
  if (type == NEX_EVENT_PUSH) {
    if(!Flag.HeatingEn) {
      DBG_SERIAL.println(F("Start heating!"));
      Flag.HeatingEn = true;
      nButHeatStart.setText("Stop");
      nTxtShirtT.setForegroundColour(NEX_COL_RED);
      switch (HeatingMode) {
        case Past: {
          DBG_SERIAL.println(F("Heating mode - PAST"));
          PastState = PastStateHeating;
          PIDSetpoint = (double)Param.SetPointValue[HeatingMode];
          consK = false;
        } break;  
        case Power: {
          DBG_SERIAL.println(F("Heating mode - POWER"));
          heating_ctrl(Param.SetPointValue[Power]); break;  
        }
        case Temp: {
          DBG_SERIAL.println(F("Heating mode - TEMP"));
          PIDSetpoint = (double)Param.SetPointValue[HeatingMode];
          consK = false;
        } break; 
        default: break;
      }
    }
    else {
      DBG_SERIAL.println(F("Stop heating!"));
      Flag.HeatingEn = false;
      nButHeatStart.setText("Start");
      nTxtShirtT.setForegroundColour(NEX_COL_BLACK);
      nTxtHeatState.setText("---------");
      nTxtHeatStateA.setText("---------");
      heating_ctrl(0);
      /*switch (HeatingMode) {
        case Past: {} break;  
        case Power: {} break;   
        case Temp: {} break; 
        default: break;
      }*/
      save_param();
    }
  }
  else if (type == NEX_EVENT_POP) {}
}

void cbPHeatShow(NextionEventType type, INextionTouchable *widget) {
  if (type == NEX_EVENT_PUSH) {
    NPage = NexPHeat;
  }
  else if (type == NEX_EVENT_POP) {}
}

void cbPSettShow(NextionEventType type, INextionTouchable *widget) {
  if (type == NEX_EVENT_PUSH) {
    NPage = NexPSett;
    nSendIPAddress();
  }
  else if (type == NEX_EVENT_POP) {}
}

void i2c_scan() {
  byte error, address;
  int nDevices;
  DBG_SERIAL.println("");
  DBG_SERIAL.println(F("Scanning..."));
  nDevices = 0;
  for(address = 1; address < 127; address++ ) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0) {
      DBG_SERIAL.print(F("I2C device found at address 0x"));
      if (address<16) {
        Serial.print(F("0"));
      }
      DBG_SERIAL.println(address,HEX);
      nDevices++;
    }
    else if (error==4) {
      DBG_SERIAL.print(F("Unknow error at address 0x"));
      if (address<16) {
        DBG_SERIAL.print(F("0"));
      }
      DBG_SERIAL.println(address,HEX);
    }    
  }
  if (nDevices == 0) {
    DBG_SERIAL.println(F("No I2C devices found\n"));
  }
  else {
    DBG_SERIAL.println(F("done\n"));
  }
}

void pidKSelect() {  
  double gap = abs(PIDSetpoint - PIDInput); //distance away from setpoint
  if (gap < 5.0) {  //we're close to setpoint, use conservative tuning parameters
    m_PID.SetTunings(consKp, consKi, consKd);      
    consK = true;
  }
  else {
    //we're far from setpoint, use aggressive tuning parameters
    m_PID.SetTunings(aggKp, aggKi, aggKd);
  }
}

void Task1code( void * pvParameters ) {
  DBG_SERIAL.print("Task1 running on core ");
  DBG_SERIAL.println(xPortGetCoreID());
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
      myIP = WiFi.localIP();
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

  prgCount = readProgList(F("ProgList.txt"));
  DBG_SERIAL.print(F("Read "));
  DBG_SERIAL.print(prgCount);
  DBG_SERIAL.println(F(" prog"));

  for(uint i = 0; i < prgCount; i++) {
    DBG_SERIAL.print(i + 1); DBG_SERIAL.print(F(" - ")); DBG_SERIAL.println(prgList[i]); 
  }

  // Регистрация обработчиков
  server.on(F("/"), h_Website);    
  server.on(F("/wifi_param"), h_wifi_param); 
  server.on(F("/pushButt"), h_pushButt);    
  server.onNotFound(handleWebRequests);       
  server.on(F("/xml"), h_XML);
  server.begin();

  /* Nextion HEATING page callbacks */
  nTxtPMixH.attachCallback(&cbPMixShow);
  nTxtPSettH.attachCallback(&cbPSettShow);
  nRButPast.attachCallback(&cbNexRButPast);
  nRButPower.attachCallback(&cbNexRButPower);
  nRButTemp.attachCallback(&cbNexRButTemp);
  nButHeatUp.attachCallback(&cbNexButHeatUp);
  nButHeatDown.attachCallback(&cbNexButHeatDown);
  nButHeatStart.attachCallback(&cbHeatStart);

  /* Nextion MIX page callbacks */
  nTxtPHeatM.attachCallback(&cbPHeatShow);
  nTxtPSettM.attachCallback(&cbPSettShow);
  nButCCRot.attachCallback(&cbButCCRot);
  nButCRot.attachCallback(&cbButCRot);
  nButPrgDec.attachCallback(&cbButPrgDec);
  nButPrgInc.attachCallback(&cbButPrgInc);
  nCropMPwrDec.attachCallback(&cbCropMPwrDec);
  nCropMPwrInc.attachCallback(&cbCropMPwrInc);
  nSliMotorPwr.attachCallback(&cbSliMotorPwr);
  nButMixStart.attachCallback(&cbButMixStartStop);

  /* Nextion SETT page callbacks */
  nTxtPHeatS.attachCallback(&cbPHeatShow);
  nTxtPMixS.attachCallback(&cbPMixShow);
  nTxtPSettS.attachCallback(&cbPSettShow);

  nTxtHMode.setText("P");
  showPastPageData();

  for(;;){
    /* Обработка запросов HTML клиента */
    server.handleClient();
    /* Обработка запросов FTP клиента */
    ftpSrv.handleFTP();
    /* Обработка сообщений Nextion дисплея */
    nex.poll();
    delay(50);
  }
}

void setup() {
  // put your setup code here, to run once:
  DBG_SERIAL.begin(DBG_BAUD);

  NEX_SERIAL.begin(NEX_BAUD);
  NEX_SERIAL.setRxBufferSize(100);
  nex.init();
  
  Wire.begin(SDA, SCL, 400000);
  i2c_scan();
  adcTemp.begin();
  adcTemp.setGain(GAIN_FOUR);

  pinMode(2, OUTPUT);
  pinMode(ENL_MOTOR_PIN, OUTPUT);
  pinMode(ENR_MOTOR_PIN, OUTPUT);
  pinMode(PWML_MOTOR_PIN, OUTPUT);
  pinMode(PWMR_MOTOR_PIN, OUTPUT);
  pinMode(PWM_RELAY_PIN, OUTPUT);

  sblink(20, 20);

  //eeprom_init();
  //save_param();
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

  addDeg(degSymbol);
  HeatingMode = Past;
  PIDSetpoint = (double)Param.SetPointValue[HeatingMode];

  //turn the PID on
  m_PID.SetSampleTime(1000);
  m_PID.SetOutputLimits(0.0, 100.0);
  m_PID.SetMode(AUTOMATIC);

  // Создаем задачу с кодом из функции Task1code(),
  // с приоритетом 1 и выполняемую на ядре 0:
  xTaskCreatePinnedToCore(
                    Task1code,   /* Функция задачи */
                    "Task1",     /* Название задачи */
                    50000,       /* Размер стека задачи */
                    NULL,        /* Параметр задачи */
                    1,           /* Приоритет задачи */
                    &Task1,      /* Идентификатор задачи, чтобы ее можно было отслеживать */
                    0);          /* Ядро для выполнения задачи (0) */                  
  delay(500);
}

void loop() {
  if(Flag.SecondTimer) {
    Flag.SecondTimer = false;
    
    Temperature[Milk] = adcTemp.readADC_SingleEnded(Milk) * 0.003125;
    Temperature[Shirt] = adcTemp.readADC_SingleEnded(Shirt) * 0.003125;   
    PIDInput = Temperature[Milk];

    /* Heating control ***********************************************************************************/ 
    if(Flag.HeatingEn) {
      switch (HeatingMode) {
        case Past: {
          pidKSelect();
          m_PID.Compute();
          HeatingPWM = (uint8_t)PIDOutput;
          switch (PastState) {
            case PastStateHeating : {
              if(Temperature[Milk] >= (double)Param.SetPointValue[Past]) {
                PastDelayCnt = 30;
                PastState = PastStateDelay;
              }
            } break;
            case PastStateDelay : {
              if(PastDelayCnt == 0) {
                HeatingPWM = 0;
                Flag.HeatingEn = false;
              }
            } break;        
            default: break;
          }
        } break;  

        case Power:{
          heating_ctrl(Param.SetPointValue[HeatingMode]);
        } break;  

        case Temp:{
          pidKSelect();
          m_PID.Compute();
          HeatingPWM = (uint8_t)PIDOutput;
          if(Temperature[Milk] >= Param.SetPointValue[Temp]) {
            HeatingPWM = 0;
            Flag.HeatingEn = false;
          }
        } break; 

      default: break;
      }
    }
    /* Heating control ***********************************************************************************/  
  }
  /* Display ********************************************************************************************/
  if(Flag.HSecond) {
    Flag.HSecond = false;
    switch(NPage) {
      case NexPHeat: {  // Heating display
        nTxtMilkT.setText(addDeg(dtostrf(Temperature[Milk], 3, 1, Text)));
        nTxtShirtT.setText(addDeg(dtostrf(Temperature[Shirt], 3, 1, Text))); 
        if(Flag.HeatingEn) {
          sprintf(Text, "%i%%%s", HeatingPWM, (HeatingMode == Power)?("   "):((consK)?("(c)"):("(a)")));
          nTxtHeatPwr.setText(Text);
          if(HeatingMode == Past) {
            if(PastState == PastStateHeating) nTxtHeatState.setText("Heating");
            if(PastState == PastStateDelay) {
              nTxtHeatState.setText("Delay");
              getTimeStrHMS(PastDelayCnt, Text);
              nTxtHeatStateA.setText(Text);
            }
          }
          if((HeatingMode == Power) || (HeatingMode == Temp)){
            nTxtHeatState.setText("Heating");
            nTxtHeatStateA.setText("---------");
          }   
        }
        else {
          if((PastState == PastStateDelay) || (HeatingMode == Power) || (HeatingMode == Temp)) {
            nTxtHeatState.setText("---------");
            nTxtHeatStateA.setText("---------");
            nTxtHeatPwr.setText("-----");
            nButHeatStart.setText("Start");
            PastState = PastStateIdle;
          }
        } 
      } break;
      case NexPMix: { // Mixer display
        if((Flag.MixProgEn) || (Flag.EndProg)) {      
          sprintf(Text, "%i/%s", (CurrCycle + 1), ((prgCycle[CurrCycle].State == 1)?("W"):("P")));
          nTxtCycleNum.setText(Text);
          if(cycle_time(CurrCycle) < 60) sprintf(Text, "0:%02i", CurrCycleTime);
          else sprintf(Text, "%i:%02i", (CurrCycleTime / 60), (CurrCycleTime % 60));
          nTxtCycleTime.setText(Text);     
          getTimeStrHMS(ProgTime, Text);
          nTxtProgTime.setText(Text);
          nSendMotorPwr(prgCycle[CurrCycle].Power);
          if(OPower != prgCycle[CurrCycle].Power) {
            nSliMotorPwr.setValue(prgCycle[CurrCycle].Power);
            OPower = prgCycle[CurrCycle].Power;
          }
          if(Flag.EndProg) {
            Flag.EndProg = false;
            nButMixStart.setText("Start");
            nTxtCycleNum.setText("--/-");
            nTxtCycleTime.setText("--:--");
            nSendProgTime();
          }
        }
        else if(Flag.MixManualEn) {
          nSendMotorPwr(MotorPower);
          if(OPower != MotorPower) {
            nSliMotorPwr.setValue(MotorPower);
            OPower = MotorPower;
          }
        }
      } break;
      case NexPSett: {

      } break;
      default: break;
    }
  }
  /* Display ********************************************************************************************/
}