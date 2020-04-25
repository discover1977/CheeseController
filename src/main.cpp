#include <Arduino.h>
#include <WiFi.h>
#include <ESP8266FtpServer.h>
#include <ESP32httpUpdate.h>
#include <FS.h>
#include <SPIFFS.h>
#include <WebServer.h>
#include <EEPROM.h>
#include <Nextion.h>
#include <NextionNumber.h>
#include <INextionNumericalValued.h>
#include <NextionText.h>
#include <NextionButton.h>
#include <NextionDualStateButton.h>
#include <NextionRadioButton.h>
#include <NextionSlidingText.h>
#include <NextionSlider.h>
#include <NextionCrop.h>
#include <NextionVariableString.h>
#include <Adafruit_ADS1015.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <ESPNexUpload.h>

#include <NTPClient.h>
#include <WiFiUdp.h>

#define SCREEN_WIDTH    128 // OLED display width, in pixels
#define SCREEN_HEIGHT   32 // OLED display height, in pixels
#define OLED_RESET      15 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 oled(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire);

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

#define OUT_1 32
#define OUT_2 33
#define OUT_3 25
#define OUT_4 26

#define PWML_MOTOR_PIN 27
#define PWMR_MOTOR_PIN 14
#define EN_MOTOR_PIN 12
#define PWM_SSR_PIN 13

#define PWM_FREQ  20000
#define PWML_TIMER_CHANN  0
#define PWMR_TIMER_CHANN  1
#define PWM_RES_BIT 8

#define OLOG_CLS_DELAY  10

TaskHandle_t WiFiCoreTaskHandle;

void WiFiCodeTask(void* param);
void OLEDMsgCodeTask(void* param);

const String TEMP_LOG_FN = "/temp_log.csv";

enum NexPage {
  NexPNone = -1,
  NexPHeat,
  NexPMix,
  NexPSett
};

#define TIME_OFFSET (3600 * 3)
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "europe.pool.ntp.org", TIME_OFFSET, 60000);
Adafruit_ADS1115  adcTemp;

Nextion nex(NEX_SERIAL);
/* Past page objects */
NextionText nTxtPHeatH(nex, NexPHeat, 17, "tPHeatH");
NextionText nTxtPMixH(nex, NexPHeat, 18, "tPMixH");
NextionText nTxtPSettH(nex, NexPHeat, 19, "tPSettH");

NextionVariableString nVarTime(nex, NexPHeat, 23, "vTimeStr");
NextionText nTxtTimePHeat(nex, NexPHeat, 16, "tTime");
NextionText nTxtHeatState(nex, NexPHeat, 1, "tHeatState");
NextionText nTxtHeatStateA(nex, NexPHeat, 23, "tHeatStateA");
//NextionText nTxtPState(nex, NexPHeat, 1, "tPState");
NextionText nTxtMilkT(nex, NexPHeat, 2, "tMilk_t");
NextionText nTxtShirtT(nex, NexPHeat, 3, "tShirt_t");
NextionText nTxtHMode(nex, NexPHeat, 13, "tHMode");
NextionText nTxtUVal(nex, NexPHeat, 15, "tUVal");
NextionText nTxtUnit(nex, NexPHeat, 14, "tUnit");
NextionText nTxtHeatPwr(nex, NexPHeat, 21, "tHeatPwr");
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

NextionText nTxtTimePMix(nex, NexPMix, 1, "tTime");
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

NextionText nTxtTimePSett(nex, NexPSett, 1, "tTime");
//NextionSlidingText nSTxtAddr(nex, NexPSett, 8, "stAddr");
NextionText nTxtStatus(nex, NexPSett, 7, "tStatus");
NextionButton nButOut1(nex, NexPSett, 12, "bOut1");
NextionButton nButOut2(nex, NexPSett, 13, "bOut2");
NextionButton nButOut3(nex, NexPSett, 14, "bOut3");
NextionButton nButOut4(nex, NexPSett, 15, "bOut4");
NextionButton nButFWUpd(nex, NexPSett, 8, "bFWUpd");
NextionButton nButGUIUpd(nex, NexPSett, 9, "bGUIUpd");
NextionButton nButReset(nex, NexPSett, 11, "bRst");

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

//TaskHandle_t LogicCoreHandle, GUICoreHandle;
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
uint16_t OLogClsTimer = OLOG_CLS_DELAY;
uint8_t OLogClsCnt = 0;

/* Pasteurizer variables */
enum TempEnum {
  Milk,
  Shirt
};

double Temperature[2] = {0, 0};
enum PIDEnum {
  Agg,
  Cons
};
struct PIDData {
  double kP[2], kI[2], kD[2];
} PIDData;

bool consK = false;
uint16_t PastDelayCnt;

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
  bool OLogCls;
  bool tcUpd;
} Flag;

float constrainF(float val, float min, float max) {
	if(val < min) return min;
	else if(val > max) return max;
	else return val;
}

volatile float pid_output;
#define MY_PID 1

uint8_t pid(float Set, float Value, float Kp, float Ki, float Kd) {
	// Формула ПИД-регулятора, используемая для расчета

	// Дискретная
	// u(t) = P (t) + I (t) + D (t);
	// P (t) = Kp * e (t);
	// I (t) = I (t — 1) + Ki * e (t);
	// D (t) = Kd * (e (t) — e (t — 1));

	// Рекуррентная
	// u(t) = u(t — 1) + P (t) + I (t) + D (t);
	// P (t) = Kp * {e (t) — e (t — 1)};
	// I (t) = I * e (t);
	// D (t) = Kd * {e (t) — 2 * e (t — 1) + e (t — 2)};

	float pid_error;
	static float Pr;
	static float In = 0;
	static float Df;
	static float err_old;

#define DEV 1.0F

	/* Calc current error */
	pid_error = Set - Value;

#if MY_PID
	/* Proportional */
	Pr = (Kp / DEV) * pid_error;

	/* Integral */
	In = (In / DEV) + (Ki * pid_error);
	In = constrainF(In, 0.0, 100.0);

	/* Differential */
	Df = (Kd / DEV) * (pid_error - err_old);
	err_old = pid_error;
	pid_output = constrainF((Pr + In + Df), 0.0, 100.0);
#else

#define PID_FUNCTIONAL_RANGE 10 // If the temperature difference between the target temperature and the actual temperature
                                // is more than PID_FUNCTIONAL_RANGE then the PID will be shut off and the heater will be set to min/max.
#define BANG_MAX 100     // Limits current to nozzle while in bang-bang mode; 255=full current
#define PID_MAX BANG_MAX // Limits current to nozzle while PID is active (see PID_FUNCTIONAL_RANGE below); 255=full current
#define PID_K1 0.95f      // Smoothing factor within any PID loop
#define PID_K2	(1.0f - PID_K1)

#define false 	0
#define true	1

	static uint8_t pid_reset = false;
  static float temp_dState = 0;
	static float temp_iState;

	Df = PID_K2 * Kd * (Value - temp_dState) + PID_K1 * Df;
	temp_dState = Value;

	if (pid_error > PID_FUNCTIONAL_RANGE) {
		pid_output = BANG_MAX;
		pid_reset = true;
	}
    else if (pid_error < -(PID_FUNCTIONAL_RANGE) || Set == 0) {
		pid_output = 0;
		pid_reset = true;
    }
    else {
		if (pid_reset == true) {
			temp_iState = 0.0;
			pid_reset = false;
		}
		Pr = Kp * pid_error;
		temp_iState += pid_error;
		In = Ki * temp_iState;

		pid_output = Pr + In - Df;

		if (pid_output > PID_MAX) {
			if (pid_error > 0) temp_iState -= pid_error; // conditional un-integration
			pid_output = PID_MAX;
		}
		else if (pid_output < 0) {
			if (pid_error < 0) temp_iState -= pid_error; // conditional un-integration
				pid_output = 0;
		}
	}

#endif

	return (uint8_t)(pid_output);
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

bool remove_flog(String path) {
  if(SPIFFS.exists(path)) {
    SPIFFS.remove(path);
    return true;
  }
  else return false;
}

bool flog(String path, String logStr, bool newFile = false) {  
  File f = SPIFFS.open(path, (newFile)?(FILE_WRITE):(FILE_APPEND));
  if (!f) {
    Serial.println(F("File open failed."));
    return false;
    led_ctrl(LOW);
  } 
  else {
    led_ctrl(HIGH);
    // Serial.println(String("File size: ") + f.size());
    if(f.size() > 100000) {
      f.close();
      SPIFFS.remove(path);
      File f = SPIFFS.open(path, FILE_WRITE);
      f.println(logStr);
    }    
    f.println(logStr);
    f.flush();
    f.close();
    led_ctrl(LOW);
    return true;    
  }
}

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

#define OLOG_LINES 4
void olog(String str) {
  static String lastStr[OLOG_LINES];
  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setTextColor(SSD1306_WHITE);

  OLogClsTimer = OLOG_CLS_DELAY;
  OLogClsCnt = 0;

  for(int i = 0; i < OLOG_LINES; i++) {
    if(i < (OLOG_LINES - 1)) lastStr[i] = lastStr[i + 1];
    else lastStr[i] = str;
    oled.setCursor(0, (i * 8));
    oled.println(lastStr[i]);
  }

  oled.display();
}

hw_timer_t *relayTimer = NULL;
void IRAM_ATTR onRelayTimer() {
  static uint8_t hsCnt = 0;
  static uint8_t pwmCnt = 0;
  if(pwmCnt < HeatingPWM) {
    digitalWrite(PWM_SSR_PIN, HIGH);
  }
  else {
    digitalWrite(PWM_SSR_PIN, LOW);
  }
  if(++pwmCnt > 100) {
    pwmCnt = 0;    
    // OLOG clear display
    if(OLogClsTimer > 0) OLogClsTimer--;

    if((OLogClsTimer == 0) && (OLogClsCnt < 4)) {
      Flag.OLogCls = true;
      OLogClsCnt++;
      OLogClsTimer = OLOG_CLS_DELAY;
    }
  }

  if(++hsCnt == 50) {
    hsCnt = 0;
    Flag.HSecond = true;
    Flag.tcUpd = true;
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

enum MixerState{
	MixerStop,
	MixerWork,
  MixerPause
};

void out_ctrl(uint8_t enable, uint8_t outnum) {
  switch (outnum) {
    case 1 : {
      if(enable == ON) digitalWrite(OUT_1, ON);
      else digitalWrite(OUT_1, OFF);
    } break;  
    case 2 : {
      if(enable == ON) digitalWrite(OUT_2, ON);
      else digitalWrite(OUT_2, OFF);
    } break; 
    case 3 : {
      if(enable == ON) digitalWrite(OUT_3, ON);
      else digitalWrite(OUT_3, OFF);
    } break; 
    case 4 : {
      if(enable == ON) digitalWrite(OUT_4, ON);
      else digitalWrite(OUT_4, OFF);
    } break;     
    default: {
      digitalWrite(OUT_1, OFF);
      digitalWrite(OUT_2, OFF);
      digitalWrite(OUT_3, OFF);
      digitalWrite(OUT_4, OFF);
    } break;
  }
}

uint8_t out_state(uint8_t outnum) {
  return digitalRead(outnum);
}

void heating_ctrl(uint8_t val) {
  if(val > 100) return;
  HeatingPWM = val;
}

void motor_ctrl(uint8_t enable, uint8_t direct, uint8_t power) {
	if(enable == ON) {
    digitalWrite(EN_MOTOR_PIN, HIGH);
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
    digitalWrite(EN_MOTOR_PIN, LOW);
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
  static uint16_t lCycleTime = 0, lPTime = 0, lRTime = 0, lDirection = FORWARD, lState = MixerWork;
  static uint16_t wifiOffDelay = 30;

  Flag.SecondTimer = true;

  if(PastDelayCnt > 0) {
    PastDelayCnt--;
  }

	if(Flag.InitLocalTimerVar) {
		Flag.InitLocalTimerVar = 0;
		lPTime = 0;
		lRTime = 0;
		lDirection = FORWARD;
		lState = MixerWork;
		lCycleCount = 0;
    CurrCycleTime = cycle_time(lCycleCount);
    CurrCycle = 0;
    ProgTime = prog_time(CurrProg);
	}

  /* Mixer motor control ************************************************/
  if(Flag.MixProgEn) {
		if(lCycleCount < cyclesCount) {
			if(prgCycle[lCycleCount].State == MixerWork) {
				if(lState == MixerWork) {
					motor_ctrl(ON, lDirection, prgCycle[lCycleCount].Power);   
					if(++lRTime == prgCycle[lCycleCount].OWTime) {
						lRTime = 0;
						lState = MixerPause;
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
						lState = MixerWork;
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
			lState = MixerWork;
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

void fw_update() {
  led_ctrl(HIGH);  
  log(F("Update FW"));
  nTxtStatus.setText((char*)"Update FW...");
  olog(F("Update FW..."));
  // yield();
  t_httpUpdate_return ret = ESPhttpUpdate.update("https://discover1977.github.io/fw/chc/firmware.bin", "", "", false);
  led_ctrl(LOW);
  switch(ret) {
    case HTTP_UPDATE_FAILED:
      sprintf(Text, "FW_UPDATE_FAILD Error (%d): %s", ESPhttpUpdate.getLastError(), ESPhttpUpdate.getLastErrorString().c_str());
      nTxtStatus.setText(Text);
      DBG_SERIAL.println(Text);
      break;

    case HTTP_UPDATE_NO_UPDATES:
      nTxtStatus.setText((char*)"FW_UPDATE_NO_UPDATES");
      DBG_SERIAL.println(F("FW_UPDATE_NO_UPDATES"));
      break;

    case HTTP_UPDATE_OK:
      nTxtStatus.setText((char*)"FW_UPDATE_OK, please reboot");
      DBG_SERIAL.println(F("FW_UPDATE_OK"));
      break;
  }
}

void gui_update() {
  led_ctrl(HIGH);  
  log((char*)"Update GUI");
  nTxtStatus.setText((char*)"Update GUI...");
  olog(F("GUI update..."));
  
  HTTPClient http;
  
  // begin http client
  #if defined ESP8266
    if(!http.begin(host, 80, url)){
  #elif defined ESP32
    if(!http.begin("https://discover1977.github.io/fw/chc/chc_hmi.tft")) {
  #endif
    DBG_SERIAL.println("connection failed");
    olog(F("connection failed"));
    return;
  }
  
  // This will send the (get) request to the server
  int code = http.GET();
  DBG_SERIAL.print("http.GET(): "); DBG_SERIAL.println(code);
  int contentLength = http.getSize();
  DBG_SERIAL.print("http.getSize(): "); DBG_SERIAL.println(contentLength);
    
  // Update the nextion display
  if(code == 200) {
    DBG_SERIAL.println("File received. Update Nextion...");
    olog(F("File received"));
    olog(F("Update Nextion..."));

    bool result;

    NEX_SERIAL.flush();
    NEX_SERIAL.end();
    delay(500);

    // initialize ESPNexUpload
    ESPNexUpload nextion(115200);

    // set callback: What to do / show during upload..... Optional!
    nextion.setUpdateProgressCallback([]() {
      DBG_SERIAL.print(".");
    });
    
    // prepare upload: setup serial connection, send update command and send the expected update size
    result = nextion.prepareUpload(contentLength);
    
    if(!result){
        DBG_SERIAL.println("Error: " + nextion.statusMessage);
        olog(String("Error: ") + nextion.statusMessage);
    }
    else {
        DBG_SERIAL.print(F("Start upload. File size is: "));
        olog(F("Start upload"));
        olog(String("File size: ") + contentLength);
        DBG_SERIAL.print(contentLength);
        DBG_SERIAL.println(F(" bytes"));
        
        // Upload the received byte Stream to the nextion
        result = nextion.upload(*http.getStreamPtr());
        
        if(result){
          // updated = true;
          DBG_SERIAL.println("Succesfully updated Nextion!");
          olog(F("Succesfully updated!"));
          delay(5000);
          esp_restart();
        }
        else {
          DBG_SERIAL.println("Error updating Nextion: " + nextion.statusMessage);
          olog(F("Error updating"));
          olog(String("") + nextion.statusMessage);
          delay(5000);
          esp_restart();
        }
        // end: wait(delay) for the nextion to finish the update process, send nextion reset command and end the serial connection to the nextion
        nextion.end();
    }
  }
  else {
    // else print http error
    DBG_SERIAL.println(String("HTTP error: ") + http.errorToString(code).c_str());
    olog(String("HTTP error: ") + http.errorToString(code).c_str());
  }

  http.end();
  DBG_SERIAL.println("Closing connection\n");
  olog(F("Closing connection"));
  led_ctrl(LOW);  
  delay(5000);
  esp_restart();
}

void h_pushButt() {  
  if(server.arg(F("buttID")) == "updFWBut") {  
    fw_update();
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
  olog(F("Wi-Fi AP mode"));
  DBG_SERIAL.println(F("AP configuring..."));
  olog(F("AP configuring..."));
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ap_ssid, ap_pass); 
  DBG_SERIAL.println(F("done"));
  olog(F("done"));
  myIP = WiFi.softAPIP();
  DBG_SERIAL.print(F("AP IP address: "));
  olog(F("AP IP address: "));
  DBG_SERIAL.println(myIP);   
  olog(myIP.toString());
}

uint8_t readProg(uint8_t progNumber, ProgCycle *pc) {
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

void parsePID_line(String str, uint8_t set) { 
  uint8_t si = 1, ei = 0;
  ei = str.indexOf("i", si);
  PIDData.kP[set] = str.substring(si, ei).toDouble();

  si = ei + 1;
  ei = str.indexOf("d", si);
  PIDData.kI[set] = str.substring(si, ei).toDouble();

  si = ei + 1;
  ei = str.length() - 1;
  PIDData.kD[set] = str.substring(si, ei).toDouble();
}

void readPID(String file) {
  File f = SPIFFS.open(file, "r");
  uint8_t i = 0;
  if (!f) {
    DBG_SERIAL.println(F("PID file open failed."));
    //olog(F("PID file open failed."));
  } else {
    while(f.available()) {
      String line = f.readStringUntil('\n');
      //olog(line);
      parsePID_line(line, i++);     
    } 
    f.close();
  }
}

int readProgList(String file) {
  uint8_t cnt = 0;  
  File f = SPIFFS.open(file, "r");
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
    nTxtProgName.setText((char*)"Manual");
  }
}

void nSendMotorPwr(uint8_t val) {
  nTxtMotorPwr.setText(getMPwrCharArr(val));
}

void nSendButMixStartStopTxt(String str) {
  nButMixStart.setText(strToCharArray(str));
}

void nSendIPAddress() {
  String str = "http://" + myIP.toString() + "/index.html";
  str.toCharArray(Text, str.length() + 1);
  nTxtStatus.setText(Text);
}

void cbButFWUpd(NextionEventType type, INextionTouchable *widget) {
  NEX_SERIAL.flush();
  if (type == NEX_EVENT_PUSH) {
    fw_update();
  }
  else if (type == NEX_EVENT_POP) {}
}

void cbButGUIUpd(NextionEventType type, INextionTouchable *widget) {
  NEX_SERIAL.flush();
  if (type == NEX_EVENT_PUSH) {
    gui_update();
  }
  else if (type == NEX_EVENT_POP) {}
}

bool OutState[4];
bool OutStatePr[4];
void cbButOut1(NextionEventType type, INextionTouchable *widget) {
  NEX_SERIAL.flush();
  olog(F("cbButOut1"));
  if (type == NEX_EVENT_PUSH) {
    if(!OutState[0]) {
      out_ctrl(ON, 1);
      OutState[0] = true;
    }
    else {
      out_ctrl(OFF, 1);
      OutState[0] = false;
    }
  }
  else if (type == NEX_EVENT_POP) {}
}

void cbButOut2(NextionEventType type, INextionTouchable *widget) {
  NEX_SERIAL.flush();
  olog(F("cbButOut2"));
  if (type == NEX_EVENT_PUSH) {
    if(!OutState[1]) {
      out_ctrl(ON, 2);
      OutState[1] = true;
    }
    else {
      out_ctrl(OFF, 2);
      OutState[1] = false;
    }
  }
  else if (type == NEX_EVENT_POP) {}
}

void cbButOut3(NextionEventType type, INextionTouchable *widget) {
  NEX_SERIAL.flush();
  olog(F("cbButOut3"));
  if (type == NEX_EVENT_PUSH) {
    if(!OutState[2]) {
      out_ctrl(ON, 3);
      OutState[2] = true;
    }
    else {
      out_ctrl(OFF, 3);
      OutState[2] = false;
    }
  }
  else if (type == NEX_EVENT_POP) {}
}

void cbButOut4(NextionEventType type, INextionTouchable *widget) {
  NEX_SERIAL.flush();
  olog(F("cbButOut4"));
  if (type == NEX_EVENT_PUSH) {
    if(!OutState[3]) {
      out_ctrl(ON, 4);
      OutState[3] = true;
    }
    else {
      out_ctrl(OFF, 4);
      OutState[3] = false;
    }
  }
  else if (type == NEX_EVENT_POP) {}
}

void cbButReset(NextionEventType type, INextionTouchable *widget) {
  NEX_SERIAL.flush();
  if (type == NEX_EVENT_PUSH) {
    int i = 5;
    while (i){
      sprintf(Text, "Reboot after: %i second", i--);
      nTxtStatus.setText(Text);
      delay(1000);
    }
    sprintf(Text, "Reboot after: %i second", i--);
    nTxtStatus.setText(Text);
    delay(100);
    esp_restart();
  }
  else if (type == NEX_EVENT_POP) {}
}

void cbButMixStartStop(NextionEventType type, INextionTouchable *widget) {
  NEX_SERIAL.flush();
  log((char*)"cbButMixStartStop");
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
  else nTxtProgTime.setText((char*)"-:--:--");
}

void cbButPrgDec(NextionEventType type, INextionTouchable *widget) {
  NEX_SERIAL.flush();
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
  NEX_SERIAL.flush();
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
  NEX_SERIAL.flush();
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
  NEX_SERIAL.flush();
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
  NEX_SERIAL.flush();
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
  NEX_SERIAL.flush();
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
  NEX_SERIAL.flush();
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
  NEX_SERIAL.flush();
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
  NEX_SERIAL.flush();
  if (type == NEX_EVENT_PUSH) {
    if (!Flag.HeatingEn) {
      HeatingMode = Past;
      nTxtHMode.setText((char*)"P");
      nTxtUnit.setText(degSymbol);
      sprintf(Text, "%i", Param.SetPointValue[HeatingMode]);
      nTxtUVal.setText(Text);
    }
  }
  else if (type == NEX_EVENT_POP) {}
}

void cbNexRButPower(NextionEventType type, INextionTouchable *widget) {
  NEX_SERIAL.flush();
  if (type == NEX_EVENT_PUSH) {
    if (!Flag.HeatingEn) {
      HeatingMode = Power;
      nTxtHMode.setText((char*)"W");
      nTxtUnit.setText(percentSymbol);
      sprintf(Text, "%i", Param.SetPointValue[HeatingMode]);
      nTxtUVal.setText(Text);
    }
  }
  else if (type == NEX_EVENT_POP) {}
}

void cbNexRButTemp(NextionEventType type, INextionTouchable *widget) {
  NEX_SERIAL.flush();
  if (type == NEX_EVENT_PUSH) {
    if (!Flag.HeatingEn) {
      HeatingMode = Temp;
      nTxtHMode.setText((char*)"t");
      nTxtUnit.setText(degSymbol);
      sprintf(Text, "%i", Param.SetPointValue[HeatingMode]);
      nTxtUVal.setText(Text);
    }
  }
  else if (type == NEX_EVENT_POP) {}
}

void cbNexButHeatUp(NextionEventType type, INextionTouchable *widget) {
  NEX_SERIAL.flush();
  if (type == NEX_EVENT_PUSH) {
    if(Param.SetPointValue[HeatingMode] < 100) Param.SetPointValue[HeatingMode]++;
    sprintf(Text, "%i", Param.SetPointValue[HeatingMode]);
    nTxtUVal.setText(Text);
  }
  else if (type == NEX_EVENT_POP) {}
}

void cbNexButHeatDown(NextionEventType type, INextionTouchable *widget) {
  NEX_SERIAL.flush();
  if (type == NEX_EVENT_PUSH) {
    if(Param.SetPointValue[HeatingMode] > 0) Param.SetPointValue[HeatingMode]--;
    sprintf(Text, "%i", Param.SetPointValue[HeatingMode]);
    nTxtUVal.setText(Text);
  }
  else if (type == NEX_EVENT_POP) {}
}

void cbHeatStart(NextionEventType type, INextionTouchable *widget) {
  NEX_SERIAL.flush();
  if (type == NEX_EVENT_PUSH) {
    if(!Flag.HeatingEn) {
      DBG_SERIAL.println(F("Start heating!"));
      remove_flog(TEMP_LOG_FN);
      switch (HeatingMode) {
        case Past: {
          DBG_SERIAL.println(F("Heating mode - PAST"));
          olog(F("Heating mode - PAST"));
          flog(TEMP_LOG_FN, "Heating mode - PAST", true);
          flog(TEMP_LOG_FN, String("PID coefficients: p-") + String(PIDData.kP[Agg], 4)
                                                  + ", i-" + String(PIDData.kI[Agg], 4)
                                                  + ", d-" + String(PIDData.kD[Agg], 4));
          PastState = PastStateHeating;
          consK = false;
        } break;  
        case Power: {
          DBG_SERIAL.println(F("Heating mode - POWER"));
          olog(F("Heating mode - POWER"));
          flog(TEMP_LOG_FN, "Heating mode - POWER");
          flog(TEMP_LOG_FN, String("Start power: ") + Param.SetPointValue[Power]);
          heating_ctrl(Param.SetPointValue[Power]); break;  
        }
        case Temp: {
          DBG_SERIAL.println(F("Heating mode - TEMP"));
          olog(F("Heating mode - TEMP"));
          flog(TEMP_LOG_FN, "Heating mode - TEMP");          
          flog(TEMP_LOG_FN, String("PID coefficients: p-") + String(PIDData.kP[Agg], 4)
                                                  + ", i-" + String(PIDData.kI[Agg], 4)
                                                  + ", d-" + String(PIDData.kD[Agg], 4));
          consK = false;
        } break; 
        default: break;
      }
      Flag.HeatingEn = true;
      nButHeatStart.setText((char*)"Stop");
      nTxtShirtT.setForegroundColour(NEX_COL_RED);
    }
    else {
      DBG_SERIAL.println(F("Stop heating!"));
      olog(F("Stop heating!"));
      Flag.HeatingEn = false;
      //nTxtHeatPwr.setText((char*)"-----");      
      nTxtHeatState.setText((char*)"---------");
      nTxtHeatStateA.setText((char*)"---------");      
      nButHeatStart.setText((char*)"Start");
      nTxtShirtT.setForegroundColour(NEX_COL_BLACK);
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
  NEX_SERIAL.flush();
  if (type == NEX_EVENT_PUSH) {
    NPage = NexPHeat;
  }
  else if (type == NEX_EVENT_POP) {}
}

void cbPSettShow(NextionEventType type, INextionTouchable *widget) {
  NEX_SERIAL.flush();
  nButOut1.setForegroundColour((OutState[0])?(NEX_COL_RED):(NEX_COL_BLACK));
  nButOut2.setForegroundColour((OutState[1])?(NEX_COL_RED):(NEX_COL_BLACK));
  nButOut3.setForegroundColour((OutState[2])?(NEX_COL_RED):(NEX_COL_BLACK));
  nButOut4.setForegroundColour((OutState[3])?(NEX_COL_RED):(NEX_COL_BLACK));
  if (type == NEX_EVENT_PUSH) {
    NPage = NexPSett;
    nex.drawLine(20, 293, 320, 293, NEX_COL_GRAY);
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

void wifi_connect() {
  uint8_t WiFiConnTimeOut = 50;
  delay(1000);
  DBG_SERIAL.println("");
  DBG_SERIAL.println(F("--- Wi-Fi config ---"));
  olog(F("--- Wi-Fi config ---"));
  
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
    olog(F("Wi-Fi STA mode"));
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
      olog(F("Wi-Fi connect timeout"));
      ap_config();        
    }
    else {
      sblink(2, 500);
      DBG_SERIAL.println("");
      DBG_SERIAL.print(F("STA connected to: ")); 
      olog(F("STA connected to:"));
      DBG_SERIAL.println(WiFi.SSID());
      olog(WiFi.SSID());
      myIP = WiFi.localIP();
      DBG_SERIAL.print(F("Local IP address: "));
      olog(F("Local IP address:"));
      DBG_SERIAL.println(myIP);
      olog(myIP.toString());
      WiFi.enableAP(false);
    }
  }
  DBG_SERIAL.println();
}

String curTime = "";

void spiffs_en() {
  if(SPIFFS.begin(true)) {
    DBG_SERIAL.println(F("SPIFFS opened!"));
    //olog(F("SPIFFS opened!"));
    DBG_SERIAL.print(F("SPIFFS total bytes: ")); DBG_SERIAL.println(SPIFFS.totalBytes());
    DBG_SERIAL.print(F("SPIFFS used bytes: ")); DBG_SERIAL.println(SPIFFS.usedBytes());
    //olog(String("FTP used: ") + SPIFFS.usedBytes());
  }
}

void WiFiCodeTask(void* param) {
    wifi_connect();
    // Регистрация обработчиков
    server.on(F("/"), h_Website);    
    server.on(F("/wifi_param"), h_wifi_param); 
    server.on(F("/pushButt"), h_pushButt);    
    server.onNotFound(handleWebRequests);       
    server.on(F("/xml"), h_XML);
    server.begin();

    ftpSrv.begin(ftpUser, ftpPass); 
    DBG_SERIAL.println(F("FTP server started!"));
    olog(F("FTP server started!"));
    DBG_SERIAL.print(F("user: ")); DBG_SERIAL.println(ftpUser);
    olog(String("user: ") + ftpUser);
    DBG_SERIAL.print(F("pass: ")); DBG_SERIAL.println(ftpPass);
    olog(String("pass: ") + ftpPass);

    for (;;) {
      // Обработка запросов HTML клиента
      server.handleClient();
      // Обработка запросов FTP клиента
      ftpSrv.handleFTP();
      if(Flag.tcUpd) {
        Flag.tcUpd = false;
        // Запрос времени
        timeClient.update();
        curTime = timeClient.getFormattedTime();
      }
    }
}

void setup() {
  // put your setup code here, to run once:
  DBG_SERIAL.begin(DBG_BAUD);
  Serial.println("");
  Serial.println(String("SDK:") + String(ESP.getSdkVersion()));

  NEX_SERIAL.begin(NEX_BAUD);
  NEX_SERIAL.setRxBufferSize(100);

  Wire.begin(SDA, SCL, 400000);

  if(!oled.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3C for 128x32
    DBG_SERIAL.println(F("SSD1306 allocation failed"));
  }  
  delay(500);
  oled.clearDisplay();
  oled.display();

  pinMode(2, OUTPUT);
  pinMode(EN_MOTOR_PIN, OUTPUT);
  pinMode(PWML_MOTOR_PIN, OUTPUT);
  pinMode(PWMR_MOTOR_PIN, OUTPUT);
  pinMode(PWM_SSR_PIN, OUTPUT);
  pinMode(OUT_1, OUTPUT);
  pinMode(OUT_2, OUTPUT);
  pinMode(OUT_3, OUTPUT);
  pinMode(OUT_4, OUTPUT);

  // Nextion HEATING page callbacks
  nTxtPMixH.attachCallback(&cbPMixShow);
  nTxtPSettH.attachCallback(&cbPSettShow);
  nRButPast.attachCallback(&cbNexRButPast);
  nRButPower.attachCallback(&cbNexRButPower);
  nRButTemp.attachCallback(&cbNexRButTemp);
  nButHeatUp.attachCallback(&cbNexButHeatUp);
  nButHeatDown.attachCallback(&cbNexButHeatDown);
  nButHeatStart.attachCallback(&cbHeatStart);

  // Nextion MIX page callbacks
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

  // Nextion SETT page callbacks
  nTxtPHeatS.attachCallback(&cbPHeatShow);
  nTxtPMixS.attachCallback(&cbPMixShow);
  nTxtPSettS.attachCallback(&cbPSettShow);
  nButFWUpd.attachCallback(&cbButFWUpd);
  nButGUIUpd.attachCallback(&cbButGUIUpd);
  nButReset.attachCallback(&cbButReset);
  nButOut1.attachCallback(&cbButOut1);
  nButOut2.attachCallback(&cbButOut2);
  nButOut3.attachCallback(&cbButOut3);
  nButOut4.attachCallback(&cbButOut4);
  
  adcTemp.begin();
  adcTemp.setGain(GAIN_FOUR);

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

  sblink(20, 20);

  xTaskCreatePinnedToCore(
              WiFiCodeTask,   /* Функция, содержащая код задачи */
              "Wi-Fi",        /* Название задачи */
              10000,          /* Размер стека в словах */
              NULL,           /* Параметр создаваемой задачи */
              0,              /* Приоритет задачи */
              &WiFiCoreTaskHandle,         /* Идентификатор задачи */
              0);             /* Ядро, на котором будет выполняться задача */

  spiffs_en();

  prgCount = readProgList(F("/ProgList.txt"));
  DBG_SERIAL.print(F("Read "));  
  DBG_SERIAL.print(prgCount);
  DBG_SERIAL.println(F(" prog"));

  for(uint i = 0; i < prgCount; i++) {
    DBG_SERIAL.print(i + 1); DBG_SERIAL.print(F(" - ")); DBG_SERIAL.println(prgList[i]); 
  }

  //olog(F("Read PID coeff"));
  readPID("/pid.txt");

  // Init Nextion display
  nex.init();
  //olog(F("Init Nextion display"));
  nTxtHMode.setText((char*)"P");
  showPastPageData();
}

void loop() {
  // Обработка сообщений Nextion дисплея
  nex.poll();
  if(Flag.SecondTimer) {
    Flag.SecondTimer = false;

    Temperature[Milk] = adcTemp.readADC_SingleEnded(Milk) * 0.003125;
    Temperature[Shirt] = adcTemp.readADC_SingleEnded(Shirt) * 0.003125;

    /* Heating control ***********************************************************************************/ 
    if(Flag.HeatingEn) {
      flog(TEMP_LOG_FN, Temperature[Shirt] + String(";") + Temperature[Milk] + ";" + HeatingPWM);
      switch (HeatingMode) {
        case Past: {
          // pidKSelect();            
          HeatingPWM = pid(Param.SetPointValue[HeatingMode], Temperature[Milk], PIDData.kP[Agg], PIDData.kI[Agg], PIDData.kD[Agg]);
          switch (PastState) {
            case PastStateHeating : {
              if(Temperature[Milk] >= (double)Param.SetPointValue[Past]) {
                PastDelayCnt = 300;
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
          // pidKSelect();
          HeatingPWM = pid(Param.SetPointValue[Past], Temperature[Milk], PIDData.kP[Agg], PIDData.kI[Agg], PIDData.kD[Agg]);
          if(Temperature[Milk] >= Param.SetPointValue[HeatingMode]) {
            HeatingPWM = 0;
            Flag.HeatingEn = false;
          }
        } break; 

      default: break;
      }
    }
    
  } /* Heating control ***********************************************************************************/  

  /* Display ********************************************************************************************/
  if(Flag.HSecond) {    
    Flag.HSecond = false;
    switch(NPage) {
      case NexPHeat: {  // Heating display
          sprintf(Text, "%s", curTime);
          nTxtTimePHeat.setText(Text);
        nTxtMilkT.setText(addDeg(dtostrf(Temperature[Milk], 3, 1, Text)));
        nTxtShirtT.setText(addDeg(dtostrf(Temperature[Shirt], 3, 1, Text))); 
        if(Flag.HeatingEn) {
          sprintf(Text, "%i%%%s", HeatingPWM, (HeatingMode == Power)?("   "):((consK)?("(c)"):("(a)")));
          nTxtHeatPwr.setText(Text);
          if(HeatingMode == Past) {
            if(PastState == PastStateHeating) nTxtHeatState.setText((char*)"Heating");
            if(PastState == PastStateDelay) {
              nTxtHeatState.setText((char*)"Delay");
              getTimeStrHMS(PastDelayCnt, Text);
              nTxtHeatStateA.setText(Text);
            }
          }
          if((HeatingMode == Power) || (HeatingMode == Temp)){
            nTxtHeatState.setText((char*)"Heating");
            nTxtHeatStateA.setText((char*)"---------");
          }   
        }
        else {
          if(PastState != PastStateIdle) {
            nTxtHeatState.setText((char*)"---------");
            nTxtHeatStateA.setText((char*)"---------");
            nTxtHeatPwr.setText((char*)"-----");
            nButHeatStart.setText((char*)"Start");
            PastState = PastStateIdle;
          }
        } 
      } break;
      case NexPMix: { // Mixer display
          sprintf(Text, "%s", curTime);
          nTxtTimePMix.setText(Text);
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
            nButMixStart.setText((char*)"Start");
            nTxtCycleNum.setText((char*)"--/-");
            nTxtCycleTime.setText((char*)"--:--");
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
          sprintf(Text, "%s", curTime);
          nTxtTimePSett.setText(Text);
        for(int i = 0 ;i < 4; i++) {
          if(OutStatePr[i] != OutState[i]) {
            OutStatePr[i] = OutState[i];
            switch (i) {
              case 0: { nButOut1.setForegroundColour((OutState[i])?(NEX_COL_RED):(NEX_COL_BLACK));} break;
              case 1: { nButOut2.setForegroundColour((OutState[i])?(NEX_COL_RED):(NEX_COL_BLACK));} break;
              case 2: { nButOut3.setForegroundColour((OutState[i])?(NEX_COL_RED):(NEX_COL_BLACK));} break;
              case 3: { nButOut4.setForegroundColour((OutState[i])?(NEX_COL_RED):(NEX_COL_BLACK));} break;
              default: break;
            }
          }
        }
      } break;
      default: break;
    }
  } /* Display ********************************************************************************************/  


  // Очистка OLED дисплея
  if(Flag.OLogCls) {
    Flag.OLogCls = false;
    olog(F("                      "));
  }
}