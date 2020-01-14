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

Nextion nex(Serial2);
/* Past page objects */
NextionText nTxtPastState(nex, NexPPast, 1, "tPState");

/* Mixer page objects */
NextionText nTxtProgName(nex, NexPMix, 12, "tProgName");
NextionButton nButButCCRot(nex, NexPMix, 6, "bCCRot");
NextionButton nButButCRot(nex, NexPMix, 8, "bCRot");

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

ProgCycle prgCycle[PRG_NUM];
String prgList[PRG_NUM];
uint8_t prgCount = 0;
uint8_t cyclesCount = 0;
uint8_t HeatingPower = 0;
char TxtChar[51];

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
  bool MixEn;
  bool InitLocalTimerVar;
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
uint8_t CurrentCycle = 0;
int8_t CurrentProgNumber = 0;
uint32_t ProgTime = 0;

void motor_ctrl(uint8_t enable, uint8_t direct, uint8_t power) {
	if(enable == ON) {
    digitalWrite(ENL_MOTOR_PIN, HIGH);
    digitalWrite(ENR_MOTOR_PIN, HIGH);
    if(direct == FORWARD) {
      ledcWrite(PWML_TIMER_CHANN, map(power, 0, 100, 0, 255));
      ledcWrite(PWMR_TIMER_CHANN, 0);
    }
    else {
      ledcWrite(PWML_TIMER_CHANN, 0);
      ledcWrite(PWML_TIMER_CHANN, map(power, 0, 100, 0, 255));
    }
	}
	else {
    digitalWrite(ENL_MOTOR_PIN, LOW);
    digitalWrite(ENR_MOTOR_PIN, LOW);
    ledcWrite(PWML_TIMER_CHANN, 0);
    ledcWrite(PWMR_TIMER_CHANN, 0);
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
	}

  /* Mixer motor control ************************************************/
  if(Flag.MixEn) {
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

			if(++lCycleTime == cycle_time(lCycleCount)) {
				lCycleTime = 0;
				lCycleCount++;
				CurrentCycle = lCycleCount;
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
			Flag.MixEn = false;
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
  Serial.println("EEPROM initialization");
  int eSize = sizeof(Param);
  EEPROM.begin(eSize);
  Param.FuseMac = ESP.getEfuseMac();
  EEPROM.put(PARAM_ADDR, Param);
  EEPROM.end();
  Serial.println("EEPROM initialization completed");
  Serial.println("");
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
  Serial.print(F("Request File: "));
  Serial.println(path);
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
  Serial.print(F("Load File: "));
  Serial.println(dataFile.name());
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
  Serial.println(F("Apply Wi-Fi parameters"));
  String ssid = server.arg(F("wifi_ssid"));
  String pass = server.arg(F("wifi_pass"));
  Serial.print(F("Wi-Fi SSID: ")); Serial.println(ssid);
  Serial.print(F("Wi-Fi pass: ")); Serial.println(pass);
  server.send(200, F("text/html"), F("Wi-Fi setting is updated, module will be rebooting..."));
  // WiFi.begin(ssid.c_str(), pass.c_str());
  delay(100);
  WiFi.softAPdisconnect(true);
  delay(100);
  strcpy(Param.ssid, ssid.c_str());
  strcpy(Param.pass, pass.c_str());
  save_param();
  Serial.print(F("Param SSID: ")); Serial.println(Param.ssid);
  Serial.print(F("Param pass: ")); Serial.println(Param.pass);
  WiFi.mode(WIFI_STA);
  WiFi.begin(Param.ssid, Param.pass);
  delay(500);
  SPIFFS.end();
  Serial.println(F("Resetting ESP..."));
  ESP.restart();
}

void handleWebRequests() {
  if(loadFromSpiffs(server.uri())) return;
  Serial.println(F("File Not Detected"));
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
  Serial.println(message);
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
  Serial.println(F("Wi-Fi AP mode"));
  Serial.println(F("AP configuring..."));
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ap_ssid, ap_pass); 
  Serial.println(F("done"));
  IPAddress myIP = WiFi.softAPIP();
  Serial.print(F("AP IP address: "));
  Serial.println(myIP);    
}

uint8_t readProg(uint8_t progNumber, ProgCycle *pc) {
  Serial.print(F("Read prog: ")); Serial.println(prgList[progNumber]);
  String fn = F("/");
  if(prgList[progNumber].indexOf("\r") != -1) {
    prgList[progNumber].setCharAt((prgList[progNumber].length() - 1), ' ');
    prgList[progNumber].trim();
  }
  fn += prgList[progNumber];
  fn += F(".csv");
  uint8_t cycleNumber = 0;
  uint8_t si, ei;
  Serial.print(F("Open file prog: ")); Serial.println(fn);
  if(SPIFFS.exists(fn)) {
    File pf = SPIFFS.open(fn, "r");  
    if (!pf) {
      Serial.println(F("Prog file open failed on read."));
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
    Serial.print(F("File: ")); Serial.print(fn); Serial.println(F(" not found!"));
  }
  return cycleNumber;
}

int read_progList(String file) {
  uint8_t cnt = 0;  
  File f = SPIFFS.open("/ProgList.txt", "r");
  if (!f) {
    Serial.println(F("ProgList file open failed."));
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

void cbNexButCCRot(NextionEventType type, INextionTouchable *widget) {
  if (type == NEX_EVENT_PUSH) {

  }
  else if (type == NEX_EVENT_POP) {

  }
}

void cbNexButCRot(NextionEventType type, INextionTouchable *widget) {
  if (type == NEX_EVENT_PUSH) {

  }
  else if (type == NEX_EVENT_POP) {

  }
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial2.begin(921600);
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
  Serial.println("");
  Serial.println(F("--- Wi-Fi config ---"));
  
  Serial.println("");
  int eSize = sizeof(Param);
  EEPROM.begin(eSize);
  EEPROM.get(PARAM_ADDR, Param);
  if(Param.FuseMac != ESP.getEfuseMac()) {
    eeprom_init();
    ap_config();
  }
  else {
    Serial.println(F("Wi-Fi STA mode"));
    //Serial.print(F("Param SSID: ")); Serial.println(Param.ssid);
    //Serial.print(F("Param pass: ")); Serial.println(Param.pass);
    WiFi.mode(WIFI_STA);
    WiFi.begin(Param.ssid, Param.pass);
    while (WiFi.status() != WL_CONNECTED) {      
      sblink(1, 10);
      delay(190);
      Serial.print(".");
      if(--WiFiConnTimeOut == 0) break;
    }
    if(WiFiConnTimeOut == 0) {
      Serial.println("");
      Serial.println(F("Wi-Fi connected timeout"));
      ap_config();        
    }
    else {
      sblink(2, 500);
      Serial.println("");
      Serial.print(F("STA connected to: ")); 
      Serial.println(WiFi.SSID());
      IPAddress myIP = WiFi.localIP();
      Serial.print(F("Local IP address: ")); Serial.println(myIP);
      WiFi.enableAP(false);
    }
  }

  Serial.println();

  delay(1000);

  if(SPIFFS.begin(true)) {
    Serial.println(F("SPIFFS opened!"));
    ftpSrv.begin(ftpUser, ftpPass); 
    Serial.println(F("FTP server started!"));
    Serial.print(F("user: ")); Serial.println(ftpUser);
    Serial.print(F("pass: ")); Serial.println(ftpPass);

    Serial.print(F("SPIFFS total bytes: ")); Serial.println(SPIFFS.totalBytes());
    Serial.print(F("SPIFFS used bytes: ")); Serial.println(SPIFFS.usedBytes());
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
  Serial.print(F("Read "));
  Serial.print(prgCount);
  Serial.println(F(" prog"));

  for(uint i = 0; i < prgCount; i++) {
    Serial.print(i + 1); Serial.print(F(" - ")); Serial.println(prgList[i]); 
  }

  cyclesCount = readProg(0, prgCycle);
  Serial.print(F("Read "));
  Serial.print(cyclesCount);
  Serial.print(F(" cycles in "));
  Serial.print(prgList[0]);
  Serial.println(F(" prog"));

  for(uint i = 0; i < cyclesCount; i++) {
    Serial.print(F("Cycle: ")); Serial.print(i + 1); Serial.print(F(" - "));
    Serial.print(prgCycle[i].State); Serial.print(F(", "));
    Serial.print(prgCycle[i].Time); Serial.print(F(", "));
    Serial.print(prgCycle[i].OWTime); Serial.print(F(", "));
    Serial.print(prgCycle[i].PBRevers); Serial.print(F(", "));
    Serial.println(prgCycle[i].Power);
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
  prgList[0].toCharArray(TxtChar, (prgList[0].length() + 1));
  nTxtProgName.setText(TxtChar);
  nButButCCRot.attachCallback(&cbNexButCCRot);
  nButButCRot.attachCallback(&cbNexButCRot);
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

  }
}