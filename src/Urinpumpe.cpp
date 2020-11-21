#include <Arduino.h>
#include <WiFi.h>
#include "password.h"

#define _SMARTDEBUG
#include "smartdebug.h"
#include "Urinpumpe.h"
#include "Adafruit_MAX31865.h"
#include "max6675.h"

WiFiServer server(80);

//PINOUT
//const int xx = 0;//dont use 1 2 and serial!!
//const int yyyErrorLED = 1;  //dont use 1 2 and serial!!
const int ReadyLED = 14;
const int SevenSegData = 15;  //Via 74hc595
const int SevenSegClock = 17;  //Via 74hc595
const int NoFartLED = 18;
const int NoUrinLED = 19;
const int GasHahn = 21;  //3, 9, 10, 11 490 Hz (pins 5 and 6: 980 Hz) IST JETZT DIGITAL
const int SevenSegLatch = 22;  //Via 74hc595
const int ZuendPin = 23;
const int ErrorLED = 25;  
const int PumpenSoftwarePWM = 26;

const int TempSpiCS = 4;
const int TempSpiDI = 13;
//const int TempSpiDO = 27;
//const int TempSpiCLK = 12;

int BurningCheckDO = 32;
int BurningCheckCS = 5;
int BurningCheckCLK = 33;


//PININ
const int UrinSensorInteruptPin = 16;
//const int FlammdetektorPin = 5;
const int GasMengenSensor = 32;
const int TempSensor = 33;
const int AnalogPlus = 34;    //Pulldown 10K
const int AnalogMinus = 35;   //Pulldown 10K
//const int unused = A5;
//... A7

unsigned long millisecs = 0;
volatile unsigned long UrinSensorHeartbeat = 0;

int TempIst = 0;

const int TempErrorMin =220;
int TempIdealMin = 240;
const int TempIdeal = 260;
int TempIdealMax = 280;
const int TempErrorMax = 300;

int UrinPumpStufe = 0;
const int GasMengeMin = 600;

int ErrorState =  0;  //1: UrinLow; 2: GasLow; 4: Gasmengensensor im Arsch

String* GetHtml()
{
    String* pHtml = new String();

    *pHtml = "HTTP/1.1 200 OK\r\n";
    *pHtml += "Content-type:text/html\r\n";
    *pHtml += "Connection: close\r\n";
    *pHtml += "\r\n";

    *pHtml += "<!DOCTYPE html><html>\r\n";
    *pHtml += "<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">\r\n";
    *pHtml += "<title>LOVE PEACE JAZZ</title>\r\n";
    *pHtml += "<style>body { background-color: black; color: white; font-size: large;} </style>\r\n";
    *pHtml += "</head>\r\n";
    *pHtml += "<body><h1>GuellePumpe Web Version I</h1>\r\n";
    *pHtml += "<form action=\"..\">    <input type=\"submit\" value=\"REFRESH\" />  </form>";
     
     char buf[256];
     sprintf(buf, "<p>Temperatur: %d</p>\r\n", TempIst); 
     *pHtml += buf;
     sprintf(buf, "<p>PumpStufe: %d</p>\r\n", UrinPumpStufe); 
     *pHtml += buf;
     sprintf(buf, "<p>IdealTemperatur: %d</p>\r\n", TempIdeal); 
     *pHtml += buf;

      *pHtml += "<form action=\"/get\">\r\n";
      *pHtml += "TempIdealMin: <input type=\"text\" name=\"TempIdealMin\">\r\n";
      *pHtml += "<input type=\"submit\" value=\"Submit\">\r\n";
      *pHtml += "</form><br>\r\n";

      *pHtml += "<form action=\"/get\">\r\n";
      *pHtml += "TempIdealMax: <input type=\"text\" name=\"TempIdealMax\">\r\n";
      *pHtml += "<input type=\"submit\" value=\"Submit\">\r\n";
      *pHtml += "</form><br>\r\n";
      *pHtml += "</body></html>\r\n";

    return pHtml;
}


const int WifiValueBufferSize = 20;
void GetWifiBufferValue(const char* header, char* paramName, char* buffer)
{
    //Serial.println(header);

    buffer[0] = 0x00;
    char* indexStart = strstr(header, paramName);
    if(!indexStart)
      return;

    Serial.println("SetVariable " + *paramName);
    //Serial.println(header);

    indexStart += strlen(paramName) + 1; // 1 ist das =

    char* indexEnd = indexStart;
    while((indexEnd < indexStart + WifiValueBufferSize) && (((*indexEnd <= '9') && (*indexEnd >= '0')) || (*indexEnd == '.') || (*indexEnd == ',')))
      indexEnd++;

    if(indexEnd == indexStart + WifiValueBufferSize)
    {
      Serial.println("SetVariable Schade eigentlich, EndIndex stimmt nicht.");
      return;
    }

    strncpy(buffer, indexStart, (int)indexEnd - (int)indexStart);
    buffer[indexEnd - indexStart] = 0x00;

    Serial.print("SetVariable " + *paramName);
    Serial.println(buffer);
}
 
void SetIntVariable(const char* header, char* paramName, int* value)
{
    char buffer[WifiValueBufferSize];

    GetWifiBufferValue(header, paramName, buffer);

    if(buffer[0] != 0x00)
    {
      *value = atoi(buffer);
    }
}

//UNTESTET
void SetFloatVariable(const char* header, char* paramName, float* value)
{
    char buffer[WifiValueBufferSize];

    GetWifiBufferValue(header, paramName, buffer);

    if(buffer[0] != 0x00)
    {
      *value = atof(buffer);
    }
}


void DoWifiCommunication()
{
  const unsigned long waitTime = 1000;  
  static unsigned long oldTime = 0;

  if(!BinIchDran(waitTime, &oldTime))
    return;

  String header;
  unsigned long currentTime = millis();
  unsigned long previousTime = 0; 
  const long timeoutTime = 2000;

  WiFiClient client = server.available();   // Listen for incoming clients

  if (client) {                             // If a new client connects,
    Serial.println("New Client.");          // print a message out in the serial port
    String currentLine = "";                // make a String to hold incoming data from the client
    currentTime = millis();
    previousTime = currentTime;
    while (client.connected() && currentTime - previousTime <= timeoutTime) { // loop while the client's connected
      currentTime = millis();         
      if (client.available()) {             // if there's bytes to read from the client,
        char c = client.read();             // read a byte, then
        Serial.write(c);                    // print it out the serial monitor
        header += c;
        if (c == '\n') {                    // if the byte is a newline character            client.println("<meta http-equiv=”refresh” content=”5″>");
          // if the current line is blank, you got two newline characters in a row.
          // that's the end of the client HTTP request, so send a response:
          if (currentLine.length() == 0) {
            String* pstr = GetHtml();
            client.println(*pstr);
            delete pstr;
            
            break;
          } else { // if you got a newline, then clear currentLine
            currentLine = "";
          }
        } else if (c != '\r') {  // if you got anything else but a carriage return character,
          currentLine += c;      // add it to the end of the currentLine
        }
      }
    }
    
    SetIntVariable(header.c_str(), (char*)"TempIdealMin", &TempIdealMin);
    SetIntVariable(header.c_str(), (char*)"TempIdealMax", &TempIdealMax);
    
    client.stop();
    Serial.println("Client disconnected.");
  }

}




void InteruptUrinSensor()
{
  UrinSensorHeartbeat = millisecs;
  DEBUG_PRINTLN("Interrupting cow"); 
}

bool BinIchDran(unsigned long waitTime, unsigned long* p_oldTime)
{
  if (*p_oldTime + waitTime < millisecs) 
  {
    *p_oldTime = millisecs;
    return true;
  }  

  return false;
}

void SetReadyLED() 
{
  const unsigned long waitTime = 2000;  
  static unsigned long oldTime = 0;

  if(BinIchDran(waitTime, &oldTime))
  {
    digitalWrite(ReadyLED, (TempIst >= TempIdealMin));
  }  
}

void LedTest() 
{

  digitalWrite(ReadyLED,1);
  digitalWrite(NoUrinLED,1);
  digitalWrite(NoFartLED,1);
  digitalWrite(ErrorLED,1);
  delay(300);
  digitalWrite(ReadyLED,0);
  digitalWrite(NoUrinLED,0);
  digitalWrite(NoFartLED,0);
  digitalWrite(ErrorLED,0);
}

// int GasHahnPositionMin = 50;        // % PWM
// const int GasHahnPositionMax = 85;  //Stellung Gashahn ganz offen % PWM
// int GasHahnPosition = 0;
// const int GasHahnPositionDelta = 2;  //Faustwert: in 20 Stellungen von Min bis Max

// bool IsEnoughGasRunning()
// {
//   int Gasmenge = analogRead(GasMengenSensor);

//   if(Gasmenge < 10)  //Gasmenge bei funktionierendem Sensor ohne Gasfluss = 1V = 190 bei Gas, 0,5 V = 100 bei Luft, der Gasmengenmesser ist unter Wert 100 kaputt
//     ErrorState = 4;

//   return (Gasmenge > GasMengeMin);
// }

inline boolean IsBurning()
{
  MAX6675 thermocouple(BurningCheckCLK, BurningCheckCS, BurningCheckDO);

  // For the MAX6675 to update, you must delay AT LEAST 250ms between reads!
  delay(251);
  int temp = (int)thermocouple.readCelsius();
  DEBUG_PRINTLN_VALUE("Flammdetektor: ", temp);
  if(temp > 30)
    return true;

  return false;

  //DEBUG_PRINTLN_VALUE("Flammdetektor: ", digitalRead(FlammdetektorPin) == 0);
  //return digitalRead(FlammdetektorPin) == 0;
}


void Zuenden()
{
  DEBUG_PRINTLN("Zuenden."); 

  //while(!IsEnoughGasRunning())
  //{
    digitalWrite (GasHahn, 1);

    for(int i = 0; i < 5; i++)
    {
      millisecs = millis();
      UrinSoftwarePWM();
      delay(50);
    }
  //}
  // if(!IsEnoughGasRunning())
  // {
  //   ErrorState = 2;
  //   return;
  // }

  DEBUG_PRINTLN("Gashahn ist offen");

  //Zuendzeit = 5sek = 100 * 50 ms
  //int count = 130;
  
  //DEBUG!!
  int count = 260;
  //DEBUG!!
  
  digitalWrite(ZuendPin,1);
  
  while(!IsBurning() && count > 0)
  {
    millisecs = millis();
    UrinSoftwarePWM();
    delay(50);
    count--;
  }

  digitalWrite(ZuendPin,0);

  if(!IsBurning())
    ErrorState = 2;

  DEBUG_PRINTLN("Burning ");
}


void GasHahnRegeln() 
{
  const unsigned long waitTime = 1000;  
  static unsigned long oldTime = 0;

  if(BinIchDran(waitTime, &oldTime))
  {
    if(!IsBurning())
    {
      digitalWrite(GasHahn,0);
    }

    if(TempIst < TempIdeal)
    {
      if(!IsBurning())
      {
        Zuenden();
      }
    }
    
    if(TempIst < TempIdealMin)
    {
      if(TempIst < TempErrorMin)
      {
        UrinPumpStufe = 0;
      }
      else
      {
        // if(UrinPumpStufe > 0)
        //     UrinPumpStufe--;
      }
    }
    if(TempIst > TempIdealMax)
    {
      if(TempIst > TempErrorMax)
      {
        digitalWrite(GasHahn,0);
      }
    }
  }  
}

void CheckPlusAnalogMinus()
{
  const unsigned long waitTime = 300;  
  static unsigned long oldTime = 0;    

  if(BinIchDran(waitTime, &oldTime))
  {

    boolean plus = digitalRead(AnalogPlus);
    boolean minus = digitalRead(AnalogMinus);

    if(plus & (UrinPumpStufe < 9) & (TempIst >= TempIdealMin))
    {
      ++UrinPumpStufe;
      UrinSensorHeartbeat = millisecs;
    }

    if(minus & (UrinPumpStufe > 0))
      --UrinPumpStufe;

    WriteSevenSegment(UrinPumpStufe);


    DEBUG_PRINTLN_VALUE("URIN - PumpStufe: ", UrinPumpStufe);
  }  
}

void WriteFault(Adafruit_MAX31865 max)
{
  uint8_t fault = max.readFault();
  if (fault) 
  {
    Serial.print("Fault 0x"); Serial.println(fault, HEX);
    if (fault & MAX31865_FAULT_HIGHTHRESH) {
      Serial.println("RTD High Threshold"); 
    }
    if (fault & MAX31865_FAULT_LOWTHRESH) {
      Serial.println("RTD Low Threshold"); 
    }
    if (fault & MAX31865_FAULT_REFINLOW) {
      Serial.println("REFIN- > 0.85 x Bias"); 
    }
    if (fault & MAX31865_FAULT_REFINHIGH) {
      Serial.println("REFIN- < 0.85 x Bias - FORCE- open"); 
    }
    if (fault & MAX31865_FAULT_RTDINLOW) {
      Serial.println("RTDIN- < 0.85 x Bias - FORCE- open"); 
    }
    if (fault & MAX31865_FAULT_OVUV) {
      Serial.println("Under/Over voltage"); 
    }
    max.clearFault();
  }
  else
  {
    //Serial.println("NO FAULT"); 
  }
}


void ReadTemp()
{
  const unsigned long waitTime = 1000;  
  static unsigned long oldTime = 0;

  if(BinIchDran(waitTime, &oldTime))
  {
    Adafruit_MAX31865 max = Adafruit_MAX31865(TempSpiCS, TempSpiDI, BurningCheckDO, BurningCheckCLK);
    max.begin(MAX31865_2WIRE);

    //uint16_t rtd =  max.readRTD();

    const float RREF = 4300.0; //pt100 <-> pt 1000

    // Serial.print("RTD value: "); Serial.println(rtd);
    // float ratio = rtd;
    // ratio /= 32768;
    // Serial.print("Ratio = "); Serial.println(ratio,8);
    // Serial.print("Resistance = "); Serial.println(RREF*ratio,8);

    TempIst = (int)max.temperature(1000, RREF);    //1000 == Ohm bei 0Grad

    WriteFault(max);

    DEBUG_PRINTLN_VALUE("TEMP Ist: ", TempIst);
  }
}


boolean IsUrinRunning()
{
  //if(millisecs - UrinSensorHeartbeat > 5000)
  //  return false;


  //DEBUG_PRINTLN("URIN RUNNNING");
  return true;
}

void UrinSoftwarePWM()
{
  static unsigned long oldTime = 0;
  static unsigned long count = 0;

  if(UrinPumpStufe == 0)
  {
    count = 0;
    return;
  }
  int UrinPumpFrequenz = 2 * UrinPumpStufe;

  if(BinIchDran(1000 / UrinPumpFrequenz, &oldTime))
  {
    if((TempIst < TempErrorMin) || (TempIst > TempErrorMax))
    {
      UrinPumpStufe = 0;
      WriteSevenSegment(0);
      DEBUG_PRINTLN("Out of Temp Range");
    }

    digitalWrite(PumpenSoftwarePWM, 1);
    delay(10);  
    digitalWrite(PumpenSoftwarePWM, 0);
    count++;
    if(count > UrinPumpFrequenz * 10)
    {
      count = 0;
      if(!IsUrinRunning())
      {
        if(UrinPumpStufe < 9)
          UrinPumpStufe++;
        else
          ErrorState = 1;
      }
    }
  }
}

//Das zuletzt geschriebene Bit liegt an Ausgang 0 an
//Nachdem Q7S nicht benötigt wird, reicht ein StorageClockPin am Ende
void WriteByte(int state)
{
  for (int i = 1; i < 0x100; i =  i << 1) {
    if ((state & i) > 0)
      digitalWrite (SevenSegData, 1);
    else
      digitalWrite (SevenSegData, 0);

    Pulse (SevenSegClock);

    delay(1);
  }
}

void Pulse(int pin)
{
  digitalWrite (pin, 0);
  //Thread.Sleep (100);
  digitalWrite (pin, 1);
}

//		aaaaa
//		f	b
//		f	b
//		ggggg
//		e	c
//		e	c
//		ddddd


// 	Pin 0 = a		128
// 	Pin 1 = b		64	
//	Pin 2 = c		32
//	Pin 3 = d		16
//	Pin 4 = e		8
//	Pin 5 = f		4
//	Pin 6 = g		2
//	Pin 7 = Point	1

//Um Eins zu schreiben muss Pin B und C high sein, also 64 und 32 
//Wenn ich mein Byte vom kleinsten an schreibe: 64 + 32 = 96
const int SegmentCode[]  {
  128 + 64 + 32 + 16 + 8 + 4,     //0
  64 + 32,		        //1
  128 + 64 + 2 + 8 + 16, 		//2
  128 + 64 + 32 + 16 + 2,		//3
  4 + 32 + 2 + 64,		//4
  128 + 4 + 2 + 32 + 16, 		//5
  128 + 4 + 8 + 16 + 32 + 2,	//6
  128 + 64 + 32, 			//7
  254, 				//8
  128 + 4 + 64 + 2 + 32 + 16,	//9
  8 + 4 + 128 + 64 + 32 + 2,	//a
  4 + 8 + 16 + 32 + 2,		//b
  128 + 4 + 8 + 16, 		//c
  64 + 32 + 16 + 8 + 2,		//d
  128 + 4 + 8 + 2 + 16, 		//e
  128 + 4 + 8 + 2			//f
};


void WriteSevenSegment(int i)
{
  digitalWrite (SevenSegLatch, 0);
  WriteByte (SegmentCode [i]);
  digitalWrite (SevenSegLatch, 1);
}


void ShowErrorState()
{

  DEBUG_PRINTLN_VALUE("Errorstate: ", ErrorState);

  if(ErrorState == 1)
    digitalWrite(NoUrinLED,1);

  if(ErrorState == 2)
    digitalWrite(NoFartLED,1);

  digitalWrite(ErrorLED,1);
  digitalWrite(ReadyLED,0);

  digitalWrite(GasHahn, 0);
  digitalWrite(ZuendPin,0);
  UrinPumpStufe = 0;

  WriteSevenSegment(14);  //E
  DoWifiCommunication();
  delay(200);  
  DoWifiCommunication();
  delay(200);  
  DoWifiCommunication();
  delay(200);  
  WriteSevenSegment(ErrorState);
  DoWifiCommunication();
  delay(200);  
  DoWifiCommunication();
  delay(200);  
  DoWifiCommunication();
  delay(200);  
}

void loop() 
{
  
  if(ErrorState == 0)
  //Thread.Sleep (100);
  {
    millisecs = millis();

    ReadTemp();
    UrinSoftwarePWM();
    SetReadyLED();  
    GasHahnRegeln();
    CheckPlusAnalogMinus();
    //DoWifiCommunication();
   }
  else
  {
    ShowErrorState();
  }

}  

void setup() 
{
  DEBUG_INIT(115200); // Initialisierung der seriellen Schnittstelle
  //Thread.Sleep (100);
  pinMode(AnalogPlus,INPUT);
  pinMode(AnalogMinus,INPUT);
  pinMode(PumpenSoftwarePWM,OUTPUT);
  pinMode(ErrorLED,OUTPUT);
  pinMode(NoFartLED,OUTPUT);
  pinMode(NoUrinLED,OUTPUT);
  pinMode(ReadyLED,OUTPUT);
  pinMode(GasHahn,OUTPUT);
  pinMode(ZuendPin,OUTPUT);
  pinMode(SevenSegData, OUTPUT);
  pinMode(SevenSegClock, OUTPUT);
  pinMode(SevenSegLatch, OUTPUT) ;
  digitalWrite (SevenSegData, 0);
  digitalWrite (SevenSegClock, 0);
  digitalWrite (GasHahn, 0);

  //pinMode(FlammdetektorPin, INPUT_PULLUP);

  pinMode(UrinSensorInteruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(UrinSensorInteruptPin), InteruptUrinSensor, FALLING); 

  WriteSevenSegment(0);
  LedTest();

  // WiFi.mode(WIFI_STA);
  // delay(500);
  // WiFi.begin(NetworkName, NetworkPWD);
  // Serial.println("Connect WIFI, waiting 4 IP from DHCP"); 
  // while(WiFi.status() != WL_CONNECTED)
  // {
  //   Serial.print(".");
  //   Serial.print(WiFi.status());
  //   delay(1500); 
  // }
  // Serial.print("WiFi connected IP address: ");
  // Serial.println(WiFi.localIP());
  // server.begin();

  DEBUG_PRINTLN("Setup finished."); 
}


