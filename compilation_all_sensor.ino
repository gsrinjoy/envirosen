

//WIFI INITIALIZATION
//http://192.168.29.46/sen/sensordata.php?AQI=355
//String server = "192.168.29.46";


#define SerialESP8266 Serial2
String server = "srinjoygg.000webhostapp.com";
//String server = "192.168.156.111";
String cadena=""; //to store HTTP request

//-------------------------------------------/

//MULTICHANNEL GAS SENSOR INITIALIZATION
 #include <Multichannel_Gas_GMXXX.h>
 #include <Wire.h>
 GAS_GMXXX<TwoWire> gas;
//-------------------------------------------/
//DHT11 SENSOR INITIALIZATION
#include "DHT.h"

#define DHTPIN 2

#define DHTTYPE DHT11

DHT dht(DHTPIN, DHTTYPE);
//-------------------------------------------/
//NEO6M INITIALIZATION
#include <TinyGPS.h>
#define ss Serial3
TinyGPS gps;
//-------------------------------------------/
//GASSENSORS INITIALIZATION
const int mq2 = A1;
const int mq7 = A0;   
const int mq135 = A2;
//------------------------------------------/
//AQI CALCULATION STATIC VALUE
float m = -0.6527; 
float b = 1.30; 
float R0 = 21.91;
//------------------------------------------/
//PM 2.5 INITIALIZATION
int measurePin = A3;
int ledPower = 3;

unsigned int samplingTime = 280;
unsigned int deltaTime = 40;
unsigned int sleepTime = 9680;

float voMeasured = 0;
float calcVoltage = 0;
float dustDensity = 0;

//------------------------------------------/
//VARIABLE INITIALIZATIONS
static float NO2,C2H5OH,VOC,CO,temp,humid,hix;
static double mq2p,mq7p,mq135p;
static String str,la,ln;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  SerialESP8266.begin(9600);//WIFI INITIALIZATION
  ss.begin(9600);//GPS SENSOR 
  gas.begin(Wire, 0x08);//MULTICHANNEL GAS SENSOR
  dht.begin();//DHT 11 TEMP HUMID SENSOR
  pinMode(mq7, INPUT);//GAS SENSOR INPUT MQ7 
  pinMode(mq135, INPUT);//GAS SENSOR INPUT MQ135  
  pinMode(mq2, INPUT);//GAS SENSOR INPUT MQ2
  pinMode(ledPower,OUTPUT);//PM2.5LED
  //---------------------------------------------/
  //wifi connection


  SerialESP8266.setTimeout(5000);
  //checking the ESP8266 response
  SerialESP8266.println("AT");
  if(SerialESP8266.find("OK"))
    Serial.println("READY");
  else
    Serial.println("Error on ESP8266");
  //ESP8266 in STA mode.
  SerialESP8266.println("AT+CWMODE=1");
  if(SerialESP8266.find("OK"))
    Serial.println("ESP8266 on STATION mode...");
  //Connecting to wifi
  SerialESP8266.println("AT+CWJAP=\"Agnisha_Bhatta\",\"agnisha31\"");//enter WIFI PASSWORN AND SSID TO CONNECT
  Serial.println("Connnecting...");
  SerialESP8266.setTimeout(10000); //waiting for connection
  if(SerialESP8266.find("OK"))
    Serial.println("WIFI OK");
  else
    Serial.println("Unable to connect...");
  SerialESP8266.setTimeout(2000);
  //Disable multiple connections
  SerialESP8266.println("AT+CIPMUX=0");
  if(SerialESP8266.find("OK"))
    Serial.println("Multiple connections disabled");

    
 //------------------------------------------------------/   

    
}

void loop() {
  // put your main code here, to run repeatedly:
  mgassen();
  neo6_m();
  dht11();
  p25();
  Serial.println("Multichannel gas sensor values");
   Serial.print("no2---");
   Serial.println(NO2);
   Serial.print("C2H5OH---");
   Serial.println(C2H5OH);
   Serial.print("VOC---");
   Serial.println(VOC);
   Serial.print("CO---");
   Serial.println(CO);
  Serial.println("------------------------------------------");
  Serial.println("DHT11");
    Serial.print(F("Humidity: "));
    Serial.print(humid);
    Serial.print(F("%  Temperature: "));
    Serial.print(temp);
    Serial.print(F("°C "));
    Serial.print(F(" Heat index: "));
    Serial.print(hix);
    Serial.print(F("°C \n"));
  Serial.println("------------------------------------------");  
  Serial.println("Gas Sensors");
    Serial.print("mq135:");
    Serial.print(q135());
    Serial.print("PPM");
    Serial.println();
    Serial.print("mq2:");
    Serial.print(q2());
    Serial.print("PPM");
    Serial.println();
    Serial.print("mq7:");
    Serial.print(q7());
    Serial.print("PPM");
    Serial.println();
  Serial.println("------------------------------------------");
  Serial.print("raw :");
  Serial.println(voMeasured);
  Serial.print(dustDensity);
    Serial.print("mg/m3");
    Serial.println();  
  Serial.println("------------------------------------------");
  Serial.println("gps value:"); 
    Serial.println("Latitude");
    Serial.println(la); 
    Serial.println("longitude");
    Serial.println(ln); 
 //---------------------------------------------------------------/
 //SENDING DATA TO THE CLOUD THROUGH WIFI MODULE
   //sendata(aqi);
   sendata(q2(),q7(),q135());
   //void sendata(String mv2,String mv7,String mv135)
   delay(1000);    
      
}
void mgassen()
{
  delay(1000);
   NO2 = gas.measure_NO2();
   C2H5OH = gas.measure_C2H5OH();
   VOC = gas.measure_VOC();
   CO = gas.measure_CO();
}
void dht11()
{
  delay(1000);
   humid = dht.readHumidity();
   temp = dht.readTemperature();
  if (isnan(humid) || isnan(temp)){
    Serial.println(F("Failed to read from DHT sensor!"));
    return;
  }
   hix =dht.computeHeatIndex(temp, humid, false);
}
String q135()
{
  String senval = String(aPPM(analogRead(mq135)));
  return(senval);
}
String q2()
{
  String senval = String(aPPM(analogRead(mq2)));
  return(senval);
}
String q7()
{
  String senval = String(aPPM(analogRead(mq7)));
  return(senval);
}
void neo6_m()
{
  while (ss.available())
    if (gps.encode(ss.read()))
    {
      Serial.print(F("Location: "));
      float lat1=0.0, lon1=0.0; 
      gps.f_get_position(&lat1,&lon1);
      
         la=String(lat1);
         
         ln=String(lon1);
     
       
    }
      
}
void p25()
{
   digitalWrite(ledPower,LOW);
  delayMicroseconds(samplingTime);

  voMeasured = analogRead(measurePin);

  delayMicroseconds(deltaTime);
  digitalWrite(ledPower,HIGH);
  delayMicroseconds(sleepTime);

  calcVoltage = voMeasured*(5.0/1024);
  dustDensity = 0.17*calcVoltage-0.1;//mg/m3

  if ( dustDensity < 0)
  {
    dustDensity = 0.00;
  }
}
 double aPPM(int aValue)
 {
  float sensor_volt;
  float RS_gas; 
  float ratio;
  int sensorValue = aValue;

  sensor_volt = sensorValue*(5.0/1023.0); 
  RS_gas = ((5.0*10.0)/sensor_volt)-10.0; 
  ratio = RS_gas/R0; 
  double ppm_log = (log10(ratio)-b)/m;
  return ppm_log;
}



void sendata(String mv2,String mv7,String mv135)
{
  SerialESP8266.println("AT+CIPSTART=\"TCP\",\"" + server + "\",80");
  Serial.println("AT+CIPSTART=\"TCP\",\"" + server + "\",80");
  if(SerialESP8266.find("OK")) 
  {
    Serial.println();
    Serial.println();
    Serial.println();
    Serial.println("Server connection successful...");
    ////////////////////////////////URL PART/////////////////////////////////////////////////////////
    String finhttp="GET /sen/sensordata.php?MQ2="+ mv2+"&MQ7="+mv7+"&MQ135="+mv135+"&NO2="+String(NO2)+"&C2H5OH="+String(C2H5OH)+"&VOC="+String(VOC)+"&CO="+String(CO)+"&HMD="+String(humid)+"&TMP="+String(temp)+"&HI="+String(hix)+"&RAWPM="+String(voMeasured)+"&DD="+String(dustDensity)+"&LAT="+la+"&LON="+ln;
    finhttp=finhttp+" HTTP/1.1\r\n";
    finhttp=finhttp+"Host: "+server+"\r\n\r\n";//to the host i want to send
    finhttp=finhttp+"Host: localhost\r\n\r\n";
    ////////////////////////////////////////////////////////////////////////////////////////////////
    //Sending the length of the HTTP request
    SerialESP8266.print("AT+CIPSEND=");
    SerialESP8266.println(finhttp.length());
    //waiting for ">" for sending HTTP request
    if(SerialESP8266.find(">")) {
      //we can send the HTTP request when > is displayed
      Serial.println("Sending HTTP request. . .");
      SerialESP8266.println(finhttp);
      if(SerialESP8266.find("SEND OK")) {
        Serial.println("HTTP request sent...:");
        Serial.println();
        Serial.println("On stand by...");
        boolean reqcheck=false;
        long tmili=millis();
        cadena="";
        while(reqcheck==false) 
        {
          while(SerialESP8266.available()>0) 
          {
            char c=SerialESP8266.read();
            Serial.write(c);
            cadena.concat(c); //store the request string on "cadena"
          }
          if(cadena.length()>3500) {
            Serial.println("The request exceeded the maximum length...");
            SerialESP8266.println("AT+CIPCLOSE");
            if( SerialESP8266.find("OK"))
              Serial.println("Connection terminated...");
            reqcheck=true;
          }
          if((millis()-tmili)>10000) {
            //Terminate if connection time exceeded the maximum
            Serial.println("Connection time exceeded...");
            SerialESP8266.println("AT+CIPCLOSE");
            if( SerialESP8266.find("OK"))
              Serial.println("Connection terminated");
            reqcheck=true;
          }
          if(cadena.indexOf("CLOSED")>0) {
            Serial.println();
            Serial.println("String OK, connection terminated");
            reqcheck=true;
          }
        }
      } else {
        Serial.println("error on HTTP request.....");
      }
    }
  } else {
    Serial.println("Unable to find the server....");
  }
}
