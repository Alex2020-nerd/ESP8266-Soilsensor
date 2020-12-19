//******************************************************************************
//  ESP12F mit SoilSensor
//
//  Version    Release Historv
// ---------+--------------------------------------------------------------------
//      2   ! Umbau mit TimeOut bei Verbindungsprobleme ==> deep sleep
//      3   ! Umbau auf neuen pi char* mqtt_server = "pi4iot.local";
//          ! - update alle 15 min anstatt 30
//      4   ! Minor Changes
//      5   ! DS2438 Integriert
//      6   ! esp_deepSleepCycles
//          !      Stromverbrauch DeepSleep 25uA
//          !      Cycle ohne WiFi ohne Messung, 30mA während 250ms
//          !      Cycle mit WiFi und Messung, 80mA während 10sec
//      7   ! Humidity Values anhand der VBat normalisiert
//*******************************************************************************

// ******* LIBARAY INCLUDE ***************
#include <ESP8266WiFi.h>  //ESP8266 Include
#include <PubSubClient.h> //MQTT Include
#include <OneWire.h>      //One Wire Library
#include <DS2438.h>       //DS2438 lokale Library von GitHub https://github.com/jbechter/arduino-onewire-DS2438

float humidity, wifi_retry, mqtt_retry, vbat, temp; // Values read from sensor

// ******* Soil Sensor Settings  ***************
const int soil_sens_vcc_pin = 12;
const int soil_sens_SensorPin = A0;

const int soil_sens_AirValue3_4V = 700;  // AD Wert des SoilSensor bei 3.4V, darüber ist es immer dieser Wert
const int soil_sens_AirValue2_7V = 530;  // AD Wert des SoilSensor bei 2.7V, darunter funktioniert der ESP nicht mehr
const int soil_sens_WaterValue3_4V = 300;  // AD Wert des SoilSensor bei 3.4V, darüber ist es immer dieser Wert
const int soil_sens_WaterValue2_7V = 170;  // AD Wert des SoilSensor bei 2.7V, darunter funktioniert der ESP nicht mehr

// ******* OneWire Settings  ***************
// define the 1-Wire address of the DS2438 battery monitor here (lsb first)
//
// Mit der funktion ds2438.determineAddress(); wird die adresse ermittelt
// falls man mehrere 1 Wire Busteilnehmer hat, muss man die Adresse
// herausfinden und in das untenstehende address Array einpflegen, dann darf aber die 
// ds2438.determineAddress(); nicht mehr aufgerufen werden!! HAL 2020-10-17
uint8_t DS2438_address[8] = { 0x26, 0x45, 0xe6, 0xf7, 0x00, 0x00, 0x00, 0x4e };
const uint8_t ONE_WIRE_PIN = 4;
OneWire ow(ONE_WIRE_PIN);
DS2438 ds2438(&ow, DS2438_address);

// ******* WIFI setting  ***************
const char *wifi_ssid = "your_SSID";  // ***CUSTOMIZING***
const char *wifi_password = "yourWIFI_Password"; // ***CUSTOMIZING***
// !!! SENSOR Nummer !!!
const char *wifi_hostname = "ESP05"; // ***CUSTOMIZING***
const int wifi_max_Retry = 10; //beim Verbinden max 10 versuche mit 1000ms delay ==> 10sec


WiFiClient espClient;



// ******* MQTT setting ***************
const char *mqtt_server = "pi4iot.local";  // ***CUSTOMIZING*** // pi4iot = Hostname des Raspberry Pi 4 mit dem IoT Stack
const int mqtt_port = 1883;
const char *mqtt_clientID = "ESP05"; // ***CUSTOMIZING***
const char *mqtt_username = "yourMQTT_Username"; // ***CUSTOMIZING***
const char *mqtt_password = "yourMQTT_Passwort"; // ***CUSTOMIZING***
// !!! SENSOR Nummer !!!
const char *mqtt_publish_topic = "yourMainTopic/Plant05"; // ***CUSTOMIZING***
const char *mqtt_subscribe_topic = "yourMainTopic/Plant05/LED"; // ***CUSTOMIZING***
const int mqtt_max_Retry = 5; //beim Verbinden max 10 versuche mit 300ms delay ==> 3sec

String arrayVariableNames[] = {"Humid", "ADval", "WIFIRetry", "MQTTRetry", "VBat", "Temp", "Sensor7", "Sensor8", "Sensor9", "Sensor10"};
float arrayVariableValues[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
int MQTTnumberVariables = 6;
const int maxMQTT_MsgSize = 2000;
String strMQTT = "";
char MQTT_Msg[maxMQTT_MsgSize];

PubSubClient client(mqtt_server, mqtt_port, espClient);

// ******* ESP setting ***************

const long esp_DeepSleepTime_sec = 1800;      // 1800 Sec => 30 Min Schlafzyklus
const long esp_DeepSleepCycles = 6;           // 6 Schalfzyklen = 6 x 30min = 3h
const long esp_ErrorDeepSleepTime_sec = 900;  // 900 Sec => 15 Min für Retry


//-------------------------------------------------
// esp_GoDeepSleep
//-------------------------------------------------

void esp_GoDeepSleep(long sleep_sec)
{

  if (WiFi.status() == WL_CONNECTED)
  {
    Serial.println("Wifi ist noch eingeschaltet");
    WiFi.disconnect();
    Serial.print("WiFi Status after Disconnect : ");
    Serial.println(WiFi.status());
  }

  Serial.println("ESP going deep sleep : Chrr.... Chr.... Chr....");
  ESP.deepSleep(sleep_sec * 1000 * 1000); // uSec ==> Achtung 32 Bit Integer ==> Überlauf verhindern!
  delay(100);
}

//-------------------------------------------------
// esp_deepSleepCycle
//-------------------------------------------------
// Deep-sleep for specified amount of cycles of sleeping periods.
// If powered on (not a deep-sleep reset), nothing will happen.
// Call this twice: in the beginning of setup (end_of_setup == false)
// and at the end of setup (end_of_setup == true).

void esp_deepSleepCycle(uint32_t periods, bool end_of_setup = false) {

    uint32_t reset_counter = 0;
    bool waking_from_sleep = ESP.getResetReason() == "Deep-Sleep Wake";

    if (!end_of_setup) {
        if (waking_from_sleep) {
            Serial.print("Waking up from deep-sleep via reset pin. Reset counter: ");
            ESP.rtcUserMemoryRead(0, &reset_counter, sizeof(reset_counter));
            reset_counter++;
            ESP.rtcUserMemoryWrite(0, &reset_counter, sizeof(reset_counter));
            Serial.println(String(reset_counter));
        } else {
            Serial.println("Zeroing reset counter.");
            ESP.rtcUserMemoryWrite(0, &reset_counter, sizeof(reset_counter));
            return;
        }
    }


    if (reset_counter < periods) {

        Serial.println("ESP01 going deep sleep : Chrr.... Chr.... Chr....");

        ESP.deepSleep(esp_DeepSleepTime_sec*1e6);
        delay (100);
    }
    reset_counter = 0;
    ESP.rtcUserMemoryWrite(0, &reset_counter, sizeof(reset_counter));

}


//-------------------------------------------------
// soil_sens_power_on
//-------------------------------------------------

void soil_sens_and_ds2438_power_on(void)
{
  Serial.println("===== soil_sens_and_ds2438__power_on on GPIO12 ======");
  pinMode(soil_sens_vcc_pin, OUTPUT);
  digitalWrite(soil_sens_vcc_pin, HIGH);
  // delay (500); //mind 500ms warten damit der kapazitive Sensor sich eingependalt hat, nicht notwedig, da WIFI Aufbau > 2sec
  Serial.println("===== POWER on GPIO12 ======");
}

//-------------------------------------------------
// soil_sens_power_off
//-------------------------------------------------

void soil_sens_and_ds2438__power_off(void)
{

  Serial.println("===== soil_sens_and_ds2438__power_OFF GPIO12 ======");
  digitalWrite(soil_sens_vcc_pin, LOW);
  Serial.println("===== POWER OFF GPIO12 ======");
}


//-------------------------------------------------
// getSoliSensAirValue
//
// Berechnet den aktuellen Referenzwert für
// Air Value abhängig der VBat
//-------------------------------------------------

int getSoliSensAirValue (float VBat)
{
  int AirValue = 0;
  float Vdiff = 0;
  float gradient = 0;
  if (VBat > 3.4) {
      return soil_sens_AirValue3_4V;
  }
  if (VBat < 2.7) {
      return soil_sens_AirValue2_7V;
  } else
  {
    gradient = (soil_sens_AirValue3_4V-soil_sens_AirValue2_7V)/(3.4 - 2.7);

    Vdiff = VBat-2.7;
    AirValue = soil_sens_AirValue2_7V + ((Vdiff)*gradient);
    return AirValue;
  }
}

//-------------------------------------------------
// getSoliSensWaterValue
//
// Berechnet den aktuellen Referenzwert für
// Water Value abhängig der VBat
//-------------------------------------------------

int getSoliSensWaterValue (float VBat)
{
  int WaterValue = 0;
  float Vdiff = 0;
  float gradient = 0;
  if (VBat > 3.4) {
      return soil_sens_WaterValue3_4V;
  }
  if (VBat < 2.7) {
      return soil_sens_WaterValue2_7V;
  } else
  {
    gradient = (soil_sens_WaterValue3_4V-soil_sens_WaterValue2_7V)/(3.4 - 2.7);
    Vdiff = VBat-2.7;
    WaterValue = soil_sens_WaterValue2_7V + ((Vdiff)*gradient);
    return WaterValue;
  }
}

//-------------------------------------------------
// soil_sens_readout Mess Routinen
//-------------------------------------------------

void soil_sens_readout()
{
  int ad_value = 0;
  int soil_sens_AirValue = 0;
  int soil_sens_WaterValue = 0;
  Serial.println("===== soil_sens_readout start Air and Water Value ======");
  Serial.print ("!! SoilSens Values about VBat : ");
  Serial.println ( vbat,2);
  soil_sens_AirValue = getSoliSensAirValue (vbat);
  Serial.print ("!! SoilSensAirValue : ");
  Serial.println ( soil_sens_AirValue );
  soil_sens_WaterValue = getSoliSensWaterValue (vbat);
  Serial.print("!! SoilSensWaterValue: ");
  Serial.println (soil_sens_WaterValue );

  Serial.println("===== soil_sens_readout start to read out ======");
  ad_value = analogRead(soil_sens_SensorPin); 
  Serial.print("AD Value: ");
  Serial.println(ad_value);

  humidity = map(ad_value, soil_sens_AirValue, soil_sens_WaterValue, 0, 100);
  Serial.print("Humidity : ");
  Serial.print(humidity);
  Serial.println(" %");
  if (humidity > 100)
  {
    humidity = 100;
  }
  if (humidity < 0)
  {
    humidity = 0;
  }
  arrayVariableValues[0] = humidity;
  arrayVariableValues[1] = ad_value;
  arrayVariableValues[2] = wifi_retry;
  arrayVariableValues[3] = mqtt_retry;
  Serial.println("===== soil_sens_readoutEND to read out ======");
}

//-------------------------------------------------
// setup_ds2438
//-------------------------------------------------

void setup_ds2438(void)
{
  delay(200);
  Serial.println("===== ds2438 Setup ======");
  Serial.println();
  Serial.println("Determine Address of DS2438....");
  ds2438.determineAddress();
  delay(2000);
  ds2438.begin();
  Serial.println("===== ds2438 Setup done");
}

//-------------------------------------------------
// ds2438_readout Mess Routinen
//-------------------------------------------------

void ds2438_readout(void)
{
  Serial.println("===== ds2438_readout start to read out ======");
  ds2438.update();
  if (ds2438.isError()) {
      Serial.println("Error reading from DS2438 device");
      temp = 0;
      vbat = 0;
  } else {
      temp = ds2438.getTemperature();
      vbat = ds2438.getVoltage(DS2438_CHA);
  }
  arrayVariableValues[4] = vbat;
  arrayVariableValues[5] = temp;
  Serial.println("===== ds2438_readout end read out ======");
}

//-------------------------------------------------
// MQTT callback  // im Moment nicht gebraucht
//                // da keine Signale an 8266
//-------------------------------------------------

void callback(char *topic, byte *payload, unsigned int length)
{
  Serial.print("Message arrived [");
  Serial.print(topic);
}

//-------------------------------------------------
// MQTT_reconnect
//-------------------------------------------------

void MQTT_reconnect()
{
  int intCount = 0;

  // Loop until we're reconnected
  while (!client.connected())
  {
    Serial.println("===== MQTT reconnect ======");
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect(mqtt_clientID, mqtt_username, mqtt_password))
    {
      Serial.println("MQTT connected");
      Serial.print("MQTT Client ID: ");
      Serial.println(mqtt_clientID);

      //Subscribe
      client.subscribe(mqtt_subscribe_topic);
    }
    else
    {

      if (intCount >= mqtt_max_Retry)
      {
        // wenn nach wifi_may_Retry keine Verbindung
        // hergestellt werden konnte, dann gehen wir
        // wieder in den deep sleep zum Stromsparen
        esp_GoDeepSleep(esp_ErrorDeepSleepTime_sec);
      }
      intCount++;
      mqtt_retry = intCount;
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 1 seconds before retrying
      delay(300);
    }
  }
}

//-------------------------------------------------
// MQTT Compile_strMQTT()
//-------------------------------------------------

void Compile_strMQTT()
{
  int i = 0;

  strMQTT = "{"; //initial

  for (i = 0; i < (MQTTnumberVariables); i++)
  { //alle Werte im Array in den strPayLoad zusammensetzen
    strMQTT = strMQTT + "\"" + String(arrayVariableNames[i]) + "\"" + ":" + String(arrayVariableValues[i]);
    if (i < (MQTTnumberVariables - 1))
    {
      strMQTT = strMQTT + ","; // es gibt noch einen weiteren Wert an den String anzufügen
    }
  }
  strMQTT = strMQTT + "}";
  // strMQTT = "Das ist doch scheisse";
}

//-------------------------------------------------
// setup_MQTT
//-------------------------------------------------

void setup_MQTT(void)
{
  client.setCallback(callback);
  // Connect to MQTT network
  // Wait for connection
  if (!client.connected())
  {
    MQTT_reconnect();
  }
  client.loop();
  Serial.print("ESP Chip ID : ");
  Serial.println(ESP.getChipId());
  Serial.println("Client started");
}

//-------------------------------------------------
// publish_MQTT
//-------------------------------------------------

void publish_MQTT(void)
{
  Serial.println("-- MQTT String --");
  Compile_strMQTT();
  Serial.println(strMQTT);
  Serial.println("-- MQTT CHAR --");
  strMQTT.toCharArray(MQTT_Msg, 100);
  Serial.println(MQTT_Msg);
  //Publish
  client.publish(mqtt_publish_topic, MQTT_Msg);
  delay(100);
}

//-------------------------------------------------
// Setup_wify
//-------------------------------------------------

void setup_wifi()
{
  int intCount = 0;
  Serial.println("===== WIFI Setup & Connect ======");

  // wenn wifi.persistent(false) nicht hinzugefügt wird, schreibt wifi.beginn immer SSID/password in den flash
  // brennt die memory zellen im flash aus!!! mit dieser Anweisung wird nur dann geflasht wenn sich die werte
  // wirklich ändern
  WiFi.persistent(false);

  WiFi.mode(WIFI_STA);
  Serial.print("Connecting to ");
  Serial.println(wifi_ssid);
  WiFi.begin(wifi_ssid, wifi_password);

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(1000);
    Serial.print(".");
    if (intCount >= wifi_max_Retry)
    {
      // wenn nach wifi_max_Retry keine Verbindung
      // hergestellt werden konnte, dann gehen wir
      // wieder in den deep sleep zum Stromsparen
      esp_GoDeepSleep(esp_ErrorDeepSleepTime_sec);
    }
    intCount++;
    wifi_retry = intCount;
  }
  Serial.println();
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  Serial.println();
  Serial.print("Wifi Hostname set to : ");
  Serial.println(wifi_hostname);
  WiFi.hostname(wifi_hostname);
}


//#################################################
// SETUP
//#################################################

void setup(void)
{
  // You can open the Arduino IDE Serial Monitor window to see what the code is doing
  Serial.begin(115200); 
  Serial.println("\n\r \n\r Starting up");

  esp_deepSleepCycle (esp_DeepSleepCycles);

  soil_sens_and_ds2438_power_on();
  setup_wifi();
  setup_MQTT();
  setup_ds2438();

  soil_sens_readout();
  ds2438_readout();

  soil_sens_and_ds2438__power_off();

  publish_MQTT();

  esp_deepSleepCycle (esp_DeepSleepCycles, true);
}

//#################################################
// LOOP
//#################################################

void loop(void)
{
  // empty! weil beim Awake immer nur die Setup druchlaufen wird und dann
  // wird der ESP01 wieder schlafen gelegt wird....
}