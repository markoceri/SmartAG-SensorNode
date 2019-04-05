/*
 * SmartAG SensorNode
 * Author: Marco Mancino
 * Updated: 04/2019
 * v 0.2.2
 * 
 * TODO: 
 *  - Parse response message
 *  - Implement very deep sleep
 *  - Change deep sleep from downlink message
 *  
 *  FEATURES:
 *  + Check battery level
 *  + Check sensors error and comunicate it in message bitfield
 *  + Switch to Long sleep time when battery in to low
 *  + Stop sleep by pulling down a pin (usefull for reprogramming)
 *  + Optimized consuption powering off sensors after reading
 *  
 *  CHENGES:
 *  + Removed one Soil Temperature sensor and added an externa Temperature ad Humidity Sensors
 *  + Removed sigfox error after success message sent
 *  + Fixed BATTERY_VOLTAGE_LOW on battery begin
 *  + Fixed Battery read
 *  + Hardware fix of battery pin
 *  + Added statistics calculations on sensors reads
 *  
*/
#define VERSION "v0.2.2"

#include <SigFox.h>
#include <ArduinoLowPower.h>

//*** General
// ------------------------------------------ 
#define DISABLE_SEND_MESSAGE // Message to Sigfox is not sent
//#define DISABLE_SLEEP // Board doesn't go to sleep mode

#define ENABLE_INTERNAL_DHT   // Enable the internal Humidity and Temperature sensor
//#define PRODUCTION          // Al messages on serial are soppressed
#define ENABLE_STATISTICS     // Enable statistics functions on sensors reads
//#define ENABLE_BAROMETER    // Enable barometric pressure sensor
// ------------------------------------------

#if defined(ENABLE_STATISTICS) 
  #include <Statistics.h>
#endif

// Times
// ------------------------------------------
int time_to_sleep_min = 15;
int time_to_sleep_min_default = 15;
#define TIME_TO_SLEEP_MIN_WARNING1 30
#define TIME_TO_SLEEP_MIN_WARNING2 60
// ------------------------------------------

// Thresholds definition
// ------------------------------------------
#define SOILMOISTURE_DRY 3870
#define SOILMOISTURE_WET 80
#define LEAFWETNESS_DRY 0
#define LEAFWETNESS_WET 4095
// ------------------------------------------

// Pins Dfinition
// ------------------------------------------
#define BATTERY_STOP_PIN 0
#define POWER_PIN 1
#define DHT_EXT_PIN 6
#define DHT_INT_PIN 7
#define STOP_SLEEP_PIN 8
#define ONE_WIRE_BUS 9
#define SOILMOISTURE_PIN A1
#define LEAFWETNESS_PIN A2
#define BATTERY_PIN A3
// ------------------------------------------

// Battery definition
// ------------------------------------------
// R1 2.18K + 6.8ohm (Transistor)
// R2 3.28K
#define BATTERY_SENSE_R1 2180
#define BATTERY_SENSE_R2 3280
#define BATTERY_SENSE_RATIO 2.17

#define BATTERY_VOLTAGE_LOW 3000
#define BATTERY_VOLTAGE_HIGHT 4200
#define BATTERY_WARNING_PERC_LEV1 10
#define BATTERY_WARNING_PERC_LEV2 5
// ------------------------------------------

// DHT Sensor types definition
// ------------------------------------------
#define DHTTYPE DHT22
// ------------------------------------------

// Number of Sensors reads to perform a mean value
// ------------------------------------------
#define SENSORS_READS 10
// ------------------------------------------

// Error codes bitfield
// ------------------------------------------
#define SOIL_TEMPERATURE_ERROR  1   //00000001
#define PRESSURE_ERROR          2   //00000010
#define SOIL_MOISTURE_ERROR     4   //00000100
#define LEAF_WETNESS_ERROR      8   //00001000
#define AIR_TEMPERATURE_ERROR   16  //00010000
#define AIR_HUMIDITY_ERROR      32  //00100000
#define SIGFOX_ERROR            64  //01000000
#define SIGFOX_MSG_RX_OK        128 //10000000
// ------------------------------------------

// Sigfox message definition
// ------------------------------------------
typedef struct __attribute__ ((packed)) sigfox_message {
  uint8_t battery;            // 1 byte
  uint8_t stCode;             // 1 byte
  int16_t soilTemperature;    // 2 byte
  uint8_t soilMoisture;       // 1 byte
  uint8_t leafWetness;        // 1 byte
  int16_t airTemperature;     // 2 byte
  uint8_t airHumidity;        // 1 byte
  //uint16_t pressure         // 2 byte
} SigfoxMessage;              // 9 byte total (12 byte can be sent)

SigfoxMessage msg;
// ------------------------------------------

// Sensors resolution
// ------------------------------------------
#define ANALOG_RESOLUTION 12
#define BATTERY_ANALOG_RESOLUTION 10

// Temperature sensor
// ------------------------------------------
#include <OneWire.h> 
#include <DallasTemperature.h>

OneWire oneWire(ONE_WIRE_BUS); 
DallasTemperature sensors(&oneWire);
uint8_t soilTemperatureAddr;

#if defined(ENABLE_STATISTICS) 
  Statistics soilTemperatureStats(SENSORS_READS);
#endif
// ------------------------------------------

// External DHT Temperature & Humidity sensor
// ------------------------------------------
#include <DHT.h>

DHT dht_ext(DHT_EXT_PIN, DHTTYPE);
float airTemperature = 0;
uint8_t airHumidity = 0;

#if defined(ENABLE_STATISTICS) 
  Statistics airTemperatureStats(SENSORS_READS);
  Statistics airHumidityStats(SENSORS_READS);
#endif
// ------------------------------------------

// Battery sensor
// ------------------------------------------
#include <Battery.h>

Battery batt = Battery(BATTERY_VOLTAGE_LOW, BATTERY_VOLTAGE_HIGHT, BATTERY_PIN);
uint16_t batteryVoltage = 0;

#if defined(ENABLE_STATISTICS) 
  Statistics batteryStats(SENSORS_READS);
#endif
// ------------------------------------------

// Internal DHT Temperature & Humidity sensor
// ------------------------------------------
#if defined(ENABLE_INTERNAL_DHT) 
  DHT dht_int(DHT_INT_PIN, DHTTYPE);  
  float airTemperatureInt = 0;
  uint8_t airHumidityInt = 0;
#endif

#if defined(ENABLE_STATISTICS) 
  Statistics airTemperatureIntStats(SENSORS_READS);
  Statistics airHumidityIntStats(SENSORS_READS);
#endif
// ------------------------------------------

// Barometer pressure sensor
// ------------------------------------------
#if defined(ENABLE_BAROMETER)
  #include <Wire.h>
  #include <Adafruit_Sensor.h>
  #include <Adafruit_BMP280.h>
  
  Adafruit_BMP280 baro;
  
  float pressure = 0;
  float baroTemp = 0;
  int baroAlt = 0;

  #if defined(ENABLE_STATISTICS) 
    Statistics pressureStats(SENSORS_READS);
    Statistics baroTempStats(SENSORS_READS);
  #endif
#endif
// ------------------------------------------

// Analog Sensors
// ------------------------------------------
int soil_analog = 0;
int leaf_analog = 0;
long int soilMoistureTotal = 0;
long int leafWetnessTotal = 0;

#if defined(ENABLE_STATISTICS) 
  Statistics soilMoistureStats(SENSORS_READS);
  Statistics leafWetnessStats(SENSORS_READS);
#endif
// ------------------------------------------

// Index counter
int i = 0;

void setup() {
  Serial.begin(9600); 
  analogReadResolution(ANALOG_RESOLUTION); // set analog resolution to 12bit

  pinMode(POWER_PIN, OUTPUT);
  pinMode(BATTERY_STOP_PIN, OUTPUT);

  if (!SigFox.begin()) {
    Serial.println("Sigfox Shield error or not present!");
    reboot();
  }

  batt.begin(3300, BATTERY_SENSE_RATIO, &sigmoidal);
  
  sensors.setResolution(ANALOG_RESOLUTION); // set resolution of all devices to 12bit
  
  //SigFox.end();   

  #if !defined(PRODUCTION)
    SigFox.debug(); // Enable debug led and disable automatic deep sleep
  #endif
  
  msg.stCode = 0;
  pinMode(STOP_SLEEP_PIN, INPUT);

  delay(1000);

  #if !defined(PRODUCTION)
    Serial.println("SmartAG SensorNode Started! (" + String(VERSION) + ")");
  #endif
}

void loop() {        
  doJobs();
  
  #if !defined(DISABLE_SLEEP)
    if (batt.level(batteryVoltage) < BATTERY_WARNING_PERC_LEV2)
      time_to_sleep_min = TIME_TO_SLEEP_MIN_WARNING2;
    else if (batt.level(batteryVoltage) < BATTERY_WARNING_PERC_LEV1)
      time_to_sleep_min = TIME_TO_SLEEP_MIN_WARNING1;
    else
      time_to_sleep_min = time_to_sleep_min_default;
      
    if (digitalRead(STOP_SLEEP_PIN)) {
      #if !defined(PRODUCTION)
        Serial.println(String("I'm going to sleep... see you in ") + String(time_to_sleep_min) + String("min"));
      #endif
      
      LowPower.sleep(time_to_sleep_min * 60 * 1000);
    } else
      #if !defined(PRODUCTION)
        Serial.println("Sleep was stopped by StopSleepPin !!!");
      #endif
  #endif

  delay(4000);
}

void doJobs(){
  #if !defined(PRODUCTION)
    Serial.println();
    Serial.println("########################################################");
    Serial.println("Powering On Sensors!");
  #endif

  // Enable PowerOn Sensors Line
  digitalWrite(POWER_PIN, HIGH);
  
  setupSensors();
  getSensors();

  #if !defined(PRODUCTION)
    Serial.println("Powering Off Sensors!");
    Serial.println("########################################################");
  #endif

  // Disable PowerOn Sensors Line
  digitalWrite(POWER_PIN, LOW);
  
  #if !defined(PRODUCTION)
    printSensors();
  #endif

  #if !defined(DISABLE_SEND_MESSAGE)
    sendMessage();
  #else
    #if !defined(PRODUCTION)
      Serial.println("SendMessage is disabled !!!");
    #endif
  #endif
}

void setupSensors() {
  #if !defined(PRODUCTION)
    Serial.println("Setting up sensors...");
  #endif

  // Pressure
  #if defined(ENABLE_BAROMETER)
    if (!baro.begin()) {  
      msg.stCode |= PRESSURE_ERROR;
    } else {
      msg.stCode &= ~(PRESSURE_ERROR);
    }
  #endif

  // Internal humidity & temperature
  #if defined(ENABLE_INTERNAL_DHT) 
    dht_int.begin();
  #endif

  // Esternal humidity & temperature
  dht_ext.begin();

  // Soil temperature
  sensors.begin();

  delay(1000);
  
  #if !defined(PRODUCTION)
    Serial.println("Sensors setup!");
  #endif
}

void getSensors(){
  #if !defined(PRODUCTION)
    Serial.println("Requesting sensors data...");
  #endif

  // **********************************************************************************
  // I2C Barometer - Pressure
  // **********************************************************************************
  #if defined(ENABLE_BAROMETER)
    
    pressure = baro.readPressure();
    baroTemp = baro.readTemperature();

    //msg.pressure = (int16_t)(pressure * 100)
  #endif

  // **********************************************************************************
  // One-Wire temperature sensors - Soil Temperature
  // **********************************************************************************
  #if !defined(PRODUCTION)
    if (sensors.isParasitePowerMode()) {
      Serial.println("WARNING -> Soil temperature requires parassite power!");
    }
  #endif

  int16_t soilTemperatureRead = 0;
  int16_t soilTemperatureTotal = 0;
     
  switch (sensors.getDS18Count()){
    // All it's ok (maybe)
    case 1:            
      #if !defined(PRODUCTION)
        Serial.print("Soil Temp:\t\t");
      #endif

      for(i=0; i<SENSORS_READS; i++){
        sensors.requestTemperatures();
        soilTemperatureRead = (int16_t)(sensors.getTempCByIndex(0)*100);
        soilTemperatureTotal += soilTemperatureRead;

        #if defined(ENABLE_STATISTICS) 
          soilTemperatureStats.addData(float(soilTemperatureRead)/100);
        #endif

        #if !defined(PRODUCTION)
          Serial.print("#");
        #endif
        delay(500);
      }
  
      #if !defined(PRODUCTION)
        Serial.println("-OK");
      #endif

      msg.soilTemperature = int16_t(soilTemperatureTotal/SENSORS_READS);

      if ((msg.soilTemperature/100) < -150 || (msg.soilTemperature/100) > 150) {
        msg.stCode |= SOIL_TEMPERATURE_ERROR;
      } else {
        // Remove error on bitfield
        msg.stCode &= ~(SOIL_TEMPERATURE_ERROR);
      }
            
      break;
    // Sensor have problem
    case 0:
      // Set errors on bitfield
      msg.stCode |= SOIL_TEMPERATURE_ERROR;     
      break;
  }

  // **********************************************************************************
  // Analog Sensors - Soil Moisture & Leaf Wetness
  // **********************************************************************************
  // Performs 10 reads
  uint16_t soilMoistureRead = 0;
  uint16_t leafWetnessRead = 0;
  soilMoistureTotal = 0;
  leafWetnessTotal = 0;
  
  #if !defined(PRODUCTION)
    Serial.print("Soil & Leaf:\t\t");
  #endif
  
  for(i=0; i<SENSORS_READS; i++){
    soilMoistureRead = analogRead(SOILMOISTURE_PIN);
    leafWetnessRead = analogRead(LEAFWETNESS_PIN);
    soilMoistureTotal += soilMoistureRead;
    leafWetnessTotal += leafWetnessRead;

    #if defined(ENABLE_STATISTICS) 
      soilMoistureStats.addData(soilMoistureRead);
      leafWetnessStats.addData(leafWetnessRead);
    #endif

    #if !defined(PRODUCTION)
      Serial.print("#");
    #endif
    delay(500);
  }
  
  #if !defined(PRODUCTION)
    Serial.println("-OK");
  #endif

  // Mean
  soil_analog = soilMoistureTotal/SENSORS_READS;
  leaf_analog = leafWetnessTotal/SENSORS_READS;
 
  // Analog Soil Moisture sensors
  msg.soilMoisture = map(soil_analog, SOILMOISTURE_DRY, SOILMOISTURE_WET, 0, 100);
  msg.leafWetness = map(leaf_analog, LEAFWETNESS_WET, LEAFWETNESS_DRY, 0, 100);

  // Check errors on reads
  if (msg.soilMoisture < 0 || msg.soilMoisture > 100 ) {
    // Set error on bitfield      
    msg.stCode |= SOIL_MOISTURE_ERROR;
  } else {
    msg.stCode &= ~(SOIL_MOISTURE_ERROR);
  }
  if (msg.leafWetness < 0 || msg.leafWetness > 100 ) {
    // Set error on bitfield      
    msg.stCode |= LEAF_WETNESS_ERROR;
  } else {
    msg.stCode &= ~(LEAF_WETNESS_ERROR);
  }

  // **********************************************************************************
  // DHT temperature and Humidity sensor
  // **********************************************************************************

  // External
  // --------------------
  float airHumidityRead = 0;
  float airTemperatureRead = 0;
  float airHumidityTotal = 0;
  float airTemperatureTotal = 0;
  
  #if !defined(PRODUCTION)
    Serial.print("DHT ext:\t\t");
  #endif
  
  for(i=0; i<SENSORS_READS; i++){
    airHumidityRead = dht_ext.readHumidity();
    airTemperatureRead = dht_ext.readTemperature();
    airHumidityTotal += airHumidityRead;
    airTemperatureTotal += airTemperatureRead;

    #if defined(ENABLE_STATISTICS) 
      airHumidityStats.addData(airHumidityRead);
      airTemperatureStats.addData(airTemperatureRead);
    #endif

    #if !defined(PRODUCTION)
      Serial.print("#");
    #endif
    delay(500);
  }
  
  #if !defined(PRODUCTION)
    Serial.println("-OK");
  #endif

  // Mean
  airTemperature = airTemperatureTotal/SENSORS_READS;
  airHumidity = airHumidityTotal/SENSORS_READS;

  if (isnan(airTemperature) || (airTemperature) > 150 || (airTemperature) < -150) {
    msg.stCode |= AIR_TEMPERATURE_ERROR;
  } else {
    msg.stCode &= ~(AIR_TEMPERATURE_ERROR);
  }

  if (isnan(airHumidity || (airHumidity) > 150)) {
    msg.stCode |= AIR_HUMIDITY_ERROR;
  } else {
    msg.stCode &= ~(AIR_HUMIDITY_ERROR);
  }
  
  msg.airTemperature = (int16_t)(airTemperature*100);
  msg.airHumidity = uint8_t(airHumidity);

  // Internal
  // --------------------
  #if defined(ENABLE_INTERNAL_DHT) 
    float airHumidityTotalInt = 0;
    float airTemperatureTotalInt = 0;
    float airHumidityIntRead = 0;
    float airTemperatureIntRead = 0;
    
    #if !defined(PRODUCTION)
      Serial.print("DHT int:\t\t");
    #endif
    
    for(i=0; i<SENSORS_READS; i++){
      airHumidityIntRead = dht_int.readHumidity();
      airTemperatureIntRead = dht_int.readTemperature();
      
      airHumidityTotalInt += airHumidityIntRead;
      airTemperatureTotalInt += airTemperatureIntRead;

      #if defined(ENABLE_STATISTICS) 
        airHumidityIntStats.addData(airHumidityIntRead);
        airTemperatureIntStats.addData(airTemperatureIntRead);
      #endif

      #if !defined(PRODUCTION)
        Serial.print("#");
      #endif
      delay(500);
    }
    
    #if !defined(PRODUCTION)
      Serial.println("-OK");
    #endif
  
    // Mean
    airTemperatureInt = int16_t((airTemperatureTotalInt/SENSORS_READS)*100);
    airHumidityInt = uint8_t(airHumidityTotalInt/SENSORS_READS);

    // Check error
    if (isnan(airTemperatureInt) || (airTemperatureInt) > 150 || (airTemperatureInt) < -150) {
      // add error
    } else {
      // remove error
    }
  
    if (isnan(airHumidityInt || (airHumidityInt) > 150)) {
      // add error
    } else {
      // remove error
    }
  #endif

  // **********************************************************************************
  // Battery read
  // **********************************************************************************
  uint16_t batteryVoltageRead = 0;
  uint16_t batteryVoltageSUM = 0;
  
  analogReadResolution(BATTERY_ANALOG_RESOLUTION);
  delay(100);
  digitalWrite(BATTERY_STOP_PIN, HIGH);
  delayMicroseconds(100);
  
  #if !defined(PRODUCTION)
    Serial.print("Battery:\t\t");
  #endif
   
  for(i=0; i<SENSORS_READS; i++){
    batteryVoltageRead = batt.voltage();
    batteryVoltageSUM += batteryVoltageRead;
    
    #if defined(ENABLE_STATISTICS) 
      batteryStats.addData(batteryVoltageRead);
    #endif

    #if !defined(PRODUCTION)
      Serial.print("#");
    #endif
    delay(500);
  }
  
  #if !defined(PRODUCTION)
    Serial.println("-OK");
  #endif

  // Mean
  batteryVoltage = (batteryVoltageSUM / (SENSORS_READS));
  msg.battery = batt.level(batteryVoltage);
  digitalWrite(BATTERY_STOP_PIN, LOW);
  analogReadResolution(ANALOG_RESOLUTION);
}

void printSensors(){
  Serial.println();
  Serial.println("########################################################");
  Serial.println("********************************************************");
  Serial.print("Battery level\t\t");
  Serial.println(String(msg.battery) + "%" + "\t -> Voltaggio: " + String(float(batteryVoltage)/1000) + "V");
  
  Serial.print("StatusCode Bitfield\t");
  print_binary(msg.stCode, 8);
  
  Serial.print("Soil Temperature \t");
  Serial.println(String(float(msg.soilTemperature)/100) + "°C" + "\t -> ID Sensore: " + String(soilTemperatureAddr));

  Serial.print("Soil Moisture\t\t");
  Serial.println(String(msg.soilMoisture) + "%" + "\t -> Analog mean: " + String(soil_analog));

  Serial.print("Leaf Wetness\t\t");
  Serial.println(String(msg.leafWetness) + "%" + "\t -> Analog mean: " + String(leaf_analog));

  Serial.print("Air Temperature\t\t");
  Serial.println(String(float(msg.airTemperature)/100) + "°C");

  Serial.print("Air Humidity\t\t");
  Serial.println(String(msg.airHumidity) + "%");

  Serial.println("--------------------------------------------------------");
  Serial.println("Not sended DATA");
  Serial.println("--------------------------------------------------------");
  #if defined(ENABLE_BAROMETER)
    Serial.print("Barometer pressure Int\t");
    Serial.println(String(float(airTemperatureInt)) + "°C");
  
    Serial.print("Barometer temperature\t");
    Serial.println(String(airHumidityInt) + "%");
  #endif   

  #if defined(ENABLE_INTERNAL_DHT) 
    Serial.print("Air Temperature Int\t");
    Serial.println(String(float(airTemperatureInt)/100) + "°C");
  
    Serial.print("Air Humidity Int\t");
    Serial.println(String((airHumidityInt)) + "%");
  #endif
  Serial.println("--------------------------------------------------------");

  #if defined(ENABLE_STATISTICS) 
    Serial.println("++++++++++++++++++++ STATISTICS ++++++++++++++++++++++++");
    Serial.println("SENSOR\t\t\tMEAN\t\t\tSTDDEV\t\t\tVAR\t\t\tMIN\t\t\tMAX");
    printStatistics("BATTERY", "\t\t\t", &batteryStats);
    printStatistics("SOIL TEMP", "\t\t", &soilTemperatureStats);
    printStatistics("SOIL MOISTURE", "\t\t", &soilMoistureStats);
    printStatistics("LEAF WETNESS", "\t\t", &leafWetnessStats);
    printStatistics("HUMIDITY EXT", "\t\t", &airHumidityStats);
    printStatistics("TEMPERATURE EXT", "\t\t", &airTemperatureStats);
    #if defined(ENABLE_INTERNAL_DHT)
      printStatistics("HUMIDITY INT", "\t\t", &airHumidityIntStats);
      printStatistics("TEMPERATURE INT", "\t\t", &airTemperatureIntStats);
    #endif
    Serial.println("++++++++++++++++++++++++++++++++++++++++++++++++++++++++");
  #endif

  Serial.println("********************** WARNING *************************");
  if (msg.stCode != 0) {
    if (msg.stCode & SOIL_TEMPERATURE_ERROR)
      Serial.println("SOIL_TEMPERATURE_ERROR");
    if (msg.stCode & SOIL_MOISTURE_ERROR)
      Serial.println("SOIL_MOISTURE_ERROR");
    if (msg.stCode & LEAF_WETNESS_ERROR)
      Serial.println("LEAF_WETNESS_ERROR");
    if (msg.stCode & AIR_TEMPERATURE_ERROR)
      Serial.println("AIR_TEMPERATURE_ERROR");
    if (msg.stCode & AIR_HUMIDITY_ERROR)
      Serial.println("AIR_HUMIDITY_ERROR");
    if (msg.stCode & SIGFOX_ERROR)
      Serial.println("SIGFOX_ERROR");
  }
  if (msg.battery < BATTERY_WARNING_PERC_LEV2)
      Serial.println(String("BATTERY_WARNING_PERC_LEV2 \t Deep sleep incresed to " + String(TIME_TO_SLEEP_MIN_WARNING2) + "minutes"));
  else
  if (msg.battery < BATTERY_WARNING_PERC_LEV1)
      Serial.println(String("BATTERY_WARNING_PERC_LEV1 \t Deep sleep incresed to " + String(TIME_TO_SLEEP_MIN_WARNING1) + "minutes"));
  
  Serial.println("********************************************************");
}

void reboot() {
   NVIC_SystemReset();
   while (1) ;
}

void sendMessage(){
  SigFox.begin();
  delay(100);

  #if !defined(PRODUCTION)
    Serial.println("Dimensione pacchetto: " + String(sizeof(msg)) + "bytes");
    Serial.print("Module temperature: ");
    Serial.println(SigFox.internalTemperature());
    Serial.print("Invio messaggio a Sigfox... ");
  #endif

  SigFox.status();
  delay(1);
  
  SigFox.beginPacket();

  SigFox.write((uint8_t*)&msg, sizeof(msg));

  int lastMessageStatus = SigFox.endPacket(true);

  if (lastMessageStatus > 0) {
    // Set error bit
    msg.stCode |= SIGFOX_ERROR;
    #if !defined(PRODUCTION)
      Serial.println("No transmission");
    #endif
  } else {
    // Remove error bit
    msg.stCode &= ~(SIGFOX_ERROR);
    #if !defined(PRODUCTION)
      Serial.println("Messaggio inviato");
    #endif
  }

  #if !defined(PRODUCTION)
    Serial.println("Stato: SIGFOX -> " + String(SigFox.status(SIGFOX)) + " | ATMEL -> " + String(SigFox.status(ATMEL)));
  #endif

  receiveMessage();
}

void receiveMessage(){
  #if !defined(PRODUCTION)
    Serial.println("####################################");
  #endif
  
  if (SigFox.parsePacket()) {
    String downlinkMessage;
    
    while (SigFox.available()) {
      downlinkMessage += SigFox.read();
    }
    msg.stCode |= SIGFOX_MSG_RX_OK;
    
    #if !defined(PRODUCTION)
      Serial.println("Response from server:");
      Serial.print("0x");
      Serial.println(downlinkMessage);
      Serial.println();
    #endif    

    downlinkMessage = "";
  } else {
    msg.stCode &= ~(SIGFOX_MSG_RX_OK);

    #if !defined(PRODUCTION)
      Serial.println("Could not get any response from the server");
      Serial.println("Check the SigFox coverage in your area");
      Serial.println("If you are indoor, check the 20dB coverage or move near a window");
    #endif
  }
  
  #if !defined(PRODUCTION)
    Serial.println("####################################");
    Serial.println();
  #endif
  
  SigFox.end();
}

void print_binary(int v, int num_places){
    int mask=0, n;

    for (n=1; n<=num_places; n++)
    {
        mask = (mask << 1) | 0x0001;
    }
    v = v & mask;  // truncate v to specified number of places

    while(num_places)
    {

        if (v & (0x0001 << num_places-1))
        {
             Serial.print("1");
        }
        else
        {
             Serial.print("0");
        }

        --num_places;
        if(((num_places%4) == 0) && (num_places != 0))
        {
            //Serial.print("_");
        }
    }
    Serial.println();
}

#if defined(ENABLE_STATISTICS) 
  void printStatistics (String sensorName, String separator, Statistics* stats){    
    Serial.print(sensorName);
    Serial.print(separator);
    Serial.print(stats->mean());
    if (stats->mean() > 9999.99)
      Serial.print("\t\t");
    else
      Serial.print("\t\t\t");
    Serial.print(stats->stdDeviation());
    if (stats->stdDeviation() > 9999.99)
      Serial.print("\t\t");
    else
      Serial.print("\t\t\t");
    Serial.print(stats->variance());
    if (stats->variance() > 9999.99)
      Serial.print("\t\t");
    else
      Serial.print("\t\t\t");
    Serial.print(stats->minVal());
    if (stats->minVal() > 9999.99)
      Serial.print("\t\t");
    else
      Serial.print("\t\t\t");
    Serial.println(stats->maxVal());
  }
#endif

