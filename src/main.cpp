#include <Arduino.h>
#include <Homie.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <X9C.h>

//https://github.com/marvinroger/async-mqtt-client/blob/develop/examples/FullyFeatured-ESP8266/FullyFeatured-ESP8266.ino
//https://github.com/marvinroger/async-mqtt-client/issues/92
#include <AsyncMqttClient.h>
AsyncMqttClient& mqttClient = Homie.getMqttClient();
String payloadBuf;
String payloadBuf_substr;

// char* topic = new char[strlen(Homie.getConfiguration().mqtt.baseTopic) + strlen(Homie.getConfiguration().deviceId) + 1 + strlen(winecoolerNode.getId()) + 1 + 7 + 1];
char* topic = new char[strlen("homie/dev00x/winecooler/testing") + 1];
uint16_t packetIdSub = 0;

bool get_settings = true; 

const int      DEFAULT_PUBLISH_INTERVAL = 10;
const int      DEFAULT_COOL_MAX         = 75;
const double   DEFAULT_TEMP_SETPOINT    = 0.0;
const double   DEFAULT_TEMP_HYSTERESIS  = 0.2;

unsigned long  last_publish = 0;
String         PublishString;

int            publish_interval;
float          setpoint;
float          hysteresis;
float          temp0            = -127.00;
float          testing_temp0    = 0.0;   // used for testing override 
int            testing_cool_pot = 0;     // used for testing override 
int            testing_heat_pwm = 0;     // used for testing override 
float          temp1            = -127.00;
int            adjust_interval;

// Temperature sensor on D5, place pull-up resistor between 2.2K or 4.7K to 5V
#define        ONE_WIRE_BUS D5      //temp sensor DS18B20 on D5(5)

// Dallas temperature variables
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

// Analog
#define PIN_ANALOG  A0
int cool_level      = 0;

// Digital potentiometer X9C
#define INC   D4   // D1 Mini D4(GPIO2)  - pulled up in H/W (10k) ->  chip pin 1
#define UD    D8   // D1 Mini D8(GPIO15)                          ->  chip pin 2
#define CS    D0   // D1 Mini D0(GPIO16) - pulled up in H/W (10k) ->  chip pin 7

// "up" and "down" make sense in relation to the wiper pin 5 [VW/RW] and the HIGH end of the pot
// i.e. pin 3 [VH/RH], leaving pin 6 [VL/RL] unused (floating). You can easily use pin 6 instead
// pin 3, but "min" will actually mean "max" and vice versa. Also, the "setPot" percentage will
// set the equivalent of 100-<value>, i.e. setPot(70) will set the resistance between pins 5 and 6
// to 30% of the maximum. (Of course in that case,the "unused" resistance between 5 and 3 will be 70%)
// Nothing to stop you using it as a full centre-tap potentiometer, the above example giving
// pin 3[H] -- 70% -- pin 5[W] -- 30% -- pin 6[L]

X9C pot;      // create a pot controller
int           cool_min             = 50;  // Based on resistor 20k + digi pot 10k  (so no original ntc and no 8.2k resistor)
int           cool_max;                   // via cool_maxSetting.get()
int           cool_pot;                   // intial potentiometer value equal to cool_max (minimal cooling)
int           cool_step            = 1;
unsigned long last_adjust          = 0;   //time previous update to potmeter, see also adjust_intervalSetting
const int     DEFAULT_ADJUST_INTV  = 0;   // cool/het adjust interval: if 0, then no adjustments, otherwise needs at least 60 (in seconds) 


// PWM
#define  PIN_PWM    D6      //heat_pwm on pin D6(GPIO6) Wemos D1 Mini PWM has 10-bit resolution, and the PWM frequency between 100 Hz and 1 kHz.

int           heat_pwm        = 0;
int           heat_min        = 0;
int           heat_max        = 1024;
int           heat_step       = 1;

//Homie
HomieNode winecoolerNode("winecooler", "temperature", "temperature"); /* middelste parm toegevoegd bij upg naar Homie 3.0.0 */

// VOORBEELD:  mosquitto_pub -t 'homie/dev00x/$implementation/config/set' -m '{"settings":{"setpoint":12}}' -r
//  zorg er voor dat er al een "settings":{}  in de initiële config zit.
HomieSetting<long>      publish_intervalSetting("publish_interval", "temp interval in seconds");
HomieSetting<double>            setpointSetting("setpoint"        , "temp setpoint");
HomieSetting<double>          hysteresisSetting("hysteresis"      , "temp hysteresis");
HomieSetting<long>       adjust_intervalSetting("adjust_interval" , "adjust interval");
HomieSetting<long>              cool_maxSetting("cool_max"        , "max cool potmeter");

void loopHandler() {
  if (millis() - last_publish >= publish_intervalSetting.get() * 1000UL || last_publish == 0) {

    winecoolerNode.setProperty("data").send("{"+String(PublishString)+"}");
    
    // last_publish = millis();  // NIET hier maar in loop() function 
  }
}

void onHomieEvent(const HomieEvent& event) {
  switch (event.type) {
    case HomieEventType::STANDALONE_MODE:
      Serial << "Standalone mode started" << endl;
      break;
    case HomieEventType::CONFIGURATION_MODE:
      Serial << "Configuration mode started" << endl;
      break;
    case HomieEventType::NORMAL_MODE:
      Serial << "Normal mode started" << endl;
      break;
    case HomieEventType::OTA_STARTED:
      Serial << "OTA started" << endl;
      break;
    case HomieEventType::OTA_PROGRESS:
      Serial << "OTA progress, " << event.sizeDone << "/" << event.sizeTotal << endl;
      break;
    case HomieEventType::OTA_FAILED:
      Serial << "OTA failed" << endl;
      break;
    case HomieEventType::OTA_SUCCESSFUL:
      Serial << "OTA successful" << endl;
      break;
    case HomieEventType::ABOUT_TO_RESET:
      Serial << "About to reset" << endl;
      break;
    case HomieEventType::WIFI_CONNECTED:
      Serial << "Wi-Fi connected, IP: " << event.ip << ", gateway: " << event.gateway << ", mask: " << event.mask << endl;
      break;
    case HomieEventType::WIFI_DISCONNECTED:
      Serial << "Wi-Fi disconnected, reason: " << (int8_t)event.wifiReason << endl;
      break;
    case HomieEventType::MQTT_READY:

      Serial << "MQTT connected" << endl;

      // idea based on https://github.com/homieiot/homie-esp8266/issues/138
      strcpy(topic, Homie.getConfiguration().mqtt.baseTopic);
      strcat(topic, Homie.getConfiguration().deviceId);
      strcat_P(topic, PSTR("/"));
      strcat(topic, winecoolerNode.getId());
      strcat_P(topic, PSTR("/testing"));

      Serial << "MQTT Subscribing to topic: " << topic << endl;
      packetIdSub = mqttClient.subscribe(topic, 0);
      Serial << "MQTT Subscribing at QoS 0, packetId: " << packetIdSub << endl;

      break;
    case HomieEventType::MQTT_DISCONNECTED:
      Serial << "MQTT disconnected, reason: " << (int8_t)event.mqttReason << endl;
      break;
    case HomieEventType::MQTT_PACKET_ACKNOWLEDGED:
      // Serial << "MQTT packet acknowledged, packetId: " << event.packetId << endl;
      break;
    case HomieEventType::READY_TO_SLEEP:
      Serial << "Ready to sleep" << endl;
      break;
    case HomieEventType::SENDING_STATISTICS:
      Serial << "Sending statistics" << endl;
      break;
  }
}

void onMqttSubscribe(uint16_t packetId, uint8_t qos) {
  Serial.println("Subscribe acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
  Serial.print("  qos: ");
  Serial.println(qos);
}

void onMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total) {
// https://github.com/marvinroger/async-mqtt-client/issues/92  
  if (index == 0) {
    payloadBuf = "";
  }

  auto pl = len;
  auto p = payload;
  while (pl--) {
    payloadBuf += *(p++);
  }

  if (index + len == total) {

      Serial << "payloadBuf=" << payloadBuf << endl;
      if (payloadBuf != "false") {


        int i1 = payloadBuf.indexOf(',');
        int i2 = payloadBuf.indexOf(',',i1+1);
        int i3 = payloadBuf.indexOf(',',i2+1);
        int i4 = payloadBuf.indexOf(',',i3+1);
        int i5 = payloadBuf.indexOf(',',i4+1);
        int i6 = payloadBuf.indexOf(',',i5+1);
        int i7 = payloadBuf.indexOf(',',i6+1);
        int i8 = payloadBuf.indexOf(',',i7+1);

        payloadBuf_substr = payloadBuf.substring(0, i1);
        testing_temp0     = atof(payloadBuf_substr.c_str());

        payloadBuf_substr = payloadBuf.substring(i1 + 1, i2);
        setpoint          = atof(payloadBuf_substr.c_str());

        payloadBuf_substr = payloadBuf.substring(i2 + 1, i3);
        hysteresis        = atof(payloadBuf_substr.c_str());

        payloadBuf_substr = payloadBuf.substring(i3 + 1, i4);
        cool_step         = atoi(payloadBuf_substr.c_str());

        payloadBuf_substr = payloadBuf.substring(i4 + 1, i5);
        cool_max          = atoi(payloadBuf_substr.c_str());

        payloadBuf_substr = payloadBuf.substring(i5 + 1, i6);
        testing_cool_pot  = atoi(payloadBuf_substr.c_str());

        payloadBuf_substr = payloadBuf.substring(i6 + 1, i7);
        heat_step         = atoi(payloadBuf_substr.c_str());

        payloadBuf_substr = payloadBuf.substring(i7 + 1, i8);
        testing_heat_pwm  = atoi(payloadBuf_substr.c_str());

        payloadBuf_substr = payloadBuf.substring(i8 + 1);
        adjust_interval   = atoi(payloadBuf_substr.c_str());

        last_adjust = 0; // forceer onmiddelijke adjust (eerste keer na restart device zal mogelijk niet direct reactie zijn, ivm millis<adjust_interval)

        // ....  -t 'homie/dev00x/testing/temp0' -m '9.0,11.2,0.4,1,75,0,1,0,20'    <==== enige juiste formaat, hieronder paar voorbeelden t.b.v juiste positional parm in kunnen vullen.

        // ....  -t 'homie/dev00x/testing/temp0' -m '              9.0,         11.2,           0.4,          1,         75,                 0,          1,                 0,                20'
        // ....  -t 'homie/dev00x/testing/temp0' -m 'testing_temp0=9.0,setpoint=11.2,hysteresis=0.4,cool_step=1,cool_max=75,testing_cool_pot=0,heat_step=1,testing_heat_pwm=0,adjust_interval=20'

        // When testing_temp0    = 0.0, then the real temperture sensor wiil be used, in stead of this manual testing override.
        // When testing_cool_pot = 0,   then the current cool_pot value wiil be used, in stead of this manual testing override.
        // When testing_heat_pwm = 0,   then the current heat_pwm value wiil be used, in stead of this manual testing override.
      }


  }

}

void setup() {
  Serial.begin(115200);
  Serial << endl << endl;

  Homie_setFirmware("winecooler", "3.0.7");
  //1.1.3 - met nieuwe ESP8266 2.4.0-rc2
  //1.1.4 - eerste versie met light sensor er bij
  //1.1.5 - relay 'modulatie' verwarming te krachtig
  //1.1.6 - nu met 'officiele' BH1750 lib https://github.com/claws/BH1750 met aanpassing, zie library "BH1750erik - master"
  //1.1.7 - geldende modcyclus en modprocent naar 'modulation' topic
  //1.9.0 - correctie versie nummering en belangerijke aanpassingen:
  //        - digitale potmeter
  //        - verwijderen licht sensoren
  //        - analoge meting van voltage peltier element, gebruik weerstanden nul - 10k - 33k -> naar plus van peltier
  //          en verbind 10uF condensator over 10k weerstand, en sluit punt tussen 10k en 33k weerstand aan op A0.
  //1.9.1 - potprocent via settings ivm vasthouden waarde na herstart wemos
  //1.9.2 - aanpassen voor laatste versie libraries: ESP8266 van 2.4.0-rc2 naar 2.6.3 (Blijkbaar heet Wemos nu LOLIN(WEMOS) D1 R2 & Mini)
  //1.10.1 - 20200414:
  //         aanpassen voor Homie 3.0.0 
  //         ook nodig library ArduinoJSON van 5.11.2 naar 6.15.1
  //         update library Bounce2 v2.3 naar 2.53
  //         update OneWire van 2.3.3 naar 2.3.5
  //         update DallasTemperature van 3.7.6 naar 3.8.0
  //         update ESPAsyncTCP-master at version 1.1.0 ===> 1.2.2
  //         update ESPAsyncWebServer-master at version 1.1.0 ===> 1.2.3 
  //         update async-mqtt-client-master at version 0.8.1 ===> 0.8.2
  //2.0.0  - 20200414:
  //         Volledige temp regeling, V1.x is alleen regeling voor warmte element, koude is hard met potmeter ingesteld
  //         Add heap / memory topic and logging at start
  //2.0.1  - 20200515: Aanpassingen voor Platformio en Github
  //         20200917  + Homie.events() ivm debugging
  //2.0.2  - 20210825: Re-compile met laatset nieuwe libs na re-install VSC op nieuwe laptop
  //2.0.3  - 20210827: voorkomen regelmatige reconnects/restarts met Homie.getMqttClient().setKeepAlive(75)
  //2.0.4  - 20220731: dynamische potmeter adjust (in addition to original NTC resistor temp control)
  //                   If "setpoint" = 0.0 then reset adjust potmeter
  //                   If "potintv" = 0 then NO auto adjust potmeter
  //2.0.5  - 20241018: Config file is not valid, reason: mqtt.password is too long https://github.com/homieiot/homie-esp8266/issues/661 aangepast ivm HomeAssistant lengte password
  //          zie: .pio\libdeps\d1_mini\Homie\src\Homie\Limits.hpp : aanpassing MAX_MQTT_CREDS_LENGTH = 32 + 1; naar MAX_MQTT_CREDS_LENGTH = 64 + 1;
  //3.0.0/3 - 20241101 MAJOR rework, rename naar WineCooler
  //                   volledig temp control in software, verwijder NTC uit regelcircuit
  //                   heater nu niet meer via relay maar PWM control
  //                   veel meer... 
  //3.0.4   - 20241122 get config settings aan begin van loop(), zodat nu ook volledig zonder connectie toch temp control werkt
  //3.0.5   - 20241123 sensor.begin naar setup() en last_adjust=0 na testing msg.
  //3.0.6   - 20250105 allow testing cool_max > 75 (beacause I see still cool_level > 0, even when cool_pot = 75 = current fixex cool_max)
  //3.0.7   - 20250106 cool_max set-able thru config.json
  
  Homie.getLogger() << "Compiled: " << __DATE__ << " | " << __TIME__ << " | " << __FILE__ <<  endl;
  Homie.getLogger() << "ESP CoreVersion       : " << ESP.getCoreVersion() << endl;
  Homie.getLogger() << "ESP FreeSketchSpace   : " << ESP.getFreeSketchSpace() << endl;
  Homie.getLogger() << "ESP FreeHeap          : " << ESP.getFreeHeap() << endl;
  Homie.getLogger() << "ESP HeapFragmentation : " << ESP.getHeapFragmentation() << endl;

  Homie.setLoopFunction(loopHandler);
  winecoolerNode.advertise("data").setName("Data").setDatatype("String");

  publish_intervalSetting.setDefaultValue(DEFAULT_PUBLISH_INTERVAL).setValidator([] (long candidate) { return candidate > 0; });
  setpointSetting.setDefaultValue(DEFAULT_TEMP_SETPOINT).setValidator([] (double candidate) { return candidate >= 0;   });
  hysteresisSetting.setDefaultValue(DEFAULT_TEMP_HYSTERESIS).setValidator([] (double candidate) { return candidate >= 0.1; });
  adjust_intervalSetting.setDefaultValue(DEFAULT_ADJUST_INTV).setValidator([] (long candidate) { return candidate == 0 || candidate >= 60; });
  cool_maxSetting.setDefaultValue(DEFAULT_COOL_MAX).setValidator([] (long candidate) { return candidate >= 75; });
  
  pot.begin(CS,INC,UD); // Initialize Digital potentiometer X9C
  sensors.begin();      // Initialize Digital thermometer DS18B20

  Homie.onEvent(onHomieEvent);

  Homie.getMqttClient().setKeepAlive(75);  //  Zie o.a. https://gitter.im/homie-iot/ESP8266?at=60a3ca03b10fc85b56a3029e  en
                                           //           https://gitter.im/homie-iot/ESP8266?at=58aa156421d548df2c2ee530
                                           //           https://github.com/homieiot/homie-esp8266/issues/340
                                           //           http://www.steves-internet-guide.com/mqtt-keep-alive-by-example/

  mqttClient.onSubscribe(onMqttSubscribe);
  mqttClient.onMessage(onMqttMessage);

  Homie.setup();

}

void loop() {

  if (get_settings == true) {
    setpoint        = setpointSetting.get();
    hysteresis      = hysteresisSetting.get();
    adjust_interval = adjust_intervalSetting.get();
    cool_max        = cool_maxSetting.get();  // Based on resistor 20k + digi pot 10k  (so no original ntc and no 8.2k resistor)
    cool_pot        = cool_max;               // initial value of cool_pot
    get_settings    = false;
  }

  if (millis() - last_publish >= publish_intervalSetting.get() * 1000UL || last_publish == 0) {

    sensors.requestTemperatures();        // Send the command to get temperatures, takes 1 second.
    temp0 = sensors.getTempCByIndex(0);   // winecooler inside temp
    if (testing_temp0 != 0.0) temp0 = testing_temp0;
    temp1 = sensors.getTempCByIndex(1);  // winecooler outside temp
    
    cool_level = analogRead(PIN_ANALOG);

    PublishString =  "\"heap_free\": "+       String(ESP.getFreeHeap())+",";
    PublishString += "\"heap_frag\": "+       String(ESP.getHeapFragmentation())+",";
    PublishString += "\"adjust_interval\": "+ String(adjust_interval)+",";
    PublishString += "\"cool_level\": "+      String(cool_level)+",";
    PublishString += "\"cool_pot\": "+        String(cool_pot)+",";
    PublishString += "\"cool_step\": "+       String(cool_step)+",";
    PublishString += "\"heat_pwm\": "+        String(heat_pwm)+",";
    PublishString += "\"heat_step\": "+       String(heat_step)+",";
    PublishString += "\"hysteresis\": "+      String(hysteresis)+",";
    PublishString += "\"publish_interval\": "+String(publish_intervalSetting.get())+",";
    PublishString += "\"setpoint\": "+        String(setpoint)+",";
    PublishString += "\"temp0\": "+           String(temp0)+",";
    PublishString += "\"temp1\": "+           String(temp1); //no trailing comma!!

    Serial << "PublishString=" << PublishString << endl;

    last_publish = millis();
    }

  if ((millis() - last_adjust >= adjust_interval * 1000UL && adjust_interval != 0 )  || last_adjust == 0) {
    
    if(setpoint > 0.0 && temp0 > -85.00 && temp0 < 85.00 && adjust_interval != 0UL)   {    // temp0  +/- 85.00 or -127.00 are invalid (disconneted, wiring pull up resistor or very first measurement)


      if (testing_cool_pot != 0) {
        cool_pot = testing_cool_pot;
        testing_cool_pot = 0;
      };

      if (testing_heat_pwm != 0) {
        heat_pwm = testing_heat_pwm;
        testing_heat_pwm = 0;
      };

      if (temp0 < (setpoint - hysteresis) ) 
        {
        if (cool_pot < cool_max) cool_pot=cool_pot+cool_step;
        if (cool_pot > cool_max) cool_pot=cool_max;

        if (cool_pot == cool_max) heat_pwm=heat_pwm+(heat_step*10);
        if (heat_pwm > heat_max)  heat_pwm=heat_max;
      }

      if (temp0 > (setpoint + hysteresis) ) 
        {
        if (cool_pot <= cool_max && cool_pot > cool_min && heat_pwm == heat_min) // currently within cooling range
          {  
          cool_pot=cool_pot-cool_step;
          if (cool_pot < cool_min) cool_pot=cool_min;
          };

        if (heat_pwm > heat_min && heat_pwm <= heat_max ) // currently in heating range
          {  
          heat_pwm = heat_pwm-(heat_step*10);
          if (heat_pwm < heat_min) heat_pwm=heat_min;
          };
      }            

    pot.setPot(uint16_t(cool_pot),true);       // true=save, so pot will keep value after shutdown if you do nothing else...
    analogWrite(PIN_PWM, heat_pwm);            // BE AWARE, range from 0-1024 !!  
    }

  last_adjust = millis();
  }

  Homie.loop();
}
