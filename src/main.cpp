#include <Arduino.h>
#include <Homie.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <X9C.h>
#include <U8g2lib.h>

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
float          dyncfg_temp0    = 0.0;   // used for dyncfg override 
int            dyncfg_cool_pot = 0;     // used for dyncfg override 
int            dyncfg_heat_pwm = 0;     // used for dyncfg override 
float          temp1            = -127.00;
int            adjust_interval;

// Temperature sensor on D5, place pull-up resistor between 2.2K or 4.7K to 5V
#define        ONE_WIRE_BUS D5      //temp sensor DS18B20 on D5(5)

// Dallas temperature variables
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
DeviceAddress addr_temp0,addr_temp1; // device addresses

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

// Reset Other
#define  PIN_RST_OTHER    D7      // Send pulse low to be used as remote reset of other arduino

// OLED display
U8G2_SH1106_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE); // Pins  D1 = SCL, D2 = SDA standaard Wemos D1 Mini
String temp0_String;
String temp1_String;

//Homie
HomieNode winecoolerNode("winecooler", "temperature", "temperature"); /* middelste parm toegevoegd bij upg naar Homie 3.0.0 */


// VOORBEELD:  mosquitto_pub -t 'homie/dev00x/$implementation/config/set' -m '{"settings":{"setpoint":12}}' -r
//  zorg er voor dat er al een "settings":{}  in de initiële config zit.
HomieSetting<long>      publish_intervalSetting("publish_interval", "temp interval in seconds");
HomieSetting<double>            setpointSetting("setpoint"        , "temp setpoint");
HomieSetting<double>          hysteresisSetting("hysteresis"      , "temp hysteresis");
HomieSetting<long>       adjust_intervalSetting("adjust_interval" , "adjust interval");
HomieSetting<long>              cool_maxSetting("cool_max"        , "max cool potmeter");
HomieSetting<const char*>      txt_temp0Setting("txt_temp0"        , "display txt_temp0");
HomieSetting<const char*>      txt_temp1Setting("txt_temp1"        , "display txt_temp1");
char txt_temp0[20];
char txt_temp1[20];

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
  
  
  bool DynConfigHandler(const HomieRange& range, const String& value) {
    
    String         value_substr;
    
    winecoolerNode.setProperty("dyncfg").send(value);
    Serial << "dynamic re-config value='" << value << "'" << endl;
    
    if (value != "") {
      int i1 = value.indexOf(',');
      int i2 = value.indexOf(',',i1+1);
      int i3 = value.indexOf(',',i2+1);
      int i4 = value.indexOf(',',i3+1);
      int i5 = value.indexOf(',',i4+1);
      int i6 = value.indexOf(',',i5+1);
      int i7 = value.indexOf(',',i6+1);
      int i8 = value.indexOf(',',i7+1);

      value_substr = value.substring(0, i1);
      dyncfg_temp0     = atof(value_substr.c_str());
    
      value_substr = value.substring(i1 + 1, i2);
      setpoint          = atof(value_substr.c_str());

      value_substr = value.substring(i2 + 1, i3);
      hysteresis        = atof(value_substr.c_str());

      value_substr = value.substring(i3 + 1, i4);
      cool_step         = atoi(value_substr.c_str());

      value_substr = value.substring(i4 + 1, i5);
      cool_max          = atoi(value_substr.c_str());

      value_substr = value.substring(i5 + 1, i6);
      dyncfg_cool_pot  = atoi(value_substr.c_str());

      value_substr = value.substring(i6 + 1, i7);
      heat_step         = atoi(value_substr.c_str());

      value_substr = value.substring(i7 + 1, i8);
      dyncfg_heat_pwm  = atoi(value_substr.c_str());

      value_substr = value.substring(i8 + 1);
      adjust_interval   = atoi(value_substr.c_str());

      last_adjust = 0; // forceer onmiddelijke adjust (eerste keer na restart device zal mogelijk niet direct reactie zijn, ivm millis<adjust_interval)

      // ....  -t 'homie/dev00x/winecooler/dyncfg/set' -m '9.0,11.2,0.4,1,75,0,1,0,20'    <==== enige juiste formaat, hieronder paar voorbeelden t.b.v juiste positional parm in kunnen vullen.

      // ....  -t 'homie/dev00x/winecooler/dyncfg/set' -m '              9.0,         11.2,           0.4,          1,         75,                 0,          1,                 0,                20'
      // ....  -t 'homie/dev00x/winecooler/dyncfg/set' -m 'dyncfg_temp0=9.0,setpoint=11.2,hysteresis=0.4,cool_step=1,cool_max=75,dyncfg_cool_pot=0,heat_step=1,dyncfg_heat_pwm=0,adjust_interval=20'

      // When dyncfg_temp0    = 0.0, then the real temperture sensor wiil be used, in stead of this manual dyncfg override.
      // When dyncfg_cool_pot = 0,   then the current cool_pot value wiil be used, in stead of this manual dyncfg override.
      // When dyncfg_heat_pwm = 0,   then the current heat_pwm value wiil be used, in stead of this manual dyncfg override.

    }

  return true;
}

bool PulseLowHandler(const HomieRange& range, const String& value) {
  if (value != "true" ) return false;

// ....   -t 'homie/dev00x/winecooler/rst_other/set' -m 'true'

  digitalWrite(PIN_RST_OTHER, LOW);
  winecoolerNode.setProperty("rst_other").send(value);
  Serial << "PIN_RST_OTHER is pulsed LOW" <<endl;
  delay(100);
  digitalWrite(PIN_RST_OTHER, HIGH);

  return true;
}

void setup() {
  Serial.begin(115200);
  Serial << endl << endl;

  Homie_setFirmware("winecooler", "3.0.12");
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
  //3.0.5   - 20241123 sensor.begin naar setup() en last_adjust=0 na dyncfg msg.
  //3.0.6   - 20250105 allow dyncfg cool_max > 75 (beacause I see still cool_level > 0, even when cool_pot = 75 = current fixex cool_max)
  //3.0.7   - 20250106 cool_max set-able thru config.json
  //3.0.8   - 20250415 winecooler nu in repo https://github.com/zz04303/homie-esp8266.git#erik  (met achter hash de naam van de branch 'erik')
  //                   DS18B20 sensors set resolution explicitly 
  //3.0.9   - 20250416 Redo with corrrect repo https://github.com/zz04303/homie-esp8266.git#erik  (met achter hash de naam van de branch 'erik' = 'develop' + 2 commits)
  //3.0.10  - 20251116 add pulse low to be used for rsetting an other arduino via output pin D1 to be connected to RST in on the other arduino
  //                   add cool_max to PublishString
  //3.0.11  - 20251118 change mqtt setup for topic 'testing' to regular Homie Handler with topic 'dyncfg' 
  //3.0.12  - 20251121 add I2C display, using https://github.com/olikraus/U8g2_Arduino (uses pins D1 and D2, which are the Wemos D1 mini pins for SCL and SDA respectively )
  //                   use pin D7 in stead of D1 for rst_other (using D3 or D4 failed, arduino's forever resetting)
  
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
  txt_temp0Setting.setDefaultValue("");
  txt_temp1Setting.setDefaultValue("");
  
  pot.begin(CS,INC,UD); // Initialize Digital potentiometer X9C
  sensors.begin();      // Initialize Digital thermometer DS18B20
  sensors.getAddress(addr_temp0, 0);
  sensors.getAddress(addr_temp1, 1);
  sensors.setResolution(addr_temp0,11); // The resolution of the temperature sensor is user-configurable to 9, 10, 11, or 12 bits, corresponding to increments of 0.5°C, 0.25°C, 0.125°C, and 0.0625°C, respectively.
  sensors.setResolution(addr_temp1,11); // The default resolution at power-up is 12-bit (ref:  https://www.analog.com/media/en/technical-documentation/data-sheets/ds18b20.pdf )

  Homie.onEvent(onHomieEvent);

  Homie.getMqttClient().setKeepAlive(75);  //  Zie o.a. https://gitter.im/homie-iot/ESP8266?at=60a3ca03b10fc85b56a3029e  en
                                           //           https://gitter.im/homie-iot/ESP8266?at=58aa156421d548df2c2ee530
                                           //           https://github.com/homieiot/homie-esp8266/issues/340
                                           //           http://www.steves-internet-guide.com/mqtt-keep-alive-by-example/

  winecoolerNode.advertise("rst_other").setName("ResetOther").setDatatype("boolean").settable(PulseLowHandler);
  pinMode(PIN_RST_OTHER, OUTPUT);
  digitalWrite(PIN_RST_OTHER, HIGH);
  
  winecoolerNode.advertise("dyncfg").setName("DynConfig").setDatatype("boolean").settable(DynConfigHandler);  
  // in above statement earlier names like "dyn_cfg" and/or "Dynamic Configuration" caused troubles (exceptions at runtime). Maybe due to underscore or space in either names? )

  u8g2.begin(); // OLED display

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
    if (dyncfg_temp0 != 0.0) temp0 = dyncfg_temp0;
    temp1 = sensors.getTempCByIndex(1);  // winecooler outside temp
    
    cool_level = analogRead(PIN_ANALOG);

    PublishString =  "\"heap_free\": "+       String(ESP.getFreeHeap())+",";
    PublishString += "\"heap_frag\": "+       String(ESP.getHeapFragmentation())+",";
    PublishString += "\"adjust_interval\": "+ String(adjust_interval)+",";
    PublishString += "\"cool_level\": "+      String(cool_level)+",";
    PublishString += "\"cool_max\": "+        String(cool_max)+",";
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

    temp0_String = String(temp0);
    temp1_String = String(temp1);
    strcpy(txt_temp0, txt_temp0Setting.get());
    strcpy(txt_temp1, txt_temp1Setting.get());
    char empty_string[] = "";

    u8g2.firstPage();
    do {
      u8g2.setFont(u8g2_font_ncenB12_tr);
      if (strcmp(txt_temp0,empty_string) > 0 ) {
        u8g2.drawStr(0,24,txt_temp0);
        u8g2.drawStr(0,48,temp0_String.c_str());
      }
      if (strcmp(txt_temp1,empty_string) > 0 ) {
      u8g2.drawStr(64,24,txt_temp1);
      u8g2.drawStr(64,48,temp1_String.c_str());
      }
    } while ( u8g2.nextPage() );    

    last_publish = millis();
    }

  if ((millis() - last_adjust >= adjust_interval * 1000UL && adjust_interval != 0 )  || last_adjust == 0) {
    
    if(setpoint > 0.0 && temp0 > -85.00 && temp0 < 85.00 && adjust_interval != 0UL)   {    // temp0  +/- 85.00 or -127.00 are invalid (disconneted, wiring pull up resistor or very first measurement)


      if (dyncfg_cool_pot != 0) {
        cool_pot = dyncfg_cool_pot;
        dyncfg_cool_pot = 0;
      };

      if (dyncfg_heat_pwm != 0) {
        heat_pwm = dyncfg_heat_pwm;
        dyncfg_heat_pwm = 0;
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
