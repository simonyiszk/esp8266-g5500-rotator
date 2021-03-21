#include <WiFiManager.h>
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <ArduinoOTA.h>


#define ROTCTLD_PORT 4533
#define MAX_CLIENTS 4 // TODO
#define ADC_AVG_CNT 50

#define AZ_HIST 2
#define EL_HIST 2

#define PIN_AZ_L 4
#define PIN_AZ_R 0
#define PIN_EL_D 12
#define PIN_EL_U 14
#define PIN_MOT_EN 5

#define PIN_ADC_AZ_DIS 16
#define PIN_ADC_EL_DIS 13


WiFiManager wifiManager;
WiFiServer server(ROTCTLD_PORT);
WiFiClient client; // TODO: concurrency

float p_az = 180, p_el = 0; // Parking
float c_az = 0, c_el = 0; // Current
float t_az = 0, t_el = 0; // Target

int az_values[ADC_AVG_CNT] = {0}, el_values[ADC_AVG_CNT] = {0};
int adc_az_0 = 32, adc_az_450 = 900;
int adc_el_0 = 32, adc_el_180 = 700;

boolean motor_enable = false;

void setup_ota() {
  ArduinoOTA.onStart([]() {
    Serial.println("Start");
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });
  
  // ArduinoOTA.setPort(8266);
  ArduinoOTA.setHostname("esp8266-rotator");
  ArduinoOTA.setPassword((const char *)"123");
  ArduinoOTA.begin();
}

void handle_motor() {
  digitalWrite(PIN_AZ_L, (t_az < c_az - AZ_HIST) && motor_enable);
  digitalWrite(PIN_AZ_R, (t_az > c_az + AZ_HIST) && motor_enable);
  digitalWrite(PIN_EL_D, (t_el < c_el - EL_HIST) && motor_enable);
  digitalWrite(PIN_EL_U, (t_el > c_el + EL_HIST) && motor_enable);
  
  if( (t_az < c_az - AZ_HIST) && (t_az > c_az + AZ_HIST) && (t_el < c_el - EL_HIST) && (t_el > c_el + EL_HIST) ) motor_enable = false;
  digitalWrite(PIN_MOT_EN, motor_enable);
}

int read_adc_mux(int pin) {
  digitalWrite(pin, 0);
  int adc = analogRead(A0);
  digitalWrite(pin, 1);
  return adc;
}

float read_adc_mux_map(int pin, int end_low, int end_high, int real_low, int real_high) {
  return map(read_adc_mux(pin), end_low, end_high, real_low, real_high);
}

int adc_read_iter = 0;
void read_position() {
  adc_read_iter++;
  if(adc_read_iter == ADC_AVG_CNT) adc_read_iter = 0;
  az_values[adc_read_iter] = read_adc_mux_map(PIN_ADC_AZ_DIS, adc_az_0, adc_az_450, 0, 450);
  el_values[adc_read_iter] = read_adc_mux_map(PIN_ADC_EL_DIS, adc_el_0, adc_el_180, 0, 180);

  float az_sum = 0, el_sum = 0;
  for(int i = 0; i < ADC_AVG_CNT; i++) {
    az_sum += az_values[i];
    el_sum += el_values[i];
  }

  c_az = az_sum / ADC_AVG_CNT;
  c_el = el_sum / ADC_AVG_CNT;
  
  /*if(motor_enable) {
    // Simulation
    if(t_az < c_az) c_az -= 0.008;
    if(t_az > c_az) c_az += 0.008;
    if(t_el < c_el) c_el -= 0.003;
    if(t_el > c_el) c_el += 0.003;
  }*/
}

void set_target_to_current() {
  t_az = c_az;
  t_el = c_el;
}

void check_clients() {
  WiFiClient new_client = server.available();
  if (new_client) {
    client = new_client;
    client.setTimeout(1000);
  }

  if(client.connected()) {
    if (client.available()) {
      
      /*bool extended = false;
      char record_ending = '\n';*/
      byte req = client.read();
      /*if(req == '+') {
        extended = true;
        req = client.read();
        if(ispunct(req)) {
          record_ending = req;
          req = client.read();
        }
      }*/
      
      if (req == 'p') { // get_pos
        client.printf("%.3f\n%.3f\n", c_az, c_el);
        
      } else if (req == 'P') { // set_pos
        float set_az, set_el;
        client.read();
        sscanf(client.readStringUntil(' ').c_str(), "%f", &set_az);
        sscanf(client.readStringUntil('\n').c_str(), "%f", &set_el);
        t_az = set_az;
        t_el = set_el;
        motor_enable = true;
        client.print("RPRT 0\n");
        
      } else if (req == 'S') { // stop
        motor_enable = false;
        client.print("S\n");
        
      } else if (req == 'K') { // park
        t_az = p_az;
        t_el = p_el;
        motor_enable = true;
        client.print("K\n");
        
      } else if (req == 'M') { // move
        client.print("RPRT -4\n");
        
      } else if (req == '_') { // info
        client.print("Wireless rotctld emulator for Yaesu G-5500 - v2021.03.21 HA5KFU\n");
        
      } else if (req == 'D') { // debug
        client.printf("ADC0 shorted: %d\n", analogRead(0));
        client.printf("ADC azimuth: %d\n", read_adc_mux(PIN_ADC_AZ_DIS));
        client.printf("ADC elevation: %d\n", read_adc_mux(PIN_ADC_EL_DIS));
        client.printf("Current state  AZ: %3.5f, EL: %3.5f\n", c_az, c_el);
        client.printf("Target state   AZ: %3.5f, EL: %3.5f\n", t_az, t_el);
        client.printf("Parking state  AZ: %3.5f, EL: %3.5f\n", p_az, p_el);
        
      } else if (req == 'q') { // close
        client.stop();
        
      } else {
        client.print("RPRT -1\n");
      }
      client.read(); // Read ending newline
      client.flush(); // Flush write buffer
    }
  }
}

void setup() {
  Serial.begin(115200);

  // Sensor
  digitalWrite(PIN_ADC_AZ_DIS, HIGH);
  pinMode(PIN_ADC_AZ_DIS, OUTPUT);
  digitalWrite(PIN_ADC_EL_DIS, HIGH);
  pinMode(PIN_ADC_EL_DIS, OUTPUT);

  // Motor
  digitalWrite(PIN_AZ_L, LOW);
  pinMode(PIN_AZ_L, OUTPUT);
  digitalWrite(PIN_AZ_R, LOW);
  pinMode(PIN_AZ_R, OUTPUT);
  digitalWrite(PIN_EL_D, LOW);
  pinMode(PIN_EL_D, OUTPUT);
  digitalWrite(PIN_EL_U, LOW);
  pinMode(PIN_EL_U, OUTPUT);
  digitalWrite(PIN_MOT_EN, LOW);
  pinMode(PIN_MOT_EN, OUTPUT);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, 1);

  read_position();
  set_target_to_current();
  
  WiFi.mode(WIFI_STA);
  wifiManager.autoConnect("HA5KFU_rotator");
  digitalWrite(LED_BUILTIN, 0);
  
  setup_ota();

  MDNS.begin("rotator");
  MDNS.addService("rotctld", "tcp", ROTCTLD_PORT);
  server.begin();
}

void loop() {
  ArduinoOTA.handle();
  MDNS.update();

  check_clients();
  read_position();
  handle_motor();

  delay(10);
}
