
#include <LiquidCrystal_I2C.h>
#include <HTTPClient.h>
#include "DHT.h"
#include <Ticker.h>
#include "esp_timer.h"
#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>


#define DHT1PIN 25
#define DHT2PIN 27
#define DHTTYPE DHT22
#define triac_pin 14
#define In_AC 26
#define NN1 19
#define NN2 18
#define NN3 5
#define NN4 17
#define Motor220V 12
#define Buzz 15
#define PUMP_PIN 4
#define FAN_PIN 2
#define HUMI_PIN 16
#define FAN220V 33
#define LIGHT_UV 32

WidgetLED led1(12);
WidgetLED led2(13);
WidgetLED led3(14);
WidgetLED led4(15);
WidgetLED led5(16);
WidgetLED led6(17);
WidgetLED led7(18);
WidgetLED led8(19);
WidgetLED led9(20);
WidgetLED led10(21);
WidgetLED led11(22);
//WidgetLED ledSystem(43);
WidgetLED online_state_led(43);






DHT dht1(DHT1PIN, DHTTYPE);
DHT dht2(DHT2PIN, DHTTYPE);
LiquidCrystal_I2C lcd(0x27, 20, 4);

String Web_App_URL = "https://script.google.com/macros/s/AKfycbz4dm4H2Peli5cflGkwtjFROdzLV-FHY6bh35v6zUT_doKtu-ZVpD0p128Ufrm7jaD8Tw/exec";
// String Web_App_URL = "https://script.google.com/macros/s/AKfycbxTglehxYSjHzqlJTh2HhvnMScEyEj72wD5xGwfj38jHpG9cc6NkzX7mLMlm0j-tkonfw/exec";




TaskHandle_t wifi_blynk_task;
Ticker triacTicker;
volatile bool zeroCrossed = false;

int RUN, SYSTEM;

char* blynk_net = "ashthieu.ddns.net";
uint16_t blynk_port = 1713;
char* blynk_token = "Eal8oXbLKDg-kZ-XU4k5kuP0STqX5nBd";

// char* blynk_token = "Pj3ffOmIhQZK3-qlo1A_cdWJEjUoQWoo";

char* wifi_network = "SaKe Quan 2";
char* wifi_password = "10021954";
// char* wifi_network = "honghai";
// char* wifi_password = "21022002";

//temp1,temp2,humi1,humi2,setpoint, sethumi, hours,minute, sec,tgdao,  firingdelay,alpha,
String sample[15];
float sample_float[15 + 3];
// fanState, humiState, pumpState, motorState, uvLightOn,fan220V, nutDAO,
int state[10] = { 0 };

const unsigned long runtime_offline = 35 * 24 * 60 * 60 * 1000;  // 32 ngày (tính bằng milliseconds)
unsigned long runtime = runtime_offline;
unsigned long previous_millis_timer = 0;
long long remaining_time = 0;
int online_mode = 0;


// Variables for threshold controller
const float temp_thres = 0.92;
int firing_thres = 4500;


const unsigned long Gui_GG = 10000;  //60 * 1000;  //60 * 60 *
unsigned long time_sheet = Gui_GG;
unsigned long previous_millis_p = 0.0;
const unsigned long sampling_time_p_set = 1 * 1000;
unsigned long sampling_time_p = sampling_time_p_set;
volatile int firing_delay = 0;
const float kp_fit = -1200;
float kp = kp_fit;
const float firing_delay_max = 9000;


const float humiset_off = 65.0;
const float tempset_off = 37.0;
const int hourset_off = 12;
const int minuset_off = 0;
const int secset_off = 0;
const int timeset_off = 9;
float GT_CD = 0.5f;

unsigned long pumpStartTime = 0, previousMillis_UV = 0, previousMillis_pump = 0, previous_millis_sheet = 0, previousMillis_count = 0, previousMillis_check = 0;  //tg chạy động cơ
int pinValue = 1, hours, minutes, seconds, timeoff;
int mode = 1, cai_dat = 1;
int water = 0;




//DCDAO//
unsigned long startMillis = 0;
unsigned long lastModeChange = 0;  // NN1
const long interval = 1000;        // count sau 1s


const unsigned long simu_update_sensors_time = 1 * 1000;  // 1s
unsigned long previous_millis_simu = 0;
float temp_simu = 32.5;
float humi_simu = 46.0;
///////////////////////////////////////////////////////////////////////////////////////
void wait_function(int waiting_time_ms) {
  unsigned long curr_time = millis();
  while (!((millis() - curr_time) >= waiting_time_ms))
    ;
}


void read_sensor() {
  for (int i = 0; i < 2; i++) {
    sample_float[i] = (i == 0) ? dht1.readTemperature() : dht2.readTemperature();
    sample[i] = String(sample_float[i], 2);

    sample_float[i + 2] = (i == 0) ? dht1.readHumidity() : dht2.readHumidity();
    sample[i + 2] = String((int)sample_float[i + 2]);
  }
}


//bi
// void control_temp_using_thres() {
//   firing_delay = (sample_float[0] > (temp_thres * sample_float[4])) ? firing_thres : 0;

//   sample_float[10] = firing_delay / 1000.0;
//   sample[10] = String(sample_float[10], 2);
//   sample_float[11] = firing_delay * 180.0 / 10000.0;
//   sample[11] = String(sample_float[11], 2);
// }




//dung kp
void adjustFiringDelay() {
  unsigned long current_millis_p = millis();
  if (current_millis_p - previous_millis_p >= sampling_time_p) {
    firing_delay += kp * (sample_float[4] - sample_float[0]);
    firing_delay = constrain(firing_delay, 0, firing_delay_max);
    previous_millis_p = current_millis_p;
  }

  sample_float[10] = firing_delay / 1000.0;
  sample[10] = String(sample_float[10], 2);
  sample_float[11] = firing_delay * 180.0 / 10000.0;
  sample[11] = String(sample_float[11], 2);
}



void printToLCD() {
  static unsigned long lastUpdate = 0;
  if (millis() - lastUpdate > 1000) {  // Chỉ cập nhật mỗi giây
    lastUpdate = millis();
    lcd.clear();
    if (mode == 1) {
      lcd.setCursor(0, 0);
      lcd.print("T1: ");
      lcd.print(String(sample_float[0], 2));
      lcd.setCursor(11, 0);
      lcd.print("T2: ");
      lcd.print(String(sample_float[1], 2));

      lcd.setCursor(0, 1);
      lcd.print("H1: ");
      lcd.print(String(sample_float[2], 0));
      lcd.setCursor(11, 1);
      lcd.print("H2: ");
      lcd.print(String(sample_float[3], 0));

      lcd.setCursor(0, 2);
      lcd.print("TIME: ");
      lcd.print(hours < 10 ? "0" : "");
      lcd.print(hours);
      lcd.print(":");
      lcd.print(minutes < 10 ? "0" : "");
      lcd.print(minutes);
      lcd.print(":");
      lcd.print(seconds < 10 ? "0" : "");
      lcd.print(seconds);

      lcd.setCursor(0, 3);
      lcd.print("SECDAO: ");
      lcd.print(timeoff);
      lcd.setCursor(11, 3);
      lcd.print("RUN: ");
      lcd.print(RUN);

    } else if (mode == 2) {
      lcd.setCursor(0, 0);
      lcd.print("TRANG THAI HOAT DONG");
      lcd.setCursor(0, 1);
      lcd.print("FAN220V:");
      lcd.print(state[5] == 1 ? "ON" : "OFF");

      lcd.setCursor(11, 1);
      lcd.print("HUMI:");
      lcd.print(state[1] == 1 ? "ON" : "OFF");

      lcd.setCursor(0, 2);
      lcd.print("DCDAO:");
      lcd.print(state[3] == 1 ? "ON" : "OFF");

      lcd.setCursor(11, 2);
      lcd.print("DENUV:");
      lcd.print(state[4] == 1 ? "ON" : "OFF");

      lcd.setCursor(0, 3);
      lcd.print("FAN12V:");
      lcd.print(state[0] == 1 ? "ON" : "OFF");

      lcd.setCursor(11, 3);
      lcd.print("PUMP:");
      lcd.print(state[2] == 1 ? "ON" : "OFF");
    } else if (mode == 3) {
      lcd.setCursor(0, 0);
      lcd.print("CAC GIA TRI CAI DAT");

      lcd.setCursor(0, 1);
      lcd.print("TEMP: ");
      lcd.print(String(sample_float[4], 2));

      lcd.setCursor(12, 1);
      lcd.print("HUMI: ");
      lcd.print(String(sample_float[5], 2));

      lcd.setCursor(0, 2);
      lcd.print("TIME: ");
      lcd.print(hours < 10 ? "0" : "");
      lcd.print(hours);
      lcd.print(":");
      lcd.print(minutes < 10 ? "0" : "");
      lcd.print(minutes);
      lcd.print(":");
      lcd.print(seconds < 10 ? "0" : "");
      lcd.print(seconds);

      lcd.setCursor(0, 3);
      lcd.print("SECDAO: ");
      lcd.print(timeoff);
    } else if (mode == 4) {
      lcd.setCursor(3, 0);
      lcd.print("CAI DAT GIA TRI");
      if (cai_dat == 1) {
        lcd.setCursor(2, 1);
        lcd.print("CAI DAT NHIET DO");
        lcd.setCursor(3, 2);
        lcd.print("TEMP: ");
        lcd.print(String(sample_float[4], 2));
        lcd.write(0xdf);  // Ký hiệu độ
        lcd.print("C");

        lcd.setCursor(3, 3);
        lcd.print("GIA TRI: ");
        lcd.print(GT_CD);
      } else if (cai_dat == 2) {
        lcd.setCursor(4, 1);
        lcd.print("CAI DAT DO AM");

        lcd.setCursor(5, 2);
        lcd.print("HUMI: ");
        lcd.print(String(sample_float[5], 0));
        lcd.print("%");
      } else if (cai_dat == 3) {
        lcd.setCursor(3, 1);
        lcd.print("CAI DAT GIO DAO");

        lcd.setCursor(5, 2);
        lcd.print("HOUR: ");
        lcd.print(String(sample_float[6], 0));
      } else if (cai_dat == 4) {
        lcd.setCursor(2, 1);
        lcd.print("CAI DAT PHUT DAO");

        lcd.setCursor(4, 2);
        lcd.print("MINUTE: ");
        lcd.print(String(sample_float[7], 0));
      } else if (cai_dat == 5) {
        lcd.setCursor(1, 1);
        lcd.print("THOI GIAN DAO TRUNG");

        lcd.setCursor(6, 2);
        lcd.print("TIMEDAO: ");
        lcd.print(String(sample_float[9], 0));
      }
    }
  }
}


void turn_off_outputs() {
  Serial.println("Turning off all outputs");
  firing_delay = firing_delay_max;
  sample_float[10] = firing_delay / 1000.0;
  sample[10] = String(sample_float[10], 2);
  sample_float[11] = firing_delay * 180.0 / 10000.0;
  sample[11] = String(sample_float[11], 2);

  digitalWrite(Motor220V, LOW);
  digitalWrite(triac_pin, LOW);
  digitalWrite(PUMP_PIN, LOW);
  digitalWrite(FAN_PIN, LOW);
  digitalWrite(HUMI_PIN, LOW);
  digitalWrite(FAN220V, LOW);
  digitalWrite(LIGHT_UV, LOW);
  digitalWrite(In_AC, LOW);
}






void counttime() {
  unsigned long currentMillis = millis();  // Lấy thời gian hiện tại từ millis()

  if (currentMillis - previousMillis_count >= interval) {
    previousMillis_count = currentMillis;

    if (hours == 0 && minutes == 0 && seconds == 0 && state[3] == 0) {
      hours = sample_float[6];
      minutes = sample_float[7];
      seconds = sample_float[8];
      timeoff = sample_float[9];
    }

    if (hours > 0 || minutes > 0 || seconds > 0) {
      seconds--;
      if (seconds < 0) {
        seconds = 59;
        minutes--;
        if (minutes < 0) {
          minutes = 59;
          hours--;
          if (hours < 0) {
            hours = 0;
            minutes = 0;
            seconds = 0;
          }
        }
      }
    }

    if (hours == 0 && minutes == 0 && seconds == 0) {
      if (state[3] == 0) {
        state[3] = 1;
        startMillis = millis();
        Serial.println("Motor started");
      }
    }

    if (state[3] == 1) {
      if (--timeoff <= 0) {
        state[3] = 0;  // Tắt motor
        Serial.println("Motor stopped");

        // Reset lại các giá trị giờ, phút, giây từ `sample_float`
        hours = sample_float[6];
        minutes = sample_float[7];
        seconds = sample_float[8];
        timeoff = sample_float[9];
      }
    }
  }
}




void controlFan12V() {
  if (state[0] == 1) {
    digitalWrite(FAN_PIN, HIGH);
  } else if (sample_float[0] > sample_float[4] && state[0] == 0) {
    digitalWrite(FAN_PIN, HIGH);
  } else {
    digitalWrite(FAN_PIN, LOW);
  }
}


void controlHumi() {
  if (state[1] == 1) {
    digitalWrite(HUMI_PIN, HIGH);
  } else if (sample_float[2] < sample_float[5] && state[1] == 0) {
    digitalWrite(HUMI_PIN, HIGH);
  } else {
    digitalWrite(HUMI_PIN, LOW);
  }
}


// void controlPump() {
//   if (state[2] == 1) {
//     digitalWrite(PUMP_PIN, HIGH);
//   } else if (sample_float[2] < sample_float[5]) {
//     digitalWrite(PUMP_PIN, HIGH);
//     previousMillis_pump = millis();
//    }
//   if (state[2] == 0 &&  millis() - previousMillis_pump >= 10000)
//     digitalWrite(PUMP_PIN, LOW);
// }




void controlPump() {
  if (state[2] == 1) {
    digitalWrite(PUMP_PIN, HIGH);
  } else {
    digitalWrite(PUMP_PIN, LOW);
  }
}






void controlMotor220() {
  if (state[3] == 1)
    digitalWrite(Motor220V, HIGH);
  else digitalWrite(Motor220V, LOW);
}


void controlUVLight() {
  if (state[4] == 1) {
    digitalWrite(LIGHT_UV, HIGH);
  } else if ((millis() - previousMillis_UV >= 24 * 60 * 60 * 1000) && state[4] == 0) {
    digitalWrite(LIGHT_UV, HIGH);
    previousMillis_UV = millis();
  } else digitalWrite(LIGHT_UV, LOW);

  if (millis() - previousMillis_UV >= 60000)
    digitalWrite(LIGHT_UV, LOW);
}




void controlFan220V() {
  if (state[5] == 1 || SYSTEM == 1 || online_mode == 0) {
    digitalWrite(FAN220V, HIGH); // Bật quạt
  } else if (state[5] == 0 || SYSTEM == 0) {
    digitalWrite(FAN220V, LOW);  // Tắt quạt
  }
}




void sendggsheet() {
  unsigned long current_millis_sheet = millis();
  if (current_millis_sheet - previous_millis_sheet >= time_sheet) {
    if (WiFi.status() == WL_CONNECTED) {

      String Send_Data_URL = Web_App_URL + "?sts=write";
      Send_Data_URL += "&temp1=" + String(sample[0]);
      Send_Data_URL += "&temp2=" + String(sample[1]);
      Send_Data_URL += "&humd1=" + String(sample[2]);
      Send_Data_URL += "&humd2=" + String(sample[3]);
      Send_Data_URL += "&tempset=" + String(sample[4]);
      Send_Data_URL += "&humdset=" + String(sample[5]);
      Send_Data_URL += "&firingdelay=" + String(sample[10]);
      Send_Data_URL += "&angle=" + String(sample[11]);
      Serial.println(Send_Data_URL);


      HTTPClient http;
      http.begin(Send_Data_URL.c_str());
      http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);

      int httpCode = http.GET();
      String payload;
      if (httpCode > 0) {
        payload = http.getString();
      }
      http.end();
    }
    previous_millis_sheet = current_millis_sheet;
  }
}



void simulate_sensors() {
  unsigned long current_millis_simu = millis();

  if (current_millis_simu - previous_millis_simu >= simu_update_sensors_time) {
    if (firing_delay == 0)
      temp_simu += 0.09667 + (random(10) / 100000.0);
    else if (firing_delay == firing_delay_max)
      temp_simu -= 0.01667 - (random(10) / 100000.0);
    else
      temp_simu += -0.00346 * log(firing_delay) + 0.09667 + (random(10) / 1000.0);

    if ((temp_simu >= 0.95 * sample_float[2]) && (temp_simu <= 1.05 * sample_float[2]))
      humi_simu += -0.01 + (0.01 + 0.01) * (random(1000) / 1000.0);
    else {
      humi_simu += 0.02 - (firing_delay_max - firing_delay) * (0.02 + 0.09) / firing_delay_max;
      humi_simu += -0.01 + (0.01 + 0.01) * (random(1000) / 1000.0);
    }
    humi_simu = constrain(humi_simu, 0.0, 100.0);

    sample_float[0] = temp_simu;
    sample_float[1] = temp_simu - 5;
    sample_float[2] = humi_simu;
    sample_float[3] = humi_simu + 10;

    previous_millis_simu = current_millis_simu;
  }
}


///////////////////////////////////BUTTON/////////////////////////////////////////////////////
void mode_button() {
  unsigned long currentMillis = millis();
  if (currentMillis - lastModeChange > 200) {
    digitalWrite(Buzz, HIGH);
    wait_function(50);
    digitalWrite(Buzz, LOW);
    if (mode != 4) {
      mode++;
      if (mode > 4) {
        mode = 1;
      }
    } else {
      if (mode == 4 && digitalRead(NN1) == LOW) {
        mode = 1;
      }
      // cai_dat++;
      // if (cai_dat > 5) {
      //   cai_dat = 1;
    }
  }
  lastModeChange = currentMillis;
}


void B_up() {
  if (digitalRead(NN2) == LOW) {
    wait_function(10);
    if (digitalRead(NN2) == LOW) {
      digitalWrite(Buzz, HIGH);
      wait_function(50);
      digitalWrite(Buzz, LOW);
      if (cai_dat == 1) {
        sample_float[4] += GT_CD;
        if (sample_float[4] > 100.00) sample_float[4] = 0;
      } else if (cai_dat == 2) {
        sample_float[5] += 1;
        if (sample_float[5] > 100) sample_float[5] = 0;
      } else if (cai_dat == 3) {
        sample_float[6] += 1;
        if (sample_float[6] > 23) sample_float[6] = 0;
      } else if (cai_dat == 4) {
        sample_float[7] += 1;
        if (sample_float[7] > 59) sample_float[7] = 0;
      } else if (cai_dat == 5) {
        sample_float[9] += 1;
        if (sample_float[9] > 59) sample_float[9] = 0;
      }
    }
    while (digitalRead(NN2) == LOW)
      ;
  }
}


void B_down() {
  if (digitalRead(NN4) == LOW) {
    wait_function(10);
    if (digitalRead(NN4) == LOW) {
      digitalWrite(Buzz, HIGH);
      wait_function(50);
      digitalWrite(Buzz, LOW);
      if (cai_dat == 1) {
        sample_float[4] -= GT_CD;
        if (sample_float[4] < 0) sample_float[4] = 100.00;
      } else if (cai_dat == 2) {
        sample_float[5] -= 1;
        if (sample_float[5] < 0) sample_float[5] = 100;
      } else if (cai_dat == 3) {
        sample_float[6] -= 1;
        if (sample_float[6] < 0) sample_float[6] = 23;
      } else if (cai_dat == 4) {
        sample_float[7] -= 1;
        if (sample_float[7] < 0) sample_float[7] = 59;
      } else if (cai_dat == 5) {
        sample_float[9] -= 1;
        if (sample_float[9] < 0) sample_float[9] = 59;
      }
    }
    while (digitalRead(NN4) == LOW)
      ;
  }
}


void B_dao() {
  if (digitalRead(NN3) == LOW) {
    wait_function(10);
    if (digitalRead(NN3) == LOW) {
      digitalWrite(Buzz, HIGH);
      wait_function(50);
      digitalWrite(Buzz, LOW);
      if (mode == 4) {
        //cai_dat = 1;
        cai_dat++;

        if (cai_dat > 5) {
          cai_dat = 1;
        } else {
          state[6] = 1;
          state[3] = 1;
          startMillis = millis();
          //motor_on();
        }
        while (digitalRead(NN3) == LOW)
          ;
      }
    }
  }
}


void sendBlynk() {
  if (!Blynk.connected()) {
    Blynk.connect();
  }
  Blynk.run();
  Blynk.syncAll();
  for (int i = 0; i < 12; i++) {
    if (i <= 4 || i == 10) {
      Blynk.virtualWrite(i, sample_float[i]);
    } else {
      Blynk.virtualWrite(i, (int)sample_float[i]);
    }
  }
  for (int i = 0; i < 10; i++) {
    Blynk.virtualWrite(V70 + i, state[i]);
  }
  remaining_time = runtime + previous_millis_timer - millis();
  Blynk.virtualWrite(45, remaining_time >= 0 ? remaining_time / 60000 : 0);


  Blynk.virtualWrite(23, kp);
  Blynk.virtualWrite(25, int(sampling_time_p / 1000));
  Blynk.virtualWrite(27, int(time_sheet / 1000));
  Blynk.virtualWrite(28, cai_dat);
  Blynk.virtualWrite(30, hours);
  Blynk.virtualWrite(31, minutes);
  Blynk.virtualWrite(32, timeoff);
  int runtime_hours = runtime / 3600000;
  Blynk.virtualWrite(47, String(runtime_hours));
  Blynk.virtualWrite(49, RUN);
  Blynk.virtualWrite(51, SYSTEM);


  Serial.print("Remaining Time: ");
  Serial.print(remaining_time);
  Serial.print(" ms, Runtime: ");
  Serial.println(runtime);



  // Trạng thái LED cho từng chức năng dựa trên `state
  led1.setValue(digitalRead(FAN_PIN) == 1 ? 255 : 0);    // Fan
  led2.setValue(digitalRead(HUMI_PIN) == 1 ? 255 : 0);   // Humi
  led3.setValue(digitalRead(PUMP_PIN) == 1 ? 255 : 0);   // Pump
  led4.setValue(digitalRead(Motor220V) == 1 ? 255 : 0);  // DcDao
  led5.setValue(digitalRead(LIGHT_UV) == 1 ? 255 : 0);   // UV
  led6.setValue(digitalRead(FAN220V) == 1 ? 255 : 0);    // Fan220 na

  // Trạng thái LED cho các chế độ `cai_dat`
  led7.setValue(cai_dat == 1 ? 255 : 0);
  led8.setValue(cai_dat == 2 ? 255 : 0);
  led9.setValue(cai_dat == 3 ? 255 : 0);
  led10.setValue(cai_dat == 4 ? 255 : 0);
  led11.setValue(cai_dat == 5 ? 255 : 0);
  online_state_led.setValue(online_mode == 1 ? 255 : 0);
}


void connecttowifi() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Attempting to connect to WiFi...");

    WiFi.begin(wifi_network, wifi_password);
    unsigned long start_time = millis();
    while (WiFi.status() != WL_CONNECTED && (millis() - start_time < 5000)) {
      Serial.print(".");
      wait_function(500);
    }
    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("\nSuccessfully connected to: " + String(wifi_network));
      Serial.println("IP address: " + String(WiFi.localIP()));
    } else {
      Serial.println("\nFailed to connect to WiFi");
    }
  } else {
    Serial.println("Already connected to WiFi");
  }
}


void wifi_blynk_task_code(void* pvParameters) {
  for (;;) {
    connecttowifi();
    sendBlynk();
  }
}


void triggerTriac() {
  digitalWrite(triac_pin, HIGH);
  delayMicroseconds(8.333);
  digitalWrite(triac_pin, LOW);
}


void IRAM_ATTR zeroCrossISR() {

  if (RUN == 1) {
    zeroCrossed = true;
    triacTicker.once_us(firing_delay, triggerTriac);
  } else {
  }
}





////////////////////////////////////////////BLYNK//////////////////////////////////////////////////
// controlFan12V
BLYNK_WRITE(V35) {
  state[0] = param.asInt();
}


// controlHumi
BLYNK_WRITE(V36) {
  state[1] = param.asInt();
}


// controlPump
BLYNK_WRITE(V37) {
  state[2] = param.asInt();
}


// controlMotor220
BLYNK_WRITE(V38) {
  state[3] = param.asInt();
}


// controlUVLight
BLYNK_WRITE(V39) {
  state[4] = param.asInt();
}


// controlFan220V
BLYNK_WRITE(V40) {
  state[5] =  param.asInt();
}


BLYNK_WRITE(V41) {
  water = param.asInt();
  Serial.println(water);
}


BLYNK_WRITE(V29) {
  int value = param.asInt();
  if (value == 1) {
    cai_dat += 1;
    if (cai_dat > 5) {
      cai_dat = 1;
      hours = sample_float[6];
      minutes = sample_float[7];
      seconds = sample_float[8];
      timeoff = sample_float[9];
    }
  }
}



BLYNK_WRITE(V33) {
  int value = param.asInt();
  if (online_mode == 1) {
    if (value == 1) {
      if (cai_dat == 1) {
        sample_float[4] += GT_CD;
        if (sample_float[4] > 100.00) sample_float[4] = 0;
      } else if (cai_dat == 2) {
        sample_float[5] += 1;
        if (sample_float[5] > 100) sample_float[5] = 0;
      } else if (cai_dat == 3) {
        sample_float[6] += 1;
        if (sample_float[6] > 23) sample_float[6] = 0;
      } else if (cai_dat == 4) {
        sample_float[7] += 1;
        if (sample_float[7] > 59) sample_float[7] = 0;
      } else if (cai_dat == 5) {
        sample_float[9] += 1;
        if (sample_float[9] > 59) sample_float[9] = 0;
      }

      sendBlynk();
    }
  }
}



BLYNK_WRITE(V34) {
  int value = param.asInt();
  if (online_mode == 1) {
    if (value == 1) {
      if (cai_dat == 1) {
        sample_float[4] -= GT_CD;
        if (sample_float[4] > 100.00) sample_float[4] = 0;
      } else if (cai_dat == 2) {
        sample_float[5] -= 1;
        if (sample_float[5] > 100) sample_float[5] = 0;
      } else if (cai_dat == 3) {
        sample_float[6] -= 1;
        if (sample_float[6] > 23) sample_float[6] = 0;
      } else if (cai_dat == 4) {
        sample_float[7] -= 1;
        if (sample_float[7] > 59) sample_float[7] = 0;
      } else if (cai_dat == 5) {
        sample_float[9] -= 1;
        if (sample_float[9] > 59) sample_float[9] = 0;
      }

      sendBlynk();
    }
  }
}



BLYNK_WRITE(V42) {
  online_mode = param.asInt();
  if (online_mode == 0) {
    sample_float[4] = tempset_off;
    sample[4] = String(sample_float[4], 2);  //tempset
    sample_float[5] = humiset_off;
    sample[5] = String(sample_float[5], 2);  //humiset
    sample_float[6] = hourset_off;
    sample[6] = String(sample_float[6], 0);  //gio
    sample_float[7] = minuset_off;
    sample[7] = String(sample_float[7], 0);  //phut
    sample_float[8] = secset_off;
    sample[8] = String(sample_float[8], 0);  //sec
    sample_float[9] = timeset_off;
    sample[9] = String(sample_float[9], 0);  //time đảo

    runtime = runtime_offline;
    previous_millis_timer = millis();

    kp = kp_fit;
    time_sheet = Gui_GG;
    sampling_time_p = sampling_time_p_set;
  }

  Serial.println("Online mode: " + String(online_mode));
  wait_function(2500);
}


BLYNK_WRITE(V44) {
  runtime = (online_mode == 1) ? param.asInt() * 24 * 60 * 60 * 1000 : runtime;
  previous_millis_timer = millis();
}


BLYNK_WRITE(V24) {
  kp = (online_mode == 1) ? -param.asInt() : kp;  // Chuyển số dương từ app thành số âm
}



// Read sampling_time_sheet from blynk
BLYNK_WRITE(V46) {
  time_sheet = (online_mode == 1) ? param.asInt() * 1000 : time_sheet;
}


// Read sampling_time_p from blynk
BLYNK_WRITE(V26) {
  sampling_time_p = (online_mode == 1) ? param.asInt() * 1000 : sampling_time_p;
}



BLYNK_WRITE(V48) {
  RUN = param.asInt();
}


BLYNK_WRITE(V50) {
  SYSTEM = param.asInt();
  if (SYSTEM == 0) {
    Serial.println("Turning off all outputs");
    sample_float[10] = firing_delay / 1000.0;
    sample[10] = String(sample_float[10], 2);
    sample_float[11] = firing_delay * 180.0 / 10000.0;
    sample[11] = String(sample_float[11], 2);
    digitalWrite(Motor220V, LOW);
    digitalWrite(PUMP_PIN, LOW);
    digitalWrite(FAN_PIN, LOW);
    digitalWrite(HUMI_PIN, LOW);
    digitalWrite(FAN220V, LOW);
    digitalWrite(LIGHT_UV, LOW);
    RUN = 0;
  } else {
  }
}



////////////////////////////////////////////////////////////////////////////////////////////
void setup() {
  Serial.begin(115200);
  lcd.init();
  lcd.backlight();
  lcd.clear();
  dht1.begin();
  dht2.begin();

  connecttowifi();  // Kết nối WiFi
  Blynk.config(blynk_token, blynk_net, blynk_port);
  Blynk.connect();
  xTaskCreatePinnedToCore(wifi_blynk_task_code, "wifi_blynk_task", 10000, NULL, 1, &wifi_blynk_task, 1);


  pinMode(triac_pin, OUTPUT);
  pinMode(In_AC, INPUT);
  pinMode(NN1, INPUT_PULLUP);
  pinMode(NN2, INPUT_PULLUP);
  pinMode(NN3, INPUT_PULLUP);
  pinMode(NN4, INPUT_PULLUP);
  pinMode(Motor220V, OUTPUT);
  pinMode(Buzz, OUTPUT);
  pinMode(LIGHT_UV, OUTPUT);
  pinMode(PUMP_PIN, OUTPUT);
  pinMode(FAN_PIN, OUTPUT);
  pinMode(HUMI_PIN, OUTPUT);
  pinMode(FAN220V, OUTPUT);

  digitalWrite(LIGHT_UV, LOW);
  digitalWrite(Motor220V, LOW);
  digitalWrite(triac_pin, LOW);
  digitalWrite(PUMP_PIN, LOW);
  digitalWrite(FAN_PIN, LOW);
  digitalWrite(HUMI_PIN, LOW);
  digitalWrite(FAN220V, LOW);
  digitalWrite(Buzz, LOW);
  wait_function(2000);

  sample_float[4] = tempset_off;
  sample[4] = String(sample_float[4]);  //tempset
  sample_float[5] = humiset_off;
  sample[5] = String(sample_float[5]);  //humiset
  sample_float[6] = hourset_off;
  sample[6] = String(sample_float[6], 0);  //gio
  sample_float[7] = minuset_off;
  sample[7] = String(sample_float[7], 0);  //phut
  sample_float[8] = secset_off;
  sample[8] = String(sample_float[8], 0);  //sec
  sample_float[9] = timeset_off;
  sample[9] = String(sample_float[9], 0);  //time đảo
  GT_CD = 0.5f;
  hours = sample_float[6];
  minutes = sample_float[7];
  seconds = sample_float[8];
  timeoff = sample_float[9];
  runtime = runtime_offline;


  attachInterrupt(digitalPinToInterrupt(In_AC), zeroCrossISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(NN1), mode_button, FALLING);
}

//////////////////////////////////////////////////////////////////////////////////////////////////
void loop() {
  read_sensor();
  //simulate_sensors();
  B_up();
  B_down();
  B_dao();
  counttime();
  printToLCD();


  if (SYSTEM == 1) {
    adjustFiringDelay();
    controlFan220V();
    controlMotor220();
    controlFan12V();
    controlHumi();
    controlPump();
    controlUVLight();
    sendggsheet();
    Serial.println(mode);
    Serial.println("CD" + String(cai_dat));

  } else {
    turn_off_outputs();
    Serial.println("System stopped\n");
  }
}
