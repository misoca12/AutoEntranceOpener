#include <M5StickCPlus.h>
#include <driver/i2s.h>
#include <arduinoFFT.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <string>

#include "WifiSetting.h"
#include "SwitchBotApiConfig.h"

#define PIN_CLK     0
#define PIN_DATA    34
#define READ_LEN    (2 * 256)
#define GAIN_FACTOR 3

#define SAMPLES 500
#define SAMPLING_FREQUENCY 40000

#define DETECT_LOWER_LIMIT 1000
#define DETECT_UPPER_LIMIT 5000
#define DETECT_COUNT_THRESHOLD 5

uint8_t BUFFER[READ_LEN] = {0};

uint16_t oldy[240];
int16_t *adcBuffer = NULL;

unsigned int sampling_period_us;

const uint16_t FFTsamples = 256;
double vReal[FFTsamples];
double vImag[FFTsamples];

bool detectBuffer[10];

// band[4] : 625.00Hz
// band[5] : 781.25Hz
int targetBand[2] = { 4, 5 };

using namespace std; 
WiFiClass wifi; 
HTTPClient http;

arduinoFFT FFT = arduinoFFT(vReal, vImag, FFTsamples, SAMPLING_FREQUENCY);

void setup() {

    // M5StickC Plusを初期化
    M5.begin();
  
    M5.Lcd.setRotation(3);
    M5.Lcd.setTextColor(BLACK, WHITE);

    // WiFi接続
    initWiFi();
    
    //マイク初期化 
    i2sInit();

    // 待ち受け開始
    xTaskCreate(mic_record_task, "mic_record_task", 4096, NULL, 1, NULL);
}

void loop() {
    M5.update();
    if (M5.BtnA.wasReleased()) {
        requestSwitchBotApi();
    } else if (M5.BtnA.wasReleasefor(2000)) { 
        // 2秒長押しでリセット
        esp_restart();
    }
}

void initWiFi() {
    M5.Lcd.fillScreen(WHITE);
    M5.Lcd.setCursor(0, 0);
    M5.Lcd.setTextSize(2);
    M5.Lcd.print("Connecting Wifi");
    wifi.begin(WifiSetting::WIFI_AP_NAME, WifiSetting::WIFI_AP_PASSWORD); 
    while (wifi.status() != WL_CONNECTED) {
      delay(500);
      M5.Lcd.print(".");
    }
    M5.Lcd.println("\nWiFi Connected!");
    
    delay(1000);
    M5.Lcd.fillScreen(WHITE);
}

void requestSwitchBotApi() {

    M5.Lcd.fillScreen(WHITE);
    M5.Lcd.setCursor(20, 45);
    M5.Lcd.setTextSize(10);
    M5.Lcd.print("OPEN!");

    String url = "https://api.switch-bot.com/v1.0/devices/";
    url += SwitchBotApiConfig::BOT_DEVICE_ID;
    url += "/commands";

    Serial.printf("Request url:%s\n\n", url.c_str());

    http.begin(url);
    http.addHeader("Authorization", SwitchBotApiConfig::TOKEN);
    http.addHeader("Content-Type", "application/json; charset=utf8");

    int httpCode = http.POST("{\"command\": \"press\"}"); // Bot押下コマンド
    if (httpCode > 0) {
        Serial.printf("Result code: %d\n", httpCode);
        if (httpCode == HTTP_CODE_OK) {
            String payload = http.getString();
            Serial.println(payload);
            Serial.println("Success");
        }
    } else {
        Serial.print("Failure, error: ");
        Serial.println(http.errorToString(httpCode).c_str());
    }

    http.end();  
}

void i2sInit() {
    i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_PDM),
        .sample_rate = 44100,
        .bits_per_sample =
            I2S_BITS_PER_SAMPLE_16BIT,  // is fixed at 12bit, stereo, MSB
        .channel_format = I2S_CHANNEL_FMT_ALL_RIGHT,
#if ESP_IDF_VERSION > ESP_IDF_VERSION_VAL(4, 1, 0)
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
#else
        .communication_format = I2S_COMM_FORMAT_I2S,
#endif
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count    = 2,
        .dma_buf_len      = 128,
    };

    i2s_pin_config_t pin_config;

#if (ESP_IDF_VERSION > ESP_IDF_VERSION_VAL(4, 3, 0))
    pin_config.mck_io_num = I2S_PIN_NO_CHANGE;
#endif

    pin_config.bck_io_num   = I2S_PIN_NO_CHANGE;
    pin_config.ws_io_num    = PIN_CLK;
    pin_config.data_out_num = I2S_PIN_NO_CHANGE;
    pin_config.data_in_num  = PIN_DATA;

    i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
    i2s_set_pin(I2S_NUM_0, &pin_config);
    i2s_set_clk(I2S_NUM_0, 44100, I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_MONO);
}

void mic_record_task(void *arg) {
    size_t bytesread;
    while (1) {
        i2s_read(I2S_NUM_0, (char *)BUFFER, READ_LEN, &bytesread,
                 (100 / portTICK_RATE_MS));
        adcBuffer = (int16_t *)BUFFER;
        showSignal();
        fft();
        callApiIfNeeded();      
        vTaskDelay(100 / portTICK_RATE_MS);
    }
}

void showSignal() { 
  int32_t offset_sum = 0;
  for (int n = 0; n < 240; n++) {
    offset_sum += (int16_t)adcBuffer[n];
  }
  int offset_val = -( offset_sum / 240 );
  // Auto Gain
  int max_val = 200;
  for (int n = 0; n < 240; n++) {
    int16_t val = (int16_t)adcBuffer[n] + offset_val;
    if ( max_val < abs(val) ) {
      max_val = abs(val);
    }
  }
  
  int y;
  for (int n = 0; n < 240; n++){
    y = adcBuffer[n] + offset_val;
    y = map(y, -max_val, max_val, 10, 125);
    M5.Lcd.drawPixel(n, oldy[n],WHITE);
    M5.Lcd.drawPixel(n,y,BLACK);
    oldy[n] = y;
  }
}

void fft(){
    for (int i = 0; i < FFTsamples; i++) {
      unsigned long t = micros();
      vReal[i] = adcBuffer[i];
      vImag[i] = 0;
      while ((micros() - t) < sampling_period_us) ;
    }
  
    // 高速フーリエ変換     
    FFT.Windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
    FFT.Compute(FFT_FORWARD);
    FFT.ComplexToMagnitude();
}

void callApiIfNeeded() {
    bool overTreshold = false;
    int nsamples = FFTsamples/2;
    for (int band = 0; band < nsamples; band++) {    
        for (int i = 0; i < sizeof(targetBand)/sizeof(*targetBand); i++) {
            if (band != targetBand[i]) continue;
            float d = vReal[band];

            Serial.print(band);
            Serial.print(" : ");
            Serial.print((band * 1.0 * SAMPLING_FREQUENCY) / FFTsamples);
            Serial.print("Hz : ");
            Serial.println(d);

            if (DETECT_LOWER_LIMIT <= d && DETECT_UPPER_LIMIT > d) {
                overTreshold = true;
                break;
            }
        }

    }  
        
    int detectCount = 0;
    for(int i = 0; i < sizeof(detectBuffer)/sizeof(*detectBuffer); i++) {
        if (i < sizeof(detectBuffer)/sizeof(*detectBuffer) - 1) {
            detectBuffer[i] = detectBuffer[i + 1];
        } else {
            detectBuffer[i] = overTreshold;  
        }          
        if (detectBuffer[i]) {
          detectCount++;
        }
    }
    
    Serial.print("detectCount : ");
    Serial.println(detectCount);

    if (detectCount >= DETECT_COUNT_THRESHOLD) {
        // 閾値を超えたらチャイムとして判定
        Serial.println("detect!");
        
        // 解錠
        requestSwitchBotApi();

        vTaskDelay(2000 / portTICK_RATE_MS);

        memset(detectBuffer, false, sizeof(detectBuffer));
        M5.Lcd.fillScreen(WHITE);
    }    
}
