#include <driver/i2s.h>

#define I2S_WS   7   // LRCLK
#define I2S_SCK  6   // BCLK
#define I2S_SD   5   // DOUT dari mic
#define I2S_PORT I2S_NUM_0

bool isRecording = false;

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Ketik 'r' untuk mulai rekam, 's' untuk stop.");

  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = 16000,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 4,
    .dma_buf_len = 512,
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0
  };

  i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_SCK,
    .ws_io_num = I2S_WS,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num = I2S_SD
  };

  i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_PORT, &pin_config);
  i2s_zero_dma_buffer(I2S_PORT);
}

void loop() {
  // Cek perintah dari Serial
  if (Serial.available()) {
    char cmd = Serial.read();
    if (cmd == 'r') {
      isRecording = true;
      Serial.println("Mulai merekam...");
    } else if (cmd == 's') {
      isRecording = false;
      Serial.println("Rekam berhenti.");
    }
  }

  if (isRecording) {
    int32_t samples[256];
    size_t bytesRead;
    i2s_read(I2S_PORT, (void*)samples, sizeof(samples), &bytesRead, portMAX_DELAY);
    int sampleCount = bytesRead / 4;

    for (int i = 0; i < sampleCount; i++) {
      int16_t s = samples[i] >> 14; // Konversi ke 16-bit
      Serial.write((uint8_t*)&s, 2); // Kirim biner ke PC
    }
  }
}
