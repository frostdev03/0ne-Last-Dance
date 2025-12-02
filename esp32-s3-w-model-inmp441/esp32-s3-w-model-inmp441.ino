#include <Klasifikasi_Audio_-_TA_inferencing.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2s.h"
#include "esp_heap_caps.h"
#include <math.h>               // for PI, sinf

// ---------------- LED CONFIG ----------------
#define USE_NEOPIXEL 1         // 1 = WS2812/NeoPixel; 0 = separate R/G/B PWM

// --- PIN CONFIG (adjust to your board) ---
#define RGB_PIN        4       // NeoPixel data pin (change if needed)
#define LED_BIG_PIN    2       // big single LED pin (change if needed)

// If not using NeoPixel, set PWM pins for R/G/B:
#define RGB_R_PIN      15
#define RGB_G_PIN      14
#define RGB_B_PIN      13

#if USE_NEOPIXEL
  #include <Adafruit_NeoPixel.h>
  #define NPIXELS 1
  Adafruit_NeoPixel pixels(NPIXELS, RGB_PIN, NEO_GRB + NEO_KHZ800);
#else
  // PWM channels for separate RGB pins
  const int PWM_FREQ = 5000;
  const int PWM_RES = 8; // 8-bit
  const int CH_R = 0;
  const int CH_G = 1;
  const int CH_B = 2;
  const int CH_BIG = 3;
#endif

// brightness default
#if USE_NEOPIXEL
  #define NP_BRIGHTNESS 80   // lower to reduce current draw
#endif

// ... (rest of your audio / inference code unchanged) ...
// make sure the led helper functions below are present (they were in your code)


// Animation state (non-blocking)
static uint32_t anim_start_ms = 0;
static int current_class = -1;
static uint32_t last_anim_tick = 0;

// Define 5 colors (R,G,B) for classes 0..4
const uint8_t class_colors[5][3] = {
  { 255, 0, 0 },    // class 0: red
  { 0, 200, 0 },    // class 1: green
  { 0, 0, 255 },    // class 2: blue
  { 255, 200, 0 },  // class 3: amber/yellow
  { 180, 0, 180 }   // class 4: magenta
};

// Each class has unique animation ID:
// 0: steady
// 1: slow breathe
// 2: double blink (two short pulses + pause)
// 3: fast blink
// 4: alternated pulse (big LED toggles opposite RGB)
const uint8_t class_anim_type[5] = { 0, 1, 2, 3, 4 };

/** Audio buffers, pointers and selectors */
typedef struct {
  signed short *buffers[2];
  unsigned char buf_select;
  unsigned char buf_ready;
  unsigned int buf_count;
  unsigned int n_samples;
} inference_t;

static inference_t inference;
static const uint32_t sample_buffer_size = 4096;  // safety margin
static signed short sampleBuffer[sample_buffer_size];
static bool debug_nn = false;  // Set true to see more debug
static int print_results = -(EI_CLASSIFIER_SLICES_PER_MODEL_WINDOW);
static bool record_status = true;

// I2S port and pins - adjust if your board uses different pins
#define I2S_PORT I2S_NUM_1
#define I2S_BCK_PIN 17      // BCLK
#define I2S_WS_PIN 16       // LRCLK
#define I2S_DATA_IN_PIN 18  // DIN / SD

static void audio_inference_callback(uint32_t n_samples) {
  // n_samples = number of int16 samples in sampleBuffer[]
  for (uint32_t i = 0; i < n_samples; i++) {
    inference.buffers[inference.buf_select][inference.buf_count++] = sampleBuffer[i];

    if (inference.buf_count >= inference.n_samples) {
      inference.buf_select ^= 1;
      inference.buf_count = 0;
      inference.buf_ready = 1;
    }
  }
}

static void capture_samples(void *arg) {
  const uint32_t i2s_bytes_to_read = (uint32_t)(uintptr_t)arg;  // bytes
  // temporary buffer must be bytes
  uint8_t *i2s_read_buf = (uint8_t *)heap_caps_malloc(i2s_bytes_to_read, MALLOC_CAP_8BIT);
  if (!i2s_read_buf) {
    ei_printf("ERR: cannot alloc i2s_read_buf (%u bytes)\n", i2s_bytes_to_read);
    vTaskDelete(NULL);
    return;
  }

  while (record_status) {
    size_t bytes_read = 0;
    esp_err_t r = i2s_read(I2S_PORT, (void *)i2s_read_buf, i2s_bytes_to_read, &bytes_read, pdMS_TO_TICKS(200));
    if (r != ESP_OK) {
      ei_printf("Error in I2S read : %d\n", r);
      continue;
    }
    if (bytes_read == 0) {
      // nothing read, continue
      continue;
    }

    const int bytes_per_frame = 4;  // INMP441 = 32-bit frames
    int frames = bytes_read / bytes_per_frame;
    if (frames <= 0) {
      ei_printf("Partial/invalid frames: %d\n", frames);
      continue;
    }

    // Convert frames -> int16_t samples
    // Ensure we don't overflow sampleBuffer
    if ((size_t)frames > sample_buffer_size) {
      frames = sample_buffer_size;
    }

    for (int i = 0; i < frames; i++) {
      // little-endian: assemble 32-bit
      uint32_t idx = i * bytes_per_frame;
      int32_t val = (int32_t)((uint32_t)i2s_read_buf[idx] | ((uint32_t)i2s_read_buf[idx + 1] << 8) | ((uint32_t)i2s_read_buf[idx + 2] << 16) | ((uint32_t)i2s_read_buf[idx + 3] << 24));
      // convert to int16 (top 16 bits)
      // int16_t s16 = (int16_t)(val >> 16);
      int16_t s16 = (int16_t)(val >> 14);  // kadang 14 atau 13 bergantung board

      // amplify a bit if microphone level is low (same as original *8)
      // int32_t scaled = (int32_t)s16 * 8;
      int32_t scaled = (int32_t)s16 * 2;
      // int32_t scaled = s16;   // tanpa amplify

      if (scaled > INT16_MAX) scaled = INT16_MAX;
      if (scaled < INT16_MIN) scaled = INT16_MIN;
      sampleBuffer[i] = (int16_t)scaled;
    }

    // push converted samples to inference buffer
    if (record_status) {
      audio_inference_callback(frames);  // frames == number of int16 samples we converted
    } else {
      break;
    }
  }

  heap_caps_free(i2s_read_buf);
  vTaskDelete(NULL);
}

static bool microphone_inference_start(uint32_t n_samples) {
  inference.buffers[0] = (signed short *)malloc(n_samples * sizeof(signed short));
  if (inference.buffers[0] == NULL) {
    return false;
  }

  inference.buffers[1] = (signed short *)malloc(n_samples * sizeof(signed short));
  if (inference.buffers[1] == NULL) {
    free(inference.buffers[0]);
    return false;
  }

  inference.buf_select = 0;
  inference.buf_count = 0;
  inference.n_samples = n_samples;
  inference.buf_ready = 0;

  if (i2s_init(EI_CLASSIFIER_FREQUENCY) != 0) {
    ei_printf("Failed to start I2S!\n");
    // free buffers
    free(inference.buffers[0]);
    free(inference.buffers[1]);
    return false;
  }

  ei_sleep(100);

  record_status = true;

  // Decide bytes-to-read per I2S transaction:
  // Make it a multiple of frames; e.g. 512 frames * 4 bytes/frame
  const uint32_t frames_per_read = 512;
  const uint32_t bytes_to_read = frames_per_read * 4;  // 4 bytes per frame (32-bit)
  // task stack bigger for safety
  BaseType_t ok = xTaskCreate(capture_samples, "CaptureSamples", 1024 * 8, (void *)(uintptr_t)bytes_to_read, 10, NULL);
  if (ok != pdPASS) {
    ei_printf("ERR: failed to create capture task\n");
    i2s_deinit();
    free(inference.buffers[0]);
    free(inference.buffers[1]);
    return false;
  }

  return true;
}

static bool microphone_inference_record(void) {
  bool ret = true;

  if (inference.buf_ready == 1) {
    ei_printf(
      "Error sample buffer overrun. Decrease the number of slices per model window "
      "(EI_CLASSIFIER_SLICES_PER_MODEL_WINDOW)\n");
    ret = false;
  }

  while (inference.buf_ready == 0) {
    delay(1);
  }

  inference.buf_ready = 0;
  return ret;
}

/**
 * Get raw audio signal data
 */
static int microphone_audio_signal_get_data(size_t offset, size_t length, float *out_ptr) {
  numpy::int16_to_float(&inference.buffers[inference.buf_select ^ 1][offset], out_ptr, length);
  return 0;
}

static void microphone_inference_end(void) {
  i2s_deinit();
  free(inference.buffers[0]);
  free(inference.buffers[1]);
}

/* I2S init for INMP441 (32-bit frames). Adjust pins to your board. */
static int i2s_init(uint32_t sampling_rate) {
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = sampling_rate,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_I2S_MSB,
    .intr_alloc_flags = 0,
    .dma_buf_count = 8,
    .dma_buf_len = 512,
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = -1,
  };

  i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_BCK_PIN,  // BCLK
    .ws_io_num = I2S_WS_PIN,    // LRCLK
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num = I2S_DATA_IN_PIN,  // DIN
  };
  esp_err_t ret = ESP_OK;

  ret = i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
  if (ret != ESP_OK) {
    ei_printf("Error in i2s_driver_install: %d\n", ret);
    return -1;
  }

  ret = i2s_set_pin(I2S_PORT, &pin_config);
  if (ret != ESP_OK) {
    ei_printf("Error in i2s_set_pin: %d\n", ret);
    return -2;
  }

  ret = i2s_zero_dma_buffer(I2S_PORT);
  if (ret != ESP_OK) {
    ei_printf("Error in initializing dma buffer with 0: %d\n", ret);
    return -3;
  }

  return 0;
}

static int i2s_deinit(void) {
  i2s_driver_uninstall(I2S_PORT);
  return 0;
}

#if !defined(EI_CLASSIFIER_SENSOR) || EI_CLASSIFIER_SENSOR != EI_CLASSIFIER_SENSOR_MICROPHONE
#error "Invalid model for current sensor."
#endif

/** Main Arduino sketch area **/
void setup() {
  Serial.begin(115200);
  while (!Serial)
    ;  // comment out if using non-native USB
  ei_printf("Edge Impulse Inferencing Demo\n");

  ei_printf("Inferencing settings:\n");
  ei_printf("\tInterval: ");
  ei_printf_float((float)EI_CLASSIFIER_INTERVAL_MS);
  ei_printf(" ms.\n");
  ei_printf("\tFrame size: %d\n", EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE);
  ei_printf("\tSample length: %d ms.\n", EI_CLASSIFIER_RAW_SAMPLE_COUNT / (EI_CLASSIFIER_FREQUENCY / 1000));
  ei_printf("\tNo. of classes: %d\n", EI_CLASSIFIER_LABEL_COUNT);

  run_classifier_init();
  ei_printf("\nStarting continuous inference in 2 seconds...\n");
  ei_sleep(2000);

  if (microphone_inference_start(EI_CLASSIFIER_SLICE_SIZE) == false) {
    ei_printf("ERR: Could not allocate audio buffer (size %d), this could be due to the window length of your model\r\n", EI_CLASSIFIER_RAW_SAMPLE_COUNT);
    return;
  }

  ei_printf("Recording...\n");

  setup_leds();
}

void loop() {
  bool m = microphone_inference_record();
  if (!m) {
    ei_printf("ERR: Failed to record audio...\n");
    return;
  }

  animate_leds();

  signal_t signal;
  signal.total_length = EI_CLASSIFIER_SLICE_SIZE;
  signal.get_data = &microphone_audio_signal_get_data;
  ei_impulse_result_t result = { 0 };

  EI_IMPULSE_ERROR r = run_classifier_continuous(&signal, &result, debug_nn);
  if (r != EI_IMPULSE_OK) {
    ei_printf("ERR: Failed to run classifier (%d)\n", r);
    return;
  }

  int best_ix = -1;
  float best_val = -1.0f;
  for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
    if (result.classification[ix].value > best_val) {
      best_val = result.classification[ix].value;
      best_ix = ix;
    }
  }
  if (best_ix >= 0) {
    indicate_class(best_ix);
  }

  if (++print_results >= (EI_CLASSIFIER_SLICES_PER_MODEL_WINDOW)) {
    ei_printf("Predictions ");
    ei_printf("(DSP: %d ms., Classification: %d ms., Anomaly: %d ms.)",
              result.timing.dsp, result.timing.classification, result.timing.anomaly);
    ei_printf(": \n");
    for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
      ei_printf("    %s: ", result.classification[ix].label);
      ei_printf_float(result.classification[ix].value);
      ei_printf("\n");
    }
#if EI_CLASSIFIER_HAS_ANOMALY == 1
    ei_printf("    anomaly score: ");
    ei_printf_float(result.anomaly);
    ei_printf("\n");
#endif
    print_results = 0;
  }
}

// led conf

// Helper: set RGB color (0..255)
static void set_rgb_raw(uint8_t r, uint8_t g, uint8_t b) {
#if USE_NEOPIXEL
  pixels.setPixelColor(0, pixels.Color(r, g, b));
  pixels.show();
#else
  // map 0..255 to PWM range 0..(2^PWM_RES -1)
  uint8_t maxv = (1 << PWM_RES) - 1;
  // If LED common anode, invert values. Adjust if your board uses common anode.
  uint8_t rr = map(r, 0, 255, 0, maxv);
  uint8_t gg = map(g, 0, 255, 0, maxv);
  uint8_t bb = map(b, 0, 255, 0, maxv);
  ledcWrite(CH_R, rr);
  ledcWrite(CH_G, gg);
  ledcWrite(CH_B, bb);
#endif
}

// Helper: set big LED (0 or 1 or PWM)
static void set_big_led(uint8_t v) {
#if USE_NEOPIXEL
  // if using NeoPixel, big LED is separate digital pin
  digitalWrite(LED_BIG_PIN, v ? HIGH : LOW);
#else
  // use PWM channel CH_BIG
  uint8_t maxv = (1 << PWM_RES) - 1;
  ledcWrite(CH_BIG, v ? maxv : 0);
#endif
}

// Call this to request display of a specific class index (0..4)
void indicate_class(int cls) {
  if (cls < 0 || cls > 4) return;
  if (cls == current_class) {
    // refresh animation start if repeated detection? we keep running
    return;
  }
  current_class = cls;
  anim_start_ms = millis();
  last_anim_tick = anim_start_ms;
  // optionally set initial static color
  const uint8_t *c = class_colors[cls];
  if (class_anim_type[cls] == 0) {
    // steady: set color and big LED on
    set_rgb_raw(c[0], c[1], c[2]);
    set_big_led(1);
  }
}

// Call this frequently (e.g. inside loop()) to update animations
void animate_leds() {
  if (current_class < 0) return;
  uint32_t now = millis();
  uint32_t t = now - anim_start_ms;
  const uint8_t *c = class_colors[current_class];
  uint8_t anim = class_anim_type[current_class];

  if (anim == 0) {
    // steady - nothing to animate (already set)
    // keep big led ON
    set_big_led(1);
    return;
  } else if (anim == 1) {
    // slow breathe: sine-like fade 0..1 every 2000ms
    float phase = (2.0f * PI) * ((float)(t % 2000) / 2000.0f);
    float factor = (sinf(phase - PI / 2.0f) + 1.0f) / 2.0f;  // 0..1
    uint8_t rr = (uint8_t)(c[0] * factor);
    uint8_t gg = (uint8_t)(c[1] * factor);
    uint8_t bb = (uint8_t)(c[2] * factor);
    set_rgb_raw(rr, gg, bb);
    // big LED pulses opposite (dim when RGB bright)
    set_big_led(factor < 0.5f ? 1 : 0);
  } else if (anim == 2) {
    // double blink: pattern = on(120ms) off(80) on(120) off(680) => period 1s
    uint32_t p = t % 1000;
    bool on = false;
    if (p < 120) on = true;
    else if (p < 200) on = false;
    else if (p < 320) on = true;
    else on = false;
    set_rgb_raw(on ? c[0] : 0, on ? c[1] : 0, on ? c[2] : 0);
    set_big_led(on ? 0 : 1);  // big LED inverse during blink
  } else if (anim == 3) {
    // fast blink every 200ms
    bool on = ((t / 100) % 2) == 0;
    set_rgb_raw(on ? c[0] : 0, on ? c[1] : 0, on ? c[2] : 0);
    set_big_led(on ? 1 : 0);
  } else if (anim == 4) {
    // alternated pulse: RGB slow pulse (1.2s), BIG LED toggles at 0.6s
    float phase = (2.0f * PI) * ((float)(t % 1200) / 1200.0f);
    float factor = (sinf(phase - PI / 2.0f) + 1.0f) / 2.0f;
    uint8_t rr = (uint8_t)(c[0] * factor);
    uint8_t gg = (uint8_t)(c[1] * factor);
    uint8_t bb = (uint8_t)(c[2] * factor);
    set_rgb_raw(rr, gg, bb);
    set_big_led(((t / 600) % 2) == 0 ? 1 : 0);
  }
}

// ------------------- setup for LEDs --------------------
void setup_leds() {
#if USE_NEOPIXEL
  pixels.begin();
  pixels.setBrightness(120);
  pixels.show();  // clear
  pinMode(LED_BIG_PIN, OUTPUT);
  digitalWrite(LED_BIG_PIN, LOW);
#else
  // configure PWM channels and pins
  ledcSetup(CH_R, PWM_FREQ, PWM_RES);
  ledcAttachPin(RGB_R_PIN, CH_R);
  ledcSetup(CH_G, PWM_FREQ, PWM_RES);
  ledcAttachPin(RGB_G_PIN, CH_G);
  ledcSetup(CH_B, PWM_FREQ, PWM_RES);
  ledcAttachPin(RGB_B_PIN, CH_B);

  ledcSetup(CH_BIG, PWM_FREQ, PWM_RES);
  ledcAttachPin(LED_BIG_PIN, CH_BIG);

  // start off
  ledcWrite(CH_R, 0);
  ledcWrite(CH_G, 0);
  ledcWrite(CH_B, 0);
  ledcWrite(CH_BIG, 0);
#endif
}