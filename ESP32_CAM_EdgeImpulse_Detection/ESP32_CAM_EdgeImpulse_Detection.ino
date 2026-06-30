/*
  ESP32-CAM Live Edge Impulse Detection
  Board: AI Thinker ESP32-CAM

  Detects:
  - water_bottle
  - xbox_controller
  - background / no object

  Required Arduino library:
  josejonathan19-project-1_inferencing
*/

#include <josejonathan19-project-1_inferencing.h>
#include "edge-impulse-sdk/dsp/image/image.hpp"
#include <Arduino.h>
#include "esp_camera.h"
#include "img_converters.h"

// ------------------------------------------------------------
// AI Thinker ESP32-CAM pin setup
// ------------------------------------------------------------
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27

#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5

#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

#define LED_GPIO_NUM       4

// ------------------------------------------------------------
// Camera frame settings
// ------------------------------------------------------------
#define EI_CAMERA_RAW_FRAME_BUFFER_COLS  320
#define EI_CAMERA_RAW_FRAME_BUFFER_ROWS  240
#define EI_CAMERA_FRAME_BYTE_SIZE        3

static bool debug_nn = false;
static bool camera_initialized = false;
static uint8_t *snapshot_buf = nullptr;

// ------------------------------------------------------------
// Function declarations
// ------------------------------------------------------------
bool ei_camera_init(void);
bool ei_camera_capture(uint32_t img_width, uint32_t img_height, uint8_t *out_buf);
static int ei_camera_get_data(size_t offset, size_t length, float *out_ptr);

// ------------------------------------------------------------
// Setup
// ------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(2000);

  Serial.println();
  Serial.println("==========================================");
  Serial.println("ESP32-CAM Edge Impulse Live Detection");
  Serial.println("Targets: water_bottle and xbox_controller");
  Serial.println("Board: AI Thinker ESP32-CAM");
  Serial.println("==========================================");

  pinMode(LED_GPIO_NUM, OUTPUT);
  digitalWrite(LED_GPIO_NUM, LOW);

  if (!ei_camera_init()) {
    Serial.println("ERROR: Camera initialization failed.");
    Serial.println("Check Arduino settings:");
    Serial.println("Board: ESP32 Dev Module");
    Serial.println("PSRAM: Enabled");
    Serial.println("Camera: AI Thinker ESP32-CAM");
    while (true) {
      delay(1000);
    }
  }

  Serial.println("Camera initialized successfully.");
  Serial.println("Starting live object detection...");
}

// ------------------------------------------------------------
// Main loop
// ------------------------------------------------------------
void loop() {
  snapshot_buf = (uint8_t*)malloc(
    EI_CAMERA_RAW_FRAME_BUFFER_COLS *
    EI_CAMERA_RAW_FRAME_BUFFER_ROWS *
    EI_CAMERA_FRAME_BYTE_SIZE
  );

  if (snapshot_buf == nullptr) {
    Serial.println("ERROR: Failed to allocate snapshot buffer.");
    delay(1000);
    return;
  }

  bool captured = ei_camera_capture(
    EI_CLASSIFIER_INPUT_WIDTH,
    EI_CLASSIFIER_INPUT_HEIGHT,
    snapshot_buf
  );

  if (!captured) {
    Serial.println("ERROR: Failed to capture image.");
    free(snapshot_buf);
    snapshot_buf = nullptr;
    delay(1000);
    return;
  }

  ei::signal_t signal;
  signal.total_length = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT;
  signal.get_data = &ei_camera_get_data;

  ei_impulse_result_t result = { 0 };

  EI_IMPULSE_ERROR err = run_classifier(&signal, &result, debug_nn);

  if (err != EI_IMPULSE_OK) {
    Serial.print("ERROR: run_classifier failed. Code: ");
    Serial.println(err);
    free(snapshot_buf);
    snapshot_buf = nullptr;
    delay(1000);
    return;
  }

  Serial.println();
  Serial.println("------------- Detection Result -------------");

  Serial.print("DSP time: ");
  Serial.print(result.timing.dsp);
  Serial.println(" ms");

  Serial.print("Classification time: ");
  Serial.print(result.timing.classification);
  Serial.println(" ms");

#if EI_CLASSIFIER_OBJECT_DETECTION == 1

  bool object_found = false;

  Serial.println("Detected objects:");

  for (uint32_t i = 0; i < result.bounding_boxes_count; i++) {
    ei_impulse_result_bounding_box_t bb = result.bounding_boxes[i];

    if (bb.value == 0) {
      continue;
    }

    if (bb.value >= 0.50) {
      object_found = true;

      Serial.print("Object: ");
      Serial.println(bb.label);

      Serial.print("Confidence: ");
      Serial.print(bb.value * 100.0);
      Serial.println("%");

      Serial.print("Bounding box: ");
      Serial.print("x=");
      Serial.print(bb.x);
      Serial.print(", y=");
      Serial.print(bb.y);
      Serial.print(", width=");
      Serial.print(bb.width);
      Serial.print(", height=");
      Serial.println(bb.height);

      Serial.println("--------------------------------------------");
    }
  }

  if (!object_found) {
    Serial.println("No water bottle or Xbox controller detected.");
    Serial.println("Result: background / no target object");
  }

#else

  float highest_score = 0.0;
  const char *best_label = "unknown";

  Serial.println("Classification predictions:");

  for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
    float score = result.classification[ix].value;

    Serial.print(result.classification[ix].label);
    Serial.print(": ");
    Serial.print(score * 100.0);
    Serial.println("%");

    if (score > highest_score) {
      highest_score = score;
      best_label = result.classification[ix].label;
    }
  }

  Serial.print("Best prediction: ");
  Serial.print(best_label);
  Serial.print(" with ");
  Serial.print(highest_score * 100.0);
  Serial.println("% confidence");

#endif

  free(snapshot_buf);
  snapshot_buf = nullptr;

  delay(1500);
}

// ------------------------------------------------------------
// Initialize ESP32-CAM
// ------------------------------------------------------------
bool ei_camera_init(void) {
  if (camera_initialized) {
    return true;
  }

  camera_config_t config;

  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;

  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;

  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;

  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;

  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;

  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  config.frame_size = FRAMESIZE_QVGA;
  config.jpeg_quality = 12;
  config.fb_count = 1;

  if (psramFound()) {
    config.fb_location = CAMERA_FB_IN_PSRAM;
    config.grab_mode = CAMERA_GRAB_LATEST;
    Serial.println("PSRAM found.");
  } else {
    config.fb_location = CAMERA_FB_IN_DRAM;
    config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
    Serial.println("WARNING: PSRAM not found. Enable PSRAM in Arduino IDE.");
  }

  esp_err_t err = esp_camera_init(&config);

  if (err != ESP_OK) {
    Serial.print("Camera init failed with error 0x");
    Serial.println(err, HEX);
    return false;
  }

  sensor_t *s = esp_camera_sensor_get();

  if (s == nullptr) {
    Serial.println("ERROR: Failed to get camera sensor.");
    return false;
  }

  s->set_brightness(s, 1);
  s->set_contrast(s, 1);
  s->set_saturation(s, 0);
  s->set_framesize(s, FRAMESIZE_QVGA);

  camera_initialized = true;
  return true;
}

// ------------------------------------------------------------
// Capture image and resize for Edge Impulse
// ------------------------------------------------------------
bool ei_camera_capture(uint32_t img_width, uint32_t img_height, uint8_t *out_buf) {
  if (!camera_initialized) {
    Serial.println("Camera is not initialized.");
    return false;
  }

  camera_fb_t *fb = esp_camera_fb_get();

  if (!fb) {
    Serial.println("Camera capture failed.");
    return false;
  }

  bool converted = fmt2rgb888(fb->buf, fb->len, PIXFORMAT_JPEG, out_buf);

  esp_camera_fb_return(fb);

  if (!converted) {
    Serial.println("JPEG to RGB888 conversion failed.");
    return false;
  }

  if ((img_width != EI_CAMERA_RAW_FRAME_BUFFER_COLS) ||
      (img_height != EI_CAMERA_RAW_FRAME_BUFFER_ROWS)) {

    ei::image::processing::crop_and_interpolate_rgb888(
      out_buf,
      EI_CAMERA_RAW_FRAME_BUFFER_COLS,
      EI_CAMERA_RAW_FRAME_BUFFER_ROWS,
      out_buf,
      img_width,
      img_height
    );
  }

  return true;
}

// ------------------------------------------------------------
// Convert image data into Edge Impulse signal
// ------------------------------------------------------------
static int ei_camera_get_data(size_t offset, size_t length, float *out_ptr) {
  size_t pixel_ix = offset * 3;
  size_t out_ptr_ix = 0;

  while (length != 0) {
    out_ptr[out_ptr_ix] =
      (snapshot_buf[pixel_ix + 2] << 16) +
      (snapshot_buf[pixel_ix + 1] << 8) +
      snapshot_buf[pixel_ix];

    out_ptr_ix++;
    pixel_ix += 3;
    length--;
  }

  return 0;
}