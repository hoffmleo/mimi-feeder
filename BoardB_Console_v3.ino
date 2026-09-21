/**
 * @file BoardB_Console_v3.ino
 * @brief Board B of the Mimi Feeder: the console board -- an ESP32-S3 that
 *        parses telemetry from Board A, shows it on a paged LCD, and
 *        bridges a BLE phone app to the feeder's command link.
 *
 * @details All tasks run on core 1. Incoming frames are checksum-verified
 *          before they update ::state, which is guarded by a mutex; the
 *          LCD and BLE notifier read snapshots of it. Two hardware timers
 *          drive button debouncing and a local countdown that keeps the
 *          displayed time smooth between schedule frames.
 */

#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

/**
 * @name BLE identity
 * @brief Custom 128-bit service exposing one write and one notify
 *        characteristic.
 * @{
 */
#define SERVICE_UUID     "6d1f7a20-3c8e-4b21-9f3a-2c5d81e47b90"  ///< Feeder service
#define CMD_CHAR_UUID    "6d1f7a21-3c8e-4b21-9f3a-2c5d81e47b90"  ///< Written by the phone to send a command
#define STATUS_CHAR_UUID "6d1f7a22-3c8e-4b21-9f3a-2c5d81e47b90"  ///< Notifies the phone of status text
#define BLE_NAME         "MimiFeeder"                            ///< Advertised device name
/** @} */

/**
 * @name I2C bus and LCD configuration
 * @{
 */
#define I2C_SDA  8     ///< I2C data pin
#define I2C_SCL  9     ///< I2C clock pin
#define LCD_ADDR 0x27  ///< PCF8574 backpack address; some boards use 0x3F
#define LCD_COLS 16    ///< LCD character columns
#define LCD_ROWS 2     ///< LCD character rows
/** @} */

/**
 * @name UART link to Board A
 * @brief Pins mirror Board A's, TX to RX.
 * @{
 */
#define UART_RX_PIN 18     ///< Serial1 RX
#define UART_TX_PIN 17     ///< Serial1 TX
#define UART_BAUD   115200 ///< Link baud rate
/** @} */

/**
 * @name Front panel buttons
 * @brief Active-low, wired against the internal pull-ups.
 * @{
 */
#define BTN_PAGE_PIN  4  ///< Advances the LCD page
#define BTN_LIGHT_PIN 5  ///< Toggles the backlight
#define NUM_BUTTONS   2  ///< Number of debounced inputs
/** @} */

/**
 * @name Debounce parameters
 * @brief A press registers only after ::DEBOUNCE_SAMPLES consecutive low
 *        readings at ::BTN_TICK_HZ.
 * @{
 */
#define BTN_TICK_HZ      64                        ///< Sampling rate in Hz
#define BTN_TICK_US      (1000000UL / BTN_TICK_HZ) ///< Timer alarm period
#define DEBOUNCE_SAMPLES 4                         ///< Agreeing samples needed
#define DEBOUNCE_MASK    ((1u << DEBOUNCE_SAMPLES) - 1)  ///< Shift-register mask
/** @} */

#define SEC_TICK_US 1000000UL  ///< Period of the one-second countdown timer

/**
 * @name Display and link behaviour
 * @{
 */
#define LCD_PERIOD_MS   250   ///< Redraw period when no button event arrives
#define LCD_PAGES       4     ///< Number of status pages in the rotation
#define PAGE_HOLD_MS    3000  ///< How long a page shows before auto-advancing
#define LINK_TIMEOUT_MS 15000 ///< Silence beyond this marks the link down
#define MSG_LEN         24    ///< Size of a notification or command string
#define NOTIFY_MAX      20    ///< Longest payload sent in a BLE notification
/** @} */

/**
 * @name UI event codes
 * @brief Values carried on ::uiQueue from the debounce task.
 * @{
 */
#define UI_EV_PAGE  0  ///< Page button pressed
#define UI_EV_LIGHT 1  ///< Backlight button pressed
/** @} */

/// Climate verdict reported by Board A.
enum ClimateState { CLIMATE_OK = 0, CLIMATE_HIGH = 1, CLIMATE_LOW = 2 };
/// Water level verdict reported by Board A.
enum WaterState { WATER_OK = 0, WATER_LOW = 1 };

/**
 * @brief Mirror of everything Board A has reported.
 * @details The valid flags separate "not received yet" from a real zero, so
 *          the LCD can show a waiting message instead of a misleading value.
 */
typedef struct {
  float    tempC;         ///< Last reported temperature in Celsius
  float    hum;           ///< Last reported relative humidity
  uint8_t  climate;       ///< Last ::ClimateState
  int32_t  waterRaw;      ///< Last raw water reading
  uint8_t  water;         ///< Last ::WaterState
  uint32_t remain;        ///< Seconds to the next feed, as last reported
  uint8_t  paused;        ///< Non-zero while feeding is paused
  uint32_t epoch;         ///< Timestamp of the most recent frame
  uint32_t feedCount;     ///< Feeds observed since boot
  bool     climateValid;  ///< True once a climate frame has arrived
  bool     schedValid;    ///< True once a schedule frame has arrived
  bool     linkOk;        ///< True while frames keep arriving
  uint32_t goodFrames;    ///< Frames that passed checksum
  uint32_t badFrames;     ///< Frames that failed checksum
  uint32_t lastFrameMs;   ///< millis() of the last accepted frame
} SysState;

LiquidCrystal_I2C lcd(LCD_ADDR, LCD_COLS, LCD_ROWS);  ///< Status display

static SemaphoreHandle_t stateMutex  = NULL;  ///< Protects ::state
static SemaphoreHandle_t i2cMutex    = NULL;  ///< Serializes access to the LCD bus
static QueueHandle_t     notifyQueue = NULL;  ///< Status strings awaiting BLE notification
static QueueHandle_t     cmdQueue    = NULL;  ///< Commands received over BLE
static QueueHandle_t     uiQueue     = NULL;  ///< Button events from the debounce task

static TaskHandle_t debounceTask = NULL;  ///< Notification target for the button timer
static hw_timer_t  *btnTimer     = NULL;  ///< Drives button sampling
static hw_timer_t  *secTimer     = NULL;  ///< Drives the local countdown

static const uint8_t buttonPins[NUM_BUTTONS] = { BTN_PAGE_PIN, BTN_LIGHT_PIN };  ///< Indexed by UI event code

static SysState state;                          ///< Mirror of Board A's reported state
static volatile bool deviceConnected = false;   ///< True while a BLE client is connected

static volatile uint32_t remainLocal = 0;      ///< Countdown ticked down locally between schedule frames
static volatile bool     pausedLocal = false;  ///< Freezes ::remainLocal while paused

static uint8_t pageIndex   = 0;     ///< Currently displayed LCD page
static uint32_t lastPageMs = 0;     ///< millis() when the page last changed
static bool backlightOn    = true;  ///< Desired backlight state

static BLECharacteristic *pStatusChar = NULL;  ///< Notify characteristic for status text

/**
 * @brief Button-timer ISR: wakes the debounce task to take a sample.
 * @details Sampling itself is left to the task so the ISR stays short and
 *          free of any I/O.
 */
static void IRAM_ATTR onBtnTick() {
  BaseType_t hpw = pdFALSE;
  if (debounceTask != NULL) vTaskNotifyGiveFromISR(debounceTask, &hpw);
  if (hpw == pdTRUE) portYIELD_FROM_ISR();
}

/**
 * @brief One-second ISR: advances the local feed countdown.
 * @details Keeps the displayed time moving between the schedule frames that
 *          resynchronise it.
 */
static void IRAM_ATTR onSecTick() {
  if (!pausedLocal && remainLocal > 0) remainLocal--;
}

/**
 * @brief Queues a status string for the serial log and BLE notification.
 * @details Never blocks; a notification is dropped rather than stalling the
 *          caller if the queue is full.
 * @param text Message to queue, truncated to ::MSG_LEN.
 */
static void pushNotify(const char *text) {
  char buf[MSG_LEN];
  snprintf(buf, sizeof(buf), "%s", text);
  xQueueSend(notifyQueue, buf, 0);
}

/**
 * @brief BLE callbacks for the command characteristic.
 * @details Normalises each write to an uppercase, null-terminated string and
 *          hands it to ::TaskUartTx, so no link I/O happens in BLE context.
 */
class CmdCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    uint8_t *data = pCharacteristic->getData();
    size_t len = pCharacteristic->getLength();
    if (len == 0) return;
    if (len >= MSG_LEN) len = MSG_LEN - 1;

    char buf[MSG_LEN];
    memcpy(buf, data, len);
    buf[len] = '\0';

    for (size_t i = 0; i < len; i++) {
      if (buf[i] >= 'a' && buf[i] <= 'z') buf[i] -= 32;
      if (buf[i] == '\r' || buf[i] == '\n') buf[i] = '\0';
    }

    xQueueSend(cmdQueue, buf, 0);
  }
};

/**
 * @brief BLE server callbacks tracking connection state.
 * @details Advertising is restarted on disconnect so the phone can reconnect
 *          without a reset.
 */
class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *pServer) {
    deviceConnected = true;
  }
  void onDisconnect(BLEServer *pServer) {
    deviceConnected = false;
    pServer->getAdvertising()->start();
  }
};

/**
 * @brief Validates a received line against the `$body*CS` framing.
 * @details Splits the line in place at the checksum delimiter, so the body
 *          is a null-terminated substring of the caller's buffer.
 * @param line Received line, modified in place.
 * @param bodyOut Receives a pointer to the body on success.
 * @return true if the framing and XOR checksum both match.
 */
static bool verifyFrame(char *line, char **bodyOut) {
  if (line[0] != '$') return false;
  char *star = strrchr(line, '*');
  if (!star || star - line < 2) return false;

  *star = '\0';
  char *body = line + 1;

  uint8_t cs = 0;
  for (char *p = body; *p; p++) cs ^= (uint8_t)*p;

  uint8_t given = (uint8_t)strtoul(star + 1, NULL, 16);
  if (cs != given) return false;

  *bodyOut = body;
  return true;
}

/**
 * @brief Reads the next comma-separated field as an integer.
 * @details Continues the strtok() walk started by handleFrame().
 * @return The field's value, or 0 if no field remains.
 */
static long nextLong(void) {
  char *tok = strtok(NULL, ",");
  return tok ? atol(tok) : 0;
}

/**
 * @brief Reads the next comma-separated field as a float.
 * @return The field's value, or 0.0f if no field remains.
 */
static float nextFloat(void) {
  char *tok = strtok(NULL, ",");
  return tok ? atof(tok) : 0.0f;
}

/**
 * @brief Parses one verified frame body into ::state and raises any
 *        resulting notification.
 * @details A schedule or boot frame also resynchronises the local countdown.
 *          Notifications are queued after the mutex is released so no BLE or
 *          queue work happens inside the critical section.
 * @param body Frame body, consumed in place by strtok().
 */
static void handleFrame(char *body) {
  char *type = strtok(body, ",");
  if (!type) return;

  char note[MSG_LEN] = "";
  bool resync = false;

  xSemaphoreTake(stateMutex, portMAX_DELAY);
  state.lastFrameMs = millis();

  if (strcmp(type, "CLIMATE") == 0) {
    state.epoch = (uint32_t)nextLong();
    state.tempC = nextFloat();
    state.hum   = nextFloat();
    uint8_t st  = (uint8_t)nextLong();
    if (state.climateValid && st != state.climate) {
      if (st == CLIMATE_HIGH)     snprintf(note, sizeof(note), "TOO HOT/HUMID");
      else if (st == CLIMATE_LOW) snprintf(note, sizeof(note), "TOO COLD/DRY");
      else                        snprintf(note, sizeof(note), "CLIMATE OK");
    }
    state.climate = st;
    state.climateValid = true;

  } else if (strcmp(type, "WATER") == 0) {
    state.epoch    = (uint32_t)nextLong();
    state.waterRaw = nextLong();
    uint8_t st     = (uint8_t)nextLong();
    if (st != state.water) {
      snprintf(note, sizeof(note), st ? "WATER LOW" : "WATER OK");
    }
    state.water = st;

  } else if (strcmp(type, "SCHED") == 0) {
    state.epoch  = (uint32_t)nextLong();
    state.remain = (uint32_t)nextLong();
    state.paused = (uint8_t)nextLong();
    state.schedValid = true;
    resync = true;

  } else if (strcmp(type, "FEED") == 0) {
    state.epoch = (uint32_t)nextLong();
    long steps  = nextLong();
    long st     = nextLong();
    state.feedCount++;
    if (st == 0)      snprintf(note, sizeof(note), "FED %ld steps", steps);
    else if (st == 1) snprintf(note, sizeof(note), "FEED FAIL OPEN");
    else              snprintf(note, sizeof(note), "FEED FAIL SHUT");

  } else if (strcmp(type, "BOOT") == 0) {
    state.epoch   = (uint32_t)nextLong();
    state.remain  = (uint32_t)nextLong();
    long restored = nextLong();
    state.schedValid = true;
    resync = true;
    snprintf(note, sizeof(note), restored ? "BOOT RESUMED" : "BOOT RESET");

  } else if (strcmp(type, "ACK") == 0) {
    state.epoch   = (uint32_t)nextLong();
    long code     = nextLong();
    long accepted = nextLong();
    const char *name = "CMD";
    switch (code) {
      case 1: name = "FEED";    break;
      case 2: name = "PAUSE";   break;
      case 3: name = "RESUME";  break;
      case 4: name = "PORTION"; break;
    }
    snprintf(note, sizeof(note), "%s %s", name, accepted ? "ACK" : "REJECT");

  } else if (strcmp(type, "FAULT") == 0) {
    state.epoch = (uint32_t)nextLong();
    long code   = nextLong();
    const char *name = "FAULT";
    switch (code) {
      case 1: name = "SENSOR FAULT"; break;
      case 2: name = "RTC FAULT";    break;
      case 3: name = "QUEUE DROP";   break;
      case 4: name = "FEED FAULT";   break;
      case 5: name = "NVRAM FAULT";  break;
    }
    snprintf(note, sizeof(note), "%s", name);
  }

  uint32_t r = state.remain;
  uint8_t  p = state.paused;
  xSemaphoreGive(stateMutex);

  if (resync) {
    pausedLocal = (p != 0);
    remainLocal = r;
  }

  if (note[0] != '\0') pushNotify(note);
}

/**
 * @brief Debounces the panel buttons and posts press events.
 * @details Woken by ::onBtnTick, with a 100ms timeout so a stalled timer
 *          cannot block it forever. Each button keeps a shift register of
 *          recent levels; an event fires on the transition to fully held,
 *          and the held flag clears only once the register is fully released.
 * @param pv Unused FreeRTOS task argument.
 */
static void TaskDebounce(void *pv) {
  uint8_t history[NUM_BUTTONS] = { 0 };
  bool    held[NUM_BUTTONS]    = { false };

  for (;;) {
    ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(100));

    for (uint8_t i = 0; i < NUM_BUTTONS; i++) {
      uint8_t level = (digitalRead(buttonPins[i]) == LOW) ? 1 : 0;
      history[i] = ((history[i] << 1) | level) & DEBOUNCE_MASK;

      if (history[i] == DEBOUNCE_MASK && !held[i]) {
        held[i] = true;
        uint8_t ev = i;
        xQueueSend(uiQueue, &ev, 0);
      } else if (history[i] == 0 && held[i]) {
        held[i] = false;
      }
    }
  }
}

/**
 * @brief Reassembles frames from Board A and tracks link health.
 * @details Polls every 20ms, counting good and bad frames separately and
 *          raising a notification when the link goes down or comes back.
 *          Oversized lines are truncated rather than allowed to overrun.
 * @param pv Unused FreeRTOS task argument.
 */
static void TaskUartRx(void *pv) {
  char buf[96];
  size_t len = 0;

  for (;;) {
    while (Serial1.available()) {
      char ch = (char)Serial1.read();
      if (ch == '\n' || ch == '\r') {
        if (len > 0) {
          buf[len] = '\0';
          char *body;
          if (verifyFrame(buf, &body)) {
            xSemaphoreTake(stateMutex, portMAX_DELAY);
            state.goodFrames++;
            bool wasDown = !state.linkOk;
            state.linkOk = true;
            xSemaphoreGive(stateMutex);
            handleFrame(body);
            if (wasDown) pushNotify("LINK OK");
          } else {
            xSemaphoreTake(stateMutex, portMAX_DELAY);
            state.badFrames++;
            xSemaphoreGive(stateMutex);
          }
          len = 0;
        }
      } else if (len < sizeof(buf) - 1) {
        buf[len++] = ch;
      }
    }

    xSemaphoreTake(stateMutex, portMAX_DELAY);
    bool timedOut = state.linkOk &&
                    (millis() - state.lastFrameMs > LINK_TIMEOUT_MS);
    if (timedOut) state.linkOk = false;
    xSemaphoreGive(stateMutex);
    if (timedOut) pushNotify("LINK LOST");

    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

/**
 * @brief Validates BLE commands and forwards them to Board A.
 * @details Blocks on ::cmdQueue. A portion outside the accepted range is
 *          rejected here rather than sent, and every outcome produces a
 *          notification so the phone sees what happened.
 * @param pv Unused FreeRTOS task argument.
 */
static void TaskUartTx(void *pv) {
  char cmd[MSG_LEN];

  for (;;) {
    if (xQueueReceive(cmdQueue, cmd, portMAX_DELAY) != pdTRUE) continue;

    if (strncmp(cmd, "FEED", 4) == 0) {
      Serial1.println("FEED");
      pushNotify("FEED SENT");
    } else if (strncmp(cmd, "PAUSE", 5) == 0) {
      Serial1.println("PAUSE");
      pushNotify("PAUSE SENT");
    } else if (strncmp(cmd, "RESUME", 6) == 0) {
      Serial1.println("RESUME");
      pushNotify("RESUME SENT");
    } else if (strncmp(cmd, "PORTION,", 8) == 0) {
      long v = atol(cmd + 8);
      if (v > 0 && v <= 8192) {
        Serial1.printf("PORTION,%ld\n", v);
        pushNotify("PORTION SET");
      } else {
        pushNotify("BAD PORTION");
      }
    } else {
      pushNotify("BAD CMD");
    }
  }
}

/**
 * @brief Writes one padded row to the LCD.
 * @details Padding to the full width overwrites whatever the previous page
 *          left behind, avoiding a clear-and-redraw flicker.
 * @param row Row index, 0 or 1.
 * @param text Text to display, truncated to ::LCD_COLS.
 */
static void lcdLine(uint8_t row, const char *text) {
  char buf[LCD_COLS + 1];
  snprintf(buf, sizeof(buf), "%-16s", text);
  lcd.setCursor(0, row);
  lcd.print(buf);
}

/**
 * @brief Renders the paged status display and handles button events.
 * @details Blocks on ::uiQueue with a ::LCD_PERIOD_MS timeout, so the page
 *          advances on a press or on its own after ::PAGE_HOLD_MS. The pages
 *          are climate, schedule, water, and BLE status; a lost link
 *          preempts all of them with a frame-count diagnostic. The countdown
 *          shown comes from ::remainLocal so it ticks every second rather
 *          than jumping between schedule frames.
 * @param pv Unused FreeRTOS task argument.
 */
static void TaskLcd(void *pv) {
  char top[24], bot[24];
  uint8_t ev;
  bool lightDirty = false;

  for (;;) {
    if (xQueueReceive(uiQueue, &ev, pdMS_TO_TICKS(LCD_PERIOD_MS)) == pdTRUE) {
      if (ev == UI_EV_PAGE) {
        pageIndex = (pageIndex + 1) % LCD_PAGES;
        lastPageMs = millis();
      } else if (ev == UI_EV_LIGHT) {
        backlightOn = !backlightOn;
        lightDirty = true;
      }
    } else if (millis() - lastPageMs >= PAGE_HOLD_MS) {
      pageIndex = (pageIndex + 1) % LCD_PAGES;
      lastPageMs = millis();
    }

    SysState s;
    xSemaphoreTake(stateMutex, portMAX_DELAY);
    s = state;
    xSemaphoreGive(stateMutex);

    uint32_t remain = remainLocal;

    if (!s.linkOk) {
      snprintf(top, sizeof(top), "LINK LOST");
      snprintf(bot, sizeof(bot), "frames %lu/%lu",
               (unsigned long)s.goodFrames, (unsigned long)s.badFrames);
    } else if (pageIndex == 0) {
      if (s.climateValid) {
        snprintf(top, sizeof(top), "T:%.1fC H:%.0f%%", s.tempC, s.hum);
        const char *c = (s.climate == CLIMATE_HIGH) ? "HIGH"
                      : (s.climate == CLIMATE_LOW)  ? "LOW" : "OK";
        snprintf(bot, sizeof(bot), "Climate: %s", c);
      } else {
        snprintf(top, sizeof(top), "Climate");
        snprintf(bot, sizeof(bot), "waiting...");
      }
    } else if (pageIndex == 1) {
      if (s.paused) {
        snprintf(top, sizeof(top), "Feeding PAUSED");
        snprintf(bot, sizeof(bot), "BLE: RESUME");
      } else if (s.schedValid) {
        snprintf(top, sizeof(top), "Next feed in");
        snprintf(bot, sizeof(bot), "%02lu:%02lu:%02lu",
                 (unsigned long)(remain / 3600),
                 (unsigned long)((remain % 3600) / 60),
                 (unsigned long)(remain % 60));
      } else {
        snprintf(top, sizeof(top), "Schedule");
        snprintf(bot, sizeof(bot), "waiting...");
      }
    } else if (pageIndex == 2) {
      snprintf(top, sizeof(top), "Water: %s", s.water ? "LOW" : "OK");
      snprintf(bot, sizeof(bot), "raw %ld", (long)s.waterRaw);
    } else {
      snprintf(top, sizeof(top), "BLE: %s", deviceConnected ? "connected" : "advert.");
      snprintf(bot, sizeof(bot), "Feeds: %lu", (unsigned long)s.feedCount);
    }

    if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(200)) == pdTRUE) {
      if (lightDirty) {
        if (backlightOn) lcd.backlight();
        else lcd.noBacklight();
        lightDirty = false;
      }
      lcdLine(0, top);
      lcdLine(1, bot);
      xSemaphoreGive(i2cMutex);
    }
  }
}

/**
 * @brief Logs queued status strings and pushes them to the BLE client.
 * @details Blocks on ::notifyQueue. Messages are always logged, but only
 *          notified while a client is connected, and are truncated to
 *          ::NOTIFY_MAX to fit one packet. The short delay after each
 *          notification keeps a burst from overrunning the BLE stack.
 * @param pv Unused FreeRTOS task argument.
 */
static void TaskBleNotify(void *pv) {
  char msg[MSG_LEN];

  for (;;) {
    if (xQueueReceive(notifyQueue, msg, portMAX_DELAY) != pdTRUE) continue;

    Serial.printf("[ble] %s\n", msg);

    if (!deviceConnected || pStatusChar == NULL) continue;

    size_t len = strlen(msg);
    if (len > NOTIFY_MAX) len = NOTIFY_MAX;

    pStatusChar->setValue((uint8_t *)msg, len);
    pStatusChar->notify();

    vTaskDelay(pdMS_TO_TICKS(30));
  }
}

/**
 * @brief Starts the button-sampling and one-second hardware timers.
 * @details Both branches configure a 1MHz tick; the ESP32 Arduino 3.x timer
 *          API replaced the prescaler-based calls used before it.
 */
static void startTimers(void) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  btnTimer = timerBegin(1000000);
  timerAttachInterrupt(btnTimer, &onBtnTick);
  timerAlarm(btnTimer, BTN_TICK_US, true, 0);

  secTimer = timerBegin(1000000);
  timerAttachInterrupt(secTimer, &onSecTick);
  timerAlarm(secTimer, SEC_TICK_US, true, 0);
#else
  btnTimer = timerBegin(0, 80, true);
  timerAttachInterrupt(btnTimer, &onBtnTick, true);
  timerAlarmWrite(btnTimer, BTN_TICK_US, true);
  timerAlarmEnable(btnTimer);

  secTimer = timerBegin(1, 80, true);
  timerAttachInterrupt(secTimer, &onSecTick, true);
  timerAlarmWrite(secTimer, SEC_TICK_US, true);
  timerAlarmEnable(secTimer);
#endif
}

/**
 * @brief Initialises the display, BLE server, tasks, and timers.
 * @details Halts on a Serial message if any RTOS object cannot be created. A
 *          missing LCD is only a warning, since the console still works over
 *          BLE and the serial log. The timers start last, once the task they
 *          notify exists.
 */
void setup() {
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && (millis() - t0) < 3000) delay(10);

  Serial.println("\n=== Mimi Feeder - Board B console ===");

  memset(&state, 0, sizeof(state));
  state.lastFrameMs = millis();
  lastPageMs = millis();

  for (uint8_t i = 0; i < NUM_BUTTONS; i++) {
    pinMode(buttonPins[i], INPUT_PULLUP);
  }

  stateMutex  = xSemaphoreCreateMutex();
  i2cMutex    = xSemaphoreCreateMutex();
  notifyQueue = xQueueCreate(12, MSG_LEN);
  cmdQueue    = xQueueCreate(4, MSG_LEN);
  uiQueue     = xQueueCreate(8, sizeof(uint8_t));

  if (!stateMutex || !i2cMutex || !notifyQueue || !cmdQueue || !uiQueue) {
    Serial.println("FATAL: RTOS object allocation failed");
    while (1) delay(1000);
  }

  Serial1.begin(UART_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(100000);

  Wire.beginTransmission(LCD_ADDR);
  if (Wire.endTransmission() != 0) {
    Serial.printf("WARNING: no LCD backpack at 0x%02X (try 0x3F)\n", LCD_ADDR);
  }

  lcd.init();
  lcd.backlight();
  lcdLine(0, "Mimi Feeder");
  lcdLine(1, "starting...");

  BLEDevice::init(BLE_NAME);
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);

  BLECharacteristic *pCmdChar = pService->createCharacteristic(
    CMD_CHAR_UUID,
    BLECharacteristic::PROPERTY_READ |
    BLECharacteristic::PROPERTY_WRITE
  );
  pCmdChar->setCallbacks(new CmdCallbacks());
  pCmdChar->setValue("READY");

  pStatusChar = pService->createCharacteristic(
    STATUS_CHAR_UUID,
    BLECharacteristic::PROPERTY_READ |
    BLECharacteristic::PROPERTY_NOTIFY
  );
  pStatusChar->addDescriptor(new BLE2902());
  pStatusChar->setValue("BOOTING");

  pService->start();

  BLEAdvertising *pAdvertising = pServer->getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->start();

  Serial.printf("Advertising as \"%s\"\n", BLE_NAME);

  xTaskCreatePinnedToCore(TaskDebounce,  "debnce", 2048, NULL, 4, &debounceTask, 1);
  xTaskCreatePinnedToCore(TaskUartRx,    "uartrx", 4096, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(TaskUartTx,    "uarttx", 4096, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(TaskLcd,       "lcd",    4096, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(TaskBleNotify, "blentf", 4096, NULL, 2, NULL, 1);

  startTimers();

  Serial.println("Tasks started, timers running.");
}

/// Unused: all work happens in FreeRTOS tasks, so the Arduino loop task exits.
void loop() {
  vTaskDelete(NULL);
}
