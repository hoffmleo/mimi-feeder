/**
 * @file BoardA_MimiFeeder_v5.ino
 * @brief Board A of the Mimi Feeder: an ESP32-S3 running FreeRTOS that
 *        senses climate and water, dispenses food on a timed schedule,
 *        and streams checksummed telemetry to the console board.
 *
 * @details Climate, water, RTC and telemetry-TX tasks run on core 1;
 *          the scheduler, feeder, and command receiver run on core 0.
 *          Shared state is protected by an I2C mutex, a time-anchor
 *          mutex, and a spinlock guarding the feed claim. The next feed
 *          deadline and the portion size persist in the DS1307's
 *          battery-backed NVRAM, so a reset does not restart the cycle.
 */

/**
 * @name Build-time options
 * @brief Optional stages compiled in or out of the build.
 * @{
 */
#define ENABLE_SERVO   0  ///< 1 = run the servo gate stage after each dispense
#define ENABLE_FEED_RX 1  ///< 1 = accept commands from the console board
/** @} */

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_AM2320.h>
#include "RTClib.h"
#include <AccelStepper.h>
#if ENABLE_SERVO
#include <ESP32Servo.h>
#endif

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"

/**
 * @name I2C bus and device addresses
 * @brief Shared bus for the AM2320 sensor and the DS1307 RTC.
 * @{
 */
#define I2C_SDA        8       ///< I2C data pin
#define I2C_SCL        9       ///< I2C clock pin
#define I2C_CLK_SPEED  100000  ///< Bus speed in Hz; the AM2320 will not go faster
#define AM2320_ADDR    0x5C    ///< AM2320 temperature/humidity address
#define RTC_ADDR       0x68    ///< DS1307 RTC address
/** @} */

/**
 * @name Stepper coil pins
 * @brief ULN2003 inputs driving the 28BYJ-48.
 * @{
 */
#define IN1 10  ///< Coil 1
#define IN2 11  ///< Coil 2
#define IN3 12  ///< Coil 3
#define IN4 13  ///< Coil 4
/** @} */

/**
 * @name Water sensing pins
 * @brief The sensor is powered only while sampling to limit corrosion.
 * @{
 */
#define POWER_ON_PIN   2  ///< Drives the water sensor's supply
#define WATER_SENS_PIN 1  ///< Water level analog input
/** @} */

/**
 * @name Indicator and actuator pins
 * @{
 */
#define LED_RED   4  ///< Lit while the climate state is CLIMATE_HIGH
#define LED_GREEN 5  ///< Lit while the climate state is CLIMATE_LOW
#define SERVO_PIN 6  ///< Gate servo signal (only used when ::ENABLE_SERVO)
/** @} */

/**
 * @name UART link to the console board
 * @{
 */
#define UART_TX_PIN 17     ///< Serial1 TX
#define UART_RX_PIN 18     ///< Serial1 RX
#define UART_BAUD   115200 ///< Link baud rate
/** @} */

/**
 * @name Climate thresholds
 * @brief Comfortable band for the cat's food area; a reading must repeat
 *        ::CLIMATE_CONFIRM times before the reported state changes.
 * @{
 */
#define TEMP_MIN_C      18.0f  ///< Below this is CLIMATE_LOW
#define TEMP_MAX_C      27.0f  ///< Above this is CLIMATE_HIGH
#define HUM_MIN_PCT     30.0f  ///< Below this is CLIMATE_LOW
#define HUM_MAX_PCT     70.0f  ///< Above this is CLIMATE_HIGH
#define CLIMATE_CONFIRM 2      ///< Consecutive agreeing samples needed to switch state
/** @} */

/**
 * @name Water thresholds
 * @{
 */
#define WATER_LOW_RAW 800  ///< Raw ADC counts below this mean the bowl is low
#define WATER_CONFIRM 3    ///< Consecutive agreeing samples needed to switch state
/** @} */

/**
 * @name Feed schedule and stepper motion
 * @{
 */
#define FEED_INTERVAL_S 300UL    ///< Seconds between scheduled feeds
#define STEPS_PER_REV   4096L    ///< Half-step count for one 28BYJ-48 revolution
#define PORTION_STEPS   2048L    ///< Default portion, half a revolution
#define STEP_RATE_SPS   1000.0f  ///< Constant dispensing speed in steps/second
#define FEED_TIMEOUT_MS 15000UL  ///< A move exceeding this is treated as jammed
/** @} */

/**
 * @name Valve return behaviour
 * @{
 */
#define VALVE_RETURN    1     ///< 1 = reverse the auger back to its start after dispensing
#define VALVE_DWELL_MS  2000  ///< Time held open before returning
#define VALVE_SETTLE_MS 400   ///< Pause before the return move begins
/** @} */

/**
 * @name Servo gate positions
 * @brief Only used when ::ENABLE_SERVO is set.
 * @{
 */
#define GATE_CLOSED_US 1000  ///< Pulse width for the closed gate
#define GATE_OPEN_US   1500  ///< Pulse width for the open gate
#define GATE_HOLD_MS   1200  ///< Time the gate stays open
/** @} */

/**
 * @name Task periods
 * @{
 */
#define CLIMATE_PERIOD_MS 2000    ///< Climate sampling period
#define WATER_PERIOD_MS   1000    ///< Water sampling period
#define RTC_PERIOD_MS     300000  ///< How often the epoch anchor is re-read from the RTC
#define SCHED_PERIOD_MS   1000    ///< Scheduler wake-up timeout
#define SCHED_REPORT_MS   5000    ///< How often a schedule frame is sent
#define DEADLINE_CHECK_S  30      ///< Software-timer period that nudges the scheduler
/** @} */

/**
 * @name Telemetry and validation limits
 * @{
 */
#define TELE_QUEUE_DEPTH 32           ///< Telemetry queue capacity in messages
#define EPOCH_SANITY     1600000000UL ///< Epochs at or below this are rejected as unset
/** @} */

/**
 * @name Persisted record layout
 * @brief Header fields that identify a valid ::FeedRecord in RTC NVRAM.
 * @{
 */
#define NVRAM_ADDR    0     ///< Offset of the record within NVRAM
#define NVRAM_MAGIC   0xA5  ///< Marks the region as written by this firmware
#define NVRAM_VERSION 1     ///< Layout version; a mismatch discards the record
/** @} */

/// Telemetry frame types carried in ::TeleMsg::type.
enum MsgType { MSG_CLIMATE, MSG_WATER, MSG_SCHED, MSG_FEED, MSG_FAULT,
               MSG_BOOT, MSG_ACK };
/// Command identifiers echoed back in an acknowledgement frame.
enum AckCode { ACK_UNKNOWN = 0, ACK_FEED = 1, ACK_PAUSE = 2,
               ACK_RESUME = 3, ACK_PORTION = 4 };
/// Debounced climate verdict.
enum ClimateState { CLIMATE_OK = 0, CLIMATE_HIGH = 1, CLIMATE_LOW = 2 };
/// Debounced water level verdict.
enum WaterState { WATER_OK = 0, WATER_LOW = 1 };
/// Fault reasons reported in a MSG_FAULT frame.
enum FaultCode { FAULT_AM2320 = 1, FAULT_RTC = 2, FAULT_QUEUE = 3,
                 FAULT_FEED = 4, FAULT_NVRAM = 5 };

/**
 * @brief One queued telemetry message.
 * @details The payload fields are generic so every frame type shares a
 *          single queue; the meaning of each depends on ::type.
 */
typedef struct {
  uint8_t  type;   ///< A ::MsgType value
  uint32_t epoch;  ///< Timestamp, filled in by post()
  float    a;      ///< Temperature, or the dropped-message count on a fault
  float    b;      ///< Humidity
  int32_t  c;      ///< Raw reading, step count, seconds remaining, or fault code
  uint8_t  state;  ///< Debounced state, feed result, or acceptance flag
} TeleMsg;

/**
 * @brief Feed schedule as stored in the DS1307's battery-backed NVRAM.
 * @details Packed so the layout is stable across builds, and validated by
 *          magic, version, and checksum before it is trusted.
 */
typedef struct __attribute__((packed)) {
  uint8_t  magic;     ///< Must equal ::NVRAM_MAGIC
  uint8_t  version;   ///< Must equal ::NVRAM_VERSION
  uint32_t nextFeed;  ///< Epoch of the next scheduled feed
  int32_t  portion;   ///< Portion size in steps
  uint8_t  checksum;  ///< XOR over every preceding byte
} FeedRecord;

Adafruit_AM2320 am2320 = Adafruit_AM2320();  ///< Temperature and humidity sensor
RTC_DS1307 rtc;                              ///< Real-time clock and NVRAM store
AccelStepper stepper(AccelStepper::HALF4WIRE, IN1, IN3, IN2, IN4);  ///< Auger drive; pin order is IN1/IN3/IN2/IN4 for correct phasing
#if ENABLE_SERVO
Servo gate;  ///< Optional food gate
#endif

static SemaphoreHandle_t i2cMutex  = NULL;   ///< Serializes access to the shared I2C bus
static SemaphoreHandle_t timeMutex = NULL;   ///< Protects ::epochAnchor and ::msAnchor
static QueueHandle_t     teleQueue = NULL;   ///< Telemetry messages awaiting transmission
static QueueHandle_t     feedQueue = NULL;   ///< Pending dispense requests, in steps
static TimerHandle_t     feedTimer = NULL;   ///< Periodically nudges the scheduler
static TaskHandle_t      schedTask = NULL;   ///< Notification target for the scheduler

static portMUX_TYPE  schedSpin   = portMUX_INITIALIZER_UNLOCKED;  ///< Guards ::feedClaimed
static volatile bool feedClaimed = false;  ///< True while a feed is queued or running

static uint32_t epochAnchor = 0;  ///< Last epoch read from the RTC
static uint32_t msAnchor    = 0;  ///< millis() value when ::epochAnchor was taken

static volatile uint32_t nextFeedEpoch = 0;             ///< Epoch of the next scheduled feed
static volatile uint32_t droppedMsgs   = 0;             ///< Telemetry messages lost to a full queue
static volatile int32_t  portionSteps  = PORTION_STEPS; ///< Current portion size in steps
static volatile bool     paused        = false;         ///< True while scheduled feeding is suspended

/**
 * @brief Anchors the wall-clock epoch to the current millis() reading.
 * @param e Epoch seconds to anchor to.
 */
static void setEpoch(uint32_t e) {
  xSemaphoreTake(timeMutex, portMAX_DELAY);
  epochAnchor = e;
  msAnchor    = millis();
  xSemaphoreGive(timeMutex);
}

/**
 * @brief Returns the current epoch, extrapolated from the last anchor.
 * @details Avoids an I2C read on every timestamp; the anchor is refreshed
 *          by TaskRtc().
 * @return Current time in epoch seconds.
 */
static uint32_t nowEpoch(void) {
  xSemaphoreTake(timeMutex, portMAX_DELAY);
  uint32_t e = epochAnchor + (millis() - msAnchor) / 1000UL;
  xSemaphoreGive(timeMutex);
  return e;
}

/**
 * @brief Timestamps a telemetry message and queues it for transmission.
 * @details Drops the message rather than blocking if the queue stays full,
 *          counting the loss in ::droppedMsgs for later reporting.
 * @param m Message to send; its epoch field is overwritten.
 */
static void post(TeleMsg m) {
  m.epoch = nowEpoch();
  if (xQueueSend(teleQueue, &m, pdMS_TO_TICKS(50)) != pdTRUE) droppedMsgs++;
}

/**
 * @brief Wraps a body in the link frame `$body*CS` and writes it out.
 * @details The checksum is an XOR over the body, matching what the console
 *          board verifies. A copy also goes to the debug serial port.
 * @param body Comma-separated frame body.
 */
static void sendLine(const char *body) {
  uint8_t cs = 0;
  for (const char *p = body; *p; p++) cs ^= (uint8_t)*p;
  Serial1.printf("$%s*%02X\n", body, cs);
  Serial.printf("$%s*%02X\n", body, cs);
}

/**
 * @brief Computes the XOR checksum over a record, excluding its own
 *        checksum byte.
 * @param r Record to check.
 * @return The expected checksum value.
 */
static uint8_t recordChecksum(const FeedRecord *r) {
  const uint8_t *p = (const uint8_t *)r;
  uint8_t cs = 0;
  for (size_t i = 0; i < sizeof(FeedRecord) - 1; i++) cs ^= p[i];
  return cs;
}

/**
 * @brief Reads the persisted feed schedule from RTC NVRAM.
 * @param out Receives the record; untouched unless validation passes.
 * @return true if the bus was acquired and the record is valid.
 */
static bool nvramLoad(FeedRecord *out) {
  FeedRecord r;
  if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(500)) != pdTRUE) return false;
  rtc.readnvram((uint8_t *)&r, sizeof(r), NVRAM_ADDR);
  xSemaphoreGive(i2cMutex);

  if (r.magic != NVRAM_MAGIC) return false;
  if (r.version != NVRAM_VERSION) return false;
  if (recordChecksum(&r) != r.checksum) return false;

  *out = r;
  return true;
}

/**
 * @brief Writes the feed schedule to RTC NVRAM so it survives a reset.
 * @param nextFeed Epoch of the next scheduled feed.
 * @param portion Portion size in steps.
 * @return true if the bus was acquired and the write was issued.
 */
static bool nvramStore(uint32_t nextFeed, int32_t portion) {
  FeedRecord r;
  r.magic    = NVRAM_MAGIC;
  r.version  = NVRAM_VERSION;
  r.nextFeed = nextFeed;
  r.portion  = portion;
  r.checksum = recordChecksum(&r);

  if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(500)) != pdTRUE) return false;
  rtc.writenvram(NVRAM_ADDR, (uint8_t *)&r, sizeof(r));
  xSemaphoreGive(i2cMutex);
  return true;
}

/**
 * @brief Converts seconds to ticks, clamped to a legal timer period.
 * @details A zero period is rejected by FreeRTOS and an overlong one
 *          overflows, so both ends are pinned.
 * @param s Interval in seconds.
 * @return Equivalent tick count, at least 1.
 */
static TickType_t secondsToTicks(uint32_t s) {
  uint64_t ticks = (uint64_t)s * (uint64_t)configTICK_RATE_HZ;
  if (ticks > (uint64_t)(portMAX_DELAY - 1)) ticks = (uint64_t)(portMAX_DELAY - 1);
  if (ticks == 0) ticks = 1;
  return (TickType_t)ticks;
}

/**
 * @brief Atomically claims a due feed so it can only be dispatched once.
 * @details Both the scheduler and the deadline timer can observe the same
 *          deadline, so the test and the claim happen inside a spinlock.
 * @param now Current epoch.
 * @return true if the caller now owns the pending feed.
 */
static bool claimFeed(uint32_t now) {
  bool claimed = false;
  taskENTER_CRITICAL(&schedSpin);
  if (!feedClaimed && !paused && now >= nextFeedEpoch) {
    feedClaimed = true;
    claimed = true;
  }
  taskEXIT_CRITICAL(&schedSpin);
  return claimed;
}

/// Releases the feed claim so the next deadline can be taken.
static void releaseFeedClaim(void) {
  taskENTER_CRITICAL(&schedSpin);
  feedClaimed = false;
  taskEXIT_CRITICAL(&schedSpin);
}

/**
 * @brief Wakes the scheduler on every deadline-check tick.
 * @param xTimer Unused timer handle.
 */
static void deadlineTimerCb(TimerHandle_t xTimer) {
  if (schedTask != NULL) xTaskNotifyGive(schedTask);
}

/// Prints every responding I2C address, tagging the two expected devices.
static void i2cScan(void) {
  Serial.println("Scanning I2C bus...");
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      const char *tag = "(other)";
      if (addr == AM2320_ADDR) tag = "<- AM2320";
      if (addr == RTC_ADDR)    tag = "<- DS1307";
      Serial.printf("  0x%02X  %s\n", addr, tag);
    }
  }
}

/**
 * @brief Sends the empty transaction the AM2320 needs to leave sleep.
 * @details The sensor ignores the first address after idling, so this
 *          throwaway write must precede any real read.
 */
static void wakeAM2320(void) {
  Wire.beginTransmission(AM2320_ADDR);
  Wire.endTransmission();
  delay(10);
}

/**
 * @brief Samples temperature and humidity, debounces the verdict, drives
 *        the indicator LEDs, and reports both.
 * @details Runs on core 1 at ::CLIMATE_PERIOD_MS. A new verdict must repeat
 *          ::CLIMATE_CONFIRM times before it replaces the stable state, so
 *          a reading sitting on a threshold does not flicker the LEDs. A
 *          failed read is reported as FAULT_AM2320 instead.
 * @param pv Unused FreeRTOS task argument.
 */
static void TaskClimate(void *pv) {
  TickType_t last = xTaskGetTickCount();
  uint8_t stable  = CLIMATE_OK;
  uint8_t pending = CLIMATE_OK;
  uint8_t count   = 0;

  for (;;) {
    float t = NAN, h = NAN;
    if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(500)) == pdTRUE) {
      t = am2320.readTemperature();
      h = am2320.readHumidity();
      xSemaphoreGive(i2cMutex);
    }

    TeleMsg m = {};
    if (isnan(t) || isnan(h)) {
      m.type = MSG_FAULT;
      m.c    = FAULT_AM2320;
      post(m);
    } else {
      uint8_t raw = CLIMATE_OK;
      if (t > TEMP_MAX_C || h > HUM_MAX_PCT) raw = CLIMATE_HIGH;
      else if (t < TEMP_MIN_C || h < HUM_MIN_PCT) raw = CLIMATE_LOW;

      if (raw == stable) {
        count = 0;
        pending = raw;
      } else if (raw == pending) {
        if (++count >= CLIMATE_CONFIRM) {
          stable = raw;
          count = 0;
        }
      } else {
        pending = raw;
        count = 1;
      }

      digitalWrite(LED_RED,   stable == CLIMATE_HIGH);
      digitalWrite(LED_GREEN, stable == CLIMATE_LOW);

      m.type  = MSG_CLIMATE;
      m.a     = t;
      m.b     = h;
      m.state = stable;
      post(m);
    }

    vTaskDelayUntil(&last, pdMS_TO_TICKS(CLIMATE_PERIOD_MS));
  }
}

/**
 * @brief Samples the water level and reports the debounced state.
 * @details Runs on core 1 at ::WATER_PERIOD_MS. The probe is powered only
 *          for the duration of each sample to slow electrolytic corrosion,
 *          and ::WATER_CONFIRM agreeing samples are required before the
 *          reported state changes.
 * @param pv Unused FreeRTOS task argument.
 */
static void TaskWater(void *pv) {
  TickType_t last = xTaskGetTickCount();
  uint8_t stable = WATER_OK;
  uint8_t count  = 0;

  for (;;) {
    digitalWrite(POWER_ON_PIN, HIGH);
    vTaskDelay(pdMS_TO_TICKS(10));
    int raw = analogRead(WATER_SENS_PIN);
    vTaskDelay(pdMS_TO_TICKS(10));
    digitalWrite(POWER_ON_PIN, LOW);

    uint8_t sample = (raw < WATER_LOW_RAW) ? WATER_LOW : WATER_OK;
    if (sample != stable) {
      if (++count >= WATER_CONFIRM) {
        stable = sample;
        count = 0;
      }
    } else {
      count = 0;
    }

    TeleMsg m = {};
    m.type  = MSG_WATER;
    m.c     = raw;
    m.state = stable;
    post(m);

    vTaskDelayUntil(&last, pdMS_TO_TICKS(WATER_PERIOD_MS));
  }
}

/**
 * @brief Re-anchors the software clock from the RTC.
 * @details Runs on core 1 at ::RTC_PERIOD_MS, correcting the drift that
 *          accumulates in the millis()-based extrapolation. A stopped clock
 *          or an implausible timestamp is reported as FAULT_RTC and the old
 *          anchor is kept.
 * @param pv Unused FreeRTOS task argument.
 */
static void TaskRtc(void *pv) {
  TickType_t last = xTaskGetTickCount();

  for (;;) {
    vTaskDelayUntil(&last, pdMS_TO_TICKS(RTC_PERIOD_MS));

    uint32_t stamp = 0;
    bool ok = false;
    if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(500)) == pdTRUE) {
      if (rtc.isrunning()) {
        stamp = rtc.now().unixtime();
        ok = (stamp > EPOCH_SANITY);
      }
      xSemaphoreGive(i2cMutex);
    }

    if (ok) {
      setEpoch(stamp);
    } else {
      TeleMsg m = {};
      m.type = MSG_FAULT;
      m.c    = FAULT_RTC;
      post(m);
    }
  }
}

/**
 * @brief Dispatches feeds when their deadline passes and publishes the
 *        countdown.
 * @details Runs on core 0, woken either by the deadline timer or by its own
 *          ::SCHED_PERIOD_MS timeout. The claim is released again if the
 *          request cannot be queued, so the feed is retried rather than
 *          lost. Accumulated telemetry drops are reported alongside each
 *          schedule frame.
 * @param pv Unused FreeRTOS task argument.
 */
static void TaskScheduler(void *pv) {
  uint32_t lastReport = 0;

  for (;;) {
    ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(SCHED_PERIOD_MS));

    uint32_t now = nowEpoch();

    if (claimFeed(now)) {
      int32_t req = portionSteps;
      if (xQueueSend(feedQueue, &req, 0) != pdTRUE) releaseFeedClaim();
    }

    if (millis() - lastReport >= SCHED_REPORT_MS) {
      lastReport = millis();

      uint32_t target = nextFeedEpoch;
      uint32_t remain = (target > now) ? (target - now) : 0;

      TeleMsg m = {};
      m.type  = MSG_SCHED;
      m.c     = (int32_t)remain;
      m.state = paused ? 1 : 0;
      post(m);

      if (droppedMsgs > 0) {
        TeleMsg f = {};
        f.type = MSG_FAULT;
        f.c    = FAULT_QUEUE;
        f.a    = (float)droppedMsgs;
        post(f);
      }
    }
  }
}

/**
 * @brief Runs the auger a fixed number of steps at constant speed.
 * @details Yields every 5ms so the busy stepping loop cannot starve the
 *          other tasks or trip the watchdog, and gives up after
 *          ::FEED_TIMEOUT_MS on the assumption the auger is jammed. Coils
 *          are de-energised on exit so the motor does not sit and heat.
 * @param steps Steps to travel; negative reverses the direction.
 * @return true if the move finished within the timeout.
 */
static bool runStepper(int32_t steps) {
  stepper.enableOutputs();
  stepper.setCurrentPosition(0);
  stepper.moveTo(-steps);
  stepper.setSpeed(STEP_RATE_SPS);

  uint32_t t0 = millis();
  uint32_t lastYield = t0;
  bool ok = true;

  while (stepper.distanceToGo() != 0) {
    stepper.runSpeedToPosition();
    if (millis() - lastYield >= 5) {
      vTaskDelay(1);
      lastYield = millis();
    }
    if (millis() - t0 > FEED_TIMEOUT_MS) {
      ok = false;
      break;
    }
  }

  stepper.disableOutputs();
  return ok;
}

/**
 * @brief Carries out queued dispense requests and reschedules the next feed.
 * @details Runs on core 0 at the highest priority, blocking on ::feedQueue.
 *          The result code distinguishes a jam on the way out (1) from one
 *          on the return (2). The new deadline is persisted and the claim
 *          released whether or not the move succeeded, so one jam does not
 *          stall the schedule.
 * @param pv Unused FreeRTOS task argument.
 */
static void TaskFeeder(void *pv) {
  int32_t steps;

  for (;;) {
    if (xQueueReceive(feedQueue, &steps, portMAX_DELAY) != pdTRUE) continue;

    uint8_t result = 0;

    if (!runStepper(steps)) {
      result = 1;
    } else {
      vTaskDelay(pdMS_TO_TICKS(VALVE_DWELL_MS));
#if VALVE_RETURN
      vTaskDelay(pdMS_TO_TICKS(VALVE_SETTLE_MS));
      if (!runStepper(-steps)) result = 2;
#endif
    }

    bool ok = (result == 0);

#if ENABLE_SERVO
    if (ok) {
      gate.writeMicroseconds(GATE_OPEN_US);
      vTaskDelay(pdMS_TO_TICKS(GATE_HOLD_MS));
      gate.writeMicroseconds(GATE_CLOSED_US);
      vTaskDelay(pdMS_TO_TICKS(300));
    }
#endif

    uint32_t now = nowEpoch();
    nextFeedEpoch = now + FEED_INTERVAL_S;

    bool stored = nvramStore(nextFeedEpoch, portionSteps);
    releaseFeedClaim();

    TeleMsg m = {};
    m.type  = MSG_FEED;
    m.c     = steps;
    m.state = result;
    post(m);

    if (!ok) {
      TeleMsg f = {};
      f.type = MSG_FAULT;
      f.c    = FAULT_FEED;
      post(f);
    }
    if (!stored) {
      TeleMsg f = {};
      f.type = MSG_FAULT;
      f.c    = FAULT_NVRAM;
      post(f);
    }
  }
}

/**
 * @brief Formats queued telemetry and sends it over the link.
 * @details Runs on core 1, blocking on ::teleQueue. Each ::MsgType maps to
 *          a comma-separated body which sendLine() frames and checksums.
 *          Unrecognised types are discarded.
 * @param pv Unused FreeRTOS task argument.
 */
static void TaskUartTx(void *pv) {
  TeleMsg m;
  char body[96];

  for (;;) {
    if (xQueueReceive(teleQueue, &m, portMAX_DELAY) != pdTRUE) continue;

    switch (m.type) {
      case MSG_CLIMATE:
        snprintf(body, sizeof(body), "CLIMATE,%lu,%.1f,%.1f,%u",
                 (unsigned long)m.epoch, m.a, m.b, m.state);
        break;
      case MSG_WATER:
        snprintf(body, sizeof(body), "WATER,%lu,%ld,%u",
                 (unsigned long)m.epoch, (long)m.c, m.state);
        break;
      case MSG_SCHED:
        snprintf(body, sizeof(body), "SCHED,%lu,%ld,%u",
                 (unsigned long)m.epoch, (long)m.c, m.state);
        break;
      case MSG_FEED:
        snprintf(body, sizeof(body), "FEED,%lu,%ld,%u",
                 (unsigned long)m.epoch, (long)m.c, m.state);
        break;
      case MSG_FAULT:
        snprintf(body, sizeof(body), "FAULT,%lu,%ld,%.0f",
                 (unsigned long)m.epoch, (long)m.c, m.a);
        break;
      case MSG_BOOT:
        snprintf(body, sizeof(body), "BOOT,%lu,%ld,%u",
                 (unsigned long)m.epoch, (long)m.c, m.state);
        break;
      case MSG_ACK:
        snprintf(body, sizeof(body), "ACK,%lu,%ld,%u",
                 (unsigned long)m.epoch, (long)m.c, m.state);
        break;
      default:
        continue;
    }

    sendLine(body);
  }
}

#if ENABLE_FEED_RX
/**
 * @brief Queues an acknowledgement for a received command.
 * @param code An ::AckCode identifying the command.
 * @param accepted 1 if the command was carried out, 0 if refused.
 */
static void postAck(uint8_t code, uint8_t accepted) {
  TeleMsg m = {};
  m.type  = MSG_ACK;
  m.c     = code;
  m.state = accepted;
  post(m);
}

/**
 * @brief Executes one command from the console board and acknowledges it.
 * @details A manual FEED bypasses the deadline but is refused if the feed
 *          queue is full; RESUME nudges the scheduler so a deadline that
 *          passed while paused is served immediately. A new portion is
 *          range-checked and persisted before being acknowledged.
 * @param cmd Null-terminated command text without its framing.
 */
static void handleCommand(const char *cmd) {
  if (strncmp(cmd, "FEED", 4) == 0) {
    int32_t req = portionSteps;
    bool sent = (xQueueSend(feedQueue, &req, 0) == pdTRUE);
    postAck(ACK_FEED, sent ? 1 : 0);

  } else if (strncmp(cmd, "PAUSE", 5) == 0) {
    paused = true;
    postAck(ACK_PAUSE, 1);

  } else if (strncmp(cmd, "RESUME", 6) == 0) {
    paused = false;
    xTaskNotifyGive(schedTask);
    postAck(ACK_RESUME, 1);

  } else if (strncmp(cmd, "PORTION,", 8) == 0) {
    int32_t v = atol(cmd + 8);
    if (v > 0 && v <= STEPS_PER_REV * 2) {
      portionSteps = v;
      nvramStore(nextFeedEpoch, portionSteps);
      postAck(ACK_PORTION, 1);
    } else {
      postAck(ACK_PORTION, 0);
    }

  } else {
    postAck(ACK_UNKNOWN, 0);
  }
}

/**
 * @brief Reassembles newline-terminated commands arriving on the link.
 * @details Runs on core 0, polling every 20ms. Characters beyond the buffer
 *          are dropped rather than allowed to overrun it.
 * @param pv Unused FreeRTOS task argument.
 */
static void TaskUartRx(void *pv) {
  char buf[64];
  size_t len = 0;

  for (;;) {
    while (Serial1.available()) {
      char ch = (char)Serial1.read();
      if (ch == '\n' || ch == '\r') {
        if (len > 0) {
          buf[len] = '\0';
          handleCommand(buf);
          len = 0;
        }
      } else if (len < sizeof(buf) - 1) {
        buf[len++] = ch;
      }
    }
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}
#endif

/**
 * @brief Brings up the peripherals, restores the schedule, and starts every
 *        task and timer.
 * @details Halts on a Serial message if an RTOS object or the RTC is
 *          missing, since nothing downstream can run without them. A stored
 *          schedule is only trusted if the clock was already running and the
 *          saved deadline is plausible; otherwise the cycle restarts from
 *          boot.
 */
void setup() {
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && (millis() - t0) < 3000) delay(10);

  Serial.println("\n=== Mimi Feeder - Board A ===");

  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(POWER_ON_PIN, OUTPUT);
  digitalWrite(LED_RED, LOW);
  digitalWrite(LED_GREEN, LOW);
  digitalWrite(POWER_ON_PIN, LOW);

  Serial1.begin(UART_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);

  i2cMutex  = xSemaphoreCreateMutex();
  timeMutex = xSemaphoreCreateMutex();
  teleQueue = xQueueCreate(TELE_QUEUE_DEPTH, sizeof(TeleMsg));
  feedQueue = xQueueCreate(4, sizeof(int32_t));
  feedTimer = xTimerCreate("deadline", secondsToTicks(DEADLINE_CHECK_S),
                           pdTRUE, NULL, deadlineTimerCb);

  if (!i2cMutex || !timeMutex || !teleQueue || !feedQueue || !feedTimer) {
    Serial.println("FATAL: RTOS object allocation failed");
    while (1) delay(1000);
  }

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(I2C_CLK_SPEED);
  wakeAM2320();
  i2cScan();

  am2320.begin();
  am2320.readTemperature();

  if (!rtc.begin(&Wire)) {
    Serial.println("FATAL: no DS1307 at 0x68");
    while (1) delay(1000);
  }

  bool rtcWasHalted = !rtc.isrunning();
  if (rtcWasHalted) {
    Serial.println("RTC halted - setting to compile time");
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  uint32_t bootEpoch = rtc.now().unixtime();
  setEpoch(bootEpoch);

  FeedRecord rec;
  bool restored = false;

  if (!rtcWasHalted && nvramLoad(&rec)) {
    if (rec.nextFeed > EPOCH_SANITY && rec.nextFeed <= bootEpoch + FEED_INTERVAL_S) {
      nextFeedEpoch = rec.nextFeed;
      if (rec.portion > 0 && rec.portion <= STEPS_PER_REV * 2) portionSteps = rec.portion;
      restored = true;
    }
  }

  if (!restored) {
    nextFeedEpoch = bootEpoch + FEED_INTERVAL_S;
    nvramStore(nextFeedEpoch, portionSteps);
  }

  uint32_t remain = (nextFeedEpoch > bootEpoch) ? (nextFeedEpoch - bootEpoch) : 0;
  Serial.printf("Schedule %s: next feed in %lu s (portion %ld steps)\n",
                restored ? "restored from NVRAM" : "initialized",
                (unsigned long)remain, (long)portionSteps);

  stepper.setMaxSpeed(2000.0);
  stepper.disableOutputs();

#if ENABLE_SERVO
  gate.setPeriodHertz(50);
  gate.attach(SERVO_PIN, 1000, 2000);
  gate.writeMicroseconds(GATE_CLOSED_US);
#endif

  xTaskCreatePinnedToCore(TaskClimate,   "climate", 4096, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(TaskWater,     "water",   4096, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(TaskRtc,       "rtc",     4096, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(TaskUartTx,    "uarttx",  4096, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(TaskScheduler, "sched",   4096, NULL, 3, &schedTask, 0);
  xTaskCreatePinnedToCore(TaskFeeder,    "feeder",  4096, NULL, 4, NULL, 0);
#if ENABLE_FEED_RX
  xTaskCreatePinnedToCore(TaskUartRx,    "uartrx",  4096, NULL, 3, NULL, 0);
#endif

  TeleMsg boot = {};
  boot.type  = MSG_BOOT;
  boot.c     = (int32_t)remain;
  boot.state = restored ? 1 : 0;
  post(boot);

  xTimerStart(feedTimer, 0);

  Serial.println("Tasks started.");
}

/// Unused: all work happens in FreeRTOS tasks, so the Arduino loop task exits.
void loop() {
  vTaskDelete(NULL);
}
