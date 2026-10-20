// FootPod capture firmware — milestone 1
// Streams raw IMU samples over BLE so you can record real runs and tune the
// stride / contact-time math offline.
//
// Board:   "Seeed nRF52 Boards" (NOT the mbed-enabled package)
//          -> Seeed XIAO nRF52840 Sense
// Library: "Seeed Arduino LSM6DS3" (Library Manager)
//
// Sleep: after SLEEP_IDLE_MS without movement (connected or not) the pod
// blinks purple three times and powers down. The IMU stays
// on in a low-power wake-on-motion mode; tap or pick up the pod and it
// restarts as if just powered on. It never sleeps while on USB power.
// Type 'q' in the Serial Monitor to sleep immediately.

#include <bluefruit.h>
#include <Wire.h>
#include <LSM6DS3.h>

// ---------------- config ----------------
#define POD_SIDE    "L"      // "L" or "R" — becomes the BLE name FootPod-L / FootPod-R
#define IMU_ODR_HZ  416      // 208 or 416
static const uint8_t SAMPLES_PER_PKT = 19;   // 8-byte header + 19*12 = 236 bytes (fits MTU 247)

// ---------------- LSM6DS3TR-C registers ----------------
#define REG_FIFO_CTRL5 0x0A
#define REG_WHO_AM_I   0x0F
#define REG_CTRL1_XL   0x10
#define REG_CTRL2_G    0x11
#define REG_CTRL3_C    0x12
#define REG_STATUS     0x1E
#define REG_OUTX_L_G   0x22   // gx,gy,gz,ax,ay,az — 12 bytes, little-endian int16
#define REG_WAKE_UP_SRC 0x1B
#define REG_CTRL6_C    0x15
#define REG_TAP_CFG    0x58
#define REG_WAKE_UP_THS 0x5B
#define REG_WAKE_UP_DUR 0x5C
#define REG_MD1_CFG    0x5E

#if IMU_ODR_HZ == 416
  #define ODR_BITS 0x60
#elif IMU_ODR_HZ == 208
  #define ODR_BITS 0x50
#else
  #error "IMU_ODR_HZ must be 208 or 416"
#endif
#define XL_FS_16G     0x04    // ±16 g   -> 0.488 mg/LSB
#define G_FS_2000DPS  0x0C    // ±2000 dps -> 70 mdps/LSB

// ---------------- sleep ----------------
const uint32_t SLEEP_IDLE_MS = 5UL * 60 * 1000;   // still this long -> sleep, even if a page is connected
// (a forgotten browser tab keeps reconnecting, so a longer "connected" limit kept pods awake for hours)
const float    MOTION_G     = 0.05f;   // accel deviation from gravity that counts as movement
const float    MOTION_DPS   = 20.0f;   // rotation (above the gyro's own offset) that counts as movement
const uint8_t  WAKE_THRESHOLD = 2;     // wake level in sleep, x31 mg (2 = ~62 mg)
static uint32_t lastActivityMs = 0;
static float gravX = 0, gravY = 0, gravZ = 1;   // low-passed accel, g
static float gyroBias[3] = {0, 0, 0};           // low-passed gyro, dps (tracks the offset)
static bool  sleepFilterInit = false;
#define REG_INT1_CTRL 0x0D

// ---------------- BLE UUIDs ----------------
// Service: a3c40000-5f1d-4b8e-9c3a-7d1e2f3b4c50
// IMU raw: a3c40001-5f1d-4b8e-9c3a-7d1e2f3b4c50 (notify)
const uint8_t SVC_UUID[16] = {0x50,0x4c,0x3b,0x2f,0x1e,0x7d,0x3a,0x9c,
                              0x8e,0x4b,0x1d,0x5f,0x00,0x00,0xc4,0xa3};
const uint8_t IMU_UUID[16] = {0x50,0x4c,0x3b,0x2f,0x1e,0x7d,0x3a,0x9c,
                              0x8e,0x4b,0x1d,0x5f,0x01,0x00,0xc4,0xa3};
// Status: a3c40002-... (read + notify, 1/s) — lets the phone see the sleep timer on battery
const uint8_t STAT_UUID[16] = {0x50,0x4c,0x3b,0x2f,0x1e,0x7d,0x3a,0x9c,
                               0x8e,0x4b,0x1d,0x5f,0x02,0x00,0xc4,0xa3};

BLEService        podSvc(SVC_UUID);
BLECharacteristic imuChr(IMU_UUID);
BLECharacteristic statChr(STAT_UUID);
struct __attribute__((packed)) Stat {
  uint32_t uptime_s;   // seconds since this boot
  uint16_t still_s;    // seconds without movement
  uint16_t limit_s;    // sleeps when still_s passes this (on battery)
  uint8_t  flags;      // bit0 USB power, bit1 woke from sleep, bit2 page connected
  uint8_t  version;
  uint8_t  sleeps;     // times it has slept since the last cold start (63 max)
};
static bool wokeFromSleep = false;
static uint8_t sleepCount = 0;
// GPREGRET2 survives System OFF (the bootloader clears RESETREAS, so we can't
// rely on that). We store 0x80 | sleep count there just before sleeping.
#define SLEEP_MARK 0x80
BLEDis            bledis;

LSM6DS3 imu(I2C_MODE, 0x6A);

// ---------------- packet ----------------
struct __attribute__((packed)) Pkt {
  uint32_t t0_us;      // micros() when the first sample in this packet was read
  uint16_t seq;        // increments every packet (gaps = dropped packets)
  uint8_t  n;          // samples in this packet
  uint8_t  odr_div8;   // ODR / 8  (52 = 416 Hz, 26 = 208 Hz)
  int16_t  s[SAMPLES_PER_PKT][6];   // raw gx,gy,gz,ax,ay,az
};
static Pkt pkt;

// stats for the serial monitor
static uint32_t statSamples = 0, statNotifyFail = 0, statLastMs = 0;

// ---------------- BLE callbacks ----------------
void onConnect(uint16_t conn_handle) {
  BLEConnection* conn = Bluefruit.Connection(conn_handle);
  conn->requestPHY();                 // 2M PHY if the central supports it
  conn->requestDataLengthUpdate();
  conn->requestMtuExchange(247);
  Serial.println("BLE connected");
}

void onDisconnect(uint16_t conn_handle, uint8_t reason) {
  (void)conn_handle;
  Serial.print("BLE disconnected, reason 0x");
  Serial.println(reason, HEX);
}

// ---------------- setup ----------------
// Undo wake-on-motion settings left over from the last sleep.
void clearImuWakeConfig() {
  imu.writeRegister(REG_MD1_CFG, 0x00);
  imu.writeRegister(REG_TAP_CFG, 0x00);
  imu.writeRegister(REG_CTRL6_C, 0x00);
  imu.writeRegister(REG_WAKE_UP_THS, 0x00);
  uint8_t junk;
  imu.readRegister(&junk, REG_WAKE_UP_SRC);
}

void configureImu() {
  clearImuWakeConfig();   // in case we just woke from sleep
  uint8_t who = 0;
  imu.readRegister(&who, REG_WHO_AM_I);
  Serial.print("IMU WHO_AM_I = 0x");
  Serial.println(who, HEX);          // expect 0x6A

  imu.writeRegister(REG_CTRL3_C, 0x44);                     // BDU + auto-increment
  imu.writeRegister(REG_FIFO_CTRL5, 0x00);                  // FIFO bypass
  imu.writeRegister(REG_CTRL1_XL, ODR_BITS | XL_FS_16G);
  imu.writeRegister(REG_CTRL2_G,  ODR_BITS | G_FS_2000DPS);
}

void setupBle() {
  Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);   // larger MTU + buffers
  Bluefruit.begin();
  Bluefruit.setTxPower(4);
  Bluefruit.setName("FootPod-" POD_SIDE);
  Bluefruit.Periph.setConnectCallback(onConnect);
  Bluefruit.Periph.setDisconnectCallback(onDisconnect);
  Bluefruit.Periph.setConnInterval(6, 12);        // 7.5–15 ms

  bledis.setManufacturer("DIY");
  bledis.setModel("FootPod capture v1");
  bledis.begin();

  podSvc.begin();
  imuChr.setProperties(CHR_PROPS_NOTIFY);
  imuChr.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  imuChr.setMaxLen(sizeof(Pkt));
  imuChr.begin();

  statChr.setProperties(CHR_PROPS_READ | CHR_PROPS_NOTIFY);
  statChr.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  statChr.setFixedLen(sizeof(Stat));
  statChr.begin();

  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addService(podSvc);
  Bluefruit.ScanResponse.addName();
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);
  Bluefruit.Advertising.setFastTimeout(30);
  Bluefruit.Advertising.start(0);
}

// ---------------- sleep ----------------
bool onUsbPower() {
  uint32_t status = 0;
  sd_power_usbregstatus_get(&status);
  return status & POWER_USBREGSTATUS_VBUSDETECT_Msk;
}

void ledsOff() {
  digitalWrite(LED_RED, HIGH); digitalWrite(LED_GREEN, HIGH); digitalWrite(LED_BLUE, HIGH);
}

void goToSleep() {
  Serial.println("Going to sleep");
  Bluefruit.autoConnLed(false);   // stop the library driving the blue LED (it gets the polarity wrong)
  pinMode(LED_RED, OUTPUT); pinMode(LED_GREEN, OUTPUT); pinMode(LED_BLUE, OUTPUT);
  for (int i = 0; i < 3; i++) {               // three purple blinks
    digitalWrite(LED_RED, LOW); digitalWrite(LED_BLUE, LOW); delay(150);
    ledsOff(); delay(150);
  }
  Bluefruit.Advertising.restartOnDisconnect(false);
  Bluefruit.Advertising.stop();
  if (Bluefruit.connected()) Bluefruit.disconnect(Bluefruit.connHandle());
  delay(100);

  // IMU: gyro off, accel 26 Hz low-power, +-2 g, wake-up interrupt on INT1.
  imu.writeRegister(REG_CTRL2_G, 0x00);
  imu.writeRegister(REG_CTRL1_XL, 0x20);
  imu.writeRegister(REG_CTRL6_C, 0x10);              // accel high-performance off
  imu.writeRegister(REG_WAKE_UP_DUR, 0x00);
  imu.writeRegister(REG_WAKE_UP_THS, WAKE_THRESHOLD & 0x3F);
  imu.writeRegister(REG_TAP_CFG, 0x80);              // interrupts on, slope filter (no settling time)
  imu.writeRegister(REG_INT1_CTRL, 0x00);            // nothing else on INT1
  imu.writeRegister(REG_MD1_CFG, 0x20);              // wake-up -> INT1
  delay(500);                                        // a dozen samples at 26 Hz
  uint8_t junk;
  pinMode(PIN_LSM6DS3TR_C_INT1, INPUT);
  for (int i = 0; i < 20; i++) {                     // wait for INT1 to be quiet
    imu.readRegister(&junk, REG_WAKE_UP_SRC);        // clear any pending event
    if (digitalRead(PIN_LSM6DS3TR_C_INT1) == LOW) break;
    delay(50);
  }

  // LEDs off and undriven: pin states are kept in System OFF, so a driven-low
  // LED pin would stay lit all night.
  ledsOff();
  pinMode(LED_RED, INPUT); pinMode(LED_GREEN, INPUT); pinMode(LED_BLUE, INPUT);

  // Remember that we slept, so the next boot knows it was a wake-up.
  uint8_t mark = SLEEP_MARK | (uint8_t)min((int)sleepCount + 1, 63);
  sd_power_gpregret_clr(1, 0xFF);
  sd_power_gpregret_set(1, mark);

  // The IMU power pin keeps its state in System OFF, so the sensor stays on.
  // INT1 going HIGH wakes the board, which restarts from setup().
  systemOff(PIN_LSM6DS3TR_C_INT1, HIGH);
}

void checkSleep(uint32_t ms, const int16_t* v) {
  float ax = v[3] * 0.000488f, ay = v[4] * 0.000488f, az = v[5] * 0.000488f;
  float g[3] = { v[0] * 0.070f, v[1] * 0.070f, v[2] * 0.070f };
  if (!sleepFilterInit) {
    gravX = ax; gravY = ay; gravZ = az;
    for (int k = 0; k < 3; k++) gyroBias[k] = g[k];
    sleepFilterInit = true;
  }
  const float a = 1.0f / IMU_ODR_HZ;                 // ~1 s time constant
  gravX += a * (ax - gravX); gravY += a * (ay - gravY); gravZ += a * (az - gravZ);
  float dx = ax - gravX, dy = ay - gravY, dz = az - gravZ;
  bool rotating = false;
  for (int k = 0; k < 3; k++) {
    gyroBias[k] += a * (g[k] - gyroBias[k]);
    if (fabsf(g[k] - gyroBias[k]) > MOTION_DPS) rotating = true;
  }
  if (dx * dx + dy * dy + dz * dz > MOTION_G * MOTION_G || rotating) lastActivityMs = ms;
}

void maybeSleep(uint32_t ms) {
  static uint32_t lastCheck = 0;
  if (ms - lastCheck < 1000) return;
  lastCheck = ms;
  if (onUsbPower()) return;                          // never sleep on USB (still-time keeps counting)
  if (ms - lastActivityMs > SLEEP_IDLE_MS) goToSleep();
}

void setup() {
  uint32_t resetReason = readResetReason();          // saved by the core at boot (often cleared by the bootloader)
  uint8_t mark = NRF_POWER->GPREGRET2;               // SoftDevice not running yet, direct access is fine
  NRF_POWER->GPREGRET2 = 0;
  wokeFromSleep = (mark & SLEEP_MARK) || (resetReason & POWER_RESETREAS_OFF_Msk);
  sleepCount = (mark & SLEEP_MARK) ? (mark & 0x3F) : 0;

  Serial.begin(115200);
  delay(500);
  if (wokeFromSleep) {                               // two green blinks = woke from sleep
    pinMode(LED_GREEN, OUTPUT);
    for (int i = 0; i < 2; i++) { digitalWrite(LED_GREEN, LOW); delay(150); digitalWrite(LED_GREEN, HIGH); delay(150); }
  }
  if (wokeFromSleep) { Serial.print("Woke from sleep, sleeps since power-on: "); Serial.println(sleepCount); }
  else Serial.println("Power-on / reset");

#ifdef PIN_LSM6DS3TR_C_POWER
  pinMode(PIN_LSM6DS3TR_C_POWER, OUTPUT);   // the Sense board switches IMU power
  digitalWrite(PIN_LSM6DS3TR_C_POWER, HIGH);
  delay(20);
#endif

  if (imu.begin() != 0) {
    Serial.println("IMU init FAILED — check board selection (XIAO nRF52840 Sense)");
  }
  configureImu();   // overrides the library's default ranges/rates

  Wire.setClock(400000);
#if defined(WIRE_INTERFACES_COUNT) && (WIRE_INTERFACES_COUNT > 1)
  Wire1.setClock(400000);   // onboard IMU may sit on the second bus
#endif

  pkt.n = 0;
  pkt.seq = 0;
  pkt.odr_div8 = IMU_ODR_HZ / 8;

  setupBle();
  lastActivityMs = millis();
  Serial.println("FootPod-" POD_SIDE " ready, advertising");
}

// ---------------- loop ----------------
void loop() {
  uint8_t status = 0;
  imu.readRegister(&status, REG_STATUS);
  if (status & 0x01) {                       // new accel sample (gyro shares the ODR)
    uint32_t now = micros();
    uint8_t raw[12];
    imu.readRegisterRegion(raw, REG_OUTX_L_G, 12);

    if (pkt.n == 0) pkt.t0_us = now;
    memcpy(pkt.s[pkt.n], raw, 12);           // nRF52 is little-endian, same as the IMU
    checkSleep(millis(), pkt.s[pkt.n]);
    statSamples++;

    if (++pkt.n == SAMPLES_PER_PKT) {
      if (Bluefruit.connected() && imuChr.notifyEnabled()) {
        if (!imuChr.notify(&pkt, sizeof(pkt))) statNotifyFail++;
      }
      pkt.seq++;
      pkt.n = 0;
    }
  }

  uint32_t ms = millis();
  if (Serial.available() && Serial.read() == 'q') goToSleep();
  maybeSleep(ms);
  if (ms - statLastMs >= 1000) {
    Serial.print("samples/s: ");
    Serial.print(statSamples);
    Serial.print("  notify fails: ");
    Serial.print(statNotifyFail);
    Serial.print("  usb: ");
    Serial.print(onUsbPower() ? "yes" : "no");
    Serial.print("  connected: ");
    Serial.print(Bluefruit.connected() ? "yes" : "no");
    Serial.print("  still for: ");
    Serial.print((ms - lastActivityMs) / 1000);
    Serial.print(" s (sleeps at ");
    Serial.print(SLEEP_IDLE_MS / 1000);
    Serial.println(" s on battery)");
    statSamples = 0;

    Stat st;
    st.uptime_s = ms / 1000;
    st.still_s  = (uint16_t)min((ms - lastActivityMs) / 1000, (uint32_t)65535);
    st.limit_s  = SLEEP_IDLE_MS / 1000;
    st.flags    = (onUsbPower() ? 1 : 0) | (wokeFromSleep ? 2 : 0) | (Bluefruit.connected() ? 4 : 0);
    st.version  = 3;
    st.sleeps   = sleepCount;
    statChr.write(&st, sizeof(st));
    if (Bluefruit.connected() && statChr.notifyEnabled()) statChr.notify(&st, sizeof(st));
    statLastMs = ms;
  }
}
