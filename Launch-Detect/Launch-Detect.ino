
#include <Wire.h>

typedef int8_t i8;
typedef int16_t i16;
typedef int32_t i32;
typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;

typedef float f32;

#define LSM6DSOX_ADDRESS 0x6A

#define LSM6DSOX_WHO_AM_I_REG 0X0F
#define LSM6DSOX_CTRL1_XL 0X10
#define LSM6DSOX_CTRL2_G 0X11

#define LSM6DSOX_STATUS_REG 0X1E

#define LSM6DSOX_OUTX_L_XL 0X28
#define LSM6DSOX_OUTX_H_XL 0X29
#define LSM6DSOX_OUTY_L_XL 0X2A
#define LSM6DSOX_OUTY_H_XL 0X2B
#define LSM6DSOX_OUTZ_L_XL 0X2C
#define LSM6DSOX_OUTZ_H_XL 0X2D

#define ACCEL_AVG_FACTOR 0.9f

bool in_flight = false;
f32 accel_mag_avg = 0.0f;
u32 flight_start = 0;

static int accel_read_registers(uint8_t address, uint8_t *data, size_t length);
static int accel_write_register(uint8_t address, uint8_t value);

struct {
  i16 x, y, z;
} accel = { 0 };

void setup() {
  Serial.begin();
  while (!Serial);

  Wire.begin();
  Wire.setClock(1000000);
  // Sets accelerometer frequency
  accel_write_register(LSM6DSOX_CTRL1_XL, 0b01011110);

  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  accel_read_registers(
    LSM6DSOX_OUTX_L_XL,
    (uint8_t *)&accel,
    6
  );

  digitalWrite(LED_BUILTIN, in_flight);

  if (!in_flight) {
    f32 accel_mag = sqrt(
      (f32)accel.x * (f32)accel.x +
      (f32)accel.y * (f32)accel.y +
      (f32)accel.z * (f32)accel.z
    ) * (8.0f / (f32)(1 << 15));
    accel_mag_avg = (ACCEL_AVG_FACTOR) * accel_mag_avg + (1.0f - ACCEL_AVG_FACTOR) * accel_mag;

    Serial.printf("0,12,7.5,%f,%f\n", accel_mag, accel_mag_avg);

    if (accel_mag_avg > 7.5f) {
      in_flight = true;
      flight_start = millis();
    }
  } else {
    if (millis() - flight_start > 5000) {
      in_flight = false;
      flight_start = 0;
      accel_mag_avg = 0.0f;
    }
  }

  delayMicroseconds(5642);
}

static int accel_read_registers(uint8_t address, uint8_t *data, size_t length) {
    Wire.beginTransmission(LSM6DSOX_ADDRESS);
    Wire.write(address);

    if (Wire.endTransmission(false) != 0) {
        return -1;
    }

    if (Wire.requestFrom(LSM6DSOX_ADDRESS, length) != length) {
        return 0;
    }

    for (size_t i = 0; i < length; i++) {
        *data++ = Wire.read();
    }

    return 1;
}

static int accel_write_register(uint8_t address, uint8_t value) {
    Wire.beginTransmission(LSM6DSOX_ADDRESS);
    Wire.write(address);
    Wire.write(value);
    if (Wire.endTransmission() != 0) {
        return 0;
    }

    return 1;
}

