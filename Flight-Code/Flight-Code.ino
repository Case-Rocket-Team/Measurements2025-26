
#include <SPI.h>
#include <SD.h>
#include <Wire.h>

#define WRITE_FILE 0

#define TEST_MODE 1

#if TEST_MODE
#    define PAD_RECORD_TIME_MS (10 * 1000)
#    define FLIGHT_RECORD_TIME_MS (30 * 1000)
#    define LAUNCH_THRESHOLD 4.0f
#    define LAUNCH_FILTER 0.9f
#else
#    define PAD_RECORD_TIME_MS (30 * 1000)
#    define FLIGHT_RECORD_TIME_MS (10 * 60 * 1000)
#    define LAUNCH_THRESHOLD 7.5f
#    define LAUNCH_FILTER 0.9f
#endif

typedef int8_t i8;
typedef int16_t i16;
typedef int32_t i32;
typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;

typedef float f32;

typedef struct {
  u8* str;
  u32 size;
} string8;

#define SPI1_CIPO_PIN 28
#define SPI1_COPI_PIN 27
#define SPI1_SCK_PIN 26

#define ADS0_CS_PIN 21
#define ADS1_CS_PIN 5
#define ADS0_DRDY_PIN 19
#define ADS1_DRDY_PIN 20

enum class ads_cmd : u8 { 
  WAKEUP   = 0x00,
  RDATA    = 0x01,
  RDATAC   = 0x03,
  SDATAC   = 0x0f,
  RREG     = 0x10, // Lower four specify register
  WREG     = 0x50, // Lower four specify register
  SELFCAL  = 0xf0,
  SELFOCAL = 0xf1,
  SELFGCAL = 0xf2,
  SYSOCAL  = 0xf3,
  SYSGCAL  = 0xf4,
  SYNC     = 0xfc,
  STANDBY  = 0xfd,
  RESET    = 0xfe,
  WAKEUP_D = 0xff
};

enum class ads_reg : u8 {
  STATUS = 0x0,
  MUX    = 0x1,
  ADCON  = 0x2,
  DRATE  = 0x3,
  IO     = 0x4,
  OFC0   = 0x5,
  OFC1   = 0x6,
  OFC2   = 0x7,
  FSC0   = 0x8,
  FSC1   = 0x9,
  FSC2   = 0xa,
};

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

const SPISettings ads_settings(1920000, MSBFIRST, SPI_MODE1);

#define ADS_PGA 0b001
#define ADS_GAIN (1 << ADS_PGA)

const float ads_conversion = (1.0f / ADS_GAIN) * 5.0f / 32768.0f;

#define DATA_BUF_SIZE 128

#define ADS0_NUM_GAUGES 8
#define ADS1_NUM_GAUGES 6

#define ADS0_CYCLES_PER_SAMPLE 3
#define ADS1_CYCLES_PER_SAMPLE 4

static_assert(
  ADS0_NUM_GAUGES * ADS0_CYCLES_PER_SAMPLE ==
  ADS1_NUM_GAUGES * ADS1_CYCLES_PER_SAMPLE
);

#pragma pack(push, 1)
union accel_sample {
  struct {
    i16 x;
    i16 y;
    i16 z;
  };
  i16 v[3];
};

struct sample {
  u32 time;
  i16 measures0[ADS0_NUM_GAUGES * ADS0_CYCLES_PER_SAMPLE];
  i16 measures1[ADS1_NUM_GAUGES * ADS1_CYCLES_PER_SAMPLE];
  accel_sample accel;
};
#pragma pack(pop)

// Used to swap the front and back buffers
sample* tmp_buf = NULL;
sample* data_front_buf = NULL;
sample* data_back_buf = NULL;

/*
 /$$$$$$$$ /$$                       /$$            /$$$$$$                               
| $$_____/|__/                      | $$           /$$__  $$                              
| $$       /$$  /$$$$$$   /$$$$$$$ /$$$$$$        | $$  \__/  /$$$$$$   /$$$$$$   /$$$$$$ 
| $$$$$   | $$ /$$__  $$ /$$_____/|_  $$_/        | $$       /$$__  $$ /$$__  $$ /$$__  $$
| $$__/   | $$| $$  \__/|  $$$$$$   | $$          | $$      | $$  \ $$| $$  \__/| $$$$$$$$
| $$      | $$| $$       \____  $$  | $$ /$$      | $$    $$| $$  | $$| $$      | $$_____/
| $$      | $$| $$       /$$$$$$$/  |  $$$$/      |  $$$$$$/|  $$$$$$/| $$      |  $$$$$$$
|__/      |__/|__/      |_______/    \___/         \______/  \______/ |__/       \_______/
*/

enum class mes_types : u8 {
  I8 = 0,
  I16 = 1,
  I32 = 2,
  U8 = 3,
  U16 = 4,
  U32 = 5,
  F32 = 6
};

#define FLASH_CS 17

#define SWITCH_PIN 25

#define FILE_INDEX_DIGITS 4
#define CUR_FILE_NAME "03-21-26-testing"
#define CUR_FILE_NAME_LEN (sizeof(CUR_FILE_NAME) - 1)
#define FILE_EXT ".mes"
#define FILE_EXT_LEN (sizeof(FILE_EXT) - 1)
#define FILE_NAME_SIZE (CUR_FILE_NAME_LEN + FILE_INDEX_DIGITS + FILE_EXT_LEN)

void find_file_name(char file_name[FILE_NAME_SIZE]);

void write_output_header(File& out_file);

File out_file0;
File out_file1;
u32 num_samples = 0;

bool in_flight = false;
f32 accel_mag_avg = 0.0f;
u32 flight_start = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial);

  pinMode(SWITCH_PIN, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  digitalWrite(LED_BUILTIN, digitalRead(SWITCH_PIN));

  if (digitalRead(SWITCH_PIN) == 0) {
    Serial.println("Switch is turned off, stopping now");
    while (1);
  }

#if WRITE_FILE
  Serial.println("Initializing SD card...");
  if (!SD.begin(FLASH_CS, SPI_FULL_SPEED)) {
    Serial.println("SD initialization failed!");
    while (1);
  }
  Serial.println("SD initialization done.");

  char file_name[FILE_NAME_SIZE + 1] = { 0 };
  find_file_name(file_name);

  Serial.printf("Creating file '%s'\n", file_name);

  out_file0 = SD.open(file_name, FILE_WRITE);
  if (!out_file0) {
    Serial.println("Failed to create output file 0");
    while (1);
  }

  write_output_header(out_file0);
#endif
}

void loop() {
  if (rp2040.fifo.available()) {
    rp2040.fifo.pop();

    num_samples += DATA_BUF_SIZE;

    u32 write_start = micros();

#if WRITE_FILE
    u32 written = out_file0.write((u8*)(data_front_buf), sizeof(sample) * DATA_BUF_SIZE);
    out_file0.flush();
    Serial.printf("%d written (%u expected) | ", written, sizeof(sample) * DATA_BUF_SIZE);
#endif

    Serial.printf("Write took %u us\n", micros() - write_start);

    if (digitalRead(SWITCH_PIN) == 0) {
#if WRITE_FILE
      u32 written = out_file0.write((u8*)&num_samples, 4);
      Serial.printf("%d written (%u expected)\n", written, 4);
      out_file0.close();
#endif

      Serial.println("Switch is turned off, stopping now");
      while (1);
    }

    f32 averages0[ADS0_NUM_GAUGES] = { 0 };
    f32 averages1[ADS1_NUM_GAUGES] = { 0 };

    for (u32 i = 0; i < DATA_BUF_SIZE; i++) {
      for (u32 j = 0; j < ADS0_NUM_GAUGES; j++) {
        averages0[j] += (f32)data_front_buf[i].measures0[j] * ads_conversion;
      }

      for (u32 j = 0; j < ADS1_NUM_GAUGES; j++) {
        averages1[j] += (f32)data_front_buf[i].measures1[j] * ads_conversion;
      }
    }

    Serial.printf("%d,%u,", in_flight, data_front_buf[0].time);
    Serial.printf(",%d,%d,%d,", data_front_buf[0].accel.x, data_front_buf[0].accel.y, data_front_buf[0].accel.z);
    for (u32 i = 0; i < ADS0_NUM_GAUGES; i++) {
      averages0[i] /= (f32)DATA_BUF_SIZE;
      Serial.printf(",%.2f", averages0[i]);
    }
    for (u32 i = 0; i < ADS1_NUM_GAUGES; i++) {
      averages1[i] /= (f32)DATA_BUF_SIZE;
      Serial.printf(",%.2f", averages1[i]);
    }
    Serial.println("");
  }
}

void find_file_name(char file_name[FILE_NAME_SIZE + 1]) {
  i32 file_index = -1;

  File entry;
  File root = SD.open("/");

  while ((entry = root.openNextFile())) {
    const char* name = entry.name();
    u32 len = strlen(name);

    if (len < CUR_FILE_NAME_LEN + FILE_INDEX_DIGITS) {
      continue;
    }

    bool same_name = true;
    for (u32 i = 0; i < CUR_FILE_NAME_LEN; i++) {
      if (name[i] != CUR_FILE_NAME[i]) {
        same_name = false;
        break;
      }
    }

    if (!same_name) { continue; }

    i32 index = 0;
    for (u32 i = 0; i < FILE_INDEX_DIGITS; i++) {
      index *= 10;
      index += name[i + CUR_FILE_NAME_LEN] - '0';
    }

    if (index > file_index) {
      file_index = index;
    }

    entry.close();
  }
  root.close();
  file_index++;

  memcpy(file_name, CUR_FILE_NAME, CUR_FILE_NAME_LEN);

  for (i32 i = 0; i < FILE_INDEX_DIGITS; i++) {
    i32 index = CUR_FILE_NAME_LEN + FILE_INDEX_DIGITS - i - 1;

    file_name[index] = file_index % 10 + '0';
    file_index /= 10;
  }

  memcpy(
    file_name + CUR_FILE_NAME_LEN + FILE_INDEX_DIGITS,
    FILE_EXT, FILE_EXT_LEN
  );
}

void write_output_header(File& out_file) {
  u8 num_fields = 1 + 
    ADS0_NUM_GAUGES * ADS0_CYCLES_PER_SAMPLE +
    ADS1_NUM_GAUGES * ADS1_CYCLES_PER_SAMPLE + 3;

  u8 header[4] = { 'M', 'E', 'S', num_fields };
  out_file.write(header, 4);

  char gauge_name[] = {
    'a', 'd', 's', '_',
    'g', 'a', 'u', 'g', 'e', '_', 
    'c', 'y', 'c', 'l', 'e', '_'
  };

  u8* field_header = header;

  // Time field
  {
    *(u16*)(field_header) = 1;
    field_header[2] = (u8)mes_types::U32;
    field_header[3] = sizeof("time") - 1;

    out_file.write(field_header, 4);
    out_file.write("time", 4);
  }

  *(u16*)(field_header) = 1;
  field_header[2] = (u8)mes_types::I16;
  field_header[3] = sizeof(gauge_name);

  gauge_name[3] = '0';
  for (u8 c = 0; c < ADS0_CYCLES_PER_SAMPLE; c++) {
    gauge_name[15] = c + '0';
    for (u8 g = 0; g < ADS0_NUM_GAUGES; g++) {
      gauge_name[9] = g + '0';

      out_file.write(field_header, 4);
      out_file.write(gauge_name, sizeof(gauge_name));
    }
  }

  gauge_name[3] = '1';
  for (u8 c = 0; c < ADS1_CYCLES_PER_SAMPLE; c++) {
    gauge_name[15] = c + '0';
    for (u8 g = 0; g < ADS1_NUM_GAUGES; g++) {
      gauge_name[9] = g + '0';

      out_file.write(field_header, 4);
      out_file.write(gauge_name, sizeof(gauge_name));
    }
  }

  *(u16*)(field_header) = 1;
  field_header[2] = (u8)mes_types::I16;
  field_header[3] = sizeof("accel_x") - 1;
  out_file.write(field_header, 4);
  out_file.write("accel_x", sizeof("accel_x") - 1);

  *(u16*)(field_header) = 1;
  field_header[2] = (u8)mes_types::I16;
  field_header[3] = sizeof("accel_y") - 1;
  out_file.write(field_header, 4);
  out_file.write("accel_y", sizeof("accel_y") - 1);

  *(u16*)(field_header) = 1;
  field_header[2] = (u8)mes_types::I16;
  field_header[3] = sizeof("accel_z") - 1;
  out_file.write(field_header, 4);
  out_file.write("accel_z", sizeof("accel_z") - 1);

  out_file.flush();
}

/*
  /$$$$$$                                                /$$        /$$$$$$                               
 /$$__  $$                                              | $$       /$$__  $$                              
| $$  \__/  /$$$$$$   /$$$$$$$  /$$$$$$  /$$$$$$$   /$$$$$$$      | $$  \__/  /$$$$$$   /$$$$$$   /$$$$$$ 
|  $$$$$$  /$$__  $$ /$$_____/ /$$__  $$| $$__  $$ /$$__  $$      | $$       /$$__  $$ /$$__  $$ /$$__  $$
 \____  $$| $$$$$$$$| $$      | $$  \ $$| $$  \ $$| $$  | $$      | $$      | $$  \ $$| $$  \__/| $$$$$$$$
 /$$  \ $$| $$_____/| $$      | $$  | $$| $$  | $$| $$  | $$      | $$    $$| $$  | $$| $$      | $$_____/
|  $$$$$$/|  $$$$$$$|  $$$$$$$|  $$$$$$/| $$  | $$|  $$$$$$$      |  $$$$$$/|  $$$$$$/| $$      |  $$$$$$$
 \______/  \_______/ \_______/ \______/ |__/  |__/ \_______/       \______/  \______/ |__/       \_______/
*/

// Sets up datarate, multiplexer, gain, etc.
static void ads_init(u8 ads_cs_pin);

static u8 ads_read_reg(u8 ads_cs_pin, ads_reg reg);
static void ads_write_reg(u8 ads_cs_pin, ads_reg reg, u8 val);
static void ads_send_cmd(u8 ads_cs_pin, ads_cmd cmd);

static int accel_read_registers(uint8_t address, uint8_t *data, size_t length);

void setup1() {
  data_front_buf = (sample*)malloc(sizeof(sample) * DATA_BUF_SIZE);
  data_back_buf = (sample*)malloc(sizeof(sample) * DATA_BUF_SIZE);
  memset(data_front_buf, 0, sizeof(sample) * DATA_BUF_SIZE);
  memset(data_back_buf, 0, sizeof(sample) * DATA_BUF_SIZE);

  pinMode(SPI1_CIPO_PIN, INPUT);
  pinMode(SPI1_COPI_PIN, OUTPUT);
  pinMode(SPI1_SCK_PIN, OUTPUT);

  SPI1.setMISO(SPI1_CIPO_PIN);
  SPI1.setMOSI(SPI1_COPI_PIN);
  SPI1.setSCK(SPI1_SCK_PIN);

  pinMode(ADS0_CS_PIN, OUTPUT);
  pinMode(ADS0_DRDY_PIN, INPUT);
  pinMode(ADS1_CS_PIN, OUTPUT);
  pinMode(ADS1_DRDY_PIN, INPUT);

  digitalWrite(ADS0_CS_PIN, HIGH);
  digitalWrite(ADS1_CS_PIN, HIGH);

  SPI1.begin();

  ads_init(ADS0_CS_PIN);
  ads_init(ADS1_CS_PIN);

  Wire.begin();
  Wire.setClock(1000000);
  // Sets accelerometer frequency
  accel_write_register(LSM6DSOX_CTRL1_XL, 0b01011110);
}

u32 data_buf_pos = 0;

u8 ads0_mux_pin = 0;
u8 ads1_mux_pin = 0;
u8 ads0_cycle = 0;
u8 ads1_cycle = 0;

void loop1() {
  u8 ads0_read_pin = ads0_mux_pin;
  ads0_mux_pin = (ads0_mux_pin + 1) % ADS0_NUM_GAUGES;

  u8 ads1_read_pin = ads1_mux_pin;
  ads1_mux_pin = (ads1_mux_pin + 1) % ADS1_NUM_GAUGES;

  i16 measure0 = 0;
  i16 measure1 = 0;

  // ADS0 measure
  {
    // Wait for DRDY  
    while (digitalRead(ADS0_DRDY_PIN) == HIGH);

    SPI1.beginTransaction(ads_settings);
    digitalWrite(ADS0_CS_PIN, LOW);

    SPI1.transfer((u8)ads_cmd::WREG | (u8)ads_reg::MUX);
    SPI1.transfer(0x00);
    SPI1.transfer((ads0_mux_pin << 4) | 0b1000);
    SPI1.transfer((u8)ads_cmd::SYNC);
    // Rounded up from 3.125
    delayMicroseconds(4);
    SPI1.transfer((u8)ads_cmd::WAKEUP);
    SPI1.transfer((u8)ads_cmd::RDATA);
    // Rounded up from 6.510
    delayMicroseconds(7);

    measure0 |= (i16)SPI1.transfer(0) << 8;
    measure0 |= (i16)SPI1.transfer(0);
    (void)SPI1.transfer(0);

    digitalWrite(ADS0_CS_PIN, HIGH);
    SPI1.endTransaction();
  }

  // ADS1 measure
  {
    // Wait for DRDY  
    while (digitalRead(ADS1_DRDY_PIN) == HIGH);

    SPI1.beginTransaction(ads_settings);
    digitalWrite(ADS1_CS_PIN, LOW);

    SPI1.transfer((u8)ads_cmd::WREG | (u8)ads_reg::MUX);
    SPI1.transfer(0x00);
    SPI1.transfer((ads1_mux_pin << 4) | 0b1000);
    SPI1.transfer((u8)ads_cmd::SYNC);
    // Rounded up from 3.125
    delayMicroseconds(4);
    SPI1.transfer((u8)ads_cmd::WAKEUP);
    SPI1.transfer((u8)ads_cmd::RDATA);
    // Rounded up from 6.510
    delayMicroseconds(7);

    measure1 |= (i16)SPI1.transfer(0) << 8;
    measure1 |= (i16)SPI1.transfer(0);
    (void)SPI1.transfer(0);

    digitalWrite(ADS1_CS_PIN, HIGH);
    SPI1.endTransaction();
  }

  // Checking for new sample
  if (
    ads0_read_pin == 0 && ads1_read_pin == 0 &&
    ads0_cycle == 0 && ads1_cycle == 0
  ) {
    data_back_buf[data_buf_pos++] = { 0 };
    data_back_buf[data_buf_pos-1].time = micros();

    accel_sample* accel = &data_back_buf[data_buf_pos-1].accel;

    accel_read_registers(LSM6DSOX_OUTX_L_XL, (uint8_t *)accel, 6);

    f32 accel_mag = sqrt(
      (f32)accel->x * (f32)accel->x +
      (f32)accel->y * (f32)accel->y +
      (f32)accel->z * (f32)accel->z
    ) * (8.0f / (f32)(1 << 15));

    accel_mag_avg = (LAUNCH_FILTER) * accel_mag_avg + (1.0f - LAUNCH_FILTER) * accel_mag;
    if (accel_mag_avg > LAUNCH_THRESHOLD) {
      in_flight = true;
      accel_mag_avg = 0;
      flight_start = millis();
    }
  }

  // Writing measures
  data_back_buf[data_buf_pos-1].measures0[
    ads0_cycle * ADS0_NUM_GAUGES + ads0_read_pin
  ] = measure0;
  data_back_buf[data_buf_pos-1].measures1[
    ads1_cycle * ADS1_NUM_GAUGES + ads1_read_pin
  ] = measure1;

  // Updating cycles
  if (ads0_mux_pin == 0) { ads0_cycle++; }
  if (ads1_mux_pin == 0) { ads1_cycle++; }

  // Check for buffer swap
  if (
    data_buf_pos >= DATA_BUF_SIZE && 
    ads0_cycle == ADS0_CYCLES_PER_SAMPLE &&
    ads1_cycle == ADS1_CYCLES_PER_SAMPLE
  ) {
    tmp_buf = data_front_buf;
    data_front_buf = data_back_buf;
    data_back_buf = tmp_buf;

    data_buf_pos = 0;

    rp2040.fifo.push(1);
  }

  // Wrapping cycles
  ads0_cycle %= ADS0_CYCLES_PER_SAMPLE;
  ads1_cycle %= ADS1_CYCLES_PER_SAMPLE;
}

static void ads_init(u8 ads_cs_pin) {
  ads_send_cmd(ads_cs_pin, ads_cmd::RESET);
  delay(100);
  ads_write_reg(ads_cs_pin, ads_reg::MUX, 0b00001000);
  delay(100);
  ads_write_reg(ads_cs_pin, ads_reg::ADCON, 0b00100000 | ADS_PGA);
  delay(100);
  ads_send_cmd(ads_cs_pin, ads_cmd::SELFCAL);
  delay(200);
}

static u8 ads_read_reg(u8 ads_cs_pin, ads_reg reg) {
  SPI1.beginTransaction(ads_settings);

  digitalWrite(ads_cs_pin, LOW);

  SPI1.transfer((u8)ads_cmd::RREG | (u8)reg);
  SPI1.transfer(0x00);

  delayMicroseconds(5);

  u8 out = SPI1.transfer(0xff);

  digitalWrite(ads_cs_pin, HIGH);

  SPI1.endTransaction();

  return out;
}

static void ads_write_reg(u8 ads_cs_pin, ads_reg reg, u8 val) {
  SPI1.beginTransaction(ads_settings);

  digitalWrite(ads_cs_pin, LOW);

  SPI1.transfer((u8)ads_cmd::WREG | (u8)reg);
  SPI1.transfer(0x00);
  SPI1.transfer(val);

  digitalWrite(ads_cs_pin, HIGH);

  SPI1.endTransaction();
}

static void ads_send_cmd(u8 ads_cs_pin, ads_cmd cmd) {
  SPI1.beginTransaction(ads_settings);

  digitalWrite(ads_cs_pin, LOW);

  // TODO: Test if these delays are really necessary
  delayMicroseconds(5);
  SPI1.transfer((u8)cmd);
  delayMicroseconds(5);

  digitalWrite(ads_cs_pin, HIGH);

  SPI1.endTransaction();
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
