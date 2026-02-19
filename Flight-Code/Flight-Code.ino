
#include <SPI.h>
#include <SD.h>

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

#define CIPO_PIN 28
#define COPI_PIN 27
#define SCK_PIN 26

#define ADS_CS_PIN 21
#define ADS_DRDY_PIN 19

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

#define DATA_BUF_SIZE 1024

#define NUM_STRAIN_GAUGES 2

struct sample {
  u32 time;
  i16 measures[NUM_STRAIN_GAUGES];
};

// Used to swap the front and back buffers
sample* tmp_buf = NULL;
sample* data_front_buf = NULL;
sample* data_back_buf = NULL;

const SPISettings ads_settings(1920000, MSBFIRST, SPI_MODE1);

#define ADS_PGA 0b001
#define ADS_GAIN (1 << ADS_PGA)

const float ads_conversion = (1.0f / ADS_GAIN) * 5.0f / 32768.0f;

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
#define CUR_FILE_NAME "02-19-26-testing"
#define CUR_FILE_NAME_LEN (sizeof(CUR_FILE_NAME) - 1)
#define FILE_EXT ".mes"
#define FILE_EXT_LEN (sizeof(FILE_EXT) - 1)
#define FILE_NAME_SIZE (CUR_FILE_NAME_LEN + FILE_INDEX_DIGITS + FILE_EXT_LEN)

void find_file_name(char file_name[FILE_NAME_SIZE]);

void write_output_header(File& out_file);

File output_file;
u8 pin_reads = 0xff;
u32 num_samples = 0;
u32 last_time = 0;

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

  Serial.println("Initializing SD card...");
  if (!SD.begin(FLASH_CS, SPI_FULL_SPEED)) {
    Serial.println("SD initialization failed!");
    while (1);
  }
  Serial.println("SD initialization done.");

  char file_name[FILE_NAME_SIZE + 1] = { 0 };
  find_file_name(file_name);

  Serial.printf("Creating file '%s'\n", file_name);

  output_file = SD.open(file_name, FILE_WRITE);
  if (!output_file) {
    Serial.println("Failed to create output file");
    while (1);
  }
  write_output_header(output_file);

  last_time = micros();
}

void loop() {
  while (rp2040.fifo.available()) {
    rp2040.fifo.pop();

    num_samples += DATA_BUF_SIZE;
    u32 write_start = micros();

    u32 written = output_file.write((u8*)(data_front_buf), sizeof(sample) * DATA_BUF_SIZE);
    output_file.flush();
    Serial.printf("%d written (%u expected) | ", written, sizeof(sample) * DATA_BUF_SIZE);

    u32 write_end = micros();

    if (digitalRead(SWITCH_PIN) == 0) {
      u32 written = output_file.write((u8*)&num_samples, 4);
      Serial.printf("%d written (%u expected) | ", written, 4);
      output_file.close();

      Serial.println("Switch is turned off, stopping now");
      while (1);
    }

    //Serial.printf("Write took %u us\n", write_end - write_start);

    f32 averages[NUM_STRAIN_GAUGES] = { 0 };

    for (u32 i = 0; i < DATA_BUF_SIZE; i++) {
      for (u32 j = 0; j < NUM_STRAIN_GAUGES; j++) {
        averages[j] += (f32)data_front_buf[i].measures[j] * ads_conversion;
      }
    }

    Serial.print(data_front_buf[0].time);
    for (u32 i = 0; i < NUM_STRAIN_GAUGES; i++) {
      averages[i] /= (f32)DATA_BUF_SIZE;
      Serial.printf(",%.5f", averages[i]);
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
  u8 header[4] = { 'M', 'E', 'S', 1 + NUM_STRAIN_GAUGES };
  out_file.write(header, 4);

  char gauge_name[] = { 'g', 'a', 'u', 'g', 'e', '0', '0' };

  u8* field_header = header;

  // Time field
  {
    *(u16*)(field_header) = 1;
    field_header[2] = (u8)mes_types::U32;
    field_header[3] = sizeof("time") - 1;

    out_file.write(field_header, 4);
    out_file.write("time", 4);
  }

  for (u32 i = 0; i < NUM_STRAIN_GAUGES; i++) {
    *(u16*)(field_header) = 1;
    field_header[2] = (u8)mes_types::I16;
    field_header[3] = sizeof(gauge_name);

    if (i < 10) {
      gauge_name[sizeof(gauge_name) - 1] = i + '0';
      gauge_name[sizeof(gauge_name) - 2] = '0';
    } else {
      gauge_name[sizeof(gauge_name) - 1] = (i % 10) + '0';
      gauge_name[sizeof(gauge_name) - 2] = (i / 10) + '0';
    }

    out_file.write(field_header, 4);
    out_file.write(gauge_name, sizeof(gauge_name));
  }

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

static u8 ads_read_reg(ads_reg reg);
static void ads_write_reg(ads_reg reg, u8 val);
static void ads_send_cmd(ads_cmd cmd);

void setup1() {
  //Serial.begin(115200);
  //while (!Serial);

  data_front_buf = (sample*)malloc(sizeof(sample) * DATA_BUF_SIZE);
  data_back_buf = (sample*)malloc(sizeof(sample) * DATA_BUF_SIZE);
  memset(data_front_buf, 0, sizeof(sample) * DATA_BUF_SIZE);
  memset(data_back_buf, 0, sizeof(sample) * DATA_BUF_SIZE);

  pinMode(CIPO_PIN, INPUT);
  pinMode(COPI_PIN, OUTPUT);
  pinMode(SCK_PIN, OUTPUT);

  SPI1.setMISO(CIPO_PIN);
  SPI1.setMOSI(COPI_PIN);
  SPI1.setSCK(SCK_PIN);

  pinMode(ADS_CS_PIN, OUTPUT);
  pinMode(ADS_DRDY_PIN, INPUT);

  digitalWrite(ADS_CS_PIN, HIGH);

  SPI1.begin();

  ads_send_cmd(ads_cmd::RESET);
  delay(100);

  ads_write_reg(ads_reg::MUX, 0b00001000);
  delay(100);

  ads_write_reg(ads_reg::ADCON, 0b00100001);
  delay(100);

  u8 status = ads_read_reg(ads_reg::STATUS);
  Serial.print("ADS Status:\n\tID - ");
  for (u32 i = 7; i >= 4; i--) {
    Serial.write((status >> i) & 1 ? '1' : '0');
  }
  Serial.print("\n\tORDER - ");
  Serial.print(status & 0b1000 ? "LSB" : "MSB");
  Serial.print("\n\tACAL - ");
  Serial.print(status & 0b100 ? "EN" : "DIS");
  Serial.print("\n\tBUFEN - ");
  Serial.print(status & 0b10 ? "EN" : "DIS");
  Serial.print("\n\tDRDY - ");
  Serial.println(status & 0b1);

  u8 ad_con = ads_read_reg(ads_reg::ADCON);
  Serial.print("ADS ADCON: ");
  for (i32 i = 7; i >= 0; i--) {
    Serial.write((ad_con >> i) & 1 ? '1' : '0');
  }
  Serial.println("");

  u8 drate = ads_read_reg(ads_reg::DRATE);
  Serial.print("DRATE: ");
  for (i32 i = 7; i >= 0; i--) {
    Serial.write((drate >> i) & 1 ? '1' : '0');
  }
  Serial.println("");

  ads_send_cmd(ads_cmd::SELFCAL);
  delay(200);
}

u32 cur_pin = 0;
u32 data_buf_pos = 0;
u32 prev_time = 0;

void loop1() {
  u32 prev_pin = cur_pin;
  cur_pin = (cur_pin + 1) % NUM_STRAIN_GAUGES;

  // Wait for DRDY  
  while (digitalRead(ADS_DRDY_PIN) == HIGH);

  SPI1.beginTransaction(ads_settings);
  digitalWrite(ADS_CS_PIN, LOW);

  SPI1.transfer((u8)ads_cmd::WREG | (u8)ads_reg::MUX);
  SPI1.transfer(0x00);
  SPI1.transfer((cur_pin << 4) | 0b1000);
  SPI1.transfer((u8)ads_cmd::SYNC);
  // Rounded up from 3.125
  delayMicroseconds(4);
  SPI1.transfer((u8)ads_cmd::WAKEUP);
  SPI1.transfer((u8)ads_cmd::RDATA);
  // Rounded up from 6.510
  delayMicroseconds(7);

  i16 sample = 0;

  sample |= (i16)SPI1.transfer(0) << 8;
  sample |= (i16)SPI1.transfer(0);
  /*ads_bytes[0] = SPI1.transfer(0);
  ads_bytes[1] = SPI1.transfer(0);
  ads_bytes[2] = SPI1.transfer(0);

  i32 sample = (ads_bytes[0] << 16) | (ads_bytes[1] << 8) | ads_bytes[2];
  sample = (sample & (1 << 23) ? sample - 0x1000000 : sample);*/

  digitalWrite(ADS_CS_PIN, HIGH);
  SPI1.endTransaction();

  //delay(1);

  /*if (cur_pin == 0) {
    Serial.print("time:");
    Serial.print(micros());
  }
  Serial.print(", Guage");
  Serial.print(prev_pin);
  Serial.print(":");
  Serial.print((f32)sample * ads_conversion);
  if (cur_pin == NUM_STRAIN_GAUGES - 1) {
    Serial.println("");
  }*/

  if (prev_pin == 0) {
    data_back_buf[data_buf_pos++] = { 0 };
    data_back_buf[data_buf_pos-1].time = micros();
  } 
  data_back_buf[data_buf_pos-1].measures[prev_pin] = sample;

  if (data_buf_pos >= DATA_BUF_SIZE) {
    tmp_buf  = data_front_buf;
    data_front_buf = data_back_buf;
    data_back_buf = tmp_buf;

    data_buf_pos = 0;

    rp2040.fifo.push(1);
  }
}

static u8 ads_read_reg(ads_reg reg) {
  SPI1.beginTransaction(ads_settings);

  digitalWrite(ADS_CS_PIN, LOW);

  SPI1.transfer((u8)ads_cmd::RREG | (u8)reg);
  SPI1.transfer(0x00);

  delayMicroseconds(5);

  u8 out = SPI1.transfer(0xff);

  digitalWrite(ADS_CS_PIN, HIGH);

  SPI1.endTransaction();

  return out;
}

static void ads_write_reg(ads_reg reg, u8 val) {
  SPI1.beginTransaction(ads_settings);

  digitalWrite(ADS_CS_PIN, LOW);

  SPI1.transfer((u8)ads_cmd::WREG | (u8)reg);
  SPI1.transfer(0x00);
  SPI1.transfer(val);

  digitalWrite(ADS_CS_PIN, HIGH);

  SPI1.endTransaction();
}

static void ads_send_cmd(ads_cmd cmd) {
  SPI1.beginTransaction(ads_settings);

  digitalWrite(ADS_CS_PIN, LOW);

  // TODO: Test if these delays are really necessary
  delayMicroseconds(5);
  SPI1.transfer((u8)cmd);
  delayMicroseconds(5);

  digitalWrite(ADS_CS_PIN, HIGH);

  SPI1.endTransaction();
}

