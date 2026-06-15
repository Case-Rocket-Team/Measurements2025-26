
#include <SPI.h>
#include <SD.h>

typedef int8_t i8;
typedef int16_t i16;
typedef int32_t i32;
typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;

typedef float f32;

#define FLASH_CS 5

#define IN_FILE_NAME "pre-comp-tests0000.mes"

namespace mes {
  enum class types : u8 {
    I8 = 0,
    I16 = 1,
    I32 = 2,
    U8 = 3,
    U16 = 4,
    U32 = 5,
    F32 = 6
  };

  struct header {
      u8 magic[3];
      u8 num_fields;
  };

  struct field {
    u16 freq;
    u8 data_type;
    u8 name_size;
  };
}

void setup() {
  Serial.begin(115200);
  while (!Serial);

  if (!SD.begin(FLASH_CS)) {
    Serial.println("SD initialization failed!");
    while (1);
  }

  {
    File root = SD.open("/");
    File entry;

    while ((entry = root.openNextFile())) {
      Serial.printf("File '%s' (%u bytes)\n", entry.name(), entry.size());

      entry.close();
    }

    root.close();
  }

  File in_file = SD.open(IN_FILE_NAME, FILE_READ);

  //Serial.printf("File is %u bytes in size\n", in_file.size());

  mes::header header = { 0 };
  in_file.read((u8*)(&header), 4);

  if (header.magic[0] != 'M' || header.magic[1] != 'E' || header.magic[2] != 'S') {
    Serial.printf("Invalid MES file %c %c %c\n", header.magic[0], header.magic[1], header.magic[2]);
    in_file.close();
    while (1);
  }

  mes::field* fields = (mes::field*)malloc(sizeof(mes::field) * header.num_fields);

  char name_chars[255] = { 0 };

  for (u32 i = 0; i < header.num_fields; i++) {
    in_file.read((u8*)(&fields[i]), 4);

    in_file.read((u8*)name_chars, fields[i].name_size);

    /*Serial.printf(
      "Field %u: { freq: %u, type: %u, name: %.*s }\n",
      i, fields[i].freq, fields[i].data_type,
      fields[i].name_size, name_chars
    );*/

    Serial.printf("%.*s", fields[i].name_size, name_chars);


    if (i != (u32)(header.num_fields - 1)) {
      Serial.print(",");
    } else {
      Serial.print("\n");
    }
  }

  u32 sample_start_pos = in_file.position();

  in_file.seek(in_file.size() - 4);
  u32 num_samples = 0;
  in_file.read((u8*)(&num_samples), 4);

  //Serial.printf("%u samples\n", num_samples);

  in_file.seek(sample_start_pos);

  for (u32 i = 0; i < num_samples && in_file.position() < in_file.size(); i++) {
    for (u32 j = 0; j < header.num_fields; j++) {
      if ((i % fields[j].freq) == 0) {
        switch (fields[j].data_type) {
          case (u8)mes::types::I8: {
            i8 d;
            in_file.read((u8*)(&d), 1);
            Serial.print(d);
          } break;

          case (u8)mes::types::I16: {
            i16 d;
            in_file.read((u8*)(&d), 2);
            Serial.print(d);
          } break;

          case (u8)mes::types::I32: {
            i32 d;
            in_file.read((u8*)(&d), 4);
            Serial.print(d);
          } break;

          case (u8)mes::types::U8: {
            u8 d;
            in_file.read((u8*)(&d), 1);
            Serial.print(d);
          } break;

          case (u8)mes::types::U16: {
            u16 d;
            in_file.read((u8*)(&d), 2);
            Serial.print(d);
          } break;

          case (u8)mes::types::U32: {
            u32 d;
            in_file.read((u8*)(&d), 4);
            Serial.print(d);
          } break;

          case (u8)mes::types::F32: {
            f32 d;
            in_file.read((u8*)(&d), 4);
            Serial.print(d);
          } break;
        }
      }

      if (j != (u32)(header.num_fields - 1)) {
        Serial.print(",");
      } else {
        Serial.print("\n");
        Serial.flush();
        delay(1);
      }
    }
  }

  free(fields);

  in_file.close();
}

void loop() { }

