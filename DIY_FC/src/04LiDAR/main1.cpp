#include <Arduino.h>
#include <vl53l8cx.h>
#include <Wire.h>

// function prototypes
void print_result(VL53L8CX_ResultsData*);
void display_commands_banner(void);
// void handle_cmd(uint8_t);


// Instantiate the VL53L8CX object
VL53L8CX lidar(&Wire, VL53L8CX_DEFAULT_I2C_ADDRESS);

bool EnableAmbient = false;
bool EnableSignal = false;
uint8_t resolution = VL53L8CX_RESOLUTION_4X4;
char report[256];
uint8_t status;

void setup(){
    Serial.begin(115200);
    Serial.println("VL53L8CX test");
    Wire.begin();

    // Initialize the VL53L8CX sensor
    lidar.begin();
    status = lidar.init();

    status = lidar.start_ranging();
}

void loop(){
    uint8_t data_ready = 0;
    VL53L8CX_ResultsData  Results;
    // Check if data is ready
    do{
        status = lidar.check_data_ready(&data_ready);
    } while (!data_ready);

    // Get the ranging data
    if ((!status) && (data_ready != 0)) {
        status = lidar.get_ranging_data(&Results);
        print_result(&Results);
    }

    // if (Serial.available() > 0 ){
    //   handle_cmd(Serial.read());
    // }
    delay(100);
}


void print_result(VL53L8CX_ResultsData *Result){
  int8_t i, j, k, l;
  uint8_t zones_per_line;
  uint8_t number_of_zones = resolution;

  zones_per_line = (number_of_zones == 16) ? 4 : 8;

  display_commands_banner();

  Serial.print("Cell Format :\n\n");

  for (l = 0; l < VL53L8CX_NB_TARGET_PER_ZONE; l++) {
    snprintf(report, sizeof(report), " \033[38;5;10m%20s\033[0m : %20s\n", "Distance [mm]", "Status");
    Serial.print(report);

    if (EnableAmbient || EnableSignal) {
      snprintf(report, sizeof(report), " %20s : %20s\n", "Signal [kcps/spad]", "Ambient [kcps/spad]");
      Serial.print(report);
    }
  }

  Serial.print("\n\n");

  for (j = 0; j < number_of_zones; j += zones_per_line) {
    for (i = 0; i < zones_per_line; i++) {
      Serial.print(" -----------------");
    }
    Serial.print("\n");

    for (i = 0; i < zones_per_line; i++) {
      Serial.print("|                 ");
    }
    Serial.print("|\n");

    for (l = 0; l < VL53L8CX_NB_TARGET_PER_ZONE; l++) {
      // Print distance and status
      for (k = (zones_per_line - 1); k >= 0; k--) {
        if (Result->nb_target_detected[j + k] > 0) {
          snprintf(report, sizeof(report), "| \033[38;5;10m%5ld\033[0m  :  %5ld ",
                   (long)Result->distance_mm[(VL53L8CX_NB_TARGET_PER_ZONE * (j + k)) + l],
                   (long)Result->target_status[(VL53L8CX_NB_TARGET_PER_ZONE * (j + k)) + l]);
          Serial.print(report);
        } else {
          snprintf(report, sizeof(report), "| %5s  :  %5s ", "X", "X");
          Serial.print(report);
        }
      }
      Serial.print("|\n");

      if (EnableAmbient || EnableSignal) {
        // Print Signal and Ambient
        for (k = (zones_per_line - 1); k >= 0; k--) {
          if (Result->nb_target_detected[j + k] > 0) {
            if (EnableSignal) {
              snprintf(report, sizeof(report), "| %5ld  :  ", (long)Result->signal_per_spad[(VL53L8CX_NB_TARGET_PER_ZONE * (j + k)) + l]);
              Serial.print(report);
            } else {
              snprintf(report, sizeof(report), "| %5s  :  ", "X");
              Serial.print(report);
            }
            if (EnableAmbient) {
              snprintf(report, sizeof(report), "%5ld ", (long)Result->ambient_per_spad[j + k]);
              Serial.print(report);
            } else {
              snprintf(report, sizeof(report), "%5s ", "X");
              Serial.print(report);
            }
          } else {
            snprintf(report, sizeof(report), "| %5s  :  %5s ", "X", "X");
            Serial.print(report);
          }
        }
        Serial.print("|\n");
      }
    }
  }
  for (i = 0; i < zones_per_line; i++) {
    Serial.print(" -----------------");
  }
  Serial.print("\n");
}

void display_commands_banner(void)
{
  snprintf(report, sizeof(report), "%c[2H", 27); /* 27 is ESC command */
  Serial.print(report);

  Serial.print("53L8A1 Simple Ranging demo application\n");
  Serial.print("--------------------------------------\n\n");

  Serial.print("Use the following keys to control application\n");
  Serial.print(" 'r' : change resolution\n");
  Serial.print(" 's' : enable signal and ambient\n");
  Serial.print(" 'c' : clear screen\n");
  Serial.print("\n");
}

// void handle_cmd(uint8_t cmd)
// {
//   switch (cmd) {
//     case 'r':
//       toggle_resolution();
//       clear_screen();
//       break;

//     case 's':
//       toggle_signal_and_ambient();
//       clear_screen();
//       break;

//     case 'c':
//       clear_screen();
//       break;

//     default:
//       break;
//   }
// }