#include <Arduino.h>
#include <Wire.h>
#include <vl53l7cx_class.h>

#define SerialPort Serial

#define LPN_PIN 6
#define I2C_RST_PIN 7
#define PWREN_PIN 8
//pini total la misto sa dea niste semnale, avand in vedere ca avem i2c separate nu ne ajuta cu nimic, dar nici nu am chef sa rescriu biblioteca


void print_result(VL53L7CX_ResultsData *Result);
void clear_screen(void);
void handle_cmd(uint8_t cmd);
void display_commands_banner(void);

// Components.
VL53L7CX vl53l7cx_st(&Wire, LPN_PIN, I2C_RST_PIN);
VL53L7CX vl53l7cx_dr(&Wire1, LPN_PIN, I2C_RST_PIN);

bool EnableAmbient = false;
bool EnableSignal = false;
uint8_t res = VL53L7CX_RESOLUTION_4X4;
char report[256];

/* Setup ---------------------------------------------------------------------*/
void setup() {
  // Initialize serial for output.
  SerialPort.begin(460800);

  // Initialize I2C bus.

  Wire.setSDA(20);
  Wire.setSCL(21);

  Wire1.setSDA(2);
  Wire1.setSCL(3);


  Wire.begin();
  Wire1.begin();

    // Configure VL53L7CX component.
  vl53l7cx_st.begin();
  vl53l7cx_dr.begin();

  vl53l7cx_st.init_sensor();
  vl53l7cx_dr.init_sensor();

  // Start Measurements
  vl53l7cx_st.vl53l7cx_start_ranging();
  vl53l7cx_dr.vl53l7cx_start_ranging();
}

void loop() {
  VL53L7CX_ResultsData ResultsST;
  uint8_t NewDataReadyST = 0;
  uint8_t statusST;

  VL53L7CX_ResultsData ResultsDR;
  uint8_t NewDataReadyDR = 0;
  uint8_t statusDR;

  //Senzor stanga:
  if (!NewDataReadyST) {
    statusST = vl53l7cx_st.vl53l7cx_check_data_ready(&NewDataReadyST);
  }
  if ((!statusST) && (NewDataReadyST != 0)) {
    statusST = vl53l7cx_st.vl53l7cx_get_ranging_data(&ResultsST);
    print_result(&ResultsST);
  }

  //Senzor dreapta:
  if (!NewDataReadyDR) {
    statusDR = vl53l7cx_dr.vl53l7cx_check_data_ready(&NewDataReadyDR);
  }
  if ((!statusDR) && (NewDataReadyDR != 0)) {
    statusDR = vl53l7cx_dr.vl53l7cx_get_ranging_data(&ResultsDR);
    print_result(&ResultsDR);
  }


  if (Serial.available() > 0) {
    handle_cmd(Serial.read());
  }
  delay(1000);
}

void print_result(VL53L7CX_ResultsData *Result) {
  int8_t i, j, k, l;
  uint8_t zones_per_line;
  uint8_t number_of_zones = res;

  zones_per_line = (number_of_zones == 16) ? 4 : 8;

  display_commands_banner();

  SerialPort.print("Cell Format :\n\n");

  for (l = 0; l < VL53L7CX_NB_TARGET_PER_ZONE; l++) {
    snprintf(report, sizeof(report), " \033[38;5;10m%20s\033[0m : %20s\n", "Distance [mm]", "Status");
    SerialPort.print(report);

    if (EnableAmbient || EnableSignal) {
      snprintf(report, sizeof(report), " %20s : %20s\n", "Signal [kcps/spad]", "Ambient [kcps/spad]");
      SerialPort.print(report);
    }
  }

  SerialPort.print("\n\n");

  for (j = 0; j < number_of_zones; j += zones_per_line) {
    for (i = 0; i < zones_per_line; i++)
      SerialPort.print(" -----------------");
    SerialPort.print("\n");

    for (i = 0; i < zones_per_line; i++)
      SerialPort.print("|                 ");
    SerialPort.print("|\n");

    for (l = 0; l < VL53L7CX_NB_TARGET_PER_ZONE; l++) {
      // Print distance and status
      for (k = (zones_per_line - 1); k >= 0; k--) {
        if (Result->nb_target_detected[j + k] > 0) {
          snprintf(report, sizeof(report), "| \033[38;5;10m%5ld\033[0m  :  %5ld ",
                   (long)Result->distance_mm[(VL53L7CX_NB_TARGET_PER_ZONE * (j + k)) + l],
                   (long)Result->target_status[(VL53L7CX_NB_TARGET_PER_ZONE * (j + k)) + l]);
          SerialPort.print(report);
        } else {
          snprintf(report, sizeof(report), "| %5s  :  %5s ", "X", "X");
          SerialPort.print(report);
        }
      }
      SerialPort.print("|\n");

      if (EnableAmbient || EnableSignal) {
        // Print Signal and Ambient
        for (k = (zones_per_line - 1); k >= 0; k--) {
          if (Result->nb_target_detected[j + k] > 0) {
            if (EnableSignal) {
              snprintf(report, sizeof(report), "| %5ld  :  ", (long)Result->signal_per_spad[(VL53L7CX_NB_TARGET_PER_ZONE * (j + k)) + l]);
              SerialPort.print(report);
            } else {
              snprintf(report, sizeof(report), "| %5s  :  ", "X");
              SerialPort.print(report);
            }
            if (EnableAmbient) {
              snprintf(report, sizeof(report), "%5ld ", (long)Result->ambient_per_spad[j + k]);
              SerialPort.print(report);
            } else {
              snprintf(report, sizeof(report), "%5s ", "X");
              SerialPort.print(report);
            }
          } else {
            snprintf(report, sizeof(report), "| %5s  :  %5s ", "X", "X");
            SerialPort.print(report);
          }
        }
        SerialPort.print("|\n");
      }
    }
  }
  for (i = 0; i < zones_per_line; i++)
    SerialPort.print(" -----------------");
  SerialPort.print("\n");
}

void toggle_resolution(void) {
  vl53l7cx_st.vl53l7cx_stop_ranging();

  switch (res) {
    case VL53L7CX_RESOLUTION_4X4:
      res = VL53L7CX_RESOLUTION_8X8;
      break;

    case VL53L7CX_RESOLUTION_8X8:
      res = VL53L7CX_RESOLUTION_4X4;
      break;

    default:
      break;
  }
  vl53l7cx_st.vl53l7cx_set_resolution(res);
  vl53l7cx_st.vl53l7cx_start_ranging();
}

void toggle_signal_and_ambient(void) {
  EnableAmbient = (EnableAmbient) ? false : true;
  EnableSignal = (EnableSignal) ? false : true;
}

void clear_screen(void) {
  snprintf(report, sizeof(report), "%c[2J", 27); /* 27 is ESC command */
  SerialPort.print(report);
}

void display_commands_banner(void) {
  snprintf(report, sizeof(report), "%c[2H", 27); /* 27 is ESC command */
  SerialPort.print(report);

  Serial.print("53L7A1 Simple Ranging demo application\n");
  Serial.print("--------------------------------------\n\n");

  Serial.print("Use the following keys to control application\n");
  Serial.print(" 'r' : change resolution\n");
  Serial.print(" 's' : enable signal and ambient\n");
  Serial.print(" 'c' : clear screen\n");
  Serial.print("\n");
}

void handle_cmd(uint8_t cmd) {
  switch (cmd) {
    case 'r':
      toggle_resolution();
      clear_screen();
      break;

    case 's':
      toggle_signal_and_ambient();
      clear_screen();
      break;

    case 'c':
      clear_screen();
      break;

    default:
      break;
  }
}
