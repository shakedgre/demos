#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

#include "app.h"

#include "commander.h"
#include "MovedecisionMakerTest.h"

#include "crtp_commander_high_level.h"

#include "FreeRTOS.h"
#include "task.h"

#include "debug.h"
#include "sensorReading.h"

void appMain() {
  initCommander();
  DEBUG_PRINT("starting\n");
  crtpCommanderHighLevelTakeoff(0.5, 1.0);
  DEBUG_PRINT("took off\n");
  vTaskDelay(M2T(1000));
  crtpCommanderHighLevelGoTo(0.5, 0.0, 0.5, 0.0, 3, false);
  vTaskDelay(M2T(1000));
  crtpCommanderHighLevelGoTo(0.5, 0.0, 0.5, 0.0, 3, false);
  vTaskDelay(M2T(1000));
  crtpCommanderHighLevelGoTo(0.5, 0.0, 0.5, 0.0, 3, false);
  DEBUG_PRINT("going forward\n");
  vTaskDelay(M2T(3000));
  crtpCommanderHighLevelGoTo(0.5, 0.0, 0.5, 0.0, 3, false);
  DEBUG_PRINT("going forward\n");
  vTaskDelay(M2T(1000));
  crtpCommanderHighLevelLand(0.2, 1.0);
  DEBUG_PRINT("landing\n");
  vTaskDelay(M2T(1000));
  crtpCommanderHighLevelStop();
  DEBUG_PRINT("ending\n");

}
