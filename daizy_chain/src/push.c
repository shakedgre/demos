/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2019 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 *
 * push.c - App layer application of the onboard push demo. The crazyflie 
 * has to have the multiranger and the flowdeck version 2.
 */

#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "app.h"

#include "commander.h"

#include "FreeRTOS.h"
#include "task.h"

#include "debug.h"

#include "log.h"
#include "param.h"

#include "configblock.h"
#include "radiolink.h"

#define DEBUG_MODULE "PUSH"

#define HIGHRSSI 89

static P2PPacket p_reply;  

float timeOfLastMsg = 0;

uint16_t upi;
uint16_t lefti;
uint16_t righti;
uint16_t fronti;
uint16_t backi;

uint16_t upo;
uint16_t lefto;
uint16_t righto;
uint16_t fronto;
uint16_t backo;


uint16_t lost_contacto = 0;

void p2pcallbackHandler(P2PPacket *p)
{
  //uint8_t id = p->data[0];
  uint8_t rssi = p->rssi;
  //uint8_t port = p->port;

  float timeNow = usecTimestamp() / 1e6;

  memcpy(&upo, &(p->data[1]), sizeof(uint16_t));
  memcpy(&lefto, &(p->data[3]), sizeof(uint16_t));
  memcpy(&righto, &(p->data[5]), sizeof(uint16_t));
  memcpy(&fronto, &(p->data[7]), sizeof(uint16_t));
  memcpy(&backo, &(p->data[9]), sizeof(uint16_t));
  //DEBUG_PRINT("X: %f, Y:%f, Z:%f\n",(double)x,(double)y,(double)Height);
  DEBUG_PRINT("recieving packet! other up is: %d, rssi: %d\n", upo, rssi);
  if(rssi > HIGHRSSI || timeNow-timeOfLastMsg > 2.0f ){
    lost_contacto = 1;
  }
  timeOfLastMsg = timeNow;

}

void sendLocPacket(){
    
    p_reply.port=0x00;
    p_reply.size= 5*sizeof(uint16_t)+1*sizeof(uint8_t);
    uint64_t address = configblockGetRadioAddress();
    uint8_t my_id =(uint8_t)((address) & 0x00000000ff);
    memcpy(&(p_reply.data[0]), &my_id, sizeof(uint8_t));
    memcpy(&(p_reply.data[1]), &upi, sizeof(uint16_t));
    memcpy(&(p_reply.data[3]), &lefti, sizeof(uint16_t));
    memcpy(&(p_reply.data[5]), &righti, sizeof(uint16_t));
    memcpy(&(p_reply.data[7]), &fronti, sizeof(uint16_t));
    memcpy(&(p_reply.data[9]), &backi, sizeof(uint16_t));
    radiolinkSendP2PPacketBroadcast(&p_reply);
}

void appMain(){
  p2pRegisterCB(p2pcallbackHandler);
  while (true){
    
    vTaskDelay(M2T(100));
    logVarId_t idUp = logGetVarId("range", "up");
    logVarId_t idLeft = logGetVarId("range", "left");
    logVarId_t idRight = logGetVarId("range", "right");
    logVarId_t idFront = logGetVarId("range", "front");
    logVarId_t idBack = logGetVarId("range", "back");

    upi = logGetUint(idUp);
    lefti = logGetUint(idLeft);
    righti = logGetUint(idRight);
    fronti = logGetUint(idFront);
    backi = logGetUint(idBack);

    sendLocPacket();
  }


}


  LOG_GROUP_START(other_cf)

  LOG_ADD_CORE(LOG_UINT16, up, &upo)
  LOG_ADD_CORE(LOG_UINT16, left, &lefto)
  LOG_ADD_CORE(LOG_UINT16, right, &righto)
  LOG_ADD_CORE(LOG_UINT16, front, &fronto)
  LOG_ADD_CORE(LOG_UINT16, back, &backo)
  LOG_ADD_CORE(LOG_UINT16, lost_contact, &lost_contacto)
  LOG_GROUP_STOP(other_cf)

 
  LOG_GROUP_START(my_cf)

  LOG_ADD_CORE(LOG_UINT16, up, &upi)
  LOG_ADD_CORE(LOG_UINT16, left, &lefti)
  LOG_ADD_CORE(LOG_UINT16, right, &righti)
  LOG_ADD_CORE(LOG_UINT16, front, &fronti)
  LOG_ADD_CORE(LOG_UINT16, back, &backi)

  LOG_GROUP_STOP(my_cf)

