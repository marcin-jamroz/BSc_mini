/* BSD 2-Clause License

Copyright (c) 2018, Marcin Jamroz
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/


//nrf24L01 communication
#include <RF24Network.h>
#include <RF24.h>
#include <SPI.h>
#include "mini.h"

RF24 radio(A0, A1);
RF24Network network(radio);

const uint16_t this_node = 01;
const uint16_t remote_node = 00;


//Driver library
#include <Driver.h>
unsigned long currentTime = 0;
unsigned long previousTime = 0;

bool isChanged = false;

Driver driver(2, 2, 2, 2);
//end

uint16_t moveTime = 0;



//debug
int odebrane = 0;
int wyslane = 0;
unsigned long debugTime = 0;
unsigned long debugSendTime = 0;

void setup() {
  Serial.begin(57600);

  driver.M1_bind(3, 4);
  driver.M2_bind(5, 7);
  driver.M3_bind(6, 8);
  driver.M4_bind(9, 10);

  SPI.begin();
  radio.begin();
  network.begin(90, this_node);

  Serial.println(F("RF24Network start"));
 // radio.setAutoAck(false);
}

void loop() {

  network.update();

//    if (millis() - debugSendTime > 500 && wyslane < 500) {
//    sendPoUART(10, 0, NULL);
//    wyslane++;
//    debugSendTime = millis();
//  }

  while (network.available()) {
    receive();
  }

  currentTime = millis();

  if(currentTime - debugTime > 10000){
   // Serial.print("--------odebrane: "); Serial.println(odebrane);
        Serial.print("--------wyslane: "); Serial.println(wyslane);

    debugTime = millis();
  }

  resend_bg();

  if (currentTime - previousTime > moveTime) {
    driver.stop();
  }
}

void forwardSrv(uint16_t a1, uint8_t a2) {
  moveTime = a1;
  previousTime = millis();
  driver.up(a2);
  forwardRet(1);

}

void backwardSrv(uint16_t a1, uint8_t a2) {
  moveTime = a1;
  previousTime = millis();
  driver.down(a2);
  backwardRet(1);
}

void leftSrv(uint16_t a1, uint8_t a2) {
  moveTime = a1 * 9.0703;
  previousTime = millis();
  driver.left(a2);
  leftRet(1);
}

void rightSrv(uint16_t a1, uint8_t a2) {
  moveTime = a1 * 9.5238;
  previousTime = millis();
  driver.right(a2);
  rightRet(1);
}

void checkProgressSrv(void) {
  uint16_t timeLeft = moveTime - (currentTime - previousTime);
  if (timeLeft > 0) {
    checkProgressRet(timeLeft);
  } else {
    checkProgressRet(0);
  }
}

void stopSrv(void) {
  driver.stop();
  stopRet(1);
}


