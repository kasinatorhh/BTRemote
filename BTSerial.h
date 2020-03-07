/*
BTSerial.h
Defines the Message Format and Check functions

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

*/

#ifndef BTSerial_h
#define BTSerial_h

#include <inttypes.h>
/******************************************************************************
* Definitions
******************************************************************************/

typedef struct __attribute__((packed)) {
  char Ident;
  uint8_t Switches;
  int16_t GyroX;
  int16_t GyroY;
  int16_t GyroZ;
  int16_t GyroAngle; //not given
  int16_t GyroG;
  int16_t JoyX;
  int16_t JoyY;
  int16_t JoyAngle;
  int16_t JoySpeed;
  uint8_t LRC; //Line check Character. Equals byte wise-sum of message before.
} Message_T;
extern Message_T Msg;
#endif
