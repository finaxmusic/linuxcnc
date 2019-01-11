// extrajoints.h:
// add support for extra joints to a kinematics module

/*
  Copyright 2019 Dewey Garrett <dgarrett@panix.com>

  This program is free software; you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation; either version 2 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
*/

#ifndef EXTRAJOINTS_H
#define EXTRAJOINTS_H

#include "hal.h"

void   extrajoints_update(void*,long);
int    extrajoints_setup(int,int,int,char*);

// arbitrary until some memory limit exceded
#define MAX_EXTRA_JOINTS 9

typedef struct {
    hal_float_t*   prehome_cmd[MAX_EXTRA_JOINTS];  //  IN pin
    hal_float_t*  posthome_cmd[MAX_EXTRA_JOINTS];  //  IN pin
    hal_bit_t*           homed[MAX_EXTRA_JOINTS];  //  IN pin
    hal_float_t*    prehome_fb[MAX_EXTRA_JOINTS];  //  IN pin
    hal_float_t*  motor_offset[MAX_EXTRA_JOINTS];  //  IN pin
    hal_float_t* motor_pos_cmd[MAX_EXTRA_JOINTS];  // OUT pin
    hal_float_t*  motor_pos_fb[MAX_EXTRA_JOINTS];  // OUT pin
} extrajoints_pins_t;

typedef struct {
    int num_kinematic_joints;
    int num_extrajoints;
} extrajoints_t;

#endif
