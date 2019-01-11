// extrajoints.c:
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

#include "extrajoints.h"
#include "motion.h"       //EMCMOT_MAX_JOINTS

static extrajoints_t      *edata;
static extrajoints_pins_t *hdata;

int extrajoints_setup (int  kinsjoints,
                       int  extrajoints,
                       int  c_id,
                       char *cname) {
    int i;
    int retval = 0;
    char buf[HAL_NAME_LEN + 1];

    if ((kinsjoints + extrajoints) > EMCMOT_MAX_JOINTS) {
        rtapi_print_msg(RTAPI_MSG_ERR,
             "\nEXTRAJOINTS max exceded kinsjoints=%d extrajoints=%d max=%d\n\n",
             kinsjoints,extrajoints,EMCMOT_MAX_JOINTS);
        return -1;
    }
    if (extrajoints < 0) {extrajoints = 0;}


    edata = hal_malloc(sizeof(extrajoints_t));
    hdata = hal_malloc(sizeof(extrajoints_pins_t));

    if (edata == 0 || hdata == 0) {
        rtapi_print_msg(RTAPI_MSG_ERR, "EXTRAJOINTS: hal_malloc fail\n");
        return -1;
    }

    edata->num_kinematic_joints = kinsjoints;
    edata->num_extrajoints      = extrajoints;

    for(i=0; i < edata->num_extrajoints; i++) {
        int j = edata->num_kinematic_joints + i;

        retval = hal_pin_float_newf(HAL_IN, &(hdata->prehome_cmd[j]), c_id,
                                              "%s.%d.prehome-cmd",cname,j);
        if (retval != 0) break;
        retval = hal_pin_float_newf(HAL_IN, &(hdata->posthome_cmd[j]), c_id,
                                              "%s.%d.posthome-cmd",cname,j);
        if (retval != 0) break;
        retval = hal_pin_bit_newf(HAL_IN, &(hdata->homed[j]), c_id,
                                            "%s.%d.homed",cname,j);
        if (retval != 0) break;
        retval = hal_pin_float_newf(HAL_IN, &(hdata->prehome_fb[j]), c_id,
                                              "%s.%d.prehome-fb",cname,j);
        if (retval != 0) break;
        retval = hal_pin_float_newf(HAL_IN, &(hdata->motor_offset[j]), c_id,
                                              "%s.%d.motor-offset",cname,j);
        if (retval != 0) break;
        retval = hal_pin_float_newf(HAL_OUT, &(hdata->motor_pos_cmd[j]), c_id,
                                               "%s.%d.motor-pos-cmd",cname,j);
        if (retval != 0) break;
        retval = hal_pin_float_newf(HAL_OUT, &(hdata->motor_pos_fb[j]), c_id,
                                               "%s.%d.motor-pos-fb",cname,j);
        if (retval != 0) break;
    }
    if (retval != 0) {
        rtapi_print_msg(RTAPI_MSG_ERR,
                        "EXTRAJOINTS: failed creating hal pin\n");
        return -1;
    }

    rtapi_snprintf(buf, sizeof(buf),"%s.extrajoints.update", cname);
    retval = hal_export_funct(buf,extrajoints_update,edata,0,0,c_id);
    if (retval != 0) {
        rtapi_print_msg(RTAPI_MSG_ERR,
                        "EXTRAJOINTS: update funct export failed\n");
        return -1;
    }
    return 0;
} // extrajoints_support()

void extrajoints_update(void *arg, long period) {
// This function is exported as a hal function and must be added
// to a thread (typically the servo-thread)
// It cannot be called by kinematicsInverse or kinematicsForward()
// since thse kinematics functions are not invoked prior to homing
// Note: This function supports all of the extra joints
    int i;
    extrajoints_t *eptr = arg;
    for(i=0; i < eptr->num_extrajoints; i++) {
        int  j = eptr->num_kinematic_joints + i;
        if (*hdata->homed[j]) {
            *hdata->motor_pos_cmd[j] = *hdata->posthome_cmd[j]
                                     + *hdata->motor_offset[j];
            *hdata->motor_pos_fb [j] = *hdata->prehome_cmd[j]; // loopback
        } else {
            *hdata->motor_pos_cmd[j] = *hdata->prehome_cmd[j];
            *hdata->motor_pos_fb [j] = *hdata->prehome_fb[j];
        }
    }
} // extrajoints_update()
