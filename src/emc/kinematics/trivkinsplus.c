// trivkinsplus.c trivkins with extra non-kinematics joints

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

#include "motion.h" // EMCMOT_MAX_JOINTS
#include "hal.h"
#include "rtapi.h"
#include "rtapi_app.h"

// Support for extrajoints ####################################################
static int extrajoints = 0; // no of extra joints
RTAPI_MP_INT (extrajoints, "Number of extra joints (not used for kinematics)");
#include "extrajoints.h"   // call extrajoints_setup();
// Support for extrajoints ####################################################

static int  num_coordinates =  0;
static char    *coordinates = "";
RTAPI_MP_STRING(coordinates, "Axis letters ordered for joint creation");

static char *kinstype = "1"; // use KINEMATICS_IDENTITY
RTAPI_MP_STRING(kinstype, "Kinematics Type (Identity,Both)");

EXPORT_SYMBOL(kinematicsType);
EXPORT_SYMBOL(kinematicsForward);
EXPORT_SYMBOL(kinematicsInverse);
MODULE_LICENSE("GPL");

struct data {
       hal_s32_t axis_number[EMCMOT_MAX_JOINTS];
} *data;

int kinematicsForward(const double *joints,
                      EmcPose * pos,
                      const KINEMATICS_FORWARD_FLAGS * fflags,
                      KINEMATICS_INVERSE_FLAGS * iflags) {
    int i;
    for(i = 0; i < EMCMOT_MAX_JOINTS; i++) {
        switch(data->axis_number[i]) {
            case 0: pos->tran.x = joints[i]; break;
            case 1: pos->tran.y = joints[i]; break;
            case 2: pos->tran.z = joints[i]; break;
            case 3: pos->a      = joints[i]; break;
            case 4: pos->b      = joints[i]; break;
            case 5: pos->c      = joints[i]; break;
            case 6: pos->u      = joints[i]; break;
            case 7: pos->v      = joints[i]; break;
            case 8: pos->w      = joints[i]; break;
        }
    }
    return 0;
} // kinematicsForward()

int kinematicsInverse(const EmcPose * pos,
                      double *joints,
                      const KINEMATICS_INVERSE_FLAGS * iflags,
                      KINEMATICS_FORWARD_FLAGS * fflags) {
    int i;
    for(i = 0; i < EMCMOT_MAX_JOINTS; i++) {
        switch(data->axis_number[i]) {
            case 0: joints[i] = pos->tran.x; break;
            case 1: joints[i] = pos->tran.y; break;
            case 2: joints[i] = pos->tran.z; break;
            case 3: joints[i] = pos->a;      break;
            case 4: joints[i] = pos->b;      break;
            case 5: joints[i] = pos->c;      break;
            case 6: joints[i] = pos->u;      break;
            case 7: joints[i] = pos->v;      break;
            case 8: joints[i] = pos->w;      break;
        }
    }
    return 0;
} // kinematicsInverse()

int kinematicsHome(EmcPose * world,
                   double *joint,
                   KINEMATICS_FORWARD_FLAGS * fflags,
                   KINEMATICS_INVERSE_FLAGS * iflags) {
    *fflags = 0;
    *iflags = 0;

    return kinematicsForward(joint, world, fflags, iflags);
} // kinematicsHome()

static KINEMATICS_TYPE ktype = -1;

KINEMATICS_TYPE kinematicsType() {
    return ktype;
} // kinematicsType()

static int next_axis_number(void) {
    while(*coordinates) {
        switch(*coordinates) {
            case 'x': case  'X': coordinates++; num_coordinates++;return 0;
            case 'y': case  'Y': coordinates++; num_coordinates++;return 1;
            case 'z': case  'Z': coordinates++; num_coordinates++;return 2;
            case 'a': case  'A': coordinates++; num_coordinates++;return 3;
            case 'b': case  'B': coordinates++; num_coordinates++;return 4;
            case 'c': case  'C': coordinates++; num_coordinates++;return 5;
            case 'u': case  'U': coordinates++; num_coordinates++;return 6;
            case 'v': case  'V': coordinates++; num_coordinates++;return 7;
            case 'w': case  'W': coordinates++; num_coordinates++;return 8;
            case ' ': case '\t': coordinates++; continue;
        }
        rtapi_print_msg(RTAPI_MSG_ERR,
                "\ntrivkinsplus: ERROR: Invalid character '%c' in coordinates\n\n",
                *coordinates);
                return -1;
    }
    return -1;
} // next_axis_number()

static int comp_id;

int rtapi_app_main(void) {
    int retval = 0;
    int i;
    comp_id = hal_init("trivkinsplus");

    if (comp_id < 0) retval = comp_id;

    if (retval == 0) {
        data   = hal_malloc(sizeof(struct data));
        retval = !data;
    }

    switch (*kinstype) {
      case 'b': case 'B': ktype = KINEMATICS_BOTH;         break;
      case 'f': case 'F': ktype = KINEMATICS_FORWARD_ONLY; break;
      case 'i': case 'I': ktype = KINEMATICS_INVERSE_ONLY; break;
      case '1': default:  ktype = KINEMATICS_IDENTITY;
    }

    if (extrajoints != 0) {
        if (*kinstype != KINEMATICS_BOTH) {
            rtapi_print_msg(RTAPI_MSG_ERR,
                "\ntrivkinsplus: extrajoints=%d, using KINEMATICS_BOTH\n",
                extrajoints);
        }
        ktype = KINEMATICS_BOTH;
    }

    if ( (extrajoints == 0) && (*coordinates == 0) ) {
        coordinates = "XYZABCUVW"; // default if not using extrajoints
    }

    for(i=0; i<EMCMOT_MAX_JOINTS; i++) {
        data->axis_number[i] = next_axis_number();
    }

    if ( (num_coordinates <= 0) && (extrajoints !=0) ) {
        rtapi_print_msg(RTAPI_MSG_ERR,
            "\ntrivkinsplus: ERROR: When using extrajoints [=%d],"\
            " coordinates= parameter must be specified\n\n",
            extrajoints);
        hal_exit(comp_id);
        return -1;
    }

    retval = extrajoints_setup(num_coordinates,// == joints used in kins
                               extrajoints,    // no of extra joints
                               comp_id,
                               "trivkinsplus");
    if (retval != 0) {
        hal_exit(comp_id);
        return -1;
    } else {
        hal_ready(comp_id);
    }

    return retval;
} // rtapi_app_main()

void rtapi_app_exit(void) { hal_exit(comp_id); }
