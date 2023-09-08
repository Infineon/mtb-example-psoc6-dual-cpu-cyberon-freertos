/* Infineon SOD configuration parameters:
 * configurator version,
 * sampling_rate,
 * frame_size,
 * IP_compnent_id,       // SOD is 9
 * number of parameters, //must equal to 2.
 * gap setting: (0,100,200,300,400,500,1000)ms,
 * sensitivity: (0-32767).
 */

#ifndef __CY_SOD_CONFIG_H
#define __CY_SOD_CONFIG_H

#include "stdint.h"

/*
 * This is manually generated configuration file since cofigurator
 * is not available at this time. 
 */
int32_t sod_config_prms[] = {
  0,     /* manunally generated configuration file set configuration version to zero */
  16000, /* sampling rate */
  160,   /* input frmae size */
  9,     /* IP_compnent_id: SOD */
  2,     /* number of parameters */
  400,   /* gap setting */
  16384  /* sensitivity */
};

/* This is the total length of sod_config_prms array */
int32_t sod_config_prms_len = 7;

#endif // __CY_SOD_CONFIG_H

