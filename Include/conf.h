#ifndef CONF_H__
#define CONF_H__

/* The advertisement interval in ms, needs to be dividable by 5 .
 * Values needs to be in range [100 - 10240]
 */
#define ADV_INTERVAL	(100)

/* Advertise only on channel 37 */
#define ONE_CHANNEL_ADV

/* Use RC (1) or Xtal (0) */
#define CFG_CLKSRC_RC      (1)

#endif
