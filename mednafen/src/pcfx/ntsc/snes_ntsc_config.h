/* Configure library by modifying this file.
   PC-FX port (Paul Daniel): input is RGB16 built from the core's RGB888
   output, and output is 32-bit to match Mednafen's surface format. */

#ifndef SNES_NTSC_CONFIG_H
#define SNES_NTSC_CONFIG_H

#define SNES_NTSC_IN_FORMAT SNES_NTSC_RGB16
#define SNES_NTSC_OUT_DEPTH 32

/* Type of input pixel values */
#define SNES_NTSC_IN_T unsigned short

#define SNES_NTSC_ADJ_IN( in ) in

#endif
