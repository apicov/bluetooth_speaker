/*
 * Shim replacing Bluedroid's bt_target.h.
 *
 * Every OI decoder source is wrapped in
 *     #if (defined(SBC_DEC_INCLUDED) && SBC_DEC_INCLUDED == TRUE)
 * and that gate is the only thing they take from the real header. Providing it
 * here keeps this component free of any dependency on the `bt` component, which
 * matters because satellites decode SBC without running Bluetooth at all.
 *
 * TRUE/FALSE are guarded: oi_stddefs.h defines them too, and is included after
 * this file.
 */
#pragma once

#ifndef TRUE
#define TRUE  1
#endif
#ifndef FALSE
#define FALSE 0
#endif

#define SBC_DEC_INCLUDED TRUE
