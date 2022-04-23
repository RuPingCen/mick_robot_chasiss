/*This file has been prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief Header file for speed_cntr.c.
 *
 * - File:               speed_cntr.h
 * - Compiler:           IAR EWAAVR 4.11A
 * - Supported devices:  All devices with a 16 bit timer can be used.
 *                       The example is written for ATmega48
 * - AppNote:            AVR446 - Linear speed control of stepper motor
 *
 * \author               Atmel Corporation: http://www.atmel.com \n
 *                       Support email: avr@atmel.com
 *
 * $Name: RELEASE_1_0 $
 * $Revision: 1.2 $
 * $RCSfile: speed_cntr.h,v $
 * $Date: 2006/05/08 12:25:58 $
 *****************************************************************************/

#ifndef SPEED_CNTR_H
#define SPEED_CNTR_H

/*! \brief Holding data used by timer interrupt for speed ramp calculation.
 *
 *  Contains data used by timer interrupt to calculate speed profile.
 *  Data is written to it by move(), when stepper motor is moving (timer
 *  interrupt running) data is read/updated when calculating a new step_delay
 */
typedef struct {
  //! What part of the speed ramp we are in.
  unsigned char run_state : 3;
  //! Direction stepper motor should move.
  unsigned char dir : 1;
  //! Peroid of next timer delay. At start this value set the accelration rate.
  unsigned int step_delay;
  //! What step_pos to start decelaration
  unsigned int decel_start;
  //! Sets deceleration rate.
  signed int decel_val;
  //! Minimum time delay (max speed)
  signed int min_delay;
  //! Counter used when accelerateing/decelerateing to calculate step_delay.
  signed int accel_count;
} speedRampData;

/*! \Brief Frequency of timer1 in [Hz].
 *
 * Modify this according to frequency used. Because of the prescaler setting,
 * the timer1 frequency is the clock frequency divided by 8.
 */
// Timer/Counter 1 running on 3,686MHz / 8 = 460,75kHz (2,17uS). (T1-FREQ 460750)
#define T1_FREQ 460750

//! Number of (full)steps per round on stepper motor in use.
#define FSPR 200

//#ifdef HALFSTEPS
//  #define SPR (FSPR*2)
//  #pragma message("[speed_cntr.c] *** Using Halfsteps ***")
//#endif
//#ifdef FULLSTEPS
//  #define SPR FSPR
//  #pragma message("[speed_cntr.c] *** Using Fullsteps ***")
//#endif
//#ifndef HALFSTEPS
//  #ifndef FULLSTEPS
//    #error FULLSTEPS/HALFSTEPS not defined!
//  #endif
//#endif

// Maths constants. To simplify maths when calculating in speed_cntr_Move().
#define ALPHA (2*3.14159/SPR)                    // 2*pi/spr
#define A_T_x100 ((long)(ALPHA*T1_FREQ*100))     // (ALPHA / T1_FREQ)*100
#define T1_FREQ_148 ((int)((T1_FREQ*0.676)/100)) // divided by 100 and scaled by 0.676
#define A_SQ (long)(ALPHA*2*10000000000)         // ALPHA*2*10000000000
#define A_x20000 (int)(ALPHA*20000)              // ALPHA*20000

// Speed ramp states
#define STOP  0
#define ACCEL 1
#define DECEL 2
#define RUN   3

void speed_cntr_Move(signed int step, unsigned int accel, unsigned int decel, unsigned int speed);
void speed_cntr_Init_Timer1(void);
static unsigned long sqrt(unsigned long v);
unsigned int min(unsigned int x, unsigned int y);

//! Global status flags
extern struct GLOBAL_FLAGS status;

#endif
