//*****************************************************************************
//
// Standard Includes
//
//*****************************************************************************

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

// # Hardware Includes # ********************************************************************************
#include "inc/hw_gpio.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_nvic.h"
#include "inc/hw_types.h"

// # Driverlib Includes # *******************************************************************************
#include "driverlib/adc.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/pwm.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"

// # GrLib Includes # ***********************************************************************************
#include "grlib/grlib.h"

// # Board Driver Includes # ****************************************************************************
#include "examples/boards/dk-tm4c123g/drivers/buttons.h"
#include "examples/boards/dk-tm4c123g/drivers/cfal96x64x16.h"

//*******************************************************************************************************
//
// User Specific Definitions
//
//*******************************************************************************************************
#define ESC_PWM_RATE 50
#define ESC_THROTTLE_OFF   1.20
#define ESC_THROTTLE_ZERO  1.28  // in ms
#define ESC_THROTTLE_MAX   1.82  // in ms
#define ESC_THROTTLE_ARM   1.90  // in ms
#define ADC_THROTTLE_END     10

//*******************************************************************************************************
//
// System Specific Definitions
//
//*******************************************************************************************************
#ifndef NULL
#define NULL 0x0
#endif

#define ROUNDU32(x) ((uint32_t) (x + 0.5))

//*******************************************************************************************************
//
// Function Prototypes
//
//*******************************************************************************************************

void ADCInit(void);
void ADCIntHandler(void);
void BusFaultHandler(void);
void ClearScreen(tContext* psContext);
void FPUInit(void);
void LEDInit(void);
void LEDInvert(void);
void ESCInit(void);
void ESCThrottleArm(void);
void ESCThrottleOff(void);
void ESCThrottleSet(float ui32Throttle);
void SoftReset(void);
void UsageFaultHandler(void);

//*******************************************************************************************************
//
// System Specific Definitions
//
//*******************************************************************************************************
// Define for offscreen buffer for non-flicker display
#define OFFSCREEN_BUF_SIZE GrOffScreen4BPPSize(96, 64)
#define NUM_PALETTE_ENTRIES (sizeof(g_pui32OffscreenPalette) / sizeof(uint32_t))

//*******************************************************************************************************
//
// Global Variables
//
//*******************************************************************************************************
tContext *g_psOnScreenContext;
tContext *g_psOffScreenContext;
tRectangle *g_psClearScreenRect;

uint8_t g_pui8OffscreenBuf[OFFSCREEN_BUF_SIZE];

uint32_t g_pui32OffscreenPalette[] = { ClrBlack, ClrWhite };

uint32_t g_ui32PWMClock;
uint32_t g_ui32OffThrottle  = (uint32_t) ((1000 / ESC_THROTTLE_OFF) + 0.5);
uint32_t g_ui32ZeroThrottle = (uint32_t) ((1000 / ESC_THROTTLE_ZERO) + 0.5);
uint32_t g_ui32MaxThrottle  = (uint32_t)  ((1000 / ESC_THROTTLE_MAX) + 0.5);
uint32_t g_ui32ArmThrottle  = (uint32_t)  ((1000 / ESC_THROTTLE_ARM) + 0.5);
float g_fThrottle;

uint32_t g_ui32ADCSamplePast, g_ui32ADCSample;
bool g_bADCDataReady = false;

int main(void) {
  //*****************************************************************************************************
  // Variable Declaration
  //*****************************************************************************************************

  // Graphics Library
  tContext sOnScreenContext;
  g_psOnScreenContext = &sOnScreenContext;
  tContext sOffScreenContext;
  g_psOffScreenContext = &sOffScreenContext;

  tDisplay sOffscreenDisplay;

  tRectangle sClearScreenRect;
  g_psClearScreenRect = &sClearScreenRect;

  char pcThrottle[50] = "";
  char pcFrq[50] = "";
  char pcTime[50] = "";

  //
  ///////////////////Initialize//////////////////////////////
  //

  //
  // Set the clocking to run at 80MHz.
  //
  MAP_SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

  //
  // Enable the floating-point unit
  //
  FPUInit();

  //
  // Set Fault Interrupts in the NVIC
  //
  IntRegister(FAULT_BUS, BusFaultHandler);
  IntRegister(FAULT_USAGE, UsageFaultHandler);

  //
  // Enable processor interrupts.
  //
  MAP_IntMasterEnable();

  //
  // Enable LED Control
  //
  LEDInit();

  //
  // Initialize the ADC
  //
  ADCInit();

  // Enable the PWM peripheral
  ESCInit();

  //
  // Initialize the display driver.
  //
  CFAL96x64x16Init();

  //
  // Initialize the graphics context for onscreen
  //
  GrContextInit(g_psOnScreenContext, &g_sCFAL96x64x16);
  GrContextFontSet(g_psOnScreenContext, g_psFontCm12/*g_psFontFixed6x8*/);
  GrContextForegroundSet(g_psOnScreenContext, ClrWhite);

  //
  // Initialize an offscreen display and assign the palette.
  //
  GrOffScreen4BPPInit(&sOffscreenDisplay, g_pui8OffscreenBuf, g_sCFAL96x64x16.ui16Width, g_sCFAL96x64x16.ui16Height);
  GrOffScreen4BPPPaletteSet(&sOffscreenDisplay, g_pui32OffscreenPalette, 0, NUM_PALETTE_ENTRIES);

  //
  // Initialize the graphics context for the offscreen buffer
  //
  GrContextInit(g_psOffScreenContext, &sOffscreenDisplay);
  GrContextFontSet(g_psOffScreenContext, g_psFontFixed6x8);
  GrContextForegroundSet(g_psOffScreenContext, ClrWhite);

  //
  // Create reusable graphical elements
  //
  sClearScreenRect.i16XMin = 0;
  sClearScreenRect.i16YMin = 0;
  sClearScreenRect.i16XMax = GrContextDpyWidthGet(g_psOnScreenContext) - 1;
  sClearScreenRect.i16YMax = GrContextDpyHeightGet(g_psOnScreenContext) - 1;

  while (true) {
    LEDInvert();

    if (g_bADCDataReady && (ADC_THROTTLE_END >= g_ui32ADCSample))
    {
      g_bADCDataReady = false;
      g_ui32ADCSamplePast = g_ui32ADCSample;

      //
      // Update the PWM
      //
      ESCThrottleOff();
      sprintf(pcThrottle,"Throttle Off");
    }
    else if (g_bADCDataReady && (g_ui32ADCSample >= (4095 - ADC_THROTTLE_END))) {
      g_bADCDataReady = false;
      g_ui32ADCSamplePast = g_ui32ADCSample;

      ESCThrottleArm();
      sprintf(pcThrottle,"Throttle Arm");
    }
    else if (g_bADCDataReady && abs(g_ui32ADCSample - g_ui32ADCSamplePast) > 3)
    {
      g_bADCDataReady = false;
      g_ui32ADCSamplePast = g_ui32ADCSample;
      g_fThrottle = (g_ui32ADCSample - ADC_THROTTLE_END) / (4095.0 - (2 * ADC_THROTTLE_END));

      //
      // Update the PWM
      //
      ESCThrottleSet(g_fThrottle);
      sprintf(pcThrottle,"%%: %11.7f", g_fThrottle * 100);
    }

    //
    // Update display strings
    //
    float fFrq = g_ui32PWMClock / (float) PWMPulseWidthGet(PWM0_BASE, PWM_GEN_0);
    sprintf(pcFrq,  "f: %11.7f",   fFrq);
    sprintf(pcTime, "t: %11.7f",   (1000 / fFrq));

    //
    // Clean the display
    //
    ClearScreen(g_psOffScreenContext);

    //
    // Draw the strings to the off screen buffer
    //
    GrStringDraw(g_psOffScreenContext, pcThrottle, -1, 5, 5, true);
    GrStringDraw(g_psOffScreenContext, pcFrq, -1, 5, 20, true);
    GrStringDraw(g_psOffScreenContext, pcTime, -1, 5, 35, true);

    //
    // Redraw the LCD
    //
    GrImageDraw(g_psOnScreenContext, g_psOffScreenContext->psDisplay->pvDisplayData, 0, 0);
  }
}

//*******************************************************************************************************
//
// Helper Functions
//
//*******************************************************************************************************

void ADCInit(void) {
  //
  // Enable the GPIO peripheral used by the accelerometer
  //
  MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
  while (!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOD))
    ;

  //
  // Enable the accelerometer peripheral
  //
  MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
  while (!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_ADC0))
    ;
  MAP_SysCtlPeripheralReset(SYSCTL_PERIPH_ADC0);
  while (!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_ADC0))
    ;

  //
  // Configure GPIO PD7 as ADC0 for potentiometer
  //
  MAP_GPIOPinTypeADC(GPIO_PORTD_BASE, GPIO_PIN_7);

  //
  // Configure ADC0 on sequence 0 as a low priority continuous trigger on Channel 0
  // with an over sample of 64x
  //
  MAP_ADCSequenceDisable(ADC0_BASE, 0);
  MAP_ADCSequenceConfigure(ADC0_BASE, 0, ADC_TRIGGER_ALWAYS, 0);
  MAP_ADCSequenceStepConfigure(ADC0_BASE, 0, 0, ADC_CTL_CH4 | ADC_CTL_IE | ADC_CTL_END);
  MAP_ADCHardwareOversampleConfigure(ADC0_BASE, 64);
  MAP_ADCSequenceEnable(ADC0_BASE, 0);

  // Enable ADC interrupts
  ADCIntRegister(ADC0_BASE, 0, ADCIntHandler);
  MAP_IntEnable(INT_ADC0SS0);
  MAP_ADCIntEnable(ADC0_BASE, 0);

  //
  // Wait for first data from ADC
  //
  while (!g_bADCDataReady);
}

void ADCIntHandler(void) {
  MAP_ADCIntClear(ADC0_BASE, 0);
  g_bADCDataReady = MAP_ADCSequenceDataGet(ADC0_BASE, 0, &g_ui32ADCSample) ? true : false;
}

void BusFaultHandler(void) {
  while (true) {
  }
}

void ClearScreen(tContext* psContext) {
  uint32_t ui32Foreground = psContext->ui32Foreground;
  psContext->ui32Foreground = 0x0;
  GrRectFill(psContext, g_psClearScreenRect);
  psContext->ui32Foreground = ui32Foreground;
}

void FPUInit(void) {
  //
  // Enable the floating-point unit
  //
  MAP_FPUEnable();

  //
  // Configure the floating-point unit to perform lazy stacking of the floating-point state.
  //
  MAP_FPULazyStackingEnable();
}

void LEDInit(void) {
  //
  // Enable the peripherals used by the on-board LED.
  //
  MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOG);

  //
  // Enable the GPIO pins for the LED.
  //
  MAP_GPIOPinTypeGPIOOutput(GPIO_PORTG_BASE, GPIO_PIN_2);
}

void LEDInvert(void) {
  if (MAP_GPIOPinRead(GPIO_PORTG_BASE, GPIO_PIN_2)) {
    MAP_GPIOPinWrite(GPIO_PORTG_BASE, GPIO_PIN_2, 0x0);
  } else {
    MAP_GPIOPinWrite(GPIO_PORTG_BASE, GPIO_PIN_2, GPIO_PIN_2);
  }
}

void ESCInit(void) {
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOH);

  //set pin and port for pwm 0 module 0
  GPIOPinConfigure(GPIO_PH0_M0PWM0);
  GPIOPinTypePWM(GPIO_PORTH_BASE, GPIO_PIN_0);

  SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
  SysCtlPeripheralReset(SYSCTL_PERIPH_PWM0);

  //
  // Disable the generator to prevent noise before initialization
  //
  PWMGenDisable(PWM0_BASE, PWM_GEN_0);

  //
  // Set the clock divider to 64
  //
  SysCtlPWMClockSet(SYSCTL_PWMDIV_64);
  g_ui32PWMClock = (SysCtlClockGet() / 64);

  //
  // Configure the PWM0 to count up/down without synchronization.
  //
  PWMGenConfigure(PWM0_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN);

  PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, g_ui32PWMClock / ESC_PWM_RATE);

  ESCThrottleSet(0);

  //
  // Enable the PWM0 Bit 0 and Bit 1 output signals.
  //
  PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT | PWM_OUT_1_BIT, true);

  PWMGenEnable(PWM0_BASE, PWM_GEN_0);
}

void ESCThrottleOff(void) {
  //
  // Enables the counter for a PWM generator block.
  //
  PWMGenDisable(PWM0_BASE, PWM_GEN_0);
  //
  // Set the PWM width
  //
  PWMPulseWidthSet(PWM0_BASE, PWM_GEN_0, g_ui32PWMClock / g_ui32OffThrottle);
  //
  // Enables the counter for a PWM generator block.
  //
  PWMGenEnable(PWM0_BASE, PWM_GEN_0);
}

void ESCThrottleArm(void) {
  //
  // Enables the counter for a PWM generator block.
  //
  PWMGenDisable(PWM0_BASE, PWM_GEN_0);
  //
  // Set the PWM width
  //
  PWMPulseWidthSet(PWM0_BASE, PWM_GEN_0, g_ui32PWMClock / g_ui32ArmThrottle);
  //
  // Enables the counter for a PWM generator block.
  //
  PWMGenEnable(PWM0_BASE, PWM_GEN_0);
}

void ESCThrottleSet(float ui32Throttle) {
  ui32Throttle = (ui32Throttle < 0) ? 0 : (ui32Throttle > 1) ? 1 : ui32Throttle;

  //
  // Enables the counter for a PWM generator block.
  //
  PWMGenDisable(PWM0_BASE, PWM_GEN_0);

  //
  // Set the PWM width
  //
  PWMPulseWidthSet(PWM0_BASE, PWM_GEN_0, g_ui32PWMClock / ROUNDU32(g_ui32ZeroThrottle + (((int64_t) g_ui32MaxThrottle - g_ui32ZeroThrottle) * ui32Throttle)));

  //
  // Enables the counter for a PWM generator block.
  //
  if (ui32Throttle) PWMGenEnable(PWM0_BASE, PWM_GEN_0);
}

void SoftReset(void) {
  HWREG(NVIC_APINT) = NVIC_APINT_VECTKEY | NVIC_APINT_SYSRESETREQ;
}

void UsageFaultHandler(void) {
  while (true) {
  }
}
