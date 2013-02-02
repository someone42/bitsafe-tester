/** \file main.c
  *
  * \brief Entry point for BitSafe tester.
  *
  * This file is licensed as described by the file LICENCE.
  */

#include <stdint.h>
#include <string.h>
#include <p32xxxx.h>
#include "usb_hal.h"
#include "usb_standard_requests.h"
#include "usb_callbacks.h"
#include "usb_hid_stream.h"
#include "pic32_system.h"
#include "serial_fifo.h"
#include "ssd1306.h"
#include "sst25x.h"
#include "adc.h"
#include "pushbuttons.h"
#include "atsha204.h"

/** Total number of tests. */
#define NUM_TESTS		4

/** This will be called whenever an unrecoverable error occurs. This should
  * not return. */
void usbFatalError(void)
{
	disableInterrupts();
	PORTDSET = 0x10; // turn on red LED
	while (1)
	{
		// do nothing
	}
}

/** Entry point. This is the first thing which is called after startup code.
  * This never returns. */
int main(void)
{
	int test_number; // should be between 0 and NUM_TESTS - 1 (inclusive)

	disableInterrupts();

	// The BitSafe development board has the Vdd/2 reference connected to
	// a pin which shares the JTAG TMS function. By default, JTAG is enabled
	// and this causes the Vdd/2 voltage to diverge significantly.
	// Disabling JTAG fixes that.
	// This must also be done before calling initSST25x() because one of the
	// external memory interface pins is shared with the JTAG TDI function.
	// Leaving JTAG enabled while calling initSST25x() will cause improper
	// operation of the external memory.
	DDPCONbits.JTAGEN = 0;

	pic32SystemInit();
	initSSD1306();
	initPushButtons();
	initSST25x();
	initATSHA204();
	initADC();
	usbInit();
	usbHIDStreamInit();
	usbDisconnect(); // just in case
	usbSetupControlEndpoint();
	restoreInterrupts(1);

	// The BitSafe development board has VBUS not connected to anything.
	// This causes the PIC32 USB module to think that there is no USB
	// connection. As a workaround, setting VBUSCHG will pull VBUS up.
	// This must be done after calling usbInit() because usbInit() sets
	// the U1OTGCON register.
	U1OTGCONbits.VBUSCHG = 1;

	// All USB-related modules should be initialised before
	// calling usbConnect().
	usbConnect();

	displayOn();
	test_number = 0;
	while (1)
	{
		clearDisplay();
		if (test_number == 0)
		{
			testSSD1306();
		}
		else if (test_number == 1)
		{
			testSST25x();
		}
		else if (test_number == 2)
		{
			testATSHA204();
		}
		else if (test_number == 3)
		{
			testADC();
		}

		waitForNoButtonPress();
		if (waitForButtonPress() == 0)
		{
			test_number++;
			if (test_number >= NUM_TESTS)
			{
				test_number = 0;
			}
		}
		else
		{
			test_number--;
			if (test_number < 0)
			{
				test_number = NUM_TESTS - 1;
			}
		}
	}
}
