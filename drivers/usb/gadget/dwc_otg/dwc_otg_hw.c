#include "dwc_otg_hw.h"

#include <linux/delay.h>
#include <mach/map.h>
#include <mach/iphone-clock.h>

/**
 * This powers up/down the hardware.
 */
int dwc_otg_hw_power(dwc_otg_core_t *core, int _pwr)
{
	if(_pwr)
	{
		iphone_power_ctrl(IPHONE_USB_POWER, 1);
		msleep(10);
		iphone_clock_gate_switch(IPHONE_USB_CLOCK, 1);
		iphone_clock_gate_switch(IPHONE_USBPHY_CLOCK, 1);
		iphone_clock_gate_switch(IPHONE_EDRAM_CLOCK, 1);
	}
	else
	{
		iphone_clock_gate_switch(IPHONE_USB_CLOCK, 0);
		iphone_clock_gate_switch(IPHONE_USBPHY_CLOCK, 0);
		iphone_clock_gate_switch(IPHONE_EDRAM_CLOCK, 0);
		iphone_power_ctrl(IPHONE_USB_POWER, 0);
		msleep(10);
	}

	return 0;
}

