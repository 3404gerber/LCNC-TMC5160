/********************************************************************
* Description:  tmc5160.c
*       This file, 'tmc5160.c', is a HAL component that provides
*       an SPI connection to external TMC5160 stepper drivers.
*  				
*		
*		
*
* Author: Luca Gerber
* License: GPL Version 3
*
*		Credit to Scott Alford, the author of the Remora component,
*		from which a lot of code has been copied, especially the
*		entire SPI device initialisation and control. Special thanks
*		also to B.Stultiens for the work on the spix driver. And to
*		all other contributor, on Linuxcnc but also on FluidNC and
*		grblHAL from were I took a lot of code. In the end, I didn't
*		code this module, I copy-paste-patchworked it.
*
* Copyright (c) 2025	All rights reserved.
*
* 2025-02-22: first release on github
********************************************************************/

#include "rtapi.h"
#include "rtapi_app.h"
#include "hal.h"
#include <stdio.h>

#include "dtcboards.h"

// Using BCM2835 driver library by Mike McCauley, why reinvent the wheel!
// http://www.airspayce.com/mikem/bcm2835/index.html
#include "bcm2835.h"
#include "bcm2835.c"

// Raspberry Pi 5 uses the RP1
#include "rp1lib.h"
#include "rp1lib.c"
#include "gpiochip_rp1.h"
#include "gpiochip_rp1.c"
#include "spi-dw.h"
#include "spi-dw.c"

#include "tmc5160.h"

int rtapi_app_main(void){
 
    char name[HAL_NAME_LEN + 1];
	int i,j,k, retval;
	int8_t max_left_drivers = MAX_TMC_DRIVER;
	uint8_t num_cs_pins = 0;
	uint8_t num_enable_pins = 0;
	uint8_t num_chains = 0;
	uint8_t num_driver = 0;

	// Initialise RPi4 / RPi5 identifier variables
	bcm = false;
	rp1 = false;

	if(!rt_detect_board())
	{
	  rtapi_print_msg(RTAPI_MSG_ERR,"rt_peripheral_init failed.\n");
      return -1;
	}

	// Check configured chains and drivers
	for (i = 0; i < MAX_TMC_DRIVER; i++) {
		if (chains[i]>0){
			if (chains[i]<= max_left_drivers){
				num_chains++;
				max_left_drivers -= chains[i];
				num_driver += chains[i];
			}else{
				rtapi_print_msg(RTAPI_MSG_ERR,
					"TMC: ERROR: too many drivers declared! \n");
				return -1;
			}
		}
		if(rp1 == true & num_chains > (2 + SPI_num)){
			rtapi_print_msg(RTAPI_MSG_ERR,
					"TMC: ERROR: only 2 chains on SPI0 or 3 on SPI1 allowed on RPi5! \n");
				return -1;
		}
    }

	// Check if chip select pins correctly declared
	if(bcm == true) {
		for (i = 0; i < MAX_TMC_DRIVER; i++) {
			if (cs_pins[i]>0){
				num_cs_pins++;
			}
		}
		if (num_cs_pins != num_chains){
			rtapi_print_msg(RTAPI_MSG_ERR, "ERROR: wrong number of chip select pins! \n");
			return -1;
		}
	}else{
		if ((2+SPI_num-CS_num)<num_chains){
			rtapi_print_msg(RTAPI_MSG_ERR, "ERROR: Max chains if starting at CS pin %u is %u (on SPI%u)! \n", CS_num, (2+SPI_num-CS_num), SPI_num);
			return -1;
		}
	}

	// Check if frequency correctly declared
	// TODO: finish this part
	if(bcm == true) {
		if(SPI_clk_div<128){
			if(TMC_freq < 12500000 | SPI_clk_div<64){
				rtapi_print_msg(RTAPI_MSG_ERR, "ERROR: The SPI_clk_div is too low for the TMC frequency! Use 128 for TMC internal clock and 64 if with external clock higher than 12.5MHz \n");
				return -1;
			}
		}else if(SPI_clk_div>128){
			DEBUG_PRINT("With a SPI_clk_div higher than 128, the transmission speed may negatively impact the thread execution time. Rather than decreasing the speed, maybe you can try to connect the drivers in several chains?");
		}
	}else{
		if (SPI_freq >0 & TMC_freq/SPI_freq < 2){
			rtapi_print_msg(RTAPI_MSG_ERR, "TMC freq has to be at least twice as fast as SPI_freq");
			return -1;
		}
	}

    // Connect to the HAL, initialise the driver
    comp_id = hal_init(modname);
    if (comp_id < 0)
	{
		rtapi_print_msg(RTAPI_MSG_ERR, "%s ERROR: hal_init() failed \n", modname);
		return -1;
    }

	// Allocate shared memory
	data = hal_malloc(sizeof(data_t));
	if (data == 0) {
		rtapi_print_msg(RTAPI_MSG_ERR,
			"%s: ERROR: hal_malloc() failed\n", modname);
		hal_exit(comp_id);
		return -1;
	}

	// Saving the number of chains and drivers for later use in routines
	tmc_drivers_count = num_driver;

	// Initialise the gpio and spi peripherals
	if(!rt_peripheral_init())
	{
	  rtapi_print_msg(RTAPI_MSG_ERR,"rt_peripheral_init failed.\n");
      return -1;
		
	}

	// Export global TMC Disable, Reset, Error and Connection bits

	retval = hal_pin_bit_newf(HAL_IN, &(data->TMCDisable),
		comp_id, "%s.TMC-Disable", prefix);
	if (retval != 0) goto error;

	retval = hal_pin_bit_newf(HAL_IN, &(data->TMCReset),
		comp_id, "%s.TMC-reset", prefix);
	if (retval != 0) goto error;

	retval = hal_pin_bit_newf(HAL_OUT, &(data->TMCError),
		comp_id, "%s.TMC-error", prefix);
	if (retval != 0) goto error;

	retval = hal_pin_bit_newf(HAL_OUT, &(data->TMCConnection),
		comp_id, "%s.TMC-Connection", prefix);
	if (retval != 0) goto error;

	// Export tmc_debug_mode pin

	retval = hal_pin_s32_newf(HAL_IN, &(data->tmc_debug_mode),
		comp_id, "%s.TMC-debug-mode", prefix);
	if (retval != 0) goto error;

	//###################### TMC-CONF bits ################################

	retval = hal_pin_bit_newf(HAL_IN, &(data->gconf_b),
		comp_id, "%s.CONF-BITS.GCONF", prefix);
	if (retval != 0) goto error;

	retval = hal_pin_bit_newf(HAL_IN, &(data->global_scaler_b),
		comp_id, "%s.CONF-BITS.GLOBALSCALER", prefix);
	if (retval != 0) goto error;

	retval = hal_pin_bit_newf(HAL_IN, &(data->ihold_irun_b),
		comp_id, "%s.CONF-BITS.IHOLD_IRUN", prefix);
	if (retval != 0) goto error;

	retval = hal_pin_bit_newf(HAL_IN, &(data->tpowerdown_b),
		comp_id, "%s.CONF-BITS.TPOWERDOWN", prefix);
	if (retval != 0) goto error;

	retval = hal_pin_bit_newf(HAL_IN, &(data->tpwmthrs_b),
		comp_id, "%s.CONF-BITS.TPWMTHRS", prefix);
	if (retval != 0) goto error;

	retval = hal_pin_bit_newf(HAL_IN, &(data->tcoolthrs_b),
		comp_id, "%s.CONF-BITS.TCOOLTHRS", prefix);
	if (retval != 0) goto error;

	retval = hal_pin_bit_newf(HAL_IN, &(data->thigh_b),
		comp_id, "%s.CONF-BITS.THIGH", prefix);
	if (retval != 0) goto error;

	retval = hal_pin_bit_newf(HAL_IN, &(data->xactual_b),
		comp_id, "%s.CONF-BITS.XACTUAL", prefix);
	if (retval != 0) goto error;

	retval = hal_pin_bit_newf(HAL_IN, &(data->amax_b),
		comp_id, "%s.CONF-BITS.AMAX", prefix);
	if (retval != 0) goto error;

	retval = hal_pin_bit_newf(HAL_IN, &(data->vdcmin_b),
		comp_id, "%s.CONF-BITS.VDCMIN", prefix);
	if (retval != 0) goto error;

	retval = hal_pin_bit_newf(HAL_IN, &(data->swmode_b),
		comp_id, "%s.CONF-BITS.SWMODE", prefix);
	if (retval != 0) goto error;

	retval = hal_pin_bit_newf(HAL_IN, &(data->ramp_stat_b),
		comp_id, "%s.CONF-BITS.RAMP_STAT", prefix);
	if (retval != 0) goto error;

	retval = hal_pin_bit_newf(HAL_IN, &(data->encmode_b),
		comp_id, "%s.CONF-BITS.ENCMODE", prefix);
	if (retval != 0) goto error;

	retval = hal_pin_bit_newf(HAL_IN, &(data->x_enc_b),
		comp_id, "%s.CONF-BITS.X_ENC", prefix);
	if (retval != 0) goto error;

	retval = hal_pin_bit_newf(HAL_IN, &(data->enc_const_b),
		comp_id, "%s.CONF-BITS.ENC_CONST", prefix);
	if (retval != 0) goto error;

	retval = hal_pin_bit_newf(HAL_IN, &(data->enc_deviation_b),
		comp_id, "%s.CONF-BITS.ENC_DEVIATION", prefix);
	if (retval != 0) goto error;

	retval = hal_pin_bit_newf(HAL_IN, &(data->chopconf_b),
		comp_id, "%s.CONF-BITS.CHOPCONF", prefix);
	if (retval != 0) goto error;

	retval = hal_pin_bit_newf(HAL_IN, &(data->coolconf_b),
		comp_id, "%s.CONF-BITS.COOLCONF", prefix);
	if (retval != 0) goto error;

	retval = hal_pin_bit_newf(HAL_IN, &(data->pwmconf_b),
		comp_id, "%s.CONF-BITS.PWMCONF", prefix);
	if (retval != 0) goto error;

	retval = hal_pin_bit_newf(HAL_IN, &(data->conf_done_b),
		comp_id, "%s.CONF-BITS.CONF_DONE_FLAG", prefix);
	if (retval != 0) goto error;

	//#####################################################################

	// Export all the variables for each chain and driver and initialise the
	// register default values.

	k=0;
    for (i = 0; i < num_chains; i++) {

		for (j = 0; j < chains[i]; j++){

			//##################### TMC Registers #####################//

			retval = hal_param_u32_newf(HAL_RW, &(data->gconf[k]),
				comp_id, "%s.chain.%01d.driver.%01d.register.gconf", prefix, i,j);
			if (retval < 0) goto error;
			data->gconf[k] = 8; //Try stealthchop with 12

			retval = hal_param_u32_newf(HAL_RW, &(data->global_scaler[k]),
				comp_id, "%s.chain.%01d.driver.%01d.register.global_scaler", prefix, i,j);
			if (retval < 0) goto error;
			// Lowest recommended setting is 129. Combined with default ihold_irun, set ~26% of driver max current for IRUN
			// and half this current for IHOLD. The driver max current is determined by the sense resistor value on the board.
			data->global_scaler[k] = 129;

			retval = hal_param_u32_newf(HAL_RW, &(data->ihold_irun[k]),
				comp_id, "%s.chain.%01d.driver.%01d.register.ihold_irun", prefix, i,j);
			if (retval < 0) goto error;
			data->ihold_irun[k] = 266248; // Set IRUN to 16 (lowest recommended value), IHOLD to 8 and IHOLDDELAY to 4.

			retval = hal_param_u32_newf(HAL_RW, &(data->tpowerdown[k]),
				comp_id, "%s.chain.%01d.driver.%01d.register.tpowerdown", prefix, i,j);
			if (retval < 0) goto error;
			// From the datasheet: Attention, a minimum setting of 2 is required to allow
			// automatic tuning of StealthChop PWM_OFS_AUTO.
			data->tpowerdown[k] = 10;

			retval = hal_param_u32_newf(HAL_RW, &(data->tpwmthrs[k]),
				comp_id, "%s.chain.%01d.driver.%01d.register.tpwmthrs", prefix, i,j);
			if (retval < 0) goto error;
			data->tpwmthrs[k] = 0;

			retval = hal_param_u32_newf(HAL_RW, &(data->tcoolthrs[k]),
				comp_id, "%s.chain.%01d.driver.%01d.register.tcoolthrs", prefix, i,j);
			if (retval < 0) goto error;
			data->tcoolthrs[k] = 0;

			retval = hal_param_u32_newf(HAL_RW, &(data->thigh[k]),
				comp_id, "%s.chain.%01d.driver.%01d.register.thigh", prefix, i,j);
			if (retval < 0) goto error;
			data->thigh[k] = 0;

			retval = hal_param_u32_newf(HAL_RW, &(data->amax[k]),
				comp_id, "%s.chain.%01d.driver.%01d.register.amax", prefix, i,j);
			if (retval < 0) goto error;
			// By default, the driver accelerate at maximum rate between each LCNC velocity command.
			data->amax[k] = MAX_TMC_ACCEL;

			retval = hal_param_u32_newf(HAL_RW, &(data->vdcmin[k]),
				comp_id, "%s.chain.%01d.driver.%01d.register.vdcmin", prefix, i,j);
			if (retval < 0) goto error;
			data->vdcmin[k] = 0;

			retval = hal_param_u32_newf(HAL_RW, &(data->sw_mode[k]),
				comp_id, "%s.chain.%01d.driver.%01d.register.sw_mode", prefix, i,j);
			if (retval < 0) goto error;
			data->sw_mode[k] = 0;

			retval = hal_param_u32_newf(HAL_RW, &(data->ramp_stat[k]),
				comp_id, "%s.chain.%01d.driver.%01d.register.ramp_stat", prefix, i,j);
			if (retval < 0) goto error;
			data->ramp_stat[k] = 0;

			retval = hal_param_u32_newf(HAL_RW, &(data->encmode[k]),
				comp_id, "%s.chain.%01d.driver.%01d.register.encmode", prefix, i,j);
			if (retval < 0) goto error;
			data->encmode[k] = 0;

			retval = hal_param_u32_newf(HAL_RW, &(data->enc_const[k]),
				comp_id, "%s.chain.%01d.driver.%01d.register.enc_const", prefix, i,j);
			if (retval < 0) goto error;
			// Default value
			data->enc_const[k] = 65536;

			retval = hal_param_u32_newf(HAL_RW, &(data->enc_deviation[k]),
				comp_id, "%s.chain.%01d.driver.%01d.register.enc_deviation", prefix, i,j);
			if (retval < 0) goto error;
			data->enc_deviation[k] = 0;

			retval = hal_param_u32_newf(HAL_RW, &(data->chopconf[k]),
				comp_id, "%s.chain.%01d.driver.%01d.register.chopconf", prefix, i,j);
			if (retval < 0) goto error;
			// Probably the most important register. Default value is a good starting point.
			data->chopconf[k] = 37814610;

			retval = hal_param_u32_newf(HAL_RW, &(data->coolconf[k]),
				comp_id, "%s.chain.%01d.driver.%01d.register.coolconf", prefix, i,j);
			if (retval < 0) goto error;
			data->coolconf[k] = 0;

			retval = hal_param_u32_newf(HAL_RW, &(data->pwmconf[k]),
				comp_id, "%s.chain.%01d.driver.%01d.register.pwmconf", prefix, i,j);
			if (retval < 0) goto error;
			// Stealthchop PWM Voltage register, disabled by default.
			data->pwmconf[k] = 0;

			//## Velocity cmd, position feedback, position scale and debug ##//

			retval = hal_pin_float_newf(HAL_IN, &(data->vel_cmd[k]),
				comp_id, "%s.chain.%01d.driver.%01d.vel-cmd", prefix, i,j);
			if (retval < 0) goto error;
			*(data->vel_cmd[k]) = 0.0;

			retval = hal_pin_float_newf(HAL_OUT, &(data->pos_fb[k]),
				comp_id, "%s.chain.%01d.driver.%01d.pos-fb", prefix, i,j);
			if (retval < 0) goto error;
			*(data->pos_fb[k]) = 0.0;

			retval = hal_param_float_newf(HAL_RW, &(data->pos_scale[k]),
				comp_id, "%s.chain.%01d.driver.%01d.pos-scale", prefix, i,j);
			if (retval < 0) goto error;
			data->pos_scale[k] = 1;

			retval = hal_pin_u32_newf(HAL_OUT, &(data->tmc_debug_out[k]),
				comp_id, "%s.chain.%01d.driver.%01d.debug_out", prefix, i,j);
			if (retval < 0) goto error;
			*(data->tmc_debug_out[k]) = 0;

			//############### TMC Status word and flags ###############//

			retval = hal_pin_bit_newf(HAL_OUT, &(data->tmc_reset_flag[k]),
				comp_id, "%s.chain.%01d.driver.%01d.status.tmc_reset_flag", prefix, i,j);
			if (retval < 0) goto error;

			retval = hal_pin_bit_newf(HAL_OUT, &(data->tmc_driver_error[k]),
				comp_id, "%s.chain.%01d.driver.%01d.status.tmc_error_flag", prefix, i,j);
			if (retval < 0) goto error;

			retval = hal_pin_bit_newf(HAL_OUT, &(data->tmc_sg2[k]),
				comp_id, "%s.chain.%01d.driver.%01d.status.tmc_sg2_flag", prefix, i,j);
			if (retval < 0) goto error;

			retval = hal_pin_bit_newf(HAL_OUT, &(data->tmc_stop_l[k]),
				comp_id, "%s.chain.%01d.driver.%01d.status.ref_l_flag", prefix, i,j);
			if (retval < 0) goto error;

			retval = hal_pin_bit_newf(HAL_OUT, &(data->tmc_stop_r[k]),
				comp_id, "%s.chain.%01d.driver.%01d.status.ref_r_flag", prefix, i,j);
			if (retval < 0) goto error;

			retval = hal_pin_u32_newf(HAL_OUT, &(data->tmc_status[k]),
				comp_id, "%s.chain.%01d.driver.%01d.status.tmc_status_word", prefix, i,j);
			if (retval < 0) goto error;

			k++;
		}

	}

	// Initialise the TMC drivers and check communication
	if(!rt_tmc_init()) {
	  rtapi_print_msg(RTAPI_MSG_ERR,"rt_tmc_init failed.\n");
      return -1;
	}

	error:
	if (retval < 0) {
		rtapi_print_msg(RTAPI_MSG_ERR,
		        "%s: ERROR: pin export failed with err=%i\n",
		        modname, retval);
		hal_exit(comp_id);
		return -1;
	}

	rtapi_snprintf(name, sizeof(name), "%s.configure", prefix);
	/* no FP operations */
	retval = hal_export_funct(name, tmc_drive_conf, data, 0, 0, comp_id);
	if (retval < 0) {
		rtapi_print_msg(RTAPI_MSG_ERR,
		        "%s: ERROR: configure function export failed\n", modname);
		hal_exit(comp_id);
		return -1;
	}

	rtapi_snprintf(name, sizeof(name), "%s.write", prefix);
	/* no FP operations */
	retval = hal_export_funct(name, tmc_write, data, 1, 0, comp_id);
	if (retval < 0) {
		rtapi_print_msg(RTAPI_MSG_ERR,
		        "%s: ERROR: write function export failed\n", modname);
		hal_exit(comp_id);
		return -1;
	}

	rtapi_snprintf(name, sizeof(name), "%s.read", prefix);
	retval = hal_export_funct(name, tmc_read, data, 1, 0, comp_id);
	if (retval < 0) {
		rtapi_print_msg(RTAPI_MSG_ERR,
		        "%s: ERROR: read function export failed\n", modname);
		hal_exit(comp_id);
		return -1;
	}
	
	rtapi_snprintf(name, sizeof(name), "%s.debug", prefix);
	retval = hal_export_funct(name, tmc_debug_read, data, 0, 0, comp_id);
	if (retval < 0) {
		rtapi_print_msg(RTAPI_MSG_ERR,
		        "%s: ERROR: read function export failed\n", modname);
		hal_exit(comp_id);
		return -1;
	}

	rtapi_print_msg(RTAPI_MSG_INFO, "%s: installed driver\n", modname);
	hal_ready(comp_id);
    return 0;	
}

void rtapi_app_exit(void)
{
    hal_exit(comp_id);
}

int rt_detect_board(void){
	FILE *fp;
	int i, j;
	char buf[256];
	ssize_t buflen;
	char *cptr;
	const int DTC_MAX = 8;
	const char *dtcs[DTC_MAX + 1];
	
	// assume were only running on >RPi3

    if ((fp = fopen("/proc/device-tree/compatible" , "rb"))) {
		// Read the 'compatible' string-list from the device-tree
		buflen = fread(buf, 1, sizeof(buf), fp);
		if(buflen < 0) {
			rtapi_print_msg(RTAPI_MSG_ERR,"Failed to read platform identity.\n");
			return -1;
		}

		// Decompose the device-tree buffer into a string-list with the pointers to
		// each string in dtcs. Don't go beyond the buffer's size.
		memset(dtcs, 0, sizeof(dtcs));
		for(i = 0, cptr = buf; i < DTC_MAX && cptr; i++) {
			dtcs[i] = cptr;
			j = strlen(cptr);
			if((cptr - buf) + j + 1 < buflen)
				cptr += j + 1;
			else
				cptr = NULL;
		}

		for(i = 0; dtcs[i] != NULL; i++) {
			rtapi_print_msg(RTAPI_MSG_ERR, "dtcs: %s\n", dtcs[i]);
			if(		!strcmp(dtcs[i], DTC_RPI_MODEL_4B)
				||	!strcmp(dtcs[i], DTC_RPI_MODEL_4CM)
				||	!strcmp(dtcs[i], DTC_RPI_MODEL_400)
				||	!strcmp(dtcs[i], DTC_RPI_MODEL_3BP)
				||	!strcmp(dtcs[i], DTC_RPI_MODEL_3AP)
				||	!strcmp(dtcs[i], DTC_RPI_MODEL_3B)) {
				DEBUG_PRINT("Raspberry Pi 3 or 4, using BCM2835 driver\n");
				bcm = true;
				break;	// Found our supported board
			}else if(!strcmp(dtcs[i], DTC_RPI_MODEL_5B) || !strcmp(dtcs[i], DTC_RPI_MODEL_5CM)) {
				DEBUG_PRINT("Raspberry Pi 5, using rp1 driver\n");
				rp1 = true;
				break;	// Found our supported board
			}else{
				rtapi_print_msg(RTAPI_MSG_ERR, "Error, RPi not detected\n");
				return -1;
			}
		}
		fclose(fp);
	}else{
		rtapi_print_msg(RTAPI_MSG_ERR,"Cannot open '/proc/device-tree/compatible' for read.\n");
	}
}

int rt_peripheral_init(void){

	if (bcm == true){
		// Map the RPi BCM2835 peripherals - uses "rtapi_open_as_root" in place of "open"
		if (!rt_bcm2835_init()){
			rtapi_print_msg(RTAPI_MSG_ERR,"rt_bcm2835_init failed. Are you running with root privlages??\n");
			return -1;
		}

		// Set the SPI0 pins to the Alt 0 function to enable SPI0 access, setup CS register
		// and clear TX and RX fifos
		if (!bcm2835_spi_begin()){
			rtapi_print_msg(RTAPI_MSG_ERR,"bcm2835_spi_begin failed. Are you running with root privlages??\n");
			return -1;
		}

		// Configure SPI0
		bcm2835_spi_setBitOrder(BCM2835_SPI_BIT_ORDER_MSBFIRST);      // The default
		bcm2835_spi_setDataMode(BCM2835_SPI_MODE3);                   // The default

		//bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_128);		// 3.125MHz on RPI3
		//bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_64);		// 6.250MHz on RPI3
		//bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_32);		// 12.5MHz on RPI3
		//bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_16);		// 25MHz on RPI3
		
		// check if the default SPI clock divider has been overriden at the command line
		if (SPI_clk_div != -1){
			// check that the setting is a power of 2
			if ((SPI_clk_div & (SPI_clk_div - 1)) == 0)
			{
				bcm2835_spi_setClockDivider(SPI_clk_div);
				rtapi_print_msg(RTAPI_MSG_INFO,"PRU: SPI clk divider overridden and set to %d\n", SPI_clk_div);			
			}
			else
			{
				// it's not a power of 2
				rtapi_print_msg(RTAPI_MSG_ERR,"ERROR: PRU SPI clock divider incorrect\n");
				return -1;
			}	
		}
		else{
			bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_16);
			rtapi_print_msg(RTAPI_MSG_INFO,"PRU: SPI default clk divider set to 16\n");
		}

		//bcm2835_spi_chipSelect(BCM2835_SPI_CS0);                    // The default
		//bcm2835_spi_setChipSelectPolarity(BCM2835_SPI_CS0, LOW);    // the default
		bcm2835_spi_chipSelect(BCM2835_SPI_CS_NONE);                  // None, will be switched manualy
		
		/* RPI_GPIO_P1_19        = 10 		MOSI when SPI0 in use
			* RPI_GPIO_P1_21        =  9 		MISO when SPI0 in use
			* RPI_GPIO_P1_23        = 11 		CLK when SPI0 in use
			* RPI_GPIO_P1_24        =  8 		CE0 when SPI0 in use
			* RPI_GPIO_P1_26        =  7 		CE1 when SPI0 in use
			*/

		// Configure pullups on SPI0 pins - source termination and CS high (does this allows for higher clock frequencies??? wiring is more important here)
		bcm2835_gpio_set_pud(RPI_GPIO_P1_19, BCM2835_GPIO_PUD_DOWN);	// MOSI
		bcm2835_gpio_set_pud(RPI_GPIO_P1_21, BCM2835_GPIO_PUD_DOWN);	// MISO
	}
	else if (rp1 == true){
		
		if (!rt_rp1lib_init()) {
			rtapi_print_msg(RTAPI_MSG_ERR,"rt_rp1_init failed.\n");
			return -1;
		}

		if (SPI_num == -1) SPI_num = 0; // default to SPI0
		if (CS_num == -1) CS_num = 0; // default to CS0
		if (SPI_freq == -1) SPI_freq = 4000000; // default to 4MHz
		
		if (!rp1spi_init(SPI_num, CS_num, SPI_MODE_3, SPI_freq)) {  // SPIx, CS, mode, freq SPI_MODE_3
			rtapi_print_msg(RTAPI_MSG_ERR,"rp1spi_init failed.\n");
			return -1;
		}
		DEBUG_PRINT("rp1spi_init done");
	}
	else{
		return -1;
	}
}

int rt_bcm2835_init(void)
{
    int  memfd;
    int  ok;
    FILE *fp;

    if (debug) {
		bcm2835_peripherals = (uint32_t*)BCM2835_PERI_BASE;
		bcm2835_pads = bcm2835_peripherals + BCM2835_GPIO_PADS/4;
		bcm2835_clk  = bcm2835_peripherals + BCM2835_CLOCK_BASE/4;
		bcm2835_gpio = bcm2835_peripherals + BCM2835_GPIO_BASE/4;
		bcm2835_pwm  = bcm2835_peripherals + BCM2835_GPIO_PWM/4;
		bcm2835_spi0 = bcm2835_peripherals + BCM2835_SPI0_BASE/4;
		bcm2835_bsc0 = bcm2835_peripherals + BCM2835_BSC0_BASE/4;
		bcm2835_bsc1 = bcm2835_peripherals + BCM2835_BSC1_BASE/4;
		bcm2835_st   = bcm2835_peripherals + BCM2835_ST_BASE/4;
		bcm2835_aux  = bcm2835_peripherals + BCM2835_AUX_BASE/4;
		bcm2835_spi1 = bcm2835_peripherals + BCM2835_SPI1_BASE/4;
		return 1; /* Success */
    }

    /* Figure out the base and size of the peripheral address block
    // using the device-tree. Required for RPi2/3/4, optional for RPi 1
    */
    if ((fp = fopen(BMC2835_RPI2_DT_FILENAME , "rb"))) {
        unsigned char buf[16];
        uint32_t base_address;
        uint32_t peri_size;
        if (fread(buf, 1, sizeof(buf), fp) >= 8) {
            base_address = (buf[4] << 24) |
              (buf[5] << 16) |
              (buf[6] << 8) |
              (buf[7] << 0);
            
            peri_size = (buf[8] << 24) |
              (buf[9] << 16) |
              (buf[10] << 8) |
              (buf[11] << 0);
            
            if (!base_address) {
                /* looks like RPI 4 */
                base_address = (buf[8] << 24) |
                      (buf[9] << 16) |
                      (buf[10] << 8) |
                      (buf[11] << 0);
                      
                peri_size = (buf[12] << 24) |
                (buf[13] << 16) |
                (buf[14] << 8) |
                (buf[15] << 0);
            }
            /* check for valid known range formats */
            if ((buf[0] == 0x7e) &&
                    (buf[1] == 0x00) &&
                    (buf[2] == 0x00) &&
                    (buf[3] == 0x00) &&
                    ((base_address == BCM2835_PERI_BASE) || (base_address == BCM2835_RPI2_PERI_BASE) || (base_address == BCM2835_RPI4_PERI_BASE)))
            {
                bcm2835_peripherals_base = (off_t)base_address;
                bcm2835_peripherals_size = (size_t)peri_size;
                if( base_address == BCM2835_RPI4_PERI_BASE )
                {
                    pud_type_rpi4 = 1;
                }
            }
        
        }
        
		fclose(fp);
    }
    /* else we are prob on RPi 1 with BCM2835, and use the hardwired defaults */

    /* Now get ready to map the peripherals block 
     * If we are not root, try for the new /dev/gpiomem interface and accept
     * the fact that we can only access GPIO
     * else try for the /dev/mem interface and get access to everything
     */
    memfd = -1;
    ok = 0;
    if (geteuid() == 0) {
      /* Open the master /dev/mem device */
    	if ((memfd = rtapi_open_as_root("/dev/mem", O_RDWR | O_SYNC) ) < 0) {
	  		fprintf(stderr, "bcm2835_init: Unable to open /dev/mem: %s\n",
		  	strerror(errno)) ;
	  		goto exit;
		}
      
		/* Base of the peripherals block is mapped to VM */
		bcm2835_peripherals = mapmem("gpio", bcm2835_peripherals_size, memfd, bcm2835_peripherals_base);
		if (bcm2835_peripherals == MAP_FAILED) goto exit;
		
		/* Now compute the base addresses of various peripherals, 
		// which are at fixed offsets within the mapped peripherals block
		// Caution: bcm2835_peripherals is uint32_t*, so divide offsets by 4
		*/
		bcm2835_gpio = bcm2835_peripherals + BCM2835_GPIO_BASE/4;
		bcm2835_pwm  = bcm2835_peripherals + BCM2835_GPIO_PWM/4;
		bcm2835_clk  = bcm2835_peripherals + BCM2835_CLOCK_BASE/4;
		bcm2835_pads = bcm2835_peripherals + BCM2835_GPIO_PADS/4;
		bcm2835_spi0 = bcm2835_peripherals + BCM2835_SPI0_BASE/4;
		bcm2835_bsc0 = bcm2835_peripherals + BCM2835_BSC0_BASE/4; /* I2C */
		bcm2835_bsc1 = bcm2835_peripherals + BCM2835_BSC1_BASE/4; /* I2C */
		bcm2835_st   = bcm2835_peripherals + BCM2835_ST_BASE/4;
		bcm2835_aux  = bcm2835_peripherals + BCM2835_AUX_BASE/4;
		bcm2835_spi1 = bcm2835_peripherals + BCM2835_SPI1_BASE/4;

		ok = 1;
    }else {
		/* Not root, try /dev/gpiomem */
		/* Open the master /dev/mem device */
		if ((memfd = open("/dev/gpiomem", O_RDWR | O_SYNC) ) < 0) {
			fprintf(stderr, "bcm2835_init: Unable to open /dev/gpiomem: %s\n",
				strerror(errno)) ;
			goto exit;
		}  
		/* Base of the peripherals block is mapped to VM */
		bcm2835_peripherals_base = 0;
		bcm2835_peripherals = mapmem("gpio", bcm2835_peripherals_size, memfd, bcm2835_peripherals_base);
		if (bcm2835_peripherals == MAP_FAILED) goto exit;
		bcm2835_gpio = bcm2835_peripherals;
		ok = 1;
    }

exit:
    if (memfd >= 0)
        close(memfd);
    if (!ok)
	bcm2835_close();

    return ok;
}

int rt_rp1lib_init(void) {
    uint64_t phys_addr = RP1_BAR1;

    DEBUG_PRINT("Initialising RP1 library: %s\n", __func__);

    // rp1_chip is declared in gpiochip_rp1.c
    chip = &rp1_chip;

    inst = rp1_create_instance(chip, phys_addr, NULL);
    if (!inst)
        return -1;

    inst->phys_addr = phys_addr;

    // map memory
    inst->mem_fd = rtapi_open_as_root("/dev/mem", O_RDWR | O_SYNC);
    if (inst->mem_fd < 0)
        return errno;

    inst->priv = mmap(
        NULL,
        RP1_BAR1_LEN,
        PROT_READ | PROT_WRITE,
        MAP_SHARED,
        inst->mem_fd,
        inst->phys_addr
        );

    DEBUG_PRINT("Base address: %11lx, size: %lx, mapped at address: %p\n", inst->phys_addr, RP1_BAR1_LEN, inst->priv);

    if (inst->priv == MAP_FAILED)
        return errno;

    return 1;
}

int rt_tmc_init(void){

	uint8_t idx;
	*data->conf_done_b = 0;
	*(data->TMCReset) = 0;
	*(data->TMCError) = 0;
	*(data->TMCDisable) = 0;
	*(data->TMCConnection) = 0;
	*(data->tmc_debug_mode) = -1;

	// Set CS directions
	// TODO: make this a bit more robust to wrong pin numbers.
	if(bcm){
		for (int i = 0; i < MAX_TMC_DRIVER; i++) {
			if (cs_pins[i]>0){
				bcm2835_gpio_fsel(cs_pins[i], BCM2835_GPIO_FSEL_OUTP);
			}
		}
	}

	// Try to connect to the driver and reset them
	if(!tmc_reset())
		return -1;

	//Set Rampmode to positive velocity
	idx= tmc_drivers_count;
	do {
		--idx;
		txData.datagram[idx].regAdr = 0x20;
		txData.datagram[idx].value = 0x01;
		data->tmc_reverse_dir[idx] = false;
	} while(idx);
	tmc_spi_write_all(false);

	//Set position XACTUAL to 0
	idx= tmc_drivers_count;
	do {
		--idx;
		txData.datagram[idx].regAdr = 0x21;
		txData.datagram[idx].value = 0x01;
	} while(idx);
	tmc_spi_write_all(false);
	*data->xactual_b = 1;

	return 1;
}

bool tmc_reset(){

	uint8_t idx;

	idx= tmc_drivers_count;
	do {
		--idx;
		txData.datagram[idx].regAdr = 0x04; //IOIN reg
	} while(idx);
	tmc_spi_read_all(false);
	tmc_spi_read_all(false);

	idx= tmc_drivers_count;
	do {
		--idx;
		if( rxData.datagram[idx].data[3] == 0 || rxData.datagram[idx].data[3] == 0xFF){
			rtapi_print_msg(RTAPI_MSG_ERR,"TMC drivers are offline.\n");
			return 0;
		}
	} while(idx);

	// If here, drivers are online
	*(data->TMCConnection) = 1;

	// Perform a status register write to clear reset flag
	idx= tmc_drivers_count;
	do {
		--idx;
		txData.datagram[idx].regAdr = 0x01; // GSTAT reg
		txData.datagram[idx].value = 0x07; // To reset all 3 WC flags (Write to Clear)
	} while(idx);
	tmc_spi_write_all(false);

	idx= tmc_drivers_count;
	do {
		--idx;
		txData.datagram[idx].regAdr = 0x35; // RAMP_STAT reg
		txData.datagram[idx].value = 0x10CC; // To reset WC bits
	} while(idx);
	tmc_spi_write_all(false);

	*(data->TMCError) = 0;
	// After a reset we will check if the registers need a new load. This is needed, for example,
	// if we use the "soft disable" function, that will set TOFF value to 0.
	*data->conf_done_b = false;

	return 1;
}

void tmc_drive_conf(){

	uint8_t idx;

	// Read IOIN to check if drivers are online
	idx= tmc_drivers_count;
	do {
		--idx;
		txData.datagram[idx].regAdr = 0x04; //IOIN reg
	} while(idx);
	tmc_spi_read_all(false);
	tmc_spi_read_all(false);

	idx= tmc_drivers_count;
	do {
		--idx;
		if( rxData.datagram[idx].data[3] == 0 || rxData.datagram[idx].data[3] == 0xFF){
			*(data->TMCConnection) = 0;
			DEBUG_PRINT("Trying to configure the drivers, but they're not answering.\n");
			return;
		}
	} while(idx);

	idx= tmc_drivers_count;
	// 0. GCONF
	if (!(*data->gconf_b)) {
		do {
			--idx;
			txData.datagram[idx].regAdr = 0x00;
			txData.datagram[idx].value = data->gconf[idx];
		} while(idx);
		tmc_spi_write_all(false);
		*data->gconf_b = 1;
		return;
	}
	
	// 8. GLOBAL_SCALER 0x0B
	if (!(*data->global_scaler_b)) {
		do {
			--idx;
			txData.datagram[idx].regAdr = 0x0B;
			txData.datagram[idx].value = data->global_scaler[idx];
		} while(idx);
		tmc_spi_write_all(false);
		*data->global_scaler_b = 1;
		return;
	}

	// 9. IHOLD_IRUN 0x10
	if (!(*data->ihold_irun_b)) {
		do {
			--idx;
			txData.datagram[idx].regAdr = 0x10;
			txData.datagram[idx].value = data->ihold_irun[idx];
		} while(idx);
		tmc_spi_write_all(false);
		*data->ihold_irun_b = 1;
		return;
	}
	// 10. TPOWERDOWN 0x11
	if (!(*data->tpowerdown_b)) {
		do {
			--idx;
			txData.datagram[idx].regAdr = 0x11;
			txData.datagram[idx].value = data->tpowerdown[idx];
		} while(idx);
		tmc_spi_write_all(false);
		*data->tpowerdown_b = 1;
		return;
	}
	// 11. TPWMTHRS 0x13
	if (!(*data->tpwmthrs_b)) {
		do {
			--idx;
			txData.datagram[idx].regAdr = 0x13;
			txData.datagram[idx].value = data->tpwmthrs[idx];
		} while(idx);
		tmc_spi_write_all(false);
		*data->tpwmthrs_b = 1;
		return;
	}
	// 12. TCOOLTHRS 0x14
	if (!(*data->tcoolthrs_b)) {
		do {
			--idx;
			txData.datagram[idx].regAdr = 0x14;
			txData.datagram[idx].value = data->tcoolthrs[idx];
		} while(idx);
		tmc_spi_write_all(false);
		*data->tcoolthrs_b = 1;
		return;
	}
	// 13. THIGH 0x15
	if (!(*data->thigh_b)) {
		do {
			--idx;
			txData.datagram[idx].regAdr = 0x15;
			txData.datagram[idx].value = data->thigh[idx];
		} while(idx);
		tmc_spi_write_all(false);
		*data->thigh_b = 1;
		return;
	}
	// 15. AMAX 0x26
	if (!(*data->amax_b)) {
		do {
			--idx;
			txData.datagram[idx].regAdr = 0x26;
			txData.datagram[idx].value = data->amax[idx];
		} while(idx);
		tmc_spi_write_all(false);
		*data->amax_b = 1;
		return;
	}
	// 16. VDCMIN 0x33){
	if (!(*data->vdcmin_b)) {
		do {
			--idx;
			txData.datagram[idx].regAdr = 0x33;
			txData.datagram[idx].value = data->vdcmin[idx];
		} while(idx);
		tmc_spi_write_all(false);
		*data->vdcmin_b = 1;
		return;
	}
	// 17. SWMODE 0x34
	if (!(*data->swmode_b)) {
		do {
			--idx;
			txData.datagram[idx].regAdr = 0x34;
			txData.datagram[idx].value = data->sw_mode[idx];
		} while(idx);
		tmc_spi_write_all(false);
		*data->swmode_b = 1;
		return;
	}
	// 1. RAMP_STAT 0x35
	if (!(*data->ramp_stat_b)) {
		do {
			--idx;
			txData.datagram[idx].regAdr = 0x35;
			txData.datagram[idx].value = data->ramp_stat[idx];
		} while(idx);
		tmc_spi_write_all(false);
		*data->ramp_stat_b = 1;
		return;
	}
	// 19. ENCMODE 0x38
	if (!(*data->encmode_b)) {
		do {
			--idx;
			txData.datagram[idx].regAdr = 0x38;
			txData.datagram[idx].value = data->encmode[idx];
		} while(idx);
		tmc_spi_write_all(false);
		*data->encmode_b = 1;
		return;
	}
	// 21. ENC_CONST 0x3A
	if (!(*data->enc_const_b)) {
		do {
			--idx;
			txData.datagram[idx].regAdr = 0x3A;
			txData.datagram[idx].value = data->enc_const[idx];
		} while(idx);
		tmc_spi_write_all(false);
		*data->enc_const_b = 1;
		return;
	}
	// 22. ENC_DEVIATION 0x3D{
	if (!(*data->enc_deviation_b)) {
		do {
			--idx;
			txData.datagram[idx].regAdr = 0x3D;
			txData.datagram[idx].value = data->enc_deviation[idx];
		} while(idx);
		tmc_spi_write_all(false);;
		*data->enc_deviation_b = 1;
		return;
	}
	// 23. CHOPCONF 0x6C
	if (!(*data->chopconf_b)) {
		do {
			--idx;
			txData.datagram[idx].regAdr = 0x6C;
			txData.datagram[idx].value = data->chopconf[idx];
		} while(idx);
		tmc_spi_write_all(false);
		*data->chopconf_b = 1;
		return;
	}
	// 24. COOLCONF 0x6D
	if (!(*data->coolconf_b)) {
		do {
			--idx;
			txData.datagram[idx].regAdr = 0x6D;
			txData.datagram[idx].value = data->coolconf[idx];
		} while(idx);
		tmc_spi_write_all(false);
		*data->coolconf_b = 1;
		return;
	}
	// 26. PWMCONF 0x70
	if (!(*data->pwmconf_b)) {
		do {
			--idx;
			txData.datagram[idx].regAdr = 0x70;
			txData.datagram[idx].value = data->pwmconf[idx];
		} while(idx);
		tmc_spi_write_all(false);
		*data->pwmconf_b = 1;
		return;
	}

	// 31. END CONFIG
	*data->conf_done_b = true;
	return;
}

void tmc_read(){

	uint8_t idx;
	bool can_check_connection = false;

	// 0. Error management
	// The first thing to do is to manage errors, e-stop and reset signals.
	// Reset drivers on reset request
	if(*data->TMCReset){
		tmc_reset();
		return;
	}
	
	// Quit if TMCError is active or if the drivers are offline
	if(*data->TMCError || (!(*data->TMCConnection)))
		return;

	// 1. Configuration management.
	// If the driver are not configured, configure one register then return. Repeat as often as necessary.
	if(!(*data->conf_done_b)) {
		tmc_drive_conf();
		return;
	}

	// Before reading, check if the returned data can be compared to txBackup to check if the connection is alive.
	// If the data sent in the previous SPI write call are the same as the one returned by next SPI read call,
	// then we're all good.
	if(txBackup_value_is_known)
		can_check_connection = true;
	
	// 2. Position reading
	// If daisy-chain mode, switch CS, send dummy data to get XACTUAL data in rxBuffer, switch CS.
	// Else if individual mode, send read XACTUAL request to all moving driver, then send dummy data
	// to all moving driver to get XACTUAL data in rxBuffer and copy in feedback variable. Should the
	// connection to the driver be faulty, the pos-feedback would be zero and trigger a LCNC position
	// error; I consider this as good enougth if this module gets the information and try to quick-stop
	// the axis during next call.
	idx = tmc_drivers_count;
	do {
		--idx;
		txData.datagram[idx].regAdr = 0x21; //XACTUAL reg
	} while(idx);
	tmc_spi_read_all(false);
	
	// Compare backup txData with rxData; they are equal if the driver are responding. Use it as Wachtdog.
	if (can_check_connection){
		idx= tmc_drivers_count;
		do {
			--idx;
			if(rxData.datagram[idx].value != txBackup[idx]){
				//I think we lost connection
				rtapi_print_msg(RTAPI_MSG_ERR,"Connection lost with driver: %1$u ! \n", idx );
				*(data->TMCConnection) = 0;
				return;
			}
		} while(idx);
	}

	// The second read sends back the data latched at first read.
	tmc_spi_read_all(false);
	
	// If for any reason the driver sends back a position "0", simply ignore the information and don't update
	// the position. Not sure if this could be a problem in some cases; it could be good to set a bit to disable
	// speed cmd update if we are not sure about axis position.
	idx = tmc_drivers_count;
	do {
		--idx;
	 	float axis_pos = (float)((int)(rxData.datagram[idx].value))/(data->pos_scale[idx]);
	 	if(axis_pos!=0.0f) {
	 		*(data->pos_fb[idx]) = last_pos[idx] = axis_pos;
	 	}else{
	 		*(data->pos_fb[idx]) = last_pos[idx];
	 	}
	} while(idx);

	// 3. Status checking.
	// For each driver, read the status bit returned by last read and update the correspondig hal status bits
	// (drv_error, reset_flag, sg2, stop_l_enable, stop_r_enable)
	// If one of the driver has a drv_error or a reset_flag, set TMCError TRUE. The axis will start EStop on next
	// function call. The Error is debounced by waiting for 2 successive errors.
	// If sg2, stop_l or stop_r is set, set corresponding HAL bit.
	
	idx = tmc_drivers_count;
	do {
		--idx;
		if((*(data->tmc_status[idx]) & (1 << 0) ) || (*(data->tmc_status[idx]) & ( 1 << 1 ))) {	
			if(hadErrorFlag) {
				*(data->TMCError) = true;
				//fprintf(stderr,"DRV_error or Reset_flag! %1$u \n", *(data->tmc_status[idx]), strerror(errno)) ;
			}else{
				hadErrorFlag = true;
			}
			return;
		}
		if((*(data->tmc_status[idx]) & ( 1 << 2 ))) {*(data->tmc_sg2[idx]) = true;}else{*(data->tmc_sg2[idx]) = false;}
		if((*(data->tmc_status[idx]) & ( 1 << 6 ))) {*(data->tmc_stop_l[idx]) = true;}else{*(data->tmc_stop_l[idx]) = false;}
		if((*(data->tmc_status[idx]) & ( 1 << 7 ))) {*(data->tmc_stop_r[idx]) = true;}else{*(data->tmc_stop_r[idx]) = false;}
	} while(idx);
	hadErrorFlag = false;
	//End of the read routine
}

void tmc_write(){

	uint8_t idx;
	bool dir_change = false;

	// If there is any error in the module or if the drivers are not yet configured, do nothing.
	// Quit if TMCError is active
	if( *data->TMCError || (!(*data->TMCConnection)) || (!(*data->conf_done_b)))
		return;

	// If soft disable active, set TOFF 0 once, reset chopconf configuration flag and quit;
  	if(*data->TMCDisable && *data->chopconf_b){
		idx= tmc_drivers_count;
		do {
			--idx;
			txData.datagram[idx].regAdr = 0x6C; // RAMP_STAT reg
			txData.datagram[idx].value = (txData.datagram[idx].value = data->chopconf[idx] &~ 0b1111); // To reset WC bits
		} while(idx);
		tmc_spi_write_all(false);
		*data->chopconf_b = false;
		return;
	}

	// If chopconf config flag is not set and TMCDisable is not anymore active, reset
	// conf done flag to force drivers configuration to restore chopconf.
	if(!*data->chopconf_b){
		if (!*data->TMCDisable){
			*data->conf_done_b = false;
		}
		return;
	}

	// Set direction if there was at least one change since last write (RAMPMODE 1 or 2)
	idx = tmc_drivers_count;
	do {
		--idx;
		txData.datagram[idx].regAdr = 0x20; // RAMPMODE reg
		if(*(data->vel_cmd[idx])<0.0f){
			txData.datagram[idx].value = 2; // Negative direction
			if(!data->tmc_reverse_dir[idx]){
				data->tmc_reverse_dir[idx] = true;
				dir_change = true;
			}
		}else{
			txData.datagram[idx].value = 1; // Positive direction
			if(data->tmc_reverse_dir[idx]){
				data->tmc_reverse_dir[idx] = false;
				dir_change = true;
			}
		}
	} while(idx);
	
	if (dir_change) {
		tmc_spi_write_all(false);
		dir_change = false;
	}
	
	// Set VMAX according to cmd_vel for each driver. 
	//uint32_t txBackup[data->tmc_drivers_count];
	idx= tmc_drivers_count;
	do {
		--idx;
		txData.datagram[idx].regAdr = 0x27; //VMAX reg
		txData.datagram[idx].value = txBackup[idx] = MIN(MAX_TMC_VEL, abs(speedFromUS(*(data->vel_cmd[idx]), idx)));
	} while(idx);

	tmc_spi_write_all(false);

	// Use a flag to determine that I can use the value saved in txBackup for communication check. Any SPI read or
	// write will reset the flag. If before the next read the flag is still high, it is safe to compare txBackup
	// with data send back from next SPI read call to check communication.
	txBackup_value_is_known = 1;
}

void tmc_debug_read(){

	uint8_t idx;
	uint8_t reg_to_read, offset = 0;
	uint32_t mask = 0;

	switch(*data->tmc_debug_mode){
		// Mode = 0 => 8 first bits of IOIN
		case 0:
			reg_to_read = 0x04;
			mask = 0xFF;
			break;

		// Mode = 1 => TSTEP value
		case 1:
			reg_to_read = 0x12;
			mask = 0xFFFFF;
			break;

		// Mode = 2 => XLATCH value
		case 2:
			reg_to_read = 0x36;
			mask = 0xFFFFFFFF;
			break;

		// Mode = 3 => 8 first bits of RAMP_STAT
		case 3:
			reg_to_read = 0x35;
			mask = 0xFF;
			break;

		// Mode = 4 => XENC value
		case 4:
			reg_to_read = 0x39;
			mask = 0xFFFFFFFF;
			break;

		// Mode = 5 => ENC_DEVIATION value
		case 5:
			reg_to_read = 0x3D;
			mask = 0xFFFFF;
			break;

		// Mode = 6 => PWM_OFS_AUTO value
		case 6:
			reg_to_read = 0x72;
			mask = 0xFF;
			break;

		// Mode = 7 => PWM_GRAD_AUTO value
		case 7:
			reg_to_read = 0x72;
			mask = 0xFF;
			offset = 9;
			break;

		// Mode = 8 => LOST_STEPS value
		case 8:
			reg_to_read = 0x73;
			mask = 0xFFFFF;
			break;

		// Mode = 9 => SG_RESULT value
		case 9:
			reg_to_read = 0x6F;
			mask = 0x1F;
			break;

		// Mode = 10 => CS_ACTUAL value
		case 10:
			reg_to_read = 0x6F;
			mask = 0xFF;
			offset = 16;
			break;

		// Default return chip version
		default:
			reg_to_read = 0x04;
			mask = 0xFF;
			offset = 24;
			break;
	}
	
	idx = tmc_drivers_count;
	do {
		--idx;
		txData.datagram[idx].regAdr = reg_to_read; //XACTUAL reg
	} while(idx);
	tmc_spi_read_all(false);
	tmc_spi_read_all(false);

	idx = tmc_drivers_count;
	do {
		--idx;
		*data->tmc_debug_out[idx] = (rxData.datagram[idx].value & (mask << offset)) >> offset;
	} while(idx); 

}

void tmc_spi_read_all (){

	uint8_t idx,idy,i = 0;
	const size_t packetLen = 5;

	txBackup_value_is_known = 0;

	for (int i=0 ; i<MAX_TMC_DRIVER; i++){
		if(chains[i]>0){
			
			size_t first_byte_chain = packetLen * idx;
			size_t total_bytes = packetLen * chains[i];
			uint8_t in[total_bytes];
			uint8_t out[total_bytes];

			idy = chains[i];
			do{
				--idy;
				out[idy+0] = txData.datagram[idx+idy].regAdr;
				out[idy+1] = 0;
				out[idy+2] = 0;
				out[idy+3] = 0;
				out[idy+4] = 0;
			}while(idy);

			if(bcm){
				bcm2835_gpio_clr(cs_pins[i]);
				bcm2835_spi_transfernb(out, in, total_bytes);
			}else if(rp1){
				rp1spi_transfer(SPI_num, i, out, in, total_bytes);
			}

			idy = chains[i];
			do{
				--idy;
				*(data->tmc_status[idx+idy]) = rxData.datagram[idx+idy].regAdr = in[idy+0];
				rxData.datagram[idx+idy].data[3] = in[idy+1];
				rxData.datagram[idx+idy].data[2] = in[idy+2];
				rxData.datagram[idx+idy].data[1] = in[idy+3];
				rxData.datagram[idx+idy].data[0] = in[idy+4];
			}while(idy);
			
			if(bcm){
				bcm2835_gpio_set(cs_pins[i]);
			}
			idx += chains[i];
		}else{
			break;
		}
	}
}

void tmc_spi_write_all () {

	uint8_t idx,idy,i = 0;
	const size_t packetLen = 5;

	txBackup_value_is_known = 0;

	for (i=0 ; i<MAX_TMC_DRIVER; i++){
		if(chains[i]>0){
			
			size_t first_byte_chain = packetLen * idx;
			size_t total_bytes = packetLen * chains[i];
			uint8_t in[total_bytes];
			uint8_t out[total_bytes];

			idy = chains[i];
			do{
				--idy;
				out[idy+0] = txData.datagram[idx+idy].regAdr | 0x80;
				out[idy+1] = txData.datagram[idx+idy].data[3];
				out[idy+2] = txData.datagram[idx+idy].data[2];
				out[idy+3] = txData.datagram[idx+idy].data[1];
				out[idy+4] = txData.datagram[idx+idy].data[0];
			}while(idy);

			if(bcm){
				bcm2835_gpio_clr(cs_pins[i]);
				bcm2835_spi_transfernb(out, in, total_bytes);
			}else if(rp1){
				rp1spi_transfer(SPI_num, i, out, in, total_bytes);
			}

			idy = chains[i];
			do{
				--idy;
				*(data->tmc_status[idx+idy]) = rxData.datagram[idx+idy].regAdr = in[idy+0];
				rxData.datagram[idx+idy].data[3] = in[idy+1];
				rxData.datagram[idx+idy].data[2] = in[idy+2];
				rxData.datagram[idx+idy].data[1] = in[idy+3];
				rxData.datagram[idx+idy].data[0] = in[idy+4];;
			}while(idy);
			
			if(bcm){
				bcm2835_gpio_set(cs_pins[i]);
			}
			idx += chains[i];
		}else{
			break;
		}
	}
}
