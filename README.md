# LCNC-TMC5160

Under construction.

## Disclaimer

This is a very early, only partially tested version of the module. Do not try
to use it on a machine before running some intense testing on a bench! I wouldn't
even consider myself as a hobby coder, so there are chances that they're undiscovered
bugs in the code.

## What is it?

It is a LinuxCNC module to create a SPI connection between a Raspberry Pi
and TMC5160 drivers.

With this connection, you can configure the drivers, read the current position and status
and write a velocity command. So, in other words, you can run TMC5160 stepper
motor drivers in full SPI mode, without having to generate steps and dir signals.

## Why did I do this?

I tried to achieve relatively high speedrates and accelerations (400mm/s, 2000mm/s2) with
more traditional step generator like grbl or FluidNC. Even if they did a good job at lower
speed, I had vibration and noisy axis at higher rates. The maximum step rates on those
mcu based sted generators is, most of the time, limited to a couple of hundered kHz, versus
MHz range for the TMC5160 internal step generator. More important, the update rate for
acceleration is at something like 10ms on a standard grbl setup, and you can try to go down
to 1ms. The internal ramp generator of the TMC updates each 512 clock cycle; this is 23 time
more than the optimised 1ms!

Of course, there are some downside. When using step generators you are running on hardware
realtime systems and if you have more than one axis they will be perfectly synchronised as
they use the same interrupt/clock. With full SPI mode, even if you can have very interesting
values, you will still need a PID to correct the velocity+time based positioning.

## How to use it?

First, you will need to install the component or compile it from the source. Then you can
load the module with following options:

### Loading the module

#### On RPi4:

Limitations:

- Works only on SPI0 (could probably use bcm_aux_spi if needed). 
- Limited to 6 drivers in total, but could be increased easly.

Minimal: this will configure 1 chain of 1 driver and use pin 22 as CS pin
```
loadrt tmc5160 chains=1 cs_pins=22
```

Daisy-chained: this will configure 1 chain of 3 daisy-chained drivers and use pin 22 as CS pin
```
loadrt tmc5160 chains=3 cs_pins=22
```

Normal SPI: this will configure 3 chains of 1 driver and use pins 8,22 and 5 as CS pins (one per driver)
```
loadrt tmc5160 chains=1,1,1 cs_pins=8,22,5
```

Other parameters:
- SPI_clk_div : standard is 128 for a SPI frequency of 3.125Mhz. 64 (only possible with external frequency source) would double the speed and 256 half it.
- TMC_freq : if your TMC drivers are connected to external clock (default internal 12000000)

Full options example:
```
loadrt tmc5160 chains=1 SPI_num=0 CS_num=0 SPI_clk=64 TMC_freq=16000000
```

#### On RPi5:

Limitations:

- Limited to 2 chains if using SPI0 and 3 chains if using SPI1.
- Limited to 6 drivers in total, but could be increased easly.

Minimal: this will configure 1 chain of 1 driver on SPI0 channel with CE0 as CS pin
```
loadrt tmc5160 chains=1
```

Daisy-chained: this will configure 1 chain of 3 daisy-chained drivers and SPI1 channel with CE1 as CS pin
```
loadrt tmc5160 chains=3 SPI_num=1 CS_num=1
```

Normal SPI: this will configure 2 chains of 1 driver and SPI0 channel with CE0 and CE1 as CS pin
```
loadrt tmc5160 chains=1,1
```

Other parameters:
- SPI_freq : to use different SPI speed than standard 4000000 (which is the maximum for internal clock)
- TMC_freq : if your TMC drivers are connected to external clock (default internal 12000000)

Full options example:
```
loadrt tmc5160 chains=1 SPI_num=0 CS_num=0 SPI_freq=8000000 TMC_freq=16000000
```

### Adding the functions

Add the function that reads drivers position (and does other stuff):
```
addf tmc5160.read servo-thread
```
Do your PID and other stuff here, then

Add the function that writes velocity command to the drivers:
```
addf tmc5160.write servo-thread
```

### Connecting pins and setting parameters

Have a look in halshow to see all the pins and parameters created by the module. Here are some of
the most useful:

Almost all the write registers are mapped. Use decimal value of the register.
```
setp tmc5160.chain.0.driver.0.register.**register_name** 12345
```

- tmc5160.TMC-Disable IN pin allows you to freewheel your drivers

- tmc5160.TMC-reset IN pin will reset the drivers

- tmc5160.TMC-error OUT pin will signal an error on the drivers

- tmc5160.TMC-Connection OUT pin will confirm that the drivers are online

- tmc5160.TMC-debug-mode IN pin allows you to show different values in tmc5160.chain.**X**.driver.**X**.debug_out

- Each driver has a OUT pin for status_word, and out of those 8 bit, 5 are mapped to separate PINS: reset_flag, error_flag, sg2_flag, ref_l_flag, ref_r_flag

### Changing parameters while running

To keep the execution time slow, the parameters are not set in one cycle. There are 32 bit (some not used yet) to
indicate if the register has been loaded. The last one (tmc5160.CONF-BITS.CONF_DONE_FLAG) indicates that the drivers
have been fully configured and are ready to operate.

The read function will first check if the CONF_DONE_FLAG is set. If not, it will call a drive_conf routine that will configure
one register and terminate the read function. In the next execution of the read function, the next register will be set, and so on.
Once all the register are configured, the read function can start its real job, which is position reading, and it will do it over and over.

So what if you want to try the Stealthchop function, or try a different value for HSTRT? Well, it is simple: first set your new
register value, then clear the flag corresponding to your register, for eaxample tmc5160.CONF-BITS.CHOPCONF and finally clear the
tmc5160.CONF_DONE_FLAG to force the read function to reload the register that have been "unchecked" by reseting their flags.
