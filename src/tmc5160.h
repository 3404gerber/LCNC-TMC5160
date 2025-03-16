#define MAX_TMC_DRIVER      6
#define SPIBUFSIZE			    5 * MAX_TMC_DRIVER
#define MAX_TMC_ACCEL       65535
#define MAX_TMC_VEL         8388096

#define MODNAME "tmc5160"
#define PREFIX "tmc5160"
#define RPI5_RP1_PERI_BASE 0x7c000000

MODULE_AUTHOR("LGE");
MODULE_DESCRIPTION("Driver for TMC5160")
MODULE_LICENSE("GPL v3");

typedef struct {
	hal_bit_t		*TMCReset; //Iput pin to send GSTAT register to reset drive errors and RAMP_STAT register to reset pos/stop events and latch status.
	hal_bit_t		*TMCError; //Output pin to signal driver error.
  hal_bit_t   *TMCDisable; //Input pin for driver soft disable.
  hal_bit_t   *TMCConnection; //Output bit indicating alive connection with TMC driver
	hal_float_t *vel_cmd[MAX_TMC_DRIVER];			// pin: velocity command (position units/sec)
	hal_float_t *pos_fb[MAX_TMC_DRIVER];			// pin: position feedback (raw counts)
  hal_float_t pos_scale[MAX_TMC_DRIVER];			// param: steps per position unit
	bool			  tmc_reverse_dir[MAX_TMC_DRIVER]; // used to indicate that at least one axis has changed direction since last write

  float 			old_scale[MAX_TMC_DRIVER];			// stored scale value
	double 			scale_recip_pos[MAX_TMC_DRIVER];  // reciprocal value used for speed scaling
  double 			scale_recip_vel[MAX_TMC_DRIVER];  // reciprocal value used for velocity scaling

	//TMC-config
  //TMC Writable Registers
  hal_u32_t   gconf[MAX_TMC_DRIVER];          //0 : 0x00
  hal_u32_t   nodeconf[MAX_TMC_DRIVER];       //1 : 0x03 *
  hal_u32_t   output[MAX_TMC_DRIVER];         //2 : 0x04 *
  hal_u32_t   x_compare[MAX_TMC_DRIVER];      //3 : 0x05 * ->PIN
  hal_u32_t   otp_prog[MAX_TMC_DRIVER];       //4 : 0x06 *
  hal_u32_t   factory_conf[MAX_TMC_DRIVER];   //5 : 0x08 *
  hal_u32_t   short_conf[MAX_TMC_DRIVER];     //6 : 0x09 *
  hal_u32_t   drv_conf[MAX_TMC_DRIVER];       //7 : 0x0A *
  hal_u32_t   global_scaler[MAX_TMC_DRIVER];  //8 : 0x0B
  hal_u32_t   ihold_irun[MAX_TMC_DRIVER];     //9 : 0x10 -> PIN
  hal_u32_t   tpowerdown[MAX_TMC_DRIVER];     //10 : 0x11
  hal_u32_t   tpwmthrs[MAX_TMC_DRIVER];       //11 : 0x13
  hal_u32_t   tcoolthrs[MAX_TMC_DRIVER];      //12 : 0x14
  hal_u32_t   thigh[MAX_TMC_DRIVER];          //13 : 0x15
  hal_u32_t   xactual[MAX_TMC_DRIVER];        //14 : 0x21 *
  hal_u32_t   amax[MAX_TMC_DRIVER];           //15 : 0x26 ->PIN
  hal_u32_t   vdcmin[MAX_TMC_DRIVER];         //16 : 0x33
  hal_u32_t   sw_mode[MAX_TMC_DRIVER];        //17 : 0x34 ->PIN
  hal_u32_t   ramp_stat[MAX_TMC_DRIVER];      //18 : 0x35 ->PIN
  hal_u32_t   encmode[MAX_TMC_DRIVER];        //19 : 0x38
  hal_u32_t   x_enc[MAX_TMC_DRIVER];          //20 : 0x39 *
  hal_u32_t   enc_const[MAX_TMC_DRIVER];      //21 : 0x3A
  hal_u32_t   enc_deviation[MAX_TMC_DRIVER];  //22 : 0x3D
  hal_u32_t   chopconf[MAX_TMC_DRIVER];       //23 : 0x6C
  hal_u32_t   coolconf[MAX_TMC_DRIVER];       //24 : 0x6D
  hal_u32_t   dcctrl[MAX_TMC_DRIVER];         //25 : 0x6E *
  hal_u32_t   pwmconf[MAX_TMC_DRIVER];        //26 : 0x70

  // Debug out
  hal_s32_t   *tmc_debug_mode; // see tmc_debug_read() function comment below
  hal_u32_t   *tmc_debug_out[MAX_TMC_DRIVER]; // Output pin for debug function

  // Sensorless homing
  hal_bit_t   *tmc_index_enable[MAX_TMC_DRIVER]; // I/O bit for sensorless homing
  hal_u32_t   tmc_homing_current[MAX_TMC_DRIVER]; // Current used for homing, in percent. (30-70 recommended) 

	//Status bits
  hal_u32_t		*tmc_status[MAX_TMC_DRIVER]; // Full SPI status word
  hal_bit_t		*tmc_reset_flag[MAX_TMC_DRIVER]; // Reset flag from SPI status word
	hal_bit_t		*tmc_driver_error[MAX_TMC_DRIVER]; // Driver_error flag from SPI status word
	hal_bit_t		*tmc_sg2[MAX_TMC_DRIVER]; // StallGuard2 flag from SPI status word
	hal_bit_t		*tmc_stop_l[MAX_TMC_DRIVER]; // Left limit switch flag from SPI status word
	hal_bit_t		*tmc_stop_r[MAX_TMC_DRIVER]; // Right limit switch flag from SPI status word

  //TMC Conf bits
  hal_bit_t   *gconf_b;         //bit0
  hal_bit_t   *gstat_b;         //bit1
  hal_bit_t   *nodeconf_b;      //bit2
  hal_bit_t   *output_b;        //bit3
  hal_bit_t   *x_compare_b;     //bit4
  hal_bit_t   *factory_conf_b;  //bit5
  hal_bit_t   *short_conf_b;    //bit6
  hal_bit_t   *drv_conf_b;      //bit7
  hal_bit_t   *global_scaler_b; //bit8
  hal_bit_t   *ihold_irun_b;    //bit9
  hal_bit_t   *tpowerdown_b;    //bit10
  hal_bit_t   *tpwmthrs_b;      //bit11
  hal_bit_t   *tcoolthrs_b;     //bit12
  hal_bit_t   *thigh_b;         //bit13
  hal_bit_t   *xactual_b;       //bit14
  hal_bit_t   *amax_b;          //bit15
  hal_bit_t   *vdcmin_b;        //bit16
  hal_bit_t   *swmode_b;        //bit17
  hal_bit_t   *ramp_stat_b;     //bit18
  hal_bit_t   *encmode_b;       //bit19
  hal_bit_t   *x_enc_b;         //bit20
  hal_bit_t   *enc_const_b;     //bit21
  hal_bit_t   *enc_deviation_b; //bit22
  hal_bit_t   *chopconf_b;      //bit23
  hal_bit_t   *coolconf_b;      //bit24
  hal_bit_t   *pwmconf_b;       //bit25
  hal_bit_t   *bit26;           //not used yet
  hal_bit_t   *bit27;           //not used yet
  hal_bit_t   *bit28;           //not used yet
  hal_bit_t   *bit29;           //not used yet
  hal_bit_t   *bit30;           //not used yet
  hal_bit_t   *conf_done_b;     //bit31

} data_t;

static data_t *data;

// Struct and Union for SPI communication with drivers
// We have one TX buffer and one RX buffer, containing the data to send (and received data) for all the
// drivers.
typedef struct {
    uint8_t regAdr;
    union{
      uint32_t value;
      uint8_t data[4];
    };
} tmcSPIDatagram_t;

typedef union {
    uint8_t buffer[SPIBUFSIZE];
    tmcSPIDatagram_t datagram[MAX_TMC_DRIVER];
} SPIData_t;

static SPIData_t txData, rxData;

// States for stallguard homing. Highly inspired from the original homing in LinuxCNC.
typedef enum {
  HOME_IDLE = 0,
  HOME_SET_IRUN, // 1
  HOME_SET_SW_MODE,// 2
  HOME_WAIT_EVENT_SG,// 3
  HOME_SET_XACTUAL,// 4
  HOME_SET_XENC,// 5
  HOME_RESTORE_IRUN,// 6
  HOME_RESTORE_SW_MODE,// 7
  HOME_RESET_DRIVERS, // 8
  HOME_FINISHED// 9
} homing_state_t;

static homing_state_t homing_state[MAX_TMC_DRIVER] = {0}; 
static bool         sg_stop_read = false; // A flip-flop variable while waiting on StallGuard stop event

/* other globals */
static int 			    comp_id;				    // component ID
static const char 	*modname = MODNAME;
static const char 	*prefix = PREFIX;

static bool			    bcm;					      // use BCM2835 driver
static bool			    rp1;					      // use RP1 driver
static bool         sg_homing;          // Do we use sensorless homing for at least one driver?
static bool         encoders;           // Do we use encoder position for at least one driver?
static bool         has_encoder[MAX_TMC_DRIVER];       // Array to know which driver uses encoder

static uint8_t      tmc_drivers_count;  // number of TMC drivers declared, used for loops

// TX buffer backup is used to check if the connection is ok. If TX last write = RX current read, we are online.
static uint32_t     txBackup[MAX_TMC_DRIVER];

// As multiple function execute SPI reading (conf, debug, homing), this bit is used to ensure that what is in
// the txBackup variable is the data sent by the last tmc_write call. This bit is reseted by each SPI read/call.
static bool         txBackup_value_is_known; 

// Backup of the last axis positions. Sometimes, the drivers send back 0 data. If so, ignore the read and just
// use last_pos as current pos. As the drivers are initialised with XACTUAL = 0x01, the position that is minus one
// increment away from the homing position is not accessible, as it will return last position. Should it be a problem,
// one could initialise the XACTUAL at a different value.
static float		    last_pos[MAX_TMC_DRIVER]={0};

/***********************************************************************
*                  COMP ARGUMENTS DECLARATIONS                         *
************************************************************************/
int chains[MAX_TMC_DRIVER] = { 0 };
RTAPI_MP_ARRAY_INT(chains,MAX_TMC_DRIVER,"List of TMC5160 daisy chains driver number");

int cs_pins[MAX_TMC_DRIVER] = { 0 };
RTAPI_MP_ARRAY_INT(cs_pins,MAX_TMC_DRIVER,"Chip select pins list. You need one per chain");

int sg2_homing[MAX_TMC_DRIVER] = { 0 };
RTAPI_MP_ARRAY_INT(sg2_homing,MAX_TMC_DRIVER,"Activate StallGuard2 sensorless homing, per driver");

//If not using internal 12MHz clock
int TMC_freq = 12000000;
RTAPI_MP_INT(TMC_freq, "TMC frequency");

// If an encoder is connected to the TMC5160 and should be used for position feedback, set 1.
// If you only want to use an encoder to check loss steps, set 0 here and configure registers accordingly.
int use_encoder[MAX_TMC_DRIVER] = { 0 };
RTAPI_MP_ARRAY_INT(use_encoder,MAX_TMC_DRIVER,"Activate encoder position reading, per driver");

// Specific for BCM based SPI (Raspberry Pi 4)
int SPI_clk_div = 128;
RTAPI_MP_INT(SPI_clk_div, "SPI clock divider");

// Specific for RP1 based SPI (Raspberry Pi 5)
int SPI_num = 0;
RTAPI_MP_INT(SPI_num, "SPI number");

int CS_num = 0;
RTAPI_MP_INT(CS_num, "CS number");

int32_t SPI_freq = -1;
RTAPI_MP_INT(SPI_freq, "SPI frequency");

/***********************************************************************
*                  LOCAL FUNCTION DECLARATIONS                         *
************************************************************************/
// Will determine wether the board is a RPi4 or RPi5, to choose between bcm2835 and rp1 library
static int rt_detect_board(void);

// Initialises bcm2835 or rp1
static int rt_peripheral_init(void);

static int rt_bcm2835_init(void);

static int rt_rp1lib_init(void);

// Initialises the connection to TMC5160 drivers
static int rt_tmc_init(void);

// StallGuard2 homing function. Largely inspired by the LinuxCNC homing.c. The idea is to keep the thread time low.
// It is called from inside the tmc_read() function, so if there was a SPI read/write call in the homing function,
// the tmc_read() will exit without doing the usual position read. This can lead to some minimal delay in the position
// feedback but should never be a problem. While waiting for the StallGuard2 event that will inform us that the axis is
// stopped, we switch between status reading and position reading on each function call. The homing uses the index only
// homing mode.
static bool tmc_sg2_homing();

// The tmc_read() function is called once per cycle. It will check for reset request or errors and configure the driver
// if they aren't. But the main purpose will be to read the current driver position stored in the XACUAL
// register and update the driver status, and this will be the done during each call if everything else is
// running normally. It will also check that the drivers are online and actualise the TMC status bits.
static void tmc_read();

// The tmc_write() function sends the vel_cmd to the TMC driver. it will execute only if there are no errors and
// if the conf is done. The function also handles the drivers soft disable.
static void tmc_write();

// This function is used during tuning. It will output different values according to input parameter "Mode":
// Mode = 0 => 8 first bits of IOIN
// Mode = 1 => TSTEP value
// Mode = 2 => XLATCH value
// Mode = 3 => 8 first bits of RAMP_STAT
// Mode = 4 => XENC value
// Mode = 5 => ENC_DEVIATION value
// Mode = 6 => PWM_OFS_AUTO value
// Mode = 7 => PWM_GRAD_AUTO value
// Mode = 8 => LOST_STEPS value
// Mode = 9 => SG_RESULT value
// Mode = 10 => CS_ACTUAL value
static void tmc_debug_read();

//This routine writes dummy data to the register specified in txData buffer to the TMC drives in READ mode. Data for the last drive in the chain goes out first.
//After the read process, the data in the rxData buffer contains the value of the last read routine; if you just want to read data from a register, call
//the routine twice with the same txData and read the rxData after the second read process. If you want to read two register, you can write the address
//of the second register after the first read, read the data of the first register after the second read and the data of the second register will be in the
//third read routine. And you can continue like this if you need more.

//This routine writes the data in the txData buffer to the TMC drives in WRITE mode. Data for the last drive in the chain goes out first.
//After the write process, the data in the rxData buffer contains the value of the last read routine. The data sent back during next write
//or read routine should match the one sent here and this is an easy way to check if the drives are online: backup txData and compare it
//to rxData.
static void tmc_spi_rw_all (bool write);

// The tmc_drive_conf() will configure the connected TMC driver. To avoid overshooting the thread time,
// the routine will set only one register on each call, using conf bits to keep track of what have been done.
// Setting the last bit will indicate to other routines that the configuration is done. Many register are write
// only and it is impossible to check if the value have been set, but we will check the communication every
// time before sending a register configuration.
static void tmc_drive_conf();

// General reset of the tMC drivers. It will Write to Clear the 3 error bit in GSTAT and WC the status and events bits in RAMP_STAT.
// ! A reset is mandatory after a connection loss. If TMCError was active, it will also force all registers to load again.
static bool tmc_reset();

// Convert speed Unit-per-Second to TMC internal speed. We only use velocity, as LinuxCNC takes car of the ramp. Max acceleration
// can be set in AMAX register.
// TMC datasheet §14.1 Real world unit conversions:
// v[Hz] = v[5160A] * ( f CLK [Hz]/2 / 2^23 )
// a[Hz/s] = a[5160A] * f CLK [Hz]^2 / (512*256) / 2^24
// This function is not used anymore, as we calculate the velocity recip to avoid division. Keep it here for information
static int32_t speedFromUS(float speedUS, uint8_t idx) { return (int32_t)(speedUS / ((float)TMC_freq / (float)(1ul << 24)) * (float)data->pos_scale[idx]); }
