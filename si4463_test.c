/*
 * port_init.c
 *
 *  Created on: 2019年9月20日
 *      Author: NEUQ
 */
/***************************** Include Files *********************************/

#include "xparameters.h"
#include "xgpio.h"
#include "xil_printf.h"
#include "delay.h"
#include "Si4463_config.h"
/************************** Constant Definitions *****************************/

#define SDN 	0x40
#define nIRQ 	0x20
#define nSEL	0x10
#define SCLK	0x08
#define SDI		0x04
#define SDO		0x02
#define GPIO1	0x01
#define PART_INFO                       0x01
#define FUNC_INFO                       0x10
#define SET_PROPERTY                    0x11
#define GET_PROPERTY                    0x12
#define GPIO_PIN_CFG                    0x13
#define GET_ADC_READING                 0x14
#define FIFO_INFO                       0x15
#define PACKET_INFO                     0x16
#define IRCAL                           0x17
#define PROTOCOL_CFG                    0x18
#define GET_INT_STATUS                  0x20
#define GET_PH_STATUS                   0x21
#define GET_MODEM_STATUS                0x22
#define GET_CHIP_STATUS                 0x23
#define START_TX                        0x31
#define START_RX                        0x32
#define REQUEST_DEVICE_STAT             0x33
#define CHANGE_STATE                    0x34
#define READ_CMD_BUFF                   0x44
#define FRR_A_READ                      0x50
#define FRR_B_READ                      0x51
#define FRR_C_READ                      0x53
#define FRR_D_READ                      0x57
#define WRITE_TX_FIFO                   0x66
#define READ_RX_FIFO                    0x77
#define START_MFSK                      0x35
#define RX_HOP                          0x36

#define payload_length  				14

#define freq_channel		0

typedef struct
{
	unsigned char reach_1s				: 1;
	unsigned char is_tx					: 1;
	unsigned char rf_reach_timeout		: 1;
}FlagType;
//const unsigned char RF_MODEM_DSA_CTRL1_5_data[] = 		{RF_MODEM_DSA_CTRL1_5};
const unsigned char RF_FREQ_CONTROL_INTE_8_data[] = 		{RF_FREQ_CONTROL_INTE_8};
const unsigned char RF_POWER_UP_data[] = 			   		{ RF_POWER_UP};

const unsigned char RF_FRR_CTL_A_MODE_4_data[] = 		   	{ RF_FRR_CTL_A_MODE_4};

const unsigned char RF_MODEM_FREQ_DEV_0_1_data[] = 		   	{ RF_MODEM_FREQ_DEV_0_1};
const unsigned char RF_MODEM_AGC_CONTROL_1_data[] = 		{ RF_MODEM_AGC_CONTROL_1};
const unsigned char RF_MODEM_MOD_TYPE_12_data[]=			{RF_MODEM_MOD_TYPE_12};
const unsigned char RF_MODEM_TX_RAMP_DELAY_12_data[]=				{RF_MODEM_TX_RAMP_DELAY_12};
const unsigned char BCR_NCO_OFFSET_2_12_data[]=					{RF_MODEM_BCR_NCO_OFFSET_2_12};
const unsigned char RF_MODEM_AFC_LIMITER_1_3_data[]=						{RF_MODEM_AFC_LIMITER_1_3};
const unsigned char AGC_WINDOW_SIZE_12_data[]=				{RF_MODEM_AGC_WINDOW_SIZE_12};
const unsigned char RF_MODEM_RAW_CONTROL_8_data[]=					{RF_MODEM_RAW_CONTROL_8};
const unsigned char COE13_7_0_12_data[]=	{RF_MODEM_CHFLT_RX1_CHFLT_COE13_7_0_12};
const unsigned char COE1_7_0_12_data[]=	{RF_MODEM_CHFLT_RX1_CHFLT_COE1_7_0_12};
const unsigned char COE7_7_0_12_data[]=	{RF_MODEM_CHFLT_RX2_CHFLT_COE7_7_0_12};
const unsigned char RF_SYNTH_PFDCP_CPFF_7_data[]=					{RF_SYNTH_PFDCP_CPFF_7};
const unsigned char RF_MODEM_RAW_SEARCH2_2_data[]={RF_MODEM_RAW_SEARCH2_2};

//const unsigned char tx_test_aa_data[14] = {0xaa,0xaa,0xaa,0xaa,0xaa,0xaa,0xaa,0xaa,0xaa,0xaa,0xaa,0xaa,0xaa,0xaa};
const unsigned char tx_ph_data[14] = {'s','w','w','x',0x41,0x42,0x43,0x44,0x45,0x46,0x47,0x48,0x49,0x6d};  //
FlagType	Flag;

u16	count_1hz, rf_timeout;
u8 spi_read_buf[20];
u8 rx_buf[25];
//U8 mode;

/*
 * The following constants map to the XPAR parameters created in the
 * xparameters.h file. They are defined here such that a user can easily
 * change all the needed parameters in one place.
 */

#define GPIO_ID  XPAR_GPIO_0_DEVICE_ID

/*
 * The following constant is used to wait after an LED is turned on to make
 * sure that it is visible to the human eye.  This constant might need to be
 * tuned for faster or slower processor speeds.
 */

#define LED_DELAY     10000000

/*
 * The following constant is used to determine which channel of the GPIO is
 * used for the LED if there are 2 channels supported.
 */

#define GPIO_CHANNEL 1

/**************************** Type Definitions *******************************/


/***************** Macros (Inline Functions) Definitions *********************/

/************************** Function Prototypes ******************************/
int gpio_init(u16 DeviceId);
void Si4463_Reset(void);
int Si4463_Init(void);
u8 spi_byte(unsigned char data);
/************************** Variable Definitions *****************************/

/*
 * The following are declared globally so they are zeroed and so they are
 * easily accessible from a debugger
 */

XGpio Gpio; /* The Instance of the GPIO Driver */

/*****************************************************************************/
/**
*
* The purpose of this function is to illustrate how to use the GPIO
* driver to driver Si4463;
*
* @param	None
*
* @return	0
*
* @note		This function will not return if the test is running.
*
******************************************************************************/

int main(void)
{
	int status;
	xil_printf("start run the si4463_test");
	/* Initialize the GPIO driver  */
	status = gpio_init(GPIO_ID);
	if (status != 0)
	{
		xil_printf("gpio_init failed, and the return status is %d\r\n",status);
	}
	/*Initialize the Si4463 */
	//Si4463_Reset();
	while(1)
	{
		XGpio_DiscreteSet(&Gpio,GPIO_CHANNEL,SDI);
		us_delay(10);
		XGpio_DiscreteClear(&Gpio,GPIO_CHANNEL,SDI);
		us_delay(50);
	}

}

/*****************************************************************************/
/**
*
* The purpose of this function is to init gpio
*
* @param	DeviceID of Gpio
*
* @return	0	init success
* 			-1	Gpio Initialization Failed
*			-2	DIR Set Failed
*
* @note		None
*
******************************************************************************/
int gpio_init(u16 DeviceId)
{
	int Status;
	u32 Reg_DIR;

	//初始化GPIO驱动实例
	Status = XGpio_Initialize(&Gpio, DeviceId);
	if (Status != XST_SUCCESS) {
			xil_printf("Gpio Initialization Failed\r\n");
			return -1;
		}

	//设置I/O方向
	XGpio_SetDataDirection(&Gpio, GPIO_CHANNEL,0x03);
	Reg_DIR = XGpio_GetDataDirection(&Gpio, GPIO_CHANNEL);
	if (Reg_DIR != 0x03)
	{
		xil_printf("DIR set Failed\r\n");
		return -2;
	}

	return 0;
}

/*****************************************************************************/
/**
*
* The purpose of this function is to Reset Si4463
*
* @param	None
*
* @return	None
*
* @note		None
*
******************************************************************************/
void Si4463_Reset(void)
{
	int i;
	u8 data = 0;
	//Power on Reset(POR) Si4463
	XGpio_DiscreteSet(&Gpio,GPIO_CHANNEL,SDN);
	us_delay(10);
	XGpio_DiscreteClear(&Gpio,GPIO_CHANNEL,SDN);
	XGpio_DiscreteSet(&Gpio,GPIO_CHANNEL,nSEL);
	ms_delay(5);
	XGpio_DiscreteClear(&Gpio,GPIO_CHANNEL,nSEL);
	for (i = 0; i< 7; i++)
	{
		data = spi_byte(RF_POWER_UP_data[i]);
		xil_printf("the send message is %d, and the message is %d,and the receive message is %d\r\n",
				i,RF_POWER_UP_data[i],data);
	}
	XGpio_DiscreteSet(&Gpio,GPIO_CHANNEL,nSEL);
}

int Si4463_Init (void)
{
	Si4463_Reset();

	return 0;
}


u8 spi_byte(unsigned char data)
{
	unsigned char i;

	for (i = 0; i < 8; i++)
	{
		if (data & 0x80)
			//SDI = 1;
			XGpio_DiscreteSet(&Gpio,GPIO_CHANNEL,SDI);
		else
			//SDI = 0;
			XGpio_DiscreteClear(&Gpio,GPIO_CHANNEL,SDI);

		data <<= 1;
		//SCK = 1;
		XGpio_DiscreteSet(&Gpio,GPIO_CHANNEL,SCLK);

		if ((XGpio_DiscreteRead(&Gpio,GPIO_CHANNEL) >> 1) & 0x1)
			data |= 0x01;
		else
			data &= 0xfe;

		//SCK = 0;
		XGpio_DiscreteClear(&Gpio,GPIO_CHANNEL,SCLK);
	}
	return (data);
}

