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
void Si4463_Init(void);
u8 spi_byte(u8 data);
void spi_write(unsigned char tx_length, unsigned char *p);
void spi_read(u8 data_length, u8 api_command );
u8 check_cts(void);
void fifo_reset(void);
void spi_write_fifo(void);
void spi_read_fifo(void);
void enable_tx_interrupt(void);
void enable_rx_interrupt(void);
void clr_interrupt(void);
void tx_start(void);
void rx_start(void);
void tx_data(void);
void rx_init(void);
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
	unsigned char chksum,i;
	volatile u32 nIRQ_status;
	xil_printf("start run the si4463_test\r\n");
	/* Initialize the GPIO driver  */
	status = gpio_init(GPIO_ID);
	if (status != 0)
	{
		xil_printf("gpio_init failed, and the return status is %d\r\n",status);
	}
	xil_printf("gpio_initilaze success\r\n");
	/*Initialize the Si4463 */
	Si4463_Reset();
	Si4463_Init();
	ms_delay(50);
	s_delay(1);
	rx_init();
	i = 0;
	while(1)
	{
		if(((XGpio_DiscreteRead(&Gpio,GPIO_CHANNEL) & 0x20)>>5 )== 0)
		{
			spi_read_fifo();
			//读取数据
			for(i = 0; i < 14; i++)
				{
					xil_printf("receive data seccess and the data[%d] is %x\r\n",i,rx_buf[i]);
				}
			fifo_reset();
			clr_interrupt();
			tx_data();
			rx_init();
		}
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
	XGpio_SetDataDirection(&Gpio, GPIO_CHANNEL,0x23);
	Reg_DIR = XGpio_GetDataDirection(&Gpio, GPIO_CHANNEL);
	if (Reg_DIR != 0x23)
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
	ms_delay(10);
	XGpio_DiscreteClear(&Gpio,GPIO_CHANNEL,SDN);
	XGpio_DiscreteSet(&Gpio,GPIO_CHANNEL,nSEL);
	ms_delay(5);
	XGpio_DiscreteClear(&Gpio,GPIO_CHANNEL,nSEL);
	for (i = 0; i< 7; i++)
	{
		data = spi_byte(RF_POWER_UP_data[i]);
	}
	XGpio_DiscreteSet(&Gpio,GPIO_CHANNEL,nSEL);
	xil_printf("si4463 reset success\r\n");
}


/*****************************************************************************/
/**
*
* The purpose of this function is to send and receive data through the spi
*
* include function
*
*	spi_byte
*	spi_write
*
*
******************************************************************************/

u8 spi_byte(unsigned char data)
{
	unsigned char i;
	volatile unsigned char temp;
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

		temp = XGpio_DiscreteRead(&Gpio,GPIO_CHANNEL) & 0x02;
		if (temp)
			data |= 0x01;
		else
			data &= 0xfe;

		//SCK = 0;
		XGpio_DiscreteClear(&Gpio,GPIO_CHANNEL,SCLK);

	}
	return (data);
}
/*---------------------------------------------------------------------------*/
 /*
 *
 *
 * 向spi上写数据
 *
 *
 * ---------------------------------------------------------------------------*/
void spi_write(unsigned char tx_length, unsigned char *p)
{
	unsigned char i,j;

	i = 0;
	while(i!=0xff)
		i = check_cts();

	//SCK = 0;
	XGpio_DiscreteClear(&Gpio,GPIO_CHANNEL,SCLK);
	//nSEL = 0;
	XGpio_DiscreteClear(&Gpio,GPIO_CHANNEL,nSEL);
	for (i = 0; i < tx_length; i++)
	{
		j = *(p+i);
		spi_byte(j);
	}

	//nSEL = 1;
	XGpio_DiscreteSet(&Gpio,GPIO_CHANNEL,nSEL);
}

/*---------------------------------------------------------------------------*/
 /*
 *
 *
 * 读SPI上数据
 *
 *
 * ---------------------------------------------------------------------------*/
void spi_read(u8 data_length, u8 api_command )
{
	u8 i;

	u8 p[1];
	p[0] = api_command;
	i = 0;
	while(i!=0xff)
		i = check_cts();

	spi_write(1, p);

	i = 0;
	while(i!=0xff)
		i = check_cts();

	//nSEL = 1;
	XGpio_DiscreteSet(&Gpio,GPIO_CHANNEL,nSEL);
	//SCK = 0;
	XGpio_DiscreteClear(&Gpio,GPIO_CHANNEL,SCLK);
	//nSEL = 0;
	XGpio_DiscreteClear(&Gpio,GPIO_CHANNEL,nSEL);
	spi_byte(0x44);
	for (i = 0; i< data_length; i++)
		spi_read_buf[i] = spi_byte(0xff);
	//nSEL = 1;
	XGpio_DiscreteSet(&Gpio,GPIO_CHANNEL,nSEL);
}


u8 check_cts(void)
{
	u8 i;

	//nSEL = 1;
	XGpio_DiscreteSet(&Gpio,GPIO_CHANNEL,nSEL);
	//SCK = 0;
	XGpio_DiscreteClear(&Gpio,GPIO_CHANNEL,SCLK);
	//nSEL = 0;
	XGpio_DiscreteClear(&Gpio,GPIO_CHANNEL,nSEL);

	spi_byte(0x44);
	i = spi_byte(0);
	//nSEL = 1;
	XGpio_DiscreteSet(&Gpio,GPIO_CHANNEL,nSEL);
	return (i);
}

void Si4463_Init(void)
{
	u8 app_command_buf[20];
	xil_printf("initialize si4463 \r\n");
	//spi_write(0x07, RF_GPIO_PIN_CFG_data);
	app_command_buf[0] = 0x13;			// SET GPIO PORT
	app_command_buf[1]  = 0x14; 		// gpio 0 ,Rx data
	app_command_buf[2]  = 0x02;    		// gpio1, output 0
	app_command_buf[3]  = 0x21;  		// gpio2, hign while in receive mode
	app_command_buf[4]  = 0x20; 		// gpio3, hign while in transmit mode
	app_command_buf[5]  = 0x27;   		// nIRQ
	app_command_buf[6]  = 0x0b;  		// sdo
	spi_write(7, app_command_buf);
	xil_printf("SET GPIO PORT SUCCESS\r\n");
	// spi_write(0x05, RF_GLOBAL_XO_TUNE_1_data);
    app_command_buf[0] = 0x11;
	app_command_buf[1]  = 0x00;
	app_command_buf[2]  = 0x01;
	app_command_buf[3]  = 0x00;
	app_command_buf[4]  = 98;  			// freq  adjustment
	spi_write(5, app_command_buf);
	xil_printf("Sets the value of a property.\r\n");
	// spi_write(0x05, RF_GLOBAL_CONFIG_1_data);
  	app_command_buf[0] = 0x11;
	app_command_buf[1]  = 0x00;
	app_command_buf[2]  = 0x01;
	app_command_buf[3]  = 0x03;
	app_command_buf[4]  = 0x40;  		// tx = rx = 64 byte,PH,high performance mode
	spi_write(5, app_command_buf);


    spi_write(0x08, RF_FRR_CTL_A_MODE_4_data);    // disable all fast response register
    xil_printf("disable all fast response register\r\n");

    // spi_write(0x0D, RF_PREAMBLE_TX_LENGTH_9_data); // set Preamble
 	app_command_buf[0] = 0x11;
	app_command_buf[1]  = 0x10;
	app_command_buf[2]  = 0x09;
	app_command_buf[3]  = 0x00;
	app_command_buf[4]  = 0x08;							//  8 bytes Preamble
	app_command_buf[5]  = 0x14;							//  detect 20 bits
	app_command_buf[6]  = 0x00;
	app_command_buf[7]  = 0x0f;
	app_command_buf[8]  = 0x31;  						//  no manchest.1010.
	app_command_buf[9]  = 0x00;
	app_command_buf[10]  = 0x00;
	app_command_buf[11]  = 0x00;
	app_command_buf[12]  = 0x00;
	spi_write(13, app_command_buf);
	xil_printf("Sets the value of a property.GPOUP 0 SUCCESS\r\n");
	//  RF_SYNC_CONFIG_5_data,							// set sync
    app_command_buf[0] = 0x11;
	app_command_buf[1]  = 0x11;
	app_command_buf[2]  = 0x05;
	app_command_buf[3]  = 0x00;
	app_command_buf[4]  = 0x01;   						// no manchest , 2 bytes
	app_command_buf[5]  = 0x2d;   						// sync byte3
	app_command_buf[6]  = 0xd4;							// sync byte2
	app_command_buf[7]  = 0x00;							// sync byte1
	app_command_buf[8]  = 0x00;							// sync byte0
    spi_write(9, app_command_buf);

	//  packet crc
    app_command_buf[0] = 0x11;
	app_command_buf[1]  = 0x12;
	app_command_buf[2]  = 0x01;
	app_command_buf[3]  = 0x00;
	app_command_buf[4]  = 0x81;							// CRC = itu-c, enable crc
    spi_write(5, app_command_buf);

	// packet   gernale configuration
    app_command_buf[0] = 0x11;
	app_command_buf[1]  = 0x12;
	app_command_buf[2]  = 0x01;
	app_command_buf[3]  = 0x06;
	app_command_buf[4]  = 0x02;							// CRC MSB data MSB
    spi_write(5, app_command_buf);

  	// spi_write(0x07, RF_PKT_LEN_3_data);
    app_command_buf[0] = 0x11;
	app_command_buf[1]  = 0x12;
	app_command_buf[2]  = 0x03;
	app_command_buf[3]  = 0x08;
	app_command_buf[4]  = 0x00;
	app_command_buf[5]  = 0x00;
	app_command_buf[6]  = 0x00;
    spi_write(7, app_command_buf);

	app_command_buf[0] = 0x11;
	app_command_buf[1]  = 0x12;
	app_command_buf[2]  = 0x0c;
	app_command_buf[3]  = 0x0d;
	app_command_buf[4]  = 0x00;
	app_command_buf[5]  = payload_length;
	app_command_buf[6]  = 0x04;
	app_command_buf[7]  = 0xaa;
	app_command_buf[8]  = 0x00;
	app_command_buf[9]  = 0x00;
	app_command_buf[10]  = 0x00;
	app_command_buf[11]  = 0x00;
	app_command_buf[12]  = 0x00;
	app_command_buf[13]  = 0x00;
	app_command_buf[14]  = 0x00;
	app_command_buf[15]  = 0x00;
	spi_write(16, app_command_buf);					// set length of Field 1 -- 4

    // spi_write(0x0C, RF_PKT_FIELD_4_LENGTH_12_8_8_data);
    app_command_buf[0] = 0x11;
	app_command_buf[1]  = 0x12;
	app_command_buf[2]  = 0x08;
	app_command_buf[3]  = 0x19;
	app_command_buf[4]  = 0x00;
	app_command_buf[5]  = 0x00;
	app_command_buf[6]  = 0x00;
	app_command_buf[7]  = 0x00;
	app_command_buf[8]  = 0x00;
	app_command_buf[9]  = 0x00;
	app_command_buf[10]  = 0x00;
	app_command_buf[11]  = 0x00;
    spi_write(12, app_command_buf);
    xil_printf("initialize si4463 is running \r\n");

    spi_write(0x10, RF_MODEM_MOD_TYPE_12_data);
	spi_write(0x05, RF_MODEM_FREQ_DEV_0_1_data);

    spi_write(0x10, RF_MODEM_TX_RAMP_DELAY_12_data);
    spi_write(0x10, BCR_NCO_OFFSET_2_12_data);
	spi_write(0x10, RF_MODEM_TX_RAMP_DELAY_12_data);
    spi_write(0x07, RF_MODEM_AFC_LIMITER_1_3_data);
	//spi_write(0x10, BCR_NCO_OFFSET_2_12_data);

    spi_write(0x05, RF_MODEM_AGC_CONTROL_1_data);
    spi_write(0x10, AGC_WINDOW_SIZE_12_data);
    spi_write(0x0c, RF_MODEM_RAW_CONTROL_8_data);
//	spi_write(0x10, AGC_WINDOW_SIZE_12_data);

	// spi_write(0x05, RF_MODEM_RSSI_COMP_1_data);
	app_command_buf[0] = 0x11;
	app_command_buf[1] = 0x20;
	app_command_buf[2] = 0x01;
	app_command_buf[3] = 0x4e;
	app_command_buf[4]  = 0x40;
    spi_write(5, app_command_buf);

    spi_write(0x10, COE13_7_0_12_data);
    spi_write(0x10, COE1_7_0_12_data);
    spi_write(0x10, COE7_7_0_12_data);

	// RF_PA
	app_command_buf[0] = 0x11;
	app_command_buf[1]  = 0x22;
	app_command_buf[2]  = 0x04;
	app_command_buf[3]  = 0x00;
	app_command_buf[4]  = 0x08;
	app_command_buf[5]  = 127;							// set max power
	app_command_buf[6]  =0x00;
	app_command_buf[7]  = 0x3d;
    spi_write(8, app_command_buf);

	spi_write(0x0B, RF_SYNTH_PFDCP_CPFF_7_data);

   	// header match
   	app_command_buf[0] = 0x11;
	app_command_buf[1]  = 0x30;
	app_command_buf[2]  = 0x0c;
	app_command_buf[3]  = 0x00;
	app_command_buf[4]  = 's';
	app_command_buf[5]  = 0xff;
	app_command_buf[6]  = 0x40;
	app_command_buf[7]  = 'w';
	app_command_buf[8]  = 0xff;
	app_command_buf[9]  = 0x01;
	app_command_buf[10] = 'w';
	app_command_buf[11]  =0xff;
	app_command_buf[12]  =0x02;
	app_command_buf[13]  = 'x';
	app_command_buf[14]  = 0xff;
	app_command_buf[15]  =0x03;
    spi_write(16, app_command_buf);

	spi_write(6, RF_MODEM_RAW_SEARCH2_2_data);
    spi_write(12, RF_FREQ_CONTROL_INTE_8_data); 	    // set frequency
    xil_printf("initialize si4463 success\r\n");
}
/*---------------------------------------------------------------------------*/
 /*
 *
 *
 * 使用Si4463发送数据tx_data()
 *
 *
 * ---------------------------------------------------------------------------*/
void tx_data(void)
{
	int i;
	u8 temp;
	Flag.is_tx = 1;
	fifo_reset();
	spi_write_fifo();
	enable_tx_interrupt();
	clr_interrupt();

	tx_start();
	xil_printf("Si4463 start transmit data \r\n");
//	rf_timeout = 0;

	i = 0;
	while((XGpio_DiscreteRead(&Gpio,GPIO_CHANNEL) & 0x20)>>5)
	{
		check_cts();
	}
	ms_delay(100);
	clr_interrupt();
	Flag.is_tx = 0;
	xil_printf("Si4463 success Transmit data\r\n");
}
/*---------------------------------------------------------------------------*/
 /*
 *
 *
 * 清空RF4463FIFO，以便于存入待发送数据
 *
 *
 * ---------------------------------------------------------------------------*/
void fifo_reset(void)			// reset FIFO
{
	u8 p[2];

	p[0] = FIFO_INFO;
	p[1] = 0x03;   // reset tx ,rx fifo
	spi_write(2,p);

}
/*---------------------------------------------------------------------------*/
 /*
 *
 *
 * 将待发送的数据存入FIFO
 *
 *
 * ---------------------------------------------------------------------------*/
void spi_write_fifo(void)
{
	u8 i;

	i = 0;
	while(i!=0xff)
		i = check_cts();
	//nSEL = 1;
	XGpio_DiscreteSet(&Gpio,GPIO_CHANNEL,nSEL);
	//SCK = 0;
	XGpio_DiscreteClear(&Gpio,GPIO_CHANNEL,SCLK);
	//nSEL = 0;
	XGpio_DiscreteClear(&Gpio,GPIO_CHANNEL,nSEL);
	spi_byte(WRITE_TX_FIFO);
	for (i = 0; i< payload_length; i++)
	{
		spi_byte(tx_ph_data[i]);
	}
	//nSEL = 1;
	XGpio_DiscreteSet(&Gpio,GPIO_CHANNEL,nSEL);
}
/*---------------------------------------------------------------------------*/
 /*
 *
 *
 * 将数据从FIFO中读出
 *
 *
 * ---------------------------------------------------------------------------*/
void spi_read_fifo(void)
{
	u8 i;

	i = 0;
	while(i!=0xff)
		i = check_cts();

	//nSEL = 1;
	XGpio_DiscreteSet(&Gpio,GPIO_CHANNEL,nSEL);
	//SCK = 0;
	XGpio_DiscreteClear(&Gpio,GPIO_CHANNEL,SCLK);
	//nSEL = 0;
	XGpio_DiscreteClear(&Gpio,GPIO_CHANNEL,nSEL);
	spi_byte(READ_RX_FIFO);
	for (i = 0; i< payload_length; i++)
		rx_buf[i] = spi_byte(0xff);
	//nSEL = 1;
	XGpio_DiscreteSet(&Gpio,GPIO_CHANNEL,nSEL);
}

/*---------------------------------------------------------------------------*/
 /*
 *
 *
 * 配置4463 的发射中断，告知模块发射完毕数据后置低NIRQ
 *
 *
 * ---------------------------------------------------------------------------*/
void enable_tx_interrupt(void)
{
	u8 p[6];

	p[0] = 0x11;
	p[1] = 0x01;
	p[2] = 0x02;
	p[3] = 0x00;
	p[4] = 0x01;
	p[5] = 0x20;
	spi_write(0x06, p);
}
/*---------------------------------------------------------------------------*/
 /*
 *
 *
 * 配置 4463 的发射中断，告知模块接收到完整的数据包后置低 NIRQ
 *
 *
 * ---------------------------------------------------------------------------*/

void enable_rx_interrupt(void)
{
	u8 p[7];

	p[0] = 0x11;
	p[1] = 0x01;  // 0x0100
	p[2] = 0x03;// 3 parameters
	p[3] = 0x00;   // 0100
	p[4] = 0x03;   // ph, modem int
	p[5] = 0x18; // 0x10;   // Pack received int
	p[6] = 0x00;   //preamble int, sync int setting
	spi_write(0x07, p);  // enable  packet receive interrupt
}

/*---------------------------------------------------------------------------*/
 /*
 *
 *
 * 清空中断，避免前一部分操作产生的中断影响程序
 *
 *
 * ---------------------------------------------------------------------------*/
void clr_interrupt(void)		// clar interrupt
{
	u8 p[4];

	p[0] = GET_INT_STATUS;
	p[1] = 0;   // clr  PH pending
	p[2] = 0;   // clr modem_pending
	p[3] = 0;   // clr chip pending
	spi_write(4,p);
	spi_read(9,GET_INT_STATUS);
}
/*---------------------------------------------------------------------------*/
 /*
 *
 *
 * 进入发射状态，发射FIFO 中的数据，NIRQ 变高
 *
 *
 * ---------------------------------------------------------------------------*/
void tx_start(void)
{
	u8 p[5];

	p[0] = START_TX ;
	p[1] = freq_channel ; 		//	channel 0
	p[2] = 0x30;
	p[3] = 0;
	p[4] = 0;
	spi_write(5, p);
}
/*---------------------------------------------------------------------------*/
 /*
 *
 *
 * 开始接收
 *
 *
 * ---------------------------------------------------------------------------*/

void rx_start(void)
{
	u8 p[8];

	p[0] = START_RX ;
	p[1] = freq_channel ; 			//	channel 0
	p[2] = 0x00;
	p[3] = 0;
	p[4] = 0;
	p[5] = 0;
	p[6] = 0x08;
	p[7] = 0x08;
	spi_write(8, p);
}
/*---------------------------------------------------------------------------*/
 /*
 *
 *
 * 初始化接收状态，准备接收数据
 *
 *
 * ---------------------------------------------------------------------------*/

void rx_init(void)
{
	Flag.is_tx = 0;
	fifo_reset();
	enable_rx_interrupt();
	clr_interrupt();
	rx_start();
}

