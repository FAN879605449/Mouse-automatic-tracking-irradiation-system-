#include "board.h"
#include "chip.h"
#include <stdio.h>
#include "string.h"
#include "peri_driver.h"




/***** 一些变量及其格式的定义 *****/
LPC_USART_T *g_pUSART = LPC_USART1;   //将指针g_pUSA设置成USART格式，并且指向USART1的基地址
static I2CM_XFER_T  i2cmXferRec;  //I2CM transfer record
static char data[10] = {0},Xdir, Ydir;  // 初始化
static int number = 0;
static int remained = 0;
static	uint32_t Xcoord = 0, Ycoord = 0;	// 定义完最好赋值初始化一下，防止成为垃圾数据

const uint8_t *g_pcFlash = (const uint8_t*)0;
volatile uint8_t g_isClearToSend = TRUE;	// Status flag shows that app is now ready for next transmit
volatile uint8_t g_txIrqPacketFlag;	// A packet has been sent flag.
#define DUMP_RANGE	0x2000
#define APP_PACKET_SIZE 1024	
uint8_t g_rxBuf[APP_PACKET_SIZE];
volatile uint32_t g_txNdx;
volatile uint32_t g_rxNdx;			// Current array index for g_rxBuf[]
volatile uint8_t g_rxErrorCode;
volatile uint8_t g_rxPacketFlag;	// A packet has been received flag.




#define I2C_CLK_DIVIDER         (50)  //System clock is set to 30MHz, I2C clock is set to 600kHz
#define I2C_BITRATE             (100000)  //100KHz I2C bit-rate

/* 7-bit I2C addresses of 24C02 */
/* Note: The ROM code requires the address to be between bits [6:0]
         bit 7 is ignored */
#define I2C_ADDR_EEPROM           (0x50)
#define BYTE_PER_PAGE             (8)
#define ADDRESS_RANGE             (255)
#define I2C_BUF_SIZE				(1 * 1)
#define TICKRATE_HZ				100
#define	EV_TICK_CT_DISPLAY		0x01

#define XDirLow()	Chip_GPIO_PinSetState(LPC_GPIO_PORT, 0, 16, 0);
#define XPulLow()	Chip_GPIO_PinSetState(LPC_GPIO_PORT, 0, 17, 0);
#define YDirLow()	Chip_GPIO_PinSetState(LPC_GPIO_PORT, 0, 18, 0);
#define YPulLow()	Chip_GPIO_PinSetState(LPC_GPIO_PORT, 0, 28, 0);

#define XDirHigh()	Chip_GPIO_PinSetState(LPC_GPIO_PORT, 0, 16, 1);
#define XPulHigh()	Chip_GPIO_PinSetState(LPC_GPIO_PORT, 0, 17, 1);
#define YDirHigh()	Chip_GPIO_PinSetState(LPC_GPIO_PORT, 0, 18, 1);
#define YPulHigh()	Chip_GPIO_PinSetState(LPC_GPIO_PORT, 0, 28, 1);

#define XEnaHigh()  Chip_GPIO_PinSetState(LPC_GPIO_PORT, 0, 13, 1);
#define YEnaHigh()  Chip_GPIO_PinSetState(LPC_GPIO_PORT, 0, 27, 1);

#define XEnaLow()  Chip_GPIO_PinSetState(LPC_GPIO_PORT, 0, 13, 0);
#define YEnaLow()  Chip_GPIO_PinSetState(LPC_GPIO_PORT, 0, 27, 0);

//uint8_t I2cWrBuf[I2C_BUF_SIZE] ;     // 定义 I2C 的写数组BUF
//uint8_t I2cRdBuf[I2C_BUF_SIZE] ;





/************ I2C 的引脚复用 10和11 ********/
static void Init_I2C_PinMux(void)
{
#if (defined(BOARD_NXP_LPCXPRESSO_812) || defined(BOARD_LPC812MAX) || defined(BOARD_NXP_LPCXPRESSO_824))
	Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_SWM);  //Enable the clock to the Switch Matrix
#if defined(BOARD_NXP_LPCXPRESSO_824)
	Chip_SWM_EnableFixedPin(SWM_FIXED_I2C0_SDA);
	Chip_SWM_EnableFixedPin(SWM_FIXED_I2C0_SCL);
#else
	Chip_SWM_MovablePinAssign(SWM_I2C_SDA_IO, 10);  // Connect the I2C_SDA and I2C_SCL signals to port pins(P0.10, P0.11)
	Chip_SWM_MovablePinAssign(SWM_I2C_SCL_IO, 11);
#endif
	
	Chip_IOCON_PinSetI2CMode(LPC_IOCON, IOCON_PIO10, PIN_I2CMODE_FASTPLUS);  // Enable Fast Mode Plus for I2C pins
	Chip_IOCON_PinSetI2CMode(LPC_IOCON, IOCON_PIO11, PIN_I2CMODE_FASTPLUS);
	Chip_Clock_DisablePeriphClock(SYSCTL_CLOCK_SWM);  //Disable the clock to the Switch Matrix to save power
#else
	/* Configure your own I2C pin muxing here if needed */
#warning "No I2C pin muxing defined"
#endif
}


/* 完成I2C的基本配置。Setup I2C handle and parameters */
static void setupI2CMaster()
{
	Chip_I2C_Init(LPC_I2C);  // Enable I2C clock and reset I2C peripheral
	Chip_I2C_SetClockDiv(LPC_I2C, I2C_CLK_DIVIDER);  //Setup clock rate for I2C
	Chip_I2CM_SetBusSpeed(LPC_I2C, I2C_BITRATE);  // Setup I2CM transfer rate
	Chip_I2CM_Enable(LPC_I2C);  //Enable Master Mode
}

/************* UART 中断服务函数   数据格式：X、正负、百、十、个*******/
void UART1_IRQHandler (void)
{
//	uint32_t i,n,j;
	uint32_t status;
	status = Chip_UART_GetStatus(g_pUSART);   // 读取uart的状态

	/***加脱机信号****/
	XEnaHigh();
	YEnaHigh();
	
	if (status & UART_STAT_RXRDY) {
			if(number < 9) {  
				data[number] = (uint8_t)Chip_UART_ReadByte(g_pUSART);   // Read a single byte data from the UART peripheral
				
      number++;
				if(data[0] != 'X')// 让首个数一定为X
					number = 0;
			}
			else if(number == 9){    // judge weather finish one page transmit
				data[number] = Chip_UART_ReadByte(g_pUSART); 
				number = 0;    // 完成一次处理，Reset
				if(data[5] == 'Y'){
				Xcoord = (((data[2]-0x30)*100) + ((data[3]-0x30)*10) + (data[4]-0x30));   //我们需要多少个1.8度的转动	
				Ycoord = (((data[7]-0x30)*100) + ((data[8]-0x30)*10) + (data[9]-0x30));   //我们需要多少个1.8度的转动
				Xdir = data[1];
				Ydir = data[6];	
        }					
			}
			else
				g_rxErrorCode = 3;	// buffer overrun	
	}
	if (status & UART_STAT_OVERRUNINT){
		Chip_UART_ClearStatus(g_pUSART, UART_STAT_OVERRUNINT);
		g_rxErrorCode = 2;	// USART h/w overrun
		//DEBUGOUT("overrun \n");
	}	
}

/***************************************
             main主函数
****************************************/

	
int main(void){
	/***** 变量定义 *****/
		uint32_t status;
	  uint32_t i = 0 ;
	
	/***** 系统板子初始化(设置主频、汇编启动函数) *****/
	  SystemCoreClockUpdate(); 
	  Board_Init();   // 这里面配置了 DEBUG_UART 为 UART1，设的 0， 4引脚
	  Chip_UART_IntEnable(g_pUSART, UART_INTEN_RXRDY | UART_INTEN_OVERRUN);      //set UART 的中断源

	/***** 陈总配置 *****/
	Chip_UART_Init(g_pUSART);
	Chip_UART_ConfigData(g_pUSART, UART_CFG_DATALEN_8 | UART_CFG_PARITY_NONE | UART_CFG_STOPLEN_1);
	Chip_UART_SetBaud(g_pUSART, 115200);
	Chip_UART_TXEnable(g_pUSART);
  Chip_UART_IntDisable(g_pUSART, UART_INTEN_TXRDY | UART_INTEN_TXIDLE);
	Chip_UART_IntEnable(g_pUSART, UART_INTEN_RXRDY | UART_INTEN_OVERRUN);
	NVIC_EnableIRQ(UART1_IRQn);
	Chip_UART_Enable(g_pUSART);
	
    XDirLow();
    XPulLow();
    YDirLow();
    YPulLow();
		XEnaLow();  
		YEnaLow(); 
 
	/***** I2C 引脚复用，开启中断 *****/
	Init_I2C_PinMux();  //引脚复用 10,11 I2C0引脚定死了
	setupI2CMaster();  // Allocate I2C handle, setup I2C rate, and initialize I2C clocking
	NVIC_EnableIRQ(I2C_IRQn);  // 打开了I2C的中断
	
	/***** 配置中断优先级 *****/
//	NVIC_SetPriority(UART1_IRQn,1);
//	NVIC_SetPriority(I2C_IRQn,0);
		
	/***** 行为层 *****/
	DEBUGOUT("Please write your data\n");
  while (1){
			while ((Xcoord) || (Ycoord)){
			/***** Disable ENA  *****/
			//DEBUGOUT("plu\n");
			XEnaLow();
			YEnaLow();
			for(i=0;i<6000;i++)		__NOP;       // 延迟>5us，我准备延迟10us
	
		/******  X方向的控制 *****/
			if (Xcoord != 0){
					if(Xdir == '-'){
							XDirHigh();  //将X输入反转      Chip_GPIO_PinSetState(LPC_GPIO_T *pGPIO, uint8_t port, uint8_t pin, bool setting)
								for(i=0;i<600;i++)		__NOP;       // 延迟>5us，我准备延迟10us
							XPulHigh();
								for(i=0;i<200;i++)		__NOP;    // delay
							XPulLow();
								for(i=0;i<200;i++)		__NOP;    // delay
							Xcoord--;
					}
					if(Xdir == '+'){
							XDirLow();  //将X输入正转     Chip_GPIO_PinSetState(LPC_GPIO_T *pGPIO, uint8_t port, uint8_t pin, bool setting)
								for(i=0;i<600;i++)		__NOP;       // 延迟>5us，我准备延迟10us
							XPulHigh();
								for(i=0;i<200;i++)		__NOP;    // delay
							XPulLow();
								for(i=0;i<200;i++)		__NOP;    // delay
							Xcoord--;
					}
			}
		/******  Y方向的控制 *****/	
			if (Ycoord != 0){
					if(Ydir == '-'){
							YDirHigh();  //将Y输入反转      Chip_GPIO_PinSetState(LPC_GPIO_T *pGPIO, uint8_t port, uint8_t pin, bool setting)
								for(i=0;i<600;i++)		__NOP;       // 延迟>5us，我准备延迟10us
							YPulHigh();
								for(i=0;i<200;i++)		__NOP;    // delay
							YPulLow();
								for(i=0;i<200;i++)		__NOP;    // delay
							Ycoord--;
					}
					if(Ydir == '+'){
							YDirLow();  //将Y输入正转     Chip_GPIO_PinSetState(LPC_GPIO_T *pGPIO, uint8_t port, uint8_t pin, bool setting)
								for(i=0;i<600;i++)		__NOP;       // 延迟>5us，我准备延迟10us
							YPulHigh();
								for(i=0;i<200;i++)		__NOP;    // delay
							YPulLow();
								for(i=0;i<200;i++)		__NOP;    // delay
							Ycoord--;
					}	
			}
		}
	}

}