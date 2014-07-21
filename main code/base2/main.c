/**
 * \file
 *
 * \brief Empty user application template
 *
 */

/*
 * Include header files for all drivers that have been imported from
 * Atmel Software Framework (ASF).
 */
#include <asf.h>
#define F_CPU 32000000UL
#include <util/delay.h>
#include "initialize.h"
#include "lcd.h"
#include "nrf24l01_L.h"
#include "transmitter.h"
#include "Menu.h"
#include <stdlib.h>



//////////TWI
#include "twi_master_driver.h"
#include "twi_slave_driver.h"
/*! Defining an example slave address. */
#define SLAVE1_ADDRESS    0x10
#define SLAVE2_ADDRESS    0x11
#define SLAVE3_ADDRESS    0x12
#define SLAVE4_ADDRESS    0x13

/*! Defining number of bytes in buffer. */
#define NUM_BYTES        1

/*! CPU speed 2MHz, BAUDRATE 100kHz and Baudrate Register Settings */
#define CPU_SPEED       32000000
#define BAUDRATE	100000
#define TWI_BAUDSETTING TWI_BAUD(CPU_SPEED, BAUDRATE)


/* Global variables */
TWI_Master_t twiMaster;    /*!< TWI master module. */
TWI_Slave_t twiSlave;      /*!< TWI slave module. */

  char buff[2];
  char tx1[1];
  char tx2[1];
  char tx3[1];
  char tx4[1];
  char rx1[3];
  char rx2[3];
  char rx3[3];
  char rx4[3];
//////////////////////





uint32_t time_ms=0,LED_Red_Time=1,LED_Green_Time=1,LED_White_Time=1,Buzzer_Time=1;
uint16_t LED_Red_Speed,LED_Green_Speed,LED_White_Speed,Buzzer_Speed;



int Seg[18] = {Segment_0,Segment_1,Segment_2,Segment_3,Segment_4,Segment_5,Segment_6,Segment_7,Segment_8,Segment_9,
				Segment_10,Segment_11,Segment_12,Segment_13,Segment_14,Segment_15,Segment_Dash};
char Buf_Rx_L[_Buffer_Size] = "00000000000000000000000000000000";
char Buf_Tx_L[_Buffer_Size] = "12345678901234567890123456789012";
char Address[_Address_Width] = { 0x11, 0x22, 0x33, 0x44, 0x55};
int main (void)
{
	/* Initialize TWI master. */
	TWI_MasterInit(&twiMaster,&TWIF,TWI_MASTER_INTLVL_LO_gc,TWI_BAUDSETTING);
	TWIF.SLAVE.CTRLA=0;  //slave disabled
	//board_init();
	
	En_RC32M();

	//Enable LowLevel & HighLevel Interrupts
	PMIC_CTRL |= PMIC_HILVLEN_bm | PMIC_LOLVLEN_bm |PMIC_MEDLVLEN_bm;

	PORT_init();
	TimerD0_init();
	TimerC0_init();
	//USARTE0_init();
	//ADCB_init();
	LCDInit();
	//wdt_enable();
	
	
	// Globally enable interrupts
	sei();

	LED_Green_Time	= 3000;	LED_Green_Speed = 500;
	LED_Red_Time	= 3000;	LED_Red_Speed	= 100;
	LED_White_Time	= 1000;	LED_White_Speed = 200;
	Buzzer_Time		= 2000;	Buzzer_Speed	= 150;
	
	
	
	
	///////////////////////////////////////////////////////////////////////////////////////////////Begin NRF Initialize
	    NRF24L01_L_CE_LOW;       //disable transceiver modes

	    SPI_Init();

	    _delay_us(10);
	    _delay_ms(100);      //power on reset delay needs 100ms
	    NRF24L01_L_Clear_Interrupts();
	    NRF24L01_L_Flush_TX();
	    NRF24L01_L_Flush_RX();
	    NRF24L01_L_CE_LOW;
	    NRF24L01_L_Init_milad(_TX_MODE, _CH, _2Mbps, Address, _Address_Width, _Buffer_Size, RF_PWR_MAX);
	    NRF24L01_L_WriteReg(W_REGISTER | DYNPD,0x01);
	    NRF24L01_L_WriteReg(W_REGISTER | FEATURE,0x06);

	    NRF24L01_L_CE_HIGH;
	    _delay_us(130);
	///////////////////////////////////////////////////////////////////////////////////////////////END   NRF Initialize
	
	// Insert application code here, after the board has been initialized.
	while(1)
	{
		
		LCDGotoXY(0,0);
		sprintf("salam");
		//LCDStringRam(str);
		////////TWI
		//tx1[0]=Robot_D[RobotID].M1;
		//tx2[0]=Robot_D[RobotID].M2;
		//tx3[0]=Robot_D[RobotID].M3;
		//tx4[0]=Robot_D[RobotID].M4;
		
				tx1[0]=0x0F;
				tx2[0]=0x12;
				tx3[0]=0x01;
				tx4[0]=0x02;
		
	
	//	TWI_MasterWriteRead(&twiMaster,SLAVE1_ADDRESS,&tx1[0],1,3);
	//	TWI_MasterWriteRead(&twiMaster,SLAVE2_ADDRESS,&tx2[0],1,3);
		TWI_MasterWriteRead(&twiMaster,SLAVE3_ADDRESS,&tx3[0],1,3);
		rx3[0]=twiMaster.readData[0];
		if (rx3[0]==1)
		{
			LED_White(ON);
		}
		
	//	TWI_MasterWriteRead(&twiMaster,SLAVE4_ADDRESS,&tx4[0],1,3);
		/////////////////////
		
		
		NRF24L01_L_Write_TX_Buf(Buf_Tx_L, _Buffer_Size);
		NRF24L01_L_RF_TX();
		if (Buf_Rx_L[0]=='f')
		{ 
			LED_Green_PORT.OUTTGL = LED_Green_PIN_bm;
		}
		_delay_ms(10);
	}
}

/*! TWIF Master Interrupt vector. */
ISR(TWIF_TWIM_vect)
{
	TWI_MasterInterruptHandler(&twiMaster);
}


ISR(PORTE_INT0_vect)////////////////////////////////////////PTX   IRQ Interrupt Pin
{
	uint8_t status_L = NRF24L01_L_WriteReg(W_REGISTER | STATUSe, _TX_DS|_MAX_RT|_RX_DR);
	if((status_L & _RX_DR) == _RX_DR)
	{
		LED_White_PORT.OUTSET = LED_White_PIN_bm;
		//1) read payload through SPI,
		NRF24L01_L_Read_RX_Buf(Buf_Rx_L, _Buffer_Size);
		//2) clear RX_DR IRQ,
		//NRF24L01_R_WriteReg(W_REGISTER | STATUSe, _RX_DR );
		//3) read FIFO_STATUS to check if there are more payloads available in RX FIFO,
		//4) if there are more data in RX FIFO, repeat from step 1).
	}
	if((status_L&_TX_DS) == _TX_DS)
	{   LED_Red_PORT.OUTSET = LED_Red_PIN_bm;
		//NRF24L01_R_WriteReg(W_REGISTER | STATUSe, _TX_DS);
	}
	if ((status_L&_MAX_RT) == _MAX_RT)
	{
	//	LED_Green_PORT.OUTSET = LED_Green_PIN_bm;
		NRF24L01_L_Flush_TX();
		//NRF24L01_R_WriteReg(W_REGISTER | STATUSe, _MAX_RT);
	}
	LED_White_PORT.OUTCLR = LED_White_PIN_bm;
	LED_Red_PORT.OUTCLR = LED_Red_PIN_bm;
	//LED_Green_PORT.OUTCLR = LED_Green_PIN_bm;
}



ISR(TCD0_OVF_vect)
{
	    //timer for 1ms
	    time_ms++;
		
		
};



ISR(USARTE0_DRE_vect)
{
	//USARTE0_STATUS = USART_DREIF_bm;
	//USARTE0_DATA = 2;//Robot_Send_PCK[Send_cnt];
	
	//if(Send_cnt < 11)
	//{
		//LED_White_PORT.OUTTGL = LED_White_PIN_bm;
		//USARTE0_DATA = Robot_Send_PCK[Send_cnt];
		//Send_cnt++;
	//}
	//else
	//usart_set_dre_interrupt_level(&USARTE0,USART_INT_LVL_OFF);
};

ISR(PORTF_INT0_vect)
{
	if(menu_time ==0 )
	{
		menu_check_sw((Menu_Set),&Menu_Set_flg);
		menu_check_sw((Menu_Cancel),&Menu_Cancel_flg);
	}
	menu_time = 30000;

	Menu_Disp(Menu_Disp_ON);
	Menu_Display();
}