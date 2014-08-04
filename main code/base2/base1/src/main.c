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
#include <stdlib.h>
#include <util/delay.h>
#define F_CPU 32000000UL

#include "lcd.h"
#include "initialize.h"
#include "nrf24l01_L.h"
#include "transmitter.h"
#include "Menu.h"


void send_ask(unsigned char);
void get_MS(char);
void disp_ans(void);

/*! Defining an example slave address. */
#define SLAVE1_ADDRESS    0
#define SLAVE2_ADDRESS    1
#define SLAVE3_ADDRESS    2
#define SLAVE4_ADDRESS    3

/* Global variables */
int flg_reply=0;
int cnt=0;
int flg=0;
int flg1=0;
int adc =0;
int count=0;
int tmp;
int master;
char rx[15];
char buff[2];
int driverTGL;

int flg_off;
char str[40];
char tx1a[1];
char tx1b[1];
char tx2a[1];
char tx2b[1];
char tx3a[1];
char tx3b[1];
char tx4a[1];
char tx4b[1];
char rx1[3];
char rx2[3];
char rx3[3];
char rx4[3];
//////////////////////
int buff_reply;
unsigned char reply2;
char buff2;
int buff_p;
int buff_i;
int buff_d;
int buff_u;

unsigned char reply2;
uint32_t time_ms=0,kck_time,LED_Red_Time=1,LED_Green_Time=1,LED_White_Time=1,Buzzer_Time=1;
uint16_t LED_Red_Speed,LED_Green_Speed,LED_White_Speed,Buzzer_Speed;

int Seg[18] = {Segment_0,Segment_1,Segment_2,Segment_3,Segment_4,Segment_5,Segment_6,Segment_7,Segment_8,Segment_9,
               Segment_10,Segment_11,Segment_12,Segment_13,Segment_14,Segment_15,Segment_Dash};
unsigned char Buf_Rx_L[_Buffer_Size] ;//= "00000000000000000000000000000000";
char Buf_Tx_L[_Buffer_Size] ;//= "bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb";
char Address[_Address_Width] = { 0x11, 0x22, 0x33, 0x44, 0x55};//pipe0 {0xE7,0xE7,0xE7,0xE7,0xE7};////

float kp,ki,kd;	
char ctrlflg=0;

inline int PD_CTRL (int Setpoint,int Feed_Back,int *PID_Err_past,int *d_past,float *i);
struct _Motor_Param
{
    int Encoder;
	int Hall;
    int Speed;
	int HSpeed;
    int Speed_past;
	int HSpeed_past;
    int Err,d,i;
    int Direction;
    char PWM
};
typedef	struct _Motor_Param Motor_Param;
Motor_Param M0,M1,M2,M3,MH;
int main (void)
{
    En_RC32M();

    //Enable LowLevel & HighLevel Interrupts
    PMIC_CTRL |= PMIC_HILVLEN_bm | PMIC_LOLVLEN_bm |PMIC_MEDLVLEN_bm;

    PORT_init();
    TimerD0_init();
    TimerC0_init();
    USARTF0_init();
    USARTF1_init();
	USARTE0_init();
    ADCA_init();
    LCDInit();
    //wdt_enable();

    // Globally enable interrupts
    sei();

    LED_Green_Time	= 3000;	LED_Green_Speed = 500;
    LED_Red_Time	= 3000;	LED_Red_Speed	= 100;
    LED_White_Time	= 1000;	LED_White_Speed = 200;
    Buzzer_Time		= 2000;	Buzzer_Speed	= 150;

    Address[0]=Address[0] + RobotID ;

    ///////////////////////////////////////////////////////////////////////////////////////////////Begin NRF Initialize
    NRF24L01_L_CE_LOW;       //disable transceiver modes

    SPI_Init();

    _delay_us(10);
    _delay_ms(100);      //power on reset delay needs 100ms
    NRF24L01_L_Clear_Interrupts();
    NRF24L01_L_Flush_TX();
    NRF24L01_L_Flush_RX();
    NRF24L01_L_CE_LOW;
    if (RobotID < 3)
        NRF24L01_L_Init_milad(_TX_MODE, _CH_0, _2Mbps, Address, _Address_Width, _Buffer_Size, RF_PWR_MAX);
    else
        NRF24L01_L_Init_milad(_TX_MODE, _CH_1, _2Mbps, Address, _Address_Width, _Buffer_Size, RF_PWR_MAX);
    NRF24L01_L_WriteReg(W_REGISTER | DYNPD,0x01);
    NRF24L01_L_WriteReg(W_REGISTER | FEATURE,0x06);

    NRF24L01_L_CE_HIGH;
    _delay_us(130);
    ///////////////////////////////////////////////////////////////////////////////////////////////END   NRF Initialize

    // Insert application code here, after the board has been initialized.
    while(1)
    {
        asm("wdr");
        if (ctrlflg)
        {
			//speed via hall
			M0.HSpeed_past = M0.HSpeed; if (abs(M0.Hall-M0.HSpeed)<2000) M0.HSpeed = M0.Hall;// M0.Hall=0;
			M0.HSpeed = M0.HSpeed_past + _FILTER_CONST*1*(M0.HSpeed - M0.HSpeed_past);
			
			M1.HSpeed_past = M1.HSpeed; if (abs(M1.Hall-M1.HSpeed)<2000) M1.HSpeed = M1.Hall;// M1.Hall=0;
			M1.HSpeed = M1.HSpeed_past + _FILTER_CONST*1*(M1.HSpeed - M1.HSpeed_past);
			
			M2.HSpeed_past = M2.HSpeed; if (abs(M2.Hall-M2.HSpeed)<2000) M2.HSpeed = M2.Hall; //M2.Hall=0; 
			M2.HSpeed = M2.HSpeed_past + _FILTER_CONST*1*(M2.HSpeed - M2.HSpeed_past);
			
			M3.HSpeed_past = M3.HSpeed; if (abs(M3.Hall-M3.HSpeed)<2000) M3.HSpeed = M3.Hall;//M3.Hall=0;  
			M3.HSpeed = M3.HSpeed_past + _FILTER_CONST*1*(M3.HSpeed - M3.HSpeed_past);
			//
			
			//speed via encoder
            M0.Speed_past = M0.Speed; M0.Speed = M0.Encoder*7.5; M0.Encoder = 0;
            M0.Speed = M0.Speed_past +_FILTER_CONST*(M0.Speed - M0.Speed_past);
			
            M1.Speed_past = M1.Speed; M1.Speed = M1.Encoder*7.5; M1.Encoder = 0;
            M1.Speed = M1.Speed_past +_FILTER_CONST*(M1.Speed - M1.Speed_past);
			
            M2.Speed_past = M2.Speed; M2.Speed = M2.Encoder*7.5; M2.Encoder = 0;
            M2.Speed = M2.Speed_past +_FILTER_CONST*(M2.Speed - M2.Speed_past);
			
            M3.Speed_past = M3.Speed; M3.Speed = M3.Encoder*7.5; M3.Encoder = 0;
            M3.Speed =( M3.Speed_past +_FILTER_CONST*(M3.Speed - M3.Speed_past));
		    //
			
            disp_ans();
            kp = (float)Robot_D[RobotID].P/100.0;
            ki = (float)Robot_D[RobotID].I/100.0;
            kd = (float)Robot_D[RobotID].D/100.0;
            ctrlflg = 0;
			
			M0.PWM=PD_CTRL(Robot_D[RobotID].M0b|(Robot_D[RobotID].M0a<<8),M0.HSpeed,&M0.Err,&M0.d,&M0.i);
			M1.PWM=PD_CTRL((Robot_D[RobotID].M1b|(Robot_D[RobotID].M1a<<8)),M1.HSpeed,&M1.Err,&M1.d,&M1.i);
			M2.PWM=PD_CTRL((Robot_D[RobotID].M2b|(Robot_D[RobotID].M2a<<8)),M2.HSpeed,&M2.Err,&M2.d,&M2.i);
			M3.PWM=PD_CTRL((Robot_D[RobotID].M3b|(Robot_D[RobotID].M3a<<8)),M3.HSpeed,&M3.Err,&M3.d,&M3.i);
			
            //M0.PWM=PD_CTRL(Robot_D[RobotID].M0b|(Robot_D[RobotID].M0a<<8),M0.Speed,&M0.Err,&M0.d,&M0.i);
            //M1.PWM=PD_CTRL((Robot_D[RobotID].M1b|(Robot_D[RobotID].M1a<<8)),M1.Speed,&M1.Err,&M1.d,&M1.i);
            //M2.PWM=PD_CTRL((Robot_D[RobotID].M2b|(Robot_D[RobotID].M2a<<8)),M2.Speed,&M2.Err,&M2.d,&M2.i);
            //M3.PWM=PD_CTRL((Robot_D[RobotID].M3b|(Robot_D[RobotID].M3a<<8)),M3.Speed,&M3.Err,&M3.d,&M3.i);
			
            usart_putchar(&USARTF0,'*');
            usart_putchar(&USARTF0,'~');
            usart_putchar(&USARTF0,50);//M0.PWM);//
            usart_putchar(&USARTF0,50);//M1.PWM);//
            usart_putchar(&USARTF0,50);//M2.PWM);
            usart_putchar(&USARTF0,50);//M3.PWM);
			switch (driverTGL)
			{
				case 0:
				usart_putchar(&USARTF0,'#');
				break;
				
				case 1:
				usart_putchar(&USARTF0,'$');
				break;
				//case 0:
				//usart_putchar(&USARTF0,0b00000000);
				//break;
				//
				//case 1:
				//usart_putchar(&USARTF0,0b00000001);
				//break;
				//
				//case 2:
				//usart_putchar(&USARTF0,0b00000010);
				//break;
				//
				//case 3:
				//usart_putchar(&USARTF0,0b00000011);
				//break;
			}
            
			
			
            LED_White_PORT.OUTTGL = LED_White_PIN_bm;
            LED_Red_PORT.OUTTGL = LED_Red_PIN_bm;
            adc = adc_get_unsigned_result(&ADCA,ADC_CH0);

            if (adc<=1240)
            {
                Buzzer_PORT.OUTSET = Buzzer_PIN_bm;//10.3 volt
				PORTC.OUTSET=PIN2_bm;
            }
			else
			{
				Buzzer_PORT.OUTCLR = Buzzer_PIN_bm;//10.3 volt
				PORTC.OUTCLR=PIN2_bm;
			}

            //LCDGotoXY(0,0);
            //sprintf(str,"%3d,%3d",Robot_D[RobotID].M0b|(Robot_D[RobotID].M0a<<8),M0.PWM);
            //LCDStringRam(str);
            //LCDGotoXY(0,1);
            //sprintf(str,"%4d",M0.Speed);
            //LCDStringRam(str);
            if (KCK_DSH_SW| (Robot_D[RobotID].KCK))
            {
	            if (KCK_Sens2)
	            {
		            flg=1;
	            }
            }

            if ((Robot_D[RobotID].CHP))
            {
	            if (KCK_Sens2)
	            {
		            flg1=1;
	            }
            }

            
            //Buf_Tx_L[0] = ((Robot_D[RobotID].M0b|(Robot_D[RobotID].M0a<<8))*40)& 0x0FF;
            //Buf_Tx_L[1] = (((Robot_D[RobotID].M0b|(Robot_D[RobotID].M0a<<8))*40)  >> 8) & 0x0FF;
            //Buf_Tx_L[2] = ((Robot_D[RobotID].M1b|(Robot_D[RobotID].M1a<<8))*40)& 0x0FF;
            //Buf_Tx_L[3] = (((Robot_D[RobotID].M1b|(Robot_D[RobotID].M1a<<8))*40)  >> 8) & 0x0FF;
            //Buf_Tx_L[4] = ((Robot_D[RobotID].M2b|(Robot_D[RobotID].M2a<<8))*40)& 0x0FF;
            //Buf_Tx_L[5] = (((Robot_D[RobotID].M2b|(Robot_D[RobotID].M2a<<8))*40)  >> 8) & 0x0FF;
			//Buf_Tx_L[6] = ((Robot_D[RobotID].M3b|(Robot_D[RobotID].M3a<<8))*40)& 0x0FF;
			//Buf_Tx_L[7] = (((Robot_D[RobotID].M3b|(Robot_D[RobotID].M3a<<8))*40)  >> 8) & 0x0FF;
			Buf_Tx_L[0] = buff_reply & 0xFF;
			Buf_Tx_L[1] = (buff_reply >> 8) & 0xFF;
			Buf_Tx_L[2] = buff_reply & 0xFF;
			Buf_Tx_L[3] = (buff_reply >> 8) & 0xFF;
			Buf_Tx_L[4] = buff_reply & 0xFF;
			Buf_Tx_L[5] = (buff_reply >> 8) & 0xFF;
			Buf_Tx_L[6] = buff_reply & 0xFF;
			Buf_Tx_L[7] = (buff_reply >> 8) & 0xFF;
            Buf_Tx_L[8] = M0.Speed & 0xFF;
            Buf_Tx_L[9] = (M0.Speed >> 8) & 0xFF;
            Buf_Tx_L[10] = M1.Speed & 0xFF;
            Buf_Tx_L[11] = (M1.Speed >> 8) & 0xFF;
            Buf_Tx_L[12] = M2.Speed & 0xFF;
            Buf_Tx_L[13] = (M2.Speed >> 8) & 0xFF;
            Buf_Tx_L[14] = M3.Speed & 0xFF;
            Buf_Tx_L[15] = (M3.Speed >> 8) & 0xFF;
            Buf_Tx_L[16] = adc >> 4;
            

            //LED_Red_PORT.OUTTGL = LED_Red_PIN_bm;
            NRF24L01_L_Write_TX_Buf(Buf_Tx_L, _Buffer_Size);
            NRF24L01_L_RF_TX();
        }
        _delay_us(1);
        //_delay_ms(100);
        //LED_Red_PORT.OUTTGL = LED_Red_PIN_bm;

    }
}




ISR(PORTE_INT0_vect)////////////////////////////////////////PTX   IRQ Interrupt Pin
{   
    uint8_t status_L = NRF24L01_L_WriteReg(W_REGISTER | STATUSe, _TX_DS|_MAX_RT|_RX_DR);
    if((status_L & _RX_DR) == _RX_DR)
    {
        LED_White_PORT.OUTTGL = LED_White_PIN_bm;
        //1) read payload through SPI,
        NRF24L01_L_Read_RX_Buf(Buf_Rx_L, _Buffer_Size);

        if(Buf_Rx_L[0] == RobotID)
        {   
			cnt=0;

            Robot_D[RobotID].RID = Buf_Rx_L[0];
            Robot_D[RobotID].M0a  = Buf_Rx_L[1];
            Robot_D[RobotID].M0b  = Buf_Rx_L[2];
            Robot_D[RobotID].M1a  = Buf_Rx_L[3];
            Robot_D[RobotID].M1b  = Buf_Rx_L[4];
            Robot_D[RobotID].M2a  = Buf_Rx_L[5];
            Robot_D[RobotID].M2b  = Buf_Rx_L[6];
            Robot_D[RobotID].M3a  = Buf_Rx_L[7];
            Robot_D[RobotID].M3b  = Buf_Rx_L[8];
            Robot_D[RobotID].KCK = Buf_Rx_L[9];
            Robot_D[RobotID].CHP = Buf_Rx_L[10];
            Robot_D[RobotID].ASK = Buf_Rx_L[11];
            Robot_D[RobotID].P = Buf_Rx_L[12];
            Robot_D[RobotID].I = Buf_Rx_L[13];
            Robot_D[RobotID].D = Buf_Rx_L[14];

        }


        //2) clear RX_DR IRQ,
        //NRF24L01_R_WriteReg(W_REGISTER | STATUSe, _RX_DR );
        //3) read FIFO_STATUS to check if there are more payloads available in RX FIFO,
        //4) if there are more data in RX FIFO, repeat from step 1).
    }
    if((status_L&_TX_DS) == _TX_DS)
    {   //LED_Red_PORT.OUTTGL = LED_Red_PIN_bm;
        //NRF24L01_R_WriteReg(W_REGISTER | STATUSe, _TX_DS);
    }
    if ((status_L&_MAX_RT) == _MAX_RT)
    {
        //LED_Green_PORT.OUTTGL = LED_Green_PIN_bm;
        NRF24L01_L_Flush_TX();
        //NRF24L01_R_WriteReg(W_REGISTER | STATUSe, _MAX_RT);
    }
}

char timectrl;

ISR(TCD0_OVF_vect)
{
    wdt_reset();
    timectrl++;

    if (timectrl>=20)
    {
        //LED_Green_PORT.OUTTGL = LED_Green_PIN_bm;
        ctrlflg=1;
		driverTGL++;
		driverTGL=driverTGL%2;
        timectrl=0;
		
    }

    //timer for 1ms
    time_ms++;
    if(flg)
    {

        if(kck_time<3000){
            kck_time++;KCK_Charge(KCK_CHARGE_OFF); KCK_Speed_DIR(KCK_SPEED_HI);}
        else {
            KCK_Speed_DIR(KCK_SPEED_OFF);KCK_Charge(KCK_CHARGE_ON); kck_time=0; flg=0;}

    }
    if(flg1)
    {
        if(kck_time<100){kck_time++; KCK_Speed_CHIP(KCK_SPEED_HI); KCK_Charge(KCK_CHARGE_OFF);}
        else {KCK_Speed_CHIP(KCK_SPEED_OFF);KCK_Charge(KCK_CHARGE_ON); kck_time=0; flg1=0;}
    }


    cnt++;
    if(cnt>=300)
    {
        //tx1[0]=0;
        //tx2[0]=0;
        //tx3[0]=0;
        //tx4[0]=0;
    }

    if(menu_time == 1)
    {
        Menu_Disp(Menu_Disp_OFF);
        Menu_Display();
        Menu_Reset();
        menu_time--;
    }
    else if (menu_time>1)
    {
        menu_time--;
        menu_check_status();

        if(menu_time<3000)
        {
            Buzzer_Time=menu_time;
            Buzzer_Speed=200;
        }
    }
    else
    {
        Disp_R_PORT.OUT = Seg[RobotID];
        Disp_L_PORT.OUT = Seg[RobotID];
        //PORTJ_OUTSET=0xFF;
        //PORTH_OUTSET=0xFF;
    }
};

ISR(PORTF_INT0_vect)
{

}

ISR(PORTQ_INT0_vect)
{
    M1.Encoder +=(PORTQ_IN&PIN1_bm)?-1:1;
}

ISR(PORTH_INT0_vect)
{
    M0.Encoder +=(PORTH_IN&PIN4_bm)?-1:1;
}

ISR(PORTC_INT0_vect)
{
    M3.Encoder +=(PORTC_IN&PIN4_bm)?-1:1;
}

ISR(PORTQ_INT1_vect)
{
    M2.Encoder +=(PORTQ_IN&PIN2_bm)?-1:1;
}

ISR(PORTH_INT1_vect)
{
	//LED_White_PORT.OUTTGL=LED_White_PIN_bm;
	if(menu_time ==0 )
	{//LED_Green_PORT.OUTTGL = LED_Green_PIN_bm;
		menu_check_sw((Menu_Set),&Menu_Set_flg);
		menu_check_sw((Menu_Cancel),&Menu_Cancel_flg);
	}
	menu_time = 30000;

	Menu_Disp(Menu_Disp_ON);
	Menu_Display();
}

ISR(PORTK_INT0_vect)
{
	
}

void disp_ans(void)
{
			//LED_Green_PORT.OUTTGL = LED_Green_PIN_bm;
			LCDGotoXY(0,0);
			//LCDStringRam("salam");
			sprintf(str,"Hall: %1d",buff_reply);
			LCDStringRam(str);
			LCDGotoXY(0,1);
			sprintf(str,"ENC: %1d",M3.Speed);
			LCDStringRam(str);
			LCDGotoXY(9,1);
			sprintf(str,"H: %1d",reply2);
			LCDStringRam(str);
	
	//char buf_test[5] ={1,2,3,4,5};
	//for (uint8_t i=0;i<5;i++)
	//{
		//usart_putchar(&USARTE0,buf_test[i]);
		//}
		
		uint8_t count1;
		char str1[200];
		count1 = sprintf(str1,"%d,%d\r",M3.HSpeed,M3.Speed);//,driverTGL*100+400);//,buff_reply);
		
		for (uint8_t i=0;i<count1;i++)
		{
			usart_putchar(&USARTE0,str1[i]);	
		}
	
}

void send_ask(unsigned char ask)
{
	switch(ask)
	{

		case 0:  //LED_Green_PORT.OUTTGL = LED_Green_PIN_bm;
		usart_putchar(&USARTF0,SLAVE1_ADDRESS);
		break;
		case 1: // LED_White_PORT.OUTTGL = LED_White_PIN_bm;
		usart_putchar(&USARTF0,SLAVE2_ADDRESS);
		break;
		case 2:  //LED_Red_PORT.OUTTGL = LED_Red_PIN_bm;
		usart_putchar(&USARTF0,SLAVE3_ADDRESS);
		break;
		case 3:
		usart_putchar(&USARTF0,SLAVE4_ADDRESS);
		break;
		default:
		usart_putchar(&USARTF0,0xff);
		break;
	};

}

void get_MS(char rx)
{

}

int ask_cnt0=0;
int ask_cnt1=0;

int buff_reply_tmp0;
int buff_reply_tmp1;
int buff_p_temp;
int buff_i_temp;
int buff_d_temp;
int buff_u_temp;
unsigned char reply2_tmp;


ISR(USARTF0_RXC_vect)        ///////////Driver M.2  &  M.3
{
	
	//char buff_reply [16];
	unsigned char data;
	data=USARTF0_DATA;
   
	

	switch(ask_cnt0)
	{
		case 0:
		if (data== '*')
		{
			LED_Green_PORT.OUTTGL = LED_Green_PIN_bm;
			ask_cnt0++;
		}
		break;

		case 1:
		buff_reply_tmp0=data&0x0ff;
		//tmp=data;
		ask_cnt0++;
		break;

		case 2:
		buff_reply_tmp0|=(data<<8)&0x0ff00;
		//master=data;
		ask_cnt0++;
		break;

		case 3:
		reply2_tmp = data;
		ask_cnt0++;
		break;

		//case 4:
		//buff_p_temp=data&0x0ff;
		//ask_cnt++;
		//break;
		//
		//case 5:
		//buff_p_temp|=(data<<8)&0x0ff00;
		//ask_cnt++;
		//break;
		//
		//case 6:
		//buff_i_temp=data&0x0ff;
		//ask_cnt++;
		//break;
		//
		//case 7:
		//buff_i_temp|=(data<<8)&0x0ff00;
		//ask_cnt++;
		//break;
		//
		//case 8:
		//buff_d_temp=data&0x0ff;
		//ask_cnt++;
		//break;
		//
		//case 9:
		//buff_d_temp|=(data<<8)&0x0ff00;
		//ask_cnt++;
		//break;
		//
		//case 10:
		//buff_u_temp=data&0x0ff;
		//ask_cnt++;
		//break;
		//
		//case 11:
		//buff_u_temp|=(data<<8)&0x0ff00;
		//ask_cnt++;
		//break;


		case 4:
		if (data=='#')
		{
		
			
			switch(reply2_tmp)
			{
				case 2:
				M2.Hall=buff_reply_tmp0;
				break;
				
				case 3:
				M3.Hall=buff_reply_tmp0;
				break; 
			}
			
			buff_reply=buff_reply_tmp0;
			reply2 = reply2_tmp;
			
			//buff_p = buff_p_temp;
			//buff_i = buff_i_temp;
			//buff_d = buff_d_temp;
			//buff_u = buff_u_temp;


			//flg_reply=0;
			ask_cnt0=0;
		}
		ask_cnt0=0;
		break;
	}
}

ISR(USARTF1_RXC_vect)   ///////////// Driver  M.0  &  M.1
{
	
	
	unsigned char data;
	data=USARTF1_DATA;

	


	switch(ask_cnt1)
	{
		case 0:
		if (data== '*')
		{
			LED_Red_PORT.OUTTGL=LED_Red_PIN_bm;
			ask_cnt1++;
		}
		break;

		case 1:
		buff_reply_tmp1=data&0x0ff;
		ask_cnt1++;
		break;

		case 2:
		buff_reply_tmp1|=(data<<8)&0x0ff00;
		ask_cnt1++;
		break;

		case 3:
		reply2_tmp = data;
		ask_cnt1++;
		break;

		//case 4:
		//buff_p_temp=data&0x0ff;
		//ask_cnt++;
		//break;
		//
		//case 5:
		//buff_p_temp|=(data<<8)&0x0ff00;
		//ask_cnt++;
		//break;
		//
		//case 6:
		//buff_i_temp=data&0x0ff;
		//ask_cnt++;
		//break;
		//
		//case 7:
		//buff_i_temp|=(data<<8)&0x0ff00;
		//ask_cnt++;
		//break;
		//
		//case 8:
		//buff_d_temp=data&0x0ff;
		//ask_cnt++;
		//break;
		//
		//case 9:
		//buff_d_temp|=(data<<8)&0x0ff00;
		//ask_cnt++;
		//break;
		//
		//case 10:
		//buff_u_temp=data&0x0ff;
		//ask_cnt++;
		//break;
		//
		//case 11:
		//buff_u_temp|=(data<<8)&0x0ff00;
		//ask_cnt++;
		//break;


		case 4:
		if (data=='#')
		{

			switch(reply2_tmp)
			{
				case 0:
				M0.Hall=buff_reply_tmp1;
				break;
							
				case 1:
				M1.Hall=buff_reply_tmp1;
				break;
			}
						
			//LED_Green_PORT.OUTTGL = LED_Green_PIN_bm;
			buff_reply=buff_reply_tmp1;
			reply2 = reply2_tmp;
			//buff_p = buff_p_temp;
			//buff_i = buff_i_temp;
			//buff_d = buff_d_temp;
			//buff_u = buff_u_temp;


			//flg_reply=0;
			ask_cnt1=0;
		}
		ask_cnt1=0;
		break;
	}
	
}

//ISR(USARTE0_RXC_vect)
//{
	//LED_Green_PORT.OUTTGL = LED_Green_PIN_bm;
//}

inline int PD_CTRL (int Setpoint,int Feed_Back,int *PID_Err_past,int *d_past,float *i)
{

	//Setpoint=(40*Setpoint);
	

	int PID_Err=Setpoint-Feed_Back;

	
	int d=(PID_Err-(*PID_Err_past))*10 ;
	// d= (*d_past) +0.05*(d-(*d_past));

	d=(abs(d)<50)?0:d;

	d=(d>2400)?(0):d;
	d=(d<-2400)?(0):d;

	int p=PID_Err*kp;


	(*i)=(*i)+(ki*PID_Err)*.020;


	if ((*i)>80)
	(*i)=80;
	if ((*i)<-80)
	(*i)=-80;

	p=(p>127)?(127):p;
	p=(p<-127)?(-127):p;

	//PID_U_past=PID_U;
	int PID_U=p+(*i)+kd*d;//(0.5)*PID_Err2_M1+(1.5)*(PID_Err1_M1+PID_Err2_M1);//+(12.5)*(float)(PID_Err2_M1-PID_Err1_M1)/10.0; //kp=0.5  kd=9

	if(PID_U>127)
	PID_U=127;
	if( PID_U<-127)
	PID_U=-127;
	*PID_Err_past=PID_Err;
	*d_past=d;
	return PID_U;
	// direction =1;

}
