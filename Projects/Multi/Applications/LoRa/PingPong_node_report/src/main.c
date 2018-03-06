//2018.3.2
//test
/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2013 Semtech

Description: Ping-Pong implementation

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Miguel Luis and Gregory Cristian
*/
/******************************************************************************
  * @file    main.c
  * @author  MCD Application Team
  * @version V1.1.4
  * @date    08-January-2018
  * @brief   this is the main!
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include "hw.h"
#include "radio.h"
#include "timeServer.h"
#include "delay.h"
#include "low_power_manager.h"
#include "vcom.h"


#if defined( USE_BAND_868 )

#define RF_FREQUENCY                                868000000 // Hz

#elif defined( USE_BAND_915 )

#define RF_FREQUENCY                                915000000 // Hz

#else
    #error "Please define a frequency band in the compiler options."
#endif

#define TX_OUTPUT_POWER                             14        // dBm

#if defined( USE_MODEM_LORA )

#define LORA_BANDWIDTH                              0         // [0: 125 kHz,
                                                              //  1: 250 kHz,
                                                              //  2: 500 kHz,
                                                              //  3: Reserved]
#define LORA_SPREADING_FACTOR                       7         // [SF7..SF12]
#define LORA_CODINGRATE                             1         // [1: 4/5,
                                                              //  2: 4/6,
                                                              //  3: 4/7,
                                                              //  4: 4/8]
#define LORA_PREAMBLE_LENGTH                        8         // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT                         5         // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON                  false
#define LORA_IQ_INVERSION_ON                        false

#elif defined( USE_MODEM_FSK )

#define FSK_FDEV                                    25e3      // Hz
#define FSK_DATARATE                                50e3      // bps
#define FSK_BANDWIDTH                               50e3      // Hz
#define FSK_AFC_BANDWIDTH                           83.333e3  // Hz
#define FSK_PREAMBLE_LENGTH                         5         // Same for Tx and Rx
#define FSK_FIX_LENGTH_PAYLOAD_ON                   false

#else
    #error "Please define a modem in the compiler options."
#endif

typedef enum
{
    LOWPOWER,
    RX,
    RX_TIMEOUT,
    RX_ERROR,
    TX,
    TX_TIMEOUT,
}States_t;

typedef struct
{
	volatile unsigned int Preamble;
	volatile unsigned int NodeAddress;
	volatile unsigned int PayLoad;
}LoRaCMD_TypeDef;

typedef struct
{
	volatile unsigned int Preamble;
	volatile unsigned int NodeAddress;
	volatile unsigned int PayLoad;
}LoRaReport_TypeDef;

#define RX_TIMEOUT_VALUE                            10000
#define BUFFER_SIZE                                 64 // Define the payload size here
#define LED_PERIOD_MS               200
#define NODE_NUMBER               10
#define TEMP_NODE_NUMBER               10
#define LORA_CMD_LENGTH               12
#define LORA_REPORT_LENGTH               12

#define LEDS_OFF   do{ \
                   LED_Off( LED_BLUE ) ;   \
                   LED_Off( LED_RED ) ;    \
                   LED_Off( LED_GREEN1 ) ; \
                   LED_Off( LED_GREEN2 ) ; \
                   } while(0) ;

									 
//=============================================================================
//very important ,to define this program runs as sensor-node or station master									 
bool isMaster = true;
States_t State = LOWPOWER;
//=============================================================================

									 
const uint8_t PingMsg[] = "PING";
uint8_t PongMsg[4];
									 
const uint8_t Node_Number =  NODE_NUMBER;

uint16_t BufferSize = BUFFER_SIZE;
uint8_t Buffer[BUFFER_SIZE];
									 
volatile unsigned int StandByTime = 0;


LoRaCMD_TypeDef LoRaCMD;
//when this program runs in station, the first LoRaEcho struct member LoRaEcho[0], used to buffer receive data from node
//when this program runs in node, use LoRaEcho[0], collect sensor data, battery data, etc. send to station 
LoRaReport_TypeDef LoRaReport[NODE_NUMBER+1];

int8_t RssiValue = 0;
int8_t SnrValue = 0;

 /* Led Timers objects*/
static  TimerEvent_t timerLed;

//__chark add for enter standby mode and use RTC wake up 
#define RTC_CLOCK_SOURCE_LSE
/*#define RTC_CLOCK_SOURCE_LSI*/

#ifdef RTC_CLOCK_SOURCE_LSI
#define RTC_ASYNCH_PREDIV    0x7C
#define RTC_SYNCH_PREDIV     0x0127
#endif  

#ifdef RTC_CLOCK_SOURCE_LSE
#define RTC_ASYNCH_PREDIV  0x7F
#define RTC_SYNCH_PREDIV   0x00FF
#endif									 
/* RTC handler declaration */


void SystemClock_Re_Config(void);
//__chark

									 
/* Private function prototypes -----------------------------------------------*/
static void RTC_AlarmConfig(unsigned char hour, unsigned char min, unsigned char sec);
	

/*!
 * Radio events function pointer
 */
static RadioEvents_t RadioEvents;

/*!
 * \brief Function to be executed on Radio Tx Done event
 */
void OnTxDone( void );

/*!
 * \brief Function to be executed on Radio Rx Done event
 */
void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr );

/*!
 * \brief Function executed on Radio Tx Timeout event
 */
void OnTxTimeout( void );

/*!
 * \brief Function executed on Radio Rx Timeout event
 */
void OnRxTimeout( void );

/*!
 * \brief Function executed on Radio Rx Error event
 */
void OnRxError( void );

/*!
 * \brief Function executed on when led timer elapses
 */
static void OnledEvent( void );
/**
 * Main application entry point.
 */
 
static void RTC_AlarmConfig(unsigned char hour, unsigned char min, unsigned char sec)
{
  RTC_DateTypeDef  sdatestructure_set = {0};
  RTC_TimeTypeDef  stimestructure = {0};
  RTC_AlarmTypeDef salarmstructure = {{0}, 0};
 
  /*##-1- Configure the Date #################################################*/
  /* Set Date: October 31th 2014 */
  sdatestructure_set.Year = 0x14;
  sdatestructure_set.Month = RTC_MONTH_OCTOBER;
  sdatestructure_set.Date = 0x31;
  
	if( sec>0x60 || min >0x60 || hour>0x5)
  {
    /* Initialization Error */
		PRINTF("rtc alarm time param error");
    Error_Handler(); 
  }
	
  if(HAL_RTC_SetDate(&RtcHandle,&sdatestructure_set,RTC_FORMAT_BCD) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler(); 
  } 
  
  /*##-2- Configure the Time #################################################*/
  /* Set Time: 08:00:00 */
  stimestructure.Hours = 0x08;
  stimestructure.Minutes = 0x00;
  stimestructure.Seconds = 0x00;
  stimestructure.TimeFormat = RTC_HOURFORMAT12_AM;
  stimestructure.DayLightSaving = RTC_DAYLIGHTSAVING_NONE ;
  stimestructure.StoreOperation = RTC_STOREOPERATION_RESET;
  if(HAL_RTC_SetTime(&RtcHandle,&stimestructure,RTC_FORMAT_BCD) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler(); 
  }  

  /*##-3- Configure the RTC Alarm peripheral #################################*/
  /* Set Alarm to 08:00:03 
     RTC Alarm Generation: Alarm on Hours, Minutes and Seconds */
  salarmstructure.Alarm = RTC_ALARM_A;
  salarmstructure.AlarmDateWeekDay = RTC_WEEKDAY_MONDAY;
  salarmstructure.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
  salarmstructure.AlarmMask = RTC_ALARMMASK_DATEWEEKDAY;
  salarmstructure.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_NONE;
  salarmstructure.AlarmTime.TimeFormat = RTC_HOURFORMAT12_AM;
  salarmstructure.AlarmTime.Hours = 0x08;
  salarmstructure.AlarmTime.Minutes = 0x00;
  salarmstructure.AlarmTime.Seconds = sec;
  
  if(HAL_RTC_SetAlarm_IT(&RtcHandle,&salarmstructure,RTC_FORMAT_BCD) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler(); 
  }
}

void EnterStandbyMode_RTC_config()
{
	RtcHandle.Instance = RTC;
	RTC_DateTypeDef  sdatestructure = {0};
	RtcHandle.Instance = RTC;
	RtcHandle.Init.HourFormat     = RTC_HOURFORMAT_24;
  RtcHandle.Init.AsynchPrediv   = RTC_ASYNCH_PREDIV;
  RtcHandle.Init.SynchPrediv    = RTC_SYNCH_PREDIV;
  RtcHandle.Init.OutPut         = RTC_OUTPUT_DISABLE;
  RtcHandle.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  RtcHandle.Init.OutPutType     = RTC_OUTPUT_TYPE_OPENDRAIN;
	
	if (HAL_RTC_Init(&RtcHandle) != HAL_OK)
  {
			/* Initialization Error */
		BSP_LED_On(LED4);
    Error_Handler();
  }
}

int main( void )
{

  
  uint8_t i;
//	uint32_t test;
	
	//__chark
//	RtcHandle.Instance = RTC;
//	
//	RTC_DateTypeDef  sdatestructure = {0};
//	RtcHandle.Instance = RTC;
//	RtcHandle.Init.HourFormat     = RTC_HOURFORMAT_24;
//  RtcHandle.Init.AsynchPrediv   = RTC_ASYNCH_PREDIV;
//  RtcHandle.Init.SynchPrediv    = RTC_SYNCH_PREDIV;
//  RtcHandle.Init.OutPut         = RTC_OUTPUT_DISABLE;
//  RtcHandle.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
//  RtcHandle.Init.OutPutType     = RTC_OUTPUT_TYPE_OPENDRAIN;
//	
	for(i=0; i<4; i++)
	{
		PongMsg[i] = 0xaa;
	}
	if(  isMaster == true)
	{
		LoRaCMD.Preamble = 0x55555555;
		LoRaCMD.NodeAddress = 0x01;
		LoRaCMD.PayLoad = 0x66666666;
	}
	else
	{
		LoRaReport[0].Preamble = 0xaaaaaaaa;
		LoRaReport[0].NodeAddress = 0x02;
		LoRaReport[0].PayLoad = 0x12345678;
	}
	
	//__chark

  HAL_Init( );
  
  SystemClock_Config( );
	//SystemClock_STANDBYMode_Config();
  
  DBG_Init( );

  HW_Init( );  
		
	  /* Check and handle if the system was resumed from StandBy mode */ 
  if(__HAL_PWR_GET_FLAG(PWR_FLAG_SB) != RESET)
  {
		//BSP_LED_On(LED2);
		
		PRINTF("wake up from standby mode\n\r");
    /* Clear Standby flag */
    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_SB); 

    /* Get the date available in RTC. */
//    if(HAL_RTC_GetDate(&RtcHandle,&sdatestructure,RTC_FORMAT_BCD) != HAL_OK)
//    {
//      /* Initialization Error */
//      Error_Handler(); 
//    } 

    /* Check that 1 day elapsed after wake-up */
    /* As RTC_AlarmConfig initializes date to 31 october 2014, check if date is
       equal to 1st of November 2014 */
//    if ((sdatestructure.Date != 1) ||(sdatestructure.Year != 0x14) || \
//        (sdatestructure.Month != RTC_MONTH_NOVEMBER))
//    {
//      Error_Handler();
//    }
//    else
//    {
//			
//      BSP_LED_On(LED2);
//			while(1);
//			//PRINTF("success to wake up\n\r");
//    }
  }

	
	//__chark
	
  /* Led Timers*/
  TimerInit(&timerLed, OnledEvent);   
  TimerSetValue( &timerLed, LED_PERIOD_MS);

  TimerStart(&timerLed );

  // Radio initialization
  RadioEvents.TxDone = OnTxDone;
  RadioEvents.RxDone = OnRxDone;
  RadioEvents.TxTimeout = OnTxTimeout;
  RadioEvents.RxTimeout = OnRxTimeout;
  RadioEvents.RxError = OnRxError;

  Radio.Init( &RadioEvents );

  Radio.SetChannel( RF_FREQUENCY );

#if defined( USE_MODEM_LORA )

  Radio.SetTxConfig( MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                                 LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                                   LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                                   true, 0, 0, LORA_IQ_INVERSION_ON, 3000000 );
    
  Radio.SetRxConfig( MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                                   LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                                   LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                                   0, true, 0, 0, LORA_IQ_INVERSION_ON, true );

#elif defined( USE_MODEM_FSK )

  Radio.SetTxConfig( MODEM_FSK, TX_OUTPUT_POWER, FSK_FDEV, 0,
                                  FSK_DATARATE, 0,
                                  FSK_PREAMBLE_LENGTH, FSK_FIX_LENGTH_PAYLOAD_ON,
                                  true, 0, 0, 0, 3000000 );
    
  Radio.SetRxConfig( MODEM_FSK, FSK_BANDWIDTH, FSK_DATARATE,
                                  0, FSK_AFC_BANDWIDTH, FSK_PREAMBLE_LENGTH,
                                  0, FSK_FIX_LENGTH_PAYLOAD_ON, 0, true,
                                  0, 0,false, true );

#else
    #error "Please define a frequency band in the compiler options."
#endif
   if(isMaster == true)
	 {		 
			Radio.Rx( RX_TIMEOUT_VALUE );
			//State = LOWPOWER;
	 }
	 else
	 {
			PRINTF("send to station once\n\r");
		 LED_Toggle(LED2);
			LoRaReport[0].NodeAddress = 0x02;
			memcpy(Buffer, &LoRaReport, LORA_REPORT_LENGTH);
			for( i = LORA_REPORT_LENGTH; i < BufferSize; i++ )
			{
				Buffer[i] = i - LORA_REPORT_LENGTH;
			}

			DelayMs( 1 ); 
			Radio.Send( Buffer, BufferSize );
			//StandByTime = 1;
			//State = LOWPOWER;
		}
	//test = HAL_RCC_GetHCLKFreq();
	//LPM_EnterOffMode();
//	BSP_LED_On(LED1);
//	BSP_LED_On(LED2);
//	BSP_LED_On(LED3);
//	BSP_LED_On(LED4);
	//LED_Toggle( LED1 ) ;
																	

	
  while( 1 )
  {
		switch( State )
    {
    case RX:
      //PRINTF("ent RX\n\r");
			if( isMaster == true )
      {
        if( BufferSize > 0 )
        {
          if( strncmp( ( const char* )Buffer, ( const char* )PongMsg, 4 ) == 0 )
          {
					
						memcpy(&LoRaReport[0], Buffer, LORA_REPORT_LENGTH);
						memcpy(&LoRaReport[ (unsigned char) LoRaReport[0].NodeAddress ], &LoRaReport[0], LORA_REPORT_LENGTH);	
            LED_Toggle( LED2 ) ;
						switch(LoRaReport[0].NodeAddress)
						{
							case 0x01:
								PRINTF("RESCEIVE 0x01\n\r");
								break;
							case 0x02:
								PRINTF("RESCEIVE 0x02\n\r");
								break;	
							case 0x03:
								PRINTF("RESCEIVE 0x03\n\r");
								break;
							case 0x04:
								PRINTF("RESCEIVE 0x04\n\r");
								break;
							case 0x05:
								PRINTF("RESCEIVE 0x05\n\r");
								break;
							default:
								break;
            }
						//after rx a frame, respond the node, indicate that a station is online
						LoRaCMD.NodeAddress =  LoRaReport[TEMP_NODE_NUMBER].NodeAddress;
						memcpy(Buffer, &LoRaCMD, LORA_CMD_LENGTH);
						for( i = LORA_CMD_LENGTH; i < BufferSize; i++ )
						{
							Buffer[i] = i - LORA_CMD_LENGTH;
						}

            DelayMs( 1 ); 
            Radio.Send( Buffer, BufferSize );
            }

          }
						
       }
      else
      {
          if( BufferSize > 0 )
          {
            if( strncmp( ( const char* )Buffer, ( const char* )PongMsg, 4 ) == 0 )
            {
              // Indicates on a LED that the received frame is a PING
              LED_Toggle( LED3 );
							memcpy(&LoRaReport[0], Buffer, LORA_REPORT_LENGTH);
							memcpy(&LoRaReport[ (unsigned char) LoRaReport[0].NodeAddress ], &LoRaReport[0], LORA_REPORT_LENGTH);	
              PRINTF("RESCEIVE report\n\r");
            }
            else // valid reception but not a PING as expected
            {    // Set device as master and start again
//              isMaster = true;
//              Radio.Rx( RX_TIMEOUT_VALUE );
            }
         }
      }
			LED_Toggle(LED1); 
      State = LOWPOWER;
      break;
    case TX:
      // Indicates on a LED that we have sent a PING [Master]
      // Indicates on a LED that we have sent a PONG [Slave]
      //GpioWrite( &Led2, GpioRead( &Led2 ) ^ 1 );
      Radio.Rx( RX_TIMEOUT_VALUE );
      State = LOWPOWER;
      break;
    case RX_TIMEOUT:
			PRINTF(".");
    case RX_ERROR:
			//PRINTF("RX_ERROR\n\r");
      if( isMaster == true )
      {

      }
      else
      {
        Radio.Rx( RX_TIMEOUT_VALUE );
      }
			
      State = LOWPOWER;
			Radio.Rx( RX_TIMEOUT_VALUE );
      
			break;
    case TX_TIMEOUT:
      Radio.Rx( RX_TIMEOUT_VALUE );
      State = LOWPOWER;
      break;
    case LOWPOWER:
      default:
            // Set low power
      break;
    }
    
    //DISABLE_IRQ( );
    /* if an interupt has occured after __disable_irq, it is kept pending 
     * and cortex will not enter low power anyway  */
    if (State == LOWPOWER)
    {
			if( isMaster == true )
			{}
			else
			{
        if(StandByTime>0)
				{	
					EnterStandbyMode_RTC_config();
					RTC_AlarmConfig(0,0,0x05);
					PRINTF("enter StandbyMode\n\r");
				
					/* Clear all related wakeup flags */
					__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
						
						/* Enter the Standby mode */
					HAL_PWR_EnterSTANDBYMode();
				}
			}
    }
  }
}

void OnTxDone( void )
{
    Radio.Sleep( );
    //PRINTF("OnTxDone\n\r");
		if(isMaster == true)
		{
			State = TX;
		}
		else
		{
			State = LOWPOWER;
			StandByTime = 1;
			LED_Toggle(LED1);
		}

}

void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr )
{
    Radio.Sleep( );
    BufferSize = size;
    memcpy( Buffer, payload, BufferSize );
    RssiValue = rssi;
    SnrValue = snr;
	
    State = RX;
    PRINTF("RssiValue=%d dBm, SnrValue=%d...", rssi, snr);
		LED_Toggle(LED1);
}

void OnTxTimeout( void )
{
    Radio.Sleep( );
    State = TX_TIMEOUT;
  
    PRINTF("OnTxTimeout\n\r");
		LED_Toggle(LED3);
}

void OnRxTimeout( void )
{
    Radio.Sleep( );
    State = RX_TIMEOUT;
    //PRINTF("OnRxTimeout\n\r");
}

void OnRxError( void )
{
    Radio.Sleep( );
    State = RX_ERROR;
    PRINTF("OnRxError\n\r");
}

static void OnledEvent( void )
{
		LED_Toggle(LED2);
		//LED_Toggle(LED1);
		TimerStart(&timerLed );
}
