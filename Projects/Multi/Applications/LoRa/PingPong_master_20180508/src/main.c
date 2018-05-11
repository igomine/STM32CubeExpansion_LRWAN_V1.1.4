
/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
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
	volatile unsigned int Preamble;	//suppose is a 0xaa aa aa aa
	volatile unsigned int NodeSN;
	volatile unsigned int PayLoad;
}LoRaNodeReport_TypeDef;

typedef struct
{
	volatile unsigned int Preamble;
	volatile unsigned int NodeAddress;
	volatile unsigned int PayLoad;
}LoRaEcho_TypeDef;

#define RX_TIMEOUT_VALUE                            3000
#define BUFFER_SIZE                                 64 // Define the payload size here
#define LED_PERIOD_MS               200
#define NODE_NUMBER               10
#define LORA_CMD_LENGTH               12
#define LORA_ECHO_LENGTH               12

#define LEDS_OFF   do{ \
                   LED_Off( LED_BLUE ) ;   \
                   LED_Off( LED_RED ) ;    \
                   LED_Off( LED_GREEN1 ) ; \
                   LED_Off( LED_GREEN2 ) ; \
                   } while(0) ;

const uint8_t PingMsg[] = "PING";
uint8_t PongMsg[4];
									 
const uint8_t Node_Number =  NODE_NUMBER;

uint16_t BufferSize = BUFFER_SIZE;
uint8_t Buffer[BUFFER_SIZE];

States_t State = LOWPOWER;
LoRaCMD_TypeDef LoRaCMD;
LoRaNodeReport_TypeDef LoRaNodeReport[NODE_NUMBER+1];
									 
//+1, the last LoRaEcho struct used to buffer receive data
LoRaEcho_TypeDef LoRaEcho[NODE_NUMBER+1];

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
//add for USART1 print __chark

UART_HandleTypeDef UartHandle_USART1;

#ifdef __GNUC__
/* With GCC, small printf (option LD Linker->Libraries->Small printf
   set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART2 and Loop until the end of transmission */
  HAL_UART_Transmit(&UartHandle_USART1, (uint8_t *)&ch, 1, 0xFFFF);

  return ch;
}

//__chark									 
/* Private function prototypes -----------------------------------------------*/
static void RTC_AlarmConfig(void);
	

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
 
static void RTC_AlarmConfig(void)
{
  RTC_DateTypeDef  sdatestructure_set = {0};
  RTC_TimeTypeDef  stimestructure = {0};
  RTC_AlarmTypeDef salarmstructure = {{0}, 0};
 
  /*##-1- Configure the Date #################################################*/
  /* Set Date: October 31th 2014 */
  sdatestructure_set.Year = 0x28;
  sdatestructure_set.Month = RTC_MONTH_MARCH;
  sdatestructure_set.Date = 0x01;
  
  if(HAL_RTC_SetDate(&RtcHandle,&sdatestructure_set,RTC_FORMAT_BCD) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler(); 
  } 
  
  /*##-2- Configure the Time #################################################*/
  /* Set Time: 23:59:55 */
  stimestructure.Hours = 0x17;
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
  /* Set Alarm to 00:00:10 
     RTC Alarm Generation: Alarm on Hours, Minutes and Seconds */
  salarmstructure.Alarm = RTC_ALARM_A;
  salarmstructure.AlarmDateWeekDay = RTC_WEEKDAY_MONDAY;
  salarmstructure.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
  salarmstructure.AlarmMask = RTC_ALARMMASK_DATEWEEKDAY;
  salarmstructure.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_NONE;
  salarmstructure.AlarmTime.TimeFormat = RTC_HOURFORMAT12_AM;
  salarmstructure.AlarmTime.Hours = 0x17;
  salarmstructure.AlarmTime.Minutes = 0x00;
  salarmstructure.AlarmTime.Seconds = 0x05;
  
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

  bool isMaster = true;
  uint8_t i;
//	uint32_t test;
	
	//__chark
	RtcHandle.Instance = RTC;
	
	RTC_DateTypeDef  sdatestructure = {0};
	RtcHandle.Instance = RTC;
	RtcHandle.Init.HourFormat     = RTC_HOURFORMAT_24;
  RtcHandle.Init.AsynchPrediv   = RTC_ASYNCH_PREDIV;
  RtcHandle.Init.SynchPrediv    = RTC_SYNCH_PREDIV;
  RtcHandle.Init.OutPut         = RTC_OUTPUT_DISABLE;
  RtcHandle.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  RtcHandle.Init.OutPutType     = RTC_OUTPUT_TYPE_OPENDRAIN;
	
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
		LoRaCMD.Preamble = 0xaaaa;
	
	//__chark

  HAL_Init( );
  
  SystemClock_Config( );
	//SystemClock_STANDBYMode_Config();
  
  DBG_Init( );

  HW_Init( );
//	PRINTF("test\n\r");
//	PRINTF("var = 0x%x\r\n", PongMsg[0]);
//		printf("test\r\n");
//	  printf("var = %d\r\n", PongMsg[i]);
//__chark init USART1
		//__chark add USART1 init
	
//	  UartHandle_USART1.Instance        = USART_chark;
//  
//  UartHandle_USART1.Init.BaudRate   = 115200;
//  UartHandle_USART1.Init.WordLength = UART_WORDLENGTH_8B;
//  UartHandle_USART1.Init.StopBits   = UART_STOPBITS_1;
//  UartHandle_USART1.Init.Parity     = UART_PARITY_NONE;
//  UartHandle_USART1.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
//  UartHandle_USART1.Init.Mode       = UART_MODE_TX_RX;
//  
//  if(HAL_UART_Init(&UartHandle_USART1) != HAL_OK)
//  {
//    /* Initialization Error */
//    Error_Handler(); 
//  }
	
	
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
                                  
  Radio.Rx( RX_TIMEOUT_VALUE );

	LED_Toggle( LED1 ) ;
																	

	
  while( 1 )
  {
    switch( State )
    {
    case RX:
      if( isMaster == true )
      {
        if( BufferSize > 0 && BufferSize == 12 )
        {
          if( strncmp( ( const char* )Buffer, ( const char* )PongMsg, 4 ) == 0 )
          {
            TimerStop(&timerLed );
            LED_Toggle( LED2 ) ;						
						memcpy(&LoRaNodeReport[0], Buffer, 12);
            PRINTF("rsv node %x, payload=0x%x\r\n", LoRaNodeReport[0].NodeSN, LoRaNodeReport[0].PayLoad);
           }

            else // valid reception but neither a PING or a PONG message
            {    // Set device as master ans start again
              PRINTF("rsv unknown msg\r\n");
            }
          }
				
					//after RX a frame, check if the frame is receive from the node of last ping address, 
					//if is True ,contiue with next Node address ,if not , wait for 
//					if(LoRaEcho[NODE_NUMBER].NodeAddress == LoRaCMD.NodeAddress)
//					{
//						PRINTF("EchoData_Match last CMD,send Next PING\n\r");
//						memcpy(Buffer, &LoRaCMD, LORA_CMD_LENGTH);
//						for( i = LORA_CMD_LENGTH; i < BufferSize; i++ )
//						{
//							Buffer[i] = i - LORA_CMD_LENGTH;
//						}
//						DelayMs( 1 ); 
//						Radio.Send( Buffer, BufferSize );
//						LoRaCMD.NodeAddress++;
//						
//						State = LOWPOWER;
//						Radio.Rx( RX_TIMEOUT_VALUE );
//					}
//					//not match ,wait one more RX_TIMEOUT_VALUE
//					else
//					{
//						PRINTF("EchoData not Match last CMD,send Next PING\n\r");
//						
//						State = LOWPOWER;
//						Radio.Rx( RX_TIMEOUT_VALUE );
//					}
						
       }
			else	//if node is a slave
       {
          if( BufferSize > 0 )
          {
            if( strncmp( ( const char* )Buffer, ( const char* )PongMsg, 4 ) == 0 )
            {
              // Indicates on a LED that the received frame is a PING
//              TimerStop(&timerLed );
//              LED_Off( LED_RED1);
//              LED_Off( LED_RED2 ) ; 
//              LED_Off( LED_GREEN ) ;
              LED_Toggle( LED1 );

              // Send the reply to the PONG string
//              Buffer[0] = 'P';
//              Buffer[1] = 'O';
//              Buffer[2] = 'N';
//              Buffer[3] = 'G';
              // We fill the buffer with numbers for the payload 
//              for( i = 4; i < BufferSize; i++ )
//              {
//                Buffer[i] = i - 4;
//              }
//              DelayMs( 1 );

//              Radio.Send( Buffer, BufferSize );
							memcpy(&LoRaEcho[NODE_NUMBER], Buffer, LORA_ECHO_LENGTH);
							memcpy(&LoRaEcho[ (unsigned char) LoRaEcho[NODE_NUMBER].NodeAddress ], &LoRaEcho[NODE_NUMBER], LORA_ECHO_LENGTH);	
              PRINTF("RESCEIVE PONG\n\r");
            }
            else // valid reception but not a PING as expected
            {    // Set device as master and start again
//              isMaster = true;
//              Radio.Rx( RX_TIMEOUT_VALUE );
            }
         }
      }
			 
      Radio.Rx( RX_TIMEOUT_VALUE );
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
			PRINTF("RX_TIMEOUT\n\r");
    case RX_ERROR:
//			//PRINTF("RX_ERROR\n\r");
//      if( isMaster == true )
//      {
//        // Send the next PING frame
//				PRINTF("RX_ERROR,SEND PING\n\r");

//				memcpy(Buffer, &LoRaCMD, LORA_CMD_LENGTH);
//        for( i = LORA_CMD_LENGTH; i < BufferSize; i++ )
//        {
//          Buffer[i] = i - LORA_CMD_LENGTH;
//        }
//        DelayMs( 1 ); 
//        Radio.Send( Buffer, BufferSize );
//				LoRaCMD.NodeAddress++;
//      }
//      else
//      {
//        Radio.Rx( RX_TIMEOUT_VALUE );
//      }			
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
			
		
//			EnterStandbyMode_RTC_config();
//			RTC_AlarmConfig();
//			PRINTF("enter StandbyMode\n\r");
//    
//			/* Clear all related wakeup flags */
//			__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
//				
//				/* Enter the Standby mode */
//			HAL_PWR_EnterSTANDBYMode();
			
			
//#ifndef LOW_POWER_DISABLE
//			SystemClock_STANDBYMode_Config( );
//      LPM_EnterLowPower( );
//#endif
    }
    //ENABLE_IRQ( );
       
  }
}

void OnTxDone( void )
{
    Radio.Sleep( );
    State = TX;
    PRINTF("OnTxDone\n\r");
}

void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr )
{
    Radio.Sleep( );
    BufferSize = size;
    memcpy( Buffer, payload, BufferSize );
    RssiValue = rssi;
    SnrValue = snr;
    State = RX;
  
    PRINTF("OnRxDone\n\r");
    PRINTF("RssiValue=%d dBm, SnrValue=%d\n\r", rssi, snr);
}

void OnTxTimeout( void )
{
    Radio.Sleep( );
    State = TX_TIMEOUT;
  
    PRINTF("OnTxTimeout\n\r");
}

void OnRxTimeout( void )
{
    Radio.Sleep( );
    State = RX_TIMEOUT;
    PRINTF("OnRxTimeout\n\r");
}

void OnRxError( void )
{
    Radio.Sleep( );
    State = RX_ERROR;
    PRINTF("OnRxError\n\r");
}

static void OnledEvent( void )
{
//  LED_Toggle( LED_BLUE ) ; 
//  LED_Toggle( LED_RED1 ) ; 
//  LED_Toggle( LED_RED2 ) ; 
//  LED_Toggle( LED_GREEN ) ;   
     LED_Toggle( LED2 ) 
  TimerStart(&timerLed );
}



