/* USER CODE BEGIN Header */
	
/* USER CODE END Header */

#include "function.h"
#include "CtlLoop.h"
#include "stdio.h"
#include "string.h"

#include "stm32g4xx_hal_def.h"

//#include "hrtim.h"
// Soft-start state flag
SState_M STState = SSInit;

// OLED refresh counter, increments every 5ms in the 5ms interrupt
uint16_t OLEDShowCnt = 0;




/** ===================================================================
**     Function Name : void StateM(void)
**     Description   : State machine function, runs in a 5ms interrupt,
**                     executes every 5ms.
**     Initialization state
**     Waiting for soft start state
**     Start state
**     Running state
**     Fault state
**     Parameters    :
**     Returns       :
** ===================================================================*/
void StateM(void)
{
    // Determine the state type
    switch(DF.SMFlag)
    {
        // Initialization state
        case Init: StateMInit();
        break;
        
        // Waiting state
        case Wait: StateMWait();
        break;
        
        // Soft start state
        case Rise: StateMRise();
        break;
        
        // Running state
        case Run: StateMRun();
        break;
        
        // Fault state
        case Err: StateMErr();
        break;
    }
}

/** ===================================================================
**     Function Name : void StateMInit(void)
**     Description   : Initialization state function, parameter initialization
**     Parameters    :
**     Returns       :
** ===================================================================*/
void StateMInit(void)
{
    // Relevant parameter initialization
    ValInit();
    // State machine transitions to waiting for soft start state
    DF.SMFlag = Wait;
}


/** ===================================================================
**     Function Name : void StateMWait(void)
**     Description :   [Garbled comment, unable to translate]
**     Parameters  :
**     Returns     :
** ===================================================================*/
void StateMWait(void)
{
	// [Garbled comment, unable to translate]
	static uint16_t CntS = 0;
	
	// Disable PWM
	DF.PWMENFlag=0;
	// [Garbled comment, unable to translate]
	CntS ++;
	// [Garbled comment, unable to translate]
	if(CntS>200)
	{
		CntS=200;
		HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TA1|HRTIM_OUTPUT_TA2); // Enable PWM outputs TA1 and TA2
		if((DF.ErrFlag==F_NOERR)&&(DF.KeyFlag1==1))
		{
			// Reset counter
			CntS=0;
			// Transition to soft start state
			DF.SMFlag  = Rise;
			// Initialize soft start state
			STState = SSInit;
		}
	}
}

/*
** ===================================================================
**     Function Name : void StateMRise(void)
**     Description : [Garbled comment, unable to translate]
**     Parameters  : none
**     Returns     : none
** ===================================================================
*/
#define MAX_SSCNT       20// Soft start duration 100ms
void StateMRise(void)
{
	// [Garbled comment, unable to translate]
	static  uint16_t  Cnt = 0;
	// [Garbled comment, unable to translate]
	static  uint16_t	BUCKMaxDutyCnt=0,BoostMaxDutyCnt=0;

	// [Garbled comment, unable to translate]
	switch(STState)
	{
		// Initialize soft start state
		case    SSInit:
		{
			// Disable PWM
			DF.PWMENFlag=0;
			HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TA1|HRTIM_OUTPUT_TA2); // Disable
			HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TB1|HRTIM_OUTPUT_TB2); // Disable				
			// [Garbled comment, unable to translate]
			CtrValue.BUCKMaxDuty  = MIN_BUKC_DUTY;
			CtrValue.BoostMaxDuty = MIN_BOOST_DUTY;
			// Reset error flags
			VErr0=0;
			VErr1=0;
			VErr2=0;
			u0 = 0;
			u1 = 0;
			// Transition to SSWait state
			STState = SSWait;

			break;
		}
		// Wait in soft start state
		case    SSWait:
		{
			// Increment counter
			Cnt++;
			// Soft start for 100ms
			if(Cnt> MAX_SSCNT)
			{
				// Reset counter
				Cnt = 0;
				// Set initial duty cycles
				CtrValue.BuckDuty = MIN_BUKC_DUTY;
				CtrValue.BUCKMaxDuty= MIN_BUKC_DUTY;
				CtrValue.BoostDuty = MIN_BOOST_DUTY;
				CtrValue.BoostMaxDuty = MIN_BOOST_DUTY;
				// Reset error flags
				VErr0=0;
				VErr1=0;
				VErr2=0;
				u0 = 0;
				u1 = 0;
				// Adjust reference voltage
				CtrValue.Voref  = CtrValue.Voref >>1;			
				STState = SSRun;	// Transition to SSRun state		
			}
			break;
		}
		// Run soft start state
		case    SSRun:
		{
			if(DF.PWMENFlag==0)// Disable PWM if flag is 0
			{
				// Reset error flags
				VErr0=0;
				VErr1=0;
				VErr2=0;
				u0 = 0;
				u1 = 0;	
				HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TA1|HRTIM_OUTPUT_TA2); // Enable PWM outputs TA1 and TA2
				HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TB1|HRTIM_OUTPUT_TB2); // Enable PWM outputs TB1 and TB2					
			}
			// Increment duty cycle counters
			DF.PWMENFlag=1;
			BUCKMaxDutyCnt++;
			BoostMaxDutyCnt++;
			// Increment maximum duty cycles
			CtrValue.BUCKMaxDuty = CtrValue.BUCKMaxDuty + BUCKMaxDutyCnt*5;
			CtrValue.BoostMaxDuty = CtrValue.BoostMaxDuty + BoostMaxDutyCnt*5;
			// Ensure maximum duty cycles do not exceed limits
			if(CtrValue.BUCKMaxDuty > MAX_BUCK_DUTY)
				CtrValue.BUCKMaxDuty  = MAX_BUCK_DUTY ;
			if(CtrValue.BoostMaxDuty > MAX_BOOST_DUTY)
				CtrValue.BoostMaxDuty  = MAX_BOOST_DUTY ;
			
			if((CtrValue.BUCKMaxDuty==MAX_BUCK_DUTY)&&(CtrValue.BoostMaxDuty==MAX_BOOST_DUTY))			
			{
				// Transition to running state
				DF.SMFlag  = Run;
				// Initialize soft start state
				STState = SSInit;	
			}
			break;
		}
		default:
		break;
	}
}

/*
** ===================================================================
**     Funtion Name :void StateMRun(void)
**     Description :正常运行，主处理函数在中断中运行
**     Parameters  : none
**     Returns     : none
** ===================================================================
*/
void StateMRun(void)
{

}

/*
** ===================================================================
**     Funtion Name :void StateMErr(void)
**     Description :故障状态
**     Parameters  : none
**     Returns     : none
** ===================================================================
*/
void StateMErr(void)
{
	// Disable PWM
	DF.PWMENFlag=0;
	HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TA1|HRTIM_OUTPUT_TA2); // Disable
	HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TB1|HRTIM_OUTPUT_TB2); // Disable		
	// [Garbled comment, unable to translate]
	if(DF.ErrFlag==F_NOERR)
			DF.SMFlag  = Wait;
}


/** ===================================================================
**     Funtion Name :void ValInit(void)
**     Description :   相关参数初始化函数
**     Parameters  :
**     Returns     :
** ===================================================================*/
void ValInit(void)
{
	// Disable PWM
	DF.PWMENFlag=0;
	HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TA1|HRTIM_OUTPUT_TA2); // Disable
	HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TB1|HRTIM_OUTPUT_TB2); // Disable		
	// [Garbled comment, unable to translate]
	DF.ErrFlag=0;
	// Reset reference voltage
	CtrValue.Voref=0;
	// Set initial duty cycles
	CtrValue.BuckDuty = MIN_BUKC_DUTY;
	CtrValue.BUCKMaxDuty= MIN_BUKC_DUTY;
	CtrValue.BoostDuty = MIN_BOOST_DUTY;
	CtrValue.BoostMaxDuty = MIN_BOOST_DUTY;	
	// Reset error flags
	VErr0=0;
	VErr1=0;
	VErr2=0;
	u0 = 0;
	u1 = 0;
}


/** ===================================================================
**     Function Name : void VrefGet(void)
**     Description :   [Garbled comment, unable to translate]
**     Parameters  :
**     Returns       :
** ===================================================================*/
#define MAX_VREF    2921// Reference voltage for 48V, 0.5V step, 48.5V/68V*Q12
#define MIN_VREF    271// Reference voltage for 5V, 0.5V step, 4.5V/68V*2^Q12
#define VREF_K      10// [Garbled comment, unable to translate]
void VrefGet(void)
{
	// [Garbled comment, unable to translate]
	int32_t VTemp = 0;	
	// [Garbled comment, unable to translate]
	static int32_t VadjSum = 0;

	// Get ADC value for Vadj with offset compensation
	SADC.Vadj = HAL_ADC_GetValue(&hadc1);
	// Perform moving average filtering
	VadjSum = VadjSum + SADC.Vadj -(VadjSum>>8);
	SADC.VadjAvg = VadjSum>>8;
	
	// Reference voltage = MIN_VREF + Vadj
	VTemp = MIN_VREF + SADC.Vadj;
	
	// Adjust reference voltage
	if( VTemp> ( CtrValue.Voref + VREF_K))
			CtrValue.Voref = CtrValue.Voref + VREF_K;
	else if( VTemp < ( CtrValue.Voref - VREF_K ))
			CtrValue.Voref =CtrValue.Voref - VREF_K;
	else
			CtrValue.Voref = VTemp ;

	// Ensure Voref does not exceed 0.85 times Vin
	if(CtrValue.Voref >((SADC.VinAvg*3482)>>12))// 0.85 * Vin 
		CtrValue.Voref =((SADC.VinAvg*3482)>>12);
}


/*
** ===================================================================
**     Function Name : void ShortOff(void)
**     Description : [Garbled comment, unable to translate]
**     Parameters  : none
**     Returns     : none
** ===================================================================
*/
#define MAX_SHORT_I     3444// Maximum short current
#define MIN_SHORT_V     289// Minimum short voltage
void ShortOff(void)
{
	static int32_t RSCnt = 0;
	static uint8_t RSNum =0 ;

	// [Garbled comment, unable to translate]
	if((SADC.Iout> MAX_SHORT_I)&&(SADC.Vout <MIN_SHORT_V))
	{
		// Disable PWM
		DF.PWMENFlag=0;
		HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TA1|HRTIM_OUTPUT_TA2); // Disable
		HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TB1|HRTIM_OUTPUT_TB2); // Disable	
		// Set short circuit error flag
		setRegBits(DF.ErrFlag,F_SW_SHORT);
		// Transition to error state
		DF.SMFlag  =Err;
	}
	// [Garbled comment, unable to translate]
	if(getRegBits(DF.ErrFlag,F_SW_SHORT))
	{
		// Increment RS counter
		RSCnt++;
		// Soft start for 2 seconds
		if(RSCnt >400)
		{
			// Reset counter
			RSCnt=0;
			// If RSNum exceeds 10, set to 11 and disable PWM
			if(RSNum > 10)
			{
				// Set RSNum to 11
				RSNum =11;
				// Disable PWM
				DF.PWMENFlag=0;
				HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TA1|HRTIM_OUTPUT_TA2); // Disable
				HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TB1|HRTIM_OUTPUT_TB2); // Disable	
			}
			else
			{
				// Increment RSNum
				RSNum++;
				// Clear short circuit error flag
				clrRegBits(DF.ErrFlag,F_SW_SHORT);
			}
		}
	}
}

/*
** ===================================================================
**     Function Name : void SwOCP(void)
**     Description : [Garbled comment, unable to translate]
**     Parameters  : none
**     Returns     : none
** ===================================================================
*/
#define MAX_OCP_VAL     3165// Maximum overcurrent value
void SwOCP(void)
{
	// [Garbled comment, unable to translate]
	static  uint16_t  OCPCnt=0;
	// [Garbled comment, unable to translate]
	static  uint16_t  RSCnt=0;
	// [Garbled comment, unable to translate]
	static  uint16_t  RSNum=0;

	// Overcurrent condition: Iout > MAX_OCP_VAL and state is Run
	if((SADC.Iout > MAX_OCP_VAL)&&(DF.SMFlag  ==Run))
	{
		// Increment OCP counter
		OCPCnt++;
		// If OCP counter exceeds threshold, disable PWM and set error flag
		if(OCPCnt > 10)
		{
			// Reset OCP counter
			OCPCnt  = 0;
			// Disable PWM
			DF.PWMENFlag=0;
			HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TA1|HRTIM_OUTPUT_TA2); // Disable
			HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TB1|HRTIM_OUTPUT_TB2); // Disable	
			// Set overcurrent error flag
			setRegBits(DF.ErrFlag,F_SW_IOUT_OCP);
			// Transition to error state
			DF.SMFlag  =Err;
		}
	}
	else
		// Reset OCP counter
		OCPCnt  = 0;

	// [Garbled comment, unable to translate]
	if(getRegBits(DF.ErrFlag,F_SW_IOUT_OCP))
	{
		if(SADC.Iout > MAX_OCP_VAL)//[Assumed translation]
		{
			// Increment recovery counter
			RSCnt++;
			// If recovery counter exceeds threshold, clear error flag
			if(RSCnt > 400)
			{
				// Reset counter
				RSCnt=0;
				// Increment RSNum
				RSNum++;
				// If RSNum exceeds 10, set to 11 and disable PWM
				if(RSNum > 10 )
				{
					// Set RSNum to 11
					RSNum =11;
					// Disable PWM
					DF.PWMENFlag=0;
					HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TA1|HRTIM_OUTPUT_TA2); // Disable
					HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TB1|HRTIM_OUTPUT_TB2); // Disable		
				}
				else
				{
				 // Clear overcurrent error flag
					clrRegBits(DF.ErrFlag,F_SW_IOUT_OCP);
				}
			}
		}
		else	
			RSCnt=0;	
	}
	else
		RSCnt=0;
}


/*
** ===================================================================
**     Function Name : void VoutSwOVP(void)
**     Description : [Garbled comment, unable to translate]
**     Parameters  : none
**     Returns     : none
** ===================================================================
*/
#define MAX_VOUT_OVP_VAL    3012// Overvoltage protection value, 50V, 50/68 * Q12
void VoutSwOVP(void)
{
	// [Garbled comment, unable to translate]
	static  uint16_t  OVPCnt=0;

	// Overvoltage condition: Vout > MAX_VOUT_OVP_VAL
	if (SADC.Vout > MAX_VOUT_OVP_VAL)
	{
		// Increment OVP counter
		OVPCnt++;
		// If OVP counter exceeds threshold, disable PWM and set error flag
		if(OVPCnt > 2)
		{
			// Reset OVP counter
			OVPCnt=0;
			// Disable PWM
			DF.PWMENFlag=0;
			HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TA1|HRTIM_OUTPUT_TA2); // Disable
			HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TB1|HRTIM_OUTPUT_TB2); // Disable		
			// Set overvoltage error flag
			setRegBits(DF.ErrFlag,F_SW_VOUT_OVP);
			// Transition to error state
			DF.SMFlag  =Err;
		}
	}
	else
		OVPCnt = 0;
}


/*
** ===================================================================
**     Function Name :void VinSwUVP(void)
**     Description : Detect Vin under-voltage condition and handle PWM shutdown
**     Parameters  : none
**     Returns     : none
** ===================================================================
*/
#define MIN_UVP_VAL    686 // 11.4V, calculated as 11.4/68 * Q12
#define MIN_UVP_VAL_RE 795 // 13.2V, calculated as 13.2/68 * Q12
void VinSwUVP(void)
{
	// Initialize under-voltage counters
	static uint16_t UVPCnt = 0;
	static uint16_t RSCnt = 0;

	// Check if Vin is below 11.4V for 200ms
	if ((SADC.Vin < MIN_UVP_VAL) && (DF.SMFlag != Init))
	{
		// Increment the under-voltage counter
		UVPCnt++;
		// Check after 10ms
		if (UVPCnt > 2)
		{
			// Reset counters
			UVPCnt = 0;
			RSCnt = 0;
			// Shutdown PWM
			DF.PWMENFlag = 0;
			HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2); // Shutdown
			HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TB1 | HRTIM_OUTPUT_TB2); // Shutdown		
			// Set error flag for Vin under-voltage protection
			setRegBits(DF.ErrFlag, F_SW_VIN_UVP);
			// Set state machine flag to Error
			DF.SMFlag = Err;
		}
	}
	else
	{
		UVPCnt = 0;
	}
	
	// Handle Vin under-voltage recovery
	// If Vin under-voltage error flag is set
	if (getRegBits(DF.ErrFlag, F_SW_VIN_UVP))
	{
		if (SADC.Vin > MIN_UVP_VAL_RE) 
		{
			// Increment recovery counter
			RSCnt++;
			// Check after 1 second
			if (RSCnt > 200)
			{
				RSCnt = 0;
				UVPCnt = 0;
				// Clear Vin under-voltage error flag
				clrRegBits(DF.ErrFlag, F_SW_VIN_UVP);
			}	
		}
		else	
		{
			RSCnt = 0;	
		}
	}
	else
	{
		RSCnt = 0;
	}
}

/*
** ===================================================================
**     Function Name :void VinSwOVP(void)
**     Description : Detect Vin over-voltage condition
**     Parameters  : none
**     Returns     : none
** ===================================================================
*/
#define MAX_VIN_OVP_VAL    3012 // 50V, calculated as 50/68 * Q12
void VinSwOVP(void)
{
	// Initialize over-voltage counter
	static uint16_t OVPCnt = 0;

	// Check if Vin is above 50V for 100ms
	if (SADC.Vin > MAX_VIN_OVP_VAL)
	{
		// Increment the over-voltage counter
		OVPCnt++;
		// Check after 10ms
		if (OVPCnt > 2)
		{
			// Reset counter
			OVPCnt = 0;
			// Shutdown PWM
			DF.PWMENFlag = 0;
			HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2); // Shutdown
			HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TB1 | HRTIM_OUTPUT_TB2); // Shutdown		
			// Set error flag for Vin over-voltage protection
			setRegBits(DF.ErrFlag, F_SW_VIN_OVP);
			// Set state machine flag to Error
			DF.SMFlag = Err;
		}
	}
	else
	{
		OVPCnt = 0;
	}
}


/** ===================================================================
**     Function Name :void LEDShow(void)
**     Description : LED indication display
**     - Shows different states: Init, Wait, Rise, Run, Error
**     - Controls Green, Yellow, and Red LEDs accordingly
**     Parameters  :
**     Returns     :
** ===================================================================*/
// LED control macros
#define SET_LED_G() HAL_GPIO_WritePin(GPIOB, LED_G_Pin, GPIO_PIN_SET) // Turn on Green LED
#define SET_LED_Y() HAL_GPIO_WritePin(GPIOB, LED_Y_Pin, GPIO_PIN_SET) // Turn on Yellow LED
#define SET_LED_R() HAL_GPIO_WritePin(GPIOB, LED_R_Pin, GPIO_PIN_SET) // Turn on Red LED
#define CLR_LED_G() HAL_GPIO_WritePin(GPIOB, LED_G_Pin, GPIO_PIN_RESET) // Turn off Green LED
#define CLR_LED_Y() HAL_GPIO_WritePin(GPIOB, LED_Y_Pin, GPIO_PIN_RESET) // Turn off Yellow LED
#define CLR_LED_R() HAL_GPIO_WritePin(GPIOB, LED_R_Pin, GPIO_PIN_RESET) // Turn off Red LED

void LEDShow(void)
{
	switch(DF.SMFlag)
	{
		// Initialization state
		case Init:
		{
			SET_LED_G();
			SET_LED_Y();
			SET_LED_R();
			break;
		}
		// Waiting state
		case Wait:
		{
			SET_LED_G();
			SET_LED_Y();
			SET_LED_R();
			break;
		}
		// Rising state
		case Rise:
		{
			SET_LED_G();
			SET_LED_Y();
			CLR_LED_R();
			break;
		}
		// Running state
		case Run:
		{
			SET_LED_G();
			CLR_LED_Y();
			CLR_LED_R();
			break;
		}
		// Error state
		case Err:
		{
			CLR_LED_G();
			CLR_LED_Y();
			SET_LED_R();
			break;
		}
	}
}






/** ===================================================================
**     Function Name :void BBMode(void)
**     Description : Buck-Boost mode selection
**          - BUCK mode when Vin < 0.8V
**          - BOOST mode when Vin > 1.2V
**          - MIX mode when 0.85V < Vin < 1.15V
**          - Avoiding conflicts between BUCK and BOOST modes
**     Parameters  :
**     Returns     :
** ===================================================================*/
void BBMode(void)
{
	DF.BBFlag = Buck; // Set to BUCK mode
}





/** ===================================================================
**     Funtion Name :void KEYFlag(void)
**     Description : The state of two buttons
**       Default state of KEYFlag is 0. When pressed, Flag becomes 1, and when pressed again, Flag becomes 0, cycling in this way.
**       When the machine is running normally or during startup, pressing the button will turn off the output and enter standby mode.
**     Parameters  :
**     Returns     :
** ===================================================================*/
#define READ_KEY1_INC_Freq() HAL_GPIO_ReadPin(GPIOA, KEY1_INC_Freq_Pin)
#define READ_KEY2_DEC_Freq() HAL_GPIO_ReadPin(GPIOA, KEY2_DEC_Freq_Pin)
void KEYFlag(void)
{
    // Timer, used for button debounce
    static uint16_t KeyDownCnt1 = 0, KeyDownCnt2 = 0;
    
    // Button pressed
    if (READ_KEY1_INC_Freq() == 0)
    {
        // Timing, button press is valid after 150 ms
        KeyDownCnt1++;
        if (KeyDownCnt1 > 30)
        {
            KeyDownCnt1 = 0; // Reset timer
            // Button state has changed
            if (DF.KeyFlag1 == 0)
                DF.KeyFlag1 = 1;
            else
                DF.KeyFlag1 = 0;
        }
    }
    else
        KeyDownCnt1 = 0; // Reset timer
    
    // Button pressed
    if (READ_KEY2_DEC_Freq() == 0)
    {
        // Timing, button press is valid after 150 ms
        KeyDownCnt2++;
        if (KeyDownCnt2 > 30)
        {
            KeyDownCnt2 = 0; // Reset timer
            // Button state has changed
            if (DF.KeyFlag2 == 0)
                DF.KeyFlag2 = 1;
            else
                DF.KeyFlag2 = 0;
        }
    }
    else
        KeyDownCnt2 = 0; // Reset timer

    // When the machine is running normally or during startup, pressing the button will turn off the output and enter standby mode
    if ((DF.KeyFlag1 == 0) && ((DF.SMFlag == Rise) || (DF.SMFlag == Run)))
    {
        DF.SMFlag = Wait;
        // Turn off PWM
        DF.PWMENFlag = 0;
        HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2); // Turn off
        HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TB1 | HRTIM_OUTPUT_TB2); // Turn off    
    }
}


/** ===================================================================
**     Function Name : void OLEDShow(void)
**     Description : OLED display routine		 
**     Display operating modes - BUCK MODE, BOOST MODE, MIX MODE
**     Display states: IDLE, RISING, RUNNING, ERROR
**     Display output voltage: converted values
**     Display output current: converted values
**     Parameters  :
**     Returns     :
** ===================================================================*/
void OLEDShow(void)
{
	u8 Vtemp[4] = {0, 0, 0, 0};
	u8 Itemp[4] = {0, 0, 0, 0};
	uint32_t VoutT = 0, IoutT = 0;
	//uint32_t VinT = 0, IinT = 0, VadjT = 0;
	static uint16_t BBFlagTemp = 10, SMFlagTemp = 10;
	
	// Calculate the output voltage and current, converted to 100x (for display adjustment) 
	VoutT = SADC.VoutAvg * 6800 >> 12;
	IoutT = (SADC.IoutAvg - 2048) * 2200 >> 12;
	//VinT = SADC.VinAvg * 6800 >> 12;
	//IinT = (SADC.IinAvg - 2048) * 2200 >> 12;
	//VadjT = CtrValue.Voref * 6800 >> 12;
	
	// Convert calculated output voltage and current to display format
	// Output voltage
	Vtemp[0] = (u8)(VoutT / 1000);
	Vtemp[1] = (u8)((VoutT - (uint8_t)Vtemp[0] * 1000) / 100);
	Vtemp[2] = (u8)((VoutT - (uint16_t)Vtemp[0] * 1000 - (uint16_t)Vtemp[1] * 100) / 10);
	Vtemp[3] = (u8)(VoutT - (uint16_t)Vtemp[0] * 1000 - (uint16_t)Vtemp[1] * 100 - (uint16_t)Vtemp[2] * 10);	
	// Input voltage
/*	Vtemp[0] = (u8)(VinT / 1000);
	Vtemp[1] = (u8)((VinT - (uint8_t)Vtemp[0] * 1000) / 100);
	Vtemp[2] = (u8)((VinT - (uint16_t)Vtemp[0] * 1000 - (uint16_t)Vtemp[1] * 100) / 10);
	Vtemp[3] = (u8)(VinT - (uint16_t)Vtemp[0] * 1000 - (uint16_t)Vtemp[1] * 100 - (uint16_t)Vtemp[2] * 10); */
	// Output current
	Itemp[0] = (u8)(IoutT / 1000);
	Itemp[1] = (u8)((IoutT - (uint8_t)Itemp[0] * 1000) / 100);
	Itemp[2] = (u8)((IoutT - (uint16_t)Itemp[0] * 1000 - (uint16_t)Itemp[1] * 100) / 10);
	Itemp[3] = (u8)(IoutT - (uint16_t)Itemp[0] * 1000 - (uint16_t)Itemp[1] * 100 - (uint16_t)Itemp[2] * 10);
	// Input current
/*	Itemp[0] = (u8)(IinT / 1000);
	Itemp[1] = (u8)((IinT - (uint8_t)Itemp[0] * 1000) / 100);
	Itemp[2] = (u8)((IinT - (uint16_t)Itemp[0] * 1000 - (uint16_t)Itemp[1] * 100) / 10);
	Itemp[3] = (u8)(IinT - (uint16_t)Itemp[0] * 1000 - (uint16_t)Itemp[1] * 100 - (uint16_t)Itemp[2] * 10); */
	// Reference voltage
/*	Vtemp[0] = (u8)(VadjT / 1000);
	Vtemp[1] = (u8)((VadjT - (uint8_t)Vtemp[0] * 1000) / 100);
	Vtemp[2] = (u8)((VadjT - (uint16_t)Vtemp[0] * 1000 - (uint16_t)Vtemp[1] * 100) / 10);
	Vtemp[3] = (u8)(VadjT - (uint16_t)Vtemp[0] * 1000 - (uint16_t)Vtemp[1] * 100 - (uint16_t)Vtemp[2] * 10); */
	
	// If the operating mode has changed, update display
	if(BBFlagTemp != DF.BBFlag)
	{
		// Save current flag status
		BBFlagTemp = DF.BBFlag;
		// Display operating mode
		switch(DF.BBFlag)
		{
			// NA
			case NA :		
			{
				OLED_ShowStr(55, 0, "Open Loop", 2);
				break;
			}
			// BUCK mode
			case Buck :		
			{
				OLED_ShowStr(25, 0, "MODE:BUCK ", 2);
				break;
			}
			// Boost mode
			case Boost :		
			{
				OLED_ShowStr(25, 0, "MODE:BOOST", 2);
				break;
			}
			// Mix mode
			case Mix :		
			{
				OLED_ShowStr(25, 0, "MODE:MIX ", 2);
				break;
			}
		}
	}
	
	// If the state has changed, update display
	if(SMFlagTemp != DF.SMFlag)
	{	
		SMFlagTemp = DF.SMFlag;
		// Display state
		switch(DF.SMFlag)
		{
			// Initialization state
			case Init :
			{
				//OLED_ShowStr(55, 2, "Init  ", 2);
				break;
			}
			// Waiting state
			case Wait :
			{
				//OLED_ShowStr(55, 2, "Waiting", 2);
				break;
			}
			// Rising state
			case Rise :
			{
				//OLED_ShowStr(55, 2, "Rising", 2);
				break;
			}
			// Running state
			case Run :
			{
				//OLED_ShowStr(55, 2, "Running", 2);
				break;
			}
			// Error state
			case Err :
			{
				//OLED_ShowStr(55, 2, "Error  ", 2);
				break;
			}
		}	
	}
	
	// Display voltage and current values
	OLEDShowData(50, 2, Vtemp[0]);
	OLEDShowData(60, 2, Vtemp[1]);
	OLEDShowData(75, 2, Vtemp[2]);
	OLEDShowData(85, 2, Vtemp[3]);

	OLEDShowData(50, 4, Vtemp[0]);
	OLEDShowData(60, 4, Vtemp[1]);
	OLEDShowData(75, 4, Vtemp[2]);
	OLEDShowData(85, 4, Vtemp[3]);

	OLEDShowData(50, 6, Itemp[0]);
	OLEDShowData(60, 6, Itemp[1]);
	OLEDShowData(75, 6, Itemp[2]);
	OLEDShowData(85, 6, Itemp[3]);
}





























/** ===================================================================
**     Function Name : void Button_Task(void)
**     Description    : The state of seven buttons

PWM Functionality:

TA1 -> Positive duty cycle 48%
TB1 -> Positive duty cycle 48%
TA2 -> Positive duty cycle 48%
TB2 -> Positive duty cycle 48%

TA1 and TB1 waveforms are complementary, TA2 and TB2 waveforms are complementary
-> When TA1 duty cycle is 48%, TB1 duty cycle is 52%
-> When TA2 duty cycle is 48%, TB2 duty cycle is 52%

TA1 and TA2 waveforms are identical, TB1 and TB2 waveforms are identical
TA1 and TB1 waveforms are complementary, TA2 and TB2 waveforms are complementary

1. TA1 and TB1 are complementary
2. TA2 and TB2 are complementary 
3. Dead time between TA1 and TB1 is between 2% and 50%, cannot be less than 2%
4. Dead time between TA2 and TB2 is between 2% and 50%, cannot be less than 2%
5. Initial frequency is 100KHz
6. Duty cycle is 50%

;---
Button Function Description:

1. PA6 -> Increase frequency of T1, TB1, TA2, TB2 simultaneously
          -> Upper limit 125K

2. PA7 -> Decrease frequency of T1, TB1, TA2, TB2 simultaneously
          -> Lower limit 77K

3. PB4 -> Increase dead time of TA1, TB1
           -> Upper limit 2%, calculated as 50-48 = 2%

4. PB5 -> Decrease dead time of TA1, TB1
           -> Lower limit 10%, calculated as 50-40 = 10%

5. PB6 -> Increase duty cycle of TA2, TB2 simultaneously
            -> Upper limit 48%

6. PB7  -> Decrease duty cycle of TA2, TB2 simultaneously
            -> Minimum to 0%

7. PB9  -> Switch operating mode
            -> Open loop / Closed loop

Button Function Description:
1. KEY1 increases frequency by 1% each time it is pressed
2. KEY2 decreases frequency by 1% each time it is pressed
3. KEY3 increases dead time by 1% each time it is pressed
4. KEY4 decreases dead time by 1% each time it is pressed
5. KEY5 increases duty cycle by 1% each time it is pressed
6. KEY6 decreases duty cycle by 1% each time it is pressed
7. KEY7 switches mode / open-loop or closed-loop

**     Parameters  :
**     Returns     :
** ===================================================================*/

// Define control modes
#define MODE_OPEN 0
#define MODE_CLOSE 1

// Minimum and maximum frequencies (Unit: Hz)
#define FREQ_MIN 70000.0f    // 70 kHz
#define FREQ_MAX 130000.0f   // 130 kHz
#define FREQ_STEP_PERCENT 0.1f  // Frequency adjustment step per button press: 0.1%

// Dead time parameters (Unit: 0.1%, each step adjusts by 0.1%)
#define DEADTIME_MIN_PX1000 0     // 0%
#define DEADTIME_MAX_PX1000 50    // 5.0%
#define DEADTIME_STEP_PX1000 1    // 0.1%

// Duty cycle parameters (Unit: 0.1%, each step adjusts by 0.1%)
#define DUTY_MIN_PX10 50    // 5.0%
#define DUTY_MAX_PX10 500   // 50.0%
#define DUTY_STEP_PX10 1     // 0.1%

// Assume CNTR_MIN and CNTR_MAX are defined as follows
#define CNTR_MIN 1000
#define CNTR_MAX 16000

// Global variables
extern HRTIM_HandleTypeDef hhrtim1;
extern HRTIM_TimeBaseCfgTypeDef pGlobalTimeBaseCfg;

volatile float currentPWMFreq = 100000.0f;        // Initial frequency 100 kHz
volatile uint8_t gCurrentDeadTimePercent = 2;     // Initial dead time 2%

// Initialize duty cycles
volatile uint8_t gCurrentDutyPercent_TA1_TB1 = 48; // Initial duty cycle for TA1/TB1 is 48%
volatile uint8_t gCurrentDutyPercent_TA2_TB2 = 48; // Initial duty cycle for TA2/TB2 is 48%

// Time base configuration structure
HRTIM_TimeBaseCfgTypeDef pGlobalTimeBaseCfg;

// Current PLL frequency
volatile uint32_t currentPLLFreq = 160000000;      // 160 MHz, set according to fHRCK




/** ===================================================================
**     Funtion Name :void Key_Scan(void)
**     Description : The state of two buttons
**       Default state of KEYFlag is 0. When pressed, Flag becomes 1, and when pressed again, Flag becomes 0, cycling in this way.
**       When the machine is running normally or during startup, pressing the button will turn off the output and enter standby mode.
**     Parameters  :
**     Returns     :
** ===================================================================*/
uint8_t Key_Scan(GPIO_TypeDef* GPIOx,uint16_t GPIO_Pin)
{			
	/* Check if the button is pressed */
	if(HAL_GPIO_ReadPin(GPIOx, GPIO_Pin) == KEY_OFF)  
	{	 
		/* Wait for the button to be released */
		while(HAL_GPIO_ReadPin(GPIOx,GPIO_Pin) == KEY_OFF);   
		return 	KEY_ON;	 
	}
	else
		return KEY_OFF;

}


/** ===================================================================
**     Funtion Name : Button_Task
**     Description : 
**     Parameters  :
**     Returns     :
** ===================================================================*/

void Button_Task(void)
{
	// KEY1/PA6 : Simultaneously increase frequency of T1, TB1, TA2, TB2
	if (Key_Scan(KEY1_INC_Freq_GPIO_Port, KEY1_INC_Freq_Pin) == KEY_ON)
	{
		if (currentPWMFreq < FREQ_MAX)
		{
			currentPWMFreq *= (1.0f + (FREQ_STEP_PERCENT / 100.0f)); // Increase by 0.1%
			if (currentPWMFreq > FREQ_MAX)
				currentPWMFreq = FREQ_MAX;

			// Update HRTIM frequency
			if (SetPWMFrequency((uint32_t)currentPWMFreq) != HAL_OK) {
				// Handle error
				Error_Handler();
			}
			HAL_GPIO_TogglePin(TEST_LED_GPIO_Port, TEST_LED_Pin);
		}
	}
	
	// KEY2/PA7 : Simultaneously decrease frequency of T1, TB1, TA2, TB2
	if (Key_Scan(KEY2_DEC_Freq_GPIO_Port, KEY2_DEC_Freq_Pin) == KEY_ON)
	{
		if (currentPWMFreq > FREQ_MIN)
		{
			currentPWMFreq *= (1.0f - (FREQ_STEP_PERCENT / 100.0f)); // Decrease by 0.1%
			if (currentPWMFreq < FREQ_MIN)
				currentPWMFreq = FREQ_MIN;

			// Update HRTIM frequency
			if (SetPWMFrequency((uint32_t)currentPWMFreq) != HAL_OK) {
				// Handle error
				Error_Handler();
			}
			HAL_GPIO_TogglePin(TEST_LED_GPIO_Port, TEST_LED_Pin);
		}
	}

    // KEY3/PB4: Increase dead time of TA1/TB1
    if (Key_Scan(KEY3_INC_DT_GPIO_Port, KEY3_INC_DT_Pin) == KEY_ON)
    {
		if (gCurrentDeadTimePercent < 50)
		{
			gCurrentDeadTimePercent += 1; // Increase by 0.1%

			if (gCurrentDeadTimePercent > 50)
				gCurrentDeadTimePercent = 50;

				// Manually adjust dead time
				SetDeadTimeManual(gCurrentDeadTimePercent);

				// Display dead time
				DisplayDeadTime((float)gCurrentDeadTimePercent * 0.1f);

				HAL_GPIO_TogglePin(TEST_LED_GPIO_Port, TEST_LED_Pin); 
		}
    }

	// KEY4/PB5: Decrease dead time of TA1/TB1
    if (Key_Scan(KEY4_DEC_DT_GPIO_Port, KEY4_DEC_DT_Pin) == KEY_ON)
    {
		if (gCurrentDeadTimePercent > 0)
		{
			gCurrentDeadTimePercent -= 1; // Decrease by 0.1%

			//if (gCurrentDeadTimePercent < 1)
			//	gCurrentDeadTimePercent = 1;

					// Manually adjust dead time
					SetDeadTimeManual(gCurrentDeadTimePercent);

					// Display dead time
					DisplayDeadTime((float)gCurrentDeadTimePercent * 0.1f);


					HAL_GPIO_TogglePin(TEST_LED_GPIO_Port, TEST_LED_Pin); 
			}
    }

	// KEY5/PB6: Simultaneously increase duty cycle of TA2/TB2
    if (Key_Scan(KEY5_INC_DUTY_GPIO_Port, KEY5_INC_DUTY_Pin) == KEY_ON)
    {
		if (gCurrentDutyPercent_TA2_TB2 < 50)
		{
			gCurrentDutyPercent_TA2_TB2 += 1; // Increase by 0.1%

			//if (gCurrentDutyPercent_TA2_TB2 > 45)
			//	gCurrentDutyPercent_TA2_TB2 = 45;
	
				// Update duty cycle
				//if (SetDutyCycle_TA2_TB2(gCurrentDutyPercent_TA2_TB2) != HAL_OK) {
					// Handle error
				//	Error_Handler();
				//}

					// Display duty cycle
					DisplayDutyCycle(gCurrentDutyPercent_TA2_TB2);

					HAL_GPIO_TogglePin(TEST_LED_GPIO_Port, TEST_LED_Pin); 
			}
    }
	// KEY6/PB7: Simultaneously decrease duty cycle of TA2/TB2
    if (Key_Scan(KEY6_DEC_DUTY_GPIO_Port, KEY6_DEC_DUTY_Pin) == KEY_ON)
    {
			if (gCurrentDutyPercent_TA2_TB2 > 0)
			{
				gCurrentDutyPercent_TA2_TB2 -= 1; // Decrease by 0.1%

				//if (gCurrentDutyPercent_TA2_TB2 < 5)
				//	gCurrentDutyPercent_TA2_TB2 = 5;
	
				// Update duty cycle
				//if (SetDutyCycle_TA2_TB2(gCurrentDutyPercent_TA2_TB2) != HAL_OK) {
					// Handle error
				//	Error_Handler();
				//}
					// Display duty cycle
					DisplayDutyCycle(gCurrentDutyPercent_TA2_TB2);

					HAL_GPIO_TogglePin(TEST_LED_GPIO_Port, TEST_LED_Pin); // Toggle LED
			}
    }

    // KEY7/PB9: Switch operating mode
    if (Key_Scan(KEY7_SWITCH_MODE_GPIO_Port, KEY7_SWITCH_MODE_Pin) == KEY_ON)
    {
		Mode_Switch(); // Switch mode
    }
	// Update OLED display
	UpdateDisplay();

}


/** ===================================================================
**     Function Name : SetPWMFrequency
**     Description : 
    
Currently using 100MHz for prescaling, timebase = 16000 = 100KHz
Mid-cycle point 50% = 8000

1. Duty cycle cannot exceed 50%
2. Frequency upper limit is 130KHz, lower limit is 70KHz
3. Dead time upper limit is 2%
    
**     Parameters  :req_tim_freq - Requested PWM frequency (Hz).
**     Returns     :HAL_StatusTypeDef - Returns HAL_OK on success, HAL_ERROR on failure.
** ===================================================================*/
HAL_StatusTypeDef SetPWMFrequency(uint32_t req_tim_freq) {
    uint32_t prescaler_value;
    uint32_t fHRCK;
    uint32_t period;

    // Define frequency range
    if (req_tim_freq < FREQ_MIN || req_tim_freq > FREQ_MAX) {
        return HAL_ERROR; // Frequency out of range
    }

    if (req_tim_freq >= 100000) {
        // Use prescaler = MUL16
        prescaler_value = 16;
        fHRCK = 100000000UL * prescaler_value; // 100MHz * 16 = 1.6GHz
    } else {
        // Use prescaler = MUL8
        prescaler_value = 8;
        fHRCK = 100000000UL * prescaler_value; // 100MHz * 8 = 800MHz (adjust according to actual clock configuration)
    }

    // Calculate new Period
    period = fHRCK / req_tim_freq;

    // Verify if Period is within allowed range
    if (period < CNTR_MIN || period > CNTR_MAX) {
        return HAL_ERROR;
    }

    // Update global time base configuration's Period and PrescalerRatio
    pGlobalTimeBaseCfg.Period = period;
    if (prescaler_value == 16) {
        pGlobalTimeBaseCfg.PrescalerRatio = HRTIM_PRESCALERRATIO_MUL16;
    } else if (prescaler_value == 8) {
        pGlobalTimeBaseCfg.PrescalerRatio = HRTIM_PRESCALERRATIO_MUL8;
    } else {
        return HAL_ERROR; // Unsupported prescaler
    }

    // Configure Timer A's time base
    if (HAL_HRTIM_TimeBaseConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, &pGlobalTimeBaseCfg) != HAL_OK) {
        return HAL_ERROR;
    }

    // Configure Timer B's time base
    if (HAL_HRTIM_TimeBaseConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, &pGlobalTimeBaseCfg) != HAL_OK) {
        return HAL_ERROR;
    }

    // Configure Compare Unit 2 as midpoint value (50% duty cycle)
    HRTIM_CompareCfgTypeDef pCompareCfg = {0};
    pCompareCfg.CompareValue = (period / 2) - 1;
    pCompareCfg.AutoDelayedMode = HRTIM_AUTODELAYEDMODE_REGULAR;
    pCompareCfg.AutoDelayedTimeout = 0x0000;

    // Configure Timer A's Compare Unit 2
    if (HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_2, &pCompareCfg) != HAL_OK) {
        return HAL_ERROR;
    }

    // Configure Timer B's Compare Unit 2
    if (HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, HRTIM_COMPAREUNIT_2, &pCompareCfg) != HAL_OK) {
        return HAL_ERROR;
    }

    // Software reset HRTIM's Timer A and Timer B to apply new settings
    if (HAL_HRTIM_SoftwareReset(&hhrtim1, HRTIM_TIMERRESET_TIMER_A | HRTIM_TIMERRESET_TIMER_B) != HAL_OK) {
        return HAL_ERROR;
    }

    // Update global frequency variables
    currentPWMFreq = req_tim_freq;
    currentPLLFreq = fHRCK;

    return HAL_OK;
}


/**
  * @brief  Manually set dead time.
  * @param  dead_time_percent - New dead time (unit: 0.1%).
  * @retval None
  */
void SetDeadTimeManual(uint8_t dead_time_percent)
{
    // Ensure dead time is within allowed range
    if (dead_time_percent > DEADTIME_MAX_PX1000) {
        return;
    }

    uint32_t period = pGlobalTimeBaseCfg.Period;
    uint32_t dead_time_ticks = (period * dead_time_percent) / 1000; // 0.1% = 1/1000

    // Adjust Timer A Compare Unit 2 (midpoint) for TA1's ResetSource
    HRTIM_CompareCfgTypeDef compareConfigA = {0};
    compareConfigA.CompareValue = dead_time_ticks;
    compareConfigA.AutoDelayedMode = HRTIM_AUTODELAYEDMODE_REGULAR;
    compareConfigA.AutoDelayedTimeout = 0x0000;

    if (HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_2, &compareConfigA) != HAL_OK) {
        Error_Handler();
    }

    // Adjust Timer B Compare Unit 2 (midpoint) for TB1's ResetSource
    HRTIM_CompareCfgTypeDef compareConfigB = {0};
    compareConfigB.CompareValue = period - dead_time_ticks;
    compareConfigB.AutoDelayedMode = HRTIM_AUTODELAYEDMODE_REGULAR;
    compareConfigB.AutoDelayedTimeout = 0x0000;

    if (HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, HRTIM_COMPAREUNIT_2, &compareConfigB) != HAL_OK) {
        Error_Handler();
    }

    // Software reset HRTIM's Timer A and Timer B to apply new settings
    if (HAL_HRTIM_SoftwareReset(&hhrtim1, HRTIM_TIMERRESET_TIMER_A | HRTIM_TIMERRESET_TIMER_B) != HAL_OK) {
        Error_Handler();
    }

    // Start Timer A and Timer B
    if (HAL_HRTIM_WaveformCounterStart(&hhrtim1, HRTIM_TIMERID_TIMER_A | HRTIM_TIMERID_TIMER_B) != HAL_OK) {
        Error_Handler();
    }
}




/**
  * @brief  Set PWM duty cycle for TA1/TB1.
  * @param  duty_percent - New duty cycle (percentage).
  * @retval HAL_StatusTypeDef - Returns HAL_OK on success, HAL_ERROR on failure.
  */
HAL_StatusTypeDef SetDutyCycle_TA1_TB1(uint8_t duty_percent)
{
    if (duty_percent < 5 || duty_percent > 95)
    {
        return HAL_ERROR;
    }

    uint32_t period = pGlobalTimeBaseCfg.Period;
    uint32_t compare_value = (period * duty_percent) / 100;

    // Update Timer A Compare Unit 2 (midpoint) corresponding to TA1's ResetSource
    HRTIM_CompareCfgTypeDef compareConfig = {0};
    compareConfig.CompareValue = compare_value;
    compareConfig.AutoDelayedMode = HRTIM_AUTODELAYEDMODE_REGULAR;
    compareConfig.AutoDelayedTimeout = 0x0000;

    if (HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_2, &compareConfig) != HAL_OK) {
        return HAL_ERROR;
    }

    if (HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, HRTIM_COMPAREUNIT_2, &compareConfig) != HAL_OK) {
        return HAL_ERROR;
    }

    return HAL_OK;
}


/**
  * @brief  Set PWM duty cycle for TA2/TB2.
  * @param  duty_percent - New duty cycle (percentage).
  * @retval HAL_StatusTypeDef - Returns HAL_OK on success, HAL_ERROR on failure.
  */
HAL_StatusTypeDef SetDutyCycle_TA2_TB2(uint8_t duty_percent)
{
    if (duty_percent < 5 || duty_percent > 45)
    {
        return HAL_ERROR;
    }

    uint32_t period = pGlobalTimeBaseCfg.Period;
    uint32_t compare_value = (period * duty_percent) / 100;

    // Update Timer A Compare Unit 2 (midpoint) corresponding to TA2's ResetSource
    HRTIM_CompareCfgTypeDef compareConfig = {0};
    compareConfig.CompareValue = compare_value;
    compareConfig.AutoDelayedMode = HRTIM_AUTODELAYEDMODE_REGULAR;
    compareConfig.AutoDelayedTimeout = 0x0000;

    if (HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_2, &compareConfig) != HAL_OK) {
        return HAL_ERROR;
    }

    if (HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, HRTIM_COMPAREUNIT_2, &compareConfig) != HAL_OK) {
        return HAL_ERROR;
    }

    return HAL_OK;
}





/*
** ===================================================================
**     Function Name :   void ADCSample(void)
**     Description :    Samples Vin, Iin, Vout, and Iout
**     Parameters  :
**     Returns     :
** ===================================================================
*/

struct _ADI SADC={2048,2048,0,0,2048,2048,0,0,0,0}; // Input and output parameter sampling values and average values
struct _Ctr_value CtrValue={0,0,0,MIN_BUKC_DUTY,0,0,0}; // Control parameters
struct _FLAG DF={0,0,0,0,0,0,0,0}; // Control flag bits
uint16_t ADC1_RESULT[4]={0,0,0,0}; // DMA data storage register for transferring ADC samples from peripheral to memory




CCMRAM void ADCSample(void)
{
	// Declare variables for averaging Vin, Iin, Vout, and Iout
	static uint32_t VinAvgSum=0, IinAvgSum=0, VoutAvgSum=0, IoutAvgSum=0;
	
	// Convert ADC readings using calibration factors (Q15 format), including offset compensation
	SADC.Vin  = ((uint32_t)ADC1_RESULT[0] * CAL_VIN_K >> 12) + CAL_VIN_B;
	SADC.Iin  = ((uint32_t)ADC1_RESULT[1] * CAL_IIN_K >> 12) + CAL_IIN_B;
	SADC.Vout = ((uint32_t)ADC1_RESULT[2] * CAL_VOUT_K >> 12) + CAL_VOUT_B;
	SADC.Iout = ((uint32_t)ADC1_RESULT[3] * CAL_IOUT_K >> 12) + CAL_IOUT_B;

	// Check for invalid readings; if Vin is below the threshold, set it to 0
	if(SADC.Vin < 100) 
		SADC.Vin = 0;
	
	// If Iin is less than 2048, set it to 2048 (0A)
	if(SADC.Iin < 2048) 
		SADC.Iin = 2048;
	
	if(SADC.Vout < 100)
		SADC.Vout = 0;
	
	if(SADC.Iout < 2048)
		SADC.Iout = 2048;

	// Calculate average values of Vin, Iin, Vout, and Iout using moving average
	VinAvgSum = VinAvgSum + SADC.Vin - (VinAvgSum >> 2); // Add current Vin value and subtract the oldest value
	SADC.VinAvg = VinAvgSum >> 2; // Update Vin average
	
	IinAvgSum = IinAvgSum + SADC.Iin - (IinAvgSum >> 2); // Add current Iin value and subtract the oldest value
	SADC.IinAvg = IinAvgSum >> 2; // Update Iin average
	
	VoutAvgSum = VoutAvgSum + SADC.Vout - (VoutAvgSum >> 2); // Add current Vout value and subtract the oldest value
	SADC.VoutAvg = VoutAvgSum >> 2; // Update Vout average
	
	IoutAvgSum = IoutAvgSum + SADC.Iout - (IoutAvgSum >> 2); // Add current Iout value and subtract the oldest value
	SADC.IoutAvg = IoutAvgSum >> 2; // Update Iout average
}



#define MODE_OPEN_LOOP 0
#define MODE_CLOSED_LOOP 1
// Mode variable, default is open-loop mode
volatile uint8_t currentMode = MODE_OPEN_LOOP;

/**
  * @brief  Switch mode function
  * @retval None
  */
void Mode_Switch(void)
{
    if (currentMode == MODE_OPEN_LOOP)
    {
        currentMode = MODE_CLOSED_LOOP;

        // Initialize frequency to 100 kHz
        currentPWMFreq = 100000.0f;
        if (SetPWMFrequency((uint32_t)currentPWMFreq) != HAL_OK)
        {
            Error_Handler();
        }

        // Display mode change
        OLED_ShowStr(55, 0, "Close", 2);
    }
    else
    {
        currentMode = MODE_OPEN_LOOP;

        // Initialize frequency to 100 kHz
        currentPWMFreq = 100000.0f;
        if (SetPWMFrequency((uint32_t)currentPWMFreq) != HAL_OK)
        {
            Error_Handler();
        }


        OLED_ShowStr(55, 0, "      ", 2); // Clear previous display
        OLED_ShowStr(55, 0, "Open", 2);   // Display mode change

		Open_Mode_Init();

    }

    HAL_GPIO_TogglePin(TEST_LED_GPIO_Port, TEST_LED_Pin); // Toggle LED to indicate mode change
}




/**
  * @brief  Update OLED display function
  * @retval None
  */
void UpdateDisplay(void)
{
    // Display different information based on mode
    if (currentMode == MODE_CLOSED_LOOP)
    {
        // Closed-loop mode: Display frequency converted from ADC voltage value

        // Ensure ADC has sampled
		HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADC1_RESULT, 4); // Start ADC1 sampling, DMA transfer for sampling input/output voltage and current
		HAL_ADC_Start(&hadc1); // Start ADC2 sampling, sampling the sliding potentiometer voltage

        ADCSample();

        // Convert ADC voltage to 0-3.3V
        float adc_voltage = (SADC.VinAvg / 4095.0f) * 3.3f;

        // Calculate corresponding frequency
        // Mid value 1.65V corresponds to 100 kHz
        // Set frequency range from 50 kHz to 150 kHz
        float frequency = 100000.0f + ((adc_voltage - 1.65f) / 1.65f) * 50000.0f; // ∮50 kHz

        // Limit frequency range
        if (frequency < FREQ_MIN)
            frequency = FREQ_MIN;
        if (frequency > FREQ_MAX)
            frequency = FREQ_MAX;

		// Update global frequency variable
		currentPWMFreq = frequency;
		//currentPLLFreq = fHRCK; // Ensure fHRCK is correctly calculated and defined
		
		// Set PWM frequency
		if (SetPWMFrequency((uint32_t)currentPWMFreq) != HAL_OK)
		{
			Error_Handler();
		}
		
		// Display frequency, keeping two decimal places
		unsigned char freqStr[10]; // Changed to unsigned char array
		sprintf((char *)freqStr, "%.2f", currentPWMFreq / 1000.0f); // For example, "100.00KHz"
		
		// Clear previous display area (adjust number of spaces as needed)
		//OLED_ShowStr(45, 2, "		 ", 2); // Clear previous display at (45,2)
		// Display new frequency
		OLED_ShowStr(45, 2, (unsigned char *)(freqStr), 2); // Display current frequency at (45,2)


	 	// Display ADC voltage value
		// Convert ADC voltage to 0-3.3V
		float adc_voltage_display = (SADC.VinAvg / 4095.0f) * 3.3f;
	
		// Convert voltage value to display format (mV)
		uint8_t Vtemp[4] = {0};
		uint32_t Vdisplay = (uint32_t)(adc_voltage_display * 1000.0f); // Convert to mV
	
		Vtemp[0] = (uint8_t)(Vdisplay / 1000);
		Vtemp[1] = (uint8_t)((Vdisplay % 1000) / 100);
		Vtemp[2] = (uint8_t)((Vdisplay % 100) / 10);
		Vtemp[3] = (uint8_t)(Vdisplay % 10);
	
		// Display ADC voltage
		OLEDShowData(50, 6, Vtemp[0]);
		OLEDShowData(65, 6, Vtemp[1]);
		OLEDShowData(75, 6, Vtemp[2]);
		OLEDShowData(85, 6, Vtemp[3]);

    }
    else
    {
        // Open-loop mode: Display manually set frequency

        // Display frequency
        //unsigned char freqStr[10];
        //sprintf(freqStr, "%.1fKHz", currentPWMFreq / 1000.0f);
        //OLED_ShowStr(60, 6, "     ", 2); // Clear previous display
        //OLED_ShowStr(60, 6, freqStr, 2);

		//OLED_ShowStr(50, 6, "	     ", 2); // Clear previous display
		//OLED_ShowStr(30, 6, " STOP   ", 2);   // Display mode change

		//OLED_ShowStr(0, 6, "ADC:", 2);
		//OLED_ShowStr(60, 6, ".", 2);
		//OLED_ShowStr(95, 6, "V", 2);

		// Display frequency, keeping two decimal places
		 unsigned char freqStr[10]; // Use unsigned char
        sprintf((char *)freqStr, "%.2f", currentPWMFreq / 1000.0f); // "100.00KHz"
		//OLED_ShowStr(45, 2, "		 ", 2); // Clear previous display
		OLED_ShowStr(45, 2, freqStr, 2); // Display current frequency
	


    }


    // Optional: Display other data, such as Iout, etc.
}



/** ===================================================================
**     Function Name : void MX_OLED_Init(void)
**     Description : OLED initialization routine
**     Initialize the OLED display interface
**     Parameters  :
**     Returns     :
** ===================================================================*/
void Open_Mode_Init(void)
{
	currentMode = MODE_OPEN_LOOP;

	// Initialize the OLED
	OLED_Init();
	OLED_CLS(); // Clear the OLED display
	
	// Display initial text labels on the OLED
	OLED_ShowStr(0, 0, "Mode:", 2);
	OLED_ShowStr(55, 0, "Open", 2);   // Display mode change

	OLED_ShowStr(0, 2, "Freq:", 2);
	OLED_ShowStr(68, 2, ".", 2);
	OLED_ShowStr(100, 2, "KHz", 2);

	OLED_ShowStr(0, 4, "Du/DT:", 2);
	OLED_ShowStr(85, 4, "/", 2);
	OLED_ShowStr(120, 4, "%", 2);

	// Display dead time
	gCurrentDeadTimePercent = 2;
	DisplayDeadTime((float)gCurrentDeadTimePercent);

	// Display duty cycle
	gCurrentDutyPercent_TA2_TB2 = 48;
	DisplayDutyCycle(gCurrentDutyPercent_TA2_TB2);

	OLED_ShowStr(0, 6, "ADC:", 2);
	OLED_ShowStr(60, 6, ".", 2);
	OLED_ShowStr(95, 6, "V", 2);

	// Display ADC voltage
	uint8_t Vtemp[4] = {0};
	OLEDShowData(50, 6, Vtemp[0]);
	OLEDShowData(65, 6, Vtemp[1]);
	OLEDShowData(75, 6, Vtemp[2]);
	OLEDShowData(85, 6, Vtemp[3]);

	OLED_ON(); // Turn on the OLED display
}




/** ===================================================================
**     Function Name : void MX_OLED_Init(void)
**     Description : OLED initialization routine
**     Initialize the OLED display interface
**     Parameters  :
**     Returns     :
** ===================================================================*/
void MX_OLED_Init(void)
{
	// Initialize the OLED
	OLED_Init();
	OLED_CLS(); // Clear the OLED display
	
	// Display initial text labels on the OLED
	OLED_ShowStr(0, 0, "Mode:", 2);

	OLED_ShowStr(0, 2, "ADC:", 2);
	OLED_ShowStr(60, 2, ".", 2);
	OLED_ShowStr(95, 2, "V", 2);

	OLED_ShowStr(0, 4, "Duty:", 2);
	OLED_ShowStr(68, 4, ".", 2);
	OLED_ShowStr(95, 4, "%", 2);

	OLED_ShowStr(0, 6, "Freq:", 2);
	OLED_ShowStr(68, 6, ".", 2);
	OLED_ShowStr(95, 6, "KHz", 2);

	OLED_ON(); // Turn on the OLED display
}



/**
  * @brief  Display Duty Cycle on OLED
  * @param  duty_percent - Current duty cycle percentage (e.g., 36.0 represents 36.0%)
  * @retval None
  */
void DisplayDutyCycle(float duty_percent)
{
    // Limit duty cycle within allowed range
    //if (duty_percent < (float)DUTY_MIN_PX10 / 10.0f)
    //    duty_percent = (float)DUTY_MIN_PX10 / 10.0f;
    //if (duty_percent > (float)DUTY_MAX_PX10 / 10.0f)
    //    duty_percent = (float)DUTY_MAX_PX10 / 10.0f;

    // Format duty cycle string, keeping one decimal place
    unsigned char dutyStr[10];
    sprintf((char *)dutyStr, "%.1f", duty_percent);

    // Display duty cycle, adjust coordinates according to OLED initialization
    // Assume "Duty:" label is at (0,4), value is displayed at (50,4)
    OLED_ShowStr(50, 4, dutyStr, 2); // Adjust x, y coordinates to fit your display layout
}

/**
  * @brief  Display Dead Time on OLED
  * @param  dead_time_percent - Current dead time percentage (e.g., 2.0 represents 2.0%)
  * @retval None
  */
void DisplayDeadTime(float dead_time_percent)
{
    // Limit dead time within allowed range
    //if (dead_time_percent < (float)DEADTIME_MIN_PX1000 / 10.0f)
    //    dead_time_percent = (float)DEADTIME_MIN_PX1000 / 10.0f;
    //if (dead_time_percent > (float)DEADTIME_MAX_PX1000 / 10.0f)
    //    dead_time_percent = (float)DEADTIME_MAX_PX1000 / 10.0f;

    // Format dead time string, keeping one decimal place
    unsigned char deadTimeStr[10];
    sprintf((char *)deadTimeStr, "%.1f", dead_time_percent);

    // Display dead time, adjust coordinates according to OLED initialization
    // Assume "Du/DT:" label is at (0,4), value is displayed at (95,4)
    OLED_ShowStr(95, 4, deadTimeStr, 2); // Adjust x, y coordinates to fit your display layout
}





/**
  * @brief  Update HRTIM PWM frequency
  * @param  period: Timer period
  * @param  half_period: Half period
  * @param  duty_cycle: Duty cycle
  * @param  dead_time: Dead time
  * @retval HAL_StatusTypeDef: Returns HAL_OK on success, HAL_ERROR on failure
  */
void UpdateHRTIM(int period, int half_period, int duty_cycle, int dead_time)
{
    /* USER CODE END HRTIM1_Init 0 */

    HRTIM_TimeBaseCfgTypeDef timeBaseConfig = {0};
    HRTIM_TimerCfgTypeDef timerConfig = {0};
    HRTIM_CompareCfgTypeDef compareConfig = {0};
    HRTIM_TimerCtlTypeDef timerControl = {0};
    HRTIM_OutputCfgTypeDef outputConfig = {0};

    /* USER CODE BEGIN HRTIM1_Init 1 */

    /* USER CODE END HRTIM1_Init 1 */
    hhrtim1.Instance = HRTIM1;
    hhrtim1.Init.HRTIMInterruptResquests = HRTIM_IT_NONE;
    hhrtim1.Init.SyncOptions = HRTIM_SYNCOPTION_NONE;
    if (HAL_HRTIM_Init(&hhrtim1) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_HRTIM_DLLCalibrationStart(&hhrtim1, HRTIM_CALIBRATIONRATE_3) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_HRTIM_PollForDLLCalibration(&hhrtim1, 10) != HAL_OK)
    {
        Error_Handler();
    }

    // Configure time base for Master timer
    timeBaseConfig.Period = period;
    timeBaseConfig.RepetitionCounter = 0x00; // Set repetition counter to 0
    timeBaseConfig.PrescalerRatio = HRTIM_PRESCALERRATIO_MUL16; // Set prescaler ratio to 16x
    timeBaseConfig.Mode = HRTIM_MODE_CONTINUOUS; // Set timer to continuous mode

    // Apply time base configuration to Master timer, handle error if fails
    if (HAL_HRTIM_TimeBaseConfig(&hhrtim1, HRTIM_TIMERINDEX_MASTER, &timeBaseConfig) != HAL_OK)
    {
        Error_Handler();
    }

    // Configure control parameters for Master timer
    timerConfig.InterruptRequests = HRTIM_MASTER_IT_NONE; // Disable Master timer interrupt requests
    timerConfig.DMARequests = HRTIM_MASTER_DMA_NONE;       // Disable Master timer DMA requests
    timerConfig.DMASrcAddress = 0x0000;                   // Set DMA source address to 0
    timerConfig.DMADstAddress = 0x0000;                   // Set DMA destination address to 0
    timerConfig.DMASize = 0x1;                            // Set DMA transfer size to 1
    timerConfig.HalfModeEnable = HRTIM_HALFMODE_DISABLED; // Disable half mode
    timerConfig.InterleavedMode = HRTIM_INTERLEAVED_MODE_DISABLED; // Disable interleaved mode
    timerConfig.StartOnSync = HRTIM_SYNCSTART_DISABLED;    // Disable synchronous start
    timerConfig.ResetOnSync = HRTIM_SYNCRESET_DISABLED;    // Disable synchronous reset
    timerConfig.DACSynchro = HRTIM_DACSYNC_NONE;           // Disable DAC synchronization
    timerConfig.PreloadEnable = HRTIM_PRELOAD_DISABLED;    // Disable preload
    timerConfig.UpdateGating = HRTIM_UPDATEGATING_INDEPENDENT; // Set update gating to independent
    timerConfig.BurstMode = HRTIM_TIMERBURSTMODE_MAINTAINCLOCK; // Set burst mode to maintain clock
    timerConfig.RepetitionUpdate = HRTIM_UPDATEONREPETITION_DISABLED; // Disable repetition update
    timerConfig.ReSyncUpdate = HRTIM_TIMERESYNC_UPDATE_UNCONDITIONAL; // Set resync update to unconditional

    // Apply Master timer control configuration, handle error if fails
    if (HAL_HRTIM_WaveformTimerConfig(&hhrtim1, HRTIM_TIMERINDEX_MASTER, &timerConfig) != HAL_OK)
    {
        Error_Handler();
    }

    // Configure Compare Unit 1 of Master timer to set TA1 and TB1 reset points
    compareConfig.CompareValue = half_period * period / 16000; // Set compare value, frequency 50%
    if (HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_MASTER, HRTIM_COMPAREUNIT_1, &compareConfig) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_HRTIM_TimeBaseConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, &timeBaseConfig) != HAL_OK)
    {
        Error_Handler();
    }

    // Configure Timer A control parameters
    timerControl.UpDownMode = HRTIM_TIMERUPDOWNMODE_UP; // Set counting mode to up
    timerControl.TrigHalf = HRTIM_TIMERTRIGHALF_DISABLED; // Disable half-cycle trigger
    timerControl.GreaterCMP1 = HRTIM_TIMERGTCMP1_EQUAL; // Set compare condition to equal
    timerControl.DualChannelDacEnable = HRTIM_TIMER_DCDE_DISABLED; // Disable dual-channel DAC

    // Apply Timer A control configuration, handle error if fails
    if (HAL_HRTIM_WaveformTimerControl(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, &timerControl) != HAL_OK)
    {
        Error_Handler();
    }

    // Configure Compare Unit 1 of Timer A to reset TA1 before dead time
    timerConfig.InterruptRequests = HRTIM_TIM_IT_NONE; // Disable Timer A interrupt requests
    timerConfig.DMARequests = HRTIM_TIM_DMA_NONE;       // Disable Timer A DMA requests
    timerConfig.PushPull = HRTIM_TIMPUSHPULLMODE_DISABLED; // Disable push-pull mode
    timerConfig.FaultEnable = HRTIM_TIMFAULTENABLE_NONE;    // Disable faults
    timerConfig.FaultLock = HRTIM_TIMFAULTLOCK_READWRITE;   // Set fault lock to read/write
    timerConfig.DeadTimeInsertion = HRTIM_TIMDEADTIMEINSERTION_DISABLED; // Disable dead time insertion
    timerConfig.DelayedProtectionMode = HRTIM_TIMER_A_B_C_DELAYEDPROTECTION_DISABLED; // Disable delayed protection mode
    timerConfig.UpdateTrigger = HRTIM_TIMUPDATETRIGGER_NONE; // Disable update trigger
    timerConfig.ResetTrigger = HRTIM_TIMRESETTRIGGER_MASTER_PER; // Set reset trigger source to Master timer period
    timerConfig.ResetUpdate = HRTIM_TIMUPDATEONRESET_DISABLED; // Disable update on reset

	// Apply Compare Unit configuration for Timer A, handle error if fails
	if (HAL_HRTIM_WaveformTimerConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, &timerConfig) != HAL_OK)
	{
	  Error_Handler();
	}


    // Set Timer B's reset trigger source to Compare Unit 1 of Master timer
    timerConfig.ResetTrigger = HRTIM_TIMRESETTRIGGER_MASTER_CMP1;

    // Apply Compare Unit configuration for Timer B, handle error if fails
    if (HAL_HRTIM_WaveformTimerConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, &timerConfig) != HAL_OK)
    {
        Error_Handler();
    }

    // Set Compare Unit 1 of Timer A, new compare value = (half_period * period / 16000) - dead_time
    compareConfig.CompareValue = half_period * period / 16000 - dead_time; // 7680 / frequency minus dead time 48%
    if (HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_1, &compareConfig) != HAL_OK)
    {
        Error_Handler();
    }
    // Set Compare Unit 2 of Timer A, set duty cycle to duty_cycle * period / 16000
    compareConfig.CompareValue = duty_cycle * period / 16000; // 3600 / period 36%
    compareConfig.AutoDelayedMode = HRTIM_AUTODELAYEDMODE_REGULAR;
    compareConfig.AutoDelayedTimeout = 0x0000;

    if (HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_2, &compareConfig) != HAL_OK)
    {
        Error_Handler();
    }
    // Configure output parameters for TA1 and TB1
    outputConfig.Polarity = HRTIM_OUTPUTPOLARITY_HIGH; // Set output polarity to high
    outputConfig.SetSource = HRTIM_OUTPUTSET_TIMPER;    // Set output set source to Master timer period
    outputConfig.ResetSource = HRTIM_OUTPUTRESET_TIMCMP1; // Set output reset source to Compare Unit 1
    outputConfig.IdleMode = HRTIM_OUTPUTIDLEMODE_NONE; // Set idle mode to none
    outputConfig.IdleLevel = HRTIM_OUTPUTIDLELEVEL_INACTIVE; // Set idle level to inactive
    outputConfig.FaultLevel = HRTIM_OUTPUTFAULTLEVEL_NONE; // Set fault level to none
    outputConfig.ChopperModeEnable = HRTIM_OUTPUTCHOPPERMODE_DISABLED; // Disable chopper mode
    outputConfig.BurstModeEntryDelayed = HRTIM_OUTPUTBURSTMODEENTRY_REGULAR; // Set burst mode entry to regular

    // Configure TA1 output, handle error if fails
    if (HAL_HRTIM_WaveformOutputConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_OUTPUT_TA1, &outputConfig) != HAL_OK)
    {
        Error_Handler();
    }
    // Configure TB1 output, handle error if fails
    if (HAL_HRTIM_WaveformOutputConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, HRTIM_OUTPUT_TB1, &outputConfig) != HAL_OK)
    {
        Error_Handler();
    }

    // Change ResetSource to Compare Unit 2 for TA2 and TB2 reset points
    outputConfig.ResetSource = HRTIM_OUTPUTRESET_TIMCMP2;

    // Configure TA2 output, handle error if fails
    if (HAL_HRTIM_WaveformOutputConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_OUTPUT_TA2, &outputConfig) != HAL_OK)
    {
        Error_Handler();
    }
    // Configure TB2 output, handle error if fails
    if (HAL_HRTIM_WaveformOutputConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, HRTIM_OUTPUT_TB2, &outputConfig) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_HRTIM_TimeBaseConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, &timeBaseConfig) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_HRTIM_WaveformTimerControl(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, &timerControl) != HAL_OK)
    {
        Error_Handler();
    }
    // Set Compare Unit 1 of Timer B, set compare value to (half_period * period / 16000) - dead_time
    compareConfig.CompareValue = half_period * period / 16000 - dead_time; // 7680 / frequency minus dead time 48%
    if (HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, HRTIM_COMPAREUNIT_1, &compareConfig) != HAL_OK)
    {
        Error_Handler();
    }
    // Set Compare Unit 2 of Timer B, set duty cycle to duty_cycle * period / 16000
    compareConfig.CompareValue = duty_cycle * period / 16000; // 3600 / period 36%

    if (HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, HRTIM_COMPAREUNIT_2, &compareConfig) != HAL_OK)
    {
        Error_Handler();
    }

	/* USER CODE BEGIN HRTIM1_Init 2 */
	// Store global time base configuration
	pGlobalTimeBaseCfg = timeBaseConfig;

    // Start four PWM outputs (TA1, TA2, TB1, TB2)
    HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2 | HRTIM_OUTPUT_TB1 | HRTIM_OUTPUT_TB2); // Enable all PWM outputs
    // Start Timer A and Timer B
    HAL_HRTIM_WaveformCounterStart(&hhrtim1, HRTIM_TIMERID_MASTER | HRTIM_TIMERID_TIMER_A | HRTIM_TIMERID_TIMER_B); // Start both PWM timers

	/* USER CODE END HRTIM1_Init 2 */
    HAL_HRTIM_MspPostInit(&hhrtim1);

}














