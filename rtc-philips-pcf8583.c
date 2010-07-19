/*******************************************************************
    File: rtc-philips-pcf8583.c
 Purpose: Enables higher level access to the Philips PCF8583 RTC,
          I2C bus communications are handled internally.
Requires: PIC Micro with MSSP port connected to an I2C bus with the
          Philips PCF8583 RTC device present.
  Author: Phil Ciebiera
 Revised: 7/19/2010
********************************************************************/          

// Include file for I2C interface
#include <i2c.h>

#include <usart.h>	// Comment out to save code space, only when commenting out RTC_DEBUG below
#define RTC_DEBUG 1	// Comment out to disable UART debugging

#define RTC_Address 						0b10100000
#define RTC_I2C_Read 						0x01
#define RTC_I2C_Write 						0x00
#define RTC_ADDR_CONFIG 					0x00
#define RTC_ADDR_ALARM_CONFIG 				0x08
#define RTC_ALARM_MODE_DATED 				0x01
#define RTC_ALARM_MODE_TIMED 				0x02
#define RTC_ALARM_INTERRUPT 				0x01
#define RTC_ALARM_INTERRUPT_NONE 			0x00

#define RTC_ALARM_TIMED_UNIT_1_100_SECONDS	0b00000001
#define RTC_ALARM_TIMER_UNIT_SECONDS 		0b00000010
#define RTC_ALARM_TIMED_UNIT_MINUTES		0b00000011
#define RTC_ALARM_TIMED_UNIT_HOURS			0b00000100
#define RTC_ALARM_TIMED_UNIT_DAYS			0b00000101

#define RTC_ADDR_ALARM_1_100_SECONDS		0x09
#define RTC_ADDR_ALARM_SECONDS				0x0A
#define RTC_ADDR_ALARM_MINUTES				0x0B
#define RTC_ADDR_ALARM_HOURS				0x0C
#define RTC_ADDR_ALARM_DATE					0x0D
#define RTC_ADDR_ALARM_MONTH				0x0E
#define RTC_ADDR_ALARM_TIMER				0x0F

#define RTC_ADDR_TIME_1_100_SECONDS			0x01
#define RTC_ADDR_TIME_SECONDS				0x02
#define RTC_ADDR_TIME_MINUTES				0x03
#define RTC_ADDR_TIME_HOURS					0x04
#define RTC_ADDR_TIME_YEAR_DATE				0x05
#define RTC_ADDR_TIME_WEEKDAYS_MONTH		0x06
#define RTC_ADDR_TIMER						0x07

#define RTC_OVERWRITE 						0x09
#define RTC_PAUSE_CLOCK 					0x01
#define RTC_DONT_PAUSE_CLOCK				0x00

unsigned char RTC_I2C_OPEN = 0;
unsigned char RTC_DEBUG_USART_OPEN = 0;
unsigned char RTC_STOPPED = 0;
unsigned int RTC_BASE_YEAR = 0;
unsigned int RTC_PREVIOUS_BASE_YEAR = 0;

struct Timestamp {
	unsigned char Month;
	unsigned char Date;
	unsigned int Year;
	unsigned char Hour;
	unsigned char Minute;
	unsigned char Second;
};

void rtcStopClock ( void );
void rtcStartClock ( void );

// Instruct compiler to include this function if RTC_DEBUG is on
#if defined ( RTC_DEBUG )
	// Ensure that the USART is open for debug outputting
	void rtcDebugOpenUSART ( void ) {
		if( RTC_DEBUG_USART_OPEN == 0 ) {
			OpenUSART(
				USART_TX_INT_OFF &
				USART_RX_INT_OFF &
				USART_ASYNCH_MODE &
				USART_EIGHT_BIT &
				USART_CONT_RX &
				USART_BRGH_HIGH,
				129);
			RTC_DEBUG_USART_OPEN = 1;
		}
	}
#endif

// Ensures that the PIC Micro has initialized its MSSP port in I2C Master Mode
void rtcCheckOpenI2C ( void ) {
	if( RTC_I2C_OPEN == 0 ) {
		OpenI2C(MASTER,SLEW_OFF);	// Open the I2C bus
		RTC_I2C_OPEN = 1;			// Set the I2C open flag so we don't open it again
	}
}

// Sets or clears a given bit in a value
unsigned char rtcBitSet ( unsigned char bIn, unsigned char bBit, unsigned char bValue ) {
	unsigned char t;
	if ( bValue == 1 ) {
		bIn |= 1 << bBit;
	} else {
		bIn &= ~(1 << bBit);
	}
	return bIn;
}

// Reads a byte from the RTC at bRegisterAddress offset
unsigned char rtcReadRegister ( unsigned char bRegisterAddress ) {
	unsigned char bIn;
	unsigned char bRtn;
	rtcCheckOpenI2C();					// Ensure the PIC MSSP is enabled for I2C Master
	IdleI2C();							// Ensure the I2C bus is idle before continuing
	StartI2C();							// Send I2C Start condition
	while ( SSPCON2bits.SEN );			// Wait until I2C Start condition is done
	bIn = RTC_Address | RTC_I2C_Write;	// Configure our address
	WriteI2C(bIn);						// Address RTC in write mode for assigning the register to be read
	WriteI2C(bRegisterAddress);			// Tell the RTC we want to read the sepcified register
	RestartI2C();						// Send an I2C Repeated Start condition
	while ( SSPCON2bits.RSEN );			// Wait until I2C Repeated Start condition is done
	bIn = RTC_Address | RTC_I2C_Read;	// set up our RTC address in read mode
	WriteI2C(bIn);						// address the RTC in read mode
	IdleI2C();							// Ensure I2C bus is Idle before reading
	bRtn = ReadI2C();					// Read the value from the I2C bus
	NotAckI2C();						// Tell the clock we have no idea what he's talking about
	while ( SSPCON2bits.ACKEN );		// Wait until the I2C NAK is done
	StopI2C();							// Stop the I2C bus
	while ( SSPCON2bits.PEN );			// Wait until the I2C Stop condition has been sent
	return ( bRtn );
}


// Adjusts the RTC register located at bRegisterAddress by setting the bit specified in bBit to bValue
// by setting bBit > 7 will allow you to overwrite the contents of the entire register with bValue
void rtcAdjustRegister ( unsigned char bRegisterAddress, unsigned char bBit, unsigned char bValue, unsigned char bPauseClock ) {
	unsigned char bWasStopped;
	unsigned char bIn;
	
	if ( bPauseClock == RTC_PAUSE_CLOCK ) {			// Check to see if we're to stop the clock for this register update
		bWasStopped = RTC_STOPPED;					// Cache the current status of the RTC clock running/stopped
		rtcStopClock();								// Stop the RTC from counting
	}
	if ( bBit <= 7 ) {								// receive the register data
		bIn = rtcReadRegister(bRegisterAddress);	// reads the specified register
		bValue = rtcBitSet(bIn,bBit,bValue);		// Tweak the returned value to set or clear the specified bit
	}
	rtcCheckOpenI2C();								// Ensure the PIC MSSP is enabled for I2C Master
	StartI2C();										// Send I2C Start condition
	while ( SSPCON2bits.SEN );						// Wait until I2C Start condition is done
	bIn = RTC_Address | RTC_I2C_Write;				// Set our address up as the RTC address in write mode
	WriteI2C(bIn);									// Address RTC in write mode for config register update
	IdleI2C();										// Ensure I2C bus is Idle before writing the next byte
	WriteI2C(bRegisterAddress);						// Inform the RTC of the register we'd like to update
	IdleI2C();										// Ensure I2C is idle
	WriteI2C(bValue);								// Write the new value to the I2C bus register
	IdleI2C();										// Ensure the I2C bus is idle
	StopI2C();										// Send I2C Stop condition
	while ( SSPCON2bits.PEN );						// Wait until the I2C Stop condition has been sent
	if ( bPauseClock == RTC_PAUSE_CLOCK ) {			// Check to see if we were supposed to pause the clock for this adjustment
		if ( bWasStopped == 0 ) rtcStartClock();	// If the clock was previously running, make sure it now continues to run
	}
}

// Stops the RTC from counting
void rtcStopClock ( void ) {
	rtcAdjustRegister(RTC_ADDR_CONFIG,7,1,RTC_DONT_PAUSE_CLOCK);
	RTC_STOPPED = 1;
}
// Starts the RTC counting
void rtcStartClock ( void ) {
	rtcAdjustRegister(RTC_ADDR_CONFIG,7,0,RTC_DONT_PAUSE_CLOCK);
	RTC_STOPPED = 0;
}


//###### INTERNAL FUNCTIONS USED TO ADJUST TIME REGISTERS IN THE RTC ######
// Computes and returns the BCD format for the RTC time registers
unsigned char rtcBCD ( unsigned char bInput ) {
	unsigned char t = 0;
	if ( bInput > 49 ) {
		t |= 0b01010000;
		bInput -= 50;
	} else if ( bInput > 39 ) {
		t |= 0b01000000;
		bInput -= 40;
	} else if ( bInput > 29 ) {
		t |= 0b00110000;
		bInput -= 30;
	} else if ( bInput > 19 ) {
		t |= 0b00100000;
		bInput -= 20;
	} else if ( bInput > 9 ) {
		t |= 0b00010000;
		bInput -= 10;
	}
	t |= bInput;
	return t;
}

//###### EXTERNAL FUNCTIONS USED TO READ THE CLOCK ######
// Support function for decoding BCD registers
unsigned char rtcBCDtoDEC ( unsigned char bInput ) {
	unsigned char bT;
	bT = bInput;
	bT &= ~(0b00001111);
	bT >>= 4;
	bT *= 10;
	bInput &= ~(0b11110000);
	bT += bInput;
	return bT;
}	
// Returns a specialized timestamp
struct Timestamp rtcGetTimestamp ( unsigned char bOffset ) {
	struct Timestamp tRTN;
	unsigned long lTimestamp = 0;
	unsigned char bMonth = 0;
	unsigned char bDate = 0;
	unsigned char bYear = 0;
	unsigned int iYear = 0;
	unsigned char bHour = 0;
	unsigned char bMinute = 0;
	unsigned char bSecond = 0;
	unsigned char bT = 0;
	unsigned long lT = 0;
	char buff[20];
	
	rtcCheckOpenI2C();	// Ensure the PIC MSSP is enabled for I2C Master
	IdleI2C();
	StartI2C();
	while ( SSPCON2bits.SEN ) ;
	bT = RTC_Address | RTC_I2C_Write;
	WriteI2C(bT);	// Address RTC in write mode for assigning the register to be read
	WriteI2C(RTC_ADDR_TIME_SECONDS + bOffset);				// Tell the RTC we want to read the sepcified register
	RestartI2C();
	while ( SSPCON2bits.RSEN ); 
	bT = RTC_Address | RTC_I2C_Read;
	WriteI2C(bT);	// address the RTC in read mode
	IdleI2C();
	bSecond = ReadI2C();	// Read the seconds register off the I2C
	AckI2C();
	while ( SSPCON2bits.ACKEN );
	bMinute = ReadI2C();
	AckI2C();
	while ( SSPCON2bits.ACKEN );
	bHour = ReadI2C();
	AckI2C();
	while ( SSPCON2bits.ACKEN );
	bDate = ReadI2C();
	AckI2C();
	while ( SSPCON2bits.ACKEN );
	bMonth = ReadI2C();
	NotAckI2C();
	while ( SSPCON2bits.ACKEN );
	bYear = bDate;
	StopI2C();
	while ( SSPCON2bits.PEN );

	bT = bYear;

	// Tweak the year from BCD to decimal
	bYear &= ~(0b00111111);
	bYear >>= 6;
	// Tweak the month from BCD to decimal
	bMonth &= ~(0b11100000);	// Mask off irrelevant data
	bMonth = rtcBCDtoDEC(bMonth);
	// Tweak the date from BCD to decimal
	bDate &= ~(0b11000000);
	bDate = rtcBCDtoDEC(bDate);
	
	bSecond = rtcBCDtoDEC(bSecond);
	bMinute = rtcBCDtoDEC(bMinute);
	bHour = rtcBCDtoDEC(bHour);
	// Handle for leap year rollover
	if ( bYear == 0 ) {
		// Every leap we have to increment the RTC_BASE_YEAR by 4
		if ( RTC_BASE_YEAR == RTC_PREVIOUS_BASE_YEAR ) {
			RTC_BASE_YEAR += 4;
			bYear = 0;
			bT &= ~(0x11000000);
			rtcAdjustRegister(RTC_ADDR_TIME_YEAR_DATE,RTC_OVERWRITE,bT,RTC_DONT_PAUSE_CLOCK);
		}
	} else {	// Once leap year has passed
		if ( RTC_BASE_YEAR != RTC_PREVIOUS_BASE_YEAR ) {
			RTC_PREVIOUS_BASE_YEAR = RTC_BASE_YEAR;	// Ensure our previous base year is equal to our base year so the next leap year rollover can occur
		}
	}
	tRTN.Month = bMonth;
	tRTN.Date = bDate;
	tRTN.Year = (bYear + RTC_BASE_YEAR);
	tRTN.Hour = bHour;
	tRTN.Minute = bMinute;
	tRTN.Second = bSecond;
	return tRTN;
}	
	
//###### EXTERNAL FUNCTIONS USED TO SET THE CLOCK AND ALARM OF THE RTC ######
// Sets the RTC year/date register for clock year
void rtcSetYear ( unsigned int iYear ) {
	unsigned char bPrevious;
	unsigned char bWasStopped;
	unsigned char bIsLeap = 0;

	//****** LEAP YEAR SUPPORT ******
	if ( iYear % 4 == 0 ) {						// Leap year occurs every four years
		if ( iYear % 100 == 0 ) {				// this bit of code is needed to ensure
			if ( iYear % 400 == 0 ) {			// proper leap year support. Basically
				bIsLeap = 1;					// the RTC only counts for up to four years
			}									// this means we need to have the actual
		} else {								// year set in an external variable, to which
			bIsLeap = 1;						// we add the value stored in the RTC clock
		}										// this becomes problematic when the RTC
	}											// rolls over from 3 back to 0. To account
	if ( bIsLeap == 1 ) {						// for this, we check to see if this is a
		RTC_BASE_YEAR = iYear;					// leap year, and if not, set the RTC leap
		iYear = 0;								// year cycle according to how long it is
	} else {									// until the next leap year. This however
		RTC_BASE_YEAR = iYear - (iYear % 4);	// introduces another problem in that, once
		iYear = iYear % 4;						// the rollover is about to occur, we have to
	}											// compensate with some other functions.

	bWasStopped = RTC_STOPPED;																	// Determine if the RTC clock was previously running
	rtcStopClock();																				// Stop the RTC clock
	bPrevious = rtcReadRegister(RTC_ADDR_TIME_YEAR_DATE);										// Read the contents of the year/date register
	bPrevious &= ~(0b11000000);
	bPrevious |= (((char)iYear) << 6);															// Clear the year setting, we add base year when outputting
	rtcAdjustRegister(RTC_ADDR_TIME_YEAR_DATE,RTC_OVERWRITE,bPrevious,RTC_DONT_PAUSE_CLOCK);	// Write the modified register value back to the year/date register
	if ( bWasStopped == 0 ) rtcStartClock();													// Start the RTC clock again if it was previously running
}
// Sets the RTC year/date register for clock date
void rtcSetDate ( unsigned char bDate ) {	
	unsigned char bPrevious;
	unsigned char bWasStopped;
	bWasStopped = RTC_STOPPED;																	// Determine if the RTC clock was previously running
	rtcStopClock();																				// Stop the RTC clock
	bPrevious = rtcReadRegister(RTC_ADDR_TIME_YEAR_DATE);										// Read the contents of the year/date register
	bPrevious &= ~(0b00111111);
	bPrevious |= rtcBCD(bDate);																	// Clear and then modify only the date bits
	rtcAdjustRegister(RTC_ADDR_TIME_YEAR_DATE,RTC_OVERWRITE,bPrevious,RTC_DONT_PAUSE_CLOCK);	// Write the modified register value back to the year/date register
	if ( bWasStopped == 0 ) rtcStartClock();													// Start the RTC clock again if it was previously running
}
// Sets the RTC weekdays/month register for clock weekday
void rtcSetDay ( unsigned char bDay ) {	
	unsigned char bPrevious;
	unsigned char bWasStopped;
	bWasStopped = RTC_STOPPED;																		// Determine if the RTC clock was previously running
	rtcStopClock();																					// Stop the RTC clock
	bPrevious = rtcReadRegister(RTC_ADDR_TIME_WEEKDAYS_MONTH);										// Read the contents of the weekdays/month register
	bPrevious &= ~(0b11100000); 
	bPrevious |= (bDay >> 3);																		// Clear and then modify only the weekday bits
	rtcAdjustRegister(RTC_ADDR_TIME_WEEKDAYS_MONTH,RTC_OVERWRITE,bPrevious,RTC_DONT_PAUSE_CLOCK);	// Write the modified register value back to the weekdays/month register
	if ( bWasStopped == 0 ) rtcStartClock();														// Start the RTC clock again if it was previously running
}	
// Sets the RTC weekdays/month register for clock month
void rtcSetMonth ( unsigned char bMonth ) {	
	unsigned char bPrevious;
	unsigned char bWasStopped;
	bWasStopped = RTC_STOPPED;																		// Determine if the RTC clock was previously running
	rtcStopClock();																					// Stop the RTC clock
	bPrevious = rtcReadRegister(RTC_ADDR_TIME_WEEKDAYS_MONTH);										// Read the contents of the weekdays/month register
	bPrevious &= ~(0b00011111);
	bPrevious |= rtcBCD(bMonth);																	// Clear and then modify only the month bits
	rtcAdjustRegister(RTC_ADDR_TIME_WEEKDAYS_MONTH,RTC_OVERWRITE,bPrevious,RTC_DONT_PAUSE_CLOCK);	// Write the modified register value back to the weekdays/month register
	if ( bWasStopped == 0 ) rtcStartClock();														// Start the RTC clock again if it was previously running
}
// Sets the RTC clock hour to bHour
void rtcSetHour ( unsigned char bHour ) {
	rtcAdjustRegister(RTC_ADDR_TIME_HOURS,RTC_OVERWRITE,rtcBCD(bHour),RTC_PAUSE_CLOCK);
}
// Sets the RTC clock minute
void rtcSetMinute ( unsigned char bMinute ) {
	rtcAdjustRegister(RTC_ADDR_TIME_MINUTES,RTC_OVERWRITE,rtcBCD(bMinute),RTC_PAUSE_CLOCK);
}
// Sets the RTC clock second
void rtcSetSecond ( unsigned char bSecond ) {
	rtcAdjustRegister(RTC_ADDR_TIME_SECONDS,RTC_OVERWRITE,rtcBCD(bSecond),RTC_PAUSE_CLOCK);
}
// Sets the RTC clock hundreths of a second
void rtcSetHundrethsSeconds ( unsigned char bHundreths ) {
	rtcAdjustRegister(RTC_ADDR_TIME_1_100_SECONDS,RTC_OVERWRITE,rtcBCD(bHundreths),RTC_PAUSE_CLOCK);
}
// Sets the RTC alarm hundreths of a second
void rtcSetAlarmHundrethsSeconds ( unsigned char bHundreths ) {
	rtcAdjustRegister(RTC_ADDR_ALARM_1_100_SECONDS,RTC_OVERWRITE,rtcBCD(bHundreths),RTC_DONT_PAUSE_CLOCK);
}	
// Sets the RTC alarm second
void rtcSetAlarmSecond ( unsigned char bSecond ) {
	rtcAdjustRegister(RTC_ADDR_ALARM_SECONDS,RTC_OVERWRITE,rtcBCD(bSecond),RTC_DONT_PAUSE_CLOCK);
}
// Sets the RTC alarm minute
void rtcSetAlarmMinute ( unsigned char bMinute ) {
	rtcAdjustRegister(RTC_ADDR_ALARM_MINUTES,RTC_OVERWRITE,rtcBCD(bMinute),RTC_DONT_PAUSE_CLOCK);
}
// Sets the RTC alarm hour
void rtcSetAlarmHour ( unsigned char bHour ) {
	rtcAdjustRegister(RTC_ADDR_ALARM_HOURS,RTC_OVERWRITE,rtcBCD(bHour),RTC_DONT_PAUSE_CLOCK);
}
// Sets the RTC alarm Date 1 - 31
void rtcSetAlarmDate ( unsigned char bDate ) {
	unsigned char bPrevious;
	bPrevious = rtcReadRegister(RTC_ADDR_ALARM_DATE);										// Read the contents of the year/date register
	bPrevious &= ~(0b00111111);
	bPrevious |= rtcBCD(bDate);																	// Clear and then modify only the date bits
	rtcAdjustRegister(RTC_ADDR_ALARM_DATE,RTC_OVERWRITE,bPrevious,RTC_DONT_PAUSE_CLOCK);	// Write the modified register value back to the year/date register
}
// Sets the RTC alarm Month 1 - 12
void rtcSetAlarmMonth ( unsigned char bMonth ) {
	//rtcAdjustRegister(RTC_ADDR_ALARM_MONTH,RTC_OVERWRITE,bMonth,RTC_PAUSE_CLOCK);	// Write the new register value to the alarm month register
		unsigned char bPrevious;																	// Stop the RTC clock
	bPrevious = rtcReadRegister(RTC_ADDR_ALARM_MONTH);										// Read the contents of the weekdays/month register
	bPrevious &= ~(0b00011111);
	bPrevious |= rtcBCD(bMonth);																	// Clear and then modify only the month bits
	rtcAdjustRegister(RTC_ADDR_ALARM_MONTH,RTC_OVERWRITE,bPrevious,RTC_DONT_PAUSE_CLOCK);	// Write the modified register value back to the weekdays/month register
}

//###### FUNCTIONS FOR CONTROLLING THE ALARMS ######
// Enables the RTC Alarm in the mode specified by the bTimerType (Dated Alarm, or Timed Alarm), bInterrupt (On or Off), and bTimerUnit (hundreths, seconds, minutes, etc)
void rtcAlarmEnable ( unsigned char bAlarmType, unsigned char bInterrupt, unsigned char bTimerUnit ) {
	unsigned char bConfig;
	if ( bAlarmType == RTC_ALARM_MODE_DATED ) {
		bConfig = 0b00110000;
		bConfig |= (bInterrupt << 7);					// Configure and enable alarm with specified interrupt state
	} else if ( bAlarmType == RTC_ALARM_MODE_TIMED ) {
		bConfig = bTimerUnit;							// Set the timer unit for a timed alarm
		bConfig |= (0b01000000 | (bInterrupt << 3));	// Configure and enable alarm with specified interrupt state
	}
	rtcAdjustRegister(RTC_ADDR_CONFIG,2,1,RTC_DONT_PAUSE_CLOCK);							// Enable alarm
	rtcAdjustRegister(RTC_ADDR_ALARM_CONFIG,RTC_OVERWRITE,bConfig,RTC_DONT_PAUSE_CLOCK);	// Write the contents of the alarm config register
}
// Enables the RTC Alarm
void rtcAlarmDisable ( void ) {
	rtcAdjustRegister(RTC_ADDR_CONFIG,2,0,RTC_DONT_PAUSE_CLOCK);
}
// Resets the RTC timer and alarm interrupt
void rtcAlarmReset ( void ) {
	unsigned char bPrevious;
	bPrevious = rtcReadRegister(RTC_ADDR_CONFIG);										// Read the contents of the config register
	bPrevious &= ~(0b00000011);															// Clear both interrupts while preserving original configured bits
	rtcAdjustRegister(RTC_ADDR_CONFIG,RTC_OVERWRITE,bPrevious,RTC_DONT_PAUSE_CLOCK);	// Write the modified register value back to the RTC config register
}
// Resets the RTC timer and alarm interrupts and resets the alarm timer counter
void rtcTimerAlarmReset ( void ) {
	unsigned char bInterval;
	bInterval = rtcReadRegister(RTC_ADDR_TIMER);									// Read the interval setting of the alarm
	bInterval = 99 - bInterval;														// Reset the timer to the specified interval
	rtcAdjustRegister(RTC_ADDR_TIMER,RTC_OVERWRITE,bInterval,RTC_DONT_PAUSE_CLOCK);	// Write the new value to the timer address
	rtcAlarmReset();																// Reset the timer and alarm interrupts
}
// Sets the RTC alarm timer interval [1 - 99]
void rtcSetAlarmInterval ( unsigned char bInterval ) {
	rtcAdjustRegister(RTC_ADDR_ALARM_TIMER,RTC_OVERWRITE,bInterval,RTC_DONT_PAUSE_CLOCK);
	bInterval = 99 - bInterval;														// Set the timer to the specified interval
	rtcAdjustRegister(RTC_ADDR_TIMER,RTC_OVERWRITE,bInterval,RTC_DONT_PAUSE_CLOCK);	// Write the new value to the timer address
}
