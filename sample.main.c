#include <p18f4620.h>
#include <delays.h>
#include <usart.h>
#include <stdio.h>

#include <rtc-philips-pcf8583.h>

// Set config bits on the PIC
#pragma config WDT = OFF
#pragma config OSC = HS
#pragma config FCMEN = ON
#pragma config IESO = ON
#pragma config BORV = 1
#pragma config BOREN = ON
#pragma config XINST = ON
#pragma config LVP = OFF

void main ( void ) {
    unsigned char i = 0;
    char buf[30];
    struct Timestamp tTimestamp;
    
    ADCON1 = 0x0F;          // Disable on chip A/D converter
    TRISB |= 0b00000001;	// Configure PORTB0 as input
    TRISC = 0b11011000;     // Configure pins for MSSP I2C operation
    SSPADD = 0b01100011;    // Configure the baud rate generator for 100kHz with 40MHz Fosc on PIC
    
    OpenUSART(				// Open the hardware USART for RS232 communication
        USART_TX_INT_OFF &
        USART_RX_INT_OFF &
        USART_ASYNCH_MODE &
        USART_EIGHT_BIT &
        USART_CONT_RX &
        USART_BRGH_HIGH,
        64
    );

	// Ensure RTC is not counting time
    rtcStopClock();
    // Set the time and date to 7/20/2010 8:00PM
    rtcSetHour(20);
    rtcSetMinute(0);
    rtcSetSecond(0);
    rtcSetHundrethsSeconds(0);
    rtcSetMonth(7);
    rtcSetDate(20);
    rtcSetYear(2010);
    // Set the alarm interval
    rtcSetAlarmInterval(10);
    // Enable the alarm
    rtcAlarmEnable(RTC_ALARM_MODE_TIMED,RTC_ALARM_INTERRUPT,RTC_ALARM_TIMER_UNIT_SECONDS);
    // Start the RTC
    rtcStartClock();

	// Loop forever
    while (1) {
        // Delay for a moment
        for ( i = 1; i <= 5; i++ ) {
            Delay10KTCYx(50);
        }
        // Retrieve the current timestamp from the RTC
        tTimestamp = rtcGetTimestamp(0);
        // Format the output into a buffer
        sprintf(buf,"%02d/%02d/%04u %02d:%02d:%02d\n\r\n\r", tTimestamp.Month, tTimestamp.Date, tTimestamp.Year, tTimestamp.Hour, tTimestamp.Minute, tTimestamp.Second);
        // Write the contents of the buffer to the USART
        putsUSART(buf);
        // Delay a moment longer
        for ( i = 1; i <= 5; i++ ) {
            Delay10KTCYx(50);
        }
        // Check to see if the RTC interrupt was thrown, interrupt is logic low on this chip
        if ( PORTBbits.RB0 == 1 ) {
            putrsUSART ((const far rom char *)"No interrupt\n\r\n\r");
        } else {
        	// If the interrupt was set, announce it, and reset the alarm
            putrsUSART ((const far rom char *)"Interrupt\n\r\n\r");
            rtcTimerAlarmReset();
        }
    }
}
