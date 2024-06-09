
//*****************************************************************************
//
// Application Name     -   Send LED strip State To AWS
// Created by           -   Mitul Mathur,
//                      -   Austin Bunuan
//
// Base code sources    - Lab 2 Submission for Austin and Mitul (Accelerometer input for controlling grey square)
//                      - Lab 3 Prelab, Question 6 (used for setting up remote interrupts)
//                      - Lab 3 Button Example from Canvas (used to setup button interrupts)
//                      - Lab 4 Blank from Canvas  (used to setup socket connection and Get and Post requests)
//
//
//*****************************************************************************

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>


// Simplelink includes
#include "simplelink.h"

//Driverlib includes
#include "hw_types.h"
#include "hw_ints.h"
#include "hw_nvic.h"
#include "hw_memmap.h"
#include "hw_common_reg.h"
#include "interrupt.h"
#include "hw_apps_rcm.h"
#include "prcm.h"
#include "rom.h"
#include "prcm.h"
#include "utils.h"
#include "systick.h"
#include "rom_map.h"
#include "gpio.h"
#include "spi.h"
#include "uart.h"


//Common interface includes
#include "pin_mux_config.h"
#include "gpio_if.h"
#include "i2c_if.h"
#include "common.h"
#include "uart_if.h"

// Custom includes
#include "utils/network_utils.h"

#include "Adafruit_SSD1351.h"
#include "Adafruit_GFX.h"


//NEED TO UPDATE THIS FOR IT TO WORK!
#define DATE                22    /* Current Date */
#define MONTH               5     /* Month 1-12 */
#define YEAR                2024  /* Current year */
#define HOUR                11    /* Time - hours */
#define MINUTE              25    /* Time - minutes */
#define SECOND              0     /* Time - seconds */


#define APPLICATION_NAME      "SSL"
#define APPLICATION_VERSION   "SQ24"
#define SERVER_NAME           "a22o8qmuv0ftp5-ats.iot.us-east-1.amazonaws.com" // CHANGE ME
#define GOOGLE_DST_PORT       8443


#define POSTHEADER "POST /things/AustinMitul_CC3200Board/shadow HTTP/1.1 \r\n"             // CHANGE ME
#define GETHEADER  "GET /things/AustinMitul_CC3200Board/shadow HTTP/1.1 \r\n"
#define HOSTHEADER "Host: a22o8qmuv0ftp5-ats.iot.us-east-1.amazonaws.com\r\n"  // CHANGE ME
#define CHEADER "Connection: Keep-Alive\r\n"
#define CTHEADER "Content-Type: application/json; charset=utf-8\r\n"
#define CLHEADER1 "Content-Length: "
#define CLHEADER2 "\r\n\r\n"
#define OPENING_DATA "{" \
        "\"state\": {\r\n"                                          \
            "\"desired\": {\r\n"                                    \
                "\"color\": \""
#define CLOSING_DATA "\"\r\n"                                       \
                "}"                                                 \
            "}"                                                     \
        "}\r\n\r\n"

// some helpful macros for systick

// the cc3200's fixed clock frequency of 80 MHz
// note the use of ULL to indicate an unsigned long long constant
#define SYSCLKFREQ 80000000ULL

// macro to convert ticks to microseconds
#define TICKS_TO_US(ticks) \
    ((((ticks) / SYSCLKFREQ) * 1000000ULL) + \
    ((((ticks) % SYSCLKFREQ) * 1000000ULL) / SYSCLKFREQ))\

// macro to convert microseconds to ticks
#define US_TO_TICKS(us) ((SYSCLKFREQ / 1000000ULL) * (us))

// systick reload value set to 40ms period
// (PERIOD_SEC) * (SYSCLKFREQ) = PERIOD_TICKS
#define SYSTICK_RELOAD_VAL 3200000UL

// track systick counter periods elapsed
// if it is not 0, we know the transmission ended
volatile int systick_cnt = 0;

#define IR_GPIO_PORT        GPIOA0_BASE
#define IR_GPIO_PIN         0x1
#define CAPTURED_BITS_SIZE  33

// Systick Pulse Measurements
#define IS_0            1125
#define IS_1            2250
#define IS_START        15000

// Decoded Buttons
#define BUTTON_0        50135295
#define BUTTON_1        50167935
#define BUTTON_2        50151615
#define BUTTON_3        50184255
#define BUTTON_4        50143455
#define BUTTON_5        50176095
#define BUTTON_6        50159775
#define BUTTON_7        50192415
#define BUTTON_8        50139375
#define BUTTON_9        50172015
#define BUTTON_MUTE     50137335
#define BUTTON_LAST     50135805
#define BUTTON_PRESS_DELAY    15

// SPI Values
#define SPI_IF_BIT_RATE  100000
#define TR_BUFF_SIZE     100
#define DATA_BUFF_SIZE   150

// OLED Values
#define SSD1351WIDTH    128
#define SSD1351HEIGHT   128
#define CHARHEIGHT      8
#define CHARWIDTH       6
#define BLACK           0x0000
#define BLUE            0x001F
#define RED             0xF800
#define GREEN           0x07E0
#define INDIGO          0x07FF
#define MAGENTA         0xF81F
#define YELLOW          0xFFE0
#define WHITE           0xFFFF
#define GREY            0x7BEF

// Remote Values
#define OLED_START_SEND_Y_INX  100
#define OLED_END_SEND_Y_INX    127
#define OLED_LAST_CHAR_ON_LINE 120

// Accelerometer Values
#define UP_OR_LEFT_LOWER_BOUND  192
#define UP_OR_LEFT_UPPER_BOUND  256
#define DOWN_OR_RIGHT_UPPER_BOUND  64
#define SQUARE_HEIGHT            4
#define SQUARE_WIDTH             4
#define RECTANGLE_BOUNDARY       80
#define RECTANGLE_WIDTH    32
#define RECTANGLE_HEIGHT   40
#define SQUAREHEIGHT 4
#define SQUAREWIDTH 4
#define ACCEL_X_LOG         0
#define ACCEL_Y_LOG         102

extern void (* const g_pfnVectors[])(void);
//*****************************************************************************
//                 GLOBAL VARIABLES -- Start
//*****************************************************************************

#if defined(ccs) || defined(gcc)
extern void (* const g_pfnVectors[])(void);
#endif
#if defined(ewarm)
extern uVectorEntry __vector_table;
#endif

#define RET_IF_ERR(Func)          {int iRetVal = (Func); \
                                   if (SUCCESS != iRetVal) \
                                     return  iRetVal;}

//*****************************************************************************
//                 GLOBAL VARIABLES -- End: df
//*****************************************************************************

volatile int systick_expired = 0;
volatile int buffer_count = 0;
volatile int received_iterator = 0;
volatile uint64_t captured_bits[CAPTURED_BITS_SIZE];
volatile char received_buffer[DATA_BUFF_SIZE];
volatile int old_systic_count = 0;
volatile unsigned long SW2_intcount;
volatile unsigned long SW3_intcount;
volatile unsigned int SW2_intflag;
volatile unsigned int SW3_intflag;
char data_to_send[DATA_BUFF_SIZE];                                          // buffer for data to send over UART1
int time_set = 0;
const unsigned int colorArray[] = {WHITE, RED, BLUE, GREEN, INDIGO, MAGENTA, YELLOW, BLACK};
char colorStrings[8][8] = {"WHITE", "RED", "BLUE", "GREEN", "INDIGO", "MAGENTA", "YELLOW", "BLACK"};
//****************************************************************************
//                      LOCAL FUNCTION PROTOTYPES
//****************************************************************************
static int set_time();
static void BoardInit(void);
static int http_post(int);
static int http_get(int);

//*****************************************************************************
//
//! Board Initialization & Configuration
//!
//! \param  None
//!
//! \return None
//
//*****************************************************************************
static void BoardInit(void) {
/* In case of TI-RTOS vector table is initialize by OS itself */
#ifndef USE_TIRTOS
  //
  // Set vector table base
  //
#if defined(ccs)
    MAP_IntVTableBaseSet((unsigned long)&g_pfnVectors[0]);
#endif
#if defined(ewarm)
    MAP_IntVTableBaseSet((unsigned long)&__vector_table);
#endif
#endif
    //
    // Enable Processor
    //
    MAP_IntMasterEnable();
    MAP_IntEnable(FAULT_SYSTICK);

    PRCMCC3200MCUInit();
}




//*****************************************************************************
//
//! This function updates the date and time of CC3200.
//!
//! \param None
//!
//! \return
//!     0 for success, negative otherwise
//!
//*****************************************************************************

static int set_time() {
    long retVal;

    g_time.tm_day = DATE;
    g_time.tm_mon = MONTH;
    g_time.tm_year = YEAR;
    g_time.tm_sec = HOUR;
    g_time.tm_hour = MINUTE;
    g_time.tm_min = SECOND;

    retVal = sl_DevSet(SL_DEVICE_GENERAL_CONFIGURATION,
                          SL_DEVICE_GENERAL_CONFIGURATION_DATE_TIME,
                          sizeof(SlDateTime),(unsigned char *)(&g_time));

    ASSERT_ON_ERROR(retVal);
    return SUCCESS;
}

//*****************************************************************************
//
//! Main 
//!
//! \param  none
//!
//! \return None
//!
//*****************************************************************************
void PostAndGet() {
    long lRetVal = -1;

    UART_PRINT("My terminal works!\n\r");

    // initialize global default app configuration
    g_app_config.host = SERVER_NAME;
    g_app_config.port = GOOGLE_DST_PORT;

    //Connect the CC3200 to the local access point
    lRetVal = connectToAccessPoint();
    //Set time so that encryption can be used
    lRetVal = set_time();
    if(lRetVal < 0 && time_set == 0) {
        UART_PRINT("Unable to set time in the device");
        LOOP_FOREVER();
    }
    time_set = 1;
    //Connect to the website with TLS encryption
    lRetVal = tls_connect();
    if(lRetVal < 0) {
        ERR_PRINT(lRetVal);
    }
    http_post(lRetVal);
    http_get(lRetVal);

    sl_Stop(SL_STOP_TIMEOUT);
}
//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************
static int http_get(int iTLSSockID) {
    char acSendBuff[512];
    char acRecvbuff[1460];
    char* pcBufHeaders;
    int lRetVal = 0;

    pcBufHeaders = acSendBuff;              // Copy headers to a string, increment the pointer to the next address to
    strcpy(pcBufHeaders, GETHEADER);        // prepare to append next string
    pcBufHeaders += strlen(GETHEADER);
    strcpy(pcBufHeaders, HOSTHEADER);
    pcBufHeaders += strlen(HOSTHEADER);
    strcpy(pcBufHeaders, CHEADER);
    pcBufHeaders += strlen(CHEADER);
    strcpy(pcBufHeaders, "\r\n\r\n");

    int testDataLength = strlen(pcBufHeaders);

    UART_PRINT(acSendBuff);

    //
    // Send the packet to the server */
    //
    lRetVal = sl_Send(iTLSSockID, acSendBuff, strlen(acSendBuff), 0);       // Use the socket send data through Socket to AWS
    if(lRetVal < 0) {
       UART_PRINT("GET failed. Error Number: %i\n\r",lRetVal);
       sl_Close(iTLSSockID);
       GPIO_IF_LedOn(MCU_RED_LED_GPIO);
       return lRetVal;
    }
    lRetVal = sl_Recv(iTLSSockID, &acRecvbuff[0], sizeof(acRecvbuff), 0);   // Open a new socket, receive data through Socket from AWS
    if(lRetVal < 0) {
       UART_PRINT("Received failed. Error Number: %i\n\r",lRetVal);
       //sl_Close(iSSLSockID);
       GPIO_IF_LedOn(MCU_RED_LED_GPIO);
          return lRetVal;
    }
    else {
       acRecvbuff[lRetVal+1] = '\0';                                        // print what was received
       UART_PRINT(acRecvbuff);
       UART_PRINT("\n\r\n\r");
    }

    return 0;
}


static int http_post(int iTLSSockID){
    char acSendBuff[512];
    char acRecvbuff[1460];
    char cCLLength[200];
    char* pcBufHeaders;
    int lRetVal = 0;

    pcBufHeaders = acSendBuff;                   // Copy headers to a string, increment the pointer to the next address to
    strcpy(pcBufHeaders, POSTHEADER);            // prepare to append next string
    pcBufHeaders += strlen(POSTHEADER);
    strcpy(pcBufHeaders, HOSTHEADER);
    pcBufHeaders += strlen(HOSTHEADER);
    strcpy(pcBufHeaders, CHEADER);
    pcBufHeaders += strlen(CHEADER);
    strcpy(pcBufHeaders, "\r\n\r\n");

    int dataLength = strlen(data_to_send) + strlen(OPENING_DATA) + strlen(CLOSING_DATA);

    strcpy(pcBufHeaders, CTHEADER);                 // Content type
    pcBufHeaders += strlen(CTHEADER);
    strcpy(pcBufHeaders, CLHEADER1);

    pcBufHeaders += strlen(CLHEADER1);              // Content length
    sprintf(cCLLength, "%d", dataLength);

    strcpy(pcBufHeaders, cCLLength);
    pcBufHeaders += strlen(cCLLength);
    strcpy(pcBufHeaders, CLHEADER2);
    pcBufHeaders += strlen(CLHEADER2);

    strcpy(pcBufHeaders, OPENING_DATA);               // Format JSON before appending data from OLED
    pcBufHeaders += strlen(OPENING_DATA);
    strcpy(pcBufHeaders, data_to_send);
    pcBufHeaders += strlen(data_to_send);
    strcpy(pcBufHeaders, CLOSING_DATA);               // Close JSON format
    pcBufHeaders += strlen(CLOSING_DATA);

    int testDataLength = strlen(pcBufHeaders);

    UART_PRINT(acSendBuff);


    //
    // Send the packet to the server */
    //
    lRetVal = sl_Send(iTLSSockID, acSendBuff, strlen(acSendBuff), 0);
    if(lRetVal < 0) {
        UART_PRINT("POST failed. Error Number: %i\n\r",lRetVal);
        sl_Close(iTLSSockID);
        GPIO_IF_LedOn(MCU_RED_LED_GPIO);
        return lRetVal;
    }
    lRetVal = sl_Recv(iTLSSockID, &acRecvbuff[0], sizeof(acRecvbuff), 0);
    if(lRetVal < 0) {
        UART_PRINT("Received failed. Error Number: %i\n\r",lRetVal);
        //sl_Close(iSSLSockID);
        GPIO_IF_LedOn(MCU_RED_LED_GPIO);
           return lRetVal;
    }
    else {
        acRecvbuff[lRetVal+1] = '\0';
        UART_PRINT(acRecvbuff);
        UART_PRINT("\n\r\n\r");
    }

    return 0;
}


/*
 * Array to keep track of what button corresponds to what letter
 * Rows correspond to different buttons
 * Columns represent different outputs depending on how many times button is pressed
*/
static const char character_map[10][4] = {
            {'\0', '\0', '\0', '\0'},   // Letters for 0
            {'\0', '\0', '\0', '\0'},   // Letters for 1
            {'A', 'B', 'C', '\0'},      // Letters for 2
            {'D', 'E', 'F', '\0'},      // Letters for 3
            {'G', 'H', 'I', '\0'},      // Letters for 4
            {'J', 'K', 'L', '\0'},      // Letters for 5
            {'M', 'N', 'O', '\0'},      // Letters for 6
            {'P', 'Q', 'R', 'S'},       // Letters for 7
            {'T', 'U', 'V', '\0'},      // Letters for 8
            {'W', 'X', 'Y', 'Z'},       // Letters for 9
};
/**
 * Reset SysTick Counter
 */
static inline void SysTickReset(void) {
    HWREG(NVIC_ST_CURRENT) = 1;

    // Reset systick count and set systick to be not expired
    systick_cnt = 0;
    systick_expired = 0;
}

typedef struct PinSetting { // Struct for convenience of accessing Buttons
    unsigned long port;
    unsigned int pin;
} PinSetting;

static const PinSetting switch2 = { .port = GPIOA2_BASE, .pin = 0x40};
static const PinSetting switch3 = { .port = GPIOA1_BASE, .pin = 0x20};
//*****************************************************************************
//
//! Board Initialization & Configuration
//!
//! \param  None
//!
//! \return None
//
//*****************************************************************************


static void GPIOA0IntHandler(void) {
    uint64_t ulStatus =  MAP_GPIOIntStatus(IR_GPIO_PORT, true);
    MAP_GPIOIntClear(IR_GPIO_PORT, ulStatus);
    if(ulStatus & IR_GPIO_PIN) {                            // check if interrupt happened on IR pin
        if(!systick_expired) {                              // check if pulse was longer than 40ms
            captured_bits[buffer_count] =
                    TICKS_TO_US(SYSTICK_RELOAD_VAL - MAP_SysTickValueGet());  // get elapsed time from last falling edge
            buffer_count += 1;

        } else {
            old_systic_count = systick_cnt;                 // get systick count before reseting,
                                                            // used later for input delay if same button is pressed
            buffer_count = 0;                               // reset buffer count
        }
        SysTickReset();
    }
}

static void GPIOA1IntHandler(void) { // SW3 handler
    unsigned long ulStatus;

    ulStatus = MAP_GPIOIntStatus (GPIOA1_BASE, true);
    MAP_GPIOIntClear(GPIOA1_BASE, ulStatus);        // clear interrupts on GPIOA1
    SW3_intcount++;
    SW3_intflag=1;
//    Report("SW2 ints = %d\tSW3 ints = %d\r\n",SW2_intcount,SW3_intcount);
}



static void GPIOA2IntHandler(void) {    // SW2 handler
    unsigned long ulStatus;

    ulStatus = MAP_GPIOIntStatus (switch2.port, true);
    MAP_GPIOIntClear(switch2.port, ulStatus);       // clear interrupts on GPIOA2
    SW2_intcount++;
    SW2_intflag=1;
//    Report("SW2 ints = %d\tSW3 ints = %d\r\n",SW2_intcount,SW3_intcount);
}

static void SysTickHandler(void) {
    systick_expired = 1;                                            // systick expired
    systick_cnt++;                                                  // increment systick count
}

// Print a string to OLED
void PrintMessage(int x, int y, const char *message, unsigned int color) {
    int messageSize = strlen(message);
    int i;

    for(i = 0; i < messageSize; i++) {
        drawChar(x, y, message[i], color, 0, 1);                    // draw Character
        if((x + CHARWIDTH) > SSD1351WIDTH) {                        // bounds check and iterate for next character
          x = 0;
          y += CHARHEIGHT;
        }
        else {
          x += CHARWIDTH;
        }
    }
    y += CHARHEIGHT;
    drawFastHLine(0, y, SSD1351WIDTH, WHITE);                       // White line to cutoff message
}

// Clear a message given starting and stopping y-coordinates
void ClearMessage(int start_y, int stop_y) {
    int i;
    for(i = start_y; i < stop_y; i++) {
        drawFastHLine(0, i, SSD1351WIDTH, BLACK);
    }
}
// Decode what button was pressed, return row corresponding to character map
int GetButton(long decoded) {
    int row_iterator = -1;                  // signifies button not recognized

    if (decoded == BUTTON_2) {
        row_iterator = 2;
    }
    else if (decoded == BUTTON_3) {
        row_iterator = 3;
    }
    else if (decoded == BUTTON_4) {
        row_iterator = 4;
    }
    else if (decoded == BUTTON_5) {
        row_iterator = 5;
    }
    else if (decoded == BUTTON_6) {
        row_iterator = 6;
    }
    else if (decoded == BUTTON_7) {
        row_iterator = 7;
    }
    else if (decoded == BUTTON_8) {
        row_iterator = 8;
    }
    else if (decoded == BUTTON_9) {
        row_iterator = 9;
    }
    return row_iterator;

}
void SetupRemoteControl() {
    const char infoMessage[] = "Available Colors";

    fillScreen(BLACK);
    PrintMessage(0, 0, infoMessage, WHITE);
    int i, j;
    int x = 0, y = 17;
    unsigned int text_size = 1;

    int colorStringsSize = sizeof(colorStrings) / sizeof(colorStrings[0]);  // Size of one string in the 2D array

    for(i = 0; i < colorStringsSize; i++) {
        char *current_color = colorStrings[i];                              // Current color in 2D array
        int len_current_color = strlen(current_color);                      // Length of string for current color

        // Draw character for current color string
        for(j = 0; j < len_current_color; j++) {
            if(strcmp(current_color, "BLACK") == 0) {                       // Can't write Black String on black background
                drawChar(x, y, current_color[j], BLACK, WHITE, text_size);
            }
            else {
                drawChar(x, y, current_color[j], colorArray[i], BLACK, text_size);
            }
            x += CHARWIDTH;
        }

        while((x % (SSD1351WIDTH/2)) != 0) {                                // Iterate until middle of screen or end
            x += 2;
        }

        if((x % SSD1351WIDTH) == 0) {                                       // Space out colors if at end of line
            x = 0;
            y += CHARHEIGHT * 2;
        }
    }
    x = 0;
    drawFastHLine(x, y, SSD1351WIDTH, WHITE);                               // Line to separate list of colors with next message
    y += 1;

    PrintMessage(x, y, "Press \"MUTE\" to send, press \"1\" to go back", WHITE);
    y += 17;

}

// Check if Remote input is a valid color
int CheckIfValidString(char *string) {
    int i;
    int colorStringsSize = sizeof(colorStrings) / sizeof(colorStrings[0]);

    for(i = 0; i < colorStringsSize; i++) {
        char *current_color = colorStrings[i];

        if(strcmp(string, current_color) == 0) {
            return 0;
        }
    }
    return 1;
}
//****************************************************************************
//
//! Main function
//!
//! \param none
//!
//!
//! \return None.
//
//****************************************************************************
void RemoteControl() {

    int row_iterator = 0;                                                       // used for character map
    int column_iterator = 0;                                                    // used for character map
    int data_to_send_index = 0;                                                 // count for data to send to AWS
    long prev_decoded = -1;                                                     // keep track of what was button was last used
    int x = 0;                                                                  // coordinates for OLED to draw characters
    int y = OLED_START_SEND_Y_INX;                                              // start at lower half of screen for OLED
    int go_back = 0;

    // Clear any old data
    memset(data_to_send, '\0', sizeof(data_to_send));


    SetupRemoteControl();
    buffer_count = 0;

    while(go_back == 0) {
        uint64_t tmp_delta_us[CAPTURED_BITS_SIZE];      // temporary variable to get values from buffer
        long decoded;                                   // used later to decode binary string
        char binary_encoding[CAPTURED_BITS_SIZE - 1];   // decode wavelengths to binary string

        // If buffer for capturing bits is full, get what button was pushed
        if (buffer_count == CAPTURED_BITS_SIZE) {
            buffer_count = 0;                           // reset to avoid re-entering if block
            int i;

            for(i = 0; i < CAPTURED_BITS_SIZE; i++) {
                tmp_delta_us[i] = captured_bits[i];     // copy data over from buffer to local variable
            }

            int binary_encoding_iterator = 0;           // iterator used for binary encoding, separate iterator here since
                                                        // binary encoding size is 1 less than buffer size due to start bit

            char oled_char = '\0';                      // initialize character for use in logic/OLED

            for(i = 0; i < CAPTURED_BITS_SIZE; i++) {
                uint64_t waveform = tmp_delta_us[i];
                if (waveform < IS_0) {
                    binary_encoding[binary_encoding_iterator++] = '0';
                }
                else if (waveform < IS_1) {
                    binary_encoding[binary_encoding_iterator++] = '1';
                }
                else if(waveform < IS_START) {
                    continue;                                       // ignore first bit in buffer, is start bit
                }
                else {
                    SysTickReset();                                 // Should not be in loop
                    break;
                }
            }

            decoded = strtol(binary_encoding, NULL, 2);             // convert binary string to a number (long type)

            // Send data to AWS
            if (decoded == BUTTON_MUTE) {
                int clear_send_screen_iterator = OLED_START_SEND_Y_INX;  // set OLED y coordinates to bottom half of screen
                data_to_send[data_to_send_index++] = '\0';               // append \0 so PostAndGet() knows when to stop sending data


                while(clear_send_screen_iterator < SSD1351HEIGHT) {      // clear top (sending side) of OLED screen
                    drawFastHLine(0, clear_send_screen_iterator, SSD1351WIDTH, BLACK);
                    clear_send_screen_iterator++;
                }

                if(CheckIfValidString(data_to_send) == 0) {
                    PrintMessage(0, OLED_START_SEND_Y_INX, "Sending color...", WHITE);
                    PostAndGet();                                            // send data to AWS

                    clear_send_screen_iterator = OLED_START_SEND_Y_INX;
                    while(clear_send_screen_iterator < SSD1351HEIGHT) {      // clear top (sending side) of OLED screen
                       drawFastHLine(0, clear_send_screen_iterator, SSD1351WIDTH, BLACK);
                       clear_send_screen_iterator++;
                    }

                    PrintMessage(0, OLED_START_SEND_Y_INX, "Sent!", WHITE);
                    UtilsDelay(10000000);
                }
                else {
                    PrintMessage(0, OLED_START_SEND_Y_INX, "Invalid color,        try again", WHITE);
                    UtilsDelay(10000000);
                }

                clear_send_screen_iterator = OLED_START_SEND_Y_INX;
                while(clear_send_screen_iterator < SSD1351HEIGHT) {      // clear top (sending side) of OLED screen
                    drawFastHLine(0, clear_send_screen_iterator, SSD1351WIDTH, BLACK);
                    clear_send_screen_iterator++;
                }

                memset(data_to_send, '\0', sizeof(data_to_send));        // clear data sent buffer
                prev_decoded = -1;                                       // set prev_decoded to non-button value
                data_to_send_index = 0;                                  // reset data sent index
                x = 0;                                                   // reset OLED coordinates to beginning values
                y = OLED_START_SEND_Y_INX;
            }

            // Delete last character from data sent buffer/OLED screen
            else if (decoded == BUTTON_LAST) {
                oled_char = data_to_send[--data_to_send_index];   // get old char before decrementing
                data_to_send[data_to_send_index] = '\0';          // replace old char with null byte in string

                if((x == 0) && (y > OLED_START_SEND_Y_INX)) {     // reset x and y to previous values
                    y -= CHARHEIGHT;
                    x = OLED_LAST_CHAR_ON_LINE;
                }
                else if(x > 0) {
                    x -= CHARWIDTH;
                }

                drawChar(x, y, oled_char, BLACK, 0, 1);         // draw over old char in OLED
            }

            // do nothing, used to avoid computations below
            else if (decoded == BUTTON_1) {
                go_back = 1;
                continue;
            }
            else if (decoded == BUTTON_0) {
                oled_char = ' ';
                data_to_send[data_to_send_index] = oled_char;                          // delete last character in data buffer by setting it to new value
                                                                                       // (e.g: change last char 'A' to 'B')
                if(data_to_send_index < DATA_BUFF_SIZE) {
                    data_to_send_index++;
                }

                drawChar(x, y, oled_char, WHITE, 0, 1);
                if((x + CHARWIDTH) > SSD1351WIDTH - 1) {                       // increment coordinates for OLED screen
                    x = 0;
                    y += CHARHEIGHT;
                }
                else {
                    x += CHARWIDTH;
                }
            }
            // Check if the same button was pressed and delay for that press is valid
            else if(decoded == prev_decoded && old_systic_count < BUTTON_PRESS_DELAY) {
               char prev_character = character_map[row_iterator][column_iterator];

               column_iterator++;

               if(column_iterator == 3 && (row_iterator != 7 && row_iterator != 9)) {  // check to see if 4th column has a character (7 and 9 only)
                   column_iterator = 0;
               }
               else {
                   column_iterator = column_iterator % 4;                               // column iterator is 0, 1, 2, 3, or 4. If 4, resets back to 0
               }

               oled_char = character_map[row_iterator][column_iterator];                // get corresponding character from map
               data_to_send[--data_to_send_index] = oled_char;                          // delete last character in data buffer by setting it to new value
                                                                                        // (e.g: change last char 'A' to 'B')
               data_to_send_index++;

               if((x == 0) && (y > OLED_START_SEND_Y_INX)) {                            // reset x and y to previous values
                   y -= CHARHEIGHT;
                   x = OLED_LAST_CHAR_ON_LINE;
               }
               else if(x > 0) {
                   x -= CHARWIDTH;
               }

               drawChar(x, y, prev_character, BLACK, 0, 1);                             // draw over old char
               drawChar(x, y, oled_char, WHITE, 0, 1);                                  // draw new char

               if((x + CHARWIDTH) > SSD1351WIDTH - 1) {                                     // bounds check, increment x and y
                  x = 0;
                  y += CHARHEIGHT;
               }
               else {
                  x += CHARWIDTH;
               }
            }
            else {
                row_iterator = GetButton(decoded);                      // get row index from character array
                if(row_iterator == -1) {
                    row_iterator = GetButton(prev_decoded);             // if button not found, get old character and set the correct row for char_map
                    continue;
                }
                column_iterator = 0;
                oled_char = character_map[row_iterator][column_iterator]; // get character
                data_to_send[data_to_send_index] = oled_char;
                if(data_to_send_index < DATA_BUFF_SIZE) {
                    data_to_send_index++;
                }
                prev_decoded = decoded;

                drawChar(x, y, oled_char, WHITE, 0, 1);
                if((x + CHARWIDTH) > SSD1351WIDTH - 1) {                       // increment coordinates for OLED screen
                    x = 0;
                    y += CHARHEIGHT;
                }
                else {
                    x += CHARWIDTH;
                }
            }
        }
    }
}
unsigned char
GetRegisterValue(char *x_or_y_register)
{
    unsigned char ucDevAddr, ucRegOffset, ucRdLen;
    unsigned char aucRdDataBuf[256];
    char hexDevAddr[] = "0x18";
    char rdLen[] = "1";
    char *pcErrPtr;

    ucDevAddr = (unsigned char)strtoul(hexDevAddr+2, &pcErrPtr, 16);
    ucRegOffset = (unsigned char)strtoul(x_or_y_register+2, &pcErrPtr, 16);
    ucRdLen = (unsigned char)strtoul(rdLen, &pcErrPtr, 10);

    RET_IF_ERR(I2C_IF_Write(ucDevAddr,&ucRegOffset,1,0));

    RET_IF_ERR(I2C_IF_Read(ucDevAddr, &aucRdDataBuf[0], ucRdLen));

    return aucRdDataBuf[0];
}

// Draw Rectangles that the user will pass over to select an LED color
void SetupAccelerometerUI() {
    int colorIterator = 0;
    int drawSquaresWidth = 32;

    fillScreen(BLACK);

    int i;
    for(i = 0; i < RECTANGLE_BOUNDARY; i++) {
       int x = 0;

       // 2nd row of colors
       if (i >= RECTANGLE_BOUNDARY/2) {
           colorIterator = 4;
       }

       // Draw the colors in rows
       drawFastHLine(x, i, drawSquaresWidth, colorArray[colorIterator]);
       x += drawSquaresWidth;
       drawFastHLine(x, i, drawSquaresWidth, colorArray[colorIterator+1]);
       x += drawSquaresWidth;
       drawFastHLine(x, i, drawSquaresWidth, colorArray[colorIterator+2]);
       x += drawSquaresWidth;
       drawFastHLine(x, i, drawSquaresWidth, colorArray[colorIterator+3]);
       x += drawSquaresWidth;
    }

    drawFastHLine(0, i++, SSD1351WIDTH, WHITE);

    const char *infoMessage = "Hit SW3 to Confirm    color, SW2 to go back";  // string is supposed to be like this

    int x = 0; int y = RECTANGLE_BOUNDARY + 2;

    PrintMessage(x, y, infoMessage, WHITE);
    PrintMessage(ACCEL_X_LOG, ACCEL_Y_LOG, "->", WHITE);    // Log message setup
}

// Used to draw over old square after user moves grey square (the thing they are controlling with ACCEL)
unsigned int GetRectangleColor(int x, int y) {
    unsigned int color = 0;

    if (x < RECTANGLE_WIDTH && y < RECTANGLE_HEIGHT) {
        color = colorArray[0];      // White
    }
    else if (x < (RECTANGLE_WIDTH * 2) && y < RECTANGLE_HEIGHT) {
        color = colorArray[1];      // Red
    }
    else if (x < (RECTANGLE_WIDTH * 3) && y < RECTANGLE_HEIGHT) {
        color = colorArray[2];      // Blue
    }
    else if (x < (RECTANGLE_WIDTH * 4) && y < RECTANGLE_HEIGHT) {
        color = colorArray[3];      // Green
    }
    else if (x < RECTANGLE_WIDTH && y >= RECTANGLE_HEIGHT) {
        color = colorArray[4];      // Indigo
    }
    else if (x < (RECTANGLE_WIDTH * 2) && y >= RECTANGLE_HEIGHT) {
        color = colorArray[5];      // Magenta
    }
    else if (x < (RECTANGLE_WIDTH * 3) && y >= RECTANGLE_HEIGHT) {
        color = colorArray[6];      // Yellow
    }
    else if (x < (RECTANGLE_WIDTH * 4) && y >= RECTANGLE_HEIGHT) {
        color = colorArray[7];      // Black
    }
    return color;
}

const char* ConvertColorToString(unsigned int color) {
    if (color == WHITE) { return "WHITE"; }
    else if (color == RED) { return "RED"; }
    else if (color == BLUE) { return "BLUE"; }
    else if (color == GREEN) { return "GREEN"; }
    else if (color == INDIGO) { return "INDIGO"; }
    else if (color == MAGENTA) { return "MAGENTA"; }
    else if (color == YELLOW) { return "YELLOW"; }
    else if (color == BLACK) { return "BLACK"; }
    return "";
}

// Main function that runs the accelerometer logic, user controls grey square
void AccelerometerControl() {
    int previous_x = 0, previous_y = 0;
    int current_x = 0, current_y = 0;

    char x_reg[] = "0x3";
    char y_reg[] = "0x5";

    SetupAccelerometerUI();
    SW2_intflag = 0;
    SW3_intflag = 0;

    while(SW2_intflag == 0) {
       unsigned char raw_x, raw_y;

       raw_x = GetRegisterValue(x_reg);                    // Accelerometer value read at x register
       raw_y = GetRegisterValue(y_reg);                    // Accelerometer value read at y register

       previous_x = current_x;                             // Assign old current values to previous for
       previous_y = current_y;                             // clearing old grey square and to calculate new position

       unsigned int color = GetRectangleColor(previous_x, previous_y);
       fillRect(previous_x, previous_y, SQUAREHEIGHT, SQUAREWIDTH, color);  // Clear old grey square

       if((raw_x >= UP_OR_LEFT_LOWER_BOUND)
               && (raw_x <= UP_OR_LEFT_UPPER_BOUND)) {     // If tilted left
           current_x = previous_x -
                   ((UP_OR_LEFT_UPPER_BOUND - raw_x) / 4); // Decrease x coordinate by value of 0 to 16
       }
       else if(raw_x <= DOWN_OR_RIGHT_UPPER_BOUND) {       // If tilted right
           current_x = previous_x + (raw_x / 4);           // Increase x coordinate by value of 0 to 16
       }

       if((raw_y >= UP_OR_LEFT_LOWER_BOUND)
               && (raw_y <= UP_OR_LEFT_UPPER_BOUND)) {     // If tilted up
           current_y = previous_y +
                   ((UP_OR_LEFT_UPPER_BOUND - raw_y) / 4); // Increase y coordinate by value of 0 to 16
       }
       else if(raw_y <= DOWN_OR_RIGHT_UPPER_BOUND) {       // If tilted down
           current_y = previous_y - (raw_y / 4);           // Decrease y coordinate by value of 0 to 16
       }

       if (current_x < previous_x) {                           // Need square to move in even coordinates so no overlap happens
           while((current_x % SQUAREWIDTH) != 0) {
               current_x++;
           }
       }
       else if (current_x >= previous_x) {                     // Need square to move in even coordinates so no overlap happens
           while((current_x % SQUAREWIDTH) != 0) {             // since cords are tracked /w upper left cord of square
              current_x--;
          }
       }

       if (current_y < previous_y) {                           // Need square to move in even coordinates so no overlap happens
          while((current_y % SQUAREHEIGHT) != 0) {
              current_y++;
          }
       }
       else if (current_y >= previous_y) {                           // Need square to move in even coordinates so no overlap happens
           while((current_y % SQUAREHEIGHT) != 0) {
               current_y++;
           }
       }

       if(current_x > (SSD1351WIDTH - SQUAREWIDTH)) {           // Bounds check for right
           current_x = SSD1351WIDTH - SQUAREWIDTH;              // Set to maximum value of x so entire circle can be displayed
       }
       else if (current_x < 0) {                      // Bounds check for left
           current_x = 0;                             // Set to minimum value of x so entire circle can be displayed
       }

       if(current_y > (RECTANGLE_BOUNDARY - SQUAREHEIGHT)) {          // Bounds check for top
           current_y = RECTANGLE_BOUNDARY - SQUAREHEIGHT;             // Set to maximum value of y so entire circle can be displayed
       }
       else if (current_y < 0) {                      // Bounds check for bottom
           current_y = 0;                             // Set to minimum value of y so entire circle can be displayed
       }

       fillRect(current_x, current_y, SQUAREWIDTH, SQUAREHEIGHT, GREY);    // Draw the circle
       UtilsDelay(60000);

       // Send POST to AWS
       if(SW3_intflag == 1) {
           memset(data_to_send, '\0', sizeof(data_to_send));
           char send_message[42] = "";

           // Let user know SW3 has been preessed and sending to AWS
           ClearMessage(ACCEL_Y_LOG, ACCEL_Y_LOG + 9);
           strcpy(send_message, "-> Color selected is  ");
           strcpy(data_to_send, ConvertColorToString(GetRectangleColor(current_x, current_y)));
           strcat(send_message, data_to_send);

           PrintMessage(ACCEL_X_LOG, ACCEL_Y_LOG, send_message, WHITE);    // Color that was selected
           PostAndGet();

           // Let user know message was sent and reset log
           ClearMessage(ACCEL_Y_LOG, ACCEL_Y_LOG + 17);
           PrintMessage(ACCEL_X_LOG, ACCEL_Y_LOG, "-> Sent!", WHITE);
           UtilsDelay(10000000);
           ClearMessage(ACCEL_Y_LOG, ACCEL_Y_LOG + 9);
           PrintMessage(ACCEL_X_LOG, ACCEL_Y_LOG, "->", WHITE);

           SW3_intflag = 0;
       }
   }

    SW2_intflag == 0;
}

// Used for main menu screen
int PrintMessageHalfScreen(int x, int y, char* message, unsigned int color, char left_or_right, int text_size) {
    int messageSize = strlen(message);
    int i;
    int boundary, original_x = x;

    // Check if using left half or right half of screen
    if(left_or_right == 'L' || left_or_right == 'l') {
        boundary = SSD1351WIDTH/2 - 3;
    }
    else if (left_or_right == 'R' || left_or_right == 'r') {
        boundary = SSD1351WIDTH;
    }

    // Write string depending on L or R
    for(i = 0; i < messageSize; i++) {
        drawChar(x, y, message[i], color, 0, text_size);
        if((x + (CHARWIDTH * text_size)) >= boundary) {
            x = original_x;
            y += CHARHEIGHT * text_size;
        }
        else {
            x += CHARWIDTH * text_size;
        }
    }
    return y;
}

void HomeScreen() {
    const char *infoMessage = "Choose how to control the LED strip";  // string is supposed to be like this

    while(1){

        int x = 0, y = 0, top_after_info_message = 17;

        fillScreen(BLACK);

        SW3_intflag = 0;
        SW2_intflag = 0;
        buffer_count = 0;

        PrintMessage(x, y, infoMessage, WHITE);

        y += (CHARHEIGHT * 2) + 1;

        y += CHARHEIGHT;

        // ACCEL Message
        char *accelerometer_Message = "To use the ACCEL, hit";
        y = PrintMessageHalfScreen(0, y, accelerometer_Message, RED, 'l', 1);
        y += CHARHEIGHT * 5;
        y = PrintMessageHalfScreen(0, y, "SW3 / SW2", GREEN, 'l', 2);

        y = top_after_info_message;

        drawFastVLine(SSD1351WIDTH/2, y, SSD1351HEIGHT, WHITE); // draw line in the middle of screen

        y += CHARHEIGHT;

        // Remote Message
        char *remote_Message = "To use the Remote,    press";
        y = PrintMessageHalfScreen(66, y, remote_Message, INDIGO, 'r', 1);
        y += CHARHEIGHT * 4;
        y = PrintMessageHalfScreen(66, y, "Any Number Button", YELLOW, 'r', 1);

        // Wait for interupt, either button or remote
        while((buffer_count < CAPTURED_BITS_SIZE) && (SW2_intflag == 0) && (SW3_intflag == 0));
        UtilsDelay(1000);

        if(buffer_count == CAPTURED_BITS_SIZE) {
            RemoteControl();
        }
        else if((SW2_intflag == 1) || (SW3_intflag == 1)) {
            AccelerometerControl();
        }

    }
}

int main() {

    BoardInit();

    PinMuxConfig();

    // Initialize UART Terminal
    InitTerm();

    memset((void *)received_buffer, 0, sizeof(received_buffer));                // clear buffer that receives UART1 messages

    //
    // Reset SPI
    //
    MAP_SPIReset(GSPI_BASE);

    //
    // Configure SPI interface
    //
    MAP_SPIConfigSetExpClk(GSPI_BASE,MAP_PRCMPeripheralClockGet(PRCM_GSPI),
                     SPI_IF_BIT_RATE,SPI_MODE_MASTER,SPI_SUB_MODE_0,
                     (SPI_SW_CTRL_CS |
                     SPI_4PIN_MODE |
                     SPI_TURBO_OFF |
                     SPI_CS_ACTIVEHIGH |
                     SPI_WL_8));

    //
    // Enable SPI for communication
    //
    MAP_SPIEnable(GSPI_BASE);

    //
    // I2C Init
    //
    I2C_IF_Open(I2C_MASTER_MODE_FST);

    // Initialize Systick
    MAP_SysTickPeriodSet(SYSTICK_RELOAD_VAL);
    MAP_SysTickIntRegister(SysTickHandler);
    MAP_SysTickIntEnable();
    MAP_SysTickEnable();

    // Register GPIO Interrupts
    MAP_GPIOIntRegister(IR_GPIO_PORT, GPIOA0IntHandler);
    MAP_GPIOIntTypeSet(IR_GPIO_PORT, IR_GPIO_PIN, GPIO_FALLING_EDGE);
    unsigned long ulStatus = MAP_GPIOIntStatus(IR_GPIO_PORT, false);
    MAP_GPIOIntClear(IR_GPIO_PORT, ulStatus);
    MAP_GPIOIntEnable(IR_GPIO_PORT, IR_GPIO_PIN);

    //
    // Register Switch handlers
    //
    MAP_GPIOIntRegister(GPIOA1_BASE, GPIOA1IntHandler);
    MAP_GPIOIntRegister(switch2.port, GPIOA2IntHandler);

    //
    // Configure rising edge interrupts on SW2 and SW3
    //
    MAP_GPIOIntTypeSet(switch3.port, switch3.pin, GPIO_RISING_EDGE);    // SW3
    MAP_GPIOIntTypeSet(switch2.port, switch2.pin, GPIO_RISING_EDGE);    // SW2

    ulStatus = MAP_GPIOIntStatus(switch3.port, false);
    MAP_GPIOIntClear(switch3.port, ulStatus);           // clear interrupts on GPIOA1
    ulStatus = MAP_GPIOIntStatus(switch2.port, false);
    MAP_GPIOIntClear(switch2.port, ulStatus);           // clear interrupts on GPIOA2

    // clear global variables
    SW2_intflag = 0;
    SW3_intflag = 0;
    buffer_count = 0;

    // Enable SW2 and SW3 interrupts
    MAP_GPIOIntEnable(switch3.port, switch3.pin);
    MAP_GPIOIntEnable(switch2.port, switch2.pin);

    // Set up OLED board
    Adafruit_Init();

    HomeScreen();
}
