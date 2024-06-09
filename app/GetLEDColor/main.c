//*****************************************************************************
//
// Application Name     -   Get LED strip State To AWS
// Created by           -   Mitul Mathur,
//                      -   Austin Bunuan
//
// Base code sources    - Lab 4 Blank from Canvas  (used to setup socket connection and Get and Post requests)
//
//*****************************************************************************
//
//! \addtogroup ssl
//! @{
//
//*****************************************************************************

#include <stdio.h>
#include <string.h>
#include "cJSON.h"

// Simplelink includes
#include "simplelink.h"

//Driverlib includes
#include "hw_types.h"
#include "hw_ints.h"
#include "hw_memmap.h"
#include "hw_common_reg.h"
#include "interrupt.h"
#include "hw_apps_rcm.h"
#include "prcm.h"
#include "rom.h"
#include "rom_map.h"
#include "prcm.h"
#include "gpio.h"
#include "utils.h"

//Common interface includes
#include "pin_mux_config.h"
#include "gpio_if.h"
#include "common.h"
#include "uart_if.h"

// Custom includes
#include "utils/network_utils.h"


//NEED TO UPDATE THIS FOR IT TO WORK!
#define REDHIGH             0x10
#define BLUEHIGH            0x8
#define GREENHIGH           0x80

#define RED                 0x10
#define BLUE                0x8
#define GREEN               0x80


#define DATE                2    /* Current Date */
#define MONTH               6     /* Month 1-12 */
#define YEAR                2024  /* Current year */
#define HOUR                12   /* Time - hours */
#define MINUTE              56    /* Time - minutes */
#define SECOND              0     /* Time - seconds */

#define GET                 0
#define POST                1


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

#define FIRST_DATA "{" \
        "\"state\": {\r\n"                                          \
            "\"reported\": {\r\n"                                   \
                "\"color\": \""
#define SECOND_DATA "\"\r\n"                                       \
                "}"                                                 \
            "}"                                                     \
        "}\r\n\r\n"
char *c_var = "";

//*****************************************************************************
//                 GLOBAL VARIABLES -- Start
//*****************************************************************************

#if defined(ccs) || defined(gcc)
extern void (* const g_pfnVectors[])(void);
#endif
#if defined(ewarm)
extern uVectorEntry __vector_table;
#endif

//*****************************************************************************
//                 GLOBAL VARIABLES -- End: df
//*****************************************************************************


//****************************************************************************
//                      LOCAL FUNCTION PROTOTYPES
//****************************************************************************
static int set_time();
static void BoardInit(void);
static int http_get(int);
static int http_post(int);
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
void setup_socket(int get_or_post) {
    long lRetVal = -1;
    UART_PRINT("My terminal works!\n\r");

    // initialize global default app configuration
    g_app_config.host = SERVER_NAME;
    g_app_config.port = GOOGLE_DST_PORT;

    //Connect the CC3200 to the local access point
    lRetVal = connectToAccessPoint();
    //Set time so that encryption can be used
    lRetVal = set_time();
    if(lRetVal < 0) {
        UART_PRINT("Unable to set time in the device");
        LOOP_FOREVER();
    }
    //Connect to the website with TLS encryption
    lRetVal = tls_connect();
    if(lRetVal < 0) {
        ERR_PRINT(lRetVal);
    }
    if(get_or_post == 0) {
        http_get(lRetVal);
    }
    else if(get_or_post == 1) {
        http_post(lRetVal);
    }


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

    pcBufHeaders = acSendBuff;
    strcpy(pcBufHeaders, GETHEADER);
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
    lRetVal = sl_Send(iTLSSockID, acSendBuff, strlen(acSendBuff), 0);
    if(lRetVal < 0) {
       UART_PRINT("GET failed. Error Number: %i\n\r",lRetVal);
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
   const char *body_start = strstr(acRecvbuff, "\r\n\r\n");          // From Get Request Response, start parsing only at JSON body
   cJSON *json = cJSON_Parse(body_start);                            // check for good JSON, if it exists it's stored
   if (json == NULL) {
       UART_PRINT("Error: parsing JSON");
       const char *error_ptr = cJSON_GetErrorPtr();
      if (error_ptr != NULL) {
          fprintf(stderr, "Error before: %s\n", error_ptr);
      }
       return 1;
   }
   cJSON *state = cJSON_GetObjectItem(json, "state");               // Find the "state" field in the JSON file and store its content
   if (state == NULL) {
       UART_PRINT("Error: no \"state\" object found\n");
       cJSON_Delete(json);
       return 1;
    }
   cJSON *desired = cJSON_GetObjectItem(state, "desired");          // Find the "desired" field and store its content
   if (desired == NULL) {
      UART_PRINT("Error: no \"desired\" object found\n");
      cJSON_Delete(json);
      return 1;
   }
   cJSON *color = cJSON_GetObjectItem(desired, "color");            // Find the "color" foe;d

   strcpy(c_var, color->valuestring);                               // Store color as a string
   UART_PRINT("%s\n\r", c_var);

    return 0;
}

static int http_post(int iTLSSockID){
    char acSendBuff[512];
    char acRecvbuff[1460];
    char cCLLength[200];
    char* pcBufHeaders;
    int lRetVal = 0;

    pcBufHeaders = acSendBuff;
    strcpy(pcBufHeaders, POSTHEADER);
    pcBufHeaders += strlen(POSTHEADER);
    strcpy(pcBufHeaders, HOSTHEADER);
    pcBufHeaders += strlen(HOSTHEADER);
    strcpy(pcBufHeaders, CHEADER);
    pcBufHeaders += strlen(CHEADER);
    strcpy(pcBufHeaders, "\r\n\r\n");

    int dataLength = strlen(c_var) + strlen(FIRST_DATA) + strlen(SECOND_DATA);

    strcpy(pcBufHeaders, CTHEADER);
    pcBufHeaders += strlen(CTHEADER);
    strcpy(pcBufHeaders, CLHEADER1);

    pcBufHeaders += strlen(CLHEADER1);
    sprintf(cCLLength, "%d", dataLength);

    strcpy(pcBufHeaders, cCLLength);
    pcBufHeaders += strlen(cCLLength);
    strcpy(pcBufHeaders, CLHEADER2);
    pcBufHeaders += strlen(CLHEADER2);

    strcpy(pcBufHeaders, FIRST_DATA);
    pcBufHeaders += strlen(FIRST_DATA);
    strcpy(pcBufHeaders, c_var);
    pcBufHeaders += strlen(c_var);
    strcpy(pcBufHeaders, SECOND_DATA);
    pcBufHeaders += strlen(SECOND_DATA);

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

int main() {
    // Initialize board configuration
    //
    BoardInit();

    PinMuxConfig();

    InitTerm();
    ClearTerm();

    while(1) {
        UtilsDelay(150000000);

        setup_socket(GET);

        // Compare stored string (the color) with valid LED colors
        if (strcmp(c_var, "RED") == 0){
            GPIOPinWrite(GPIOA0_BASE, BLUE, 0);    // blue
            GPIOPinWrite(GPIOA0_BASE, RED, REDHIGH);  // red
            GPIOPinWrite(GPIOA0_BASE, GREEN, 0);  // green
        }
        else if (strcmp(c_var, "BLUE") == 0){
            GPIOPinWrite(GPIOA0_BASE, BLUE, BLUEHIGH);    // blue
            GPIOPinWrite(GPIOA0_BASE, RED, 0);  // red
            GPIOPinWrite(GPIOA0_BASE, GREEN, 0);  // green
        }
        else if (strcmp(c_var, "GREEN") == 0){
            GPIOPinWrite(GPIOA0_BASE, BLUE, 0);    // blue
            GPIOPinWrite(GPIOA0_BASE, RED, 0);  // red
            GPIOPinWrite(GPIOA0_BASE, GREEN, GREENHIGH);  // green
        }
        else if (strcmp(c_var, "YELLOW") == 0){
            GPIOPinWrite(GPIOA0_BASE, BLUE, 0);    // blue
            GPIOPinWrite(GPIOA0_BASE, RED, REDHIGH);  // red
            GPIOPinWrite(GPIOA0_BASE, GREEN, GREENHIGH);  // green
        }
        else if (strcmp(c_var, "WHITE") == 0){
            GPIOPinWrite(GPIOA0_BASE, BLUE, BLUEHIGH);    // blue
            GPIOPinWrite(GPIOA0_BASE, RED, REDHIGH);  // red
            GPIOPinWrite(GPIOA0_BASE, GREEN, GREENHIGH);  // green
        }
        else if (strcmp(c_var, "BLACK") == 0){
            GPIOPinWrite(GPIOA0_BASE, BLUE, 0);    // blue
            GPIOPinWrite(GPIOA0_BASE, RED, 0);  // red
            GPIOPinWrite(GPIOA0_BASE, GREEN, 0);  // green
        }
        else if (strcmp(c_var, "INDIGO") == 0){
            GPIOPinWrite(GPIOA0_BASE, BLUE, BLUEHIGH);    // blue
            GPIOPinWrite(GPIOA0_BASE, RED, 0);  // red
            GPIOPinWrite(GPIOA0_BASE, GREEN, GREENHIGH);  // green
        }
        else if (strcmp(c_var, "MAGENTA") == 0){
            GPIOPinWrite(GPIOA0_BASE, BLUE, BLUEHIGH);    // blue
            GPIOPinWrite(GPIOA0_BASE, RED, REDHIGH);  // red
            GPIOPinWrite(GPIOA0_BASE, GREEN, 0);  // green
        }
        else {
            int i = 0;
            while (i < 10){
                GPIOPinWrite(GPIOA0_BASE, BLUE, BLUEHIGH);    // blue
                GPIOPinWrite(GPIOA0_BASE, RED, REDHIGH);  // red
                GPIOPinWrite(GPIOA0_BASE, GREEN, GREENHIGH);  // green
                MAP_UtilsDelay(8000000);
                GPIOPinWrite(GPIOA0_BASE, BLUE, 0);    // blue
                GPIOPinWrite(GPIOA0_BASE, RED, 0);  // red
                GPIOPinWrite(GPIOA0_BASE, GREEN, 0);  // green
                MAP_UtilsDelay(8000000);
                i++;
            }
            c_var = "";
        }
        setup_socket(POST);
    }



}
