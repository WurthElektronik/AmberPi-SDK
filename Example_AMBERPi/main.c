/**
 ***************************************************************************************************
 * This file is part of WIRELESS CONNECTIVITY SDK:
 * https://www.we-online.com/wireless-connectivity
 *
 * THE SOFTWARE INCLUDING THE SOURCE CODE IS PROVIDED “AS IS”. YOU ACKNOWLEDGE THAT WÜRTH ELEKTRONIK
 * EISOS MAKES NO REPRESENTATIONS AND WARRANTIES OF ANY KIND RELATED TO, BUT NOT LIMITED
 * TO THE NON-INFRINGEMENT OF THIRD PARTIES’ INTELLECTUAL PROPERTY RIGHTS OR THE
 * MERCHANTABILITY OR FITNESS FOR YOUR INTENDED PURPOSE OR USAGE. WÜRTH ELEKTRONIK EISOS DOES NOT
 * WARRANT OR REPRESENT THAT ANY LICENSE, EITHER EXPRESS OR IMPLIED, IS GRANTED UNDER ANY PATENT
 * RIGHT, COPYRIGHT, MASK WORK RIGHT, OR OTHER INTELLECTUAL PROPERTY RIGHT RELATING TO ANY
 * COMBINATION, MACHINE, OR PROCESS IN WHICH THE PRODUCT IS USED. INFORMATION PUBLISHED BY
 * WÜRTH ELEKTRONIK EISOS REGARDING THIRD-PARTY PRODUCTS OR SERVICES DOES NOT CONSTITUTE A LICENSE
 * FROM WÜRTH ELEKTRONIK EISOS TO USE SUCH PRODUCTS OR SERVICES OR A WARRANTY OR ENDORSEMENT
 * THEREOF
 *
 * THIS SOURCE CODE IS PROTECTED BY A LICENSE.
 * FOR MORE INFORMATION PLEASE CAREFULLY READ THE LICENSE AGREEMENT FILE LOCATED
 * IN THE ROOT DIRECTORY OF THIS DRIVER PACKAGE.
 *
 * COPYRIGHT (c) 2019 Würth Elektronik eiSos GmbH & Co. KG
 *
 ***************************************************************************************************
 **/

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>

/* tested with wiringPi version 2.38 */
#include <wiringPi.h>
#include <wiringPiI2C.h>

#include "../drivers/TarvosIII/TarvosIII.h"
#include "../drivers/WE-common.h"
#include "../drivers/global/global.h"
#include "../drivers/lis3dh/lis3dh_driver.h"
#include "../drivers/lis2dw12/lis2dw12_driver.h"
#include "../drivers/hts221/HTS221_Driver.h"
#include "../drivers/lps22hb/LPS22HB_Driver.h"
#include "../drivers/WSEN_PADS_2511020213301/WSEN_PADS_2511020213301.h"


#define MAX_RETRIES_SENSORS 5
static void Application(void);

static void RX_test(void);

static void TarvosIII_test_function(void);

static void RXcallback(uint8_t* payload, uint8_t payload_length, uint8_t dest_network_id, uint8_t dest_address_lsb, uint8_t dest_address_msb, int8_t rssi);

pthread_t thread_main;
pthread_t thread_heartbeat;

bool AbortMainLoop = false;
bool AbortUartHeartBeatThread = false;

int pwm_pin;

void *heartbeat_Thread(void *pArgs)
{
    /* apply a higher priority to this thread to be prioritized w.r.t. the main function  */
    setThreadPrio(PRIO_AMBERPI_HEARTBEATTHREAD);

    int pwm = 15;
    int rate = 22;
    int i = 0;
    int pwm_max = 600; /* power of the PWM, max value of pwm_max is 1023 (~100%) */

    pinMode(pwm_pin,PWM_OUTPUT);
    pullUpDnControl(pwm_pin, PUD_OFF);
    while(1)
    {
        if(AbortUartHeartBeatThread == true)
        {
            /* abort thread */
            AbortUartHeartBeatThread = false;
            pinMode (pwm_pin, OUTPUT) ;
            digitalWrite (pwm_pin,  LOW) ;
            pinMode (pwm_pin, INPUT) ;
            break;
        }

        /* apply heart beat pattern */
        for(i = 1; i < pwm; i++)
        {
            pwmWrite (pwm_pin, (pwm_max*i)/pwm) ;
            delay(((60000/rate)*.1)/pwm);
        }
        for (i = pwm; i > 1; i--)
        {
            pwmWrite (pwm_pin, (pwm_max*i)/pwm) ;
            delay(((60000/rate)*.2)/pwm);
        }
        for(i = 1; i < pwm; i++)
        {
            pwmWrite (pwm_pin, (pwm_max*i)/pwm) ;
            delay(((60000/rate)*.1)/pwm);
        }
        for (i = pwm; i > 1; i--)
        {
            pwmWrite (pwm_pin, (pwm_max*i)/pwm) ;
            delay(((60000/rate)*.6)/pwm);
        }
    }
    return 0;
}

void *main_Thread(void *pArgs)
{
    /* apply a higher priority to this thread to be prioritized w.r.t. the main function  */
    setThreadPrio(PRIO_MAIN_THREAD);

#if 1
    /* default application - reading and transmitting sensor values */
    Application();
#elif 0
    /* enable this to test the rx capabilities */
    RX_test();
#else
    /* function to test all functions of the TarvosIII driver */
    TarvosIII_test_function();
#endif

    AbortMainLoop = true;
    return 0;
}

/* the main function simply starts the MainThread */
int main ()
{
    AbortMainLoop = false;

    /* initialize wiringPi */
    if (wiringPiSetup () == -1)
    {
        fprintf (stdout, "Unable to start wiringPi: %s\n", strerror (errno)) ;
    }
    else
    {
        /* wiring PI started successfully */
        int wiring_version_major;
        int wiring_version_minor;
        wiringPiVersion(&wiring_version_major, &wiring_version_minor);
        fprintf (stdout, "WiringPi library version %d.%d\n\n",wiring_version_major,wiring_version_minor);
    }

    if(pthread_create(&thread_main, NULL, &main_Thread, NULL))
    {
        fprintf(stdout, "Failed to start thread_main");
        return false;
    }

    /* set the PWM pin */
    pwm_pin = TarvosIII_PIN_PWM;
    /* start heart beat thread */
    if(pthread_create(&thread_heartbeat, NULL, &heartbeat_Thread, NULL))
    {
        fprintf(stdout, "Failed to start thread_heartbeat");
        return false;
    }

    while(1)
    {
        delay(1000);
        if(AbortMainLoop == true)
        {
            /* jump out of the main loop and exit program */
            return 0;
        }
    }

    return 0;
}


/* callback for data reception */
static void RXcallback(uint8_t* payload, uint8_t payload_length, uint8_t dest_network_id, uint8_t dest_address_lsb, uint8_t dest_address_msb, int8_t rssi)
{
    int i = 0;
    printf ("Received data from address (NetID:0x%02x,Addr:0x%02x%02x) with %d dBm:\n-> ", dest_network_id, dest_address_lsb, dest_address_msb, rssi);
    printf("0x ");
    for(i=0; i<payload_length; i++)
    {
        printf ("%02x ", *(payload+i)) ;
    }
    printf ("\n-> ") ;
    for(i=0; i<payload_length; i++)
    {
        printf ("%c", *(payload+i)) ;
    }
    printf ("\n") ;
    fflush (stdout) ;
}

static bool SetupHTS221()
{
    const int addr_hts221 = ADDRESS_I2C_HTS221;

    HTS221_Error_et status;
    uint8_t Data = 0;
    bool ret = false;

    /* init sensor. Proceed if OK*/
    if(HTS221_OK == HTS221_Init(addr_hts221))
    {
        HTS221_Get_DeviceID(&Data);
        ret = (Data == HTS221_WHO_AM_I_VAL);
        Debug_out("HTS221 Communication", ret);

        /* If communication is open, configure sensor*/
        if(ret)
        {
            /* configure the sensor */
            if(HTS221_OK == HTS221_MemoryBoot())
            {
                delay(100);
                HTS221_Init_st hts221_init;
                hts221_init.avg_h = HTS221_AVGH_32; // set humidity average mode
                hts221_init.avg_t = HTS221_AVGT_16; // set temperature average mode
                hts221_init.odr = HTS221_ODR_7HZ; //set update interval
                hts221_init.bdu_status = HTS221_ENABLE; //update continiously
                hts221_init.heater_status = HTS221_DISABLE; //disable heater
                hts221_init.irq_output_type = HTS221_OPENDRAIN; //interrupts are disabled
                hts221_init.irq_level = HTS221_HIGH_LVL;
                hts221_init.irq_enable = HTS221_DISABLE;

                ret = (HTS221_OK == HTS221_Set_InitConfig(&hts221_init));
            }
            else
            {
                ret = false;
            }
        }

        if(ret)
        {
            /* set hts221 to active mode if not oneshot method is used */
            ret = (HTS221_OK == HTS221_Activate());
        }
    }

    return ret;
}

static bool SetupLPS22HB()
{
    const int addr_lps22hb = ADDRESS_I2C_LPS22HB;
    bool ret = false;

    if(LPS22HB_OK == LPS22HB_Init(addr_lps22hb))
    {
        uint8_t data = 0;

        /* first communication test */
        LPS22HB_Get_DeviceID(&data);
        ret = (data == LPS22HB_WHO_AM_I_VAL);
        Debug_out("LPS22HB Communication", ret);
    }

    return ret;
}

static bool SetupLIS2DW()
{
    bool ret = false;
    status_t lis2dw_status;

    if(MEMS_SUCCESS == LIS2DW_SpiInit(SPI_CHANNEL_1))
    {
        ret = (MEMS_SUCCESS == LIS2DW_SetSLPModeSel(LIS2DW_SLP_MODE_SELECT_SPI));
    }

    /* first communication test */
    if(ret)
    {
        uint8_t data = 0;
        if(MEMS_SUCCESS == LIS2DW_GetWHO_AM_I(&data))
        {
            /* received response */
            ret = (data == LIS2DW_WHO_AM_I_RESPONSE);
            Debug_out("LIS2DW Communication", ret);
        }
    }

    if(ret)
    {
        /* lis2dw sensor configuration */
        lis2dw_status = LIS2DW_SetODR(LIS2DW_ODR_12_5Hz_NP_1_6Hz_LP);
        lis2dw_status = LIS2DW_SetMode(LIS2DW_SINGLE_CONVERSION);
        lis2dw_status = LIS2DW_SetLPMode(LIS2DW_LOW_POWER_1);
        lis2dw_status = LIS2DW_SetFullScale(LIS2DW_FULLSCALE_4);
        lis2dw_status = LIS2DW_SetAxis(LIS2DW_Axis_Z_Enable|LIS2DW_Axis_Y_Enable|LIS2DW_Axis_X_Enable);
    }

    return ret;
}

static bool SetupWsenPADS()
{
    //On the WSEN_PADS evaluation board, the SAO pin is set to high by default
    //Using the I2C address 1 = 0x5D
    const int addr_wsen_pads = PADS_ADDRESS_I2C_1;
    int8_t status = 0;
    uint8_t deviceIdValue = 0;
    bool ret = false;

    /* initialize the platform I2C drivers */
    status = I2CInit(addr_wsen_pads);

    if(status == WE_SUCCESS)
    {
        /* first communication test */
        deviceIdValue = PADS_getDeviceID();

        if(deviceIdValue == PADS_DEVICE_ID_VALUE)
        {

            Debug_out("WSEN_PADS Communication success\r\n", true);
            ret = true;

            /*set ODR to Power down*/
            PADS_setOutputDataRate(outputDataRatePowerDown);

            /*set low pass filter to BW=ODR/20*/
            PADS_setLowPassFilterConf(lPfilterBW2);

            /*enable low pass filter */
            PADS_setLowPassFilter(enable);

            /*enable Block Data Update*/
            PADS_setBlockDataUpdate(enable);

            /*select Power Mode [0:low Current Mode; 1:low noise Mode]*/
            PADS_setPowerMode(lowNoise);

            /*enable the Auto Increment */
            PADS_setAutoIncrement(enable);
        }
        else
        {
            Debug_out("WSEN_PADS Communication fail\r\n", false);
            ret = false;
        }
    }
    else
    {
        Debug_out("Platform I2C Init error ",false);
    }

    return ret;
}

static void Application()
{
    bool ret = false;
    uint8_t Data = 0;
    char ascii_data[200];

    /* variables for HTS221 */
    bool isSetupHTS221 = false;
    int16_t HTS221_temperature = 0;
    uint16_t HTS221_humidity = 0;
    HTS221_Error_et status;
    char outputDataHTS221[64];
    char rfDataHTS221[50];

    /* variables for LPS22HB */
    bool isSetupLPS22HB = false;
    status_t lis2dw_status;
    LPS22HB_MeasureTypeDef_st LPS22HB_measurement;
    char outputDataLPS22HB[64];
    char rfDataLPS22HB[50];

    /* variables for LIS2DW*/
    bool isSetupLIS2DW = false;
    AxesRaw_t LIS2DW_motion;
    int16_t LIS2DW_temperaturedata;
    float LIS2DW_temperaturevalue;
    char outputDataLIS2DW[64];
    char rfDataLIS2DW[50];

    /* variables for WSEN_PADS */
    bool isSetUpWsenPads = false;
    uint8_t WSEN_PADS_pressureDataStatus = 0;
    uint8_t WSEN_PADS_temperatureDataStatus = 0;
    float WSEN_PADS_temperatureData = 0;
    float WSEN_PADS_pressureData = 0;
    char outputDataWsenPads[64];
    char rfDataWsenPads[50];

    /* initialize the module TarvosIII */
    ret = TarvosIII_Init(115200, TarvosIII_PIN_RESET, TarvosIII_PIN_WAKEUP, TarvosIII_PIN_BOOT, RXcallback, AddressMode_0);
    Debug_out("TarvosIII_Init", ret);

    /* If TarvosIII was initialized, continue with application*/
    if(ret)
    {
        /* initialize the hts221 temperature and humidity sensor */
        isSetupHTS221 = SetupHTS221();

        /* initialize the lps22hb pressure sensor */
        isSetupLPS22HB = SetupLPS22HB();

        /* initialite the lis2dw motion sensor */
        isSetupLIS2DW = SetupLIS2DW();

        /* init WSEN_PADS */
        isSetUpWsenPads = SetupWsenPADS();


        while(1)
        {
            delay(1000);

            /* Read LPS22HB  */
            LPS22HB_measurement.Tout = 0;
            LPS22HB_measurement.Pout = 0;
            sprintf(outputDataLPS22HB, "");
            sprintf(rfDataLPS22HB, "");

            if(isSetupLPS22HB)
            {
                if(LPS22HB_OK == LPS22HB_StartOneShotMeasurement())
                {
                    uint8_t bitStatus = LPS22HB_RESET;
                    uint8_t maxretries = MAX_RETRIES_SENSORS;
                    do
                    {
                        if(LPS22HB_OK != LPS22HB_IsMeasurementCompleted(&bitStatus))
                        {
                            break;
                        }
                        delay(10);
                    }
                    while((bitStatus == LPS22HB_RESET) && (maxretries-- > 0));
                    if(bitStatus == LPS22HB_SET)
                    {
                        status = LPS22HB_Get_Measurement(&LPS22HB_measurement);
                        if(LPS22HB_OK == status)
                        {
                            sprintf(outputDataLPS22HB, "LPS22HB T: %3.2f°C P: %4.2fmbar\n", (float)LPS22HB_measurement.Tout/100, (float)LPS22HB_measurement.Pout/100);
                            sprintf(rfDataLPS22HB, "LPS22HB(T:%3.2f°C P:%4.2fmbar)", (float)LPS22HB_measurement.Tout/100, (float)LPS22HB_measurement.Pout/100);
                        }
                    }
                }
            }

            /* Read HTS221 */
            HTS221_humidity = 0;
            HTS221_temperature = 0;
            sprintf(outputDataHTS221, "");
            sprintf(rfDataHTS221, "");

            if(isSetupHTS221)
            {
                HTS221_BitStatus_et bitStatus = HTS221_RESET;
                uint8_t maxretries = MAX_RETRIES_SENSORS;
                do
                {
                    if(HTS221_OK != HTS221_IsMeasurementCompleted(&bitStatus))
                    {
                        break;
                    }
                    delay(10);
                }
                while((bitStatus == HTS221_RESET) && (maxretries-- > 0));
                if(bitStatus == HTS221_SET)
                {
                    status = HTS221_Get_Measurement(&HTS221_humidity, &HTS221_temperature);
                    if(HTS221_OK == status)
                    {
                        sprintf(outputDataHTS221, "HTS221  T: %3.2f°C H: %2.2f%%\n", (float)HTS221_temperature/10, (float)HTS221_humidity/10);
                        sprintf(rfDataHTS221, "HTS221(H:%2.2f%% T:%3.2f°C)",(float)HTS221_humidity/10, (float)HTS221_temperature/10);
                    }
                }
            }

            /* Read LIS2DW */
            LIS2DW_temperaturevalue = 0;
            LIS2DW_motion.AXIS_X = 0;
            LIS2DW_motion.AXIS_Y = 0;
            LIS2DW_motion.AXIS_Z = 0;
            sprintf(outputDataLIS2DW, "");
            sprintf(rfDataLIS2DW, "");

            if(isSetupLIS2DW)
            {
                if(MEMS_SUCCESS == LIS2DW_SetSLPModeEnable())
                {
                    uint8_t bitStatus = MEMS_SET;
                    uint8_t maxretries = MAX_RETRIES_SENSORS;
                    do
                    {
                        if(MEMS_SUCCESS != LIS2DW_GetDataAvailable(&bitStatus))
                        {
                            break;
                        }
                        delay(10);
                    }
                    while((bitStatus == MEMS_SET) && (maxretries-- > 0));
                    if(bitStatus == MEMS_RESET)
                    {
                        LIS2DW_GetAccAxesRaw(&LIS2DW_motion);
                        LIS2DW_ConvAccValue(&LIS2DW_motion);

                        LIS2DW_GetTempRaw(&LIS2DW_temperaturedata);
                        LIS2DW_ConvTempValue(&LIS2DW_temperaturedata,&LIS2DW_temperaturevalue);

                        sprintf(outputDataLIS2DW,"LIS2DW  T: %5.2f°C X: %6dmg Y: %6dmg Z: %6dmg\n",LIS2DW_temperaturevalue,LIS2DW_motion.AXIS_X,LIS2DW_motion.AXIS_Y,LIS2DW_motion.AXIS_Z);
                        sprintf(rfDataLIS2DW, "LIS2DW(X:%6d Y:%6d Z:%6d T:%5f°C)", LIS2DW_motion.AXIS_X,LIS2DW_motion.AXIS_Y,LIS2DW_motion.AXIS_Z, LIS2DW_temperaturevalue);
                    }
                }
            }

            WSEN_PADS_temperatureData = 0;
            WSEN_PADS_pressureData = 0;
            sprintf(outputDataWsenPads,"");
            sprintf(rfDataWsenPads,"");

            if(isSetUpWsenPads)
            {
                /*Start a conversion*/
                if(WE_SUCCESS == PADS_setSingleConvMode(enable))
                {
                    uint8_t maxRetries = MAX_RETRIES_SENSORS;
                    do
                    {
                        /*check the temp and press Data status*/
                        WSEN_PADS_temperatureDataStatus = PADS_getTempStatus();
                        WSEN_PADS_pressureDataStatus = PADS_getPresStatus();
                        delay(10);

                    }
                    while(((WSEN_PADS_pressureDataStatus == 0) || (WSEN_PADS_temperatureDataStatus == 0)) && (maxRetries-- > 0));

                    if(1 == WSEN_PADS_pressureDataStatus)
                    {
                        WSEN_PADS_pressureData = PADS_getPressure();
                    }

                    if(1 == WSEN_PADS_temperatureDataStatus)
                    {
                        WSEN_PADS_temperatureData = PADS_getTemperature();
                    }

                    sprintf(outputDataWsenPads, "WSEN_PADS T: %3.2f°C P: %4.2fmbar\n", WSEN_PADS_temperatureData, WSEN_PADS_pressureData);
                    sprintf(rfDataWsenPads, "WSEN_PADS(T:%3.2f°C P:%4.2fmbar)", WSEN_PADS_temperatureData, WSEN_PADS_pressureData);
                }
            }

            /* display the sensor data */
            fprintf(stdout,"\n\n");
            fprintf(stdout, outputDataLIS2DW);
            fprintf(stdout, outputDataLPS22HB);
            fprintf(stdout, outputDataHTS221);
            fprintf(stdout, outputDataWsenPads);

            /* transmit sensor data */
            sprintf(ascii_data, "%s %s %s %s \n", rfDataLIS2DW, rfDataLPS22HB, rfDataHTS221, rfDataWsenPads);
            ret = TarvosIII_Transmit((uint8_t*)ascii_data,strlen(ascii_data));
        }
    }

    ret = TarvosIII_Deinit();
    Debug_out("TarvosIII_Deinit", ret);
}

/* test function to only stay on RX */
static void RX_test()
{
    bool ret = false;

    /* initialize the module TarvosIII */
    ret = TarvosIII_Init(115200, TarvosIII_PIN_RESET, TarvosIII_PIN_WAKEUP, TarvosIII_PIN_BOOT, RXcallback, AddressMode_0);
    Debug_out("TarvosIII_Init", ret);

    ret = TarvosIII_PinReset();
    Debug_out("PinReset", ret);
    delay(500);

    printf ("Waiting for incoming data\n");
    fflush (stdout) ;
    while(1)
    {
        /* do nothing, just stay on RX */
        delay(1000);
    }
}

/* test function to demonstrate the usage of the TarvosIII functions */
static void TarvosIII_test_function()
{
    bool ret = false;
    uint8_t serial_number[4];
    uint8_t driver_version[3];

    ret = GetDriverVersion(driver_version);
    Debug_out("GetDriverVersion",ret);
    if(ret)
    {
        fprintf (stdout, COLOR_CYAN "TarvosIII driver version %d.%d.%d\n" COLOR_RESET,driver_version[0],driver_version[1],driver_version[2]);
    }
    delay(500);

    /* initialize the module TarvosIII */
    ret = TarvosIII_Init(115200, TarvosIII_PIN_RESET, TarvosIII_PIN_WAKEUP, TarvosIII_PIN_BOOT, RXcallback, AddressMode_0);
    Debug_out("TarvosIII_Init", ret);

    if(ret)
    {

        ret = TarvosIII_PinReset();
        Debug_out("PinReset", ret);
        delay(500);

        ret = TarvosIII_GetSerialNumber(serial_number);
        Debug_out("GetSerialNumber",ret);
        if(ret)
        {
            uint32_t serial_number_dez = serial_number[1] << 16 | serial_number[2] << 8 | serial_number[3];
            fprintf (stdout, COLOR_CYAN "Serial number %02d.%06d ",serial_number[0],serial_number_dez);
            fprintf (stdout, "(0x%02x.%02x.%02x.%02x)\n" COLOR_RESET,serial_number[0],serial_number[1],serial_number[2],serial_number[3]);
        }
        delay(500);

#if 0
        ret = TarvosIII_FactoryReset();
        Debug_out("facReset", ret);
        delay(500);

        ret = TarvosIII_Standby();
        Debug_out("Standby", ret);
        delay(500);
#endif

        ret = TarvosIII_Shutdown();
        Debug_out("Shutdown", ret);
        delay(500);

        ret = TarvosIII_PinWakeup();
        Debug_out("Wakeup", ret);
        fprintf (stdout, "\n");
        delay(500);

        uint8_t data[6];
        uint8_t data_length = 6;
        ret = TarvosIII_Get(TarvosIII_CMD_SETGET_OPTION_DEFAULTRFTXPOWER, data, &data_length);
        Debug_out("Get TXpower", ret);
        fprintf (stdout,"TXPower is %d dBm\n",data[0]);
        delay(500);

        data[0] = 0; /* 0dbm */
        data_length = 1;
        ret = TarvosIII_Set(TarvosIII_CMD_SETGET_OPTION_DEFAULTRFTXPOWER, data, data_length);
        Debug_out("Set TXpower to 0 dBm", ret);
        delay(500);

        data[0] = 88;
        ret = TarvosIII_Get(TarvosIII_CMD_SETGET_OPTION_DEFAULTRFTXPOWER, data, &data_length);
        Debug_out("Get TXpower", ret);
        fprintf (stdout,"TXPower is %d dBm\n",data[0]);
        delay(500);

        ret = TarvosIII_SetVolatile_TXPower(10);
        Debug_out("Set volatile TXpower to 10 dBm", ret);
        delay(500);

        ret = TarvosIII_SetVolatile_Channel(110);
        Debug_out("Set volatile Channel to 110", ret);
        delay(500);

        ret = TarvosIII_SetVolatile_DestNetID(1);
        Debug_out("Set volatile DestNetID to 1", ret);
        delay(500);

        ret = TarvosIII_SetVolatile_DestAddr(0x00,0x00);
        Debug_out("Set volatile DestAddr to 0", ret);
        delay(500);

        fprintf (stdout, "\n");

        ret = TarvosIII_PinReset();
        Debug_out("PinReset", ret);
        delay(500);

        uint8_t txpower = 10;
        ret = TarvosIII_GetDefaultTXPower(&txpower);
        Debug_out("TarvosIII_GetDefaultTXPower", ret);
        fprintf (stdout,"TXPower is %d dBm\n",txpower);
        delay(500);
        ret = TarvosIII_SetDefaultTXPower(2);
        Debug_out("TarvosIII_SetDefaultTXPower to 2dBm", ret);
        delay(500);


        uint8_t channel = 0;
        ret = TarvosIII_GetDefaultRFChannel(&channel);
        Debug_out("TarvosIII_GetDefaultRFChannel", ret);
        fprintf (stdout,"RFChannel is %d\n",channel);
        delay(500);
        ret = TarvosIII_SetDefaultRFChannel(101);
        Debug_out("TarvosIII_SetDefaultRFChannel to 101", ret);
        delay(500);


        uint8_t srcnetid = 0;
        ret = TarvosIII_GetSourceNetID(&srcnetid);
        Debug_out("TarvosIII_GetSourceNetID", ret);
        fprintf (stdout,"Source net ID is %d\n",srcnetid);
        delay(500);
        ret = TarvosIII_SetSourceNetID(17);
        Debug_out("TarvosIII_SetSourceNetID to 17", ret);
        delay(500);

        uint8_t destnetid = 0;
        ret = TarvosIII_GetDefaultDestNetID(&destnetid);
        Debug_out("TarvosIII_GetDefaultDestNetID", ret);
        fprintf (stdout,"Destination net ID is 0x%x\n",destnetid);
        delay(500);
        ret = TarvosIII_SetDefaultDestNetID(0x12);
        Debug_out("TarvosIII_SetDefaultDestNetID to 0x12", ret);
        delay(500);



        uint8_t srcaddr_lsb = 0, srcaddr_msb = 0;
        ret = TarvosIII_GetSourceAddr(&srcaddr_lsb,&srcaddr_msb);
        Debug_out("TarvosIII_GetSourceAddr", ret);
        fprintf (stdout,"Source address is 0x%x%x\n",srcaddr_lsb,srcaddr_msb);
        delay(500);
        ret = TarvosIII_SetSourceAddr(0x13,0x14);
        Debug_out("TarvosIII_SetSourceAddr to 0x1314", ret);
        delay(500);

        uint8_t destaddr_lsb = 0, destaddr_msb = 0;
        ret = TarvosIII_GetDefaultDestAddr(&destaddr_lsb,&destaddr_msb);
        Debug_out("TarvosIII_GetDefaultDestAddr", ret);
        fprintf (stdout,"Destination address is 0x%x%x\n",destaddr_lsb,destaddr_msb);
        delay(500);
        ret = TarvosIII_SetDefaultDestAddr(0x20,0x21);
        Debug_out("TarvosIII_SetDefaultDestAddr to 0x2021", ret);
        delay(500);

        uint8_t profile = 0;
        ret = TarvosIII_GetDefaultRFProfile(&profile);
        Debug_out("TarvosIII_GetDefaultRFProfile", ret);
        fprintf (stdout,"RF profile is %d\n",profile);
        delay(500);
        ret = TarvosIII_SetDefaultRFProfile(2);
        Debug_out("TarvosIII_SetDefaultRFProfile to 2", ret);
        delay(500);

        ret = TarvosIII_PinReset();
        Debug_out("TarvosIII_PinReset", ret);
        delay(500);

        fprintf(stdout, "\n");

        txpower = 88;
        ret = TarvosIII_GetDefaultTXPower(&txpower);
        Debug_out("TarvosIII_GetDefaultTXPower", ret);
        fprintf (stdout,"TXPower is %d dBm\n",txpower);
        fprintf (stdout, "\n");
        delay(500);

        ret = TarvosIII_GetDefaultRFChannel(&channel);
        Debug_out("TarvosIII_GetDefaultRFChannel", ret);
        fprintf (stdout,"RFChannel is %d\n",channel);
        fprintf (stdout, "\n");
        delay(500);

        ret = TarvosIII_GetSourceNetID(&srcnetid);
        Debug_out("TarvosIII_GetSourceNetID", ret);
        fprintf (stdout,"Source net ID is %d\n",srcnetid);
        fprintf (stdout, "\n");
        delay(500);

        ret = TarvosIII_GetDefaultDestNetID(&destnetid);
        Debug_out("TarvosIII_GetDefaultDestNetID", ret);
        fprintf (stdout,"Destination net ID is 0x%x\n",destnetid);
        fprintf (stdout, "\n");
        delay(500);

        ret = TarvosIII_GetSourceAddr(&srcaddr_lsb,&srcaddr_msb);
        fprintf (stdout,"Source address is 0x%x%x\n",srcaddr_lsb,srcaddr_msb);
        Debug_out("TarvosIII_GetSourceAddr", ret);
        fprintf (stdout, "\n");
        delay(500);

        ret = TarvosIII_GetDefaultDestAddr(&destaddr_lsb,&destaddr_msb);
        Debug_out("TarvosIII_GetDefaultDestAddr", ret);
        fprintf (stdout,"Destination address is 0x%x%x\n",destaddr_lsb,destaddr_msb);
        fprintf (stdout, "\n");
        delay(500);

        ret = TarvosIII_GetDefaultRFProfile(&profile);
        Debug_out("TarvosIII_GetDefaultRFProfile", ret);
        fprintf (stdout,"RF profile is %d\n",profile);
        fprintf (stdout, "\n");
        delay(500);

        ret = TarvosIII_FactoryReset();
        Debug_out("TarvosIII_FactoryReset", ret);
        delay(500);

        ret = TarvosIII_GetDefaultTXPower(&txpower);
        Debug_out("TarvosIII_GetDefaultTXPower", ret);
        fprintf (stdout,"TXPower is %d dBm\n",txpower);
        fprintf (stdout, "\n");
        delay(500);

        // Transmit "Raspberry Alive"
        uint8_t payload[15] = { 'R','a','s','p','b','e','r','r','y',' ','A','l','i','v','e' };
        ret = TarvosIII_Transmit(payload,sizeof(payload));
        Debug_out("Transmit", ret);
        delay(500);

        // Transmit "Raspberry Alive" on channel 106 to address 0 0 0
        ret = TarvosIII_Transmit_Extended(payload,sizeof(payload),106,0,0,0);
        Debug_out("Transmit extended", ret);
        delay(500);
    }
    ret = TarvosIII_Deinit();
    Debug_out("TarvosIII_Deinit", ret);
}

