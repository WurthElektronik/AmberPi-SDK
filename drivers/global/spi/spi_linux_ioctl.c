/**
 ***************************************************************************************************
 * This file is part of WE sensors SDK:
 * https://www.we-online.com/sensors
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
 * COPYRIGHT (c) 2020 Würth Elektronik eiSos GmbH & Co. KG
 *
 ***************************************************************************************************
 **/

#include "platform_spi.h"
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include "linux/spi/spidev.h"

#define DELAY         0

static int spi_handle = 0; // global handle to the  i2c interface

static int spi_bitsPerWords = 8;
static int spi_speed  = 0;

/**
* @brief  Initialize the SPI Interface
* @param  speed
* @param  mode
* @retval Error Code
*/
bool SPI_Init(int channel, int speed, int mode)
{
    char spiDevice[32];
    snprintf (spiDevice, 31, "/dev/spidev0.%d", channel) ;

    if((spi_handle = open(spiDevice, O_RDWR)) < 0)
    {
        printf("Could not open spi device");
        return false;
    }

    spi_speed = speed;
    mode &= 3;

    if(ioctl(spi_handle, SPI_IOC_WR_MODE, &mode) < 0)
    {
        printf("Could not set spi mode");
        return false;
    }
    if(ioctl(spi_handle, SPI_IOC_WR_BITS_PER_WORD, &spi_bitsPerWords) < 0)
    {
        printf("Could not set spi bitsPerWords");
        return false;
    }
    if(ioctl(spi_handle, SPI_IOC_WR_MAX_SPEED_HZ, &spi_speed) < 0)
    {
        printf("Could not set spi speed");
        return false;
    }

    return true;
}


bool SPI_ReadWrite(uint8_t *dataP, int numByte)
{
    struct spi_ioc_transfer spi ;

    memset (&spi, 0, sizeof (spi)) ;

    spi.tx_buf        = (unsigned long)dataP ;
    spi.rx_buf        = (unsigned long)dataP ;
    spi.len           = numByte ;
    spi.delay_usecs   = DELAY ;
    spi.speed_hz      = spi_speed;
    spi.bits_per_word = spi_bitsPerWords ;

    if(ioctl (spi_handle, SPI_IOC_MESSAGE(1), &spi) < 0)
    {
        fprintf(stdout, "Could not read/write SPI data");
        return false;
    }

    return true;
}
/**         EOF         */
