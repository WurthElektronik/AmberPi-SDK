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

#include <errno.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include "string.h"
#include <stdio.h>

#include "../../drivers/WE-common.h"
#include "global.h"


#ifndef _global_termios_defined
#define _global_termios_defined
#ifdef _global_serial_defined
#error "second interface defined"
#elifdef _global_ftdi_defined
#error "second interface defined"
#endif

/**************************************
 *     Static function declarations   *
 **************************************/

static bool ConfigureBaudrate(struct termios *serialConfigurationP, int baudrate);
static bool ConfigureParity(struct termios *serialConfigurationP, Serial_ParityBit_t parity);
static bool SetPortAttributes(int serial_handle, int baudrate, Serial_ParityBit_t parity);

/**************************************
 *          Static variables          *
 **************************************/
int serial_handle = -1;
const char *serial_device = "/dev/serial0";

/**************************************
 *         Global functions           *
 **************************************/

bool SetPin( int pin_number, SetPin_InputOutput_t inout, SetPin_Pull_t pull, SetPin_Out_t out)
{
    return true;
}

bool SendBytes(uint8_t* dataP, uint16_t length)
{
    uint16_t bytesWritten = 0;

    if(-1 == serial_handle)
    {
        /* invalid serial_handle*/
        return false;
    }

    bytesWritten = write(serial_handle, dataP, length);
    if (bytesWritten != length) {
        printf("Error from write: %d, %d\n", bytesWritten, errno);
        return false;
    }

    return true;
}

bool BytesAvailable()
{
    int bytesInBuffer = 0;

    if(-1 == serial_handle)
    {
        /* invalid handle */
        return false;
    }

    ioctl(serial_handle, FIONREAD, &bytesInBuffer);

    return (0 < bytesInBuffer);
}

bool ReadByte(uint8_t *readBufferP)
{
    int bytesRead = 0;

    if(-1 == serial_handle)
    {
        return false;
    }

    read(serial_handle, readBufferP, 1);

    /* no bytes were read */
    if(bytesRead < 0)
    {
        return false;
    }

    return true;
}

bool CloseSerial()
{
    close(serial_handle);
    serial_handle = -1;

    return true;
}

bool FlushSerial()
{
    return false;

}


bool InitPin(int pin_number)
{
    return true;
}

bool DeinitPin(int pin_number)
{
    return true;
}

bool InitSerial()
{
    return true;
}

bool DeinitSerial()
{
    return true;
}

bool OpenSerial(int baudrate)
{
    /* 8n1 as default configuration*/
    return OpenSerialWithParity(baudrate, Serial_ParityBit_NONE);
}

bool OpenSerialWithParity(int baudrate, Serial_ParityBit_t parityBit)
{
    serial_handle = open(serial_device, O_RDWR | O_NOCTTY | O_SYNC);
    if(serial_handle < 0)
    {
        printf("Could not open serial device. Failed with error: %s\n", strerror(errno));
        return false;
    }

    if(false == SetPortAttributes(serial_handle, baudrate, parityBit))
    {
        return false;
    }

    return true;
}

/**************************************
 *         Static functions           *
 **************************************/

static bool ConfigureBaudrate(struct termios *serialConfigurationP, int baudrate)
{
    int baudrateToSet = -1;

    if(NULL == serialConfigurationP)
    {
        return false;
    }

    switch(baudrate)
    {
    case 1200:
    {
        baudrateToSet = B1200;
        break;
    }

    case 2400:
    {
        baudrateToSet = B2400;
        break;
    }

    case 4800:
    {
        baudrateToSet = B4800;
        break;
    }

    case 9600:
    {
        baudrateToSet = B9600;
        break;
    }

    case 19200:
    {
        baudrateToSet = B19200;
        break;
    }

    case 38400:
    {
        baudrateToSet = B38400;
        break;
    }

    case 57600:
    {
        baudrateToSet = B57600;
        break;
    }

    case 115200:
    {
        baudrateToSet = B115200;
        break;
    }

    case 230400:
    {
        baudrateToSet = B230400;
        break;
    }

    case 460800:
    {
        baudrateToSet = B460800;
        break;
    }

    case 921600:
    {
        baudrateToSet = B921600;
        break;
    }

    case 1000000:
    {
        baudrateToSet = B1000000;
        break;
    }

    default:
    {
        /* invalid baudrate */
        return false;
    }
    }

    cfsetospeed(serialConfigurationP, baudrateToSet);
    cfsetispeed(serialConfigurationP, baudrateToSet);

    return true;
}

static bool ConfigureParity(struct termios *serialConfigurationP, Serial_ParityBit_t parity)
{
    if(NULL == serialConfigurationP)
    {
        return false;
    }

    switch(parity)
    {
        case Serial_ParityBit_NONE:
        {
            serialConfigurationP->c_cflag &= ~PARENB;
            serialConfigurationP->c_cflag &= ~CSTOPB;
            serialConfigurationP->c_cflag &= ~CSIZE;
            serialConfigurationP->c_cflag |= CS8;
            break;
        }

        case Serial_ParityBit_EVEN:
        {
            serialConfigurationP->c_cflag |= PARENB;
            serialConfigurationP->c_cflag &= ~PARODD;
            serialConfigurationP->c_cflag &= ~CSTOPB;
            serialConfigurationP->c_cflag &= ~CSIZE;
            serialConfigurationP->c_cflag |= CS7;
            break;
        }

        case Serial_ParityBit_ODD:
        {
            serialConfigurationP->c_cflag |= PARENB;
            serialConfigurationP->c_cflag |= PARODD;
            serialConfigurationP->c_cflag &= ~CSTOPB;
            serialConfigurationP->c_cflag &= ~CSIZE;
            serialConfigurationP->c_cflag |= CS7;
            break;
        }

        default:
        {
            return false;
        }
    }

    return true;
}

static bool SetPortAttributes(int serial_handle, int baudrate, Serial_ParityBit_t parity)
{
    if(-1 == serial_handle)
    {
        /* serial_handle not open */
        return false;
    }

    struct termios tty;
    /* get current attributes/configuration*/
    if (tcgetattr (serial_handle, &tty) != 0)
    {
        printf("error %d from tcgetattr", errno);
        return false;
    }

    if(false == ConfigureBaudrate(&tty, baudrate))
    {
        /* invalid baudrate  */
        printf("Could not set baudrate: %i", baudrate);
        return false;
    }

    if(false == ConfigureParity(&tty, parity))
    {
        /* invalid parity*/
        printf("Could not set parity bit");
        return false;
    }

    tty.c_cflag |= (CLOCAL | CREAD);    /* ignore modem controls */
    tty.c_cflag &= ~CRTSCTS;    /* no hardware flowcontrol */

    /* setup for non-canonical mode */
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
    tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN); /* raw input - received as is*/
    tty.c_oflag &= ~OPOST; /* raw output - transmitted as is */

    /* Set attributes/configurations*/
    if (tcsetattr (serial_handle, TCSANOW, &tty) != 0)
    {
        printf("error %d from tcsetattr", errno);
        return false;
    }

    return true;
}

#endif // _global_termios_defined
