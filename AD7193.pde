/***************************************************************************//**
 *   @file   AD7193 sketch.
 *   @brief  Demo project for AD7193 using MPIDE Serial Monitor.
 *   @author Dan Nechita
********************************************************************************
 * Copyright 2012(c) Analog Devices, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *  - Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  - Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  - Neither the name of Analog Devices, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *  - The use of this software may or may not infringe the patent rights
 *    of one or more patent holders.  This license does not release you
 *    from the requirement that you obtain separate licenses from these
 *    patent holders to use this software.
 *  - Use of the software either in source or binary form, must be run
 *    on or directly connected to an Analog Devices Inc. component.
 *
 * THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT,
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
********************************************************************************
 *   SVN Revision: 699
*******************************************************************************/

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include <AD7193.h>
#include <DSPI.h>

/***************************************************************************//**

 *   @file   AD7193.c

 *   @brief  Implementation of AD7193 Driver.

 *   @author Dan Nechita

********************************************************************************

 * Copyright 2012(c) Analog Devices, Inc.

 *

 * All rights reserved.

 *

 * Redistribution and use in source and binary forms, with or without

 * modification, are permitted provided that the following conditions are met:

 *  - Redistributions of source code must retain the above copyright

 *    notice, this list of conditions and the following disclaimer.

 *  - Redistributions in binary form must reproduce the above copyright

 *    notice, this list of conditions and the following disclaimer in

 *    the documentation and/or other materials provided with the

 *    distribution.

 *  - Neither the name of Analog Devices, Inc. nor the names of its

 *    contributors may be used to endorse or promote products derived

 *    from this software without specific prior written permission.

 *  - The use of this software may or may not infringe the patent rights

 *    of one or more patent holders.  This license does not release you

 *    from the requirement that you obtain separate licenses from these

 *    patent holders to use this software.

 *  - Use of the software either in source or binary form, must be run

 *    on or directly connected to an Analog Devices Inc. component.

 *

 * THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR

 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT,

 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.

 * IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT,

 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT

 * LIMITED TO, INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR

 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER

 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,

 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE

 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 *

********************************************************************************

 *   SVN Revision: 699

*******************************************************************************/



/******************************************************************************/

/***************************** Include Files **********************************/

/******************************************************************************/

#include "AD7193.h"	



/******************************************************************************/

/************************ Functions Definitions *******************************/

/******************************************************************************/



/***************************************************************************//**

 * @brief Checks if the AD7139 part is present.

 *

 * @return status - Indicates if the part is present or not.

*******************************************************************************/

unsigned char AD7193::Init(void)

{

    unsigned char status = 1;

    unsigned char regVal = 0;

    

    /*! Initialize class members. */

	mGain = 1;     /*!<  Gain = 1. Vref = 2.5V. Analong input range = +-2.5V */

	mPolarity = 0; /*!<  Bipolar mode */

	mVref = 3.3;   /*!<  Vref = 2.5V on PmodAD5 */

	/*! Initialize SPI communication. */

	oDspi.begin(AD7193_CS_PIN);

	oDspi.setMode(DSPI_MODE3);

	oDspi.setSpeed(DEVICE_SPI_CLK_FREQ);

    Reset();

    regVal = GetRegisterValue(AD7193_REG_ID, 1, 1);

    if( (regVal & AD7193_ID_MASK)  != ID_AD7193)

    {

        status = 0;

    }

	

    return(status);

}



/***************************************************************************//**

 * @brief Writes data into a register.

 *

 * @param registerAddress - Address of the register.

 * @param registerValue - Data value to write.

 * @param bytesNumber - Number of bytes to be written.

 * @param modifyCS - Allows Chip Select to be modified.

 *

 * @return none.

*******************************************************************************/

void AD7193::SetRegisterValue(unsigned char registerAddress,

                              unsigned long registerValue,

                              unsigned char bytesNumber,

                              unsigned char modifyCS)

{

    unsigned char commandByte = 0;

	unsigned char txBuffer[4] = {0, 0, 0 ,0};



    commandByte = AD7193_COMM_WRITE | AD7193_COMM_ADDR(registerAddress);

	txBuffer[0] = (registerValue >> 0)  & 0x000000FF;

	txBuffer[1] = (registerValue >> 8)  & 0x000000FF;

	txBuffer[2] = (registerValue >> 16) & 0x000000FF;

	txBuffer[3] = (registerValue >> 24) & 0x000000FF;

	if(modifyCS == 1)

	{

		digitalWrite(AD7193_CS_PIN, LOW);

	}

	oDspi.transfer(commandByte);

    while(bytesNumber > 0)

    {

        oDspi.transfer(txBuffer[bytesNumber - 1]);

        bytesNumber--;

    }

	if(modifyCS == 1)

	{

		digitalWrite(AD7193_CS_PIN, HIGH);

	}

}



/***************************************************************************//**

 * @brief Reads the value of a register.

 *

 * @param registerAddress - Address of the register.

 * @param bytesNumber - Number of bytes that will be read.

 * @param modifyCS    - Allows Chip Select to be modified.

 *

 * @return buffer - Value of the register.

*******************************************************************************/

unsigned long AD7193::GetRegisterValue(unsigned char registerAddress,

                                       unsigned char bytesNumber,

                                       unsigned char modifyCS)

{

    unsigned char receiveBuffer = 0;

	unsigned char writeByte = 0;

    unsigned char byteIndex = 0;

	unsigned long buffer = 0;

    

    writeByte = AD7193_COMM_READ | AD7193_COMM_ADDR(registerAddress);

    if(modifyCS == 1)

	{

		digitalWrite(AD7193_CS_PIN, LOW);

	}

	oDspi.transfer(writeByte);

    while(byteIndex < bytesNumber)

    {

        receiveBuffer = oDspi.transfer(0);

		buffer = (buffer << 8) + receiveBuffer;

        byteIndex++;

    }

    if(modifyCS == 1)

	{

		digitalWrite(AD7193_CS_PIN, HIGH);

	}

	

    return(buffer);

}



/***************************************************************************//**

 * @brief Resets the device.

 *

 * @return none.

*******************************************************************************/

void AD7193::Reset(void)

{

    char i = 0;

    

	for(i = 0; i < 6; i++)

	{

		oDspi.transfer(0xFF);

	}    

}



/***************************************************************************//**

 * @brief Selects the AD7193's operating mode.

 *

 * @param mode - Operating mode.

 *               Example: AD7193_MODE_CONT - Continuous Conversion Mode.

 *                        AD7193_MODE_SINGLE - Single Conversion Mode.

 *                        AD7193_MODE_IDLE - Idle Mode.

 *                        AD7193_MODE_PWRDN - Power-Down Mode.

 *

 * @return none.

*******************************************************************************/

void AD7193::SetMode(unsigned char mode)

{

    unsigned long oldRegValue = 0;

    unsigned long newRegValue = 0;



    oldRegValue = GetRegisterValue(AD7193_REG_MODE, 3, 1);

    oldRegValue &= ~AD7193_MODE_SEL(0x7);

    newRegValue = oldRegValue | AD7193_MODE_SEL(mode);

    SetRegisterValue(AD7193_REG_MODE,

                     newRegValue,

                     3,

                     1);

}



/***************************************************************************//**

 * @brief Waits for RDY pin to go low.

 *

 * @return none.

*******************************************************************************/

void AD7193::WaitRdyGoLow(void)

{

    while( digitalRead(AD7193_RDY_STATE) )

    {

        ;

    }

}



/***************************************************************************//**

 * @brief Selects the channel to be enabled.

 *

 * @param channel - Selects a channel.

 *                  Example: AD7193_CH_0 - AIN1(+) - AIN2(-);  (Pseudo = 0)

 *                           AD7193_CH_1 - AIN3(+) - AIN4(-);  (Pseudo = 0)

 *                           AD7193_TEMP - Temperature sensor

 *                           AD7193_SHORT - AIN2(+) - AIN2(-); (Pseudo = 0)

 *  

 * @return none.

*******************************************************************************/

void AD7193::ChannelSelect(unsigned short channel)

{

    unsigned long oldRegValue = 0x0;

    unsigned long newRegValue = 0x0;

     

    oldRegValue = GetRegisterValue(AD7193_REG_CONF, 3, 1);

    oldRegValue &= ~(AD7193_CONF_CHAN(0x3F));

    newRegValue = oldRegValue | AD7193_CONF_CHAN(1 << channel);   

    SetRegisterValue(AD7193_REG_CONF, newRegValue, 3, 1);

}



/***************************************************************************//**

 * @brief Performs the given calibration to the specified channel.

 *

 * @param mode - Calibration type.

 * @param channel - Channel to be calibrated.

 *

 * @return none.

*******************************************************************************/

void AD7193::Calibrate(unsigned char mode, unsigned char channel)

{

    unsigned long oldRegValue = 0x0;

    unsigned long newRegValue = 0x0;

    

    ChannelSelect(channel);

    oldRegValue = GetRegisterValue(AD7193_REG_MODE, 3, 1);

    oldRegValue &= ~AD7193_MODE_SEL(0x7);

    newRegValue = oldRegValue | AD7193_MODE_SEL(mode);

    digitalWrite(AD7193_CS_PIN, LOW); 

    SetRegisterValue(AD7193_REG_MODE, newRegValue, 3, 0); /*!< CS is not modified. */

    WaitRdyGoLow();

    digitalWrite(AD7193_CS_PIN, HIGH);

}



/***************************************************************************//**

 * @brief Selects the polarity of the conversion and the ADC input range.

 *

 * @param polarity - Polarity select bit. 

                     Example: 0 - bipolar operation is selected.

                              1 - unipolar operation is selected.

* @param range - Gain select bits. These bits are written by the user to select 

                 the ADC input range.     

 *

 * @return none.

*******************************************************************************/

void AD7193::RangeSetup(unsigned char polarity, unsigned char range)

{

    unsigned long oldRegValue = 0x0;

    unsigned long newRegValue = 0x0;

    

    oldRegValue = GetRegisterValue(AD7193_REG_CONF,3, 1);

    oldRegValue &= ~(AD7193_CONF_UNIPOLAR |

                     AD7193_CONF_GAIN(0x7));

    newRegValue = oldRegValue | 

                  (polarity * AD7193_CONF_UNIPOLAR) |

                  AD7193_CONF_GAIN(range); 

    SetRegisterValue(AD7193_REG_CONF, newRegValue, 3, 1);

	/*! Update class members. These values are used to convert raw data to voltage. */

	mGain = range;

	mPolarity = polarity;

}



/***************************************************************************//**

 * @brief Returns the result of a single conversion.

 *

 * @return regData - Result of a single analog-to-digital conversion.

*******************************************************************************/

unsigned long AD7193::SingleConversion(void)

{

    unsigned long command = 0x0;

    unsigned long regData = 0x0;



    command = AD7193_MODE_SEL(AD7193_MODE_SINGLE) | 

              AD7193_MODE_CLKSRC(AD7193_CLK_INT) |

              AD7193_MODE_RATE(0x060);

    digitalWrite(AD7193_CS_PIN, LOW);

    SetRegisterValue(AD7193_REG_MODE, command, 3, 0); /*!< CS is not modified. */

	WaitRdyGoLow();

    regData = GetRegisterValue(AD7193_REG_DATA, 3, 0);

    digitalWrite(AD7193_CS_PIN, HIGH);

    

    return(regData);

}



/***************************************************************************//**

 * @brief Returns the average of several conversion results.

 *

 * @return samplesAverage - The average of the conversion results.

*******************************************************************************/

unsigned long AD7193::ContinuousReadAvg(unsigned char sampleNumber)

{

    unsigned long samplesAverage = 0x0;

    unsigned char count = 0x0;

    unsigned long command = 0x0;

    

    command = AD7193_MODE_SEL(AD7193_MODE_CONT) | 

              AD7193_MODE_CLKSRC(AD7193_CLK_INT) |

              AD7193_MODE_RATE(0x060);

    digitalWrite(AD7193_CS_PIN, LOW);

    SetRegisterValue(AD7193_REG_MODE, command, 3, 0); /*!< CS is not modified. */

    for(count = 0;count < sampleNumber;count ++)

    {

        WaitRdyGoLow();

        samplesAverage += GetRegisterValue(AD7193_REG_DATA, 3, 0); /*!< CS is not modified. */

    }

    digitalWrite(AD7193_CS_PIN, HIGH);

    samplesAverage = samplesAverage / sampleNumber;

    

    return(samplesAverage);

}



/***************************************************************************//**

 * @brief Read data from temperature sensor and converts it to Celsius degrees.

 *

 * @return temperature - Celsius degrees.

*******************************************************************************/

unsigned long AD7193::TemperatureRead(void)

{

    unsigned char temperature = 0x0;

    unsigned long dataReg = 0x0;

    unsigned long oldRegValue = 0x0;

    unsigned long oldConfReg = 0x0;

	

    /*! Get the current configuration in order to restore it later. */

	oldConfReg = GetRegisterValue(AD7193_REG_CONF, 3, 1);

	RangeSetup(0, AD7193_CONF_GAIN_1); /*!< Bipolar operation, 0 Gain. */

    oldRegValue = GetRegisterValue(AD7193_REG_MODE, 3, 1);

	ChannelSelect(AD7193_CH_TEMP);

    dataReg = SingleConversion();

    dataReg -= 0x800000;

    dataReg /= 2815;   /*!< Kelvin Temperature */

    dataReg -= 273;    /*!< Celsius Temperature */

    temperature = (unsigned long) dataReg;

    SetRegisterValue(AD7193_REG_MODE, oldRegValue, 3, 1);

    /*! Set back the old configuration. */

	SetRegisterValue(AD7193_REG_CONF, oldConfReg, 3, 1);

    

	return(temperature);

}



/***************************************************************************//**

 * @brief Converts raw data read from ADC to a voltage value.

 *

 * @param rawData - data read from the ADC register.

 *

 * @return voltage - Value of the measured voltage.

*******************************************************************************/

double AD7193::BinaryToVoltage(long rawData)

{
	double voltage = 0;
	if(mPolarity == 1)
	{
		voltage = ((double)rawData / (1ul << 24) / (1 << mGain)) * mVref;
	}
	if(mPolarity == 0)
	{
		voltage = (((double)rawData / (1ul << 23)) - 1) * (mVref / (1 << mGain));
	}
	return(voltage);
}



/******************************************************************************/
/************************ Constants Definitions *******************************/
/******************************************************************************/
/*! List of available commands */
const char* commandsList[] =
    {"help?",
     "reset=",
     "mode?",
     "mode=",
     "pseudo?",
     "pseudo=",
     "channel?",
     "channel=",
     "data?",
     "voltage?",
     "temperature?"};
const char* commandsDescription[] =
    {"  -  Displays all available commands.",
     "  -  Resets the AD7193.",
     "  -  Displays the selected operating mode.",
     "  -  Selects the AD7193's operating mode. Accepted values: 0 - 3.",
     "  -  Displays the Pseudo Bit value (AD7193_REG_CONF).",
     "  -  Sets the Pseudo Bit Value (AD7193_REG_CONF). Accepted values: 0, 1.",
     "  -  Displays the enabled channel.",
     "  -  Enables one channel on the AD7193. Accepted values: 0 ? 7.",
     "  -  Displays the value of the data register (AD7193_REG_DATA).",
     "  -  Displays the voltage applied to enabled channel.",
     "  -  Displays the temperature."};

/******************************************************************************/
/************************ Variables Definitions *******************************/
/******************************************************************************/
AD7193 myAD7193;
/*! Variables used for console operations */
char   commandsNumber = (sizeof(commandsList) / sizeof(const char*));
char   receivedCommand[20];
char   invalidCommand = 0;
char   commandType = 0;
char   command = 0;
char   displayCommand = 0;
double commandParam = 0;
/*! Variables holding information about the device */
unsigned char channel = 0;      /*!< Enabled channel */
unsigned char pseudoBit = 1;    /*!< Pseudo Bit value */
/*! Temporary variables */
unsigned long data = 0;
unsigned long regValue = 0;
unsigned char mode = 0;
char          tempString [10]  = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};           
double        voltage = 0;

/***************************************************************************//**
 * @brief Reads one command from UART.
 *
 * @param command - Read command.
 *
 * @return None.
*******************************************************************************/
void CONSOLE_GetCommand(char* command)

{
    unsigned char receivedChar = 0;
    
    unsigned char charNumber = 0;
    
    while(receivedChar != 0x0D)
    
    {
      if (Serial.available() > 0)
      {
        
        receivedChar = Serial.read();
        command[charNumber++] = receivedChar; 
      }
    }
    command[charNumber] = 0;
}

/***************************************************************************//** 
 * @param receivedCommand - Received command.
 * @param expectedCommand - Expected command.
 * @param commandParameter - Command parameter.
 *
 * @return commandType - Type of the command.
 *                       Example: 0 - Commands don't match.
 *                                1 - Write command.
 *                                2 - Read command.
*******************************************************************************/
unsigned char CONSOLE_CheckCommands(char* receivedCommand,
                                    const char* expectedCommand,
                                    double* commandParameter)
{
    unsigned char commandType = 1;
    unsigned char charIndex = 0;

    unsigned char parameterString[5] = {0, 0, 0, 0, 0};
    unsigned char parameterIndex = 0;

    while((expectedCommand[charIndex] != '?') &&
          (expectedCommand[charIndex] != '=') &&
          
          (commandType != 0))
    {
        if(expectedCommand[charIndex] != receivedCommand[charIndex])
        {
            commandType = 0;
        }
        charIndex++;
    }
    if(commandType != 0)
    {
        if(expectedCommand[charIndex] == '=')
        {
            if(receivedCommand[charIndex] == '=')
            {
                charIndex++;
     
                while(receivedCommand[charIndex] != 0x0D)
                {
                    parameterString[parameterIndex] = receivedCommand[charIndex];
                    charIndex++;
                    parameterIndex++;
                    
                }
                *commandParameter = atof((const char*)parameterString);
            }
            else
            {
                commandType = 0;
            }
        }
        if(expectedCommand[charIndex] == '?')
        {
            if(receivedCommand[charIndex] == '?')
            {
                    commandType = 2;
            }
            else
            {
                    commandType = 0;
            }
        }
    }

    return commandType;
}

/***************************************************************************//**
 * @brief Setup function.
 *
 * @return none.
*******************************************************************************/    


float convertToVolts(unsigned long data, float vRef)
{
  int maxData = 1ul << 24;
  return( vRef*((float)data) / maxData);
}

void setup()
{
  #define CHANNEL AD7193_CH_0
//  #define CHANNEL_NAME #CHANNEL#
  Serial.begin(9600);
  Serial.println("DAQ.cerebot AD7193 v0.1.2.CH1");
  delay(5000);// wait a while so AD7193 startup
  if(myAD7193.Init())
  {
      Serial.println("AD7193 OK");
  }
  else
  {
      Serial.println("AD7193 Error");
  }
  /*! Resets the device. */
    myAD7193.Reset();
    /*! Select Channel 0 */
    myAD7193.ChannelSelect(CHANNEL);
    /*! Calibrates channel 0. */
    myAD7193.Calibrate(AD7193_MODE_CAL_INT_ZERO, CHANNEL);
    /*! Selects unipolar operation and ADC's input range to +-2.5V. */
    myAD7193.RangeSetup(0, AD7193_CONF_GAIN_1);
    regValue = myAD7193.GetRegisterValue(AD7193_REG_CONF, 3, 1);  
    regValue |= AD7193_CONF_PSEUDO ;
    myAD7193.SetRegisterValue(AD7193_REG_CONF, regValue, 3, 1);
    Serial.println("OK");
}

/***************************************************************************//**
 * @brief Loop function.
 *
 * @return none.
*******************************************************************************/
void loop()
{

  unsigned long data = 0;
  data = myAD7193.ContinuousReadAvg(10);
  //data = AD7193_ContinuousReadAvg(1000);//return an average of 1000 convertion on selected channel
  float volt= myAD7193.BinaryToVoltage(data);// convert binary data to volt
  Serial.print("volt=");
  Serial.print(volt,DEC);// print the voltage of selected channel
  Serial.print("  data=");
  Serial.println(data,DEC);
  delay(10);// wait a while
}

