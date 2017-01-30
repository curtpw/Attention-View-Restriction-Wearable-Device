
/*

    Copyright (c) 2014 RedBearLab, All rights reserved.

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
    See the GNU Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
    EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
    MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
    IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
    CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
    TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
    SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

*/

#include "wire.h"
#include "nrf_soc.h"
#include "nrf_sdm.h"

//extern Serial pc;
/**********************************************************************
name :
function : return: 0--SUCCESS, 1--FAIL
**********************************************************************/
bool TwoWire::twi_master_clear_bus(void)
{
    bool bus_clear;
    uint32_t twi_state;
    uint32_t sck_pin_config;
    uint32_t sda_pin_config;
    
    twi_state = twi->ENABLE;
    twi->ENABLE = TWI_ENABLE_ENABLE_Disabled << TWI_ENABLE_ENABLE_Pos;
    
    sck_pin_config = NRF_GPIO->PIN_CNF[SCL_Pin]; 
    NRF_GPIO->PIN_CNF[SCL_Pin] = (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos) 
                                  | (GPIO_PIN_CNF_DRIVE_S0D1     << GPIO_PIN_CNF_DRIVE_Pos) 
                                  | (GPIO_PIN_CNF_PULL_Pullup    << GPIO_PIN_CNF_PULL_Pos)  
                                  | (GPIO_PIN_CNF_INPUT_Connect  << GPIO_PIN_CNF_INPUT_Pos) 
                                  | (GPIO_PIN_CNF_DIR_Output     << GPIO_PIN_CNF_DIR_Pos);

    sda_pin_config = NRF_GPIO->PIN_CNF[SDA_Pin];    
    NRF_GPIO->PIN_CNF[SDA_Pin] = (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos) 
                                  | (GPIO_PIN_CNF_DRIVE_S0D1     << GPIO_PIN_CNF_DRIVE_Pos) 
                                  | (GPIO_PIN_CNF_PULL_Pullup    << GPIO_PIN_CNF_PULL_Pos)  
                                  | (GPIO_PIN_CNF_INPUT_Connect  << GPIO_PIN_CNF_INPUT_Pos) 
                                  | (GPIO_PIN_CNF_DIR_Output     << GPIO_PIN_CNF_DIR_Pos);  
                                  
    NRF_GPIO->OUTSET = ( 1 << SCL_Pin );
    NRF_GPIO->OUTSET = ( 1 << SDA_Pin );
    TWI_DELAY(4);
    
    if( ( (NRF_GPIO->IN >> SCL_Pin) & 0X1UL ) && ( (NRF_GPIO->IN >> SDA_Pin) & 0x1UL ) )
    {
        bus_clear = 0;
    }
    else
    {
        uint_fast8_t index;
        bus_clear = 1;
        for( index=18; index--;)
        {
            NRF_GPIO->OUTCLR = ( 1 << SCL_Pin );
            TWI_DELAY(4);
            NRF_GPIO->OUTSET = ( 1 << SDA_Pin );
            TWI_DELAY(4);
            
            if( (NRF_GPIO->IN >> SDA_Pin) & 0x1UL == 1 )
            {
                bus_clear = 0;
                break;
            }
        }
    }
    
    NRF_GPIO->PIN_CNF[SCL_Pin] = sck_pin_config;
    NRF_GPIO->PIN_CNF[SDA_Pin]  = sda_pin_config;

    twi->ENABLE = twi_state;    
    
    return bus_clear;
}

/**********************************************************************
name :
function : return: 0--SUCCESS, 1--FAIL
**********************************************************************/
bool TwoWire::twi_master_init(void)
{   
    uint8_t softdevice_enabled;
    //uint32_t err_code = NRF_SUCCESS;

    NRF_GPIO->PIN_CNF[SCL_Pin] = (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos)
                                  | (GPIO_PIN_CNF_DRIVE_S0D1     << GPIO_PIN_CNF_DRIVE_Pos)
                                  | (GPIO_PIN_CNF_PULL_Pullup    << GPIO_PIN_CNF_PULL_Pos)
                                  | (GPIO_PIN_CNF_INPUT_Connect  << GPIO_PIN_CNF_INPUT_Pos)
                                  | (GPIO_PIN_CNF_DIR_Input      << GPIO_PIN_CNF_DIR_Pos);  
                                  
    NRF_GPIO->PIN_CNF[SDA_Pin] = (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos)
                                  | (GPIO_PIN_CNF_DRIVE_S0D1     << GPIO_PIN_CNF_DRIVE_Pos)
                                  | (GPIO_PIN_CNF_PULL_Pullup    << GPIO_PIN_CNF_PULL_Pos)
                                  | (GPIO_PIN_CNF_INPUT_Connect  << GPIO_PIN_CNF_INPUT_Pos)
                                  | (GPIO_PIN_CNF_DIR_Input      << GPIO_PIN_CNF_DIR_Pos);
                                  
    twi->EVENTS_RXDREADY = 0;
    twi->EVENTS_TXDSENT  = 0;
    twi->PSELSCL = SCL_Pin;
    twi->PSELSDA = SDA_Pin;
    twi->FREQUENCY = twi_frequency; //TWI_FREQUENCY_FREQUENCY_K100 << TWI_FREQUENCY_FREQUENCY_Pos;
    
    sd_softdevice_is_enabled(&softdevice_enabled);
    //APP_ERROR_CHECK(err_code);
    if (softdevice_enabled == 0)
    {   
        NRF_PPI->CH[7].EEP  =   (uint32_t)&twi->EVENTS_BB;
        NRF_PPI->CH[7].TEP  =   (uint32_t)&twi->TASKS_SUSPEND;
        NRF_PPI->CHEN &= ~(1 << 7);
    }
    else
    {
        sd_ppi_channel_assign(7, &twi->EVENTS_BB, &twi->TASKS_SUSPEND);
        //APP_ERROR_CHECK(err_code);
        sd_ppi_channel_enable_clr(1 << 7);
        //APP_ERROR_CHECK(err_code);
    }
    
    twi->ENABLE = TWI_ENABLE_ENABLE_Enabled << TWI_ENABLE_ENABLE_Pos;  
    
    return twi_master_clear_bus();
}
/**********************************************************************
name :
function : return: 0--SUCCESS, 1--FAIL
**********************************************************************/
uint8_t TwoWire::twi_master_write(uint8_t *data, uint8_t data_length, uint8_t issue_stop_condition)
{
    uint32_t timeout = MAX_TIMEOUT_LOOPS;
    
    if(data_length == 0)
    {
        return 1;
    }
    twi->TXD = *data++;
    twi->TASKS_STARTTX = 1;
    while(1)
    {   
        while( (twi->EVENTS_TXDSENT == 0) && (--timeout) );//&& (twi->EVENTS_ERROR == 0) );
        
        if( 0 == timeout )//|| twi->EVENTS_ERROR != 0)
        {   
            twi->EVENTS_ERROR = 0;
            twi->ENABLE     = TWI_ENABLE_ENABLE_Disabled << TWI_ENABLE_ENABLE_Pos; 
            twi->POWER        = 0; 
            TWI_DELAY(5);
            twi->POWER        = 1; 
            twi->ENABLE     = TWI_ENABLE_ENABLE_Enabled << TWI_ENABLE_ENABLE_Pos;
            
            twi_master_init();
            return 1;
        }
        twi->EVENTS_TXDSENT = 0;
        if( --data_length == 0)
        {   
            break;
        }
        
        twi->TXD = *data++;
    }
    if(issue_stop_condition)
    {   
        twi->EVENTS_STOPPED = 0;
        twi->TASKS_STOP     = 1;
        while(twi->EVENTS_STOPPED == 0)
        {
            //do nothing, wait for stop sequence is sent
        }       
    }
    return 0;
}
/**********************************************************************
name :
function : return: 0--SUCCESS, 1--FAIL
**********************************************************************/
uint8_t TwoWire::twi_master_read(uint8_t *data, uint8_t data_length, uint8_t issue_stop_condition)
{   
    uint8_t softdevice_enabled;
    uint32_t timeout = MAX_TIMEOUT_LOOPS;// err_code = NRF_SUCCESS;

    sd_softdevice_is_enabled(&softdevice_enabled);
    //APP_ERROR_CHECK(err_code);
    if( 0 == data_length )
    {
        return 1;
    }
    else if( 1== data_length )//&& issue_stop_condition == 1)
    {
        if (softdevice_enabled == 0)
        {   
            NRF_PPI->CH[7].EEP = (uint32_t)&twi->EVENTS_BB;
            NRF_PPI->CH[7].TEP = (uint32_t)&twi->TASKS_STOP;
            NRF_PPI->CHEN |= (1 << 7);
        }
        else
        {
            sd_ppi_channel_assign(7, &twi->EVENTS_BB, &twi->TASKS_STOP);
            //APP_ERROR_CHECK(err_code);
            sd_ppi_channel_enable_set(1 << 7);
            //APP_ERROR_CHECK(err_code);
        }
    }
    else
    {   
        if (softdevice_enabled == 0)
        {   
            NRF_PPI->CH[7].EEP = (uint32_t)&twi->EVENTS_BB;
            NRF_PPI->CH[7].TEP = (uint32_t)&twi->TASKS_SUSPEND;
            NRF_PPI->CHEN |= (1 << 7);
        }
        else
        {
            sd_ppi_channel_assign(7, &twi->EVENTS_BB, &twi->TASKS_SUSPEND);
            //APP_ERROR_CHECK(err_code);
            sd_ppi_channel_enable_set(1 << 7);
            //APP_ERROR_CHECK(err_code);          
        }
    }
    
    twi->EVENTS_RXDREADY = 0;
    twi->TASKS_STARTRX = 1;
    
    while(1)
    {   
        while( twi->EVENTS_RXDREADY == 0 && (--timeout) )
        {
            //do nothing, just wait
        }
    
        if( timeout == 0 )
        {   
            twi->EVENTS_ERROR = 0;
            twi->ENABLE       = TWI_ENABLE_ENABLE_Disabled << TWI_ENABLE_ENABLE_Pos; 
            twi->POWER        = 0; 
            TWI_DELAY(5);
            twi->POWER        = 1; 
            twi->ENABLE       = TWI_ENABLE_ENABLE_Enabled << TWI_ENABLE_ENABLE_Pos;
    
            twi_master_init();          
          
            return 1;           
        }
        
        twi->EVENTS_RXDREADY = 0;
        *data++ = twi->RXD;
    
        if( --data_length == 1 )
        {   
            if (softdevice_enabled == 0)
            {   
                //NRF_PPI->CH[7].EEP = (uint32_t)&twi->EVENTS_BB;
                NRF_PPI->CH[7].TEP = (uint32_t)&twi->TASKS_STOP;
            }
            else
            {
                sd_ppi_channel_assign(7, &twi->EVENTS_BB, &twi->TASKS_STOP);
                //APP_ERROR_CHECK(err_code);
            }
        }
        
        if( data_length == 0 )
        {   
            twi->TASKS_STOP = 1;
            break;
        }
        TWI_DELAY(20);
        twi->TASKS_RESUME = 1;
    }
    while( twi->EVENTS_STOPPED == 0 )
    {
        //do nothing
    }

    twi->EVENTS_STOPPED = 0;
    
    if (softdevice_enabled == 0)
    {       
        NRF_PPI->CHEN &= ~(1 << 7);
    }
    else
    {
        sd_ppi_channel_enable_clr( 1 << 7 );
        //APP_ERROR_CHECK(err_code);
    }
    return 0;
}

/**********************************************************************
name :
function : 
**********************************************************************/
TwoWire::TwoWire(NRF_TWI_Type *twi_use)
{
    twi = twi_use;
    
    RX_BufferIndex  = 0;
    RX_BufferLength = 0;
    TX_BufferIndex  = 0;
    TX_BufferLength = 0;
    
    Transform_Addr  = 0;
    
    twi_status      = UNINITIALIZED;
}
/**********************************************************************
name :
function : 
**********************************************************************/
void TwoWire::begin(uint32_t scl, uint32_t sda, uint8_t speed)
{     
    if( speed == 2 )
    {
        twi_frequency = TWI_FREQUENCY_FREQUENCY_K400;
    }
    else if( speed == 1 )
    {
        twi_frequency = TWI_FREQUENCY_FREQUENCY_K250;
    }
    else
    {
        twi_frequency = TWI_FREQUENCY_FREQUENCY_K100;   
    }
    
    SCL_Pin = scl;
    SDA_Pin = sda;
    twi_master_init();

    twi_status = MASTER_IDLE;   
}
/**********************************************************************
name :
function : 
**********************************************************************/
void TwoWire::begin()
{
    begin(TWI_SCL, TWI_SDA, 0);
}
/**********************************************************************
name :
function : 
**********************************************************************/
void TwoWire::beginTransmission( uint8_t address )
{
    Transform_Addr = address;
    TX_BufferIndex = 0;
    twi_status = MASTER_SEND;
}
/**********************************************************************
name :
function : 
**********************************************************************/
void TwoWire::beginTransmission( int address )
{
    beginTransmission( (uint8_t)address );
}
/**********************************************************************
name :
function :  return: 0--SUCCESS, 1--FAIL
**********************************************************************/
uint8_t TwoWire::endTransmission( uint8_t stop)
{
    uint8_t twi_flag = 1;

    if(TX_BufferLength > 0 && !(twi_master_clear_bus()) )
    {   
        twi->ADDRESS = ( Transform_Addr >> 1);
        twi_flag = twi_master_write(TX_Buffer, TX_BufferLength, stop);
    }
    
    TX_BufferLength = 0;
    twi_status = MASTER_IDLE;

    return twi_flag;
}
/**********************************************************************
name :
function : return: 0--SUCCESS, 1--FAIL
**********************************************************************/
uint8_t TwoWire::endTransmission(void)
{
    uint8_t twi_flag;
    
    twi_flag = endTransmission(1);
    
    return twi_flag;
}
/**********************************************************************
name :
function : return: 0--SUCCESS, -1--FAIL,
**********************************************************************/
int TwoWire::write(uint8_t data)
{
    if(twi_status == MASTER_SEND)
    {   
        if(TX_BufferLength >= BUFF_MAX_LENGTH)
        {
            return -1;
        }
        TX_Buffer[TX_BufferLength++] = data;
        return 0;
    }
    else
    {
        return -1;
    }
}
/**********************************************************************
name :
function : return: -1--FAIL, else--the length of data stored
**********************************************************************/
int TwoWire::write(const uint8_t *data, size_t quantity )
{
    if( twi_status == MASTER_SEND )
    {
        for( size_t i=0; i<quantity; ++i )
        {   
            if(TX_BufferLength >= BUFF_MAX_LENGTH)
            {
                return i;
            }
            TX_Buffer[TX_BufferLength++] = data[i];
        }
    }
    else
    {
        return -1;
    }
    
    return quantity;
}
/**********************************************************************
name :
function : return: 0--SUCCESS, 1--FAIL
**********************************************************************/
uint8_t TwoWire::requestFrom(uint8_t addr, uint8_t quantity, uint8_t stop)
{
    uint8_t twi_flag = 1;
    
    if(quantity > BUFF_MAX_LENGTH)
    {   
        quantity = BUFF_MAX_LENGTH;
    }
    if(quantity > 0 && !(twi_master_clear_bus()) )
    {   
        twi->ADDRESS = ( addr >> 1 );
        twi_flag = twi_master_read(RX_Buffer, quantity, stop);
    }
    RX_BufferIndex = 0;
    RX_BufferLength = quantity;
    
    return twi_flag;
}
/**********************************************************************
name :
function : 
**********************************************************************/
uint8_t TwoWire::requestFrom(uint8_t address, uint8_t quantity)
{
    return requestFrom((uint8_t) address, (uint8_t) quantity, (uint8_t) true);
}
/**********************************************************************
name :
function : 
**********************************************************************/
uint8_t TwoWire::requestFrom(int address, int quantity) 
{
    return requestFrom((uint8_t) address, (uint8_t) quantity, (uint8_t) true);
}
/**********************************************************************
name :
function : 
**********************************************************************/
uint8_t TwoWire::requestFrom(int address, int quantity, int sendStop) 
{
    return requestFrom((uint8_t) address, (uint8_t) quantity, (uint8_t) sendStop);
}
/**********************************************************************
name :
function : return:-1--FAIL, else:the length of data that could be read
**********************************************************************/
int TwoWire::available(void)
{
    if(RX_BufferIndex <= RX_BufferLength)
    {
        return (RX_BufferLength - RX_BufferIndex);
    }
    else
    {
        return -1;
    }
}
/**********************************************************************
name :
function : 
**********************************************************************/
int TwoWire::read(void)
{
    if(RX_BufferIndex < RX_BufferLength)
    {
        return RX_Buffer[RX_BufferIndex++];
    }
    else
    {
        return -1;
    }
}







