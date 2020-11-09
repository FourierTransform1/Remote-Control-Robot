//
//  i2c.c
//  i2c
//
//  Created by Michael Köhler on 09.10.17.
//	Modified by Nils Svensson on 19.03.28
//


#define F_CPU 16000000UL
#define I2C_WRITE   0
#define I2C_READ  1

#include "i2c.h"
#include <compat/twi.h>


#if defined (__AVR_ATmega328__) || defined(__AVR_ATmega328P__) || \
defined(__AVR_ATmega168P__) || defined(__AVR_ATmega168PA__) || \
defined(__AVR_ATmega88P__) || \
defined(__AVR_ATmega48P__) || \
defined(__AVR_ATmega1284P__) || \
defined (__AVR_ATmega324A__) || defined (__AVR_ATmega324P__) || defined (__AVR_ATmega324PA__) || \
defined (__AVR_ATmega644__) || defined (__AVR_ATmega644A__) || defined (__AVR_ATmega644P__) || defined (__AVR_ATmega644PA__) || \
defined (__AVR_ATmega1284P__) || \
defined (__AVR_ATmega2560__)
#if PSC_I2C != 1 && PSC_I2C != 4 && PSC_I2C != 16 && PSC_I2C != 64
#error "Wrong prescaler for TWI !"
#elif SET_TWBR < 0 || SET_TWBR > 255
#error "TWBR out of range, change PSC_I2C or F_I2C !"
#endif

uint8_t I2C_ErrorCode;
/**********************************************
 Public Function: i2c_init
 
 Purpose: Initialise TWI/I2C interface
 
 Input Parameter: none
 
 Return Value: none
 **********************************************/
void i2c_init(void){
	
    // set clock
    switch (PSC_I2C) {
        case 4:
            TWSR = 0x1;
            break;
        case 16:
            TWSR = 0x2;
            break;
        case 64:
            TWSR = 0x3;
            break;
        default:
            TWSR = 0x00;
            break;
    }
    TWBR = (uint8_t)SET_TWBR;
    // enable
    TWCR = (1 << TWEN);
}
/**********************************************
 Public Function: i2c_start
 
 Purpose: Start TWI/I2C interface
 
 Input Parameter:
 - uint8_t i2c_addr: Adress of reciever
 
 Return Value: none
 **********************************************/
/*void i2c_start(uint8_t i2c_addr){
    // i2c start
    TWCR = (1 << TWINT)|(1 << TWSTA)|(1 << TWEN);
	uint16_t timeout = F_CPU/F_I2C*2.0;
    while((TWCR & (1 << TWINT)) == 0 &&
		timeout !=0){
		timeout--;
		if(timeout == 0){
			I2C_ErrorCode |= (1 << I2C_START);
			return;
		}
	};
    // send adress
    TWDR = i2c_addr;
    TWCR = (1 << TWINT)|( 1 << TWEN);
    timeout = F_CPU/F_I2C*2.0;
    while((TWCR & (1 << TWINT)) == 0 &&
		  timeout !=0){
		timeout--;
		if(timeout == 0){
			I2C_ErrorCode |= (1 << I2C_SENDADRESS);
			return;
		}
	};
}*/

/************************************************************************
*Namn: i2c_start                                                        *
*                                                                       *
*Syfte: Starts the I2C connection  				                        *
*                                                                       *
*Indata: The adders of the slave                                        *
*                                                                       *
*Utdata: void								                            *
*                                                                       *
*Anropar:void													        *
************************************************************************/
void i2c_start(uint8_t i2c_addr){

	// i2c start
	TWCR = 0xA4;//(1 << TWINT)|(1 << TWSTA)|(1 << TWEN);
	while((TWCR & (1 << TWINT)) == 0);
	
	// send adress
	TWDR = i2c_addr;
	TWCR = 0x84;//(1 << TWINT)|( 1 << TWEN);
	while((TWCR & (1 << 7)) == 0);//(1 << TWINT)) == 0);
	
	}


/************************************************************************
*Namn: i2c_stop                                                         *
*                                                                       *
*Syfte: Stops the I2C connection  				                        *
*                                                                       *
*Indata: void                                                           *
*                                                                       *
*Utdata: void								                            *
*                                                                       *
*Anropar:void													        *
************************************************************************/
void i2c_stop(void){
    // i2c stop
    TWCR = 0x94;//(1 << TWINT)|(1 << TWSTO)|(1 << TWEN);//or 0x94
}
/**********************************************
 Public Function: i2c_byte
 
 Purpose: Send byte at TWI/I2C interface
 
 Input Parameter:
 - uint8_t byte: Byte to send to reciever
 
 Return Value: none
 **********************************************/
/*void i2c_byte(uint8_t byte){
    TWDR = byte;
    TWCR = 0x84;// (1 << TWINT)|( 1 << TWEN);
    //uint16_t timeout = F_CPU/F_I2C*2.0;
    while((TWCR & (1 << TWINT)) == 0);
		 timeout !=0){
		timeout--;
		if(timeout == 0){
			I2C_ErrorCode |= (1 << I2C_BYTE);
			return;
		}
	};
}*/

/************************************************************************
*Namn: i2c_byte                                                         *
*                                                                       *
*Syfte: SSend a byte on the bus 				                        *
*                                                                       *
*Indata: The adders of the slave                                        *
*                                                                       *
*Utdata: void								                            *
*                                                                       *
*Anropar:void													        *
************************************************************************/
void i2c_byte(uint8_t byte){
    TWDR = byte;
    TWCR = 0x84;// (1 << TWINT)|( 1 << TWEN);
    while((TWCR & (1 << TWINT)) == 0);
}

/**********************************************
 Public Function: i2c_readAck
 
 Purpose: read acknowledge from TWI/I2C Interface
 
 Input Parameter: none
 
 Return Value: uint8_t
  - TWDR: recieved value at TWI/I2C-Interface, 0 at timeout
  - 0:    Error at read
 **********************************************/
/*uint8_t i2c_readAck(void){
    TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWEA);
    uint16_t timeout = F_CPU/F_I2C*2.0;
    while((TWCR & (1 << TWINT)) == 0 &&
		  timeout !=0){
		timeout--;
		if(timeout == 0){
			I2C_ErrorCode |= (1 << I2C_READACK);
			return 0;
		}
	};
    return TWDR;
}*/

/************************************************************************
*Namn: i2c_readAck                                                      *
*                                                                       *
*Syfte: read acknowledge from the bus			                        *
*                                                                       *
*Indata: void                                                           *
*                                                                       *
*Utdata: The received value from the bus                                *
*                                                                       *
*Anropar: void  												        *
************************************************************************/
uint8_t i2c_readAck(void){
    TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWEA);
    uint16_t timeout = F_CPU/F_I2C*2.0;
    while((TWCR & (1 << TWINT)) == 0 &&
		  timeout !=0){
		timeout--;
		if(timeout == 0){
			I2C_ErrorCode |= (1 << I2C_READACK);
			return 0;
		}
	};
    return TWDR;
}

 /************************************************************************
 *Namn: i2c_readNAck                                                     *
 *                                                                       *
 *Syfte: read non-acknowledge from the bus		                         *
 *                                                                       *
 *Indata: void                                                           *
 *                                                                       *
 *Utdata: The received value from the bus                                *
 *                                                                       *
 *Anropar: void  												         *
 ************************************************************************/
uint8_t i2c_readNAck(void){
    TWCR = (1<<TWINT)|(1<<TWEN);
    while((TWCR & (1 << TWINT)) == 0);
    return TWDR;
}

 /**********************************************
 Public Function: i2c_readNAck
 
 Purpose: read non-acknowledge from TWI/I2C Interface
 
 Input Parameter: none
 
 Return Value: uint8_t
  - TWDR: recieved value at TWI/I2C-Interface
  - 0:    Error at read
 **********************************************/
/*uint8_t i2c_readNAck(void){
    TWCR = (1<<TWINT)|(1<<TWEN);
    uint16_t timeout = F_CPU/F_I2C*2.0;
    while((TWCR & (1 << TWINT)) == 0 &&
		  timeout !=0){
		timeout--;
		if(timeout == 0){
			I2C_ErrorCode |= (1 << I2C_READNACK);
            return 0;
		}
	};
    return TWDR;
}*/



/*************************************************************************
 Issues a start condition and sends address and transfer direction.
 If device is busy, use ack polling to wait until device is ready
 
 Input:   address and transfer direction of I2C device
*************************************************************************/
void i2c_start_wait(unsigned char address)
{
    uint8_t   twst;


    while ( 1 )
    {
	    // send START condition
	    TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);
    
    	// wait until transmission completed
    	while(!(TWCR & (1<<TWINT)));
    
    	// check value of TWI Status Register. Mask prescaler bits.
    	twst = TW_STATUS & 0xF8;
    	if ( (twst != TW_START) && (twst != TW_REP_START)) continue;
    
    	// send device address
    	TWDR = address;
    	TWCR = (1<<TWINT) | (1<<TWEN);
    
    	// wail until transmission completed
    	while(!(TWCR & (1<<TWINT)));
    
    	// check value of TWI Status Register. Mask prescaler bits.
    	twst = TW_STATUS & 0xF8;
    	if ( (twst == TW_MT_SLA_NACK )||(twst ==TW_MR_DATA_NACK) ) 
    	{    	    
    	    /* device busy, send stop condition to terminate write operation */
	        TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);
	        
	        // wait until stop condition is executed and bus released
	        while(TWCR & (1<<TWSTO));
	        
    	    continue;
    	}
    	//if( twst != TW_MT_SLA_ACK) return 1;
    	break;
     }

}/* i2c_start_wait */

void i2c_read_byte(uint8_t dev_addr, uint8_t reg_addr, uint8_t* data){

	i2c_start_wait(dev_addr+I2C_WRITE); 	//start i2c to write register address
	i2c_byte(reg_addr);			//write address of register to read
	i2c_start(dev_addr+I2C_READ);	//restart i2c to start reading
	*data = i2c_readNAck();
	i2c_stop();

}

// write one byte to dev
void i2c_write_byte(uint8_t dev_addr, uint8_t reg_addr, uint8_t data){
	
	i2c_start_wait(dev_addr+I2C_WRITE);
	i2c_byte(reg_addr);
	i2c_byte(data);
	i2c_stop();

}

#else
#error "Micorcontroller not supported now!"
#endif
