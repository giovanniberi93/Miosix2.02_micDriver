/***************************************************************************
 *   Copyright (C) 2011 by Terraneo Federico                               *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   As a special exception, if other files instantiate templates or use   *
 *   macros or inline functions from this file, or you compile this file   *
 *   and link it with other works to produce a work based on this file,    *
 *   this file does not by itself cause the resulting work to be covered   *
 *   by the GNU General Public License. However the source code for this   *
 *   file must still be made available in accordance with the GNU General  *
 *   Public License. This exception does not invalidate any other reasons  *
 *   why a work based on this file might be covered by the GNU General     *
 *   Public License.                                                       *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, see <http://www.gnu.org/licenses/>   *
 ***************************************************************************/

#include <algorithm>
#include <stdexcept>
#include <cstring>
#include "miosix/kernel/scheduler/scheduler.h"
#include "miosix/kernel/queue.h"
#include "util/software_i2c.h"
#include "player.h"

using namespace std;
using namespace miosix;

typedef Gpio<GPIOB_BASE,6>  scl;
typedef Gpio<GPIOB_BASE,9>  sda;
typedef Gpio<GPIOA_BASE,4>  lrck;
typedef Gpio<GPIOC_BASE,7>  mclk;
typedef Gpio<GPIOC_BASE,10> sclk;
typedef Gpio<GPIOC_BASE,12> sdin;
typedef Gpio<GPIOD_BASE,4>  reset;
typedef SoftwareI2C<sda,scl> i2c;


static const int bufferSize=256; //Buffer RAM is 4*bufferSize bytes
static Thread *waiting;
static BufferQueue<unsigned short,bufferSize> *bq;
static bool enobuf=true;

/**
 * Configure the DMA to do another transfer
 */
static void IRQdmaRefill()
{
    const unsigned short *buffer;
	unsigned int size;
	if(bq->tryGetReadableBuffer(buffer,size)==false)
	{
		enobuf=true;
		return;
	}
    DMA1_Stream5->CR=0;
	DMA1_Stream5->PAR=reinterpret_cast<unsigned int>(&SPI3->DR);
	DMA1_Stream5->M0AR=reinterpret_cast<unsigned int>(buffer);
	DMA1_Stream5->NDTR=size;
	DMA1_Stream5->CR=DMA_SxCR_PL_1    | //High priority DMA stream
                     DMA_SxCR_MSIZE_0 | //Read  16bit at a time from RAM
					 DMA_SxCR_PSIZE_0 | //Write 16bit at a time to SPI
				     DMA_SxCR_MINC    | //Increment RAM pointer
			         DMA_SxCR_DIR_0   | //Memory to peripheral direction
			         DMA_SxCR_TCIE    | //Interrupt on completion
			  	     DMA_SxCR_EN;       //Start the DMA

}

/**
 * Helper function used to start a DMA transfer from non-interrupt code
 */
static void dmaRefill()
{
	FastInterruptDisableLock dLock;
	IRQdmaRefill();
}

/**
 * DMA end of transfer interrupt
 */
void __attribute__((naked)) DMA1_Stream5_IRQHandler()
{
    saveContext();
	asm volatile("bl _Z17I2SdmaHandrerImplv");
	restoreContext();
}

/**
 * DMA end of transfer interrupt actual implementation
 */
void __attribute__((used)) I2SdmaHandrerImpl()
{
	DMA1->HIFCR=DMA_HIFCR_CTCIF5  |
                DMA_HIFCR_CTEIF5  |
                DMA_HIFCR_CDMEIF5 |
                DMA_HIFCR_CFEIF5;
	bq->bufferEmptied();
	IRQdmaRefill();
	waiting->IRQwakeup();
	if(waiting->IRQgetPriority()>Thread::IRQgetCurrentThread()->IRQgetPriority())
		Scheduler::IRQfindNextThread();
}

static void cs43l22send(unsigned char index, unsigned char data)
{
    i2c::sendStart();
    i2c::send(0x94);
    i2c::send(index);
    i2c::send(data);
    i2c::sendStop();
}

/**
 * \param db volume level in db (0 to -102). Warning: 0db volume is LOUD!
 * \return value to store in register 0x20 and 0x21
 */
void cs43l22volume(int db)
{
    db=2*max(-102,min(0,db));
    unsigned char vol=static_cast<unsigned int>(db) & 0xff;
    cs43l22send(0x20,vol);
    cs43l22send(0x21,vol);
}


/**
 * This function allows to atomically check if a variable equals a specific
 * value and if not, put the thread in wait state until said condition is
 * satisfied. To wake the thread, another thread or an interrupt routine
 * should first set the variable to the desired value, and then call
 * wakeup() (or IRQwakeup()) on the sleeping thread.
 * \param T type of the variable to test
 * \param variable the variable to test
 * \param value the value that will cause this function to return.
 */
template<typename T>
static void atomicTestAndWaitUntil(volatile T& variable, T value)
{
	FastInterruptDisableLock dLock;
	while(variable!=value)
	{
		Thread::IRQgetCurrentThread()->IRQwait();
		{
			FastInterruptEnableLock eLock(dLock);
			Thread::yield();
		}
	}
}

/**
 * Helper function that waits until a buffer is available for writing
 * \return a writable buffer from bq
 */
static unsigned short *getWritableBuffer()
{
	FastInterruptDisableLock dLock;
	unsigned short *result;
	while(bq->tryGetWritableBuffer(result)==false)
	{
		waiting->IRQwait();
		{
			FastInterruptEnableLock eLock(dLock);
			Thread::yield();
		}
	}
	return result;
}

/**
 * Helper function called after a buffer is filled
 */
static void bufferFilled()
{
	FastInterruptDisableLock dLock;
	bq->bufferFilled(bq->bufferMaxSize());
}

//
// class Player
//

Player& Player::instance()
{
	static Player singleton;
	return singleton;
}

bool Player::fillStereoBuffer(unsigned short *buffer, int size)
{
        int remaining=soundSize-soundIndex;
	int length=std::min(size,remaining*2);
	for(int i=0;i<length;i+=2)
	{
		unsigned short in=soundBuffer[soundIndex++];
		buffer[i+0]=buffer[i+1]=in;
	}
	if(soundIndex<soundSize) return false;
	for(int i=length;i<size;i++) buffer[i]=0;
	return true;
}

void Player::play(unsigned short* buf, unsigned short size)
{
	Lock<Mutex> l(mutex);
    
    soundBuffer = buf;
    soundSize = size;
    soundIndex = 0;
        
    waiting=Thread::getCurrentThread();
    fillStereoBuffer(getWritableBuffer(),bufferSize);
    fillStereoBuffer(getWritableBuffer(),bufferSize);
    for(;;)
    {
        if(enobuf)
	{
                enobuf=false;
                dmaRefill();
	}
        if(fillStereoBuffer(getWritableBuffer(),bufferSize)) break;
	bufferFilled();
    }
    
    //Trailing blank audio, so as to be sure audio is played to the end    
    /*
    memset(getWritableBuffer(),0x8000,bufferSize*sizeof(unsigned short));
	bufferFilled();
    */
     
}

void Player::init(){
    
    {
		FastInterruptDisableLock dLock;
        //Enable DMA1 and SPI3/I2S3
        RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;
        RCC->APB1ENR |= RCC_APB1ENR_SPI3EN;   
        //Configure GPIOs
        i2c::init();
        mclk::mode(Mode::ALTERNATE);
        mclk::alternateFunction(6);
        sclk::mode(Mode::ALTERNATE);
        sclk::alternateFunction(6);
        sdin::mode(Mode::ALTERNATE);
        sdin::alternateFunction(6);
        lrck::mode(Mode::ALTERNATE);
        lrck::alternateFunction(6);
        reset::mode(Mode::OUTPUT);
        //Enable audio PLL (settings for 44100Hz audio)
        RCC->PLLI2SCFGR=(2<<28) | (271<<6);
        RCC->CR |= RCC_CR_PLLI2SON;
    }
    //Wait for PLL to lock
    while((RCC->CR & RCC_CR_PLLI2SRDY)==0) ;
    
    reset::low(); //Keep in reset state
    delayUs(5);
    reset::high();
    delayUs(5);
    cs43l22send(0x00,0x99); //These five command are the "magic" initialization
    cs43l22send(0x47,0x80);
    cs43l22send(0x32,0xbb);
    cs43l22send(0x32,0x3b);
    cs43l22send(0x00,0x00);
    
    cs43l22send(0x05,0x20); //AUTO=0, SPEED=01, 32K=0, VIDEO=0, RATIO=0, MCLK=0
    cs43l22send(0x04,0xaf); //Headphone always ON, Speaker always OFF
    cs43l22send(0x06,0x04); //I2S Mode
    cs43l22volume(0);
    
    SPI3->CR2=SPI_CR2_TXDMAEN;
    SPI3->I2SPR=  SPI_I2SPR_MCKOE | 6;
	SPI3->I2SCFGR=SPI_I2SCFGR_I2SMOD    //I2S mode selected
                | SPI_I2SCFGR_I2SE      //I2S Enabled
                | SPI_I2SCFGR_I2SCFG_1; //Master transmit
    bq=new BufferQueue<unsigned short,bufferSize>();
    NVIC_SetPriority(DMA1_Stream5_IRQn,2);//High priority for DMA
    NVIC_EnableIRQ(DMA1_Stream5_IRQn);
    cs43l22send(0x02,0x9e);
}

bool Player::isPlaying() const
{
	if(mutex.tryLock()==false) return true;
	mutex.unlock();
	return false;
}

Player::Player() {}

Player::~Player() {
    NVIC_DisableIRQ(DMA1_Stream5_IRQn);
    delete bq;
}
