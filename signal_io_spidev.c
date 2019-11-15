/////////////////////////////////////////////////////////////////////////////////
//                                                                             //
//  Copyright (c) 2019 Leonardo Consoni <leonardjc@protonmail.com>             //
//                                                                             //
//  This file is part of Signal-IO-SPIDev-AksIM.                               //
//                                                                             //
//  Signal-IO-SPIDev-AksIM is free software: you can redistribute it and/or    //
//  modify it under the terms of the GNU Lesser General Public License as      //
//  published by the Free Software Foundation, either version 3 of the         //
//  License, or (at your option) any later version.                            //
//                                                                             //
//  Signal-IO-SPIDev-AksIM is distributed in the hope that it will be          //
//  useful, but WITHOUT ANY WARRANTY; without even the implied warranty of     //
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the               //
//  GNU Lesser General Public License for more details.                        //
//                                                                             //
//  You should have received a copy of the GNU Lesser General Public License   //
//  along with Signal-IO-SPIDev-AksIM. If not, see                             //
//  <http://www.gnu.org/licenses/>.                                            //
//                                                                             //
/////////////////////////////////////////////////////////////////////////////////  

#include "interface/signal_io.h"

#include <fcntl.h>
#include <errno.h>
#include <string.h>
#include <sys/ioctl.h>
#include <asm/ioctl.h>
#include <linux/spi/spidev.h>

const char* SPI_DEVS[] = { "/dev/spidev0.0", "/dev/spidev0.1" };

int32_t inputValues[ 2 ] = { 0, 0 };
int spiFDs[ 2 ] = { 0, 0 };
struct spi_ioc_transfer spiIOCs[ 2 ];

DECLARE_MODULE_INTERFACE( SIGNAL_IO_INTERFACE );

long int InitDevice( const char* configuration )
{  
  uint32_t speedHz = (uint32_t) strtoul( configuration, NULL, 0 );
  
  uint8_t mode = SPI_MODE_1;
  uint8_t bitsPerWord = 8;
  for( size_t channel = 0; channel < 2; channel++ )
  {
    if( spiFDs[ channel ] != 0 )
    {
      if( (spiFDs[ channel ] = open( SPI_DEVS[ channel ], O_RDWR )) < 0 )
        fprintf( stderr, "Unable to open SPI device %s: %s\n", SPI_DEVS[ channel ], strerror( errno ) );
      if( ioctl( spiFDs[ channel ], SPI_IOC_WR_MODE, &mode ) < 0 )
        fprintf( stderr, "Unable to change SPI device %s mode: %s\n", SPI_DEVS[ channel ], strerror( errno ) );  
      if( ioctl( spiFDs[ channel ], SPI_IOC_WR_BITS_PER_WORD, &bitsPerWord ) < 0 )
        fprintf( stderr, "Unable to change SPI device %s BPW: %s\n", SPI_DEVS[ channel ], strerror( errno ) );
      if( ioctl( spiFDs[ channel ], SPI_IOC_WR_MAX_SPEED_HZ, &speedHz ) < 0 )
        fprintf( stderr, "Unable to change SPI device %s speed: %s\n", SPI_DEVS[ channel ], strerror( errno ) );
  
      spiIOCs[ channel ].len = 2;
      spiIOCs[ channel ].delay_usecs = 8;
      spiIOCs[ channel ].bits_per_word = bitsPerWord;
      spiIOCs[ channel ].speed_hz = speedHz;
    }
  }
  
  if( spiFDs[ 0 ] < 0 || spiFDs[ 1 ] < 0 )
  {
    EndDevice( 0 );
    return SIGNAL_IO_DEVICE_INVALID_ID;
  }

  return 0;
}

void EndDevice( long int deviceID )
{
  if( spiFDs[ 0 ] > 0 ) close( spiFDs[ 0 ] );
  if( spiFDs[ 1 ] > 0 ) close( spiFDs[ 1 ] );
  
  return;
}

size_t GetMaxInputSamplesNumber( long int deviceID )
{
  if( deviceID == SIGNAL_IO_DEVICE_INVALID_ID ) return 0;
  
  return 1;
}

size_t Read( long int deviceID, unsigned int channel, double* ref_value )
{
  *ref_value = 0.0;
  
  if( deviceID == SIGNAL_IO_DEVICE_INVALID_ID ) return 0;
  
  if( channel > 2 ) return 0;
   
  // SPI transfer buffer is output (before) and input (after)
  uint8_t inputData[ 2 ];
  spiIOCs[ channel ].tx_buf = spiIOCs[ channel ].rx_buf = (uint64_t) inputData;
  if( ioctl( spiFDs[ channel ], SPI_IOC_MESSAGE( 1 ), &(spiIOCs[ channel ]) ) < 0 )
    fprintf( stderr, "Unable to read from SPI device %s: %s\n", SPI_DEVS[ channel ], strerror( errno ) );
  uint16_t rawInputValue = ( ( inputData[ 0 ] << 8 ) & 0xFF00 ) + inputData[ 1 ];
  
  int32_t overflowsNumber = inputValues[ channel ] / UINT16_MAX;
  if( inputValues[ channel ] < 0 ) overflowsNumber--;
  int32_t newInputValue = overflowsNumber * UINT16_MAX + rawInputValue;
  if( ( inputValues[ channel ] - newInputValue ) > ( UINT16_MAX / 2 ) ) overflowsNumber++;
  if( ( inputValues[ channel ] - newInputValue ) < ( -UINT16_MAX / 2 ) ) overflowsNumber--;
  inputValues[ channel ] = overflowsNumber * UINT16_MAX + rawInputValue;
  
  *ref_value = (double) inputValues[ channel ];

  return 1;
}

bool HasError( long int deviceID )
{
  return false;
}

void Reset( long int deviceID )
{
  return;
}

bool CheckInputChannel( long int deviceID, unsigned int channel )
{
  if( deviceID == SIGNAL_IO_DEVICE_INVALID_ID ) return false;
  
  if( channel > 2 ) return false;
  
  return true;
}

bool Write( long int deviceID, unsigned int channel, double value )
{
  return false;
}

bool AcquireOutputChannel( long int deviceID, unsigned int channel )
{
  return false;
}

void ReleaseOutputChannel( long int deviceID, unsigned int channel )
{
  return;
}

