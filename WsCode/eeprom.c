/*******************************************************************************
* FILE NAME: eeprom.c
*
* DESCRIPTION:
*
* USAGE:
*
*******************************************************************************/

#include "ifi_aliases.h"
#include "ifi_default.h"
#include "ifi_utilities.h"

#include "eeprom.h"

/* initialize control bits and pointers to false */
static eeControlStruct eeControl = {0,0,0,0,0,0,};


/*******************************************************************************
* FUNCTION NAME: readEE
* PURPOSE:       Read value from EEPROM
* CALLED FROM:
* ARGUMENTS:     address - address of value to read
* RETURNS:       value at address
*******************************************************************************/
unsigned char readEE(unsigned short address)
{
  /* should not change EEADR/EEADRH while a write is happening */
  if (eeControl.writeInProgress) return 0;

  /* Load address into address register */
  EEADRH = (unsigned char)(address>>8);
  EEADR = (unsigned char)(address&0xFF);

  /* Configuration as per manual */
  EECON1bits.EEPGD = 0;
  EECON1bits.CFGS = 0;
  EECON1bits.RD = 1;

  return (EEDATA);
}


/*******************************************************************************
* FUNCTION NAME: writeEE
* PURPOSE:       Add data to EEPROM write buffer
* CALLED FROM:
* ARGUMENTS:     address - address of value to write
*                data - data to store in EEPROM
* RETURNS:       none
*******************************************************************************/
void writeEE(unsigned short address, unsigned char data)
{
  if (eeControl.bufferFull) return;

  eeControl.dataBuffer[eeControl.bufferEnd] = data;
  eeControl.addressBuffer[eeControl.bufferEnd++] = address;
  eeControl.bufferFull = (eeControl.bufferEnd == eeControl.bufferPtr);
  eeControl.bufferNotEmpty = 1;

  processEEQueue();
}

/*******************************************************************************
* FUNCTION NAME: processEEQueue
* PURPOSE:       Transfer data from buffer to EEPROM
* CALLED FROM:
* ARGUMENTS:
* RETURNS:       none
*******************************************************************************/
void processEEQueue(void)
{

  /*
   * call this once each main loop, or more often, to process the queue
   * of bytes to be written to eeprom. Note that there is no buffer
   * overrun detection. Enlarge the buffer and change the size of bufferPtr
   * and bufferEnd if you need more buffer space to avoid overrun.
   */

  /* previous write not complete */
  if (eeControl.writeInProgress && !PIR2bits.EEIF) return;

  PIR2bits.EEIF = 0; // reset EE write done interrupt flag

  if (!eeControl.bufferNotEmpty)
  {
    eeControl.writeInProgress = 0;
    return;
  }

  /* OK, previous write is done and something is in the buffer, write it */
  eeControl.writeInProgress = 1;

  /* Load address into address register */
  EEADR = (UINT8)(eeControl.addressBuffer[eeControl.bufferPtr] & 0xff);
  EEADRH = (UINT8)(eeControl.addressBuffer[eeControl.bufferPtr] >> 8);
  EEDATA = eeControl.dataBuffer[eeControl.bufferPtr++];
  eeControl.bufferNotEmpty = (eeControl.bufferPtr != eeControl.bufferEnd);

  /* with a little more work, we could check for error on previous write and
     redo it */

  /* write sequence as per manual */
  EECON1bits.EEPGD =0;
  EECON1bits.CFGS =0;
  EECON1bits.WREN =1;
  INTCONbits.GIE = 0;
  EECON2 = 0x55;
  EECON2 = 0xAA;
  EECON1bits.WR = 1;
  INTCONbits.GIE = 1;
  EECON1bits.WREN = 0;
}



/*******************************************************************************
* FUNCTION NAME: checkEEQueue
* PURPOSE:       Check if the write queue is empty
* CALLED FROM:
* ARGUMENTS:
* RETURNS:       EMPTY if queue is empty, else NOT_EMPTY
*******************************************************************************/
UINT8 checkEEQueue(void)
{
  if (eeControl.bufferNotEmpty == 1)
  {
    return (EE_NOT_EMPTY);
  }
  else
  {
    return (EE_EMPTY);
  }
}



