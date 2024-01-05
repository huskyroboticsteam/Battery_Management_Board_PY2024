/***************************************************************************//**
* \file Jetty_I2C_MASTER.c
* \version 4.0
*
* \brief
*  This file provides the source code to the API for the SCB Component in
*  I2C Master mode.
*
* Note:
*
*******************************************************************************
* \copyright
* Copyright 2013-2017, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#include "Jetty_PVT.h"
#include "Jetty_I2C_PVT.h"

#if(Jetty_I2C_MASTER_CONST)

/***************************************
*      I2C Master Private Vars
***************************************/

/* Master variables */
volatile uint16 Jetty_mstrStatus;      /* Master Status byte  */
volatile uint8  Jetty_mstrControl;     /* Master Control byte */

/* Receive buffer variables */
volatile uint8 * Jetty_mstrRdBufPtr;   /* Pointer to Master Read buffer */
volatile uint32  Jetty_mstrRdBufSize;  /* Master Read buffer size       */
volatile uint32  Jetty_mstrRdBufIndex; /* Master Read buffer Index      */

/* Transmit buffer variables */
volatile uint8 * Jetty_mstrWrBufPtr;   /* Pointer to Master Write buffer */
volatile uint32  Jetty_mstrWrBufSize;  /* Master Write buffer size       */
volatile uint32  Jetty_mstrWrBufIndex; /* Master Write buffer Index      */
volatile uint32  Jetty_mstrWrBufIndexTmp; /* Master Write buffer Index Tmp */

#if (!Jetty_CY_SCBIP_V0 && \
    Jetty_I2C_MULTI_MASTER_SLAVE_CONST && Jetty_I2C_WAKE_ENABLE_CONST)
    static void Jetty_I2CMasterDisableEcAm(void);
#else
    #define Jetty_I2CMasterDisableEcAm()     do{ /* Empty */ }while(0)
#endif /* (!Jetty_CY_SCBIP_V0) */

static uint32 Jetty_I2CMasterHandleStatus(uint32 status);
static uint32 Jetty_I2CMasterWaitOneUnit(uint32 *timeout);


/*******************************************************************************
* Function Name: Jetty_I2CMasterWriteBuf
****************************************************************************//**
*
*  Automatically writes an entire buffer of data to a slave device.
*  Once the data transfer is initiated by this function, further data transfer
*  is handled by the included ISR.
*  Enables the I2C interrupt and clears Jetty_I2C_MSTAT_WR_CMPLT
*  status.
*
*  \param slaveAddr: 7-bit slave address.
*  \param xferData: Pointer to buffer of data to be sent.
*  \param cnt: Size of buffer to send.
*  \param mode: Transfer mode defines:
*  (1) Whether a start or restart condition is generated at the beginning
*  of the transfer, and
*  (2) Whether the transfer is completed or halted before the stop
*  condition is generated on the bus.Transfer mode, mode constants
*  may be ORed together.
*  - Jetty_I2C_MODE_COMPLETE_XFER - Perform complete transfer
*    from Start to Stop.
*  - Jetty_I2C_MODE_REPEAT_START - Send Repeat Start instead
*    of Start. A Stop is generated after transfer is completed unless
*    NO_STOP is specified.
*  - Jetty_I2C_MODE_NO_STOP Execute transfer without a Stop.
*    The following transfer expected to perform ReStart.
*
* \return
*  Error status.
*  - Jetty_I2C_MSTR_NO_ERROR - Function complete without error.
*    The master started the transfer.
*  - Jetty_I2C_MSTR_BUS_BUSY - Bus is busy. Nothing was sent on
*    the bus. The attempt has to be retried.
*  - Jetty_I2C_MSTR_NOT_READY - Master is not ready for to start
*    transfer. A master still has not completed previous transaction or a
*    slave operation is in progress (in multi-master-slave configuration).
*    Nothing was sent on the bus. The attempt has to be retried.
*
* \globalvars
*  Jetty_mstrStatus  - used to store current status of I2C Master.
*  Jetty_state       - used to store current state of software FSM.
*  Jetty_mstrControl - used to control master end of transaction with
*  or without the Stop generation.
*  Jetty_mstrWrBufPtr - used to store pointer to master write buffer.
*  Jetty_mstrWrBufIndex - used to current index within master write
*  buffer.
*  Jetty_mstrWrBufSize - used to store master write buffer size.
*
*******************************************************************************/
uint32 Jetty_I2CMasterWriteBuf(uint32 slaveAddress, uint8 * wrData, uint32 cnt, uint32 mode)
{
    uint32 errStatus;

    errStatus = Jetty_I2C_MSTR_NOT_READY;

    if(NULL != wrData)  /* Check buffer pointer */
    {
        /* Check FSM state and bus before generating Start/ReStart condition */
        if(Jetty_CHECK_I2C_FSM_IDLE)
        {
            Jetty_DisableInt();  /* Lock from interruption */

            /* Check bus state */
            errStatus = Jetty_CHECK_I2C_STATUS(Jetty_I2C_STATUS_BUS_BUSY) ?
                            Jetty_I2C_MSTR_BUS_BUSY : Jetty_I2C_MSTR_NO_ERROR;
        }
        else if(Jetty_CHECK_I2C_FSM_HALT)
        {
            Jetty_mstrStatus &= (uint16) ~Jetty_I2C_MSTAT_XFER_HALT;
                              errStatus  = Jetty_I2C_MSTR_NO_ERROR;
        }
        else
        {
            /* Unexpected FSM state: exit */
        }
    }

    /* Check if master is ready to start  */
    if(Jetty_I2C_MSTR_NO_ERROR == errStatus) /* No error proceed */
    {
        /* Non-empty for master-slave mode when wakeup enabled */
        Jetty_I2CMasterDisableEcAm();

        /* Set up write transaction */
        Jetty_state = Jetty_I2C_FSM_MSTR_WR_ADDR;
        Jetty_mstrWrBufIndexTmp = 0u;
        Jetty_mstrWrBufIndex    = 0u;
        Jetty_mstrWrBufSize     = cnt;
        Jetty_mstrWrBufPtr      = (volatile uint8 *) wrData;
        Jetty_mstrControl       = (uint8) mode;

        slaveAddress = Jetty_GET_I2C_8BIT_ADDRESS(slaveAddress);

        Jetty_mstrStatus &= (uint16) ~Jetty_I2C_MSTAT_WR_CMPLT;

        Jetty_ClearTxInterruptSource    (Jetty_INTR_TX_UNDERFLOW);
        Jetty_ClearMasterInterruptSource(Jetty_INTR_MASTER_ALL);
        Jetty_SetMasterInterruptMode    (Jetty_I2C_INTR_MASTER_MASK);

        /* The TX and RX FIFO have to be EMPTY */

        /* Enable interrupt source to catch when address is sent */
        Jetty_SetTxInterruptMode(Jetty_INTR_TX_UNDERFLOW);

        /* Generate Start or ReStart */
        if(Jetty_CHECK_I2C_MODE_RESTART(mode))
        {
            Jetty_I2C_MASTER_GENERATE_RESTART;
            Jetty_TX_FIFO_WR_REG = slaveAddress;
        }
        else
        {
            Jetty_TX_FIFO_WR_REG = slaveAddress;
            Jetty_I2C_MASTER_GENERATE_START;
        }
    }

    Jetty_EnableInt();   /* Release lock */

    return(errStatus);
}


/*******************************************************************************
* Function Name: Jetty_I2CMasterReadBuf
****************************************************************************//**
*
*  Automatically reads an entire buffer of data from a slave device.
*  Once the data transfer is initiated by this function, further data transfer
*  is handled by the included ISR.
*  Enables the I2C interrupt and clears Jetty_I2C_MSTAT_RD_CMPLT
*  status.
*
*  \param slaveAddr: 7-bit slave address.
*  \param xferData: Pointer to buffer of data to be sent.
*  \param cnt: Size of buffer to send.
*  \param mode: Transfer mode defines:
*  (1) Whether a start or restart condition is generated at the beginning
*  of the transfer, and
*  (2) Whether the transfer is completed or halted before the stop
*  condition is generated on the bus.Transfer mode, mode constants may
*  be ORed together. See Jetty_I2CMasterWriteBuf()
*  function for constants.
*
* \return
*  Error status.See Jetty_I2CMasterWriteBuf()
*  function for constants.
*
* \globalvars
*  Jetty_mstrStatus  - used to store current status of I2C Master.
*  Jetty_state       - used to store current state of software FSM.
*  Jetty_mstrControl - used to control master end of transaction with
*  or without the Stop generation.
*  Jetty_mstrRdBufPtr - used to store pointer to master read buffer.
*  Jetty_mstrRdBufIndex - used to current index within master read
*  buffer.
*  Jetty_mstrRdBufSize - used to store master read buffer size.
*
*******************************************************************************/
uint32 Jetty_I2CMasterReadBuf(uint32 slaveAddress, uint8 * rdData, uint32 cnt, uint32 mode)
{
    uint32 errStatus;

    errStatus = Jetty_I2C_MSTR_NOT_READY;

    if(NULL != rdData)
    {
        /* Check FSM state and bus before generating Start/ReStart condition */
        if(Jetty_CHECK_I2C_FSM_IDLE)
        {
            Jetty_DisableInt();  /* Lock from interruption */

            /* Check bus state */
            errStatus = Jetty_CHECK_I2C_STATUS(Jetty_I2C_STATUS_BUS_BUSY) ?
                            Jetty_I2C_MSTR_BUS_BUSY : Jetty_I2C_MSTR_NO_ERROR;
        }
        else if(Jetty_CHECK_I2C_FSM_HALT)
        {
            Jetty_mstrStatus &= (uint16) ~Jetty_I2C_MSTAT_XFER_HALT;
                              errStatus  =  Jetty_I2C_MSTR_NO_ERROR;
        }
        else
        {
            /* Unexpected FSM state: exit */
        }
    }

    /* Check master ready to proceed */
    if(Jetty_I2C_MSTR_NO_ERROR == errStatus) /* No error proceed */
    {
        /* Non-empty for master-slave mode when wakeup enabled */
        Jetty_I2CMasterDisableEcAm();

        /* Set up read transaction */
        Jetty_state = Jetty_I2C_FSM_MSTR_RD_ADDR;
        Jetty_mstrRdBufIndex = 0u;
        Jetty_mstrRdBufSize  = cnt;
        Jetty_mstrRdBufPtr   = (volatile uint8 *) rdData;
        Jetty_mstrControl    = (uint8) mode;

        slaveAddress = (Jetty_GET_I2C_8BIT_ADDRESS(slaveAddress) | Jetty_I2C_READ_FLAG);

        Jetty_mstrStatus &= (uint16) ~Jetty_I2C_MSTAT_RD_CMPLT;

        Jetty_ClearMasterInterruptSource(Jetty_INTR_MASTER_ALL);
        Jetty_SetMasterInterruptMode    (Jetty_I2C_INTR_MASTER_MASK);

        /* TX and RX FIFO have to be EMPTY */

        /* Prepare reading */
        if(Jetty_mstrRdBufSize < Jetty_I2C_FIFO_SIZE)
        {
            /* Reading byte-by-byte */
            Jetty_SetRxInterruptMode(Jetty_INTR_RX_NOT_EMPTY);
        }
        else
        {
            /* Receive RX FIFO chunks */
            Jetty_ENABLE_MASTER_AUTO_DATA_ACK;
            Jetty_SetRxInterruptMode(Jetty_INTR_RX_FULL);
        }

        /* Generate Start or ReStart */
        if(Jetty_CHECK_I2C_MODE_RESTART(mode))
        {
            Jetty_I2C_MASTER_GENERATE_RESTART;
            Jetty_TX_FIFO_WR_REG = slaveAddress;
        }
        else
        {
            Jetty_TX_FIFO_WR_REG = slaveAddress;
            Jetty_I2C_MASTER_GENERATE_START;
        }
    }

    Jetty_EnableInt();   /* Release lock */

    return(errStatus);
}


/*******************************************************************************
* Function Name: Jetty_I2CMasterSendStart
****************************************************************************//**
*
* Generates Start condition and sends slave address with read/write bit.
* Disables the I2C interrupt.
* This function is blocking and does not return until start condition and
* address byte are sent and ACK/NACK response is received or errors occurred.
*
* \param slaveAddress
* Right justified 7-bit Slave address (valid range 8 to 120).
*
* \param bitRnW
* Direction of the following transfer. It is defined by read/write bit within
* address byte.
*  - Jetty_I2C_WRITE_XFER_MODE - Set write direction for the
*    following transfer.
*  - Jetty_I2C_READ_XFER_MODE - Set read direction for the
*    following transfer.
*
* \param timeoutMs
* Defines in milliseconds the time that this function can block for.
* If that time expires, the function returns. If a zero is passed,
* the function waits forever for the action to complete. If a timeout occurs,
* the SCB block is reset. Note The maximum value is (maximum uint32)/1000.
*
* \return
* Error status.
*  - Jetty_I2C_MSTR_NO_ERROR - Function complete without error.
*  - Jetty_I2C_MSTR_BUS_BUSY - Bus is busy.
*    Nothing was sent on the bus. The attempt has to be retried.
*  - Jetty_I2C_MSTR_NOT_READY - Master is not ready for to
*    start transfer.
*    A master still has not completed previous transaction or a slave
*    operation is in progress (in multi-master-slave configuration).
*    Nothing was sent on the bus. The attempt has to be retried.
*  - Jetty_I2C_MSTR_ERR_LB_NAK - Error condition: Last byte was
*    NAKed.
*  - Jetty_I2C_MSTR_ERR_ARB_LOST - Error condition: Master lost
*    arbitration.
*  - Jetty_I2C_MSTR_ERR_BUS_ERR - Error condition: Master
*    encountered a bus error. Bus error is misplaced start or stop detection.
*  - Jetty_I2C_MSTR_ERR_ABORT_START - Error condition: The start
*    condition generation was aborted due to beginning of Slave operation.
*    This error condition is only applicable for Multi-Master-Slave mode.
*
* \globalvars
* Jetty_state - used to store current state of software FSM.
*
*******************************************************************************/
uint32 Jetty_I2CMasterSendStart(uint32 slaveAddress, uint32 bitRnW, uint32 timeoutMs)
{
    uint32 errStatus = Jetty_I2C_MSTR_NOT_READY;

    if(Jetty_CHECK_I2C_FSM_IDLE)
    {
        Jetty_DisableInt();  /* Lock from interruption */

        if(Jetty_CHECK_I2C_STATUS(Jetty_I2C_STATUS_BUS_BUSY))
        {
            errStatus = Jetty_I2C_MSTR_BUS_BUSY;
        }
        else
        {
            uint32 locStatus;
            uint32 timeout = Jetty_I2C_CONVERT_TIMEOUT_TO_US(timeoutMs);

            /* Non-empty for master-slave mode when wakeup enabled */
            Jetty_I2CMasterDisableEcAm();

            /* Clean up the hardware before a transaction. */
            Jetty_SetMasterInterruptMode    (Jetty_NO_INTR_SOURCES);
            Jetty_ClearMasterInterruptSource(Jetty_INTR_MASTER_ALL);
            Jetty_ClearRxInterruptSource    (Jetty_INTR_RX_ALL);

            slaveAddress = Jetty_GET_I2C_8BIT_ADDRESS(slaveAddress);

            if(0u == bitRnW) /* Write direction */
            {
                Jetty_state = Jetty_I2C_FSM_MSTR_WR_DATA;
            }
            else             /* Read direction */
            {
                Jetty_state = Jetty_I2C_FSM_MSTR_RD_DATA;
                         slaveAddress |= Jetty_I2C_READ_FLAG;
            }

            /* TX and RX FIFO have to be empty at here */

             /* Generate Start and send address */
            Jetty_TX_FIFO_WR_REG = slaveAddress;
            Jetty_I2C_MASTER_GENERATE_START;

            /* Wait until address has been transferred. Note that for master reads at least one byte. */
            do
            {
                locStatus  = Jetty_GetMasterInterruptSource() & Jetty_INTR_MASTER_SEND_BYTE;
                locStatus |= Jetty_GetSlaveInterruptSource() & Jetty_SLAVE_INTR_I2C_ADDR;
                locStatus |= Jetty_I2CMasterWaitOneUnit(&timeout);
            }
            while (0u == locStatus);

            /* Handle completion event and convert to status */
            errStatus = Jetty_I2CMasterHandleStatus(locStatus);
        }

        Jetty_EnableInt();  /* Release lock */
    }

    return(errStatus);
}


/*******************************************************************************
* Function Name: Jetty_I2CMasterSendRestart
****************************************************************************//**
*
* Generates Restart condition and sends slave address with read/write bit.
* This function is blocking and does not return until start condition and
* address are sent and ACK/NACK response is received or errors occurred.
*
* \param slaveAddress
* Right justified 7-bit Slave address (valid range 8 to 120).
*
* \param bitRnW
* Direction of the following transfer. It is defined by read/write bit within
* address byte.See Jetty_I2CMasterSendStart() function for constants.
*
* \param timeoutMs
* Defines in milliseconds the time that this function can block for.
* If that time expires, the function returns. If a zero is passed,
* the function waits forever for the action to complete. If a timeout occurs,
* the SCB block is reset. Note The maximum value is (maximum uint32)/1000.
*
* \return
* Error status.
* See Jetty_I2CMasterSendStart() function for constants.
*
* \sideeffect
* A valid Start or ReStart condition must be generated before calling
* this function. This function does nothing if Start or ReStart conditions
* failed before this function was called.
* For read transaction, at least one byte has to be read before ReStart
* generation.
*
* \globalvars
* Jetty_state - used to store current state of software FSM.
*
*******************************************************************************/
uint32 Jetty_I2CMasterSendRestart(uint32 slaveAddress, uint32 bitRnW, uint32 timeoutMs)
{
    uint32 errStatus = Jetty_I2C_MSTR_NOT_READY;

    /* Check FSM state before generating ReStart condition */
    if(Jetty_CHECK_I2C_MASTER_ACTIVE)
    {
        uint32 locStatus;
        uint32 timeout = Jetty_I2C_CONVERT_TIMEOUT_TO_US(timeoutMs);

        slaveAddress = Jetty_GET_I2C_8BIT_ADDRESS(slaveAddress);

        if(0u == bitRnW) /* Write direction */
        {
            Jetty_state = Jetty_I2C_FSM_MSTR_WR_DATA;
        }
        else             /* Read direction */
        {
            Jetty_state  = Jetty_I2C_FSM_MSTR_RD_DATA;
                      slaveAddress |= Jetty_I2C_READ_FLAG;
        }

        /* TX and RX FIFO have to be empty at here */

        /* A proper ReStart sequence is: set command to generate ReStart, then put an address byte in the TX FIFO.
        * Put address into the TX FIFO the 1st makes scb IP think that this data byte for current write transaction.
        */
        Jetty_I2C_MASTER_GENERATE_RESTART;
        Jetty_TX_FIFO_WR_REG = slaveAddress;

        /* Wait until address has been transferred. Note that for master reads at least one byte. */
        do
        {
            locStatus  = Jetty_GetMasterInterruptSource() & Jetty_INTR_MASTER_SEND_BYTE;
            locStatus |= Jetty_I2CMasterWaitOneUnit(&timeout);
        }
        while (0u == locStatus);


        /* Handle completion event and convert to status */
        errStatus = Jetty_I2CMasterHandleStatus(locStatus);
    }

    return(errStatus);
}


/*******************************************************************************
* Function Name: Jetty_I2CMasterSendStop
****************************************************************************//**
*
* Generates Stop condition on the bus.
* The NAK is generated before Stop in case of a read transaction.
* At least one byte has to be read if a Start or ReStart condition with read
* direction was generated before.
* This function is blocking and does not return until a Stop condition is
* generated or error occurred.
*
* \param timeoutMs
* Defines in milliseconds the time that this function can block for.
* If that time expires, the function returns. If a zero is passed,
* the function waits forever for the action to complete. If a timeout occurs,
* the SCB block is reset. Note The maximum value is (maximum uint32)/1000.
*
* \return
* Error status.
* See Jetty_I2CMasterSendStart() function for constants.
*
* \sideeffect
* A valid Start or ReStart condition must be generated before calling
* this function. This function does nothing if Start or ReStart conditions
* failed before this function was called.
* For read transaction, at least one byte has to be read before ReStart
* generation.
*
* \globalvars
* Jetty_state - used to store current state of software FSM.
*
*******************************************************************************/
uint32 Jetty_I2CMasterSendStop(uint32 timeoutMs)
{
    uint32 errStatus = Jetty_I2C_MSTR_NOT_READY;

    /* Check FSM state before generating Stop condition */
    if(Jetty_CHECK_I2C_MASTER_ACTIVE)
    {
        uint32 locStatus;
        uint32 timeout = Jetty_I2C_CONVERT_TIMEOUT_TO_US(timeoutMs);

        /* Write direction: generates Stop; Read  direction: generates NACK and Stop; */
        Jetty_I2C_MASTER_GENERATE_STOP;

        /* Wait for a completion event from the master */
        do
        {
            locStatus  = Jetty_GetMasterInterruptSource() & Jetty_INTR_MASTER_SEND_STOP;
            locStatus |= Jetty_I2CMasterWaitOneUnit(&timeout);
        }
        while (0u == locStatus);

        /* Handle completion event and convert to status */
        errStatus = Jetty_I2CMasterHandleStatus(locStatus);
    }

    return(errStatus);
}


/*******************************************************************************
* Function Name: Jetty_I2CMasterWriteByte
****************************************************************************//**
*
* Sends one byte to a slave.
* This function is blocking and does not return until byte is transmitted
* or error occurred.
*
* \param wrByte
* The data byte to send to the slave.
*
* \param timeoutMs
* Defines in milliseconds the time that this function can block for.
* If that time expires, the function returns. If a zero is passed,
* the function waits forever for the action to complete. If a timeout occurs,
* the SCB block is reset. Note The maximum value is (maximum uint32)/1000.
*
* \return
* Error status.
* See Jetty_I2CMasterSendStart() function for constants.
*
* \sideeffect
* A valid Start or ReStart condition must be generated before calling
* this function. This function does nothing if Start or ReStart condition
* failed before this function was called.
*
* \globalvars
* Jetty_state - used to store current state of software FSM.
*
*******************************************************************************/
uint32 Jetty_I2CMasterWriteByte(uint32 wrByte, uint32 timeoutMs)
{
    uint32 errStatus = Jetty_I2C_MSTR_NOT_READY;

    /* Check FSM state before write byte */
    if(Jetty_CHECK_I2C_MASTER_ACTIVE)
    {
        uint32 locStatus;
        uint32 timeout = Jetty_I2C_CONVERT_TIMEOUT_TO_US(timeoutMs);

        Jetty_TX_FIFO_WR_REG = wrByte;

        /* Wait for a completion event from the master */
        do
        {
            locStatus  = Jetty_GetMasterInterruptSource() & Jetty_INTR_MASTER_SEND_BYTE;
            locStatus |= Jetty_I2CMasterWaitOneUnit(&timeout);
        }
        while (0u == locStatus);

        /* Handle completion event and convert to status */
        errStatus = Jetty_I2CMasterHandleStatus(locStatus);
    }

    return(errStatus);
}



/*******************************************************************************
* Function Name: Jetty_I2CMasterReadByte
****************************************************************************//**
*
* Reads one byte from a slave and generates ACK or prepares to generate NAK.
* The NAK will be generated before Stop or ReStart condition by
* Jetty_I2CMasterSendStop() or
* Jetty_I2CMasterSendRestart() function appropriately.
* This function is blocking. It does not return until a byte is received or
* an error occurs.
*
* \param ackNack
* Response to received byte.
*  - Jetty_I2C_ACK_DATA - Generates ACK.
*     The master notifies slave that transfer continues.
*  - Jetty_I2C_NAK_DATA - Prepares to generate NAK.
*     The master will notify slave that transfer is completed.
*
* \param rdByte
* The pointer to the location to store the data byte that was read from
* the slave.
* Note that the byte should be ignored if error status is returned.
*
* \param timeoutMs
* Defines in milliseconds the time that this function can block for.
* If that time expires, the function returns. If a zero is passed,
* the function waits forever for the action to complete. If a timeout occurs,
* the SCB block is reset. Note The maximum value is (maximum uint32)/1000.
*
* \return
* Error status.
* See Jetty_I2CMasterSendStart() function for constants.
*
* \sideeffect
* A valid Start or ReStart condition must be generated before calling
* this function. This function does nothing if Start or ReStart condition
* failed before this function was called.
*
* \globalvars
* Jetty_state - used to store current state of software FSM.
*
*******************************************************************************/
uint32 Jetty_I2CMasterReadByte(uint32 ackNack, uint8 *rdByte, uint32 timeoutMs)
{
    uint32 errStatus = Jetty_I2C_MSTR_NOT_READY;

    /* Check FSM state before read byte */
    if(Jetty_CHECK_I2C_MASTER_ACTIVE)
    {
        uint32 locStatus;
        uint32 rxNotEmpty;
        uint32 timeout = Jetty_I2C_CONVERT_TIMEOUT_TO_US(timeoutMs);

        /* Wait for a completion event from the master */
        do
        {
            locStatus  = Jetty_GetMasterInterruptSource() & Jetty_INTR_MASTER_RECEIVE_BYTE;
            rxNotEmpty = Jetty_GetRxInterruptSource() & Jetty_INTR_RX_NOT_EMPTY;
            locStatus |= Jetty_I2CMasterWaitOneUnit(&timeout);
        }
        while ((0u == locStatus) && (0u == rxNotEmpty));

        /* Get byte from RX FIFO */
        if (0u != rxNotEmpty)
        {
            *rdByte = (uint8)Jetty_RX_FIFO_RD_REG;
            Jetty_ClearRxInterruptSource(Jetty_INTR_RX_ALL);
        }

        /* Handle completion event and convert to status */
        errStatus = Jetty_I2CMasterHandleStatus(locStatus);

        if (Jetty_I2C_MSTR_NO_ERROR == errStatus)
        {
            if(Jetty_I2C_ACK_DATA == ackNack)
            {
                Jetty_I2C_MASTER_GENERATE_ACK;
            }
            else
            {
                /* NACK is generated by Stop or ReStart command */
            }
        }
    }

    return(errStatus);
}


/*******************************************************************************
* Function Name: Jetty_I2CMasterGetReadBufSize
****************************************************************************//**
*
*  Returns the number of bytes that has been transferred with an
*  Jetty_I2CMasterReadBuf() function.
*
* \return
*  Byte count of transfer. If the transfer is not yet complete, it returns
*  the byte count transferred so far.
*
* \sideeffect
*  This function returns not valid value if
*  Jetty_I2C_MSTAT_ERR_ARB_LOST or
*  Jetty_I2C_MSTAT_ERR_BUS_ERROR occurred while read transfer.
*
* \globalvars
*  Jetty_mstrRdBufIndex - used to current index within master read
*  buffer.
*
*******************************************************************************/
uint32 Jetty_I2CMasterGetReadBufSize(void)
{
    return(Jetty_mstrRdBufIndex);
}


/*******************************************************************************
* Function Name: Jetty_I2CMasterGetWriteBufSize
****************************************************************************//**
*
*  Returns the number of bytes that have been transferred with an
*  SCB_I2CMasterWriteBuf() function.
*
* \return
*   Byte count of transfer. If the transfer is not yet complete, it returns
*   zero unit transfer completion.
*
* \sideeffect
*   This function returns not valid value if
*   Jetty_I2C_MSTAT_ERR_ARB_LOST or
*   Jetty_I2C_MSTAT_ERR_BUS_ERROR occurred while read transfer.
*
* \globalvars
*  Jetty_mstrWrBufIndex - used to current index within master write
*  buffer.
*
*******************************************************************************/
uint32 Jetty_I2CMasterGetWriteBufSize(void)
{
    return(Jetty_mstrWrBufIndex);
}


/*******************************************************************************
* Function Name: Jetty_I2CMasterClearReadBuf
****************************************************************************//**
*
*  Resets the read buffer pointer back to the first byte in the buffer.
*
* \globalvars
*  Jetty_mstrRdBufIndex - used to current index within master read
*   buffer.
*  Jetty_mstrStatus - used to store current status of I2C Master.
*
*******************************************************************************/
void Jetty_I2CMasterClearReadBuf(void)
{
    Jetty_DisableInt();  /* Lock from interruption */

    Jetty_mstrRdBufIndex = 0u;
    Jetty_mstrStatus    &= (uint16) ~Jetty_I2C_MSTAT_RD_CMPLT;

    Jetty_EnableInt();   /* Release lock */
}


/*******************************************************************************
* Function Name: Jetty_I2CMasterClearWriteBuf
****************************************************************************//**
*
*  Resets the write buffer pointer back to the first byte in the buffer.
*
* \globalvars
*  Jetty_mstrRdBufIndex - used to current index within master read
*   buffer.
*  Jetty_mstrStatus - used to store current status of I2C Master.
*
*******************************************************************************/
void Jetty_I2CMasterClearWriteBuf(void)
{
    Jetty_DisableInt();  /* Lock from interruption */

    Jetty_mstrWrBufIndex = 0u;
    Jetty_mstrStatus    &= (uint16) ~Jetty_I2C_MSTAT_WR_CMPLT;

    Jetty_EnableInt();   /* Release lock */
}


/*******************************************************************************
* Function Name: Jetty_I2CMasterStatus
****************************************************************************//**
*
*  Returns the master's communication status.
*
* \return
*  Current status of I2C master. This status incorporates status constants.
*  Each constant is a bit field value. The value returned may have multiple
*  bits set to indicate the status of the read or write transfer.
*  - Jetty_I2C_MSTAT_RD_CMPLT - Read transfer complete.
*    The error condition status bits must be checked to ensure that
*    read transfer was completed successfully.
*  - Jetty_I2C_MSTAT_WR_CMPLT - Write transfer complete.
*    The error condition status bits must be checked to ensure that write
*    transfer was completed successfully.
*  - Jetty_I2C_MSTAT_XFER_INP - Transfer in progress.
*  - Jetty_I2C_MSTAT_XFER_HALT - Transfer has been halted.
*    The I2C bus is waiting for ReStart or Stop condition generation.
*  - Jetty_I2C_MSTAT_ERR_SHORT_XFER - Error condition: Write
*    transfer completed before all bytes were transferred. The slave NAKed
*    the byte which was expected to be ACKed.
*  - Jetty_I2C_MSTAT_ERR_ADDR_NAK - Error condition: Slave did
*    not acknowledge address.
*  - Jetty_I2C_MSTAT_ERR_ARB_LOST - Error condition: Master lost
*    arbitration during communications with slave.
*  - Jetty_I2C_MSTAT_ERR_BUS_ERROR - Error condition: bus error
*    occurred during master transfer due to misplaced Start or Stop
*    condition on the bus.
*  - Jetty_I2C_MSTAT_ERR_ABORT_XFER - Error condition: Slave was
*    addressed by another master while master performed the start condition
*    generation. As a result, master has automatically switched to slave
*    mode and is responding. The master transaction has not taken place
*    This error condition only applicable for Multi-Master-Slave mode.
*  - Jetty_I2C_MSTAT_ERR_XFER - Error condition: This is the
*    ORed value of all error conditions provided above.
*
* \globalvars
*  Jetty_mstrStatus - used to store current status of I2C Master.
*
*******************************************************************************/
uint32 Jetty_I2CMasterStatus(void)
{
    uint32 status;

    Jetty_DisableInt();  /* Lock from interruption */

    status = (uint32) Jetty_mstrStatus;

    if (Jetty_CHECK_I2C_MASTER_ACTIVE)
    {
        /* Add status of master pending transaction: MSTAT_XFER_INP */
        status |= (uint32) Jetty_I2C_MSTAT_XFER_INP;
    }

    Jetty_EnableInt();   /* Release lock */

    return(status);
}


/*******************************************************************************
* Function Name: Jetty_I2CMasterClearStatus
****************************************************************************//**
*
*  Clears all status flags and returns the master status.
*
* \return
*  Current status of master. See the Jetty_I2CMasterStatus()
*  function for constants.
*
* \globalvars
*  Jetty_mstrStatus - used to store current status of I2C Master.
*
*******************************************************************************/
uint32 Jetty_I2CMasterClearStatus(void)
{
    uint32 status;

    Jetty_DisableInt();  /* Lock from interruption */

    /* Read and clear master status */
    status = (uint32) Jetty_mstrStatus;
    Jetty_mstrStatus = Jetty_I2C_MSTAT_CLEAR;

    Jetty_EnableInt();   /* Release lock */

    return(status);
}


/*******************************************************************************
* Function Name: Jetty_I2CReStartGeneration
****************************************************************************//**
*
*  Generates a ReStart condition:
*  - SCB IP V1 and later: Generates ReStart using the scb IP functionality
*    Sets the I2C_MASTER_CMD_M_START and I2C_MASTER_CMD_M_NACK (if the previous
*    transaction was read) bits in the SCB.I2C_MASTER_CMD register.
*    This combination forces the master to generate ReStart.
*
*  - SCB IP V0: Generates Restart using the GPIO and scb IP functionality.
*    After the master completes write or read, the SCL is stretched.
*    The master waits until SDA line is released by the slave. Then the GPIO
*    function is enabled and the scb IP disabled as it already does not drive
*    the bus. In case of the previous transfer was read, the NACK is generated
*    by the GPIO. The delay of tLOW is added to manage the hold time.
*    Set I2C_M_CMD.START and enable the scb IP. The ReStart generation
*    is started after the I2C function is enabled for the SCL.
*    Note1: the scb IP due re-enable generates Start but on the I2C bus it
*           appears as ReStart.
*    Note2: the I2C_M_CMD.START is queued if scb IP is disabled.
*    Note3: the I2C_STATUS_M_READ is cleared is address was NACKed before.
*
* \sideeffect
*  SCB IP V0: The NACK generation by the GPIO may cause a greater SCL period
*             than expected for the selected master data rate.
*
*******************************************************************************/
void Jetty_I2CReStartGeneration(void)
{
#if(Jetty_CY_SCBIP_V0)
    /* Generates Restart use GPIO and scb IP functionality. Ticket ID#143715,
    * ID#145238 and ID#173656.
    * In case of timeout the Jetty_I2CMasterSendRestart handles this
    * case using own timeout and execute appropriate recovery.
    */

    const uint16 oneUnit = 1u;
    uint32 timeout = Jetty_I2C_PHASE_GEN_TIMEOUT;

    /* Store direction of transfer */
    uint32 readDirection = Jetty_I2C_STATUS_REG & Jetty_I2C_STATUS_M_READ;

    /* Wait until slave releases SDA line: SCL tHIGH is complete */
    while ((Jetty_WAIT_SDA_SET_HIGH) && (timeout > 0u))
    {
        CyDelayUs(oneUnit);
        --timeout;
    }

    if (timeout > 0u)
    {
        /* Refill timeout */
        timeout = Jetty_I2C_PHASE_GEN_TIMEOUT;

        /* Drive SCL line use GPIO */
        Jetty_SET_I2C_SCL_DR       (Jetty_I2C_SCL_LOW);
        Jetty_SET_I2C_SCL_HSIOM_SEL(Jetty_HSIOM_GPIO_SEL);

        /* Disable SCB block */
        Jetty_CTRL_REG &= (uint32) ~Jetty_CTRL_ENABLED;

        if (0u != readDirection)
        {
            /* Generate NACK for read transactions use GPIO functionality */

            /* Drive tLOW: SCL low phase */
            Jetty_SET_I2C_SCL_DR(Jetty_I2C_SCL_LOW);
            CyDelayUs(Jetty_I2C_TLOW_TIME);

            /* Drive tHIGH: SCL high phase */
            Jetty_SET_I2C_SCL_DR(Jetty_I2C_SCL_HIGH);
            CyDelayUs(Jetty_I2C_THIGH_TIME);

            /* Wait until slave releases SDA line: SCL tHIGH is complete */
            while ((Jetty_WAIT_SCL_SET_HIGH) && (timeout > 0u))
            {
                CyDelayUs(oneUnit);
                --timeout;
            }
        }

        if (timeout > 0u)
        {
            /* Drive tLOW: hold time for write and read */
            Jetty_SET_I2C_SCL_DR(Jetty_I2C_SCL_LOW);
            CyDelayUs(Jetty_I2C_TLOW_TIME);

            /* Set Start command, it will appear as ReStart on the bus */
            Jetty_I2C_MASTER_CMD_REG  = Jetty_I2C_MASTER_CMD_M_START;
        }

        /* Enable SCB block: ReStart will appear on the bus as soon as SCL
        * configuration is restored and bus busy cleared.
        */
        Jetty_CTRL_REG |= Jetty_CTRL_ENABLED;

        /* Restore SCL configuration */
        Jetty_SET_I2C_SCL_HSIOM_SEL(Jetty_HSIOM_I2C_SEL);
        Jetty_SET_I2C_SCL_DR       (Jetty_I2C_SCL_HIGH);
    }
#else
    uint32 cmd;

    /* Generates ReStart use scb IP functionality */
    cmd  = Jetty_I2C_MASTER_CMD_M_START;
    cmd |= Jetty_CHECK_I2C_STATUS(Jetty_I2C_STATUS_M_READ) ?
                (Jetty_I2C_MASTER_CMD_M_NACK) : (0u);

    Jetty_I2C_MASTER_CMD_REG = cmd;
#endif /* (Jetty_CY_SCBIP_V1) */
}

#endif /* (Jetty_I2C_MASTER_CONST) */


#if (!Jetty_CY_SCBIP_V0 && \
    Jetty_I2C_MULTI_MASTER_SLAVE_CONST && Jetty_I2C_WAKE_ENABLE_CONST)
    /*******************************************************************************
    * Function Name: Jetty_I2CMasterDisableEcAm
    ****************************************************************************//**
    *
    *  Disables externally clocked address match logic to enable master operation
    *  in active mode.
    *
    *******************************************************************************/
    static void Jetty_I2CMasterDisableEcAm(void)
    {
        /* Disables externally clocked address match logic to enable master operation
        * in active mode. This applicable only for Multi-Master-Slave when wakeup is
        * enabled. Ticket ID#192742.
        */
        if (0u != (Jetty_CTRL_REG & Jetty_CTRL_EC_AM_MODE))
        {
            /* Enable external address match logic */
            Jetty_Stop();
            Jetty_CTRL_REG &= (uint32) ~Jetty_CTRL_EC_AM_MODE;
            Jetty_Enable();
        }
    }
#endif /* (!Jetty_CY_SCBIP_V0) */


/*******************************************************************************
* Function Name: Jetty_I2CMasterHandleStatus
****************************************************************************//**
*
* Converts hardware status to firmware status and executes required processing.
*
* \param status
* Hardware I2C master status to handle. It also includes
* Jetty_I2C_MASTER_TIMEOUT and Jetty_SLAVE_INTR_I2C_ADDR.
*
* \return
* Error status.
* See Jetty_I2CMasterSendStart() function for constants.
*
*******************************************************************************/
static uint32 Jetty_I2CMasterHandleStatus(uint32 status)
{
    uint32 retStatus;
    uint32 resetBlock = 0u;

    if (0u != (status & Jetty_I2C_MASTER_TIMEOUT))
    {
        retStatus  = Jetty_I2C_MASTER_TIMEOUT;
        resetBlock = Jetty_I2C_RESET_ERROR;
    }
    else if (0u != (Jetty_SLAVE_INTR_I2C_ADDR & status))
    {
        /* Abort the master operation, the slave was addressed first */
        retStatus = Jetty_I2C_MSTR_ERR_ABORT_START;

        Jetty_I2C_MASTER_CMD_REG = 0u;

        Jetty_state = Jetty_I2C_FSM_IDLE;
    }
    else if (0u != (status & Jetty_INTR_MASTER_I2C_ARB_LOST))
    {
        retStatus  = Jetty_I2C_MSTR_ERR_ARB_LOST;

        if (Jetty_CHECK_I2C_FSM_ADDR)
        {
            /* If slave is not enabled reset IP, otherwise give it a chance to match address */
            if (0u == (Jetty_I2C_CTRL_REG & Jetty_I2C_CTRL_SLAVE_MODE))
            {
                resetBlock = Jetty_I2C_RESET_ERROR;
            }

            Jetty_state = Jetty_I2C_FSM_IDLE;
        }
        else
        {
            /* Reset IP block when on address stage */
            resetBlock = Jetty_I2C_RESET_ERROR;
        }
    }
    else if (0u != (status & Jetty_INTR_MASTER_I2C_BUS_ERROR))
    {
        retStatus  = Jetty_I2C_MSTR_ERR_BUS_ERR;
        resetBlock = Jetty_I2C_RESET_ERROR;
    }
    else if (0u != (status & Jetty_INTR_MASTER_I2C_NACK))
    {
        retStatus = Jetty_I2C_MSTR_ERR_LB_NAK;
    }
    else /* Includes: INTR_MASTER_I2C_STOP and INTR_MASTER_I2C_ACK */
    {
        retStatus = Jetty_I2C_MSTR_NO_ERROR;

        if (0u != (status & Jetty_INTR_MASTER_I2C_STOP))
        {
            /* Stop: end of transaction, go to idle state */
            Jetty_state = Jetty_I2C_FSM_IDLE;
        }
        else
        {
            /* ACK: continue transaction */
            if (Jetty_CHECK_I2C_FSM_ADDR)
            {
                /* Move from address state to data */
                Jetty_state = (Jetty_CHECK_I2C_FSM_RD) ?
                                          Jetty_I2C_FSM_MSTR_RD_DATA : Jetty_I2C_FSM_MSTR_WR_DATA;
            }
        }
    }

    if (Jetty_I2C_RESET_ERROR == resetBlock)
    {
        Jetty_I2CFwBlockReset();
    }
    else
    {
        Jetty_ClearMasterInterruptSource(Jetty_INTR_MASTER_ALL);
    }

    return (retStatus);
}


/******************************************************************************
* Function Name: I2CMasterWaitOneUnit
****************************************************************************//**
*
* Waits for one unit before unblock code execution.
* Note If a timeout value is 0, this function does nothing and returns 0.
*
* \param timeout
* The pointer to a timeout value.
*
* \return
* Returns 0 if a timeout does not expire or the timeout mask.
*
*******************************************************************************/
static uint32 Jetty_I2CMasterWaitOneUnit(uint32 *timeout)
{
    const uint16 oneUnit = 1u;
    uint32 status = 0u;

    /* If the timeout equal to 0. Ignore the timeout */
    if (*timeout > 0u)
    {
        CyDelayUs(oneUnit);
        --(*timeout);

        if (0u == *timeout)
        {
            status = Jetty_I2C_MASTER_TIMEOUT;
        }
    }

    return (status);
}




/* [] END OF FILE */
