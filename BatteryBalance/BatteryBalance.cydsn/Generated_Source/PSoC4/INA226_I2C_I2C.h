/***************************************************************************//**
* \file INA226_I2C_I2C.h
* \version 4.0
*
* \brief
*  This file provides constants and parameter values for the SCB Component in
*  the I2C mode.
*
* Note:
*
********************************************************************************
* \copyright
* Copyright 2013-2017, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#if !defined(CY_SCB_I2C_INA226_I2C_H)
#define CY_SCB_I2C_INA226_I2C_H

#include "INA226_I2C.h"


/***************************************
*   Initial Parameter Constants
****************************************/

#define INA226_I2C_I2C_MODE                   (2u)
#define INA226_I2C_I2C_OVS_FACTOR_LOW         (8u)
#define INA226_I2C_I2C_OVS_FACTOR_HIGH        (8u)
#define INA226_I2C_I2C_MEDIAN_FILTER_ENABLE   (1u)
#define INA226_I2C_I2C_SLAVE_ADDRESS          (0x8u)
#define INA226_I2C_I2C_SLAVE_ADDRESS_MASK     (0xFEu)
#define INA226_I2C_I2C_ACCEPT_ADDRESS         (0u)
#define INA226_I2C_I2C_ACCEPT_GENERAL_CALL    (0u)
#define INA226_I2C_I2C_WAKE_ENABLE            (0u)
#define INA226_I2C_I2C_DATA_RATE              (100u)
#define INA226_I2C_I2C_DATA_RATE_ACTUAL       (100u)
#define INA226_I2C_I2C_CLOCK_FROM_TERM        (0u)
#define INA226_I2C_I2C_EXTERN_INTR_HANDLER    (0u)
#define INA226_I2C_I2C_BYTE_MODE_ENABLE       (0u)
#define INA226_I2C_I2C_MANUAL_OVS_CONTROL     (1u)


/***************************************
*  Conditional Compilation Parameters
****************************************/

/* I2C sub mode enum */
#define INA226_I2C_I2C_MODE_SLAVE              (0x01u)
#define INA226_I2C_I2C_MODE_MASTER             (0x02u)
#define INA226_I2C_I2C_MODE_MULTI_MASTER       (0x06u)
#define INA226_I2C_I2C_MODE_MULTI_MASTER_SLAVE (0x07u)
#define INA226_I2C_I2C_MODE_MULTI_MASTER_MASK  (0x04u)

#if (INA226_I2C_SCB_MODE_UNCONFIG_CONST_CFG)

    /* Returns true if slave mode is enabled */
    #define INA226_I2C_I2C_SLAVE  (0u != (INA226_I2C_I2C_MODE_SLAVE & INA226_I2C_mode))

    /* Returns true if master mode is enabled */
    #define INA226_I2C_I2C_MASTER (0u != (INA226_I2C_I2C_MODE_MASTER & INA226_I2C_mode))

    /* Returns true if multi-master mode is enabled */
    #define INA226_I2C_I2C_MULTI_MASTER \
                            (0u != (INA226_I2C_I2C_MODE_MULTI_MASTER_MASK & INA226_I2C_mode))

    /* Returns true if mode is multi-master-slave */
    #define INA226_I2C_I2C_MULTI_MASTER_SLAVE \
                            (INA226_I2C_I2C_MODE_MULTI_MASTER_SLAVE == INA226_I2C_mode)

    /* Returns true if address accepts in RX FIFO */
    #define INA226_I2C_CHECK_I2C_ACCEPT_ADDRESS   (0u != INA226_I2C_acceptAddr)
    #define INA226_I2C_CHECK_I2C_GENERAL_CALL \
                (0u != (INA226_I2C_I2C_CTRL_REG & INA226_I2C_I2C_CTRL_S_GENERAL_IGNORE))

    #define INA226_I2C_I2C_WAKE_ENABLE_CONST              (1u)
    #define INA226_I2C_I2C_SLAVE_CONST                    (1u)
    #define INA226_I2C_I2C_MASTER_CONST                   (1u)
    #define INA226_I2C_I2C_MULTI_MASTER_SLAVE_CONST       (1u)
    #define INA226_I2C_CHECK_I2C_ACCEPT_ADDRESS_CONST     (1u)
    #define INA226_I2C_CHECK_I2C_GENERAL_CALL_CONST       (1u)
    #define INA226_I2C_I2C_CUSTOM_ADDRESS_HANDLER_CONST   (1u)

    /* Returns FIFO size */
    #if (INA226_I2C_CY_SCBIP_V0 || INA226_I2C_CY_SCBIP_V1)
        #define INA226_I2C_I2C_FIFO_SIZE      (INA226_I2C_FIFO_SIZE)
    #else
        #define INA226_I2C_I2C_FIFO_SIZE      (INA226_I2C_GET_FIFO_SIZE(INA226_I2C_CTRL_REG & \
                                                                                    INA226_I2C_CTRL_BYTE_MODE))
    #endif /* (INA226_I2C_CY_SCBIP_V0 || INA226_I2C_CY_SCBIP_V1) */

#else

    /* Returns true if slave mode is enabled */
    #define INA226_I2C_I2C_SLAVE   (0u != (INA226_I2C_I2C_MODE_SLAVE & INA226_I2C_I2C_MODE))

    /* Returns true if master mode is enabled */
    #define INA226_I2C_I2C_MASTER  (0u != (INA226_I2C_I2C_MODE_MASTER & INA226_I2C_I2C_MODE))

    /* Returns true if multi-master mode is enabled */
    #define INA226_I2C_I2C_MULTI_MASTER \
                                    (0u != (INA226_I2C_I2C_MODE_MULTI_MASTER_MASK & INA226_I2C_I2C_MODE))

    /* Returns true if mode is multi-master-slave */
    #define INA226_I2C_I2C_MULTI_MASTER_SLAVE \
                                    (INA226_I2C_I2C_MODE_MULTI_MASTER_SLAVE == INA226_I2C_I2C_MODE)

    /* Returns true if address accepts in RX FIFO */
    #define INA226_I2C_CHECK_I2C_ACCEPT_ADDRESS   (0u != INA226_I2C_I2C_ACCEPT_ADDRESS)
    #define INA226_I2C_CHECK_I2C_GENERAL_CALL     (0u != INA226_I2C_I2C_ACCEPT_GENERAL_CALL)

    /* Returns true if wakeup on address match is enabled */
    #define INA226_I2C_I2C_WAKE_ENABLE_CONST  (0u != INA226_I2C_I2C_WAKE_ENABLE)

    #define INA226_I2C_I2C_SLAVE_CONST    (INA226_I2C_I2C_SLAVE)
    #define INA226_I2C_I2C_MASTER_CONST   (INA226_I2C_I2C_MASTER)
    #define INA226_I2C_I2C_MULTI_MASTER_SLAVE_CONST   (INA226_I2C_I2C_MULTI_MASTER_SLAVE)
    #define INA226_I2C_CHECK_I2C_ACCEPT_ADDRESS_CONST (INA226_I2C_CHECK_I2C_ACCEPT_ADDRESS)
    #define INA226_I2C_CHECK_I2C_GENERAL_CALL_CONST   (INA226_I2C_CHECK_I2C_GENERAL_CALL)
    #define INA226_I2C_I2C_CUSTOM_ADDRESS_HANDLER_CONST   (INA226_I2C_CHECK_I2C_ACCEPT_ADDRESS_CONST || \
                                                                  INA226_I2C_CHECK_I2C_GENERAL_CALL_CONST)

    /* I2C: TX or RX FIFO size */
    #define INA226_I2C_I2C_FIFO_SIZE  INA226_I2C_GET_FIFO_SIZE(INA226_I2C_I2C_BYTE_MODE_ENABLE)

    /* Adjust AF and DF filter settings. Ticket ID#176179 */
    #if ((INA226_I2C_I2C_MODE_SLAVE == INA226_I2C_I2C_MODE) ||     \
            ((INA226_I2C_I2C_MODE_SLAVE != INA226_I2C_I2C_MODE) && \
             (INA226_I2C_I2C_DATA_RATE_ACTUAL <= INA226_I2C_I2C_DATA_RATE_FS_MODE_MAX)))

        #define INA226_I2C_I2C_MEDIAN_FILTER_ENABLE_ADJ       (0u)
        #define INA226_I2C_I2C_CFG_ANALOG_FITER_ENABLE_ADJ    do{;}while(0)
    #else
        #define INA226_I2C_I2C_MEDIAN_FILTER_ENABLE_ADJ       (1u)
        #define INA226_I2C_I2C_CFG_ANALOG_FITER_ENABLE_ADJ    INA226_I2C_I2C_CFG_ANALOG_FITER_DISABLE
    #endif

    /* Select oversampling factor low and high */
    #define INA226_I2C_I2C_OVS_FACTOR_LOW_MIN     ((0u != INA226_I2C_I2C_MANUAL_OVS_CONTROL) ? \
                                                            (8u) : (8u))

    #define INA226_I2C_I2C_OVS_FACTOR_HIGH_MIN    ((0u != INA226_I2C_I2C_MANUAL_OVS_CONTROL) ? \
                                                            (8u) : (8u))

#endif /* (INA226_I2C_SCB_MODE_UNCONFIG_CONST_CFG) */

#define INA226_I2C_I2C_CUSTOM_ADDRESS_HANDLER (INA226_I2C_CHECK_I2C_GENERAL_CALL || \
                                                     INA226_I2C_CHECK_I2C_ACCEPT_ADDRESS)


/***************************************
*       Type Definitions
***************************************/

/**
* \addtogroup group_structures
* @{
*/
typedef struct
{
    /** Mode of operation for I2C. The following defines are available choices:
     *  - INA226_I2C_I2C_MODE_SLAVE
     *  - INA226_I2C_I2C_MODE_MASTER
     *  - INA226_I2C_I2C_MODE_MULTI_MASTER
     *  - INA226_I2C_I2C_MODE_MULTI_MASTER_SLAVE
    */
    uint32 mode;

    /** Oversampling factor for the low phase of the I2C clock. Ignored for
     *  Slave mode operation.  The oversampling factors need to be chosen in
     *  conjunction with the clock rate in order to generate the desired rate
     *  of I2C operation.
    */
    uint32 oversampleLow;

    /** Oversampling factor for the high phase of the I2C clock. Ignored for
     *  Slave mode operation.
    */
    uint32 oversampleHigh;

    /** This field is left for compatibility and its value is ignored.
     *  Median filter is enabled or disabled depends on the data rate and
     *  operation mode.
    */
    uint32 enableMedianFilter;

    /** 7-bit slave address. Ignored for non-slave modes.
    */
    uint32 slaveAddr;

    /** 8-bit slave address mask. Bit 0 must have a value of 0. Ignored for
     *  non-slave modes.
     *  - Bit value 0 – excludes bit from address comparison.
     *  - Bit value 1 – the bit needs to match with the corresponding bit
     *   of the device address.
    */
    uint32 slaveAddrMask;

    /** When enabled the matching address is received into the RX FIFO:
     *  0 – disable, 1 – enable.
     *
     *  The callback function has to be registered to handle the address
     *  accepted in the RX FIFO. Refer to section Accept matching address
     *  RX FIFO for more information.
    */
    uint32 acceptAddr;

    /** When enabled the matching address generates a wakeup request:
     *  0 – disable, 1 – enable. 1 – enable. Ignored for non-slave modes.
    */
    uint32 enableWake;

    /** When enabled the TX and RX FIFO depth is doubled and equal to
     *  16 bytes: 0 – disable, 1 – enable.
     *
     * Ignored for all devices other than PSoC 4100 BLE / PSoC 4200 BLE /
     * PSoC 4100M / PSoC 4200M / PSoC 4200L / PSoC 4000S / PSoC 4100S /
     * PSoC Analog Coprocessor.
    */
    uint8  enableByteMode;

    /** Data rate in kbps used while the of I2C is in operation. Valid values
      * are between 1 and 1000. Note that this filed must be initialized
      * for correct operation if Unconfigured SCB was utilized with previous
      * version of the component.
    */
    uint16 dataRate;

    /** When enabled the I2C general call address (0x00) will be accepted by
     *  the I2C hardware and trigger an interrupt: 0 – disable, 1 – enable.
     *
     *  The callback function has to be registered to handle a general call
     *  address. Refer to section Accept General Call for more information.
    */
    uint8  acceptGeneralAddr;
} INA226_I2C_I2C_INIT_STRUCT;
/** @} structures */

/***************************************
*        Function Prototypes
***************************************/

/**
* \addtogroup group_i2c
* @{
*/
/* Common functions */
#if(INA226_I2C_SCB_MODE_UNCONFIG_CONST_CFG)
    void INA226_I2C_I2CInit(const INA226_I2C_I2C_INIT_STRUCT *config);
#endif /* (INA226_I2C_SCB_MODE_UNCONFIG_CONST_CFG) */
/** @} i2c */

/**
* \addtogroup group_interrupt
* @{
*/
#if (INA226_I2C_I2C_CUSTOM_ADDRESS_HANDLER_CONST)
    void INA226_I2C_SetI2cAddressCustomInterruptHandler(uint32 (*func) (void));
#endif /* (INA226_I2C_I2C_CUSTOM_ADDRESS_HANDLER_CONST) */
/** @} interrupt */

/**
* \addtogroup group_i2c
* @{
*/
/* I2C Master functions prototypes */
#if(INA226_I2C_I2C_MASTER_CONST)
    /* Read and Clear status functions */
    uint32 INA226_I2C_I2CMasterStatus(void);
    uint32 INA226_I2C_I2CMasterClearStatus(void);

    /* Interrupt based operation functions */
    uint32 INA226_I2C_I2CMasterWriteBuf(uint32 slaveAddress, uint8 * wrData, uint32 cnt, uint32 mode);
    uint32 INA226_I2C_I2CMasterReadBuf(uint32 slaveAddress, uint8 * rdData, uint32 cnt, uint32 mode);
    uint32 INA226_I2C_I2CMasterGetReadBufSize(void);
    uint32 INA226_I2C_I2CMasterGetWriteBufSize(void);
    void   INA226_I2C_I2CMasterClearReadBuf(void);
    void   INA226_I2C_I2CMasterClearWriteBuf(void);

    /* Manual operation functions */
    uint32 INA226_I2C_I2CMasterSendStart(uint32 slaveAddress, uint32 bitRnW, uint32 timeoutMs);
    uint32 INA226_I2C_I2CMasterSendRestart(uint32 slaveAddress, uint32 bitRnW, uint32 timeoutMs);
    uint32 INA226_I2C_I2CMasterSendStop(uint32 timeoutMs);
    uint32 INA226_I2C_I2CMasterWriteByte(uint32 wrByte, uint32 timeoutMs);
    uint32 INA226_I2C_I2CMasterReadByte(uint32 ackNack, uint8* rdByte, uint32 timeoutMs);
#endif /* (INA226_I2C_I2C_MASTER_CONST) */

/* I2C Slave functions prototypes */
#if(INA226_I2C_I2C_SLAVE_CONST)
    /* Read and Clear status functions */
    uint32 INA226_I2C_I2CSlaveStatus(void);
    uint32 INA226_I2C_I2CSlaveClearReadStatus(void);
    uint32 INA226_I2C_I2CSlaveClearWriteStatus(void);

    /* Set Slave address and mask */
    void   INA226_I2C_I2CSlaveSetAddress(uint32 address);
    void   INA226_I2C_I2CSlaveSetAddressMask(uint32 addressMask);

    /* Interrupt based operation functions */
    void   INA226_I2C_I2CSlaveInitReadBuf(uint8 * rdBuf, uint32 bufSize);
    void   INA226_I2C_I2CSlaveInitWriteBuf(uint8 * wrBuf, uint32 bufSize);
    uint32 INA226_I2C_I2CSlaveGetReadBufSize(void);
    uint32 INA226_I2C_I2CSlaveGetWriteBufSize(void);
    void   INA226_I2C_I2CSlaveClearReadBuf(void);
    void   INA226_I2C_I2CSlaveClearWriteBuf(void);
#endif /* (INA226_I2C_I2C_SLAVE_CONST) */
/** @} i2c */

CY_ISR_PROTO(INA226_I2C_I2C_ISR);


/***************************************
*            API Constants
***************************************/

/* I2C sub mode enum */
#define INA226_I2C_I2C_MODE_SLAVE              (0x01u)
#define INA226_I2C_I2C_MODE_MASTER             (0x02u)
#define INA226_I2C_I2C_MODE_MULTI_MASTER       (0x06u)
#define INA226_I2C_I2C_MODE_MULTI_MASTER_SLAVE (0x07u)
#define INA226_I2C_I2C_MODE_MULTI_MASTER_MASK  (0x04u)

/* Master/Slave control constants */
#define INA226_I2C_I2C_WRITE_XFER_MODE    (0u)    /* Write    */
#define INA226_I2C_I2C_READ_XFER_MODE     (1u)    /* Read     */
#define INA226_I2C_I2C_ACK_ADDR           (0u)    /* Send ACK to address */
#define INA226_I2C_I2C_NAK_ADDR           (1u)    /* Send NAK to address */
#define INA226_I2C_I2C_ACK_DATA           (0u)    /* Send ACK to data */
#define INA226_I2C_I2C_NAK_DATA           (1u)    /* Send NAK to data */

/* "Mode" constants for MasterWriteBuf() or MasterReadBuf() function */
#define INA226_I2C_I2C_MODE_COMPLETE_XFER     (0x00u)    /* Full transfer with Start and Stop       */
#define INA226_I2C_I2C_MODE_REPEAT_START      (0x01u)    /* Begin with a ReStart instead of a Start */
#define INA226_I2C_I2C_MODE_NO_STOP           (0x02u)    /* Complete the transfer without a Stop    */

/* Master status */
#define INA226_I2C_I2C_MSTAT_CLEAR            ((uint16) 0x00u)   /* Clear (init) status value */

#define INA226_I2C_I2C_MSTAT_RD_CMPLT         ((uint16) 0x01u)   /* Read complete               */
#define INA226_I2C_I2C_MSTAT_WR_CMPLT         ((uint16) 0x02u)   /* Write complete              */
#define INA226_I2C_I2C_MSTAT_XFER_INP         ((uint16) 0x04u)   /* Master transfer in progress */
#define INA226_I2C_I2C_MSTAT_XFER_HALT        ((uint16) 0x08u)   /* Transfer is halted          */

#define INA226_I2C_I2C_MSTAT_ERR_MASK         ((uint16) 0x3F0u) /* Mask for all errors                          */
#define INA226_I2C_I2C_MSTAT_ERR_SHORT_XFER   ((uint16) 0x10u)  /* Master NAKed before end of packet            */
#define INA226_I2C_I2C_MSTAT_ERR_ADDR_NAK     ((uint16) 0x20u)  /* Slave did not ACK                            */
#define INA226_I2C_I2C_MSTAT_ERR_ARB_LOST     ((uint16) 0x40u)  /* Master lost arbitration during communication */
#define INA226_I2C_I2C_MSTAT_ERR_ABORT_XFER   ((uint16) 0x80u)  /* The Slave was addressed before the Start gen */
#define INA226_I2C_I2C_MSTAT_ERR_BUS_ERROR    ((uint16) 0x100u) /* The misplaced Start or Stop was occurred     */
#define INA226_I2C_I2C_MSTAT_ERR_XFER         ((uint16) 0x200u) /* Error during transfer                        */

/* Master API returns */
#define INA226_I2C_I2C_MSTR_NO_ERROR          (0x00u)  /* Function complete without error                       */
#define INA226_I2C_I2C_MSTR_ERR_ARB_LOST      (0x01u)  /* Master lost arbitration: INTR_MASTER_I2C_ARB_LOST     */
#define INA226_I2C_I2C_MSTR_ERR_LB_NAK        (0x02u)  /* Last Byte Naked: INTR_MASTER_I2C_NACK                 */
#define INA226_I2C_I2C_MSTR_NOT_READY         (0x04u)  /* Master on the bus or Slave operation is in progress   */
#define INA226_I2C_I2C_MSTR_BUS_BUSY          (0x08u)  /* Bus is busy, process not started                      */
#define INA226_I2C_I2C_MSTR_ERR_ABORT_START   (0x10u)  /* Slave was addressed before master begin Start gen     */
#define INA226_I2C_I2C_MSTR_ERR_BUS_ERR       (0x100u) /* Bus error has: INTR_MASTER_I2C_BUS_ERROR              */
#define INA226_I2C_I2C_MSTR_ERR_TIMEOUT       (INA226_I2C_I2C_MASTER_TIMEOUT) /* Operation timeout        */

/* Slave Status Constants */
#define INA226_I2C_I2C_SSTAT_RD_CMPLT         (0x01u)    /* Read transfer complete                        */
#define INA226_I2C_I2C_SSTAT_RD_BUSY          (0x02u)    /* Read transfer in progress                     */
#define INA226_I2C_I2C_SSTAT_RD_OVFL          (0x04u)    /* Read overflow: master reads above buffer size */
#define INA226_I2C_I2C_SSTAT_RD_ERR           (0x08u)    /* Read was interrupted by misplaced Start/Stop  */
#define INA226_I2C_I2C_SSTAT_RD_MASK          (0x0Fu)    /* Read Status Mask                              */
#define INA226_I2C_I2C_SSTAT_RD_NO_ERR        (0x00u)    /* Read no Error                                 */

#define INA226_I2C_I2C_SSTAT_WR_CMPLT         (0x10u)    /* Write transfer complete                         */
#define INA226_I2C_I2C_SSTAT_WR_BUSY          (0x20u)    /* Write transfer in progress                      */
#define INA226_I2C_I2C_SSTAT_WR_OVFL          (0x40u)    /* Write overflow: master writes above buffer size */
#define INA226_I2C_I2C_SSTAT_WR_ERR           (0x80u)    /* Write was interrupted by misplaced Start/Stop   */
#define INA226_I2C_I2C_SSTAT_WR_MASK          (0xF0u)    /* Write Status Mask                               */
#define INA226_I2C_I2C_SSTAT_WR_NO_ERR        (0x00u)    /* Write no Error                                  */

#define INA226_I2C_I2C_SSTAT_RD_CLEAR         (0x0Du)    /* Read Status clear: do not clear */
#define INA226_I2C_I2C_SSTAT_WR_CLEAR         (0xD0u)    /* Write Status Clear */

/* Internal I2C component constants */
#define INA226_I2C_I2C_READ_FLAG              (0x01u)     /* Read flag of the Address */
#define INA226_I2C_I2C_SLAVE_OVFL_RETURN      (0xFFu)     /* Return by slave when overflow */

#define INA226_I2C_I2C_RESET_ERROR            (0x01u)     /* Flag to re-enable SCB IP */

#define INA226_I2C_I2C_TX_OVERFLOW_COUNT      (INA226_I2C_I2C_FIFO_SIZE + 2u)


/***************************************
*     Vars with External Linkage
***************************************/

#if(INA226_I2C_SCB_MODE_UNCONFIG_CONST_CFG)
    extern const INA226_I2C_I2C_INIT_STRUCT INA226_I2C_configI2C;
#endif /* (INA226_I2C_SCB_MODE_UNCONFIG_CONST_CFG) */


/***************************************
*           FSM states
***************************************/

#define INA226_I2C_I2C_FSM_EXIT_IDLE      (0x00u) /* Master and Slave are not active, need to exit to IDLE */

/* Slave mode FSM states */
#define INA226_I2C_I2C_FSM_IDLE           (0x10u) /* Idle I2C state                */
#define INA226_I2C_I2C_FSM_SLAVE          (0x10u) /* Slave mask for all its states */

#define INA226_I2C_I2C_FSM_SL_WR          (0x11u) /* Slave write states */
#define INA226_I2C_I2C_FSM_SL_RD          (0x12u) /* Slave read  states */

/* Master mode FSM states */
#define INA226_I2C_I2C_FSM_MASTER         (0x20u) /* Master mask for all its states */
#define INA226_I2C_I2C_FSM_MSTR_ADDR      (0x08u) /* Master address phase           */
#define INA226_I2C_I2C_FSM_MSTR_DATA      (0x04u) /* Master data phase              */
#define INA226_I2C_I2C_FSM_MSTR_RD        (0x01u) /* Master read phase              */

#define INA226_I2C_I2C_FSM_MSTR_WR_ADDR   (0x28u) /* FSM master transmit address with write          */
#define INA226_I2C_I2C_FSM_MSTR_RD_ADDR   (0x29u) /* FSM master transmit address with read           */
#define INA226_I2C_I2C_FSM_MSTR_WR_DATA   (0x24u) /* FSM master writes data into the slave           */
#define INA226_I2C_I2C_FSM_MSTR_RD_DATA   (0x25u) /* FSM master reads data from the slave            */
#define INA226_I2C_I2C_FSM_MSTR_HALT      (0x60u) /* FSM master halt state: wait for Stop or ReStart */

/* Requests to complete any on-going transfer */
#define INA226_I2C_I2C_CMPLT_ANY_TRANSFER     (0x01u)

/* Returns true if FSM is in any Master state */
#define INA226_I2C_CHECK_I2C_FSM_MASTER   (0u != (INA226_I2C_I2C_FSM_MASTER & INA226_I2C_state))

/* Returns true if FSM is in any Slave state */
#define INA226_I2C_CHECK_I2C_FSM_SLAVE    (0u != (INA226_I2C_I2C_FSM_SLAVE  & INA226_I2C_state))

/* Returns true if FSM is in Master send address state */
#define INA226_I2C_CHECK_I2C_FSM_ADDR (0u != (INA226_I2C_I2C_FSM_MSTR_ADDR & INA226_I2C_state))

/* Returns true if FSM is in Master send or receive data state */
#define INA226_I2C_CHECK_I2C_FSM_DATA (0u != (INA226_I2C_I2C_FSM_MSTR_DATA  & INA226_I2C_state))

/* Returns true if FSM is in any of read states: applied for Slave and Master */
#define INA226_I2C_CHECK_I2C_FSM_RD   (0u != (INA226_I2C_I2C_FSM_MSTR_RD  & INA226_I2C_state))

/* Returns true if FSM is in IDLE state */
#define INA226_I2C_CHECK_I2C_FSM_IDLE (INA226_I2C_I2C_FSM_IDLE == INA226_I2C_state)

/* Returns true if FSM is HALT state */
#define INA226_I2C_CHECK_I2C_FSM_HALT (INA226_I2C_I2C_FSM_MSTR_HALT == INA226_I2C_state)

/* Set Master read or write completion depends on state */
#define INA226_I2C_GET_I2C_MSTAT_CMPLT (INA226_I2C_CHECK_I2C_FSM_RD ?           \
                                                    (INA226_I2C_I2C_MSTAT_RD_CMPLT) : \
                                                    (INA226_I2C_I2C_MSTAT_WR_CMPLT))


/***************************************
*       Macro Definitions
***************************************/

/* Returns TRUE if sourceMask is set in INA226_I2C_I2C_MASTER_CMD_REG: used to check if Start was generated */
#define INA226_I2C_CHECK_I2C_MASTER_CMD(sourceMask)   (0u != (INA226_I2C_I2C_MASTER_CMD_REG & (sourceMask)))

/* Returns TRUE if INA226_I2C_MODE_NO_STOP is set in INA226_I2C_mstrControl: detects NoStop condition */
#define INA226_I2C_CHECK_I2C_MODE_NO_STOP(mode)   (0u != (INA226_I2C_I2C_MODE_NO_STOP & (mode)))

/* Returns TRUE if INA226_I2C_MODE_REPEAT_START is set: used to detect when generate ReStart condition */
#define INA226_I2C_CHECK_I2C_MODE_RESTART(mode)   (0u != (INA226_I2C_I2C_MODE_REPEAT_START  & (mode)))

/* Returns TRUE if INA226_I2C_state is in one of master states */
#define INA226_I2C_CHECK_I2C_MASTER_ACTIVE    (INA226_I2C_CHECK_I2C_FSM_MASTER)

/* Timeout mask: with value should not intersect with INTR_MASTER and INTR_SLAVE bit definitions */
#define INA226_I2C_I2C_MASTER_TIMEOUT_POS (31u)
#define INA226_I2C_I2C_MASTER_TIMEOUT     ((uint32) 0x01u << INA226_I2C_I2C_MASTER_TIMEOUT_POS)

/* Slave address match or general call */
#define INA226_I2C_SLAVE_INTR_I2C_ADDR    (INA226_I2C_INTR_SLAVE_I2C_ADDR_MATCH | \
                                                 INA226_I2C_INTR_SLAVE_I2C_GENERAL)

/* Send byte condition: used by */
#define INA226_I2C_INTR_MASTER_SEND_BYTE   (INA226_I2C_INTR_MASTER_I2C_ACK      | \
                                                  INA226_I2C_INTR_MASTER_I2C_NACK     | \
                                                  INA226_I2C_INTR_MASTER_I2C_ARB_LOST | \
                                                  INA226_I2C_INTR_MASTER_I2C_BUS_ERROR)
/* Receive byte error conditions */
#define INA226_I2C_INTR_MASTER_RECEIVE_BYTE   (INA226_I2C_INTR_MASTER_I2C_ARB_LOST | \
                                                     INA226_I2C_INTR_MASTER_I2C_BUS_ERROR)
/* Stop condition */
#define INA226_I2C_INTR_MASTER_SEND_STOP  (INA226_I2C_INTR_MASTER_I2C_STOP     | \
                                                 INA226_I2C_INTR_MASTER_I2C_ARB_LOST | \
                                                 INA226_I2C_INTR_MASTER_I2C_BUS_ERROR)

/***************************************
*      Common Register Settings
***************************************/

#define INA226_I2C_CTRL_I2C       (INA226_I2C_CTRL_MODE_I2C)

#define INA226_I2C_I2C_CTRL       (INA226_I2C_I2C_CTRL_S_GENERAL_IGNORE)

#define INA226_I2C_I2C_RX_CTRL    ((INA226_I2C_ONE_BYTE_WIDTH - 1u) | \
                                          INA226_I2C_RX_CTRL_MSB_FIRST    | \
                                          INA226_I2C_RX_CTRL_ENABLED)

#define INA226_I2C_I2C_TX_CTRL    ((INA226_I2C_ONE_BYTE_WIDTH - 1u) | \
                                         INA226_I2C_TX_CTRL_MSB_FIRST     | \
                                         INA226_I2C_TX_CTRL_ENABLED)

#define INA226_I2C_I2C_INTR_SLAVE_MASK    (INA226_I2C_INTR_SLAVE_I2C_ADDR_MATCH | \
                                                 INA226_I2C_INTR_SLAVE_I2C_NACK       | \
                                                 INA226_I2C_INTR_SLAVE_I2C_WRITE_STOP | \
                                                 INA226_I2C_INTR_SLAVE_I2C_BUS_ERROR  | \
                                                 INA226_I2C_INTR_SLAVE_I2C_ARB_LOST)

#define INA226_I2C_I2C_INTR_MASTER_MASK   (INA226_I2C_INTR_MASTER_I2C_ARB_LOST | \
                                                 INA226_I2C_INTR_MASTER_I2C_NACK     | \
                                                 INA226_I2C_INTR_MASTER_I2C_STOP     | \
                                                 INA226_I2C_INTR_MASTER_I2C_BUS_ERROR)

/* Calculates tLOW in uS units */
#define INA226_I2C_I2C_TLOW_TIME  ((1000u / INA226_I2C_I2C_DATA_RATE) + \
                                            ((0u != (1000u % INA226_I2C_I2C_DATA_RATE)) ? (1u) : (0u)))
/* tHIGH = tLOW */
#define INA226_I2C_I2C_THIGH_TIME (INA226_I2C_I2C_TLOW_TIME)

#define INA226_I2C_I2C_SCL_LOW    (0u)
#define INA226_I2C_I2C_SCL_HIGH   (1u)

/* Timeout in us for tLOW and tHIGH generation (equal to 20ms) */
#define INA226_I2C_I2C_PHASE_GEN_TIMEOUT      (20000u)

#define INA226_I2C_I2C_IGNORE_GENERAL_CALL    ((uint32) (0u == INA226_I2C_I2C_ACCEPT_GENERAL_CALL))

/* Convert the timeout in milliseconds to microseconds */
#define INA226_I2C_I2C_CONVERT_TIMEOUT_TO_US(timeoutMs)   ((timeoutMs) * 1000u)


/***************************************
*    Initialization Register Settings
***************************************/

#if(INA226_I2C_SCB_MODE_I2C_CONST_CFG)

    #if (!INA226_I2C_CY_SCBIP_V0)
        #define INA226_I2C_I2C_WAKE_ENABLE_ADJ    (INA226_I2C_I2C_MULTI_MASTER_SLAVE ? \
                                                            (0u) : (INA226_I2C_I2C_WAKE_ENABLE))
    #else
        #define INA226_I2C_I2C_WAKE_ENABLE_ADJ    (INA226_I2C_I2C_WAKE_ENABLE)
    #endif /* (!INA226_I2C_CY_SCBIP_V0) */

    #define INA226_I2C_I2C_MODE_MASKED    (INA226_I2C_I2C_MODE & \
                                                (INA226_I2C_I2C_MODE_SLAVE | INA226_I2C_I2C_MODE_MASTER))

    #define INA226_I2C_I2C_DEFAULT_CTRL \
                                (INA226_I2C_GET_CTRL_BYTE_MODE  (INA226_I2C_I2C_BYTE_MODE_ENABLE) | \
                                 INA226_I2C_GET_CTRL_ADDR_ACCEPT(INA226_I2C_I2C_ACCEPT_ADDRESS)   | \
                                 INA226_I2C_GET_CTRL_EC_AM_MODE (INA226_I2C_I2C_WAKE_ENABLE_ADJ))

    #define INA226_I2C_I2C_DEFAULT_I2C_CTRL \
                    (INA226_I2C_GET_I2C_CTRL_HIGH_PHASE_OVS(INA226_I2C_I2C_OVS_FACTOR_HIGH_MIN)   | \
                     INA226_I2C_GET_I2C_CTRL_LOW_PHASE_OVS (INA226_I2C_I2C_OVS_FACTOR_LOW_MIN)    | \
                     INA226_I2C_GET_I2C_CTRL_S_GENERAL_IGNORE(INA226_I2C_I2C_IGNORE_GENERAL_CALL) | \
                     INA226_I2C_GET_I2C_CTRL_SL_MSTR_MODE  (INA226_I2C_I2C_MODE_MASKED))

    #define INA226_I2C_I2C_DEFAULT_RX_MATCH ((INA226_I2C_I2C_SLAVE) ? \
                                (INA226_I2C_GET_I2C_8BIT_ADDRESS(INA226_I2C_I2C_SLAVE_ADDRESS) | \
                                 INA226_I2C_GET_RX_MATCH_MASK   (INA226_I2C_I2C_SLAVE_ADDRESS_MASK)) : (0u))

    #define INA226_I2C_I2C_DEFAULT_RX_CTRL \
                                (INA226_I2C_GET_RX_CTRL_MEDIAN(INA226_I2C_I2C_MEDIAN_FILTER_ENABLE_ADJ) | \
                                 INA226_I2C_I2C_RX_CTRL)

    #define INA226_I2C_I2C_DEFAULT_TX_CTRL  (INA226_I2C_I2C_TX_CTRL)

    #define INA226_I2C_I2C_DEFAULT_RX_FIFO_CTRL (0u)
    #define INA226_I2C_I2C_DEFAULT_TX_FIFO_CTRL (0u)

    /* Interrupt sources */
    #define INA226_I2C_I2C_DEFAULT_INTR_I2C_EC_MASK   (INA226_I2C_NO_INTR_SOURCES)
    #define INA226_I2C_I2C_DEFAULT_INTR_SPI_EC_MASK   (INA226_I2C_NO_INTR_SOURCES)
    #define INA226_I2C_I2C_DEFAULT_INTR_RX_MASK       (INA226_I2C_NO_INTR_SOURCES)
    #define INA226_I2C_I2C_DEFAULT_INTR_TX_MASK       (INA226_I2C_NO_INTR_SOURCES)

    #define INA226_I2C_I2C_DEFAULT_INTR_SLAVE_MASK    ((INA226_I2C_I2C_SLAVE) ? \
                (INA226_I2C_I2C_INTR_SLAVE_MASK | \
                 INA226_I2C_GET_INTR_SLAVE_I2C_GENERAL(INA226_I2C_I2C_ACCEPT_GENERAL_CALL)) : (0u))

    #define INA226_I2C_I2C_DEFAULT_INTR_MASTER_MASK   (INA226_I2C_NO_INTR_SOURCES)

#endif /* (INA226_I2C_SCB_MODE_I2C_CONST_CFG) */

#endif /* (CY_SCB_I2C_INA226_I2C_H) */


/* [] END OF FILE */
