/***************************************************************************//**
* \file .h
* \version 4.0
*
* \brief
*  This private file provides constants and parameter values for the
*  SCB Component.
*  Please do not use this file or its content in your project.
*
* Note:
*
********************************************************************************
* \copyright
* Copyright 2013-2017, Cypress Semiconductor Corporation. All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#if !defined(CY_SCB_PVT_INA226_I2C_H)
#define CY_SCB_PVT_INA226_I2C_H

#include "INA226_I2C.h"


/***************************************
*     Private Function Prototypes
***************************************/

/* APIs to service INTR_I2C_EC register */
#define INA226_I2C_SetI2CExtClkInterruptMode(interruptMask) INA226_I2C_WRITE_INTR_I2C_EC_MASK(interruptMask)
#define INA226_I2C_ClearI2CExtClkInterruptSource(interruptMask) INA226_I2C_CLEAR_INTR_I2C_EC(interruptMask)
#define INA226_I2C_GetI2CExtClkInterruptSource()                (INA226_I2C_INTR_I2C_EC_REG)
#define INA226_I2C_GetI2CExtClkInterruptMode()                  (INA226_I2C_INTR_I2C_EC_MASK_REG)
#define INA226_I2C_GetI2CExtClkInterruptSourceMasked()          (INA226_I2C_INTR_I2C_EC_MASKED_REG)

#if (!INA226_I2C_CY_SCBIP_V1)
    /* APIs to service INTR_SPI_EC register */
    #define INA226_I2C_SetSpiExtClkInterruptMode(interruptMask) \
                                                                INA226_I2C_WRITE_INTR_SPI_EC_MASK(interruptMask)
    #define INA226_I2C_ClearSpiExtClkInterruptSource(interruptMask) \
                                                                INA226_I2C_CLEAR_INTR_SPI_EC(interruptMask)
    #define INA226_I2C_GetExtSpiClkInterruptSource()                 (INA226_I2C_INTR_SPI_EC_REG)
    #define INA226_I2C_GetExtSpiClkInterruptMode()                   (INA226_I2C_INTR_SPI_EC_MASK_REG)
    #define INA226_I2C_GetExtSpiClkInterruptSourceMasked()           (INA226_I2C_INTR_SPI_EC_MASKED_REG)
#endif /* (!INA226_I2C_CY_SCBIP_V1) */

#if(INA226_I2C_SCB_MODE_UNCONFIG_CONST_CFG)
    extern void INA226_I2C_SetPins(uint32 mode, uint32 subMode, uint32 uartEnableMask);
#endif /* (INA226_I2C_SCB_MODE_UNCONFIG_CONST_CFG) */


/***************************************
*     Vars with External Linkage
***************************************/

#if (INA226_I2C_SCB_IRQ_INTERNAL)
#if !defined (CY_REMOVE_INA226_I2C_CUSTOM_INTR_HANDLER)
    extern cyisraddress INA226_I2C_customIntrHandler;
#endif /* !defined (CY_REMOVE_INA226_I2C_CUSTOM_INTR_HANDLER) */
#endif /* (INA226_I2C_SCB_IRQ_INTERNAL) */

extern INA226_I2C_BACKUP_STRUCT INA226_I2C_backup;

#if(INA226_I2C_SCB_MODE_UNCONFIG_CONST_CFG)
    /* Common configuration variables */
    extern uint8 INA226_I2C_scbMode;
    extern uint8 INA226_I2C_scbEnableWake;
    extern uint8 INA226_I2C_scbEnableIntr;

    /* I2C configuration variables */
    extern uint8 INA226_I2C_mode;
    extern uint8 INA226_I2C_acceptAddr;

    /* SPI/UART configuration variables */
    extern volatile uint8 * INA226_I2C_rxBuffer;
    extern uint8   INA226_I2C_rxDataBits;
    extern uint32  INA226_I2C_rxBufferSize;

    extern volatile uint8 * INA226_I2C_txBuffer;
    extern uint8   INA226_I2C_txDataBits;
    extern uint32  INA226_I2C_txBufferSize;

    /* EZI2C configuration variables */
    extern uint8 INA226_I2C_numberOfAddr;
    extern uint8 INA226_I2C_subAddrSize;
#endif /* (INA226_I2C_SCB_MODE_UNCONFIG_CONST_CFG) */

#if (! (INA226_I2C_SCB_MODE_I2C_CONST_CFG || \
        INA226_I2C_SCB_MODE_EZI2C_CONST_CFG))
    extern uint16 INA226_I2C_IntrTxMask;
#endif /* (! (INA226_I2C_SCB_MODE_I2C_CONST_CFG || \
              INA226_I2C_SCB_MODE_EZI2C_CONST_CFG)) */


/***************************************
*        Conditional Macro
****************************************/

#if(INA226_I2C_SCB_MODE_UNCONFIG_CONST_CFG)
    /* Defines run time operation mode */
    #define INA226_I2C_SCB_MODE_I2C_RUNTM_CFG     (INA226_I2C_SCB_MODE_I2C      == INA226_I2C_scbMode)
    #define INA226_I2C_SCB_MODE_SPI_RUNTM_CFG     (INA226_I2C_SCB_MODE_SPI      == INA226_I2C_scbMode)
    #define INA226_I2C_SCB_MODE_UART_RUNTM_CFG    (INA226_I2C_SCB_MODE_UART     == INA226_I2C_scbMode)
    #define INA226_I2C_SCB_MODE_EZI2C_RUNTM_CFG   (INA226_I2C_SCB_MODE_EZI2C    == INA226_I2C_scbMode)
    #define INA226_I2C_SCB_MODE_UNCONFIG_RUNTM_CFG \
                                                        (INA226_I2C_SCB_MODE_UNCONFIG == INA226_I2C_scbMode)

    /* Defines wakeup enable */
    #define INA226_I2C_SCB_WAKE_ENABLE_CHECK       (0u != INA226_I2C_scbEnableWake)
#endif /* (INA226_I2C_SCB_MODE_UNCONFIG_CONST_CFG) */

/* Defines maximum number of SCB pins */
#if (!INA226_I2C_CY_SCBIP_V1)
    #define INA226_I2C_SCB_PINS_NUMBER    (7u)
#else
    #define INA226_I2C_SCB_PINS_NUMBER    (2u)
#endif /* (!INA226_I2C_CY_SCBIP_V1) */

#endif /* (CY_SCB_PVT_INA226_I2C_H) */


/* [] END OF FILE */
