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

#if !defined(CY_SCB_PVT_Jetty_H)
#define CY_SCB_PVT_Jetty_H

#include "Jetty.h"


/***************************************
*     Private Function Prototypes
***************************************/

/* APIs to service INTR_I2C_EC register */
#define Jetty_SetI2CExtClkInterruptMode(interruptMask) Jetty_WRITE_INTR_I2C_EC_MASK(interruptMask)
#define Jetty_ClearI2CExtClkInterruptSource(interruptMask) Jetty_CLEAR_INTR_I2C_EC(interruptMask)
#define Jetty_GetI2CExtClkInterruptSource()                (Jetty_INTR_I2C_EC_REG)
#define Jetty_GetI2CExtClkInterruptMode()                  (Jetty_INTR_I2C_EC_MASK_REG)
#define Jetty_GetI2CExtClkInterruptSourceMasked()          (Jetty_INTR_I2C_EC_MASKED_REG)

#if (!Jetty_CY_SCBIP_V1)
    /* APIs to service INTR_SPI_EC register */
    #define Jetty_SetSpiExtClkInterruptMode(interruptMask) \
                                                                Jetty_WRITE_INTR_SPI_EC_MASK(interruptMask)
    #define Jetty_ClearSpiExtClkInterruptSource(interruptMask) \
                                                                Jetty_CLEAR_INTR_SPI_EC(interruptMask)
    #define Jetty_GetExtSpiClkInterruptSource()                 (Jetty_INTR_SPI_EC_REG)
    #define Jetty_GetExtSpiClkInterruptMode()                   (Jetty_INTR_SPI_EC_MASK_REG)
    #define Jetty_GetExtSpiClkInterruptSourceMasked()           (Jetty_INTR_SPI_EC_MASKED_REG)
#endif /* (!Jetty_CY_SCBIP_V1) */

#if(Jetty_SCB_MODE_UNCONFIG_CONST_CFG)
    extern void Jetty_SetPins(uint32 mode, uint32 subMode, uint32 uartEnableMask);
#endif /* (Jetty_SCB_MODE_UNCONFIG_CONST_CFG) */


/***************************************
*     Vars with External Linkage
***************************************/

#if (Jetty_SCB_IRQ_INTERNAL)
#if !defined (CY_REMOVE_Jetty_CUSTOM_INTR_HANDLER)
    extern cyisraddress Jetty_customIntrHandler;
#endif /* !defined (CY_REMOVE_Jetty_CUSTOM_INTR_HANDLER) */
#endif /* (Jetty_SCB_IRQ_INTERNAL) */

extern Jetty_BACKUP_STRUCT Jetty_backup;

#if(Jetty_SCB_MODE_UNCONFIG_CONST_CFG)
    /* Common configuration variables */
    extern uint8 Jetty_scbMode;
    extern uint8 Jetty_scbEnableWake;
    extern uint8 Jetty_scbEnableIntr;

    /* I2C configuration variables */
    extern uint8 Jetty_mode;
    extern uint8 Jetty_acceptAddr;

    /* SPI/UART configuration variables */
    extern volatile uint8 * Jetty_rxBuffer;
    extern uint8   Jetty_rxDataBits;
    extern uint32  Jetty_rxBufferSize;

    extern volatile uint8 * Jetty_txBuffer;
    extern uint8   Jetty_txDataBits;
    extern uint32  Jetty_txBufferSize;

    /* EZI2C configuration variables */
    extern uint8 Jetty_numberOfAddr;
    extern uint8 Jetty_subAddrSize;
#endif /* (Jetty_SCB_MODE_UNCONFIG_CONST_CFG) */

#if (! (Jetty_SCB_MODE_I2C_CONST_CFG || \
        Jetty_SCB_MODE_EZI2C_CONST_CFG))
    extern uint16 Jetty_IntrTxMask;
#endif /* (! (Jetty_SCB_MODE_I2C_CONST_CFG || \
              Jetty_SCB_MODE_EZI2C_CONST_CFG)) */


/***************************************
*        Conditional Macro
****************************************/

#if(Jetty_SCB_MODE_UNCONFIG_CONST_CFG)
    /* Defines run time operation mode */
    #define Jetty_SCB_MODE_I2C_RUNTM_CFG     (Jetty_SCB_MODE_I2C      == Jetty_scbMode)
    #define Jetty_SCB_MODE_SPI_RUNTM_CFG     (Jetty_SCB_MODE_SPI      == Jetty_scbMode)
    #define Jetty_SCB_MODE_UART_RUNTM_CFG    (Jetty_SCB_MODE_UART     == Jetty_scbMode)
    #define Jetty_SCB_MODE_EZI2C_RUNTM_CFG   (Jetty_SCB_MODE_EZI2C    == Jetty_scbMode)
    #define Jetty_SCB_MODE_UNCONFIG_RUNTM_CFG \
                                                        (Jetty_SCB_MODE_UNCONFIG == Jetty_scbMode)

    /* Defines wakeup enable */
    #define Jetty_SCB_WAKE_ENABLE_CHECK       (0u != Jetty_scbEnableWake)
#endif /* (Jetty_SCB_MODE_UNCONFIG_CONST_CFG) */

/* Defines maximum number of SCB pins */
#if (!Jetty_CY_SCBIP_V1)
    #define Jetty_SCB_PINS_NUMBER    (7u)
#else
    #define Jetty_SCB_PINS_NUMBER    (2u)
#endif /* (!Jetty_CY_SCBIP_V1) */

#endif /* (CY_SCB_PVT_Jetty_H) */


/* [] END OF FILE */
