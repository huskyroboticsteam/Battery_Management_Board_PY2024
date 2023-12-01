/***************************************************************************//**
* \file .h
* \version 4.0
*
* \brief
*  This private file provides constants and parameter values for the
*  SCB Component in I2C mode.
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

#if !defined(CY_SCB_I2C_PVT_Jetty_H)
#define CY_SCB_I2C_PVT_Jetty_H

#include "Jetty_I2C.h"


/***************************************
*     Private Global Vars
***************************************/

extern volatile uint8 Jetty_state; /* Current state of I2C FSM */

#if(Jetty_I2C_SLAVE_CONST)
    extern volatile uint8 Jetty_slStatus;          /* Slave Status */

    /* Receive buffer variables */
    extern volatile uint8 * Jetty_slWrBufPtr;      /* Pointer to Receive buffer  */
    extern volatile uint32  Jetty_slWrBufSize;     /* Slave Receive buffer size  */
    extern volatile uint32  Jetty_slWrBufIndex;    /* Slave Receive buffer Index */

    /* Transmit buffer variables */
    extern volatile uint8 * Jetty_slRdBufPtr;      /* Pointer to Transmit buffer  */
    extern volatile uint32  Jetty_slRdBufSize;     /* Slave Transmit buffer size  */
    extern volatile uint32  Jetty_slRdBufIndex;    /* Slave Transmit buffer Index */
    extern volatile uint32  Jetty_slRdBufIndexTmp; /* Slave Transmit buffer Index Tmp */
    extern volatile uint8   Jetty_slOverFlowCount; /* Slave Transmit Overflow counter */
#endif /* (Jetty_I2C_SLAVE_CONST) */

#if(Jetty_I2C_MASTER_CONST)
    extern volatile uint16 Jetty_mstrStatus;      /* Master Status byte  */
    extern volatile uint8  Jetty_mstrControl;     /* Master Control byte */

    /* Receive buffer variables */
    extern volatile uint8 * Jetty_mstrRdBufPtr;   /* Pointer to Master Read buffer */
    extern volatile uint32  Jetty_mstrRdBufSize;  /* Master Read buffer size       */
    extern volatile uint32  Jetty_mstrRdBufIndex; /* Master Read buffer Index      */

    /* Transmit buffer variables */
    extern volatile uint8 * Jetty_mstrWrBufPtr;   /* Pointer to Master Write buffer */
    extern volatile uint32  Jetty_mstrWrBufSize;  /* Master Write buffer size       */
    extern volatile uint32  Jetty_mstrWrBufIndex; /* Master Write buffer Index      */
    extern volatile uint32  Jetty_mstrWrBufIndexTmp; /* Master Write buffer Index Tmp */
#endif /* (Jetty_I2C_MASTER_CONST) */

#if (Jetty_I2C_CUSTOM_ADDRESS_HANDLER_CONST)
    extern uint32 (*Jetty_customAddressHandler) (void);
#endif /* (Jetty_I2C_CUSTOM_ADDRESS_HANDLER_CONST) */

/***************************************
*     Private Function Prototypes
***************************************/

#if(Jetty_SCB_MODE_I2C_CONST_CFG)
    void Jetty_I2CInit(void);
#endif /* (Jetty_SCB_MODE_I2C_CONST_CFG) */

void Jetty_I2CStop(void);
void Jetty_I2CFwBlockReset(void);

void Jetty_I2CSaveConfig(void);
void Jetty_I2CRestoreConfig(void);

#if(Jetty_I2C_MASTER_CONST)
    void Jetty_I2CReStartGeneration(void);
#endif /* (Jetty_I2C_MASTER_CONST) */

#endif /* (CY_SCB_I2C_PVT_Jetty_H) */


/* [] END OF FILE */
