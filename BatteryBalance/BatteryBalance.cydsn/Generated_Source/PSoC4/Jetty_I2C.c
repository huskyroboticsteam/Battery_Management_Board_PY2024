/***************************************************************************//**
* \file Jetty_I2C.c
* \version 4.0
*
* \brief
*  This file provides the source code to the API for the SCB Component in
*  I2C mode.
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


/***************************************
*      I2C Private Vars
***************************************/

volatile uint8 Jetty_state;  /* Current state of I2C FSM */

#if(Jetty_SCB_MODE_UNCONFIG_CONST_CFG)

    /***************************************
    *  Configuration Structure Initialization
    ***************************************/

    /* Constant configuration of I2C */
    const Jetty_I2C_INIT_STRUCT Jetty_configI2C =
    {
        Jetty_I2C_MODE,
        Jetty_I2C_OVS_FACTOR_LOW,
        Jetty_I2C_OVS_FACTOR_HIGH,
        Jetty_I2C_MEDIAN_FILTER_ENABLE,
        Jetty_I2C_SLAVE_ADDRESS,
        Jetty_I2C_SLAVE_ADDRESS_MASK,
        Jetty_I2C_ACCEPT_ADDRESS,
        Jetty_I2C_WAKE_ENABLE,
        Jetty_I2C_BYTE_MODE_ENABLE,
        Jetty_I2C_DATA_RATE,
        Jetty_I2C_ACCEPT_GENERAL_CALL,
    };

    /*******************************************************************************
    * Function Name: Jetty_I2CInit
    ****************************************************************************//**
    *
    *
    *  Configures the Jetty for I2C operation.
    *
    *  This function is intended specifically to be used when the Jetty 
    *  configuration is set to “Unconfigured Jetty” in the customizer. 
    *  After initializing the Jetty in I2C mode using this function, 
    *  the component can be enabled using the Jetty_Start() or 
    * Jetty_Enable() function.
    *  This function uses a pointer to a structure that provides the configuration 
    *  settings. This structure contains the same information that would otherwise 
    *  be provided by the customizer settings.
    *
    *  \param config: pointer to a structure that contains the following list of 
    *   fields. These fields match the selections available in the customizer. 
    *   Refer to the customizer for further description of the settings.
    *
    *******************************************************************************/
    void Jetty_I2CInit(const Jetty_I2C_INIT_STRUCT *config)
    {
        uint32 medianFilter;
        uint32 locEnableWake;

        if(NULL == config)
        {
            CYASSERT(0u != 0u); /* Halt execution due to bad function parameter */
        }
        else
        {
            /* Configure pins */
            Jetty_SetPins(Jetty_SCB_MODE_I2C, Jetty_DUMMY_PARAM,
                                     Jetty_DUMMY_PARAM);

            /* Store internal configuration */
            Jetty_scbMode       = (uint8) Jetty_SCB_MODE_I2C;
            Jetty_scbEnableWake = (uint8) config->enableWake;
            Jetty_scbEnableIntr = (uint8) Jetty_SCB_IRQ_INTERNAL;

            Jetty_mode          = (uint8) config->mode;
            Jetty_acceptAddr    = (uint8) config->acceptAddr;

        #if (Jetty_CY_SCBIP_V0)
            /* Adjust SDA filter settings. Ticket ID#150521 */
            Jetty_SET_I2C_CFG_SDA_FILT_TRIM(Jetty_EC_AM_I2C_CFG_SDA_FILT_TRIM);
        #endif /* (Jetty_CY_SCBIP_V0) */

            /* Adjust AF and DF filter settings. Ticket ID#176179 */
            if (((Jetty_I2C_MODE_SLAVE != config->mode) &&
                 (config->dataRate <= Jetty_I2C_DATA_RATE_FS_MODE_MAX)) ||
                 (Jetty_I2C_MODE_SLAVE == config->mode))
            {
                /* AF = 1, DF = 0 */
                Jetty_I2C_CFG_ANALOG_FITER_ENABLE;
                medianFilter = Jetty_DIGITAL_FILTER_DISABLE;
            }
            else
            {
                /* AF = 0, DF = 1 */
                Jetty_I2C_CFG_ANALOG_FITER_DISABLE;
                medianFilter = Jetty_DIGITAL_FILTER_ENABLE;
            }

        #if (!Jetty_CY_SCBIP_V0)
            locEnableWake = (Jetty_I2C_MULTI_MASTER_SLAVE) ? (0u) : (config->enableWake);
        #else
            locEnableWake = config->enableWake;
        #endif /* (!Jetty_CY_SCBIP_V0) */

            /* Configure I2C interface */
            Jetty_CTRL_REG     = Jetty_GET_CTRL_BYTE_MODE  (config->enableByteMode) |
                                            Jetty_GET_CTRL_ADDR_ACCEPT(config->acceptAddr)     |
                                            Jetty_GET_CTRL_EC_AM_MODE (locEnableWake);

            Jetty_I2C_CTRL_REG = Jetty_GET_I2C_CTRL_HIGH_PHASE_OVS(config->oversampleHigh) |
                    Jetty_GET_I2C_CTRL_LOW_PHASE_OVS (config->oversampleLow)                          |
                    Jetty_GET_I2C_CTRL_S_GENERAL_IGNORE((uint32)(0u == config->acceptGeneralAddr))    |
                    Jetty_GET_I2C_CTRL_SL_MSTR_MODE  (config->mode);

            /* Configure RX direction */
            Jetty_RX_CTRL_REG      = Jetty_GET_RX_CTRL_MEDIAN(medianFilter) |
                                                Jetty_I2C_RX_CTRL;
            Jetty_RX_FIFO_CTRL_REG = Jetty_CLEAR_REG;

            /* Set default address and mask */
            Jetty_RX_MATCH_REG    = ((Jetty_I2C_SLAVE) ?
                                                (Jetty_GET_I2C_8BIT_ADDRESS(config->slaveAddr) |
                                                 Jetty_GET_RX_MATCH_MASK(config->slaveAddrMask)) :
                                                (Jetty_CLEAR_REG));


            /* Configure TX direction */
            Jetty_TX_CTRL_REG      = Jetty_I2C_TX_CTRL;
            Jetty_TX_FIFO_CTRL_REG = Jetty_CLEAR_REG;

            /* Configure interrupt with I2C handler but do not enable it */
            CyIntDisable    (Jetty_ISR_NUMBER);
            CyIntSetPriority(Jetty_ISR_NUMBER, Jetty_ISR_PRIORITY);
            (void) CyIntSetVector(Jetty_ISR_NUMBER, &Jetty_I2C_ISR);

            /* Configure interrupt sources */
        #if(!Jetty_CY_SCBIP_V1)
            Jetty_INTR_SPI_EC_MASK_REG = Jetty_NO_INTR_SOURCES;
        #endif /* (!Jetty_CY_SCBIP_V1) */

            Jetty_INTR_I2C_EC_MASK_REG = Jetty_NO_INTR_SOURCES;
            Jetty_INTR_RX_MASK_REG     = Jetty_NO_INTR_SOURCES;
            Jetty_INTR_TX_MASK_REG     = Jetty_NO_INTR_SOURCES;

            Jetty_INTR_SLAVE_MASK_REG  = ((Jetty_I2C_SLAVE) ?
                            (Jetty_GET_INTR_SLAVE_I2C_GENERAL(config->acceptGeneralAddr) |
                             Jetty_I2C_INTR_SLAVE_MASK) : (Jetty_CLEAR_REG));

            Jetty_INTR_MASTER_MASK_REG = Jetty_NO_INTR_SOURCES;

            /* Configure global variables */
            Jetty_state = Jetty_I2C_FSM_IDLE;

            /* Internal slave variables */
            Jetty_slStatus        = 0u;
            Jetty_slRdBufIndex    = 0u;
            Jetty_slWrBufIndex    = 0u;
            Jetty_slOverFlowCount = 0u;

            /* Internal master variables */
            Jetty_mstrStatus     = 0u;
            Jetty_mstrRdBufIndex = 0u;
            Jetty_mstrWrBufIndex = 0u;
        }
    }

#else

    /*******************************************************************************
    * Function Name: Jetty_I2CInit
    ****************************************************************************//**
    *
    *  Configures the SCB for the I2C operation.
    *
    *******************************************************************************/
    void Jetty_I2CInit(void)
    {
    #if(Jetty_CY_SCBIP_V0)
        /* Adjust SDA filter settings. Ticket ID#150521 */
        Jetty_SET_I2C_CFG_SDA_FILT_TRIM(Jetty_EC_AM_I2C_CFG_SDA_FILT_TRIM);
    #endif /* (Jetty_CY_SCBIP_V0) */

        /* Adjust AF and DF filter settings. Ticket ID#176179 */
        Jetty_I2C_CFG_ANALOG_FITER_ENABLE_ADJ;

        /* Configure I2C interface */
        Jetty_CTRL_REG     = Jetty_I2C_DEFAULT_CTRL;
        Jetty_I2C_CTRL_REG = Jetty_I2C_DEFAULT_I2C_CTRL;

        /* Configure RX direction */
        Jetty_RX_CTRL_REG      = Jetty_I2C_DEFAULT_RX_CTRL;
        Jetty_RX_FIFO_CTRL_REG = Jetty_I2C_DEFAULT_RX_FIFO_CTRL;

        /* Set default address and mask */
        Jetty_RX_MATCH_REG     = Jetty_I2C_DEFAULT_RX_MATCH;

        /* Configure TX direction */
        Jetty_TX_CTRL_REG      = Jetty_I2C_DEFAULT_TX_CTRL;
        Jetty_TX_FIFO_CTRL_REG = Jetty_I2C_DEFAULT_TX_FIFO_CTRL;

        /* Configure interrupt with I2C handler but do not enable it */
        CyIntDisable    (Jetty_ISR_NUMBER);
        CyIntSetPriority(Jetty_ISR_NUMBER, Jetty_ISR_PRIORITY);
    #if(!Jetty_I2C_EXTERN_INTR_HANDLER)
        (void) CyIntSetVector(Jetty_ISR_NUMBER, &Jetty_I2C_ISR);
    #endif /* (Jetty_I2C_EXTERN_INTR_HANDLER) */

        /* Configure interrupt sources */
    #if(!Jetty_CY_SCBIP_V1)
        Jetty_INTR_SPI_EC_MASK_REG = Jetty_I2C_DEFAULT_INTR_SPI_EC_MASK;
    #endif /* (!Jetty_CY_SCBIP_V1) */

        Jetty_INTR_I2C_EC_MASK_REG = Jetty_I2C_DEFAULT_INTR_I2C_EC_MASK;
        Jetty_INTR_SLAVE_MASK_REG  = Jetty_I2C_DEFAULT_INTR_SLAVE_MASK;
        Jetty_INTR_MASTER_MASK_REG = Jetty_I2C_DEFAULT_INTR_MASTER_MASK;
        Jetty_INTR_RX_MASK_REG     = Jetty_I2C_DEFAULT_INTR_RX_MASK;
        Jetty_INTR_TX_MASK_REG     = Jetty_I2C_DEFAULT_INTR_TX_MASK;

        /* Configure global variables */
        Jetty_state = Jetty_I2C_FSM_IDLE;

    #if(Jetty_I2C_SLAVE)
        /* Internal slave variable */
        Jetty_slStatus        = 0u;
        Jetty_slRdBufIndex    = 0u;
        Jetty_slWrBufIndex    = 0u;
        Jetty_slOverFlowCount = 0u;
    #endif /* (Jetty_I2C_SLAVE) */

    #if(Jetty_I2C_MASTER)
    /* Internal master variable */
        Jetty_mstrStatus     = 0u;
        Jetty_mstrRdBufIndex = 0u;
        Jetty_mstrWrBufIndex = 0u;
    #endif /* (Jetty_I2C_MASTER) */
    }
#endif /* (Jetty_SCB_MODE_UNCONFIG_CONST_CFG) */


/*******************************************************************************
* Function Name: Jetty_I2CStop
****************************************************************************//**
*
*  Resets the I2C FSM into the default state.
*
*******************************************************************************/
void Jetty_I2CStop(void)
{
    /* Clear command registers because they keep assigned value after IP block was disabled */
    Jetty_I2C_MASTER_CMD_REG = 0u;
    Jetty_I2C_SLAVE_CMD_REG  = 0u;
    
    Jetty_state = Jetty_I2C_FSM_IDLE;
}


/*******************************************************************************
* Function Name: Jetty_I2CFwBlockReset
****************************************************************************//**
*
* Resets the scb IP block and I2C into the known state.
*
*******************************************************************************/
void Jetty_I2CFwBlockReset(void)
{
    /* Disable scb IP: stop respond to I2C traffic */
    Jetty_CTRL_REG &= (uint32) ~Jetty_CTRL_ENABLED;

    /* Clear command registers they are not cleared after scb IP is disabled */
    Jetty_I2C_MASTER_CMD_REG = 0u;
    Jetty_I2C_SLAVE_CMD_REG  = 0u;

    Jetty_DISABLE_AUTO_DATA;

    Jetty_SetTxInterruptMode(Jetty_NO_INTR_SOURCES);
    Jetty_SetRxInterruptMode(Jetty_NO_INTR_SOURCES);
    
#if(Jetty_CY_SCBIP_V0)
    /* Clear interrupt sources as they are not cleared after scb IP is disabled */
    Jetty_ClearTxInterruptSource    (Jetty_INTR_TX_ALL);
    Jetty_ClearRxInterruptSource    (Jetty_INTR_RX_ALL);
    Jetty_ClearSlaveInterruptSource (Jetty_INTR_SLAVE_ALL);
    Jetty_ClearMasterInterruptSource(Jetty_INTR_MASTER_ALL);
#endif /* (Jetty_CY_SCBIP_V0) */

    Jetty_state = Jetty_I2C_FSM_IDLE;

    /* Enable scb IP: start respond to I2C traffic */
    Jetty_CTRL_REG |= (uint32) Jetty_CTRL_ENABLED;
}


#if(Jetty_I2C_WAKE_ENABLE_CONST)
    /*******************************************************************************
    * Function Name: Jetty_I2CSaveConfig
    ****************************************************************************//**
    *
    *  Enables Jetty_INTR_I2C_EC_WAKE_UP interrupt source. This interrupt
    *  triggers on address match and wakes up device.
    *
    *******************************************************************************/
    void Jetty_I2CSaveConfig(void)
    {
    #if (!Jetty_CY_SCBIP_V0)
        #if (Jetty_I2C_MULTI_MASTER_SLAVE_CONST && Jetty_I2C_WAKE_ENABLE_CONST)
            /* Enable externally clocked address match if it was not enabled before.
            * This applicable only for Multi-Master-Slave. Ticket ID#192742 */
            if (0u == (Jetty_CTRL_REG & Jetty_CTRL_EC_AM_MODE))
            {
                /* Enable external address match logic */
                Jetty_Stop();
                Jetty_CTRL_REG |= Jetty_CTRL_EC_AM_MODE;
                Jetty_Enable();
            }
        #endif /* (Jetty_I2C_MULTI_MASTER_SLAVE_CONST) */

        #if (Jetty_SCB_CLK_INTERNAL)
            /* Disable clock to internal address match logic. Ticket ID#187931 */
            Jetty_SCBCLK_Stop();
        #endif /* (Jetty_SCB_CLK_INTERNAL) */
    #endif /* (!Jetty_CY_SCBIP_V0) */

        Jetty_SetI2CExtClkInterruptMode(Jetty_INTR_I2C_EC_WAKE_UP);
    }


    /*******************************************************************************
    * Function Name: Jetty_I2CRestoreConfig
    ****************************************************************************//**
    *
    *  Disables Jetty_INTR_I2C_EC_WAKE_UP interrupt source. This interrupt
    *  triggers on address match and wakes up device.
    *
    *******************************************************************************/
    void Jetty_I2CRestoreConfig(void)
    {
        /* Disable wakeup interrupt on address match */
        Jetty_SetI2CExtClkInterruptMode(Jetty_NO_INTR_SOURCES);

    #if (!Jetty_CY_SCBIP_V0)
        #if (Jetty_SCB_CLK_INTERNAL)
            /* Enable clock to internal address match logic. Ticket ID#187931 */
            Jetty_SCBCLK_Start();
        #endif /* (Jetty_SCB_CLK_INTERNAL) */
    #endif /* (!Jetty_CY_SCBIP_V0) */
    }
#endif /* (Jetty_I2C_WAKE_ENABLE_CONST) */


/* [] END OF FILE */
