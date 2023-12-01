/***************************************************************************//**
* \file Jetty_PM.c
* \version 4.0
*
* \brief
*  This file provides the source code to the Power Management support for
*  the SCB Component.
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

#include "Jetty.h"
#include "Jetty_PVT.h"

#if(Jetty_SCB_MODE_I2C_INC)
    #include "Jetty_I2C_PVT.h"
#endif /* (Jetty_SCB_MODE_I2C_INC) */

#if(Jetty_SCB_MODE_EZI2C_INC)
    #include "Jetty_EZI2C_PVT.h"
#endif /* (Jetty_SCB_MODE_EZI2C_INC) */

#if(Jetty_SCB_MODE_SPI_INC || Jetty_SCB_MODE_UART_INC)
    #include "Jetty_SPI_UART_PVT.h"
#endif /* (Jetty_SCB_MODE_SPI_INC || Jetty_SCB_MODE_UART_INC) */


/***************************************
*   Backup Structure declaration
***************************************/

#if(Jetty_SCB_MODE_UNCONFIG_CONST_CFG || \
   (Jetty_SCB_MODE_I2C_CONST_CFG   && (!Jetty_I2C_WAKE_ENABLE_CONST))   || \
   (Jetty_SCB_MODE_EZI2C_CONST_CFG && (!Jetty_EZI2C_WAKE_ENABLE_CONST)) || \
   (Jetty_SCB_MODE_SPI_CONST_CFG   && (!Jetty_SPI_WAKE_ENABLE_CONST))   || \
   (Jetty_SCB_MODE_UART_CONST_CFG  && (!Jetty_UART_WAKE_ENABLE_CONST)))

    Jetty_BACKUP_STRUCT Jetty_backup =
    {
        0u, /* enableState */
    };
#endif


/*******************************************************************************
* Function Name: Jetty_Sleep
****************************************************************************//**
*
*  Prepares the Jetty component to enter Deep Sleep.
*  The “Enable wakeup from Deep Sleep Mode” selection has an influence on this 
*  function implementation:
*  - Checked: configures the component to be wakeup source from Deep Sleep.
*  - Unchecked: stores the current component state (enabled or disabled) and 
*    disables the component. See SCB_Stop() function for details about component 
*    disabling.
*
*  Call the Jetty_Sleep() function before calling the 
*  CyPmSysDeepSleep() function. 
*  Refer to the PSoC Creator System Reference Guide for more information about 
*  power management functions and Low power section of this document for the 
*  selected mode.
*
*  This function should not be called before entering Sleep.
*
*******************************************************************************/
void Jetty_Sleep(void)
{
#if(Jetty_SCB_MODE_UNCONFIG_CONST_CFG)

    if(Jetty_SCB_WAKE_ENABLE_CHECK)
    {
        if(Jetty_SCB_MODE_I2C_RUNTM_CFG)
        {
            Jetty_I2CSaveConfig();
        }
        else if(Jetty_SCB_MODE_EZI2C_RUNTM_CFG)
        {
            Jetty_EzI2CSaveConfig();
        }
    #if(!Jetty_CY_SCBIP_V1)
        else if(Jetty_SCB_MODE_SPI_RUNTM_CFG)
        {
            Jetty_SpiSaveConfig();
        }
        else if(Jetty_SCB_MODE_UART_RUNTM_CFG)
        {
            Jetty_UartSaveConfig();
        }
    #endif /* (!Jetty_CY_SCBIP_V1) */
        else
        {
            /* Unknown mode */
        }
    }
    else
    {
        Jetty_backup.enableState = (uint8) Jetty_GET_CTRL_ENABLED;

        if(0u != Jetty_backup.enableState)
        {
            Jetty_Stop();
        }
    }

#else

    #if (Jetty_SCB_MODE_I2C_CONST_CFG && Jetty_I2C_WAKE_ENABLE_CONST)
        Jetty_I2CSaveConfig();

    #elif (Jetty_SCB_MODE_EZI2C_CONST_CFG && Jetty_EZI2C_WAKE_ENABLE_CONST)
        Jetty_EzI2CSaveConfig();

    #elif (Jetty_SCB_MODE_SPI_CONST_CFG && Jetty_SPI_WAKE_ENABLE_CONST)
        Jetty_SpiSaveConfig();

    #elif (Jetty_SCB_MODE_UART_CONST_CFG && Jetty_UART_WAKE_ENABLE_CONST)
        Jetty_UartSaveConfig();

    #else

        Jetty_backup.enableState = (uint8) Jetty_GET_CTRL_ENABLED;

        if(0u != Jetty_backup.enableState)
        {
            Jetty_Stop();
        }

    #endif /* defined (Jetty_SCB_MODE_I2C_CONST_CFG) && (Jetty_I2C_WAKE_ENABLE_CONST) */

#endif /* (Jetty_SCB_MODE_UNCONFIG_CONST_CFG) */
}


/*******************************************************************************
* Function Name: Jetty_Wakeup
****************************************************************************//**
*
*  Prepares the Jetty component for Active mode operation after 
*  Deep Sleep.
*  The “Enable wakeup from Deep Sleep Mode” selection has influence on this 
*  function implementation:
*  - Checked: restores the component Active mode configuration.
*  - Unchecked: enables the component if it was enabled before enter Deep Sleep.
*
*  This function should not be called after exiting Sleep.
*
*  \sideeffect
*   Calling the Jetty_Wakeup() function without first calling the 
*   Jetty_Sleep() function may produce unexpected behavior.
*
*******************************************************************************/
void Jetty_Wakeup(void)
{
#if(Jetty_SCB_MODE_UNCONFIG_CONST_CFG)

    if(Jetty_SCB_WAKE_ENABLE_CHECK)
    {
        if(Jetty_SCB_MODE_I2C_RUNTM_CFG)
        {
            Jetty_I2CRestoreConfig();
        }
        else if(Jetty_SCB_MODE_EZI2C_RUNTM_CFG)
        {
            Jetty_EzI2CRestoreConfig();
        }
    #if(!Jetty_CY_SCBIP_V1)
        else if(Jetty_SCB_MODE_SPI_RUNTM_CFG)
        {
            Jetty_SpiRestoreConfig();
        }
        else if(Jetty_SCB_MODE_UART_RUNTM_CFG)
        {
            Jetty_UartRestoreConfig();
        }
    #endif /* (!Jetty_CY_SCBIP_V1) */
        else
        {
            /* Unknown mode */
        }
    }
    else
    {
        if(0u != Jetty_backup.enableState)
        {
            Jetty_Enable();
        }
    }

#else

    #if (Jetty_SCB_MODE_I2C_CONST_CFG  && Jetty_I2C_WAKE_ENABLE_CONST)
        Jetty_I2CRestoreConfig();

    #elif (Jetty_SCB_MODE_EZI2C_CONST_CFG && Jetty_EZI2C_WAKE_ENABLE_CONST)
        Jetty_EzI2CRestoreConfig();

    #elif (Jetty_SCB_MODE_SPI_CONST_CFG && Jetty_SPI_WAKE_ENABLE_CONST)
        Jetty_SpiRestoreConfig();

    #elif (Jetty_SCB_MODE_UART_CONST_CFG && Jetty_UART_WAKE_ENABLE_CONST)
        Jetty_UartRestoreConfig();

    #else

        if(0u != Jetty_backup.enableState)
        {
            Jetty_Enable();
        }

    #endif /* (Jetty_I2C_WAKE_ENABLE_CONST) */

#endif /* (Jetty_SCB_MODE_UNCONFIG_CONST_CFG) */
}


/* [] END OF FILE */
