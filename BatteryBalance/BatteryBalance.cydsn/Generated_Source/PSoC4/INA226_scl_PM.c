/*******************************************************************************
* File Name: INA226_scl.c  
* Version 2.20
*
* Description:
*  This file contains APIs to set up the Pins component for low power modes.
*
* Note:
*
********************************************************************************
* Copyright 2015, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions, 
* disclaimers, and limitations in the end user license agreement accompanying 
* the software package with which this file was provided.
*******************************************************************************/

#include "cytypes.h"
#include "INA226_scl.h"

static INA226_scl_BACKUP_STRUCT  INA226_scl_backup = {0u, 0u, 0u};


/*******************************************************************************
* Function Name: INA226_scl_Sleep
****************************************************************************//**
*
* \brief Stores the pin configuration and prepares the pin for entering chip 
*  deep-sleep/hibernate modes. This function applies only to SIO and USBIO pins.
*  It should not be called for GPIO or GPIO_OVT pins.
*
* <b>Note</b> This function is available in PSoC 4 only.
*
* \return 
*  None 
*  
* \sideeffect
*  For SIO pins, this function configures the pin input threshold to CMOS and
*  drive level to Vddio. This is needed for SIO pins when in device 
*  deep-sleep/hibernate modes.
*
* \funcusage
*  \snippet INA226_scl_SUT.c usage_INA226_scl_Sleep_Wakeup
*******************************************************************************/
void INA226_scl_Sleep(void)
{
    #if defined(INA226_scl__PC)
        INA226_scl_backup.pcState = INA226_scl_PC;
    #else
        #if (CY_PSOC4_4200L)
            /* Save the regulator state and put the PHY into suspend mode */
            INA226_scl_backup.usbState = INA226_scl_CR1_REG;
            INA226_scl_USB_POWER_REG |= INA226_scl_USBIO_ENTER_SLEEP;
            INA226_scl_CR1_REG &= INA226_scl_USBIO_CR1_OFF;
        #endif
    #endif
    #if defined(CYIPBLOCK_m0s8ioss_VERSION) && defined(INA226_scl__SIO)
        INA226_scl_backup.sioState = INA226_scl_SIO_REG;
        /* SIO requires unregulated output buffer and single ended input buffer */
        INA226_scl_SIO_REG &= (uint32)(~INA226_scl_SIO_LPM_MASK);
    #endif  
}


/*******************************************************************************
* Function Name: INA226_scl_Wakeup
****************************************************************************//**
*
* \brief Restores the pin configuration that was saved during Pin_Sleep(). This 
* function applies only to SIO and USBIO pins. It should not be called for
* GPIO or GPIO_OVT pins.
*
* For USBIO pins, the wakeup is only triggered for falling edge interrupts.
*
* <b>Note</b> This function is available in PSoC 4 only.
*
* \return 
*  None
*  
* \funcusage
*  Refer to INA226_scl_Sleep() for an example usage.
*******************************************************************************/
void INA226_scl_Wakeup(void)
{
    #if defined(INA226_scl__PC)
        INA226_scl_PC = INA226_scl_backup.pcState;
    #else
        #if (CY_PSOC4_4200L)
            /* Restore the regulator state and come out of suspend mode */
            INA226_scl_USB_POWER_REG &= INA226_scl_USBIO_EXIT_SLEEP_PH1;
            INA226_scl_CR1_REG = INA226_scl_backup.usbState;
            INA226_scl_USB_POWER_REG &= INA226_scl_USBIO_EXIT_SLEEP_PH2;
        #endif
    #endif
    #if defined(CYIPBLOCK_m0s8ioss_VERSION) && defined(INA226_scl__SIO)
        INA226_scl_SIO_REG = INA226_scl_backup.sioState;
    #endif
}


/* [] END OF FILE */
