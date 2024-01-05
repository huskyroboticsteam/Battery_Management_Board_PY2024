/*******************************************************************************
* File Name: INA226_sda.h  
* Version 2.20
*
* Description:
*  This file contains the Alias definitions for Per-Pin APIs in cypins.h. 
*  Information on using these APIs can be found in the System Reference Guide.
*
* Note:
*
********************************************************************************
* Copyright 2008-2015, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions, 
* disclaimers, and limitations in the end user license agreement accompanying 
* the software package with which this file was provided.
*******************************************************************************/

#if !defined(CY_PINS_INA226_sda_ALIASES_H) /* Pins INA226_sda_ALIASES_H */
#define CY_PINS_INA226_sda_ALIASES_H

#include "cytypes.h"
#include "cyfitter.h"
#include "cypins.h"


/***************************************
*              Constants        
***************************************/
#define INA226_sda_0			(INA226_sda__0__PC)
#define INA226_sda_0_PS		(INA226_sda__0__PS)
#define INA226_sda_0_PC		(INA226_sda__0__PC)
#define INA226_sda_0_DR		(INA226_sda__0__DR)
#define INA226_sda_0_SHIFT	(INA226_sda__0__SHIFT)
#define INA226_sda_0_INTR	((uint16)((uint16)0x0003u << (INA226_sda__0__SHIFT*2u)))

#define INA226_sda_INTR_ALL	 ((uint16)(INA226_sda_0_INTR))


#endif /* End Pins INA226_sda_ALIASES_H */


/* [] END OF FILE */
