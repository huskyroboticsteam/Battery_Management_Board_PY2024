/*******************************************************************************
* File Name: Jetty_scl.h  
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

#if !defined(CY_PINS_Jetty_scl_ALIASES_H) /* Pins Jetty_scl_ALIASES_H */
#define CY_PINS_Jetty_scl_ALIASES_H

#include "cytypes.h"
#include "cyfitter.h"
#include "cypins.h"


/***************************************
*              Constants        
***************************************/
#define Jetty_scl_0			(Jetty_scl__0__PC)
#define Jetty_scl_0_PS		(Jetty_scl__0__PS)
#define Jetty_scl_0_PC		(Jetty_scl__0__PC)
#define Jetty_scl_0_DR		(Jetty_scl__0__DR)
#define Jetty_scl_0_SHIFT	(Jetty_scl__0__SHIFT)
#define Jetty_scl_0_INTR	((uint16)((uint16)0x0003u << (Jetty_scl__0__SHIFT*2u)))

#define Jetty_scl_INTR_ALL	 ((uint16)(Jetty_scl_0_INTR))


#endif /* End Pins Jetty_scl_ALIASES_H */


/* [] END OF FILE */
