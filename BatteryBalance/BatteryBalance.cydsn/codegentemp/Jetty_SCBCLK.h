/*******************************************************************************
* File Name: Jetty_SCBCLK.h
* Version 2.20
*
*  Description:
*   Provides the function and constant definitions for the clock component.
*
*  Note:
*
********************************************************************************
* Copyright 2008-2012, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions, 
* disclaimers, and limitations in the end user license agreement accompanying 
* the software package with which this file was provided.
*******************************************************************************/

#if !defined(CY_CLOCK_Jetty_SCBCLK_H)
#define CY_CLOCK_Jetty_SCBCLK_H

#include <cytypes.h>
#include <cyfitter.h>


/***************************************
*        Function Prototypes
***************************************/
#if defined CYREG_PERI_DIV_CMD

void Jetty_SCBCLK_StartEx(uint32 alignClkDiv);
#define Jetty_SCBCLK_Start() \
    Jetty_SCBCLK_StartEx(Jetty_SCBCLK__PA_DIV_ID)

#else

void Jetty_SCBCLK_Start(void);

#endif/* CYREG_PERI_DIV_CMD */

void Jetty_SCBCLK_Stop(void);

void Jetty_SCBCLK_SetFractionalDividerRegister(uint16 clkDivider, uint8 clkFractional);

uint16 Jetty_SCBCLK_GetDividerRegister(void);
uint8  Jetty_SCBCLK_GetFractionalDividerRegister(void);

#define Jetty_SCBCLK_Enable()                         Jetty_SCBCLK_Start()
#define Jetty_SCBCLK_Disable()                        Jetty_SCBCLK_Stop()
#define Jetty_SCBCLK_SetDividerRegister(clkDivider, reset)  \
    Jetty_SCBCLK_SetFractionalDividerRegister((clkDivider), 0u)
#define Jetty_SCBCLK_SetDivider(clkDivider)           Jetty_SCBCLK_SetDividerRegister((clkDivider), 1u)
#define Jetty_SCBCLK_SetDividerValue(clkDivider)      Jetty_SCBCLK_SetDividerRegister((clkDivider) - 1u, 1u)


/***************************************
*             Registers
***************************************/
#if defined CYREG_PERI_DIV_CMD

#define Jetty_SCBCLK_DIV_ID     Jetty_SCBCLK__DIV_ID

#define Jetty_SCBCLK_CMD_REG    (*(reg32 *)CYREG_PERI_DIV_CMD)
#define Jetty_SCBCLK_CTRL_REG   (*(reg32 *)Jetty_SCBCLK__CTRL_REGISTER)
#define Jetty_SCBCLK_DIV_REG    (*(reg32 *)Jetty_SCBCLK__DIV_REGISTER)

#define Jetty_SCBCLK_CMD_DIV_SHIFT          (0u)
#define Jetty_SCBCLK_CMD_PA_DIV_SHIFT       (8u)
#define Jetty_SCBCLK_CMD_DISABLE_SHIFT      (30u)
#define Jetty_SCBCLK_CMD_ENABLE_SHIFT       (31u)

#define Jetty_SCBCLK_CMD_DISABLE_MASK       ((uint32)((uint32)1u << Jetty_SCBCLK_CMD_DISABLE_SHIFT))
#define Jetty_SCBCLK_CMD_ENABLE_MASK        ((uint32)((uint32)1u << Jetty_SCBCLK_CMD_ENABLE_SHIFT))

#define Jetty_SCBCLK_DIV_FRAC_MASK  (0x000000F8u)
#define Jetty_SCBCLK_DIV_FRAC_SHIFT (3u)
#define Jetty_SCBCLK_DIV_INT_MASK   (0xFFFFFF00u)
#define Jetty_SCBCLK_DIV_INT_SHIFT  (8u)

#else 

#define Jetty_SCBCLK_DIV_REG        (*(reg32 *)Jetty_SCBCLK__REGISTER)
#define Jetty_SCBCLK_ENABLE_REG     Jetty_SCBCLK_DIV_REG
#define Jetty_SCBCLK_DIV_FRAC_MASK  Jetty_SCBCLK__FRAC_MASK
#define Jetty_SCBCLK_DIV_FRAC_SHIFT (16u)
#define Jetty_SCBCLK_DIV_INT_MASK   Jetty_SCBCLK__DIVIDER_MASK
#define Jetty_SCBCLK_DIV_INT_SHIFT  (0u)

#endif/* CYREG_PERI_DIV_CMD */

#endif /* !defined(CY_CLOCK_Jetty_SCBCLK_H) */

/* [] END OF FILE */
