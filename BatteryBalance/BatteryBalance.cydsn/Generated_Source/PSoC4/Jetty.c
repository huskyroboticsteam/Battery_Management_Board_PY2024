/***************************************************************************//**
* \file Jetty.c
* \version 4.0
*
* \brief
*  This file provides the source code to the API for the SCB Component.
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

#if (Jetty_SCB_MODE_I2C_INC)
    #include "Jetty_I2C_PVT.h"
#endif /* (Jetty_SCB_MODE_I2C_INC) */

#if (Jetty_SCB_MODE_EZI2C_INC)
    #include "Jetty_EZI2C_PVT.h"
#endif /* (Jetty_SCB_MODE_EZI2C_INC) */

#if (Jetty_SCB_MODE_SPI_INC || Jetty_SCB_MODE_UART_INC)
    #include "Jetty_SPI_UART_PVT.h"
#endif /* (Jetty_SCB_MODE_SPI_INC || Jetty_SCB_MODE_UART_INC) */


/***************************************
*    Run Time Configuration Vars
***************************************/

/* Stores internal component configuration for Unconfigured mode */
#if (Jetty_SCB_MODE_UNCONFIG_CONST_CFG)
    /* Common configuration variables */
    uint8 Jetty_scbMode = Jetty_SCB_MODE_UNCONFIG;
    uint8 Jetty_scbEnableWake;
    uint8 Jetty_scbEnableIntr;

    /* I2C configuration variables */
    uint8 Jetty_mode;
    uint8 Jetty_acceptAddr;

    /* SPI/UART configuration variables */
    volatile uint8 * Jetty_rxBuffer;
    uint8  Jetty_rxDataBits;
    uint32 Jetty_rxBufferSize;

    volatile uint8 * Jetty_txBuffer;
    uint8  Jetty_txDataBits;
    uint32 Jetty_txBufferSize;

    /* EZI2C configuration variables */
    uint8 Jetty_numberOfAddr;
    uint8 Jetty_subAddrSize;
#endif /* (Jetty_SCB_MODE_UNCONFIG_CONST_CFG) */


/***************************************
*     Common SCB Vars
***************************************/
/**
* \addtogroup group_general
* \{
*/

/** Jetty_initVar indicates whether the Jetty 
*  component has been initialized. The variable is initialized to 0 
*  and set to 1 the first time SCB_Start() is called. This allows 
*  the component to restart without reinitialization after the first 
*  call to the Jetty_Start() routine.
*
*  If re-initialization of the component is required, then the 
*  Jetty_Init() function can be called before the 
*  Jetty_Start() or Jetty_Enable() function.
*/
uint8 Jetty_initVar = 0u;


#if (! (Jetty_SCB_MODE_I2C_CONST_CFG || \
        Jetty_SCB_MODE_EZI2C_CONST_CFG))
    /** This global variable stores TX interrupt sources after 
    * Jetty_Stop() is called. Only these TX interrupt sources 
    * will be restored on a subsequent Jetty_Enable() call.
    */
    uint16 Jetty_IntrTxMask = 0u;
#endif /* (! (Jetty_SCB_MODE_I2C_CONST_CFG || \
              Jetty_SCB_MODE_EZI2C_CONST_CFG)) */
/** \} globals */

#if (Jetty_SCB_IRQ_INTERNAL)
#if !defined (CY_REMOVE_Jetty_CUSTOM_INTR_HANDLER)
    void (*Jetty_customIntrHandler)(void) = NULL;
#endif /* !defined (CY_REMOVE_Jetty_CUSTOM_INTR_HANDLER) */
#endif /* (Jetty_SCB_IRQ_INTERNAL) */


/***************************************
*    Private Function Prototypes
***************************************/

static void Jetty_ScbEnableIntr(void);
static void Jetty_ScbModeStop(void);
static void Jetty_ScbModePostEnable(void);


/*******************************************************************************
* Function Name: Jetty_Init
****************************************************************************//**
*
*  Initializes the Jetty component to operate in one of the selected
*  configurations: I2C, SPI, UART or EZI2C.
*  When the configuration is set to "Unconfigured SCB", this function does
*  not do any initialization. Use mode-specific initialization APIs instead:
*  Jetty_I2CInit, Jetty_SpiInit, 
*  Jetty_UartInit or Jetty_EzI2CInit.
*
*******************************************************************************/
void Jetty_Init(void)
{
#if (Jetty_SCB_MODE_UNCONFIG_CONST_CFG)
    if (Jetty_SCB_MODE_UNCONFIG_RUNTM_CFG)
    {
        Jetty_initVar = 0u;
    }
    else
    {
        /* Initialization was done before this function call */
    }

#elif (Jetty_SCB_MODE_I2C_CONST_CFG)
    Jetty_I2CInit();

#elif (Jetty_SCB_MODE_SPI_CONST_CFG)
    Jetty_SpiInit();

#elif (Jetty_SCB_MODE_UART_CONST_CFG)
    Jetty_UartInit();

#elif (Jetty_SCB_MODE_EZI2C_CONST_CFG)
    Jetty_EzI2CInit();

#endif /* (Jetty_SCB_MODE_UNCONFIG_CONST_CFG) */
}


/*******************************************************************************
* Function Name: Jetty_Enable
****************************************************************************//**
*
*  Enables Jetty component operation: activates the hardware and 
*  internal interrupt. It also restores TX interrupt sources disabled after the 
*  Jetty_Stop() function was called (note that level-triggered TX 
*  interrupt sources remain disabled to not cause code lock-up).
*  For I2C and EZI2C modes the interrupt is internal and mandatory for 
*  operation. For SPI and UART modes the interrupt can be configured as none, 
*  internal or external.
*  The Jetty configuration should be not changed when the component
*  is enabled. Any configuration changes should be made after disabling the 
*  component.
*  When configuration is set to “Unconfigured Jetty”, the component 
*  must first be initialized to operate in one of the following configurations: 
*  I2C, SPI, UART or EZ I2C, using the mode-specific initialization API. 
*  Otherwise this function does not enable the component.
*
*******************************************************************************/
void Jetty_Enable(void)
{
#if (Jetty_SCB_MODE_UNCONFIG_CONST_CFG)
    /* Enable SCB block, only if it is already configured */
    if (!Jetty_SCB_MODE_UNCONFIG_RUNTM_CFG)
    {
        Jetty_CTRL_REG |= Jetty_CTRL_ENABLED;

        Jetty_ScbEnableIntr();

        /* Call PostEnable function specific to current operation mode */
        Jetty_ScbModePostEnable();
    }
#else
    Jetty_CTRL_REG |= Jetty_CTRL_ENABLED;

    Jetty_ScbEnableIntr();

    /* Call PostEnable function specific to current operation mode */
    Jetty_ScbModePostEnable();
#endif /* (Jetty_SCB_MODE_UNCONFIG_CONST_CFG) */
}


/*******************************************************************************
* Function Name: Jetty_Start
****************************************************************************//**
*
*  Invokes Jetty_Init() and Jetty_Enable().
*  After this function call, the component is enabled and ready for operation.
*  When configuration is set to "Unconfigured SCB", the component must first be
*  initialized to operate in one of the following configurations: I2C, SPI, UART
*  or EZI2C. Otherwise this function does not enable the component.
*
* \globalvars
*  Jetty_initVar - used to check initial configuration, modified
*  on first function call.
*
*******************************************************************************/
void Jetty_Start(void)
{
    if (0u == Jetty_initVar)
    {
        Jetty_Init();
        Jetty_initVar = 1u; /* Component was initialized */
    }

    Jetty_Enable();
}


/*******************************************************************************
* Function Name: Jetty_Stop
****************************************************************************//**
*
*  Disables the Jetty component: disable the hardware and internal 
*  interrupt. It also disables all TX interrupt sources so as not to cause an 
*  unexpected interrupt trigger because after the component is enabled, the 
*  TX FIFO is empty.
*  Refer to the function Jetty_Enable() for the interrupt 
*  configuration details.
*  This function disables the SCB component without checking to see if 
*  communication is in progress. Before calling this function it may be 
*  necessary to check the status of communication to make sure communication 
*  is complete. If this is not done then communication could be stopped mid 
*  byte and corrupted data could result.
*
*******************************************************************************/
void Jetty_Stop(void)
{
#if (Jetty_SCB_IRQ_INTERNAL)
    Jetty_DisableInt();
#endif /* (Jetty_SCB_IRQ_INTERNAL) */

    /* Call Stop function specific to current operation mode */
    Jetty_ScbModeStop();

    /* Disable SCB IP */
    Jetty_CTRL_REG &= (uint32) ~Jetty_CTRL_ENABLED;

    /* Disable all TX interrupt sources so as not to cause an unexpected
    * interrupt trigger after the component will be enabled because the 
    * TX FIFO is empty.
    * For SCB IP v0, it is critical as it does not mask-out interrupt
    * sources when it is disabled. This can cause a code lock-up in the
    * interrupt handler because TX FIFO cannot be loaded after the block
    * is disabled.
    */
    Jetty_SetTxInterruptMode(Jetty_NO_INTR_SOURCES);

#if (Jetty_SCB_IRQ_INTERNAL)
    Jetty_ClearPendingInt();
#endif /* (Jetty_SCB_IRQ_INTERNAL) */
}


/*******************************************************************************
* Function Name: Jetty_SetRxFifoLevel
****************************************************************************//**
*
*  Sets level in the RX FIFO to generate a RX level interrupt.
*  When the RX FIFO has more entries than the RX FIFO level an RX level
*  interrupt request is generated.
*
*  \param level: Level in the RX FIFO to generate RX level interrupt.
*   The range of valid level values is between 0 and RX FIFO depth - 1.
*
*******************************************************************************/
void Jetty_SetRxFifoLevel(uint32 level)
{
    uint32 rxFifoCtrl;

    rxFifoCtrl = Jetty_RX_FIFO_CTRL_REG;

    rxFifoCtrl &= ((uint32) ~Jetty_RX_FIFO_CTRL_TRIGGER_LEVEL_MASK); /* Clear level mask bits */
    rxFifoCtrl |= ((uint32) (Jetty_RX_FIFO_CTRL_TRIGGER_LEVEL_MASK & level));

    Jetty_RX_FIFO_CTRL_REG = rxFifoCtrl;
}


/*******************************************************************************
* Function Name: Jetty_SetTxFifoLevel
****************************************************************************//**
*
*  Sets level in the TX FIFO to generate a TX level interrupt.
*  When the TX FIFO has less entries than the TX FIFO level an TX level
*  interrupt request is generated.
*
*  \param level: Level in the TX FIFO to generate TX level interrupt.
*   The range of valid level values is between 0 and TX FIFO depth - 1.
*
*******************************************************************************/
void Jetty_SetTxFifoLevel(uint32 level)
{
    uint32 txFifoCtrl;

    txFifoCtrl = Jetty_TX_FIFO_CTRL_REG;

    txFifoCtrl &= ((uint32) ~Jetty_TX_FIFO_CTRL_TRIGGER_LEVEL_MASK); /* Clear level mask bits */
    txFifoCtrl |= ((uint32) (Jetty_TX_FIFO_CTRL_TRIGGER_LEVEL_MASK & level));

    Jetty_TX_FIFO_CTRL_REG = txFifoCtrl;
}


#if (Jetty_SCB_IRQ_INTERNAL)
    /*******************************************************************************
    * Function Name: Jetty_SetCustomInterruptHandler
    ****************************************************************************//**
    *
    *  Registers a function to be called by the internal interrupt handler.
    *  First the function that is registered is called, then the internal interrupt
    *  handler performs any operation such as software buffer management functions
    *  before the interrupt returns.  It is the user's responsibility not to break
    *  the software buffer operations. Only one custom handler is supported, which
    *  is the function provided by the most recent call.
    *  At the initialization time no custom handler is registered.
    *
    *  \param func: Pointer to the function to register.
    *        The value NULL indicates to remove the current custom interrupt
    *        handler.
    *
    *******************************************************************************/
    void Jetty_SetCustomInterruptHandler(void (*func)(void))
    {
    #if !defined (CY_REMOVE_Jetty_CUSTOM_INTR_HANDLER)
        Jetty_customIntrHandler = func; /* Register interrupt handler */
    #else
        if (NULL != func)
        {
            /* Suppress compiler warning */
        }
    #endif /* !defined (CY_REMOVE_Jetty_CUSTOM_INTR_HANDLER) */
    }
#endif /* (Jetty_SCB_IRQ_INTERNAL) */


/*******************************************************************************
* Function Name: Jetty_ScbModeEnableIntr
****************************************************************************//**
*
*  Enables an interrupt for a specific mode.
*
*******************************************************************************/
static void Jetty_ScbEnableIntr(void)
{
#if (Jetty_SCB_IRQ_INTERNAL)
    /* Enable interrupt in NVIC */
    #if (Jetty_SCB_MODE_UNCONFIG_CONST_CFG)
        if (0u != Jetty_scbEnableIntr)
        {
            Jetty_EnableInt();
        }

    #else
        Jetty_EnableInt();

    #endif /* (Jetty_SCB_MODE_UNCONFIG_CONST_CFG) */
#endif /* (Jetty_SCB_IRQ_INTERNAL) */
}


/*******************************************************************************
* Function Name: Jetty_ScbModePostEnable
****************************************************************************//**
*
*  Calls the PostEnable function for a specific operation mode.
*
*******************************************************************************/
static void Jetty_ScbModePostEnable(void)
{
#if (Jetty_SCB_MODE_UNCONFIG_CONST_CFG)
#if (!Jetty_CY_SCBIP_V1)
    if (Jetty_SCB_MODE_SPI_RUNTM_CFG)
    {
        Jetty_SpiPostEnable();
    }
    else if (Jetty_SCB_MODE_UART_RUNTM_CFG)
    {
        Jetty_UartPostEnable();
    }
    else
    {
        /* Unknown mode: do nothing */
    }
#endif /* (!Jetty_CY_SCBIP_V1) */

#elif (Jetty_SCB_MODE_SPI_CONST_CFG)
    Jetty_SpiPostEnable();

#elif (Jetty_SCB_MODE_UART_CONST_CFG)
    Jetty_UartPostEnable();

#else
    /* Unknown mode: do nothing */
#endif /* (Jetty_SCB_MODE_UNCONFIG_CONST_CFG) */
}


/*******************************************************************************
* Function Name: Jetty_ScbModeStop
****************************************************************************//**
*
*  Calls the Stop function for a specific operation mode.
*
*******************************************************************************/
static void Jetty_ScbModeStop(void)
{
#if (Jetty_SCB_MODE_UNCONFIG_CONST_CFG)
    if (Jetty_SCB_MODE_I2C_RUNTM_CFG)
    {
        Jetty_I2CStop();
    }
    else if (Jetty_SCB_MODE_EZI2C_RUNTM_CFG)
    {
        Jetty_EzI2CStop();
    }
#if (!Jetty_CY_SCBIP_V1)
    else if (Jetty_SCB_MODE_SPI_RUNTM_CFG)
    {
        Jetty_SpiStop();
    }
    else if (Jetty_SCB_MODE_UART_RUNTM_CFG)
    {
        Jetty_UartStop();
    }
#endif /* (!Jetty_CY_SCBIP_V1) */
    else
    {
        /* Unknown mode: do nothing */
    }
#elif (Jetty_SCB_MODE_I2C_CONST_CFG)
    Jetty_I2CStop();

#elif (Jetty_SCB_MODE_EZI2C_CONST_CFG)
    Jetty_EzI2CStop();

#elif (Jetty_SCB_MODE_SPI_CONST_CFG)
    Jetty_SpiStop();

#elif (Jetty_SCB_MODE_UART_CONST_CFG)
    Jetty_UartStop();

#else
    /* Unknown mode: do nothing */
#endif /* (Jetty_SCB_MODE_UNCONFIG_CONST_CFG) */
}


#if (Jetty_SCB_MODE_UNCONFIG_CONST_CFG)
    /*******************************************************************************
    * Function Name: Jetty_SetPins
    ****************************************************************************//**
    *
    *  Sets the pins settings accordingly to the selected operation mode.
    *  Only available in the Unconfigured operation mode. The mode specific
    *  initialization function calls it.
    *  Pins configuration is set by PSoC Creator when a specific mode of operation
    *  is selected in design time.
    *
    *  \param mode:      Mode of SCB operation.
    *  \param subMode:   Sub-mode of SCB operation. It is only required for SPI and UART
    *             modes.
    *  \param uartEnableMask: enables TX or RX direction and RTS and CTS signals.
    *
    *******************************************************************************/
    void Jetty_SetPins(uint32 mode, uint32 subMode, uint32 uartEnableMask)
    {
        uint32 pinsDm[Jetty_SCB_PINS_NUMBER];
        uint32 i;
        
    #if (!Jetty_CY_SCBIP_V1)
        uint32 pinsInBuf = 0u;
    #endif /* (!Jetty_CY_SCBIP_V1) */
        
        uint32 hsiomSel[Jetty_SCB_PINS_NUMBER] = 
        {
            Jetty_RX_SCL_MOSI_HSIOM_SEL_GPIO,
            Jetty_TX_SDA_MISO_HSIOM_SEL_GPIO,
            0u,
            0u,
            0u,
            0u,
            0u,
        };

    #if (Jetty_CY_SCBIP_V1)
        /* Supress compiler warning. */
        if ((0u == subMode) || (0u == uartEnableMask))
        {
        }
    #endif /* (Jetty_CY_SCBIP_V1) */

        /* Set default HSIOM to GPIO and Drive Mode to Analog Hi-Z */
        for (i = 0u; i < Jetty_SCB_PINS_NUMBER; i++)
        {
            pinsDm[i] = Jetty_PIN_DM_ALG_HIZ;
        }

        if ((Jetty_SCB_MODE_I2C   == mode) ||
            (Jetty_SCB_MODE_EZI2C == mode))
        {
        #if (Jetty_RX_SCL_MOSI_PIN)
            hsiomSel[Jetty_RX_SCL_MOSI_PIN_INDEX] = Jetty_RX_SCL_MOSI_HSIOM_SEL_I2C;
            pinsDm  [Jetty_RX_SCL_MOSI_PIN_INDEX] = Jetty_PIN_DM_OD_LO;
        #elif (Jetty_RX_WAKE_SCL_MOSI_PIN)
            hsiomSel[Jetty_RX_WAKE_SCL_MOSI_PIN_INDEX] = Jetty_RX_WAKE_SCL_MOSI_HSIOM_SEL_I2C;
            pinsDm  [Jetty_RX_WAKE_SCL_MOSI_PIN_INDEX] = Jetty_PIN_DM_OD_LO;
        #else
        #endif /* (Jetty_RX_SCL_MOSI_PIN) */
        
        #if (Jetty_TX_SDA_MISO_PIN)
            hsiomSel[Jetty_TX_SDA_MISO_PIN_INDEX] = Jetty_TX_SDA_MISO_HSIOM_SEL_I2C;
            pinsDm  [Jetty_TX_SDA_MISO_PIN_INDEX] = Jetty_PIN_DM_OD_LO;
        #endif /* (Jetty_TX_SDA_MISO_PIN) */
        }
    #if (!Jetty_CY_SCBIP_V1)
        else if (Jetty_SCB_MODE_SPI == mode)
        {
        #if (Jetty_RX_SCL_MOSI_PIN)
            hsiomSel[Jetty_RX_SCL_MOSI_PIN_INDEX] = Jetty_RX_SCL_MOSI_HSIOM_SEL_SPI;
        #elif (Jetty_RX_WAKE_SCL_MOSI_PIN)
            hsiomSel[Jetty_RX_WAKE_SCL_MOSI_PIN_INDEX] = Jetty_RX_WAKE_SCL_MOSI_HSIOM_SEL_SPI;
        #else
        #endif /* (Jetty_RX_SCL_MOSI_PIN) */
        
        #if (Jetty_TX_SDA_MISO_PIN)
            hsiomSel[Jetty_TX_SDA_MISO_PIN_INDEX] = Jetty_TX_SDA_MISO_HSIOM_SEL_SPI;
        #endif /* (Jetty_TX_SDA_MISO_PIN) */
        
        #if (Jetty_SCLK_PIN)
            hsiomSel[Jetty_SCLK_PIN_INDEX] = Jetty_SCLK_HSIOM_SEL_SPI;
        #endif /* (Jetty_SCLK_PIN) */

            if (Jetty_SPI_SLAVE == subMode)
            {
                /* Slave */
                pinsDm[Jetty_RX_SCL_MOSI_PIN_INDEX] = Jetty_PIN_DM_DIG_HIZ;
                pinsDm[Jetty_TX_SDA_MISO_PIN_INDEX] = Jetty_PIN_DM_STRONG;
                pinsDm[Jetty_SCLK_PIN_INDEX] = Jetty_PIN_DM_DIG_HIZ;

            #if (Jetty_SS0_PIN)
                /* Only SS0 is valid choice for Slave */
                hsiomSel[Jetty_SS0_PIN_INDEX] = Jetty_SS0_HSIOM_SEL_SPI;
                pinsDm  [Jetty_SS0_PIN_INDEX] = Jetty_PIN_DM_DIG_HIZ;
            #endif /* (Jetty_SS0_PIN) */

            #if (Jetty_TX_SDA_MISO_PIN)
                /* Disable input buffer */
                 pinsInBuf |= Jetty_TX_SDA_MISO_PIN_MASK;
            #endif /* (Jetty_TX_SDA_MISO_PIN) */
            }
            else 
            {
                /* (Master) */
                pinsDm[Jetty_RX_SCL_MOSI_PIN_INDEX] = Jetty_PIN_DM_STRONG;
                pinsDm[Jetty_TX_SDA_MISO_PIN_INDEX] = Jetty_PIN_DM_DIG_HIZ;
                pinsDm[Jetty_SCLK_PIN_INDEX] = Jetty_PIN_DM_STRONG;

            #if (Jetty_SS0_PIN)
                hsiomSel [Jetty_SS0_PIN_INDEX] = Jetty_SS0_HSIOM_SEL_SPI;
                pinsDm   [Jetty_SS0_PIN_INDEX] = Jetty_PIN_DM_STRONG;
                pinsInBuf |= Jetty_SS0_PIN_MASK;
            #endif /* (Jetty_SS0_PIN) */

            #if (Jetty_SS1_PIN)
                hsiomSel [Jetty_SS1_PIN_INDEX] = Jetty_SS1_HSIOM_SEL_SPI;
                pinsDm   [Jetty_SS1_PIN_INDEX] = Jetty_PIN_DM_STRONG;
                pinsInBuf |= Jetty_SS1_PIN_MASK;
            #endif /* (Jetty_SS1_PIN) */

            #if (Jetty_SS2_PIN)
                hsiomSel [Jetty_SS2_PIN_INDEX] = Jetty_SS2_HSIOM_SEL_SPI;
                pinsDm   [Jetty_SS2_PIN_INDEX] = Jetty_PIN_DM_STRONG;
                pinsInBuf |= Jetty_SS2_PIN_MASK;
            #endif /* (Jetty_SS2_PIN) */

            #if (Jetty_SS3_PIN)
                hsiomSel [Jetty_SS3_PIN_INDEX] = Jetty_SS3_HSIOM_SEL_SPI;
                pinsDm   [Jetty_SS3_PIN_INDEX] = Jetty_PIN_DM_STRONG;
                pinsInBuf |= Jetty_SS3_PIN_MASK;
            #endif /* (Jetty_SS3_PIN) */

                /* Disable input buffers */
            #if (Jetty_RX_SCL_MOSI_PIN)
                pinsInBuf |= Jetty_RX_SCL_MOSI_PIN_MASK;
            #elif (Jetty_RX_WAKE_SCL_MOSI_PIN)
                pinsInBuf |= Jetty_RX_WAKE_SCL_MOSI_PIN_MASK;
            #else
            #endif /* (Jetty_RX_SCL_MOSI_PIN) */

            #if (Jetty_SCLK_PIN)
                pinsInBuf |= Jetty_SCLK_PIN_MASK;
            #endif /* (Jetty_SCLK_PIN) */
            }
        }
        else /* UART */
        {
            if (Jetty_UART_MODE_SMARTCARD == subMode)
            {
                /* SmartCard */
            #if (Jetty_TX_SDA_MISO_PIN)
                hsiomSel[Jetty_TX_SDA_MISO_PIN_INDEX] = Jetty_TX_SDA_MISO_HSIOM_SEL_UART;
                pinsDm  [Jetty_TX_SDA_MISO_PIN_INDEX] = Jetty_PIN_DM_OD_LO;
            #endif /* (Jetty_TX_SDA_MISO_PIN) */
            }
            else /* Standard or IrDA */
            {
                if (0u != (Jetty_UART_RX_PIN_ENABLE & uartEnableMask))
                {
                #if (Jetty_RX_SCL_MOSI_PIN)
                    hsiomSel[Jetty_RX_SCL_MOSI_PIN_INDEX] = Jetty_RX_SCL_MOSI_HSIOM_SEL_UART;
                    pinsDm  [Jetty_RX_SCL_MOSI_PIN_INDEX] = Jetty_PIN_DM_DIG_HIZ;
                #elif (Jetty_RX_WAKE_SCL_MOSI_PIN)
                    hsiomSel[Jetty_RX_WAKE_SCL_MOSI_PIN_INDEX] = Jetty_RX_WAKE_SCL_MOSI_HSIOM_SEL_UART;
                    pinsDm  [Jetty_RX_WAKE_SCL_MOSI_PIN_INDEX] = Jetty_PIN_DM_DIG_HIZ;
                #else
                #endif /* (Jetty_RX_SCL_MOSI_PIN) */
                }

                if (0u != (Jetty_UART_TX_PIN_ENABLE & uartEnableMask))
                {
                #if (Jetty_TX_SDA_MISO_PIN)
                    hsiomSel[Jetty_TX_SDA_MISO_PIN_INDEX] = Jetty_TX_SDA_MISO_HSIOM_SEL_UART;
                    pinsDm  [Jetty_TX_SDA_MISO_PIN_INDEX] = Jetty_PIN_DM_STRONG;
                    
                    /* Disable input buffer */
                    pinsInBuf |= Jetty_TX_SDA_MISO_PIN_MASK;
                #endif /* (Jetty_TX_SDA_MISO_PIN) */
                }

            #if !(Jetty_CY_SCBIP_V0 || Jetty_CY_SCBIP_V1)
                if (Jetty_UART_MODE_STD == subMode)
                {
                    if (0u != (Jetty_UART_CTS_PIN_ENABLE & uartEnableMask))
                    {
                        /* CTS input is multiplexed with SCLK */
                    #if (Jetty_SCLK_PIN)
                        hsiomSel[Jetty_SCLK_PIN_INDEX] = Jetty_SCLK_HSIOM_SEL_UART;
                        pinsDm  [Jetty_SCLK_PIN_INDEX] = Jetty_PIN_DM_DIG_HIZ;
                    #endif /* (Jetty_SCLK_PIN) */
                    }

                    if (0u != (Jetty_UART_RTS_PIN_ENABLE & uartEnableMask))
                    {
                        /* RTS output is multiplexed with SS0 */
                    #if (Jetty_SS0_PIN)
                        hsiomSel[Jetty_SS0_PIN_INDEX] = Jetty_SS0_HSIOM_SEL_UART;
                        pinsDm  [Jetty_SS0_PIN_INDEX] = Jetty_PIN_DM_STRONG;
                        
                        /* Disable input buffer */
                        pinsInBuf |= Jetty_SS0_PIN_MASK;
                    #endif /* (Jetty_SS0_PIN) */
                    }
                }
            #endif /* !(Jetty_CY_SCBIP_V0 || Jetty_CY_SCBIP_V1) */
            }
        }
    #endif /* (!Jetty_CY_SCBIP_V1) */

    /* Configure pins: set HSIOM, DM and InputBufEnable */
    /* Note: the DR register settings do not effect the pin output if HSIOM is other than GPIO */

    #if (Jetty_RX_SCL_MOSI_PIN)
        Jetty_SET_HSIOM_SEL(Jetty_RX_SCL_MOSI_HSIOM_REG,
                                       Jetty_RX_SCL_MOSI_HSIOM_MASK,
                                       Jetty_RX_SCL_MOSI_HSIOM_POS,
                                        hsiomSel[Jetty_RX_SCL_MOSI_PIN_INDEX]);

        Jetty_uart_rx_i2c_scl_spi_mosi_SetDriveMode((uint8) pinsDm[Jetty_RX_SCL_MOSI_PIN_INDEX]);

        #if (!Jetty_CY_SCBIP_V1)
            Jetty_SET_INP_DIS(Jetty_uart_rx_i2c_scl_spi_mosi_INP_DIS,
                                         Jetty_uart_rx_i2c_scl_spi_mosi_MASK,
                                         (0u != (pinsInBuf & Jetty_RX_SCL_MOSI_PIN_MASK)));
        #endif /* (!Jetty_CY_SCBIP_V1) */
    
    #elif (Jetty_RX_WAKE_SCL_MOSI_PIN)
        Jetty_SET_HSIOM_SEL(Jetty_RX_WAKE_SCL_MOSI_HSIOM_REG,
                                       Jetty_RX_WAKE_SCL_MOSI_HSIOM_MASK,
                                       Jetty_RX_WAKE_SCL_MOSI_HSIOM_POS,
                                       hsiomSel[Jetty_RX_WAKE_SCL_MOSI_PIN_INDEX]);

        Jetty_uart_rx_wake_i2c_scl_spi_mosi_SetDriveMode((uint8)
                                                               pinsDm[Jetty_RX_WAKE_SCL_MOSI_PIN_INDEX]);

        Jetty_SET_INP_DIS(Jetty_uart_rx_wake_i2c_scl_spi_mosi_INP_DIS,
                                     Jetty_uart_rx_wake_i2c_scl_spi_mosi_MASK,
                                     (0u != (pinsInBuf & Jetty_RX_WAKE_SCL_MOSI_PIN_MASK)));

         /* Set interrupt on falling edge */
        Jetty_SET_INCFG_TYPE(Jetty_RX_WAKE_SCL_MOSI_INTCFG_REG,
                                        Jetty_RX_WAKE_SCL_MOSI_INTCFG_TYPE_MASK,
                                        Jetty_RX_WAKE_SCL_MOSI_INTCFG_TYPE_POS,
                                        Jetty_INTCFG_TYPE_FALLING_EDGE);
    #else
    #endif /* (Jetty_RX_WAKE_SCL_MOSI_PIN) */

    #if (Jetty_TX_SDA_MISO_PIN)
        Jetty_SET_HSIOM_SEL(Jetty_TX_SDA_MISO_HSIOM_REG,
                                       Jetty_TX_SDA_MISO_HSIOM_MASK,
                                       Jetty_TX_SDA_MISO_HSIOM_POS,
                                        hsiomSel[Jetty_TX_SDA_MISO_PIN_INDEX]);

        Jetty_uart_tx_i2c_sda_spi_miso_SetDriveMode((uint8) pinsDm[Jetty_TX_SDA_MISO_PIN_INDEX]);

    #if (!Jetty_CY_SCBIP_V1)
        Jetty_SET_INP_DIS(Jetty_uart_tx_i2c_sda_spi_miso_INP_DIS,
                                     Jetty_uart_tx_i2c_sda_spi_miso_MASK,
                                    (0u != (pinsInBuf & Jetty_TX_SDA_MISO_PIN_MASK)));
    #endif /* (!Jetty_CY_SCBIP_V1) */
    #endif /* (Jetty_RX_SCL_MOSI_PIN) */

    #if (Jetty_SCLK_PIN)
        Jetty_SET_HSIOM_SEL(Jetty_SCLK_HSIOM_REG,
                                       Jetty_SCLK_HSIOM_MASK,
                                       Jetty_SCLK_HSIOM_POS,
                                       hsiomSel[Jetty_SCLK_PIN_INDEX]);

        Jetty_spi_sclk_SetDriveMode((uint8) pinsDm[Jetty_SCLK_PIN_INDEX]);

        Jetty_SET_INP_DIS(Jetty_spi_sclk_INP_DIS,
                                     Jetty_spi_sclk_MASK,
                                     (0u != (pinsInBuf & Jetty_SCLK_PIN_MASK)));
    #endif /* (Jetty_SCLK_PIN) */

    #if (Jetty_SS0_PIN)
        Jetty_SET_HSIOM_SEL(Jetty_SS0_HSIOM_REG,
                                       Jetty_SS0_HSIOM_MASK,
                                       Jetty_SS0_HSIOM_POS,
                                       hsiomSel[Jetty_SS0_PIN_INDEX]);

        Jetty_spi_ss0_SetDriveMode((uint8) pinsDm[Jetty_SS0_PIN_INDEX]);

        Jetty_SET_INP_DIS(Jetty_spi_ss0_INP_DIS,
                                     Jetty_spi_ss0_MASK,
                                     (0u != (pinsInBuf & Jetty_SS0_PIN_MASK)));
    #endif /* (Jetty_SS0_PIN) */

    #if (Jetty_SS1_PIN)
        Jetty_SET_HSIOM_SEL(Jetty_SS1_HSIOM_REG,
                                       Jetty_SS1_HSIOM_MASK,
                                       Jetty_SS1_HSIOM_POS,
                                       hsiomSel[Jetty_SS1_PIN_INDEX]);

        Jetty_spi_ss1_SetDriveMode((uint8) pinsDm[Jetty_SS1_PIN_INDEX]);

        Jetty_SET_INP_DIS(Jetty_spi_ss1_INP_DIS,
                                     Jetty_spi_ss1_MASK,
                                     (0u != (pinsInBuf & Jetty_SS1_PIN_MASK)));
    #endif /* (Jetty_SS1_PIN) */

    #if (Jetty_SS2_PIN)
        Jetty_SET_HSIOM_SEL(Jetty_SS2_HSIOM_REG,
                                       Jetty_SS2_HSIOM_MASK,
                                       Jetty_SS2_HSIOM_POS,
                                       hsiomSel[Jetty_SS2_PIN_INDEX]);

        Jetty_spi_ss2_SetDriveMode((uint8) pinsDm[Jetty_SS2_PIN_INDEX]);

        Jetty_SET_INP_DIS(Jetty_spi_ss2_INP_DIS,
                                     Jetty_spi_ss2_MASK,
                                     (0u != (pinsInBuf & Jetty_SS2_PIN_MASK)));
    #endif /* (Jetty_SS2_PIN) */

    #if (Jetty_SS3_PIN)
        Jetty_SET_HSIOM_SEL(Jetty_SS3_HSIOM_REG,
                                       Jetty_SS3_HSIOM_MASK,
                                       Jetty_SS3_HSIOM_POS,
                                       hsiomSel[Jetty_SS3_PIN_INDEX]);

        Jetty_spi_ss3_SetDriveMode((uint8) pinsDm[Jetty_SS3_PIN_INDEX]);

        Jetty_SET_INP_DIS(Jetty_spi_ss3_INP_DIS,
                                     Jetty_spi_ss3_MASK,
                                     (0u != (pinsInBuf & Jetty_SS3_PIN_MASK)));
    #endif /* (Jetty_SS3_PIN) */
    }

#endif /* (Jetty_SCB_MODE_UNCONFIG_CONST_CFG) */


#if (Jetty_CY_SCBIP_V0 || Jetty_CY_SCBIP_V1)
    /*******************************************************************************
    * Function Name: Jetty_I2CSlaveNackGeneration
    ****************************************************************************//**
    *
    *  Sets command to generate NACK to the address or data.
    *
    *******************************************************************************/
    void Jetty_I2CSlaveNackGeneration(void)
    {
        /* Check for EC_AM toggle condition: EC_AM and clock stretching for address are enabled */
        if ((0u != (Jetty_CTRL_REG & Jetty_CTRL_EC_AM_MODE)) &&
            (0u == (Jetty_I2C_CTRL_REG & Jetty_I2C_CTRL_S_NOT_READY_ADDR_NACK)))
        {
            /* Toggle EC_AM before NACK generation */
            Jetty_CTRL_REG &= ~Jetty_CTRL_EC_AM_MODE;
            Jetty_CTRL_REG |=  Jetty_CTRL_EC_AM_MODE;
        }

        Jetty_I2C_SLAVE_CMD_REG = Jetty_I2C_SLAVE_CMD_S_NACK;
    }
#endif /* (Jetty_CY_SCBIP_V0 || Jetty_CY_SCBIP_V1) */


/* [] END OF FILE */
