/***************************************************************************//**
* \file INA226_PINS.h
* \version 4.0
*
* \brief
*  This file provides constants and parameter values for the pin components
*  buried into SCB Component.
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

#if !defined(CY_SCB_PINS_INA226_H)
#define CY_SCB_PINS_INA226_H

#include "cydevice_trm.h"
#include "cyfitter.h"
#include "cytypes.h"


/***************************************
*   Conditional Compilation Parameters
****************************************/

/* Unconfigured pins */
#define INA226_REMOVE_RX_WAKE_SCL_MOSI_PIN  (1u)
#define INA226_REMOVE_RX_SCL_MOSI_PIN      (1u)
#define INA226_REMOVE_TX_SDA_MISO_PIN      (1u)
#define INA226_REMOVE_CTS_SCLK_PIN      (1u)
#define INA226_REMOVE_RTS_SS0_PIN      (1u)
#define INA226_REMOVE_SS1_PIN                 (1u)
#define INA226_REMOVE_SS2_PIN                 (1u)
#define INA226_REMOVE_SS3_PIN                 (1u)

/* Mode defined pins */
#define INA226_REMOVE_I2C_PINS                (0u)
#define INA226_REMOVE_SPI_MASTER_PINS         (1u)
#define INA226_REMOVE_SPI_MASTER_SCLK_PIN     (1u)
#define INA226_REMOVE_SPI_MASTER_MOSI_PIN     (1u)
#define INA226_REMOVE_SPI_MASTER_MISO_PIN     (1u)
#define INA226_REMOVE_SPI_MASTER_SS0_PIN      (1u)
#define INA226_REMOVE_SPI_MASTER_SS1_PIN      (1u)
#define INA226_REMOVE_SPI_MASTER_SS2_PIN      (1u)
#define INA226_REMOVE_SPI_MASTER_SS3_PIN      (1u)
#define INA226_REMOVE_SPI_SLAVE_PINS          (1u)
#define INA226_REMOVE_SPI_SLAVE_MOSI_PIN      (1u)
#define INA226_REMOVE_SPI_SLAVE_MISO_PIN      (1u)
#define INA226_REMOVE_UART_TX_PIN             (1u)
#define INA226_REMOVE_UART_RX_TX_PIN          (1u)
#define INA226_REMOVE_UART_RX_PIN             (1u)
#define INA226_REMOVE_UART_RX_WAKE_PIN        (1u)
#define INA226_REMOVE_UART_RTS_PIN            (1u)
#define INA226_REMOVE_UART_CTS_PIN            (1u)

/* Unconfigured pins */
#define INA226_RX_WAKE_SCL_MOSI_PIN (0u == INA226_REMOVE_RX_WAKE_SCL_MOSI_PIN)
#define INA226_RX_SCL_MOSI_PIN     (0u == INA226_REMOVE_RX_SCL_MOSI_PIN)
#define INA226_TX_SDA_MISO_PIN     (0u == INA226_REMOVE_TX_SDA_MISO_PIN)
#define INA226_CTS_SCLK_PIN     (0u == INA226_REMOVE_CTS_SCLK_PIN)
#define INA226_RTS_SS0_PIN     (0u == INA226_REMOVE_RTS_SS0_PIN)
#define INA226_SS1_PIN                (0u == INA226_REMOVE_SS1_PIN)
#define INA226_SS2_PIN                (0u == INA226_REMOVE_SS2_PIN)
#define INA226_SS3_PIN                (0u == INA226_REMOVE_SS3_PIN)

/* Mode defined pins */
#define INA226_I2C_PINS               (0u == INA226_REMOVE_I2C_PINS)
#define INA226_SPI_MASTER_PINS        (0u == INA226_REMOVE_SPI_MASTER_PINS)
#define INA226_SPI_MASTER_SCLK_PIN    (0u == INA226_REMOVE_SPI_MASTER_SCLK_PIN)
#define INA226_SPI_MASTER_MOSI_PIN    (0u == INA226_REMOVE_SPI_MASTER_MOSI_PIN)
#define INA226_SPI_MASTER_MISO_PIN    (0u == INA226_REMOVE_SPI_MASTER_MISO_PIN)
#define INA226_SPI_MASTER_SS0_PIN     (0u == INA226_REMOVE_SPI_MASTER_SS0_PIN)
#define INA226_SPI_MASTER_SS1_PIN     (0u == INA226_REMOVE_SPI_MASTER_SS1_PIN)
#define INA226_SPI_MASTER_SS2_PIN     (0u == INA226_REMOVE_SPI_MASTER_SS2_PIN)
#define INA226_SPI_MASTER_SS3_PIN     (0u == INA226_REMOVE_SPI_MASTER_SS3_PIN)
#define INA226_SPI_SLAVE_PINS         (0u == INA226_REMOVE_SPI_SLAVE_PINS)
#define INA226_SPI_SLAVE_MOSI_PIN     (0u == INA226_REMOVE_SPI_SLAVE_MOSI_PIN)
#define INA226_SPI_SLAVE_MISO_PIN     (0u == INA226_REMOVE_SPI_SLAVE_MISO_PIN)
#define INA226_UART_TX_PIN            (0u == INA226_REMOVE_UART_TX_PIN)
#define INA226_UART_RX_TX_PIN         (0u == INA226_REMOVE_UART_RX_TX_PIN)
#define INA226_UART_RX_PIN            (0u == INA226_REMOVE_UART_RX_PIN)
#define INA226_UART_RX_WAKE_PIN       (0u == INA226_REMOVE_UART_RX_WAKE_PIN)
#define INA226_UART_RTS_PIN           (0u == INA226_REMOVE_UART_RTS_PIN)
#define INA226_UART_CTS_PIN           (0u == INA226_REMOVE_UART_CTS_PIN)


/***************************************
*             Includes
****************************************/

#if (INA226_RX_WAKE_SCL_MOSI_PIN)
    #include "INA226_uart_rx_wake_i2c_scl_spi_mosi.h"
#endif /* (INA226_RX_SCL_MOSI) */

#if (INA226_RX_SCL_MOSI_PIN)
    #include "INA226_uart_rx_i2c_scl_spi_mosi.h"
#endif /* (INA226_RX_SCL_MOSI) */

#if (INA226_TX_SDA_MISO_PIN)
    #include "INA226_uart_tx_i2c_sda_spi_miso.h"
#endif /* (INA226_TX_SDA_MISO) */

#if (INA226_CTS_SCLK_PIN)
    #include "INA226_uart_cts_spi_sclk.h"
#endif /* (INA226_CTS_SCLK) */

#if (INA226_RTS_SS0_PIN)
    #include "INA226_uart_rts_spi_ss0.h"
#endif /* (INA226_RTS_SS0_PIN) */

#if (INA226_SS1_PIN)
    #include "INA226_spi_ss1.h"
#endif /* (INA226_SS1_PIN) */

#if (INA226_SS2_PIN)
    #include "INA226_spi_ss2.h"
#endif /* (INA226_SS2_PIN) */

#if (INA226_SS3_PIN)
    #include "INA226_spi_ss3.h"
#endif /* (INA226_SS3_PIN) */

#if (INA226_I2C_PINS)
    #include "INA226_scl.h"
    #include "INA226_sda.h"
#endif /* (INA226_I2C_PINS) */

#if (INA226_SPI_MASTER_PINS)
#if (INA226_SPI_MASTER_SCLK_PIN)
    #include "INA226_sclk_m.h"
#endif /* (INA226_SPI_MASTER_SCLK_PIN) */

#if (INA226_SPI_MASTER_MOSI_PIN)
    #include "INA226_mosi_m.h"
#endif /* (INA226_SPI_MASTER_MOSI_PIN) */

#if (INA226_SPI_MASTER_MISO_PIN)
    #include "INA226_miso_m.h"
#endif /*(INA226_SPI_MASTER_MISO_PIN) */
#endif /* (INA226_SPI_MASTER_PINS) */

#if (INA226_SPI_SLAVE_PINS)
    #include "INA226_sclk_s.h"
    #include "INA226_ss_s.h"

#if (INA226_SPI_SLAVE_MOSI_PIN)
    #include "INA226_mosi_s.h"
#endif /* (INA226_SPI_SLAVE_MOSI_PIN) */

#if (INA226_SPI_SLAVE_MISO_PIN)
    #include "INA226_miso_s.h"
#endif /*(INA226_SPI_SLAVE_MISO_PIN) */
#endif /* (INA226_SPI_SLAVE_PINS) */

#if (INA226_SPI_MASTER_SS0_PIN)
    #include "INA226_ss0_m.h"
#endif /* (INA226_SPI_MASTER_SS0_PIN) */

#if (INA226_SPI_MASTER_SS1_PIN)
    #include "INA226_ss1_m.h"
#endif /* (INA226_SPI_MASTER_SS1_PIN) */

#if (INA226_SPI_MASTER_SS2_PIN)
    #include "INA226_ss2_m.h"
#endif /* (INA226_SPI_MASTER_SS2_PIN) */

#if (INA226_SPI_MASTER_SS3_PIN)
    #include "INA226_ss3_m.h"
#endif /* (INA226_SPI_MASTER_SS3_PIN) */

#if (INA226_UART_TX_PIN)
    #include "INA226_tx.h"
#endif /* (INA226_UART_TX_PIN) */

#if (INA226_UART_RX_TX_PIN)
    #include "INA226_rx_tx.h"
#endif /* (INA226_UART_RX_TX_PIN) */

#if (INA226_UART_RX_PIN)
    #include "INA226_rx.h"
#endif /* (INA226_UART_RX_PIN) */

#if (INA226_UART_RX_WAKE_PIN)
    #include "INA226_rx_wake.h"
#endif /* (INA226_UART_RX_WAKE_PIN) */

#if (INA226_UART_RTS_PIN)
    #include "INA226_rts.h"
#endif /* (INA226_UART_RTS_PIN) */

#if (INA226_UART_CTS_PIN)
    #include "INA226_cts.h"
#endif /* (INA226_UART_CTS_PIN) */


/***************************************
*              Registers
***************************************/

#if (INA226_RX_SCL_MOSI_PIN)
    #define INA226_RX_SCL_MOSI_HSIOM_REG   (*(reg32 *) INA226_uart_rx_i2c_scl_spi_mosi__0__HSIOM)
    #define INA226_RX_SCL_MOSI_HSIOM_PTR   ( (reg32 *) INA226_uart_rx_i2c_scl_spi_mosi__0__HSIOM)
    
    #define INA226_RX_SCL_MOSI_HSIOM_MASK      (INA226_uart_rx_i2c_scl_spi_mosi__0__HSIOM_MASK)
    #define INA226_RX_SCL_MOSI_HSIOM_POS       (INA226_uart_rx_i2c_scl_spi_mosi__0__HSIOM_SHIFT)
    #define INA226_RX_SCL_MOSI_HSIOM_SEL_GPIO  (INA226_uart_rx_i2c_scl_spi_mosi__0__HSIOM_GPIO)
    #define INA226_RX_SCL_MOSI_HSIOM_SEL_I2C   (INA226_uart_rx_i2c_scl_spi_mosi__0__HSIOM_I2C)
    #define INA226_RX_SCL_MOSI_HSIOM_SEL_SPI   (INA226_uart_rx_i2c_scl_spi_mosi__0__HSIOM_SPI)
    #define INA226_RX_SCL_MOSI_HSIOM_SEL_UART  (INA226_uart_rx_i2c_scl_spi_mosi__0__HSIOM_UART)
    
#elif (INA226_RX_WAKE_SCL_MOSI_PIN)
    #define INA226_RX_WAKE_SCL_MOSI_HSIOM_REG   (*(reg32 *) INA226_uart_rx_wake_i2c_scl_spi_mosi__0__HSIOM)
    #define INA226_RX_WAKE_SCL_MOSI_HSIOM_PTR   ( (reg32 *) INA226_uart_rx_wake_i2c_scl_spi_mosi__0__HSIOM)
    
    #define INA226_RX_WAKE_SCL_MOSI_HSIOM_MASK      (INA226_uart_rx_wake_i2c_scl_spi_mosi__0__HSIOM_MASK)
    #define INA226_RX_WAKE_SCL_MOSI_HSIOM_POS       (INA226_uart_rx_wake_i2c_scl_spi_mosi__0__HSIOM_SHIFT)
    #define INA226_RX_WAKE_SCL_MOSI_HSIOM_SEL_GPIO  (INA226_uart_rx_wake_i2c_scl_spi_mosi__0__HSIOM_GPIO)
    #define INA226_RX_WAKE_SCL_MOSI_HSIOM_SEL_I2C   (INA226_uart_rx_wake_i2c_scl_spi_mosi__0__HSIOM_I2C)
    #define INA226_RX_WAKE_SCL_MOSI_HSIOM_SEL_SPI   (INA226_uart_rx_wake_i2c_scl_spi_mosi__0__HSIOM_SPI)
    #define INA226_RX_WAKE_SCL_MOSI_HSIOM_SEL_UART  (INA226_uart_rx_wake_i2c_scl_spi_mosi__0__HSIOM_UART)    
   
    #define INA226_RX_WAKE_SCL_MOSI_INTCFG_REG (*(reg32 *) INA226_uart_rx_wake_i2c_scl_spi_mosi__0__INTCFG)
    #define INA226_RX_WAKE_SCL_MOSI_INTCFG_PTR ( (reg32 *) INA226_uart_rx_wake_i2c_scl_spi_mosi__0__INTCFG)
    #define INA226_RX_WAKE_SCL_MOSI_INTCFG_TYPE_POS  (INA226_uart_rx_wake_i2c_scl_spi_mosi__SHIFT)
    #define INA226_RX_WAKE_SCL_MOSI_INTCFG_TYPE_MASK ((uint32) INA226_INTCFG_TYPE_MASK << \
                                                                           INA226_RX_WAKE_SCL_MOSI_INTCFG_TYPE_POS)
#else
    /* None of pins INA226_RX_SCL_MOSI_PIN or INA226_RX_WAKE_SCL_MOSI_PIN present.*/
#endif /* (INA226_RX_SCL_MOSI_PIN) */

#if (INA226_TX_SDA_MISO_PIN)
    #define INA226_TX_SDA_MISO_HSIOM_REG   (*(reg32 *) INA226_uart_tx_i2c_sda_spi_miso__0__HSIOM)
    #define INA226_TX_SDA_MISO_HSIOM_PTR   ( (reg32 *) INA226_uart_tx_i2c_sda_spi_miso__0__HSIOM)
    
    #define INA226_TX_SDA_MISO_HSIOM_MASK      (INA226_uart_tx_i2c_sda_spi_miso__0__HSIOM_MASK)
    #define INA226_TX_SDA_MISO_HSIOM_POS       (INA226_uart_tx_i2c_sda_spi_miso__0__HSIOM_SHIFT)
    #define INA226_TX_SDA_MISO_HSIOM_SEL_GPIO  (INA226_uart_tx_i2c_sda_spi_miso__0__HSIOM_GPIO)
    #define INA226_TX_SDA_MISO_HSIOM_SEL_I2C   (INA226_uart_tx_i2c_sda_spi_miso__0__HSIOM_I2C)
    #define INA226_TX_SDA_MISO_HSIOM_SEL_SPI   (INA226_uart_tx_i2c_sda_spi_miso__0__HSIOM_SPI)
    #define INA226_TX_SDA_MISO_HSIOM_SEL_UART  (INA226_uart_tx_i2c_sda_spi_miso__0__HSIOM_UART)
#endif /* (INA226_TX_SDA_MISO_PIN) */

#if (INA226_CTS_SCLK_PIN)
    #define INA226_CTS_SCLK_HSIOM_REG   (*(reg32 *) INA226_uart_cts_spi_sclk__0__HSIOM)
    #define INA226_CTS_SCLK_HSIOM_PTR   ( (reg32 *) INA226_uart_cts_spi_sclk__0__HSIOM)
    
    #define INA226_CTS_SCLK_HSIOM_MASK      (INA226_uart_cts_spi_sclk__0__HSIOM_MASK)
    #define INA226_CTS_SCLK_HSIOM_POS       (INA226_uart_cts_spi_sclk__0__HSIOM_SHIFT)
    #define INA226_CTS_SCLK_HSIOM_SEL_GPIO  (INA226_uart_cts_spi_sclk__0__HSIOM_GPIO)
    #define INA226_CTS_SCLK_HSIOM_SEL_I2C   (INA226_uart_cts_spi_sclk__0__HSIOM_I2C)
    #define INA226_CTS_SCLK_HSIOM_SEL_SPI   (INA226_uart_cts_spi_sclk__0__HSIOM_SPI)
    #define INA226_CTS_SCLK_HSIOM_SEL_UART  (INA226_uart_cts_spi_sclk__0__HSIOM_UART)
#endif /* (INA226_CTS_SCLK_PIN) */

#if (INA226_RTS_SS0_PIN)
    #define INA226_RTS_SS0_HSIOM_REG   (*(reg32 *) INA226_uart_rts_spi_ss0__0__HSIOM)
    #define INA226_RTS_SS0_HSIOM_PTR   ( (reg32 *) INA226_uart_rts_spi_ss0__0__HSIOM)
    
    #define INA226_RTS_SS0_HSIOM_MASK      (INA226_uart_rts_spi_ss0__0__HSIOM_MASK)
    #define INA226_RTS_SS0_HSIOM_POS       (INA226_uart_rts_spi_ss0__0__HSIOM_SHIFT)
    #define INA226_RTS_SS0_HSIOM_SEL_GPIO  (INA226_uart_rts_spi_ss0__0__HSIOM_GPIO)
    #define INA226_RTS_SS0_HSIOM_SEL_I2C   (INA226_uart_rts_spi_ss0__0__HSIOM_I2C)
    #define INA226_RTS_SS0_HSIOM_SEL_SPI   (INA226_uart_rts_spi_ss0__0__HSIOM_SPI)
#if !(INA226_CY_SCBIP_V0 || INA226_CY_SCBIP_V1)
    #define INA226_RTS_SS0_HSIOM_SEL_UART  (INA226_uart_rts_spi_ss0__0__HSIOM_UART)
#endif /* !(INA226_CY_SCBIP_V0 || INA226_CY_SCBIP_V1) */
#endif /* (INA226_RTS_SS0_PIN) */

#if (INA226_SS1_PIN)
    #define INA226_SS1_HSIOM_REG  (*(reg32 *) INA226_spi_ss1__0__HSIOM)
    #define INA226_SS1_HSIOM_PTR  ( (reg32 *) INA226_spi_ss1__0__HSIOM)
    
    #define INA226_SS1_HSIOM_MASK     (INA226_spi_ss1__0__HSIOM_MASK)
    #define INA226_SS1_HSIOM_POS      (INA226_spi_ss1__0__HSIOM_SHIFT)
    #define INA226_SS1_HSIOM_SEL_GPIO (INA226_spi_ss1__0__HSIOM_GPIO)
    #define INA226_SS1_HSIOM_SEL_I2C  (INA226_spi_ss1__0__HSIOM_I2C)
    #define INA226_SS1_HSIOM_SEL_SPI  (INA226_spi_ss1__0__HSIOM_SPI)
#endif /* (INA226_SS1_PIN) */

#if (INA226_SS2_PIN)
    #define INA226_SS2_HSIOM_REG     (*(reg32 *) INA226_spi_ss2__0__HSIOM)
    #define INA226_SS2_HSIOM_PTR     ( (reg32 *) INA226_spi_ss2__0__HSIOM)
    
    #define INA226_SS2_HSIOM_MASK     (INA226_spi_ss2__0__HSIOM_MASK)
    #define INA226_SS2_HSIOM_POS      (INA226_spi_ss2__0__HSIOM_SHIFT)
    #define INA226_SS2_HSIOM_SEL_GPIO (INA226_spi_ss2__0__HSIOM_GPIO)
    #define INA226_SS2_HSIOM_SEL_I2C  (INA226_spi_ss2__0__HSIOM_I2C)
    #define INA226_SS2_HSIOM_SEL_SPI  (INA226_spi_ss2__0__HSIOM_SPI)
#endif /* (INA226_SS2_PIN) */

#if (INA226_SS3_PIN)
    #define INA226_SS3_HSIOM_REG     (*(reg32 *) INA226_spi_ss3__0__HSIOM)
    #define INA226_SS3_HSIOM_PTR     ( (reg32 *) INA226_spi_ss3__0__HSIOM)
    
    #define INA226_SS3_HSIOM_MASK     (INA226_spi_ss3__0__HSIOM_MASK)
    #define INA226_SS3_HSIOM_POS      (INA226_spi_ss3__0__HSIOM_SHIFT)
    #define INA226_SS3_HSIOM_SEL_GPIO (INA226_spi_ss3__0__HSIOM_GPIO)
    #define INA226_SS3_HSIOM_SEL_I2C  (INA226_spi_ss3__0__HSIOM_I2C)
    #define INA226_SS3_HSIOM_SEL_SPI  (INA226_spi_ss3__0__HSIOM_SPI)
#endif /* (INA226_SS3_PIN) */

#if (INA226_I2C_PINS)
    #define INA226_SCL_HSIOM_REG  (*(reg32 *) INA226_scl__0__HSIOM)
    #define INA226_SCL_HSIOM_PTR  ( (reg32 *) INA226_scl__0__HSIOM)
    
    #define INA226_SCL_HSIOM_MASK     (INA226_scl__0__HSIOM_MASK)
    #define INA226_SCL_HSIOM_POS      (INA226_scl__0__HSIOM_SHIFT)
    #define INA226_SCL_HSIOM_SEL_GPIO (INA226_sda__0__HSIOM_GPIO)
    #define INA226_SCL_HSIOM_SEL_I2C  (INA226_sda__0__HSIOM_I2C)
    
    #define INA226_SDA_HSIOM_REG  (*(reg32 *) INA226_sda__0__HSIOM)
    #define INA226_SDA_HSIOM_PTR  ( (reg32 *) INA226_sda__0__HSIOM)
    
    #define INA226_SDA_HSIOM_MASK     (INA226_sda__0__HSIOM_MASK)
    #define INA226_SDA_HSIOM_POS      (INA226_sda__0__HSIOM_SHIFT)
    #define INA226_SDA_HSIOM_SEL_GPIO (INA226_sda__0__HSIOM_GPIO)
    #define INA226_SDA_HSIOM_SEL_I2C  (INA226_sda__0__HSIOM_I2C)
#endif /* (INA226_I2C_PINS) */

#if (INA226_SPI_SLAVE_PINS)
    #define INA226_SCLK_S_HSIOM_REG   (*(reg32 *) INA226_sclk_s__0__HSIOM)
    #define INA226_SCLK_S_HSIOM_PTR   ( (reg32 *) INA226_sclk_s__0__HSIOM)
    
    #define INA226_SCLK_S_HSIOM_MASK      (INA226_sclk_s__0__HSIOM_MASK)
    #define INA226_SCLK_S_HSIOM_POS       (INA226_sclk_s__0__HSIOM_SHIFT)
    #define INA226_SCLK_S_HSIOM_SEL_GPIO  (INA226_sclk_s__0__HSIOM_GPIO)
    #define INA226_SCLK_S_HSIOM_SEL_SPI   (INA226_sclk_s__0__HSIOM_SPI)
    
    #define INA226_SS0_S_HSIOM_REG    (*(reg32 *) INA226_ss0_s__0__HSIOM)
    #define INA226_SS0_S_HSIOM_PTR    ( (reg32 *) INA226_ss0_s__0__HSIOM)
    
    #define INA226_SS0_S_HSIOM_MASK       (INA226_ss0_s__0__HSIOM_MASK)
    #define INA226_SS0_S_HSIOM_POS        (INA226_ss0_s__0__HSIOM_SHIFT)
    #define INA226_SS0_S_HSIOM_SEL_GPIO   (INA226_ss0_s__0__HSIOM_GPIO)  
    #define INA226_SS0_S_HSIOM_SEL_SPI    (INA226_ss0_s__0__HSIOM_SPI)
#endif /* (INA226_SPI_SLAVE_PINS) */

#if (INA226_SPI_SLAVE_MOSI_PIN)
    #define INA226_MOSI_S_HSIOM_REG   (*(reg32 *) INA226_mosi_s__0__HSIOM)
    #define INA226_MOSI_S_HSIOM_PTR   ( (reg32 *) INA226_mosi_s__0__HSIOM)
    
    #define INA226_MOSI_S_HSIOM_MASK      (INA226_mosi_s__0__HSIOM_MASK)
    #define INA226_MOSI_S_HSIOM_POS       (INA226_mosi_s__0__HSIOM_SHIFT)
    #define INA226_MOSI_S_HSIOM_SEL_GPIO  (INA226_mosi_s__0__HSIOM_GPIO)
    #define INA226_MOSI_S_HSIOM_SEL_SPI   (INA226_mosi_s__0__HSIOM_SPI)
#endif /* (INA226_SPI_SLAVE_MOSI_PIN) */

#if (INA226_SPI_SLAVE_MISO_PIN)
    #define INA226_MISO_S_HSIOM_REG   (*(reg32 *) INA226_miso_s__0__HSIOM)
    #define INA226_MISO_S_HSIOM_PTR   ( (reg32 *) INA226_miso_s__0__HSIOM)
    
    #define INA226_MISO_S_HSIOM_MASK      (INA226_miso_s__0__HSIOM_MASK)
    #define INA226_MISO_S_HSIOM_POS       (INA226_miso_s__0__HSIOM_SHIFT)
    #define INA226_MISO_S_HSIOM_SEL_GPIO  (INA226_miso_s__0__HSIOM_GPIO)
    #define INA226_MISO_S_HSIOM_SEL_SPI   (INA226_miso_s__0__HSIOM_SPI)
#endif /* (INA226_SPI_SLAVE_MISO_PIN) */

#if (INA226_SPI_MASTER_MISO_PIN)
    #define INA226_MISO_M_HSIOM_REG   (*(reg32 *) INA226_miso_m__0__HSIOM)
    #define INA226_MISO_M_HSIOM_PTR   ( (reg32 *) INA226_miso_m__0__HSIOM)
    
    #define INA226_MISO_M_HSIOM_MASK      (INA226_miso_m__0__HSIOM_MASK)
    #define INA226_MISO_M_HSIOM_POS       (INA226_miso_m__0__HSIOM_SHIFT)
    #define INA226_MISO_M_HSIOM_SEL_GPIO  (INA226_miso_m__0__HSIOM_GPIO)
    #define INA226_MISO_M_HSIOM_SEL_SPI   (INA226_miso_m__0__HSIOM_SPI)
#endif /* (INA226_SPI_MASTER_MISO_PIN) */

#if (INA226_SPI_MASTER_MOSI_PIN)
    #define INA226_MOSI_M_HSIOM_REG   (*(reg32 *) INA226_mosi_m__0__HSIOM)
    #define INA226_MOSI_M_HSIOM_PTR   ( (reg32 *) INA226_mosi_m__0__HSIOM)
    
    #define INA226_MOSI_M_HSIOM_MASK      (INA226_mosi_m__0__HSIOM_MASK)
    #define INA226_MOSI_M_HSIOM_POS       (INA226_mosi_m__0__HSIOM_SHIFT)
    #define INA226_MOSI_M_HSIOM_SEL_GPIO  (INA226_mosi_m__0__HSIOM_GPIO)
    #define INA226_MOSI_M_HSIOM_SEL_SPI   (INA226_mosi_m__0__HSIOM_SPI)
#endif /* (INA226_SPI_MASTER_MOSI_PIN) */

#if (INA226_SPI_MASTER_SCLK_PIN)
    #define INA226_SCLK_M_HSIOM_REG   (*(reg32 *) INA226_sclk_m__0__HSIOM)
    #define INA226_SCLK_M_HSIOM_PTR   ( (reg32 *) INA226_sclk_m__0__HSIOM)
    
    #define INA226_SCLK_M_HSIOM_MASK      (INA226_sclk_m__0__HSIOM_MASK)
    #define INA226_SCLK_M_HSIOM_POS       (INA226_sclk_m__0__HSIOM_SHIFT)
    #define INA226_SCLK_M_HSIOM_SEL_GPIO  (INA226_sclk_m__0__HSIOM_GPIO)
    #define INA226_SCLK_M_HSIOM_SEL_SPI   (INA226_sclk_m__0__HSIOM_SPI)
#endif /* (INA226_SPI_MASTER_SCLK_PIN) */

#if (INA226_SPI_MASTER_SS0_PIN)
    #define INA226_SS0_M_HSIOM_REG    (*(reg32 *) INA226_ss0_m__0__HSIOM)
    #define INA226_SS0_M_HSIOM_PTR    ( (reg32 *) INA226_ss0_m__0__HSIOM)
    
    #define INA226_SS0_M_HSIOM_MASK       (INA226_ss0_m__0__HSIOM_MASK)
    #define INA226_SS0_M_HSIOM_POS        (INA226_ss0_m__0__HSIOM_SHIFT)
    #define INA226_SS0_M_HSIOM_SEL_GPIO   (INA226_ss0_m__0__HSIOM_GPIO)
    #define INA226_SS0_M_HSIOM_SEL_SPI    (INA226_ss0_m__0__HSIOM_SPI)
#endif /* (INA226_SPI_MASTER_SS0_PIN) */

#if (INA226_SPI_MASTER_SS1_PIN)
    #define INA226_SS1_M_HSIOM_REG    (*(reg32 *) INA226_ss1_m__0__HSIOM)
    #define INA226_SS1_M_HSIOM_PTR    ( (reg32 *) INA226_ss1_m__0__HSIOM)
    
    #define INA226_SS1_M_HSIOM_MASK       (INA226_ss1_m__0__HSIOM_MASK)
    #define INA226_SS1_M_HSIOM_POS        (INA226_ss1_m__0__HSIOM_SHIFT)
    #define INA226_SS1_M_HSIOM_SEL_GPIO   (INA226_ss1_m__0__HSIOM_GPIO)
    #define INA226_SS1_M_HSIOM_SEL_SPI    (INA226_ss1_m__0__HSIOM_SPI)
#endif /* (INA226_SPI_MASTER_SS1_PIN) */

#if (INA226_SPI_MASTER_SS2_PIN)
    #define INA226_SS2_M_HSIOM_REG    (*(reg32 *) INA226_ss2_m__0__HSIOM)
    #define INA226_SS2_M_HSIOM_PTR    ( (reg32 *) INA226_ss2_m__0__HSIOM)
    
    #define INA226_SS2_M_HSIOM_MASK       (INA226_ss2_m__0__HSIOM_MASK)
    #define INA226_SS2_M_HSIOM_POS        (INA226_ss2_m__0__HSIOM_SHIFT)
    #define INA226_SS2_M_HSIOM_SEL_GPIO   (INA226_ss2_m__0__HSIOM_GPIO)
    #define INA226_SS2_M_HSIOM_SEL_SPI    (INA226_ss2_m__0__HSIOM_SPI)
#endif /* (INA226_SPI_MASTER_SS2_PIN) */

#if (INA226_SPI_MASTER_SS3_PIN)
    #define INA226_SS3_M_HSIOM_REG    (*(reg32 *) INA226_ss3_m__0__HSIOM)
    #define INA226_SS3_M_HSIOM_PTR    ( (reg32 *) INA226_ss3_m__0__HSIOM)
    
    #define INA226_SS3_M_HSIOM_MASK      (INA226_ss3_m__0__HSIOM_MASK)
    #define INA226_SS3_M_HSIOM_POS       (INA226_ss3_m__0__HSIOM_SHIFT)
    #define INA226_SS3_M_HSIOM_SEL_GPIO  (INA226_ss3_m__0__HSIOM_GPIO)
    #define INA226_SS3_M_HSIOM_SEL_SPI   (INA226_ss3_m__0__HSIOM_SPI)
#endif /* (INA226_SPI_MASTER_SS3_PIN) */

#if (INA226_UART_RX_PIN)
    #define INA226_RX_HSIOM_REG   (*(reg32 *) INA226_rx__0__HSIOM)
    #define INA226_RX_HSIOM_PTR   ( (reg32 *) INA226_rx__0__HSIOM)
    
    #define INA226_RX_HSIOM_MASK      (INA226_rx__0__HSIOM_MASK)
    #define INA226_RX_HSIOM_POS       (INA226_rx__0__HSIOM_SHIFT)
    #define INA226_RX_HSIOM_SEL_GPIO  (INA226_rx__0__HSIOM_GPIO)
    #define INA226_RX_HSIOM_SEL_UART  (INA226_rx__0__HSIOM_UART)
#endif /* (INA226_UART_RX_PIN) */

#if (INA226_UART_RX_WAKE_PIN)
    #define INA226_RX_WAKE_HSIOM_REG   (*(reg32 *) INA226_rx_wake__0__HSIOM)
    #define INA226_RX_WAKE_HSIOM_PTR   ( (reg32 *) INA226_rx_wake__0__HSIOM)
    
    #define INA226_RX_WAKE_HSIOM_MASK      (INA226_rx_wake__0__HSIOM_MASK)
    #define INA226_RX_WAKE_HSIOM_POS       (INA226_rx_wake__0__HSIOM_SHIFT)
    #define INA226_RX_WAKE_HSIOM_SEL_GPIO  (INA226_rx_wake__0__HSIOM_GPIO)
    #define INA226_RX_WAKE_HSIOM_SEL_UART  (INA226_rx_wake__0__HSIOM_UART)
#endif /* (INA226_UART_WAKE_RX_PIN) */

#if (INA226_UART_CTS_PIN)
    #define INA226_CTS_HSIOM_REG   (*(reg32 *) INA226_cts__0__HSIOM)
    #define INA226_CTS_HSIOM_PTR   ( (reg32 *) INA226_cts__0__HSIOM)
    
    #define INA226_CTS_HSIOM_MASK      (INA226_cts__0__HSIOM_MASK)
    #define INA226_CTS_HSIOM_POS       (INA226_cts__0__HSIOM_SHIFT)
    #define INA226_CTS_HSIOM_SEL_GPIO  (INA226_cts__0__HSIOM_GPIO)
    #define INA226_CTS_HSIOM_SEL_UART  (INA226_cts__0__HSIOM_UART)
#endif /* (INA226_UART_CTS_PIN) */

#if (INA226_UART_TX_PIN)
    #define INA226_TX_HSIOM_REG   (*(reg32 *) INA226_tx__0__HSIOM)
    #define INA226_TX_HSIOM_PTR   ( (reg32 *) INA226_tx__0__HSIOM)
    
    #define INA226_TX_HSIOM_MASK      (INA226_tx__0__HSIOM_MASK)
    #define INA226_TX_HSIOM_POS       (INA226_tx__0__HSIOM_SHIFT)
    #define INA226_TX_HSIOM_SEL_GPIO  (INA226_tx__0__HSIOM_GPIO)
    #define INA226_TX_HSIOM_SEL_UART  (INA226_tx__0__HSIOM_UART)
#endif /* (INA226_UART_TX_PIN) */

#if (INA226_UART_RX_TX_PIN)
    #define INA226_RX_TX_HSIOM_REG   (*(reg32 *) INA226_rx_tx__0__HSIOM)
    #define INA226_RX_TX_HSIOM_PTR   ( (reg32 *) INA226_rx_tx__0__HSIOM)
    
    #define INA226_RX_TX_HSIOM_MASK      (INA226_rx_tx__0__HSIOM_MASK)
    #define INA226_RX_TX_HSIOM_POS       (INA226_rx_tx__0__HSIOM_SHIFT)
    #define INA226_RX_TX_HSIOM_SEL_GPIO  (INA226_rx_tx__0__HSIOM_GPIO)
    #define INA226_RX_TX_HSIOM_SEL_UART  (INA226_rx_tx__0__HSIOM_UART)
#endif /* (INA226_UART_RX_TX_PIN) */

#if (INA226_UART_RTS_PIN)
    #define INA226_RTS_HSIOM_REG      (*(reg32 *) INA226_rts__0__HSIOM)
    #define INA226_RTS_HSIOM_PTR      ( (reg32 *) INA226_rts__0__HSIOM)
    
    #define INA226_RTS_HSIOM_MASK     (INA226_rts__0__HSIOM_MASK)
    #define INA226_RTS_HSIOM_POS      (INA226_rts__0__HSIOM_SHIFT)    
    #define INA226_RTS_HSIOM_SEL_GPIO (INA226_rts__0__HSIOM_GPIO)
    #define INA226_RTS_HSIOM_SEL_UART (INA226_rts__0__HSIOM_UART)    
#endif /* (INA226_UART_RTS_PIN) */


/***************************************
*        Registers Constants
***************************************/

/* HSIOM switch values. */ 
#define INA226_HSIOM_DEF_SEL      (0x00u)
#define INA226_HSIOM_GPIO_SEL     (0x00u)
/* The HSIOM values provided below are valid only for INA226_CY_SCBIP_V0 
* and INA226_CY_SCBIP_V1. It is not recommended to use them for 
* INA226_CY_SCBIP_V2. Use pin name specific HSIOM constants provided 
* above instead for any SCB IP block version.
*/
#define INA226_HSIOM_UART_SEL     (0x09u)
#define INA226_HSIOM_I2C_SEL      (0x0Eu)
#define INA226_HSIOM_SPI_SEL      (0x0Fu)

/* Pins settings index. */
#define INA226_RX_WAKE_SCL_MOSI_PIN_INDEX   (0u)
#define INA226_RX_SCL_MOSI_PIN_INDEX       (0u)
#define INA226_TX_SDA_MISO_PIN_INDEX       (1u)
#define INA226_CTS_SCLK_PIN_INDEX       (2u)
#define INA226_RTS_SS0_PIN_INDEX       (3u)
#define INA226_SS1_PIN_INDEX                  (4u)
#define INA226_SS2_PIN_INDEX                  (5u)
#define INA226_SS3_PIN_INDEX                  (6u)

/* Pins settings mask. */
#define INA226_RX_WAKE_SCL_MOSI_PIN_MASK ((uint32) 0x01u << INA226_RX_WAKE_SCL_MOSI_PIN_INDEX)
#define INA226_RX_SCL_MOSI_PIN_MASK     ((uint32) 0x01u << INA226_RX_SCL_MOSI_PIN_INDEX)
#define INA226_TX_SDA_MISO_PIN_MASK     ((uint32) 0x01u << INA226_TX_SDA_MISO_PIN_INDEX)
#define INA226_CTS_SCLK_PIN_MASK     ((uint32) 0x01u << INA226_CTS_SCLK_PIN_INDEX)
#define INA226_RTS_SS0_PIN_MASK     ((uint32) 0x01u << INA226_RTS_SS0_PIN_INDEX)
#define INA226_SS1_PIN_MASK                ((uint32) 0x01u << INA226_SS1_PIN_INDEX)
#define INA226_SS2_PIN_MASK                ((uint32) 0x01u << INA226_SS2_PIN_INDEX)
#define INA226_SS3_PIN_MASK                ((uint32) 0x01u << INA226_SS3_PIN_INDEX)

/* Pin interrupt constants. */
#define INA226_INTCFG_TYPE_MASK           (0x03u)
#define INA226_INTCFG_TYPE_FALLING_EDGE   (0x02u)

/* Pin Drive Mode constants. */
#define INA226_PIN_DM_ALG_HIZ  (0u)
#define INA226_PIN_DM_DIG_HIZ  (1u)
#define INA226_PIN_DM_OD_LO    (4u)
#define INA226_PIN_DM_STRONG   (6u)


/***************************************
*          Macro Definitions
***************************************/

/* Return drive mode of the pin */
#define INA226_DM_MASK    (0x7u)
#define INA226_DM_SIZE    (3u)
#define INA226_GET_P4_PIN_DM(reg, pos) \
    ( ((reg) & (uint32) ((uint32) INA226_DM_MASK << (INA226_DM_SIZE * (pos)))) >> \
                                                              (INA226_DM_SIZE * (pos)) )

#if (INA226_TX_SDA_MISO_PIN)
    #define INA226_CHECK_TX_SDA_MISO_PIN_USED \
                (INA226_PIN_DM_ALG_HIZ != \
                    INA226_GET_P4_PIN_DM(INA226_uart_tx_i2c_sda_spi_miso_PC, \
                                                   INA226_uart_tx_i2c_sda_spi_miso_SHIFT))
#endif /* (INA226_TX_SDA_MISO_PIN) */

#if (INA226_RTS_SS0_PIN)
    #define INA226_CHECK_RTS_SS0_PIN_USED \
                (INA226_PIN_DM_ALG_HIZ != \
                    INA226_GET_P4_PIN_DM(INA226_uart_rts_spi_ss0_PC, \
                                                   INA226_uart_rts_spi_ss0_SHIFT))
#endif /* (INA226_RTS_SS0_PIN) */

/* Set bits-mask in register */
#define INA226_SET_REGISTER_BITS(reg, mask, pos, mode) \
                    do                                           \
                    {                                            \
                        (reg) = (((reg) & ((uint32) ~(uint32) (mask))) | ((uint32) ((uint32) (mode) << (pos)))); \
                    }while(0)

/* Set bit in the register */
#define INA226_SET_REGISTER_BIT(reg, mask, val) \
                    ((val) ? ((reg) |= (mask)) : ((reg) &= ((uint32) ~((uint32) (mask)))))

#define INA226_SET_HSIOM_SEL(reg, mask, pos, sel) INA226_SET_REGISTER_BITS(reg, mask, pos, sel)
#define INA226_SET_INCFG_TYPE(reg, mask, pos, intType) \
                                                        INA226_SET_REGISTER_BITS(reg, mask, pos, intType)
#define INA226_SET_INP_DIS(reg, mask, val) INA226_SET_REGISTER_BIT(reg, mask, val)

/* INA226_SET_I2C_SCL_DR(val) - Sets I2C SCL DR register.
*  INA226_SET_I2C_SCL_HSIOM_SEL(sel) - Sets I2C SCL HSIOM settings.
*/
/* SCB I2C: scl signal */
#if (INA226_CY_SCBIP_V0)
#if (INA226_I2C_PINS)
    #define INA226_SET_I2C_SCL_DR(val) INA226_scl_Write(val)

    #define INA226_SET_I2C_SCL_HSIOM_SEL(sel) \
                          INA226_SET_HSIOM_SEL(INA226_SCL_HSIOM_REG,  \
                                                         INA226_SCL_HSIOM_MASK, \
                                                         INA226_SCL_HSIOM_POS,  \
                                                         (sel))
    #define INA226_WAIT_SCL_SET_HIGH  (0u == INA226_scl_Read())

/* Unconfigured SCB: scl signal */
#elif (INA226_RX_WAKE_SCL_MOSI_PIN)
    #define INA226_SET_I2C_SCL_DR(val) \
                            INA226_uart_rx_wake_i2c_scl_spi_mosi_Write(val)

    #define INA226_SET_I2C_SCL_HSIOM_SEL(sel) \
                    INA226_SET_HSIOM_SEL(INA226_RX_WAKE_SCL_MOSI_HSIOM_REG,  \
                                                   INA226_RX_WAKE_SCL_MOSI_HSIOM_MASK, \
                                                   INA226_RX_WAKE_SCL_MOSI_HSIOM_POS,  \
                                                   (sel))

    #define INA226_WAIT_SCL_SET_HIGH  (0u == INA226_uart_rx_wake_i2c_scl_spi_mosi_Read())

#elif (INA226_RX_SCL_MOSI_PIN)
    #define INA226_SET_I2C_SCL_DR(val) \
                            INA226_uart_rx_i2c_scl_spi_mosi_Write(val)


    #define INA226_SET_I2C_SCL_HSIOM_SEL(sel) \
                            INA226_SET_HSIOM_SEL(INA226_RX_SCL_MOSI_HSIOM_REG,  \
                                                           INA226_RX_SCL_MOSI_HSIOM_MASK, \
                                                           INA226_RX_SCL_MOSI_HSIOM_POS,  \
                                                           (sel))

    #define INA226_WAIT_SCL_SET_HIGH  (0u == INA226_uart_rx_i2c_scl_spi_mosi_Read())

#else
    #define INA226_SET_I2C_SCL_DR(val)        do{ /* Does nothing */ }while(0)
    #define INA226_SET_I2C_SCL_HSIOM_SEL(sel) do{ /* Does nothing */ }while(0)

    #define INA226_WAIT_SCL_SET_HIGH  (0u)
#endif /* (INA226_I2C_PINS) */

/* SCB I2C: sda signal */
#if (INA226_I2C_PINS)
    #define INA226_WAIT_SDA_SET_HIGH  (0u == INA226_sda_Read())
/* Unconfigured SCB: sda signal */
#elif (INA226_TX_SDA_MISO_PIN)
    #define INA226_WAIT_SDA_SET_HIGH  (0u == INA226_uart_tx_i2c_sda_spi_miso_Read())
#else
    #define INA226_WAIT_SDA_SET_HIGH  (0u)
#endif /* (INA226_MOSI_SCL_RX_PIN) */
#endif /* (INA226_CY_SCBIP_V0) */

/* Clear UART wakeup source */
#if (INA226_RX_SCL_MOSI_PIN)
    #define INA226_CLEAR_UART_RX_WAKE_INTR        do{ /* Does nothing */ }while(0)
    
#elif (INA226_RX_WAKE_SCL_MOSI_PIN)
    #define INA226_CLEAR_UART_RX_WAKE_INTR \
            do{                                      \
                (void) INA226_uart_rx_wake_i2c_scl_spi_mosi_ClearInterrupt(); \
            }while(0)

#elif(INA226_UART_RX_WAKE_PIN)
    #define INA226_CLEAR_UART_RX_WAKE_INTR \
            do{                                      \
                (void) INA226_rx_wake_ClearInterrupt(); \
            }while(0)
#else
#endif /* (INA226_RX_SCL_MOSI_PIN) */


/***************************************
* The following code is DEPRECATED and
* must not be used.
***************************************/

/* Unconfigured pins */
#define INA226_REMOVE_MOSI_SCL_RX_WAKE_PIN    INA226_REMOVE_RX_WAKE_SCL_MOSI_PIN
#define INA226_REMOVE_MOSI_SCL_RX_PIN         INA226_REMOVE_RX_SCL_MOSI_PIN
#define INA226_REMOVE_MISO_SDA_TX_PIN         INA226_REMOVE_TX_SDA_MISO_PIN
#ifndef INA226_REMOVE_SCLK_PIN
#define INA226_REMOVE_SCLK_PIN                INA226_REMOVE_CTS_SCLK_PIN
#endif /* INA226_REMOVE_SCLK_PIN */
#ifndef INA226_REMOVE_SS0_PIN
#define INA226_REMOVE_SS0_PIN                 INA226_REMOVE_RTS_SS0_PIN
#endif /* INA226_REMOVE_SS0_PIN */

/* Unconfigured pins */
#define INA226_MOSI_SCL_RX_WAKE_PIN   INA226_RX_WAKE_SCL_MOSI_PIN
#define INA226_MOSI_SCL_RX_PIN        INA226_RX_SCL_MOSI_PIN
#define INA226_MISO_SDA_TX_PIN        INA226_TX_SDA_MISO_PIN
#ifndef INA226_SCLK_PIN
#define INA226_SCLK_PIN               INA226_CTS_SCLK_PIN
#endif /* INA226_SCLK_PIN */
#ifndef INA226_SS0_PIN
#define INA226_SS0_PIN                INA226_RTS_SS0_PIN
#endif /* INA226_SS0_PIN */

#if (INA226_MOSI_SCL_RX_WAKE_PIN)
    #define INA226_MOSI_SCL_RX_WAKE_HSIOM_REG     INA226_RX_WAKE_SCL_MOSI_HSIOM_REG
    #define INA226_MOSI_SCL_RX_WAKE_HSIOM_PTR     INA226_RX_WAKE_SCL_MOSI_HSIOM_REG
    #define INA226_MOSI_SCL_RX_WAKE_HSIOM_MASK    INA226_RX_WAKE_SCL_MOSI_HSIOM_REG
    #define INA226_MOSI_SCL_RX_WAKE_HSIOM_POS     INA226_RX_WAKE_SCL_MOSI_HSIOM_REG

    #define INA226_MOSI_SCL_RX_WAKE_INTCFG_REG    INA226_RX_WAKE_SCL_MOSI_HSIOM_REG
    #define INA226_MOSI_SCL_RX_WAKE_INTCFG_PTR    INA226_RX_WAKE_SCL_MOSI_HSIOM_REG

    #define INA226_MOSI_SCL_RX_WAKE_INTCFG_TYPE_POS   INA226_RX_WAKE_SCL_MOSI_HSIOM_REG
    #define INA226_MOSI_SCL_RX_WAKE_INTCFG_TYPE_MASK  INA226_RX_WAKE_SCL_MOSI_HSIOM_REG
#endif /* (INA226_RX_WAKE_SCL_MOSI_PIN) */

#if (INA226_MOSI_SCL_RX_PIN)
    #define INA226_MOSI_SCL_RX_HSIOM_REG      INA226_RX_SCL_MOSI_HSIOM_REG
    #define INA226_MOSI_SCL_RX_HSIOM_PTR      INA226_RX_SCL_MOSI_HSIOM_PTR
    #define INA226_MOSI_SCL_RX_HSIOM_MASK     INA226_RX_SCL_MOSI_HSIOM_MASK
    #define INA226_MOSI_SCL_RX_HSIOM_POS      INA226_RX_SCL_MOSI_HSIOM_POS
#endif /* (INA226_MOSI_SCL_RX_PIN) */

#if (INA226_MISO_SDA_TX_PIN)
    #define INA226_MISO_SDA_TX_HSIOM_REG      INA226_TX_SDA_MISO_HSIOM_REG
    #define INA226_MISO_SDA_TX_HSIOM_PTR      INA226_TX_SDA_MISO_HSIOM_REG
    #define INA226_MISO_SDA_TX_HSIOM_MASK     INA226_TX_SDA_MISO_HSIOM_REG
    #define INA226_MISO_SDA_TX_HSIOM_POS      INA226_TX_SDA_MISO_HSIOM_REG
#endif /* (INA226_MISO_SDA_TX_PIN_PIN) */

#if (INA226_SCLK_PIN)
    #ifndef INA226_SCLK_HSIOM_REG
    #define INA226_SCLK_HSIOM_REG     INA226_CTS_SCLK_HSIOM_REG
    #define INA226_SCLK_HSIOM_PTR     INA226_CTS_SCLK_HSIOM_PTR
    #define INA226_SCLK_HSIOM_MASK    INA226_CTS_SCLK_HSIOM_MASK
    #define INA226_SCLK_HSIOM_POS     INA226_CTS_SCLK_HSIOM_POS
    #endif /* INA226_SCLK_HSIOM_REG */
#endif /* (INA226_SCLK_PIN) */

#if (INA226_SS0_PIN)
    #ifndef INA226_SS0_HSIOM_REG
    #define INA226_SS0_HSIOM_REG      INA226_RTS_SS0_HSIOM_REG
    #define INA226_SS0_HSIOM_PTR      INA226_RTS_SS0_HSIOM_PTR
    #define INA226_SS0_HSIOM_MASK     INA226_RTS_SS0_HSIOM_MASK
    #define INA226_SS0_HSIOM_POS      INA226_RTS_SS0_HSIOM_POS
    #endif /* INA226_SS0_HSIOM_REG */
#endif /* (INA226_SS0_PIN) */

#define INA226_MOSI_SCL_RX_WAKE_PIN_INDEX INA226_RX_WAKE_SCL_MOSI_PIN_INDEX
#define INA226_MOSI_SCL_RX_PIN_INDEX      INA226_RX_SCL_MOSI_PIN_INDEX
#define INA226_MISO_SDA_TX_PIN_INDEX      INA226_TX_SDA_MISO_PIN_INDEX
#ifndef INA226_SCLK_PIN_INDEX
#define INA226_SCLK_PIN_INDEX             INA226_CTS_SCLK_PIN_INDEX
#endif /* INA226_SCLK_PIN_INDEX */
#ifndef INA226_SS0_PIN_INDEX
#define INA226_SS0_PIN_INDEX              INA226_RTS_SS0_PIN_INDEX
#endif /* INA226_SS0_PIN_INDEX */

#define INA226_MOSI_SCL_RX_WAKE_PIN_MASK INA226_RX_WAKE_SCL_MOSI_PIN_MASK
#define INA226_MOSI_SCL_RX_PIN_MASK      INA226_RX_SCL_MOSI_PIN_MASK
#define INA226_MISO_SDA_TX_PIN_MASK      INA226_TX_SDA_MISO_PIN_MASK
#ifndef INA226_SCLK_PIN_MASK
#define INA226_SCLK_PIN_MASK             INA226_CTS_SCLK_PIN_MASK
#endif /* INA226_SCLK_PIN_MASK */
#ifndef INA226_SS0_PIN_MASK
#define INA226_SS0_PIN_MASK              INA226_RTS_SS0_PIN_MASK
#endif /* INA226_SS0_PIN_MASK */

#endif /* (CY_SCB_PINS_INA226_H) */


/* [] END OF FILE */
