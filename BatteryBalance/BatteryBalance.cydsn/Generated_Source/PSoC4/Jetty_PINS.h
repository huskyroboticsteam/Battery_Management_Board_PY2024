/***************************************************************************//**
* \file Jetty_PINS.h
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

#if !defined(CY_SCB_PINS_Jetty_H)
#define CY_SCB_PINS_Jetty_H

#include "cydevice_trm.h"
#include "cyfitter.h"
#include "cytypes.h"


/***************************************
*   Conditional Compilation Parameters
****************************************/

/* Unconfigured pins */
#define Jetty_REMOVE_RX_WAKE_SCL_MOSI_PIN  (1u)
#define Jetty_REMOVE_RX_SCL_MOSI_PIN      (1u)
#define Jetty_REMOVE_TX_SDA_MISO_PIN      (1u)
#define Jetty_REMOVE_CTS_SCLK_PIN      (1u)
#define Jetty_REMOVE_RTS_SS0_PIN      (1u)
#define Jetty_REMOVE_SS1_PIN                 (1u)
#define Jetty_REMOVE_SS2_PIN                 (1u)
#define Jetty_REMOVE_SS3_PIN                 (1u)

/* Mode defined pins */
#define Jetty_REMOVE_I2C_PINS                (0u)
#define Jetty_REMOVE_SPI_MASTER_PINS         (1u)
#define Jetty_REMOVE_SPI_MASTER_SCLK_PIN     (1u)
#define Jetty_REMOVE_SPI_MASTER_MOSI_PIN     (1u)
#define Jetty_REMOVE_SPI_MASTER_MISO_PIN     (1u)
#define Jetty_REMOVE_SPI_MASTER_SS0_PIN      (1u)
#define Jetty_REMOVE_SPI_MASTER_SS1_PIN      (1u)
#define Jetty_REMOVE_SPI_MASTER_SS2_PIN      (1u)
#define Jetty_REMOVE_SPI_MASTER_SS3_PIN      (1u)
#define Jetty_REMOVE_SPI_SLAVE_PINS          (1u)
#define Jetty_REMOVE_SPI_SLAVE_MOSI_PIN      (1u)
#define Jetty_REMOVE_SPI_SLAVE_MISO_PIN      (1u)
#define Jetty_REMOVE_UART_TX_PIN             (1u)
#define Jetty_REMOVE_UART_RX_TX_PIN          (1u)
#define Jetty_REMOVE_UART_RX_PIN             (1u)
#define Jetty_REMOVE_UART_RX_WAKE_PIN        (1u)
#define Jetty_REMOVE_UART_RTS_PIN            (1u)
#define Jetty_REMOVE_UART_CTS_PIN            (1u)

/* Unconfigured pins */
#define Jetty_RX_WAKE_SCL_MOSI_PIN (0u == Jetty_REMOVE_RX_WAKE_SCL_MOSI_PIN)
#define Jetty_RX_SCL_MOSI_PIN     (0u == Jetty_REMOVE_RX_SCL_MOSI_PIN)
#define Jetty_TX_SDA_MISO_PIN     (0u == Jetty_REMOVE_TX_SDA_MISO_PIN)
#define Jetty_CTS_SCLK_PIN     (0u == Jetty_REMOVE_CTS_SCLK_PIN)
#define Jetty_RTS_SS0_PIN     (0u == Jetty_REMOVE_RTS_SS0_PIN)
#define Jetty_SS1_PIN                (0u == Jetty_REMOVE_SS1_PIN)
#define Jetty_SS2_PIN                (0u == Jetty_REMOVE_SS2_PIN)
#define Jetty_SS3_PIN                (0u == Jetty_REMOVE_SS3_PIN)

/* Mode defined pins */
#define Jetty_I2C_PINS               (0u == Jetty_REMOVE_I2C_PINS)
#define Jetty_SPI_MASTER_PINS        (0u == Jetty_REMOVE_SPI_MASTER_PINS)
#define Jetty_SPI_MASTER_SCLK_PIN    (0u == Jetty_REMOVE_SPI_MASTER_SCLK_PIN)
#define Jetty_SPI_MASTER_MOSI_PIN    (0u == Jetty_REMOVE_SPI_MASTER_MOSI_PIN)
#define Jetty_SPI_MASTER_MISO_PIN    (0u == Jetty_REMOVE_SPI_MASTER_MISO_PIN)
#define Jetty_SPI_MASTER_SS0_PIN     (0u == Jetty_REMOVE_SPI_MASTER_SS0_PIN)
#define Jetty_SPI_MASTER_SS1_PIN     (0u == Jetty_REMOVE_SPI_MASTER_SS1_PIN)
#define Jetty_SPI_MASTER_SS2_PIN     (0u == Jetty_REMOVE_SPI_MASTER_SS2_PIN)
#define Jetty_SPI_MASTER_SS3_PIN     (0u == Jetty_REMOVE_SPI_MASTER_SS3_PIN)
#define Jetty_SPI_SLAVE_PINS         (0u == Jetty_REMOVE_SPI_SLAVE_PINS)
#define Jetty_SPI_SLAVE_MOSI_PIN     (0u == Jetty_REMOVE_SPI_SLAVE_MOSI_PIN)
#define Jetty_SPI_SLAVE_MISO_PIN     (0u == Jetty_REMOVE_SPI_SLAVE_MISO_PIN)
#define Jetty_UART_TX_PIN            (0u == Jetty_REMOVE_UART_TX_PIN)
#define Jetty_UART_RX_TX_PIN         (0u == Jetty_REMOVE_UART_RX_TX_PIN)
#define Jetty_UART_RX_PIN            (0u == Jetty_REMOVE_UART_RX_PIN)
#define Jetty_UART_RX_WAKE_PIN       (0u == Jetty_REMOVE_UART_RX_WAKE_PIN)
#define Jetty_UART_RTS_PIN           (0u == Jetty_REMOVE_UART_RTS_PIN)
#define Jetty_UART_CTS_PIN           (0u == Jetty_REMOVE_UART_CTS_PIN)


/***************************************
*             Includes
****************************************/

#if (Jetty_RX_WAKE_SCL_MOSI_PIN)
    #include "Jetty_uart_rx_wake_i2c_scl_spi_mosi.h"
#endif /* (Jetty_RX_SCL_MOSI) */

#if (Jetty_RX_SCL_MOSI_PIN)
    #include "Jetty_uart_rx_i2c_scl_spi_mosi.h"
#endif /* (Jetty_RX_SCL_MOSI) */

#if (Jetty_TX_SDA_MISO_PIN)
    #include "Jetty_uart_tx_i2c_sda_spi_miso.h"
#endif /* (Jetty_TX_SDA_MISO) */

#if (Jetty_CTS_SCLK_PIN)
    #include "Jetty_uart_cts_spi_sclk.h"
#endif /* (Jetty_CTS_SCLK) */

#if (Jetty_RTS_SS0_PIN)
    #include "Jetty_uart_rts_spi_ss0.h"
#endif /* (Jetty_RTS_SS0_PIN) */

#if (Jetty_SS1_PIN)
    #include "Jetty_spi_ss1.h"
#endif /* (Jetty_SS1_PIN) */

#if (Jetty_SS2_PIN)
    #include "Jetty_spi_ss2.h"
#endif /* (Jetty_SS2_PIN) */

#if (Jetty_SS3_PIN)
    #include "Jetty_spi_ss3.h"
#endif /* (Jetty_SS3_PIN) */

#if (Jetty_I2C_PINS)
    #include "Jetty_scl.h"
    #include "Jetty_sda.h"
#endif /* (Jetty_I2C_PINS) */

#if (Jetty_SPI_MASTER_PINS)
#if (Jetty_SPI_MASTER_SCLK_PIN)
    #include "Jetty_sclk_m.h"
#endif /* (Jetty_SPI_MASTER_SCLK_PIN) */

#if (Jetty_SPI_MASTER_MOSI_PIN)
    #include "Jetty_mosi_m.h"
#endif /* (Jetty_SPI_MASTER_MOSI_PIN) */

#if (Jetty_SPI_MASTER_MISO_PIN)
    #include "Jetty_miso_m.h"
#endif /*(Jetty_SPI_MASTER_MISO_PIN) */
#endif /* (Jetty_SPI_MASTER_PINS) */

#if (Jetty_SPI_SLAVE_PINS)
    #include "Jetty_sclk_s.h"
    #include "Jetty_ss_s.h"

#if (Jetty_SPI_SLAVE_MOSI_PIN)
    #include "Jetty_mosi_s.h"
#endif /* (Jetty_SPI_SLAVE_MOSI_PIN) */

#if (Jetty_SPI_SLAVE_MISO_PIN)
    #include "Jetty_miso_s.h"
#endif /*(Jetty_SPI_SLAVE_MISO_PIN) */
#endif /* (Jetty_SPI_SLAVE_PINS) */

#if (Jetty_SPI_MASTER_SS0_PIN)
    #include "Jetty_ss0_m.h"
#endif /* (Jetty_SPI_MASTER_SS0_PIN) */

#if (Jetty_SPI_MASTER_SS1_PIN)
    #include "Jetty_ss1_m.h"
#endif /* (Jetty_SPI_MASTER_SS1_PIN) */

#if (Jetty_SPI_MASTER_SS2_PIN)
    #include "Jetty_ss2_m.h"
#endif /* (Jetty_SPI_MASTER_SS2_PIN) */

#if (Jetty_SPI_MASTER_SS3_PIN)
    #include "Jetty_ss3_m.h"
#endif /* (Jetty_SPI_MASTER_SS3_PIN) */

#if (Jetty_UART_TX_PIN)
    #include "Jetty_tx.h"
#endif /* (Jetty_UART_TX_PIN) */

#if (Jetty_UART_RX_TX_PIN)
    #include "Jetty_rx_tx.h"
#endif /* (Jetty_UART_RX_TX_PIN) */

#if (Jetty_UART_RX_PIN)
    #include "Jetty_rx.h"
#endif /* (Jetty_UART_RX_PIN) */

#if (Jetty_UART_RX_WAKE_PIN)
    #include "Jetty_rx_wake.h"
#endif /* (Jetty_UART_RX_WAKE_PIN) */

#if (Jetty_UART_RTS_PIN)
    #include "Jetty_rts.h"
#endif /* (Jetty_UART_RTS_PIN) */

#if (Jetty_UART_CTS_PIN)
    #include "Jetty_cts.h"
#endif /* (Jetty_UART_CTS_PIN) */


/***************************************
*              Registers
***************************************/

#if (Jetty_RX_SCL_MOSI_PIN)
    #define Jetty_RX_SCL_MOSI_HSIOM_REG   (*(reg32 *) Jetty_uart_rx_i2c_scl_spi_mosi__0__HSIOM)
    #define Jetty_RX_SCL_MOSI_HSIOM_PTR   ( (reg32 *) Jetty_uart_rx_i2c_scl_spi_mosi__0__HSIOM)
    
    #define Jetty_RX_SCL_MOSI_HSIOM_MASK      (Jetty_uart_rx_i2c_scl_spi_mosi__0__HSIOM_MASK)
    #define Jetty_RX_SCL_MOSI_HSIOM_POS       (Jetty_uart_rx_i2c_scl_spi_mosi__0__HSIOM_SHIFT)
    #define Jetty_RX_SCL_MOSI_HSIOM_SEL_GPIO  (Jetty_uart_rx_i2c_scl_spi_mosi__0__HSIOM_GPIO)
    #define Jetty_RX_SCL_MOSI_HSIOM_SEL_I2C   (Jetty_uart_rx_i2c_scl_spi_mosi__0__HSIOM_I2C)
    #define Jetty_RX_SCL_MOSI_HSIOM_SEL_SPI   (Jetty_uart_rx_i2c_scl_spi_mosi__0__HSIOM_SPI)
    #define Jetty_RX_SCL_MOSI_HSIOM_SEL_UART  (Jetty_uart_rx_i2c_scl_spi_mosi__0__HSIOM_UART)
    
#elif (Jetty_RX_WAKE_SCL_MOSI_PIN)
    #define Jetty_RX_WAKE_SCL_MOSI_HSIOM_REG   (*(reg32 *) Jetty_uart_rx_wake_i2c_scl_spi_mosi__0__HSIOM)
    #define Jetty_RX_WAKE_SCL_MOSI_HSIOM_PTR   ( (reg32 *) Jetty_uart_rx_wake_i2c_scl_spi_mosi__0__HSIOM)
    
    #define Jetty_RX_WAKE_SCL_MOSI_HSIOM_MASK      (Jetty_uart_rx_wake_i2c_scl_spi_mosi__0__HSIOM_MASK)
    #define Jetty_RX_WAKE_SCL_MOSI_HSIOM_POS       (Jetty_uart_rx_wake_i2c_scl_spi_mosi__0__HSIOM_SHIFT)
    #define Jetty_RX_WAKE_SCL_MOSI_HSIOM_SEL_GPIO  (Jetty_uart_rx_wake_i2c_scl_spi_mosi__0__HSIOM_GPIO)
    #define Jetty_RX_WAKE_SCL_MOSI_HSIOM_SEL_I2C   (Jetty_uart_rx_wake_i2c_scl_spi_mosi__0__HSIOM_I2C)
    #define Jetty_RX_WAKE_SCL_MOSI_HSIOM_SEL_SPI   (Jetty_uart_rx_wake_i2c_scl_spi_mosi__0__HSIOM_SPI)
    #define Jetty_RX_WAKE_SCL_MOSI_HSIOM_SEL_UART  (Jetty_uart_rx_wake_i2c_scl_spi_mosi__0__HSIOM_UART)    
   
    #define Jetty_RX_WAKE_SCL_MOSI_INTCFG_REG (*(reg32 *) Jetty_uart_rx_wake_i2c_scl_spi_mosi__0__INTCFG)
    #define Jetty_RX_WAKE_SCL_MOSI_INTCFG_PTR ( (reg32 *) Jetty_uart_rx_wake_i2c_scl_spi_mosi__0__INTCFG)
    #define Jetty_RX_WAKE_SCL_MOSI_INTCFG_TYPE_POS  (Jetty_uart_rx_wake_i2c_scl_spi_mosi__SHIFT)
    #define Jetty_RX_WAKE_SCL_MOSI_INTCFG_TYPE_MASK ((uint32) Jetty_INTCFG_TYPE_MASK << \
                                                                           Jetty_RX_WAKE_SCL_MOSI_INTCFG_TYPE_POS)
#else
    /* None of pins Jetty_RX_SCL_MOSI_PIN or Jetty_RX_WAKE_SCL_MOSI_PIN present.*/
#endif /* (Jetty_RX_SCL_MOSI_PIN) */

#if (Jetty_TX_SDA_MISO_PIN)
    #define Jetty_TX_SDA_MISO_HSIOM_REG   (*(reg32 *) Jetty_uart_tx_i2c_sda_spi_miso__0__HSIOM)
    #define Jetty_TX_SDA_MISO_HSIOM_PTR   ( (reg32 *) Jetty_uart_tx_i2c_sda_spi_miso__0__HSIOM)
    
    #define Jetty_TX_SDA_MISO_HSIOM_MASK      (Jetty_uart_tx_i2c_sda_spi_miso__0__HSIOM_MASK)
    #define Jetty_TX_SDA_MISO_HSIOM_POS       (Jetty_uart_tx_i2c_sda_spi_miso__0__HSIOM_SHIFT)
    #define Jetty_TX_SDA_MISO_HSIOM_SEL_GPIO  (Jetty_uart_tx_i2c_sda_spi_miso__0__HSIOM_GPIO)
    #define Jetty_TX_SDA_MISO_HSIOM_SEL_I2C   (Jetty_uart_tx_i2c_sda_spi_miso__0__HSIOM_I2C)
    #define Jetty_TX_SDA_MISO_HSIOM_SEL_SPI   (Jetty_uart_tx_i2c_sda_spi_miso__0__HSIOM_SPI)
    #define Jetty_TX_SDA_MISO_HSIOM_SEL_UART  (Jetty_uart_tx_i2c_sda_spi_miso__0__HSIOM_UART)
#endif /* (Jetty_TX_SDA_MISO_PIN) */

#if (Jetty_CTS_SCLK_PIN)
    #define Jetty_CTS_SCLK_HSIOM_REG   (*(reg32 *) Jetty_uart_cts_spi_sclk__0__HSIOM)
    #define Jetty_CTS_SCLK_HSIOM_PTR   ( (reg32 *) Jetty_uart_cts_spi_sclk__0__HSIOM)
    
    #define Jetty_CTS_SCLK_HSIOM_MASK      (Jetty_uart_cts_spi_sclk__0__HSIOM_MASK)
    #define Jetty_CTS_SCLK_HSIOM_POS       (Jetty_uart_cts_spi_sclk__0__HSIOM_SHIFT)
    #define Jetty_CTS_SCLK_HSIOM_SEL_GPIO  (Jetty_uart_cts_spi_sclk__0__HSIOM_GPIO)
    #define Jetty_CTS_SCLK_HSIOM_SEL_I2C   (Jetty_uart_cts_spi_sclk__0__HSIOM_I2C)
    #define Jetty_CTS_SCLK_HSIOM_SEL_SPI   (Jetty_uart_cts_spi_sclk__0__HSIOM_SPI)
    #define Jetty_CTS_SCLK_HSIOM_SEL_UART  (Jetty_uart_cts_spi_sclk__0__HSIOM_UART)
#endif /* (Jetty_CTS_SCLK_PIN) */

#if (Jetty_RTS_SS0_PIN)
    #define Jetty_RTS_SS0_HSIOM_REG   (*(reg32 *) Jetty_uart_rts_spi_ss0__0__HSIOM)
    #define Jetty_RTS_SS0_HSIOM_PTR   ( (reg32 *) Jetty_uart_rts_spi_ss0__0__HSIOM)
    
    #define Jetty_RTS_SS0_HSIOM_MASK      (Jetty_uart_rts_spi_ss0__0__HSIOM_MASK)
    #define Jetty_RTS_SS0_HSIOM_POS       (Jetty_uart_rts_spi_ss0__0__HSIOM_SHIFT)
    #define Jetty_RTS_SS0_HSIOM_SEL_GPIO  (Jetty_uart_rts_spi_ss0__0__HSIOM_GPIO)
    #define Jetty_RTS_SS0_HSIOM_SEL_I2C   (Jetty_uart_rts_spi_ss0__0__HSIOM_I2C)
    #define Jetty_RTS_SS0_HSIOM_SEL_SPI   (Jetty_uart_rts_spi_ss0__0__HSIOM_SPI)
#if !(Jetty_CY_SCBIP_V0 || Jetty_CY_SCBIP_V1)
    #define Jetty_RTS_SS0_HSIOM_SEL_UART  (Jetty_uart_rts_spi_ss0__0__HSIOM_UART)
#endif /* !(Jetty_CY_SCBIP_V0 || Jetty_CY_SCBIP_V1) */
#endif /* (Jetty_RTS_SS0_PIN) */

#if (Jetty_SS1_PIN)
    #define Jetty_SS1_HSIOM_REG  (*(reg32 *) Jetty_spi_ss1__0__HSIOM)
    #define Jetty_SS1_HSIOM_PTR  ( (reg32 *) Jetty_spi_ss1__0__HSIOM)
    
    #define Jetty_SS1_HSIOM_MASK     (Jetty_spi_ss1__0__HSIOM_MASK)
    #define Jetty_SS1_HSIOM_POS      (Jetty_spi_ss1__0__HSIOM_SHIFT)
    #define Jetty_SS1_HSIOM_SEL_GPIO (Jetty_spi_ss1__0__HSIOM_GPIO)
    #define Jetty_SS1_HSIOM_SEL_I2C  (Jetty_spi_ss1__0__HSIOM_I2C)
    #define Jetty_SS1_HSIOM_SEL_SPI  (Jetty_spi_ss1__0__HSIOM_SPI)
#endif /* (Jetty_SS1_PIN) */

#if (Jetty_SS2_PIN)
    #define Jetty_SS2_HSIOM_REG     (*(reg32 *) Jetty_spi_ss2__0__HSIOM)
    #define Jetty_SS2_HSIOM_PTR     ( (reg32 *) Jetty_spi_ss2__0__HSIOM)
    
    #define Jetty_SS2_HSIOM_MASK     (Jetty_spi_ss2__0__HSIOM_MASK)
    #define Jetty_SS2_HSIOM_POS      (Jetty_spi_ss2__0__HSIOM_SHIFT)
    #define Jetty_SS2_HSIOM_SEL_GPIO (Jetty_spi_ss2__0__HSIOM_GPIO)
    #define Jetty_SS2_HSIOM_SEL_I2C  (Jetty_spi_ss2__0__HSIOM_I2C)
    #define Jetty_SS2_HSIOM_SEL_SPI  (Jetty_spi_ss2__0__HSIOM_SPI)
#endif /* (Jetty_SS2_PIN) */

#if (Jetty_SS3_PIN)
    #define Jetty_SS3_HSIOM_REG     (*(reg32 *) Jetty_spi_ss3__0__HSIOM)
    #define Jetty_SS3_HSIOM_PTR     ( (reg32 *) Jetty_spi_ss3__0__HSIOM)
    
    #define Jetty_SS3_HSIOM_MASK     (Jetty_spi_ss3__0__HSIOM_MASK)
    #define Jetty_SS3_HSIOM_POS      (Jetty_spi_ss3__0__HSIOM_SHIFT)
    #define Jetty_SS3_HSIOM_SEL_GPIO (Jetty_spi_ss3__0__HSIOM_GPIO)
    #define Jetty_SS3_HSIOM_SEL_I2C  (Jetty_spi_ss3__0__HSIOM_I2C)
    #define Jetty_SS3_HSIOM_SEL_SPI  (Jetty_spi_ss3__0__HSIOM_SPI)
#endif /* (Jetty_SS3_PIN) */

#if (Jetty_I2C_PINS)
    #define Jetty_SCL_HSIOM_REG  (*(reg32 *) Jetty_scl__0__HSIOM)
    #define Jetty_SCL_HSIOM_PTR  ( (reg32 *) Jetty_scl__0__HSIOM)
    
    #define Jetty_SCL_HSIOM_MASK     (Jetty_scl__0__HSIOM_MASK)
    #define Jetty_SCL_HSIOM_POS      (Jetty_scl__0__HSIOM_SHIFT)
    #define Jetty_SCL_HSIOM_SEL_GPIO (Jetty_sda__0__HSIOM_GPIO)
    #define Jetty_SCL_HSIOM_SEL_I2C  (Jetty_sda__0__HSIOM_I2C)
    
    #define Jetty_SDA_HSIOM_REG  (*(reg32 *) Jetty_sda__0__HSIOM)
    #define Jetty_SDA_HSIOM_PTR  ( (reg32 *) Jetty_sda__0__HSIOM)
    
    #define Jetty_SDA_HSIOM_MASK     (Jetty_sda__0__HSIOM_MASK)
    #define Jetty_SDA_HSIOM_POS      (Jetty_sda__0__HSIOM_SHIFT)
    #define Jetty_SDA_HSIOM_SEL_GPIO (Jetty_sda__0__HSIOM_GPIO)
    #define Jetty_SDA_HSIOM_SEL_I2C  (Jetty_sda__0__HSIOM_I2C)
#endif /* (Jetty_I2C_PINS) */

#if (Jetty_SPI_SLAVE_PINS)
    #define Jetty_SCLK_S_HSIOM_REG   (*(reg32 *) Jetty_sclk_s__0__HSIOM)
    #define Jetty_SCLK_S_HSIOM_PTR   ( (reg32 *) Jetty_sclk_s__0__HSIOM)
    
    #define Jetty_SCLK_S_HSIOM_MASK      (Jetty_sclk_s__0__HSIOM_MASK)
    #define Jetty_SCLK_S_HSIOM_POS       (Jetty_sclk_s__0__HSIOM_SHIFT)
    #define Jetty_SCLK_S_HSIOM_SEL_GPIO  (Jetty_sclk_s__0__HSIOM_GPIO)
    #define Jetty_SCLK_S_HSIOM_SEL_SPI   (Jetty_sclk_s__0__HSIOM_SPI)
    
    #define Jetty_SS0_S_HSIOM_REG    (*(reg32 *) Jetty_ss0_s__0__HSIOM)
    #define Jetty_SS0_S_HSIOM_PTR    ( (reg32 *) Jetty_ss0_s__0__HSIOM)
    
    #define Jetty_SS0_S_HSIOM_MASK       (Jetty_ss0_s__0__HSIOM_MASK)
    #define Jetty_SS0_S_HSIOM_POS        (Jetty_ss0_s__0__HSIOM_SHIFT)
    #define Jetty_SS0_S_HSIOM_SEL_GPIO   (Jetty_ss0_s__0__HSIOM_GPIO)  
    #define Jetty_SS0_S_HSIOM_SEL_SPI    (Jetty_ss0_s__0__HSIOM_SPI)
#endif /* (Jetty_SPI_SLAVE_PINS) */

#if (Jetty_SPI_SLAVE_MOSI_PIN)
    #define Jetty_MOSI_S_HSIOM_REG   (*(reg32 *) Jetty_mosi_s__0__HSIOM)
    #define Jetty_MOSI_S_HSIOM_PTR   ( (reg32 *) Jetty_mosi_s__0__HSIOM)
    
    #define Jetty_MOSI_S_HSIOM_MASK      (Jetty_mosi_s__0__HSIOM_MASK)
    #define Jetty_MOSI_S_HSIOM_POS       (Jetty_mosi_s__0__HSIOM_SHIFT)
    #define Jetty_MOSI_S_HSIOM_SEL_GPIO  (Jetty_mosi_s__0__HSIOM_GPIO)
    #define Jetty_MOSI_S_HSIOM_SEL_SPI   (Jetty_mosi_s__0__HSIOM_SPI)
#endif /* (Jetty_SPI_SLAVE_MOSI_PIN) */

#if (Jetty_SPI_SLAVE_MISO_PIN)
    #define Jetty_MISO_S_HSIOM_REG   (*(reg32 *) Jetty_miso_s__0__HSIOM)
    #define Jetty_MISO_S_HSIOM_PTR   ( (reg32 *) Jetty_miso_s__0__HSIOM)
    
    #define Jetty_MISO_S_HSIOM_MASK      (Jetty_miso_s__0__HSIOM_MASK)
    #define Jetty_MISO_S_HSIOM_POS       (Jetty_miso_s__0__HSIOM_SHIFT)
    #define Jetty_MISO_S_HSIOM_SEL_GPIO  (Jetty_miso_s__0__HSIOM_GPIO)
    #define Jetty_MISO_S_HSIOM_SEL_SPI   (Jetty_miso_s__0__HSIOM_SPI)
#endif /* (Jetty_SPI_SLAVE_MISO_PIN) */

#if (Jetty_SPI_MASTER_MISO_PIN)
    #define Jetty_MISO_M_HSIOM_REG   (*(reg32 *) Jetty_miso_m__0__HSIOM)
    #define Jetty_MISO_M_HSIOM_PTR   ( (reg32 *) Jetty_miso_m__0__HSIOM)
    
    #define Jetty_MISO_M_HSIOM_MASK      (Jetty_miso_m__0__HSIOM_MASK)
    #define Jetty_MISO_M_HSIOM_POS       (Jetty_miso_m__0__HSIOM_SHIFT)
    #define Jetty_MISO_M_HSIOM_SEL_GPIO  (Jetty_miso_m__0__HSIOM_GPIO)
    #define Jetty_MISO_M_HSIOM_SEL_SPI   (Jetty_miso_m__0__HSIOM_SPI)
#endif /* (Jetty_SPI_MASTER_MISO_PIN) */

#if (Jetty_SPI_MASTER_MOSI_PIN)
    #define Jetty_MOSI_M_HSIOM_REG   (*(reg32 *) Jetty_mosi_m__0__HSIOM)
    #define Jetty_MOSI_M_HSIOM_PTR   ( (reg32 *) Jetty_mosi_m__0__HSIOM)
    
    #define Jetty_MOSI_M_HSIOM_MASK      (Jetty_mosi_m__0__HSIOM_MASK)
    #define Jetty_MOSI_M_HSIOM_POS       (Jetty_mosi_m__0__HSIOM_SHIFT)
    #define Jetty_MOSI_M_HSIOM_SEL_GPIO  (Jetty_mosi_m__0__HSIOM_GPIO)
    #define Jetty_MOSI_M_HSIOM_SEL_SPI   (Jetty_mosi_m__0__HSIOM_SPI)
#endif /* (Jetty_SPI_MASTER_MOSI_PIN) */

#if (Jetty_SPI_MASTER_SCLK_PIN)
    #define Jetty_SCLK_M_HSIOM_REG   (*(reg32 *) Jetty_sclk_m__0__HSIOM)
    #define Jetty_SCLK_M_HSIOM_PTR   ( (reg32 *) Jetty_sclk_m__0__HSIOM)
    
    #define Jetty_SCLK_M_HSIOM_MASK      (Jetty_sclk_m__0__HSIOM_MASK)
    #define Jetty_SCLK_M_HSIOM_POS       (Jetty_sclk_m__0__HSIOM_SHIFT)
    #define Jetty_SCLK_M_HSIOM_SEL_GPIO  (Jetty_sclk_m__0__HSIOM_GPIO)
    #define Jetty_SCLK_M_HSIOM_SEL_SPI   (Jetty_sclk_m__0__HSIOM_SPI)
#endif /* (Jetty_SPI_MASTER_SCLK_PIN) */

#if (Jetty_SPI_MASTER_SS0_PIN)
    #define Jetty_SS0_M_HSIOM_REG    (*(reg32 *) Jetty_ss0_m__0__HSIOM)
    #define Jetty_SS0_M_HSIOM_PTR    ( (reg32 *) Jetty_ss0_m__0__HSIOM)
    
    #define Jetty_SS0_M_HSIOM_MASK       (Jetty_ss0_m__0__HSIOM_MASK)
    #define Jetty_SS0_M_HSIOM_POS        (Jetty_ss0_m__0__HSIOM_SHIFT)
    #define Jetty_SS0_M_HSIOM_SEL_GPIO   (Jetty_ss0_m__0__HSIOM_GPIO)
    #define Jetty_SS0_M_HSIOM_SEL_SPI    (Jetty_ss0_m__0__HSIOM_SPI)
#endif /* (Jetty_SPI_MASTER_SS0_PIN) */

#if (Jetty_SPI_MASTER_SS1_PIN)
    #define Jetty_SS1_M_HSIOM_REG    (*(reg32 *) Jetty_ss1_m__0__HSIOM)
    #define Jetty_SS1_M_HSIOM_PTR    ( (reg32 *) Jetty_ss1_m__0__HSIOM)
    
    #define Jetty_SS1_M_HSIOM_MASK       (Jetty_ss1_m__0__HSIOM_MASK)
    #define Jetty_SS1_M_HSIOM_POS        (Jetty_ss1_m__0__HSIOM_SHIFT)
    #define Jetty_SS1_M_HSIOM_SEL_GPIO   (Jetty_ss1_m__0__HSIOM_GPIO)
    #define Jetty_SS1_M_HSIOM_SEL_SPI    (Jetty_ss1_m__0__HSIOM_SPI)
#endif /* (Jetty_SPI_MASTER_SS1_PIN) */

#if (Jetty_SPI_MASTER_SS2_PIN)
    #define Jetty_SS2_M_HSIOM_REG    (*(reg32 *) Jetty_ss2_m__0__HSIOM)
    #define Jetty_SS2_M_HSIOM_PTR    ( (reg32 *) Jetty_ss2_m__0__HSIOM)
    
    #define Jetty_SS2_M_HSIOM_MASK       (Jetty_ss2_m__0__HSIOM_MASK)
    #define Jetty_SS2_M_HSIOM_POS        (Jetty_ss2_m__0__HSIOM_SHIFT)
    #define Jetty_SS2_M_HSIOM_SEL_GPIO   (Jetty_ss2_m__0__HSIOM_GPIO)
    #define Jetty_SS2_M_HSIOM_SEL_SPI    (Jetty_ss2_m__0__HSIOM_SPI)
#endif /* (Jetty_SPI_MASTER_SS2_PIN) */

#if (Jetty_SPI_MASTER_SS3_PIN)
    #define Jetty_SS3_M_HSIOM_REG    (*(reg32 *) Jetty_ss3_m__0__HSIOM)
    #define Jetty_SS3_M_HSIOM_PTR    ( (reg32 *) Jetty_ss3_m__0__HSIOM)
    
    #define Jetty_SS3_M_HSIOM_MASK      (Jetty_ss3_m__0__HSIOM_MASK)
    #define Jetty_SS3_M_HSIOM_POS       (Jetty_ss3_m__0__HSIOM_SHIFT)
    #define Jetty_SS3_M_HSIOM_SEL_GPIO  (Jetty_ss3_m__0__HSIOM_GPIO)
    #define Jetty_SS3_M_HSIOM_SEL_SPI   (Jetty_ss3_m__0__HSIOM_SPI)
#endif /* (Jetty_SPI_MASTER_SS3_PIN) */

#if (Jetty_UART_RX_PIN)
    #define Jetty_RX_HSIOM_REG   (*(reg32 *) Jetty_rx__0__HSIOM)
    #define Jetty_RX_HSIOM_PTR   ( (reg32 *) Jetty_rx__0__HSIOM)
    
    #define Jetty_RX_HSIOM_MASK      (Jetty_rx__0__HSIOM_MASK)
    #define Jetty_RX_HSIOM_POS       (Jetty_rx__0__HSIOM_SHIFT)
    #define Jetty_RX_HSIOM_SEL_GPIO  (Jetty_rx__0__HSIOM_GPIO)
    #define Jetty_RX_HSIOM_SEL_UART  (Jetty_rx__0__HSIOM_UART)
#endif /* (Jetty_UART_RX_PIN) */

#if (Jetty_UART_RX_WAKE_PIN)
    #define Jetty_RX_WAKE_HSIOM_REG   (*(reg32 *) Jetty_rx_wake__0__HSIOM)
    #define Jetty_RX_WAKE_HSIOM_PTR   ( (reg32 *) Jetty_rx_wake__0__HSIOM)
    
    #define Jetty_RX_WAKE_HSIOM_MASK      (Jetty_rx_wake__0__HSIOM_MASK)
    #define Jetty_RX_WAKE_HSIOM_POS       (Jetty_rx_wake__0__HSIOM_SHIFT)
    #define Jetty_RX_WAKE_HSIOM_SEL_GPIO  (Jetty_rx_wake__0__HSIOM_GPIO)
    #define Jetty_RX_WAKE_HSIOM_SEL_UART  (Jetty_rx_wake__0__HSIOM_UART)
#endif /* (Jetty_UART_WAKE_RX_PIN) */

#if (Jetty_UART_CTS_PIN)
    #define Jetty_CTS_HSIOM_REG   (*(reg32 *) Jetty_cts__0__HSIOM)
    #define Jetty_CTS_HSIOM_PTR   ( (reg32 *) Jetty_cts__0__HSIOM)
    
    #define Jetty_CTS_HSIOM_MASK      (Jetty_cts__0__HSIOM_MASK)
    #define Jetty_CTS_HSIOM_POS       (Jetty_cts__0__HSIOM_SHIFT)
    #define Jetty_CTS_HSIOM_SEL_GPIO  (Jetty_cts__0__HSIOM_GPIO)
    #define Jetty_CTS_HSIOM_SEL_UART  (Jetty_cts__0__HSIOM_UART)
#endif /* (Jetty_UART_CTS_PIN) */

#if (Jetty_UART_TX_PIN)
    #define Jetty_TX_HSIOM_REG   (*(reg32 *) Jetty_tx__0__HSIOM)
    #define Jetty_TX_HSIOM_PTR   ( (reg32 *) Jetty_tx__0__HSIOM)
    
    #define Jetty_TX_HSIOM_MASK      (Jetty_tx__0__HSIOM_MASK)
    #define Jetty_TX_HSIOM_POS       (Jetty_tx__0__HSIOM_SHIFT)
    #define Jetty_TX_HSIOM_SEL_GPIO  (Jetty_tx__0__HSIOM_GPIO)
    #define Jetty_TX_HSIOM_SEL_UART  (Jetty_tx__0__HSIOM_UART)
#endif /* (Jetty_UART_TX_PIN) */

#if (Jetty_UART_RX_TX_PIN)
    #define Jetty_RX_TX_HSIOM_REG   (*(reg32 *) Jetty_rx_tx__0__HSIOM)
    #define Jetty_RX_TX_HSIOM_PTR   ( (reg32 *) Jetty_rx_tx__0__HSIOM)
    
    #define Jetty_RX_TX_HSIOM_MASK      (Jetty_rx_tx__0__HSIOM_MASK)
    #define Jetty_RX_TX_HSIOM_POS       (Jetty_rx_tx__0__HSIOM_SHIFT)
    #define Jetty_RX_TX_HSIOM_SEL_GPIO  (Jetty_rx_tx__0__HSIOM_GPIO)
    #define Jetty_RX_TX_HSIOM_SEL_UART  (Jetty_rx_tx__0__HSIOM_UART)
#endif /* (Jetty_UART_RX_TX_PIN) */

#if (Jetty_UART_RTS_PIN)
    #define Jetty_RTS_HSIOM_REG      (*(reg32 *) Jetty_rts__0__HSIOM)
    #define Jetty_RTS_HSIOM_PTR      ( (reg32 *) Jetty_rts__0__HSIOM)
    
    #define Jetty_RTS_HSIOM_MASK     (Jetty_rts__0__HSIOM_MASK)
    #define Jetty_RTS_HSIOM_POS      (Jetty_rts__0__HSIOM_SHIFT)    
    #define Jetty_RTS_HSIOM_SEL_GPIO (Jetty_rts__0__HSIOM_GPIO)
    #define Jetty_RTS_HSIOM_SEL_UART (Jetty_rts__0__HSIOM_UART)    
#endif /* (Jetty_UART_RTS_PIN) */


/***************************************
*        Registers Constants
***************************************/

/* HSIOM switch values. */ 
#define Jetty_HSIOM_DEF_SEL      (0x00u)
#define Jetty_HSIOM_GPIO_SEL     (0x00u)
/* The HSIOM values provided below are valid only for Jetty_CY_SCBIP_V0 
* and Jetty_CY_SCBIP_V1. It is not recommended to use them for 
* Jetty_CY_SCBIP_V2. Use pin name specific HSIOM constants provided 
* above instead for any SCB IP block version.
*/
#define Jetty_HSIOM_UART_SEL     (0x09u)
#define Jetty_HSIOM_I2C_SEL      (0x0Eu)
#define Jetty_HSIOM_SPI_SEL      (0x0Fu)

/* Pins settings index. */
#define Jetty_RX_WAKE_SCL_MOSI_PIN_INDEX   (0u)
#define Jetty_RX_SCL_MOSI_PIN_INDEX       (0u)
#define Jetty_TX_SDA_MISO_PIN_INDEX       (1u)
#define Jetty_CTS_SCLK_PIN_INDEX       (2u)
#define Jetty_RTS_SS0_PIN_INDEX       (3u)
#define Jetty_SS1_PIN_INDEX                  (4u)
#define Jetty_SS2_PIN_INDEX                  (5u)
#define Jetty_SS3_PIN_INDEX                  (6u)

/* Pins settings mask. */
#define Jetty_RX_WAKE_SCL_MOSI_PIN_MASK ((uint32) 0x01u << Jetty_RX_WAKE_SCL_MOSI_PIN_INDEX)
#define Jetty_RX_SCL_MOSI_PIN_MASK     ((uint32) 0x01u << Jetty_RX_SCL_MOSI_PIN_INDEX)
#define Jetty_TX_SDA_MISO_PIN_MASK     ((uint32) 0x01u << Jetty_TX_SDA_MISO_PIN_INDEX)
#define Jetty_CTS_SCLK_PIN_MASK     ((uint32) 0x01u << Jetty_CTS_SCLK_PIN_INDEX)
#define Jetty_RTS_SS0_PIN_MASK     ((uint32) 0x01u << Jetty_RTS_SS0_PIN_INDEX)
#define Jetty_SS1_PIN_MASK                ((uint32) 0x01u << Jetty_SS1_PIN_INDEX)
#define Jetty_SS2_PIN_MASK                ((uint32) 0x01u << Jetty_SS2_PIN_INDEX)
#define Jetty_SS3_PIN_MASK                ((uint32) 0x01u << Jetty_SS3_PIN_INDEX)

/* Pin interrupt constants. */
#define Jetty_INTCFG_TYPE_MASK           (0x03u)
#define Jetty_INTCFG_TYPE_FALLING_EDGE   (0x02u)

/* Pin Drive Mode constants. */
#define Jetty_PIN_DM_ALG_HIZ  (0u)
#define Jetty_PIN_DM_DIG_HIZ  (1u)
#define Jetty_PIN_DM_OD_LO    (4u)
#define Jetty_PIN_DM_STRONG   (6u)


/***************************************
*          Macro Definitions
***************************************/

/* Return drive mode of the pin */
#define Jetty_DM_MASK    (0x7u)
#define Jetty_DM_SIZE    (3u)
#define Jetty_GET_P4_PIN_DM(reg, pos) \
    ( ((reg) & (uint32) ((uint32) Jetty_DM_MASK << (Jetty_DM_SIZE * (pos)))) >> \
                                                              (Jetty_DM_SIZE * (pos)) )

#if (Jetty_TX_SDA_MISO_PIN)
    #define Jetty_CHECK_TX_SDA_MISO_PIN_USED \
                (Jetty_PIN_DM_ALG_HIZ != \
                    Jetty_GET_P4_PIN_DM(Jetty_uart_tx_i2c_sda_spi_miso_PC, \
                                                   Jetty_uart_tx_i2c_sda_spi_miso_SHIFT))
#endif /* (Jetty_TX_SDA_MISO_PIN) */

#if (Jetty_RTS_SS0_PIN)
    #define Jetty_CHECK_RTS_SS0_PIN_USED \
                (Jetty_PIN_DM_ALG_HIZ != \
                    Jetty_GET_P4_PIN_DM(Jetty_uart_rts_spi_ss0_PC, \
                                                   Jetty_uart_rts_spi_ss0_SHIFT))
#endif /* (Jetty_RTS_SS0_PIN) */

/* Set bits-mask in register */
#define Jetty_SET_REGISTER_BITS(reg, mask, pos, mode) \
                    do                                           \
                    {                                            \
                        (reg) = (((reg) & ((uint32) ~(uint32) (mask))) | ((uint32) ((uint32) (mode) << (pos)))); \
                    }while(0)

/* Set bit in the register */
#define Jetty_SET_REGISTER_BIT(reg, mask, val) \
                    ((val) ? ((reg) |= (mask)) : ((reg) &= ((uint32) ~((uint32) (mask)))))

#define Jetty_SET_HSIOM_SEL(reg, mask, pos, sel) Jetty_SET_REGISTER_BITS(reg, mask, pos, sel)
#define Jetty_SET_INCFG_TYPE(reg, mask, pos, intType) \
                                                        Jetty_SET_REGISTER_BITS(reg, mask, pos, intType)
#define Jetty_SET_INP_DIS(reg, mask, val) Jetty_SET_REGISTER_BIT(reg, mask, val)

/* Jetty_SET_I2C_SCL_DR(val) - Sets I2C SCL DR register.
*  Jetty_SET_I2C_SCL_HSIOM_SEL(sel) - Sets I2C SCL HSIOM settings.
*/
/* SCB I2C: scl signal */
#if (Jetty_CY_SCBIP_V0)
#if (Jetty_I2C_PINS)
    #define Jetty_SET_I2C_SCL_DR(val) Jetty_scl_Write(val)

    #define Jetty_SET_I2C_SCL_HSIOM_SEL(sel) \
                          Jetty_SET_HSIOM_SEL(Jetty_SCL_HSIOM_REG,  \
                                                         Jetty_SCL_HSIOM_MASK, \
                                                         Jetty_SCL_HSIOM_POS,  \
                                                         (sel))
    #define Jetty_WAIT_SCL_SET_HIGH  (0u == Jetty_scl_Read())

/* Unconfigured SCB: scl signal */
#elif (Jetty_RX_WAKE_SCL_MOSI_PIN)
    #define Jetty_SET_I2C_SCL_DR(val) \
                            Jetty_uart_rx_wake_i2c_scl_spi_mosi_Write(val)

    #define Jetty_SET_I2C_SCL_HSIOM_SEL(sel) \
                    Jetty_SET_HSIOM_SEL(Jetty_RX_WAKE_SCL_MOSI_HSIOM_REG,  \
                                                   Jetty_RX_WAKE_SCL_MOSI_HSIOM_MASK, \
                                                   Jetty_RX_WAKE_SCL_MOSI_HSIOM_POS,  \
                                                   (sel))

    #define Jetty_WAIT_SCL_SET_HIGH  (0u == Jetty_uart_rx_wake_i2c_scl_spi_mosi_Read())

#elif (Jetty_RX_SCL_MOSI_PIN)
    #define Jetty_SET_I2C_SCL_DR(val) \
                            Jetty_uart_rx_i2c_scl_spi_mosi_Write(val)


    #define Jetty_SET_I2C_SCL_HSIOM_SEL(sel) \
                            Jetty_SET_HSIOM_SEL(Jetty_RX_SCL_MOSI_HSIOM_REG,  \
                                                           Jetty_RX_SCL_MOSI_HSIOM_MASK, \
                                                           Jetty_RX_SCL_MOSI_HSIOM_POS,  \
                                                           (sel))

    #define Jetty_WAIT_SCL_SET_HIGH  (0u == Jetty_uart_rx_i2c_scl_spi_mosi_Read())

#else
    #define Jetty_SET_I2C_SCL_DR(val)        do{ /* Does nothing */ }while(0)
    #define Jetty_SET_I2C_SCL_HSIOM_SEL(sel) do{ /* Does nothing */ }while(0)

    #define Jetty_WAIT_SCL_SET_HIGH  (0u)
#endif /* (Jetty_I2C_PINS) */

/* SCB I2C: sda signal */
#if (Jetty_I2C_PINS)
    #define Jetty_WAIT_SDA_SET_HIGH  (0u == Jetty_sda_Read())
/* Unconfigured SCB: sda signal */
#elif (Jetty_TX_SDA_MISO_PIN)
    #define Jetty_WAIT_SDA_SET_HIGH  (0u == Jetty_uart_tx_i2c_sda_spi_miso_Read())
#else
    #define Jetty_WAIT_SDA_SET_HIGH  (0u)
#endif /* (Jetty_MOSI_SCL_RX_PIN) */
#endif /* (Jetty_CY_SCBIP_V0) */

/* Clear UART wakeup source */
#if (Jetty_RX_SCL_MOSI_PIN)
    #define Jetty_CLEAR_UART_RX_WAKE_INTR        do{ /* Does nothing */ }while(0)
    
#elif (Jetty_RX_WAKE_SCL_MOSI_PIN)
    #define Jetty_CLEAR_UART_RX_WAKE_INTR \
            do{                                      \
                (void) Jetty_uart_rx_wake_i2c_scl_spi_mosi_ClearInterrupt(); \
            }while(0)

#elif(Jetty_UART_RX_WAKE_PIN)
    #define Jetty_CLEAR_UART_RX_WAKE_INTR \
            do{                                      \
                (void) Jetty_rx_wake_ClearInterrupt(); \
            }while(0)
#else
#endif /* (Jetty_RX_SCL_MOSI_PIN) */


/***************************************
* The following code is DEPRECATED and
* must not be used.
***************************************/

/* Unconfigured pins */
#define Jetty_REMOVE_MOSI_SCL_RX_WAKE_PIN    Jetty_REMOVE_RX_WAKE_SCL_MOSI_PIN
#define Jetty_REMOVE_MOSI_SCL_RX_PIN         Jetty_REMOVE_RX_SCL_MOSI_PIN
#define Jetty_REMOVE_MISO_SDA_TX_PIN         Jetty_REMOVE_TX_SDA_MISO_PIN
#ifndef Jetty_REMOVE_SCLK_PIN
#define Jetty_REMOVE_SCLK_PIN                Jetty_REMOVE_CTS_SCLK_PIN
#endif /* Jetty_REMOVE_SCLK_PIN */
#ifndef Jetty_REMOVE_SS0_PIN
#define Jetty_REMOVE_SS0_PIN                 Jetty_REMOVE_RTS_SS0_PIN
#endif /* Jetty_REMOVE_SS0_PIN */

/* Unconfigured pins */
#define Jetty_MOSI_SCL_RX_WAKE_PIN   Jetty_RX_WAKE_SCL_MOSI_PIN
#define Jetty_MOSI_SCL_RX_PIN        Jetty_RX_SCL_MOSI_PIN
#define Jetty_MISO_SDA_TX_PIN        Jetty_TX_SDA_MISO_PIN
#ifndef Jetty_SCLK_PIN
#define Jetty_SCLK_PIN               Jetty_CTS_SCLK_PIN
#endif /* Jetty_SCLK_PIN */
#ifndef Jetty_SS0_PIN
#define Jetty_SS0_PIN                Jetty_RTS_SS0_PIN
#endif /* Jetty_SS0_PIN */

#if (Jetty_MOSI_SCL_RX_WAKE_PIN)
    #define Jetty_MOSI_SCL_RX_WAKE_HSIOM_REG     Jetty_RX_WAKE_SCL_MOSI_HSIOM_REG
    #define Jetty_MOSI_SCL_RX_WAKE_HSIOM_PTR     Jetty_RX_WAKE_SCL_MOSI_HSIOM_REG
    #define Jetty_MOSI_SCL_RX_WAKE_HSIOM_MASK    Jetty_RX_WAKE_SCL_MOSI_HSIOM_REG
    #define Jetty_MOSI_SCL_RX_WAKE_HSIOM_POS     Jetty_RX_WAKE_SCL_MOSI_HSIOM_REG

    #define Jetty_MOSI_SCL_RX_WAKE_INTCFG_REG    Jetty_RX_WAKE_SCL_MOSI_HSIOM_REG
    #define Jetty_MOSI_SCL_RX_WAKE_INTCFG_PTR    Jetty_RX_WAKE_SCL_MOSI_HSIOM_REG

    #define Jetty_MOSI_SCL_RX_WAKE_INTCFG_TYPE_POS   Jetty_RX_WAKE_SCL_MOSI_HSIOM_REG
    #define Jetty_MOSI_SCL_RX_WAKE_INTCFG_TYPE_MASK  Jetty_RX_WAKE_SCL_MOSI_HSIOM_REG
#endif /* (Jetty_RX_WAKE_SCL_MOSI_PIN) */

#if (Jetty_MOSI_SCL_RX_PIN)
    #define Jetty_MOSI_SCL_RX_HSIOM_REG      Jetty_RX_SCL_MOSI_HSIOM_REG
    #define Jetty_MOSI_SCL_RX_HSIOM_PTR      Jetty_RX_SCL_MOSI_HSIOM_PTR
    #define Jetty_MOSI_SCL_RX_HSIOM_MASK     Jetty_RX_SCL_MOSI_HSIOM_MASK
    #define Jetty_MOSI_SCL_RX_HSIOM_POS      Jetty_RX_SCL_MOSI_HSIOM_POS
#endif /* (Jetty_MOSI_SCL_RX_PIN) */

#if (Jetty_MISO_SDA_TX_PIN)
    #define Jetty_MISO_SDA_TX_HSIOM_REG      Jetty_TX_SDA_MISO_HSIOM_REG
    #define Jetty_MISO_SDA_TX_HSIOM_PTR      Jetty_TX_SDA_MISO_HSIOM_REG
    #define Jetty_MISO_SDA_TX_HSIOM_MASK     Jetty_TX_SDA_MISO_HSIOM_REG
    #define Jetty_MISO_SDA_TX_HSIOM_POS      Jetty_TX_SDA_MISO_HSIOM_REG
#endif /* (Jetty_MISO_SDA_TX_PIN_PIN) */

#if (Jetty_SCLK_PIN)
    #ifndef Jetty_SCLK_HSIOM_REG
    #define Jetty_SCLK_HSIOM_REG     Jetty_CTS_SCLK_HSIOM_REG
    #define Jetty_SCLK_HSIOM_PTR     Jetty_CTS_SCLK_HSIOM_PTR
    #define Jetty_SCLK_HSIOM_MASK    Jetty_CTS_SCLK_HSIOM_MASK
    #define Jetty_SCLK_HSIOM_POS     Jetty_CTS_SCLK_HSIOM_POS
    #endif /* Jetty_SCLK_HSIOM_REG */
#endif /* (Jetty_SCLK_PIN) */

#if (Jetty_SS0_PIN)
    #ifndef Jetty_SS0_HSIOM_REG
    #define Jetty_SS0_HSIOM_REG      Jetty_RTS_SS0_HSIOM_REG
    #define Jetty_SS0_HSIOM_PTR      Jetty_RTS_SS0_HSIOM_PTR
    #define Jetty_SS0_HSIOM_MASK     Jetty_RTS_SS0_HSIOM_MASK
    #define Jetty_SS0_HSIOM_POS      Jetty_RTS_SS0_HSIOM_POS
    #endif /* Jetty_SS0_HSIOM_REG */
#endif /* (Jetty_SS0_PIN) */

#define Jetty_MOSI_SCL_RX_WAKE_PIN_INDEX Jetty_RX_WAKE_SCL_MOSI_PIN_INDEX
#define Jetty_MOSI_SCL_RX_PIN_INDEX      Jetty_RX_SCL_MOSI_PIN_INDEX
#define Jetty_MISO_SDA_TX_PIN_INDEX      Jetty_TX_SDA_MISO_PIN_INDEX
#ifndef Jetty_SCLK_PIN_INDEX
#define Jetty_SCLK_PIN_INDEX             Jetty_CTS_SCLK_PIN_INDEX
#endif /* Jetty_SCLK_PIN_INDEX */
#ifndef Jetty_SS0_PIN_INDEX
#define Jetty_SS0_PIN_INDEX              Jetty_RTS_SS0_PIN_INDEX
#endif /* Jetty_SS0_PIN_INDEX */

#define Jetty_MOSI_SCL_RX_WAKE_PIN_MASK Jetty_RX_WAKE_SCL_MOSI_PIN_MASK
#define Jetty_MOSI_SCL_RX_PIN_MASK      Jetty_RX_SCL_MOSI_PIN_MASK
#define Jetty_MISO_SDA_TX_PIN_MASK      Jetty_TX_SDA_MISO_PIN_MASK
#ifndef Jetty_SCLK_PIN_MASK
#define Jetty_SCLK_PIN_MASK             Jetty_CTS_SCLK_PIN_MASK
#endif /* Jetty_SCLK_PIN_MASK */
#ifndef Jetty_SS0_PIN_MASK
#define Jetty_SS0_PIN_MASK              Jetty_RTS_SS0_PIN_MASK
#endif /* Jetty_SS0_PIN_MASK */

#endif /* (CY_SCB_PINS_Jetty_H) */


/* [] END OF FILE */
