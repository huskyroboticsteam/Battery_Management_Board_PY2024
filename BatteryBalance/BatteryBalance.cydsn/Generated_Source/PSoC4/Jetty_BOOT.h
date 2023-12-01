/***************************************************************************//**
* \file Jetty_BOOT.h
* \version 4.0
*
* \brief
*  This file provides constants and parameter values of the bootloader
*  communication APIs for the SCB Component.
*
* Note:
*
********************************************************************************
* \copyright
* Copyright 2014-2017, Cypress Semiconductor Corporation. All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#if !defined(CY_SCB_BOOT_Jetty_H)
#define CY_SCB_BOOT_Jetty_H

#include "Jetty_PVT.h"

#if (Jetty_SCB_MODE_I2C_INC)
    #include "Jetty_I2C.h"
#endif /* (Jetty_SCB_MODE_I2C_INC) */

#if (Jetty_SCB_MODE_EZI2C_INC)
    #include "Jetty_EZI2C.h"
#endif /* (Jetty_SCB_MODE_EZI2C_INC) */

#if (Jetty_SCB_MODE_SPI_INC || Jetty_SCB_MODE_UART_INC)
    #include "Jetty_SPI_UART.h"
#endif /* (Jetty_SCB_MODE_SPI_INC || Jetty_SCB_MODE_UART_INC) */


/***************************************
*  Conditional Compilation Parameters
****************************************/

/* Bootloader communication interface enable */
#define Jetty_BTLDR_COMM_ENABLED ((CYDEV_BOOTLOADER_IO_COMP == CyBtldr_Jetty) || \
                                             (CYDEV_BOOTLOADER_IO_COMP == CyBtldr_Custom_Interface))

/* Enable I2C bootloader communication */
#if (Jetty_SCB_MODE_I2C_INC)
    #define Jetty_I2C_BTLDR_COMM_ENABLED     (Jetty_BTLDR_COMM_ENABLED && \
                                                            (Jetty_SCB_MODE_UNCONFIG_CONST_CFG || \
                                                             Jetty_I2C_SLAVE_CONST))
#else
     #define Jetty_I2C_BTLDR_COMM_ENABLED    (0u)
#endif /* (Jetty_SCB_MODE_I2C_INC) */

/* EZI2C does not support bootloader communication. Provide empty APIs */
#if (Jetty_SCB_MODE_EZI2C_INC)
    #define Jetty_EZI2C_BTLDR_COMM_ENABLED   (Jetty_BTLDR_COMM_ENABLED && \
                                                         Jetty_SCB_MODE_UNCONFIG_CONST_CFG)
#else
    #define Jetty_EZI2C_BTLDR_COMM_ENABLED   (0u)
#endif /* (Jetty_EZI2C_BTLDR_COMM_ENABLED) */

/* Enable SPI bootloader communication */
#if (Jetty_SCB_MODE_SPI_INC)
    #define Jetty_SPI_BTLDR_COMM_ENABLED     (Jetty_BTLDR_COMM_ENABLED && \
                                                            (Jetty_SCB_MODE_UNCONFIG_CONST_CFG || \
                                                             Jetty_SPI_SLAVE_CONST))
#else
        #define Jetty_SPI_BTLDR_COMM_ENABLED (0u)
#endif /* (Jetty_SPI_BTLDR_COMM_ENABLED) */

/* Enable UART bootloader communication */
#if (Jetty_SCB_MODE_UART_INC)
       #define Jetty_UART_BTLDR_COMM_ENABLED    (Jetty_BTLDR_COMM_ENABLED && \
                                                            (Jetty_SCB_MODE_UNCONFIG_CONST_CFG || \
                                                             (Jetty_UART_RX_DIRECTION && \
                                                              Jetty_UART_TX_DIRECTION)))
#else
     #define Jetty_UART_BTLDR_COMM_ENABLED   (0u)
#endif /* (Jetty_UART_BTLDR_COMM_ENABLED) */

/* Enable bootloader communication */
#define Jetty_BTLDR_COMM_MODE_ENABLED    (Jetty_I2C_BTLDR_COMM_ENABLED   || \
                                                     Jetty_SPI_BTLDR_COMM_ENABLED   || \
                                                     Jetty_EZI2C_BTLDR_COMM_ENABLED || \
                                                     Jetty_UART_BTLDR_COMM_ENABLED)


/***************************************
*        Function Prototypes
***************************************/

#if defined(CYDEV_BOOTLOADER_IO_COMP) && (Jetty_I2C_BTLDR_COMM_ENABLED)
    /* I2C Bootloader physical layer functions */
    void Jetty_I2CCyBtldrCommStart(void);
    void Jetty_I2CCyBtldrCommStop (void);
    void Jetty_I2CCyBtldrCommReset(void);
    cystatus Jetty_I2CCyBtldrCommRead       (uint8 pData[], uint16 size, uint16 * count, uint8 timeOut);
    cystatus Jetty_I2CCyBtldrCommWrite(const uint8 pData[], uint16 size, uint16 * count, uint8 timeOut);

    /* Map I2C specific bootloader communication APIs to SCB specific APIs */
    #if (Jetty_SCB_MODE_I2C_CONST_CFG)
        #define Jetty_CyBtldrCommStart   Jetty_I2CCyBtldrCommStart
        #define Jetty_CyBtldrCommStop    Jetty_I2CCyBtldrCommStop
        #define Jetty_CyBtldrCommReset   Jetty_I2CCyBtldrCommReset
        #define Jetty_CyBtldrCommRead    Jetty_I2CCyBtldrCommRead
        #define Jetty_CyBtldrCommWrite   Jetty_I2CCyBtldrCommWrite
    #endif /* (Jetty_SCB_MODE_I2C_CONST_CFG) */

#endif /* defined(CYDEV_BOOTLOADER_IO_COMP) && (Jetty_I2C_BTLDR_COMM_ENABLED) */


#if defined(CYDEV_BOOTLOADER_IO_COMP) && (Jetty_EZI2C_BTLDR_COMM_ENABLED)
    /* Bootloader physical layer functions */
    void Jetty_EzI2CCyBtldrCommStart(void);
    void Jetty_EzI2CCyBtldrCommStop (void);
    void Jetty_EzI2CCyBtldrCommReset(void);
    cystatus Jetty_EzI2CCyBtldrCommRead       (uint8 pData[], uint16 size, uint16 * count, uint8 timeOut);
    cystatus Jetty_EzI2CCyBtldrCommWrite(const uint8 pData[], uint16 size, uint16 * count, uint8 timeOut);

    /* Map EZI2C specific bootloader communication APIs to SCB specific APIs */
    #if (Jetty_SCB_MODE_EZI2C_CONST_CFG)
        #define Jetty_CyBtldrCommStart   Jetty_EzI2CCyBtldrCommStart
        #define Jetty_CyBtldrCommStop    Jetty_EzI2CCyBtldrCommStop
        #define Jetty_CyBtldrCommReset   Jetty_EzI2CCyBtldrCommReset
        #define Jetty_CyBtldrCommRead    Jetty_EzI2CCyBtldrCommRead
        #define Jetty_CyBtldrCommWrite   Jetty_EzI2CCyBtldrCommWrite
    #endif /* (Jetty_SCB_MODE_EZI2C_CONST_CFG) */

#endif /* defined(CYDEV_BOOTLOADER_IO_COMP) && (Jetty_EZI2C_BTLDR_COMM_ENABLED) */

#if defined(CYDEV_BOOTLOADER_IO_COMP) && (Jetty_SPI_BTLDR_COMM_ENABLED)
    /* SPI Bootloader physical layer functions */
    void Jetty_SpiCyBtldrCommStart(void);
    void Jetty_SpiCyBtldrCommStop (void);
    void Jetty_SpiCyBtldrCommReset(void);
    cystatus Jetty_SpiCyBtldrCommRead       (uint8 pData[], uint16 size, uint16 * count, uint8 timeOut);
    cystatus Jetty_SpiCyBtldrCommWrite(const uint8 pData[], uint16 size, uint16 * count, uint8 timeOut);

    /* Map SPI specific bootloader communication APIs to SCB specific APIs */
    #if (Jetty_SCB_MODE_SPI_CONST_CFG)
        #define Jetty_CyBtldrCommStart   Jetty_SpiCyBtldrCommStart
        #define Jetty_CyBtldrCommStop    Jetty_SpiCyBtldrCommStop
        #define Jetty_CyBtldrCommReset   Jetty_SpiCyBtldrCommReset
        #define Jetty_CyBtldrCommRead    Jetty_SpiCyBtldrCommRead
        #define Jetty_CyBtldrCommWrite   Jetty_SpiCyBtldrCommWrite
    #endif /* (Jetty_SCB_MODE_SPI_CONST_CFG) */

#endif /* defined(CYDEV_BOOTLOADER_IO_COMP) && (Jetty_SPI_BTLDR_COMM_ENABLED) */

#if defined(CYDEV_BOOTLOADER_IO_COMP) && (Jetty_UART_BTLDR_COMM_ENABLED)
    /* UART Bootloader physical layer functions */
    void Jetty_UartCyBtldrCommStart(void);
    void Jetty_UartCyBtldrCommStop (void);
    void Jetty_UartCyBtldrCommReset(void);
    cystatus Jetty_UartCyBtldrCommRead       (uint8 pData[], uint16 size, uint16 * count, uint8 timeOut);
    cystatus Jetty_UartCyBtldrCommWrite(const uint8 pData[], uint16 size, uint16 * count, uint8 timeOut);

    /* Map UART specific bootloader communication APIs to SCB specific APIs */
    #if (Jetty_SCB_MODE_UART_CONST_CFG)
        #define Jetty_CyBtldrCommStart   Jetty_UartCyBtldrCommStart
        #define Jetty_CyBtldrCommStop    Jetty_UartCyBtldrCommStop
        #define Jetty_CyBtldrCommReset   Jetty_UartCyBtldrCommReset
        #define Jetty_CyBtldrCommRead    Jetty_UartCyBtldrCommRead
        #define Jetty_CyBtldrCommWrite   Jetty_UartCyBtldrCommWrite
    #endif /* (Jetty_SCB_MODE_UART_CONST_CFG) */

#endif /* defined(CYDEV_BOOTLOADER_IO_COMP) && (Jetty_UART_BTLDR_COMM_ENABLED) */

/**
* \addtogroup group_bootloader
* @{
*/

#if defined(CYDEV_BOOTLOADER_IO_COMP) && (Jetty_BTLDR_COMM_ENABLED)
    #if (Jetty_SCB_MODE_UNCONFIG_CONST_CFG)
        /* Bootloader physical layer functions */
        void Jetty_CyBtldrCommStart(void);
        void Jetty_CyBtldrCommStop (void);
        void Jetty_CyBtldrCommReset(void);
        cystatus Jetty_CyBtldrCommRead       (uint8 pData[], uint16 size, uint16 * count, uint8 timeOut);
        cystatus Jetty_CyBtldrCommWrite(const uint8 pData[], uint16 size, uint16 * count, uint8 timeOut);
    #endif /* (Jetty_SCB_MODE_UNCONFIG_CONST_CFG) */

    /* Map SCB specific bootloader communication APIs to common APIs */
    #if (CYDEV_BOOTLOADER_IO_COMP == CyBtldr_Jetty)
        #define CyBtldrCommStart    Jetty_CyBtldrCommStart
        #define CyBtldrCommStop     Jetty_CyBtldrCommStop
        #define CyBtldrCommReset    Jetty_CyBtldrCommReset
        #define CyBtldrCommWrite    Jetty_CyBtldrCommWrite
        #define CyBtldrCommRead     Jetty_CyBtldrCommRead
    #endif /* (CYDEV_BOOTLOADER_IO_COMP == CyBtldr_Jetty) */

#endif /* defined(CYDEV_BOOTLOADER_IO_COMP) && (Jetty_BTLDR_COMM_ENABLED) */

/** @} group_bootloader */

/***************************************
*           API Constants
***************************************/

/* Timeout unit in milliseconds */
#define Jetty_WAIT_1_MS  (1u)

/* Return number of bytes to copy into bootloader buffer */
#define Jetty_BYTES_TO_COPY(actBufSize, bufSize) \
                            ( ((uint32)(actBufSize) < (uint32)(bufSize)) ? \
                                ((uint32) (actBufSize)) : ((uint32) (bufSize)) )

/* Size of Read/Write buffers for I2C bootloader  */
#define Jetty_I2C_BTLDR_SIZEOF_READ_BUFFER   (64u)
#define Jetty_I2C_BTLDR_SIZEOF_WRITE_BUFFER  (64u)

/* Byte to byte time interval: calculated basing on current component
* data rate configuration, can be defined in project if required.
*/
#ifndef Jetty_SPI_BYTE_TO_BYTE
    #define Jetty_SPI_BYTE_TO_BYTE   (160u)
#endif

/* Byte to byte time interval: calculated basing on current component
* baud rate configuration, can be defined in the project if required.
*/
#ifndef Jetty_UART_BYTE_TO_BYTE
    #define Jetty_UART_BYTE_TO_BYTE  (2500u)
#endif /* Jetty_UART_BYTE_TO_BYTE */

#endif /* (CY_SCB_BOOT_Jetty_H) */


/* [] END OF FILE */
