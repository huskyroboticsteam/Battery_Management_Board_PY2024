/***************************************************************************//**
* \file INA226_BOOT.h
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

#if !defined(CY_SCB_BOOT_INA226_H)
#define CY_SCB_BOOT_INA226_H

#include "INA226_PVT.h"

#if (INA226_SCB_MODE_I2C_INC)
    #include "INA226_I2C.h"
#endif /* (INA226_SCB_MODE_I2C_INC) */

#if (INA226_SCB_MODE_EZI2C_INC)
    #include "INA226_EZI2C.h"
#endif /* (INA226_SCB_MODE_EZI2C_INC) */

#if (INA226_SCB_MODE_SPI_INC || INA226_SCB_MODE_UART_INC)
    #include "INA226_SPI_UART.h"
#endif /* (INA226_SCB_MODE_SPI_INC || INA226_SCB_MODE_UART_INC) */


/***************************************
*  Conditional Compilation Parameters
****************************************/

/* Bootloader communication interface enable */
#define INA226_BTLDR_COMM_ENABLED ((CYDEV_BOOTLOADER_IO_COMP == CyBtldr_INA226) || \
                                             (CYDEV_BOOTLOADER_IO_COMP == CyBtldr_Custom_Interface))

/* Enable I2C bootloader communication */
#if (INA226_SCB_MODE_I2C_INC)
    #define INA226_I2C_BTLDR_COMM_ENABLED     (INA226_BTLDR_COMM_ENABLED && \
                                                            (INA226_SCB_MODE_UNCONFIG_CONST_CFG || \
                                                             INA226_I2C_SLAVE_CONST))
#else
     #define INA226_I2C_BTLDR_COMM_ENABLED    (0u)
#endif /* (INA226_SCB_MODE_I2C_INC) */

/* EZI2C does not support bootloader communication. Provide empty APIs */
#if (INA226_SCB_MODE_EZI2C_INC)
    #define INA226_EZI2C_BTLDR_COMM_ENABLED   (INA226_BTLDR_COMM_ENABLED && \
                                                         INA226_SCB_MODE_UNCONFIG_CONST_CFG)
#else
    #define INA226_EZI2C_BTLDR_COMM_ENABLED   (0u)
#endif /* (INA226_EZI2C_BTLDR_COMM_ENABLED) */

/* Enable SPI bootloader communication */
#if (INA226_SCB_MODE_SPI_INC)
    #define INA226_SPI_BTLDR_COMM_ENABLED     (INA226_BTLDR_COMM_ENABLED && \
                                                            (INA226_SCB_MODE_UNCONFIG_CONST_CFG || \
                                                             INA226_SPI_SLAVE_CONST))
#else
        #define INA226_SPI_BTLDR_COMM_ENABLED (0u)
#endif /* (INA226_SPI_BTLDR_COMM_ENABLED) */

/* Enable UART bootloader communication */
#if (INA226_SCB_MODE_UART_INC)
       #define INA226_UART_BTLDR_COMM_ENABLED    (INA226_BTLDR_COMM_ENABLED && \
                                                            (INA226_SCB_MODE_UNCONFIG_CONST_CFG || \
                                                             (INA226_UART_RX_DIRECTION && \
                                                              INA226_UART_TX_DIRECTION)))
#else
     #define INA226_UART_BTLDR_COMM_ENABLED   (0u)
#endif /* (INA226_UART_BTLDR_COMM_ENABLED) */

/* Enable bootloader communication */
#define INA226_BTLDR_COMM_MODE_ENABLED    (INA226_I2C_BTLDR_COMM_ENABLED   || \
                                                     INA226_SPI_BTLDR_COMM_ENABLED   || \
                                                     INA226_EZI2C_BTLDR_COMM_ENABLED || \
                                                     INA226_UART_BTLDR_COMM_ENABLED)


/***************************************
*        Function Prototypes
***************************************/

#if defined(CYDEV_BOOTLOADER_IO_COMP) && (INA226_I2C_BTLDR_COMM_ENABLED)
    /* I2C Bootloader physical layer functions */
    void INA226_I2CCyBtldrCommStart(void);
    void INA226_I2CCyBtldrCommStop (void);
    void INA226_I2CCyBtldrCommReset(void);
    cystatus INA226_I2CCyBtldrCommRead       (uint8 pData[], uint16 size, uint16 * count, uint8 timeOut);
    cystatus INA226_I2CCyBtldrCommWrite(const uint8 pData[], uint16 size, uint16 * count, uint8 timeOut);

    /* Map I2C specific bootloader communication APIs to SCB specific APIs */
    #if (INA226_SCB_MODE_I2C_CONST_CFG)
        #define INA226_CyBtldrCommStart   INA226_I2CCyBtldrCommStart
        #define INA226_CyBtldrCommStop    INA226_I2CCyBtldrCommStop
        #define INA226_CyBtldrCommReset   INA226_I2CCyBtldrCommReset
        #define INA226_CyBtldrCommRead    INA226_I2CCyBtldrCommRead
        #define INA226_CyBtldrCommWrite   INA226_I2CCyBtldrCommWrite
    #endif /* (INA226_SCB_MODE_I2C_CONST_CFG) */

#endif /* defined(CYDEV_BOOTLOADER_IO_COMP) && (INA226_I2C_BTLDR_COMM_ENABLED) */


#if defined(CYDEV_BOOTLOADER_IO_COMP) && (INA226_EZI2C_BTLDR_COMM_ENABLED)
    /* Bootloader physical layer functions */
    void INA226_EzI2CCyBtldrCommStart(void);
    void INA226_EzI2CCyBtldrCommStop (void);
    void INA226_EzI2CCyBtldrCommReset(void);
    cystatus INA226_EzI2CCyBtldrCommRead       (uint8 pData[], uint16 size, uint16 * count, uint8 timeOut);
    cystatus INA226_EzI2CCyBtldrCommWrite(const uint8 pData[], uint16 size, uint16 * count, uint8 timeOut);

    /* Map EZI2C specific bootloader communication APIs to SCB specific APIs */
    #if (INA226_SCB_MODE_EZI2C_CONST_CFG)
        #define INA226_CyBtldrCommStart   INA226_EzI2CCyBtldrCommStart
        #define INA226_CyBtldrCommStop    INA226_EzI2CCyBtldrCommStop
        #define INA226_CyBtldrCommReset   INA226_EzI2CCyBtldrCommReset
        #define INA226_CyBtldrCommRead    INA226_EzI2CCyBtldrCommRead
        #define INA226_CyBtldrCommWrite   INA226_EzI2CCyBtldrCommWrite
    #endif /* (INA226_SCB_MODE_EZI2C_CONST_CFG) */

#endif /* defined(CYDEV_BOOTLOADER_IO_COMP) && (INA226_EZI2C_BTLDR_COMM_ENABLED) */

#if defined(CYDEV_BOOTLOADER_IO_COMP) && (INA226_SPI_BTLDR_COMM_ENABLED)
    /* SPI Bootloader physical layer functions */
    void INA226_SpiCyBtldrCommStart(void);
    void INA226_SpiCyBtldrCommStop (void);
    void INA226_SpiCyBtldrCommReset(void);
    cystatus INA226_SpiCyBtldrCommRead       (uint8 pData[], uint16 size, uint16 * count, uint8 timeOut);
    cystatus INA226_SpiCyBtldrCommWrite(const uint8 pData[], uint16 size, uint16 * count, uint8 timeOut);

    /* Map SPI specific bootloader communication APIs to SCB specific APIs */
    #if (INA226_SCB_MODE_SPI_CONST_CFG)
        #define INA226_CyBtldrCommStart   INA226_SpiCyBtldrCommStart
        #define INA226_CyBtldrCommStop    INA226_SpiCyBtldrCommStop
        #define INA226_CyBtldrCommReset   INA226_SpiCyBtldrCommReset
        #define INA226_CyBtldrCommRead    INA226_SpiCyBtldrCommRead
        #define INA226_CyBtldrCommWrite   INA226_SpiCyBtldrCommWrite
    #endif /* (INA226_SCB_MODE_SPI_CONST_CFG) */

#endif /* defined(CYDEV_BOOTLOADER_IO_COMP) && (INA226_SPI_BTLDR_COMM_ENABLED) */

#if defined(CYDEV_BOOTLOADER_IO_COMP) && (INA226_UART_BTLDR_COMM_ENABLED)
    /* UART Bootloader physical layer functions */
    void INA226_UartCyBtldrCommStart(void);
    void INA226_UartCyBtldrCommStop (void);
    void INA226_UartCyBtldrCommReset(void);
    cystatus INA226_UartCyBtldrCommRead       (uint8 pData[], uint16 size, uint16 * count, uint8 timeOut);
    cystatus INA226_UartCyBtldrCommWrite(const uint8 pData[], uint16 size, uint16 * count, uint8 timeOut);

    /* Map UART specific bootloader communication APIs to SCB specific APIs */
    #if (INA226_SCB_MODE_UART_CONST_CFG)
        #define INA226_CyBtldrCommStart   INA226_UartCyBtldrCommStart
        #define INA226_CyBtldrCommStop    INA226_UartCyBtldrCommStop
        #define INA226_CyBtldrCommReset   INA226_UartCyBtldrCommReset
        #define INA226_CyBtldrCommRead    INA226_UartCyBtldrCommRead
        #define INA226_CyBtldrCommWrite   INA226_UartCyBtldrCommWrite
    #endif /* (INA226_SCB_MODE_UART_CONST_CFG) */

#endif /* defined(CYDEV_BOOTLOADER_IO_COMP) && (INA226_UART_BTLDR_COMM_ENABLED) */

/**
* \addtogroup group_bootloader
* @{
*/

#if defined(CYDEV_BOOTLOADER_IO_COMP) && (INA226_BTLDR_COMM_ENABLED)
    #if (INA226_SCB_MODE_UNCONFIG_CONST_CFG)
        /* Bootloader physical layer functions */
        void INA226_CyBtldrCommStart(void);
        void INA226_CyBtldrCommStop (void);
        void INA226_CyBtldrCommReset(void);
        cystatus INA226_CyBtldrCommRead       (uint8 pData[], uint16 size, uint16 * count, uint8 timeOut);
        cystatus INA226_CyBtldrCommWrite(const uint8 pData[], uint16 size, uint16 * count, uint8 timeOut);
    #endif /* (INA226_SCB_MODE_UNCONFIG_CONST_CFG) */

    /* Map SCB specific bootloader communication APIs to common APIs */
    #if (CYDEV_BOOTLOADER_IO_COMP == CyBtldr_INA226)
        #define CyBtldrCommStart    INA226_CyBtldrCommStart
        #define CyBtldrCommStop     INA226_CyBtldrCommStop
        #define CyBtldrCommReset    INA226_CyBtldrCommReset
        #define CyBtldrCommWrite    INA226_CyBtldrCommWrite
        #define CyBtldrCommRead     INA226_CyBtldrCommRead
    #endif /* (CYDEV_BOOTLOADER_IO_COMP == CyBtldr_INA226) */

#endif /* defined(CYDEV_BOOTLOADER_IO_COMP) && (INA226_BTLDR_COMM_ENABLED) */

/** @} group_bootloader */

/***************************************
*           API Constants
***************************************/

/* Timeout unit in milliseconds */
#define INA226_WAIT_1_MS  (1u)

/* Return number of bytes to copy into bootloader buffer */
#define INA226_BYTES_TO_COPY(actBufSize, bufSize) \
                            ( ((uint32)(actBufSize) < (uint32)(bufSize)) ? \
                                ((uint32) (actBufSize)) : ((uint32) (bufSize)) )

/* Size of Read/Write buffers for I2C bootloader  */
#define INA226_I2C_BTLDR_SIZEOF_READ_BUFFER   (64u)
#define INA226_I2C_BTLDR_SIZEOF_WRITE_BUFFER  (64u)

/* Byte to byte time interval: calculated basing on current component
* data rate configuration, can be defined in project if required.
*/
#ifndef INA226_SPI_BYTE_TO_BYTE
    #define INA226_SPI_BYTE_TO_BYTE   (160u)
#endif

/* Byte to byte time interval: calculated basing on current component
* baud rate configuration, can be defined in the project if required.
*/
#ifndef INA226_UART_BYTE_TO_BYTE
    #define INA226_UART_BYTE_TO_BYTE  (2500u)
#endif /* INA226_UART_BYTE_TO_BYTE */

#endif /* (CY_SCB_BOOT_INA226_H) */


/* [] END OF FILE */
