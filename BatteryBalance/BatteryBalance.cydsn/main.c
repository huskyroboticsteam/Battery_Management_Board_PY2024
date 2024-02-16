/* ========================================
 *
 * Copyright YOUR COMPANY, THE YEAR
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF your company.
 *
 * ========================================
*/
#include "project.h"
#include "io.h"
#include <stdint.h>
#include <INA226.h>
#include "./HindsightCAN/CANCommon.h"
#include "./HindsightCAN/CANPacket.h"
#include "./HindsightCAN/Port.h"

int main(void)
{
    CyGlobalIntEnable; /* Enable global interrupts. */

    /* Place your initialization/startup code here (e.g. MyInst_Start()) */
    BatteryBalanceInit();
    init_INA226();
    DBG_UART_Start();
    InitCAN(0x3, 0x3);

    uint8_t current;
    uint8_t status;
    uint16_t* id = 0;
    CANPacket packet;
    CANPacket receivePacket;

    for(;;)
    {
        while (PollAndReceiveCANPacket(&receivePacket) > 2) {}
        
       
        status = getVoltage(&current);
        
        if (status == SUCCESS) {
            // PrintInt(current);
            // DBG_UART_UartPutString("mA\n\r");
            // DBG_UART_UartPutString("Constructing Packet\n\r");
            AssembleTelemetryReportPacket(&packet, 0x2, 0x1, 0x1, current);
            // DBG_UART_UartPutString("Packet Constructed\n\r");
            SendCANPacket(&packet);
        } else {
            DBG_UART_UartPutString("FAIL");   
        }
        
        CyDelay(100); 
    }
}

/* [] END OF FILE */
