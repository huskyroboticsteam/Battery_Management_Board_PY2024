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
    DBG_UART_Init();
    
    uint8_t current;
    uint8_t status;
    uint16_t* id = 0;
    CANPacket* data;

    DBG_UART_UartPutString("Test");
    for(;;)
    {
        status = getCurrent(&current);
        if (status == SUCCESS) {
            //*id = ConstructCANID(0, 0, 0);
            //*data = ConstructCANPacket(*id, (uint8_t)1, &current);
            // SendCANPacket(data);
            //DBG_UART_UartPutChar(data);
            DBG_UART_UartPutString("Test");
            // count++;
        }
        
        CyDelay(100);
    }
}

/* [] END OF FILE */
