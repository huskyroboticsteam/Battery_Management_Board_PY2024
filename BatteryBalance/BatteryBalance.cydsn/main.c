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
#include "./HindsightCAN/CANPacket.h"
#include "./HindsightCAN/Port.h"

int main(void)
{
    CyGlobalIntEnable; /* Enable global interrupts. */

    /* Place your initialization/startup code here (e.g. MyInst_Start()) */
    BatteryBalanceInit();
    init_INA226();
    
    uint8_t current;
    uint8_t status;
    uint16_t count = 0;
    CANPacket* data;

    for(;;)
    {
        status = getCurrent(&current);
        if (status == SUCCESS) {
            *data = ConstructCANPacket(count, sizeof(uint8_t), &current);
            SendCANPacket(data);
            count++;
        }
        
        CyDelay(100);
    }
}

/* [] END OF FILE */
