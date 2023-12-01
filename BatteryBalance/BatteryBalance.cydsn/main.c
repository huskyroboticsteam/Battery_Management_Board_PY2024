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

int main(void)
{
    CyGlobalIntEnable; /* Enable global interrupts. */

    /* Place your initialization/startup code here (e.g. MyInst_Start()) */
    BatteryBalanceInit();
    
    uint32 current;
    uint8 ina226;

    for(;;)
    {
        DataPeriphRead(DATA_ADDRESS, &ina226);
        current = convertValue(ina226);
        
        JettySend(JETTY, current);
        
        CyDelay(100);
    }
}

/* [] END OF FILE */
