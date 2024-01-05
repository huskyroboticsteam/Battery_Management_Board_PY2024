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


int main(void)
{
    CyGlobalIntEnable; /* Enable global interrupts. */

    /* Place your initialization/startup code here (e.g. MyInst_Start()) */
    BatteryBalanceInit();
    init_INA226();
    
    uint8_t current;
    uint8_t status;

    for(;;)
    {
        // INA226Read(DATA_ADDRESS, &ina226);
        // current = convertValue(ina226);
        status = getCurrent(&current);
        if (status == SUCCESS) {
            JettySend(JETTY, current);  
        }
        
        CyDelay(100);
    }
}

/* [] END OF FILE */
