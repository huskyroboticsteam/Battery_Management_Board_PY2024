# THIS FILE IS AUTOMATICALLY GENERATED
# Project: C:\Users\tsien\Documents\GitHub\Battery_Management_Board_PY2023\BatteryBalance\BatteryBalance.cydsn\BatteryBalance.cyprj
# Date: Fri, 26 Jan 2024 03:33:48 GMT
#set_units -time ns
create_clock -name {INA226_I2C_SCBCLK(FFB)} -period 625 -waveform {0 312.5} [list [get_pins {ClockBlock/ff_div_2}]]
create_clock -name {DBG_UART_SCBCLK(FFB)} -period 6520.833333333333 -waveform {0 3260.41666666667} [list [get_pins {ClockBlock/ff_div_5}]]
create_clock -name {CyRouted1} -period 20.833333333333332 -waveform {0 10.4166666666667} [list [get_pins {ClockBlock/dsi_in_0}]]
create_clock -name {CyILO} -period 31250 -waveform {0 15625} [list [get_pins {ClockBlock/ilo}]]
create_clock -name {CyLFClk} -period 31250 -waveform {0 15625} [list [get_pins {ClockBlock/lfclk}]]
create_clock -name {CyIMO} -period 20.833333333333332 -waveform {0 10.4166666666667} [list [get_pins {ClockBlock/imo}]]
create_clock -name {CyHFClk} -period 20.833333333333332 -waveform {0 10.4166666666667} [list [get_pins {ClockBlock/hfclk}]]
create_clock -name {CySysClk} -period 20.833333333333332 -waveform {0 10.4166666666667} [list [get_pins {ClockBlock/sysclk}]]
create_generated_clock -name {INA226_I2C_SCBCLK} -source [get_pins {ClockBlock/hfclk}] -edges {1 31 61} [list]
create_generated_clock -name {DBG_UART_SCBCLK} -source [get_pins {ClockBlock/hfclk}] -edges {1 313 627} [list]


# Component constraints for C:\Users\tsien\Documents\GitHub\Battery_Management_Board_PY2023\BatteryBalance\BatteryBalance.cydsn\TopDesign\TopDesign.cysch
# Project: C:\Users\tsien\Documents\GitHub\Battery_Management_Board_PY2023\BatteryBalance\BatteryBalance.cydsn\BatteryBalance.cyprj
# Date: Fri, 26 Jan 2024 03:33:43 GMT
