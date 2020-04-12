Welcome to the respiArt!

respiArt - extremely simply and cheap ventilator with advanced automatic modes.  Current state and functions: https://www.youtube.com/watch?v=BUc_lD-wEiQ

CMV – Continous Mandatory Ventilation SIMV-PC – Synchronized Intermittent Mandatory Ventilation – Pressure Controlled PRVC – Pressure Regulated Volume Controlled

Assumptions:
- very simply mechanics with AMBU bag. AMBU provides valves and mixture with oxygen
- arduino nano control
- driven by stepper motor Nema 17 + DRV8255 driver
- single point measurement of pressure
- function of warming and humidification of inspired air
- 4-6 x 18650 battery power backup
- setting on three potentiometers: 

a) frequency 10-20 breath/min. Set to minimum to activate automatic assist mode with pressure trigger 

b) volume of single breath [mL] 

c) inspiration pressure - maintained by control of stepper speed by actual pressure measurement

- alarm LEDs: 

a) in assist mode - trigger not activated. Then mandatory breath is triggered 

b) set volume not achieved - due to set cycle time and set pressure limit; inspiration is stopped go to expiration phase. 

c) set pressure is not achieved during cycle for set volume - due to lack of resistance

Parameters and pressure history displayed on the small OLED display
