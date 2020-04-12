Welcome to the respiArt!

respiArt - extremely simply and cheap ventilator with advanced automatic modes.  

Current state (v4) and functions: https://www.youtube.com/watch?v=BUc_lD-wEiQ

General view of drive system working: https://www.youtube.com/watch?v=JRJ65TWT_jk


On the board:

CMV – Continous Mandatory Ventilation

SIMV-PC – Synchronized Intermittent Mandatory Ventilation – Pressure Controlled 

PRVC – Pressure Regulated Volume Controlled

Assumptions:
- very simply mechanics with AMBU bag. AMBU provides valves and mixture with oxygen
- arduino nano control
- driven by stepper motor Nema 17 + DRV8255 driver
- single point measurement of pressure
- function of warming and humidification of inspired air
- 4-6 x 18650 battery power backup; now batteries are just in parallel to main power supply 15V
- setting on three potentiometers: 

   a) frequency 10-20 breath/min. Set to minimum swith from CMV mode to automatic assist mode (SIMV-PC) with pressure trigger 

   b) volume of single breath [mL] 

   c) inspiration pressure - maintained by control of stepper speed by actual pressure measurement

- alarm LEDs: 

   a) in assist mode - trigger not activated. Then mandatory breath is triggered 

   b) set volume not achieved - due to set cycle time and set pressure limit; inspiration is stopped go to expiration phase. 

   c) set pressure is not achieved during cycle for set volume - due to lack of resistance

Parameters and pressure history displayed on the small OLED display.

Parameters can be set (display) only before start. Control pannel is locked during operation.
After activation, in the display there is a pressure history of the very last cycle. Full scale on vertical axis - 60cmH20.
To stop, START/STOP button has to be pressed for at least 2s. 

Data to create copy:

STP file (in STEP format) contains complete assembly.

Software for adruino nano attached.
