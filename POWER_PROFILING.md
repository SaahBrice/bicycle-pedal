# Controlled algorithm comparison

Branch: `investigate/power-profile`. Target: nRF54L15 DK application core,
nRF Connect SDK 3.2.4.



## Power build

Starts in Sleep. Each button-0 press advances:

1. Sleep.
2. Standard continuous.
3. Simplified continuous.
4. Standard duty-cycled, 200 Hz.
5. Simplified duty-cycled, 200 Hz.
6. Both algorithms duty-cycled, 200 Hz (standard then simplified per sample).
7. Back to Sleep.

Every mode change restarts the input table. 

Scope marker: **P1.11**, active high, with board GND as reference. 