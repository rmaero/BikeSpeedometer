This project is for the class Mediciones Electronicas - FCEIA - UNR
The task was to create a measuring device. We chose to measure bicycle speed with four magnets on the wheel and a hall effect sensor. 

The specifictions for the instrument were
-Speed: 0-40 km/h
-Resolution: 0.1 km/h

We wanted to measure as precise as possible so we chose an ESP32 which has a 64-bit timer that runs at 80 MHz (theoretically, but you get a runtime error when running at that speed so you can use it at 40 Mhz only).
The resolution of this timer is 1/40e6 = 25 ns.
The maximum value it can reach is 2e64 - 1 = 4.6e11 seconds  = 14.6 years! so no need to reset the timer pretty much ever.

