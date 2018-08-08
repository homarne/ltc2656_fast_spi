# Fast SPI library for LTC2656 8-channel DAC

This code grew out of a project to build an 8-channel LFO for use as mouldations soureces for synthesizers.
   
An 8KHz sample rate will support voice-grade audio channels up to 4KHz with a high-order filter (4th to 8th order?) or LFOs up to 500Hz with a simple 2nd order filter.
 
At an interrupt rate of 8KHz, the avialable time per interupt is ~125 uS.
 
Using the stock Arduino APIs, toggling a select pin and sending three bytes via SPI to a DAC takes ~5 uS.  For eight DAC channels, thats 40 uS, or nearly 1/3 of the avialble compute time per interrupt.
 
Optimizations implimented in this library reduce the time to send three bytes to under 1 uS (including chip select).  For eight DAC channels, that's less than 8 uS, i.e. less than 7% of avialable compute time per interupt.

In the current implimentation, the time to update 8 channels is 12 uS, likely due to call overhead; this could be further reduced through judicious use of in-lining.
