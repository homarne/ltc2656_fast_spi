# Fast SPI library for LTC2656 8-channel DAC

This code grew out of a project to use and Arduino Due to build an 8-channel LFO for use as moduldations soureces for synthesizers.  The arduino Due was used because it has a sufficienyl high clock rate to do the required calulations for multi-channel LFO generation in real-time.
   
I wanted to use a sample rate high enough to generate a smooth LFO waveform, but low enough to leave pelnty of time for code execution.  I settled on an 8KHz sample rate becuase it delivered a nice sharp LFO waveform (with a fourth order lowpass filter at 2KHz), while leaving a comfortable amount of time on the Arduino Due to do waveform calculations in each interrupt.
 
At an interrupt rate of 8KHz, the avialable time per interupt is ~125 uS.
 
Using the stock Arduino APIs, toggling a select pin and sending three bytes to one DAC channel via SPI takes ~5 uS.  For eight DAC channels, thats 40 uS, or nearly 1/3 of the avialble compute time per interrupt.
 
Optimizations implimented in this library reduce the time to send three bytes to under 1 uS (including chip select).  For eight DAC channels, that's less than 8 uS, i.e. less than 7% of avialable compute time per interupt.

In the current implimentation, the time to update 8 channels is 12 uS, likely due to call overhead; this could be further reduced through judicious use of in-lining.

This code was writen and tested specifically on the Arduino Due.  Milage may vary on other Arduino platforms.
