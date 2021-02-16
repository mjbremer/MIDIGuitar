
# ECE 395 Final Report - Audio-Driven MIDI Controller
Mikey Brewer  
Matt Bremer  
For our project, we chose to create a MIDI controller driven by audio input, such that a user can
connect a guitar or other instrument, and upon playing a note, generate a corresponding MIDI
signal with which to control a synthesizer or other device. The main challenges in solving this
problem were building the circuit and creating minimum-latency and maximum-accuracy
pitch-detection and attack-detection algorithms.  
For our circuit we chose the following components: an STM32F407 discovery board as
the core processor, an AKM5720 ADC for collecting audio, and an FTDI232 UART-to-USB
chip for debugging. These devices were chosen primarily based on availability, although the
STM32F407 was preferable to the supplied ARM M0 processor, as it contained a faster ARM
M4 processor, as well as a built-in I2S interface, and multiple UARTs, which was useful, as this
allowed us to use one UART as the actual MIDI interface and use the other purely for debugging
purposes, specifically transferring full buffers of data so that we could verify the data was what
was expected.  
  
Our first objective was to get the most important pieces of our hardware working
functionally and successfully communicating with each other early on. We connected the
STM32F407, AKM5720, and FTDI232, and once we had initialized the connection between the
ADC and microcontroller, verified that data was being transferred via a high-frequency
oscilloscope. Lastly, we wrote a program that would collect a buffer from the ADC via DMA,
then transfer that data over the debug UART to be collected into a file on our computer. After
graphing this data and seeing the expected waveform (supplied to the ADC via a signal
generator) we were ready to begin working on signal processing.  
  
To begin, we wrote a script in Python to perform online pitch detection. We received
audio from a Scarlett 2i2 audio interface, performed cepstral pitch detection on it, quantized the
determined frequency to the nearest equal temperament frequency, and printed the associated
note names to the screen. Once this program was working to some degree, we began porting the
Python code to C in order to run on the microcontroller. Once the code was ported, we were able
to get similar results from our circuit as we were from our original Python script.
The next step to this project is attack detection, in order to send MIDI signals only when
notes are initially played instead of constantly. There are many methods that people use to go
about this and so finding the most optimal one given what we’re trying to accomplish requires
much thought, and trial and error to follow. Once we have functioning attack detection, we will
have achieved Minimum Viable Product, and can continue to improve the core algorithms and
add more features, such as support for polyphony, physical MIDI support, velocity-sensitivity, a
better form factor by placing the design on a PCB, as well as increasing the performance by
exchanging the microcontroller for a more powerful one, or moving to a DSP chip instead.


