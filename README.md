# OpenPedalFX
This is the firmware for an open source digital guitar pedal. It is based on an STM32F0 arm cortex M0 cpu.

# Hardware
All the audio processing is done in the arm cpu with the build in 12 bit ADC and DAC, no external DAC or DSP chip is used.
The input and output audio signal is buffered with a opamp to provide the correct impedance to the guitar/other pedals and to
correct the levels to the ADC to 0-3.3 volt.

To display pedal settings the pedal uses a 128x64 pixels OLED screen connected on the i2c bus. I havent decided yet on the input controls,
I might use a rotary encoder and buttons or I connect some pots to spare ADC pins and use the absolute values.

# Building

For building you need the arm-none-eabi-gcc toolchain and openocd for programming the microcontroller.

```bash
# Building the test pedal (DigiBuffer)
$ make

# Building a other pedal from the Pedals/ directory
$ make PEDAL=Tremolololo

# Flash the new firmware to the pedal
$ make upload
```
