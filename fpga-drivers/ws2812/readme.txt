INTRODUCTION
    The four pins on the WS28 card each control a string
of up to 64 RGBW or RGB WS2812 or SK6812 addressable LEDs.
Adafruit Industries makes the WS2812 popular under the
name 'neopixel'.


HARDWARE
    Signals to drive the WS2812 LED chains are available
on all four pins. However the pins are at 3.3 Volts and
and the WS2812 required at least 4 Volts to drive the
data input.  You will need to add a 3.3 to 5 volt level
translator in the data path.  You have the option to
invert the output lines in case your level translator
also inverts the input.  A 74HC14 is a good choice for
an inverting level translator.


RESOURCES
 led : Which string and the hex value to write to that
LED string.  Use three bytes per LED for RGB and four
bytes per LED for RGBW LEDs.  The first parameter is
which of the four LED strings to address and the second
parameter is a sequence of hex characters to write to
the string.  The number of hex characters must be even
since LEDs have 3 or 4 bytes of LED data.
 config : Set this value to a 1 to invert the outputs.
The default value is to not invert the outputs.


EXAMPLES
dpset ws28 config 1
dpset ws28 led 1 ffffffffffffffffffffffffffffffffffffffffffffffff
dpset ws28 led 4 111111111111111111111111111111111111111111111111
dpset ws28 led 4 00ff00ff00000000ff00ff00ff00000000ff00ff00ff0000


