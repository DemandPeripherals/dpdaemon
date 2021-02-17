============================================================
Quad/Octal Serial Output

  The serial output peripheral provides four or eight ports
of low speed serial output.  Data has 8 bits, no parity,
and between one and four stop bits.  The baud rate can be
between 2400 and 38400.
  Each serial port in the FPGA has a buffer size of 32
bytes.  Each port has an additional buffer of 256 bytes
in the serout4 driver.  No characters are written if writing
to a port would overflow the driver buffer.  An error is
returned on buffer overflow.


HARDWARE
   The serial output peripheral can be used with either
of the GPIO cards.  It does not have a dedicated daughter card.


RESOURCES
config : Baud rate as one of 38400, 19200, 9600, 4800, or
    2400, followed by the number of stop bits in the range
    of 1 to 4.  All generated baud rates are within 0.2
    percent of the target rate.  

text : Characters to send to a port.
    Specify the port and the ASCII printable character to send.

hex : Characters to send as 8 bit hex values.
    Specify the port and the hexadecimal values to send.


EXAMPLES
   Set the baud rate to 19200 with two stop bits.  Send 'hello
world' followed by a carriage return and newline to port 1.
         dpset serout4 config 19200 2
         dpset text hello world
         dpset hex 0d 0a


NOTES
   Later releases of this software may include parity, the use
of TCP ports for sending raw characters (with one TCP port
per serial output port), or the addition of a 16 port version.
Please let Demand Peripherals know if any of these features are
critical to your application.


