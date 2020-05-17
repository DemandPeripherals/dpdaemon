============================================================

enumerator

At system startup the enumerator interrogates the FPGA to
get a list of the peripherals built into the FPGA image.
It then does a edloadso on the peripherals listed.
Other driver modules communicate with this driver
using this driver's 'tx_pkt()' routine.  Each driver
that manages an FPGA peripheral must offer a 'rx_pkt'
routine.


RESOURCES
port : The full path to the Linux serial port device.
Changing this causes the old device to be closed and
the new one opened.  The default value of 'device' is
/dev/ttyUSB0.

text : full text of the FPGA build ROM


EXAMPLES
Use USB1 for the serial interface to the FPGA, and
show the contents of the FPGA ROM.

 dpset enumerator port /dev/ttyUSB1
 dpget enumerator text


