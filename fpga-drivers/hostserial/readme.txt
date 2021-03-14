============================================================

DESCRIPTION
    Alternate interface between the host the Baseboard.


HARDWARE
    The FPGA Tx line _to_ the host is on the first FPGA pin.
The Rx line for data _from_ the host is on the second FPGA pin.
The third and fourth FPGA pins are unused and should not be
used.  The system default to using the USB interface.  To use
the serial interface set it to enabled with the correct baud
rate.  Only one interface, USB or serial, can be enabled at a
time.


RESOURCES
    config: the baud rate and enable flag.
The baud rate must be one of 460800, 230400, 153600, or 115200.
The enable flag is either 'e' or 'd' to enable or disable the
host serial interface.
    The enumerator controls access to the host.  After enabling
the host serial interface be sure to set the enumerator 'port'
to the new serial port.


EXAMPLES
    Switch from the FTDI/USB interface to the Tx/Rx interface
at 115200 baud.
    dpset hostserial config 115200 e
    dpset enumerator port /dev/ttyS0


NOTES
    The host serial interface has a 1K buffer.  When this buffer
overflows a packet is sent to the host and an error log message
is generated.  ANY buffer overflow is probably and indication 
that your link is overloaded and you should consider configuring
your system to use less bandwidth.

