============================================================

HARDWARE
The qtr8 provides eight channels of input from a Pololu QTR-RC
sensor.  The sensors work by charging a capacitor to Vcc (3.3
volts in our case) and monitoring the capacitor discharge.  The
discharge rate depends on the amount of IR light reflected off
a surface and onto a phototransistor.  A sensor is considered
light if the capacitor has discharged below the logic 1 level
of the FPGA and is considered dark if the voltage is still
above the logic 1 level.  Sensitivity is controlled by a delay
in reading the inputs.  The longer the delay, the longer the
cap can discharge.  Sensitiviy is in units of 10us but since
the discharge of a capacitor is exponential the sensitivity is
very non-linear. Sensitivity in the range of 5 to 25 seems to
work fairly well.
    Wire each output of the QTR sensor directly to an FPGA pin.
Be sure to wire the sensor for 3.3 volt operation.


RESOURCES
The quad QTR interface lets you select the update period as
well as the sensitivity of the sensor.  Sensor output is 
available using dpcat.

qtrval : A single hex digit followed by a newline to indicate
the state of the QTR sensors.  A set bit indicated a dark 
sensor and a cleared bit indicates a lighted sensor.

update_period : Update period for the qtrval resource in tens
of milliseconds.  That is, the dpcat command and select() on
qtrval will give a readable file descriptor every update_period
milliseconds.  Setting this to zero turns off all output from
the sensor.  The update period must be between 0 and 150
milliseconds in steps of 10 milliseconds.   That is, valid
values are 0, 10, 20, 30, 40, ... 140, or 150.

sensitivity : The light sensitivity as a number between 1 and
250.  Higher values give the sensor capacitor more time od
discharge down to a logic 0 level.  Sensitivity specified the
number of 10 microsecond intervals to wait before reading the
sensor outputs.  The discharge curve of a capacitor is very
non-linear so the sensitivity has no meaningful units of
measure.


EXAMPLES
Set the sample rate to 50 milliseconds and the sensitivity to
mid-range.

  dpset qtr8 update_period 50
  dpset qtr8 sensitivity 20
  dpcat qtr8 qtrval


