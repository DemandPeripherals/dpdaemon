PWMIN4

The pwmin4 peripheral provides four channels of pulse width 
measurement.  The output has the time (in clock ticks) of 
the low period and the high period for each input.  The user
can set the clock rate to one of fourteen rates with a value 
of zero turning off the peripheral.  A PWM measurement fails
if there are no transitions on any input for 65535 ticks.
Failed measurements are reported as having zero widths.
    Measurements are started by either a rising or falling 
edge on any input.  No measurements are made if all inputs 
are at a DC level.  PWMIN4 can report readings as often as
250 times per second.


HARDWARE
The first channel appears on the BaseBoard connector pin 2
or on pin 10 depending on the slot number for the peripheral.
The second channel is on pin 6 or on pin 14.
   The pwmin4 peripheral is most often used with the GPIO4
card or the GPIO4-ST card, both of which can limit the input
to the maximum of 3.3 volts as required by the FPGA.


RESOURCES
You can specify the PWM clock frequency and request readings
using the clock_rate and counts resources.

clock_rate:
   The clock_rate resource specifies the PWM clock frequency.
The counts described below are given in units of clock ticks
where the frequency of the clock ticks is set by clock_rate.
The clock_rate frequency must be one of the following:
  20000000 -- 20 MHz
  10000000 -- 10 MHz
   5000000 -- 5 MHz
   1000000 -- 1 MHz
    500000 -- 500 KHz
    100000 -- 100 KHz
     50000 --  50 KHz
     10000 --  10 KHz
      5000 --   5 KHz
      1000 --   1 KHz
       500 -- 500 Hz
       100 -- 100 Hz
        50 --  50 Hz
         0 --  Off

counts:
    The low and high times for each input in units of clock
ticks.  Values of zero indicate that the input did not complete
a full cycle during the measurement.  Use the clock rate to 
convert the counts to times.  The format of the output on the
counts resource is:
<p1 low> <p1 high> <p2 low> <p2 high> <p3 low> <p3 high> <p4 low> <p4 high>


EXAMPLES
A quad servo controller updates the servos every 10 milliseconds.
Monitor the servo lines to get the servo position.  An oscilloscope
shows the servo outputs to be arranged as:
1) ____/------\\___________________________________________________..
2) _____________/---------\\_______________________________________..
3) ______________________________/----\\___________________________..
4) ________________________________________/------\\_______________..
The longest time without an edge is from the falling edge of 4 to
the rising edge of 1.  This time could be as long as 3 milliseconds.
Although PWMIN4 reports the low and high tick counts, it _measures_
the ticks from one edge to the next.  This means that our clock rate
should allow no more that 65535 counts in 3 milliseconds.  A clock
rate of 20 MHz meets this requirement.

   dpset pwmin4 clock_rate 20000000
   dpcat pwmin4 counts

A sample output from 'counts' might be:
   370000 30000 360000 40000 380000 20000 370000 30000

The sum of the low and high counts for each input corresponds to a
period of 10 milliseconds, as you would expect.

Note that accurate multichannel RC input can also be performed by
the RCRX peripheral.
