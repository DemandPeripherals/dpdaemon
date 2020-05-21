# dpdaemon
User space daemon to control FPGA-based peripherals


DPDAEMON is a set of user space device drivers implemented
as plug-in modules. The daemon starts empty in the sense
that the daemon itself provides only the command line
interface, leaving the real functionality of the drivers
to the set of loadable shared object libraries (plug-ins).
While intended to support the FPGA-based peripherals in a
Demand Peripherals FPGA image, you may find that dpdaemon
has several features you'll find useful for your next
Linux-to-hardware  project:
  - Command line tools to view and set plug-in parameters
  - Simple publish/subscribe mechanism for sensor data
  - All commands and data are printable ASCII over TCP
  - Modular plug-ins (drivers) for easy development
  - No dependencies (excluding libc)
  - Event-driven and C means low CPU/memory footprint
  - GPLv2 License.


<p> &nbsp; </p>

# System Architecture
The following diagram shows the major components in the Demand
Peripherals system. Major components include: daughter cards,
the FPGA card, dpdaemon, and your application. (The FPGA card
supports eight daughter cards although this diagram shows only
three.)
<p align='center'>
  <img src='https://demandperipherals.com/images/arch_v2.svg' width='90%' />
</p>
<p> &nbsp; </p>

**Dpdaemon** (this repository) provides an API and acts as a
multiplexer for packets to and from the FPGA.

**DPCore** is the FPGA part of the peripherals.  DPCore has the
timing and logic needed to drive the electronics on the daughter
cards. For example, the FPGA part of the dc2 peripheral does the
timing for the H-bridges that controls two DC motors. Logic in
the FPGA implements an internal address and data bus and each
peripheral has a set of 8-bit registers which configure the
peripheral. For example, the FPGA peripheral part of dc2 has an
8-bit register that sets the mode (brake, coast, forward, or
reverse) of each motor. Other registers in DC2 set the duty
cycle of the H-bridge FETs and configure the watchdog timer.

The defining feature of DPCore is that 32 FPGA pins are 
organized into eight 4-pin **slots**.  Since they all have four
pins, any DPCore peripheral can be attached to any slot. It
is this flexibility that makes it trivial to change the 
selection or ordering of peripherals in an application.

**Daughter cards** contain the electronic components needed.
Daughter cards, called interface cards on the web site,might
contain the sensors, H-Bridge FETs, or the headers to connect
to servo motors. Each daughter card connects to the FPGA card
over an eight-wire IDC cable connected to one slot. The cable
has power lines and four pins from the FPGA.  Daughter card
designs are released under a Creative Commons license.

<p> &nbsp; </p>
The picture below shows a typical system with a single board
computer, an FPGA card, four daughter cards, and two dpdaemon
managed USB peripherals. 
<p align='center'>
  <img src='https://demandperipherals.com/images/typical.jpg' width='90%'/>
</p>

