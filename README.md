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



