# Parked: I2C-cockpit firmware (superseded)

`main.c` + `i2c_peripheral.{c,h}` are the Phase-2 firmware built around an I2C
register-file cockpit (Pi5 as I2C master). Two things retired it:

1. The command architecture moved the cockpit to **UART** so the airframe can
   push asynchronous events (docs/Wanderer_Command_Architecture.md §8).
2. Its shared register map (`protocol/i2c_registers.h`) was removed in the
   fresh-start commit, so it no longer compiles.

Kept for reference: the control-loop scaffold, watchdog/latched-fault handling,
and encoder-sampling structure in `main.c` are still good templates for the new
UART-cockpit main. Not part of any build target.
