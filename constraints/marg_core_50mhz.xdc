# Default low-power timing constraint for xc7z020clg400-2.
# This serial arithmetic baseline targets 50 MHz (20 ns).  Add physical pin and
# I/O-standard constraints in the board-specific wrapper rather than here.
create_clock -name marg_clk -period 20.000 [get_ports clk]
