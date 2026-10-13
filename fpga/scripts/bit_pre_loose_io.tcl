# pre-hook of write_bitstream used by build.tcl (argument loose_io): allow a bitstream while
# I/O standards / pins of the not-yet-assigned user ports are missing.  Bring-up experiments only.
set_property SEVERITY {Warning} [get_drc_checks {NSTD-1 UCIO-1}]
