# =============================================================================
# ax7a200b_board.xdc - ALINX AX7A200B (AC7A200 core, XC7A200T-2FBG484I) board I/O of ax7a200b_top
# plus the PCIe refclk / PERST# pins.  GT LOCs and clock groups: ax7a200b_cdc.xdc (implementation only).
# Pins from the ALINX AX7A200B User Manual Rev 1.0 (sections 2.3, 3.4, 3.9, 3.13, 3.14) and the
# package-pin query of Vivado 2025.2 (STAND/AX7A200B_ARCHITECTURE.md 3.1).  Lines marked
# "verify schematic" must be compared with the carrier schematic before the first board run.
# =============================================================================

# ---------------------------------------------------------------------------
# AC7A200 SYS_CLK 200 MHz, bank 34 (VCCO 1.5 V, DDR3 bank; DDR3 is not used) -> free_clk / link_clk
# ---------------------------------------------------------------------------
create_clock -name free_clk -period 5.000 [get_ports free_clk_p]
set_property PACKAGE_PIN R4          [get_ports free_clk_p]     ;# SYS_CLK_P, IO_L13P_T2_MRCC_34
set_property PACKAGE_PIN T4          [get_ports free_clk_n]     ;# SYS_CLK_N
set_property IOSTANDARD DIFF_SSTL15  [get_ports {free_clk_p free_clk_n}]

# ---------------------------------------------------------------------------
# PCIe reference clock 100 MHz from the slot (adapter Si52142, AC-coupled on the carrier): MGTREFCLK1_216
# ---------------------------------------------------------------------------
create_clock -name sys_clk -period 10.000 [get_ports sys_clk_p]
set_property PACKAGE_PIN F10 [get_ports sys_clk_p]
set_property PACKAGE_PIN E10 [get_ports sys_clk_n]

# ---------------------------------------------------------------------------
# PERST# to the M.2 SSD: PCIE_PERST of the carrier = edge finger A11 (via R102) -> FPGA L16
# (bank 15, VCCIO 3.3 V, IO_L23P_T3_FOE_B_15; FOE_B is a BPI-only function, the board boots from SPI).
# ALINX AX7A200B manual (CN), table "PCIe x2 FPGA pins": PCIE_PERST = L16 (an input for endpoint designs;
# here the Root Port DRIVES it).  Path: L16 -> A11 -> adapter slot PERST_HOST# -> jumper XP1
# (PERST_M2# <-> PERST_HOST#) -> M.2 PERST#.  No wire needed.
# Fallback if the carrier had a one-way buffer on this net: J11 pin 3 (G21) + wire to XP1 PERST_M2#.
# ---------------------------------------------------------------------------
set_property PACKAGE_PIN L16      [get_ports pcie_perst_n]      ;# PCIE_PERST (edge A11)
set_property IOSTANDARD LVCMOS33  [get_ports pcie_perst_n]
set_property DRIVE 8              [get_ports pcie_perst_n]      ;# used only with PERST_DRIVE = 1 (inout port)
set_property SLEW SLOW            [get_ports pcie_perst_n]

# ---------------------------------------------------------------------------
# Keys (active low) and LEDs (active low), 3.3 V banks
# ---------------------------------------------------------------------------
set_property PACKAGE_PIN F15 [get_ports rst_key_n]              ;# RESET key
set_property PACKAGE_PIN L19 [get_ports {key_n[0]}]             ;# KEY1 START
set_property PACKAGE_PIN L20 [get_ports {key_n[1]}]             ;# KEY2 STOP
set_property PACKAGE_PIN K17 [get_ports {key_n[2]}]             ;# KEY3 LED page 1
set_property PACKAGE_PIN J17 [get_ports {key_n[3]}]             ;# KEY4 LED page 2
set_property IOSTANDARD LVCMOS33 [get_ports {rst_key_n key_n[*]}]

set_property PACKAGE_PIN L13 [get_ports {led_n[0]}]             ;# LED1
set_property PACKAGE_PIN M13 [get_ports {led_n[1]}]             ;# LED2
set_property PACKAGE_PIN K14 [get_ports {led_n[2]}]             ;# LED3
set_property PACKAGE_PIN K13 [get_ports {led_n[3]}]             ;# LED4
set_property IOSTANDARD LVCMOS33 [get_ports {led_n[*]}]
set_property DRIVE 8             [get_ports {led_n[*]}]
set_property SLEW SLOW           [get_ports {led_n[*]}]

# USB-UART: FPGA TX -> UART1_TXD (L15).  (UART1_RXD = L14, not used yet)
set_property PACKAGE_PIN L15     [get_ports uart_tx]
set_property IOSTANDARD LVCMOS33 [get_ports uart_tx]

# ---------------------------------------------------------------------------
# Asynchronous / static user I/O
# ---------------------------------------------------------------------------
set_false_path -from [get_ports {rst_key_n key_n[*] pcie_perst_n}]
set_false_path -to   [get_ports {led_n[*] pcie_perst_n uart_tx}]
set_false_path -to   [get_cells -hier -filter {ASYNC_REG == TRUE}]

# ---------------------------------------------------------------------------
# Configuration: bank 0 at 3.3 V, QSPI flash (N25Q128 / 128 Mbit) in x4 mode
# ---------------------------------------------------------------------------
set_property CFGBVS VCCO                       [current_design]
set_property CONFIG_VOLTAGE 3.3                [current_design]
set_property BITSTREAM.GENERAL.COMPRESS TRUE   [current_design]
set_property BITSTREAM.CONFIG.SPI_BUSWIDTH 4   [current_design]
set_property BITSTREAM.CONFIG.CONFIGRATE 50    [current_design]
