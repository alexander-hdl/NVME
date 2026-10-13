# =============================================================================
# kc705_board.xdc - KC705 board-level I/O of kc705_top (XC7K325T-2 FFG900):
# SYSCLK, CPU_RESET, GPIO push buttons / DIP switches / LEDs, board configuration properties.
# The FMC / PCIe part (GBTCLK0 refclk, PERST#, GT LOCs) is in kc705_fmc_m2.xdc; the
# clock-domain-crossing groups (implementation only) are in kc705_fmc_m2_cdc.xdc.
# Each constraint exists exactly once in the three files.
#
# Pin numbers are from memory of the KC705 master XDC / UG810 (User I/O tables).
# Every line marked "verify UG810" must be compared with the board file before the first
# board run (the Vivado DRC only checks that the pin exists and the bank voltage is consistent).
# =============================================================================

# ---------------------------------------------------------------------------
# Free-running 200 MHz board oscillator SYSCLK (bank 33)  -> link_clk (link_bringup, PERST# timing)
# (pcie_7x_0 uses IBUFDS_GTE2.O -> BUFG internally, so IBUFDS_GTE2.ODIV2 cannot be used for this)
# ---------------------------------------------------------------------------
create_clock -name free_clk -period 5.000 [get_ports free_clk_p]
set_property PACKAGE_PIN AD12 [get_ports free_clk_p]        ;# SYSCLK_P, verify UG810
set_property PACKAGE_PIN AD11 [get_ports free_clk_n]        ;# SYSCLK_N, verify UG810
set_property IOSTANDARD LVDS  [get_ports {free_clk_p free_clk_n}]

# ---------------------------------------------------------------------------
# CPU_RESET push button (ACTIVE HIGH on the KC705; kc705_top inverts it to board_rst_n)
# ---------------------------------------------------------------------------
set_property PACKAGE_PIN AB7     [get_ports cpu_reset]      ;# CPU_RESET, verify UG810
set_property IOSTANDARD LVCMOS15 [get_ports cpu_reset]

# ---------------------------------------------------------------------------
# GPIO push buttons (active high)
# ---------------------------------------------------------------------------
set_property PACKAGE_PIN G12     [get_ports gpio_sw_c]      ;# GPIO_SW_C (START), verify UG810
set_property IOSTANDARD LVCMOS25 [get_ports gpio_sw_c]
set_property PACKAGE_PIN AB12    [get_ports gpio_sw_s]      ;# GPIO_SW_S (STOP), verify UG810
set_property IOSTANDARD LVCMOS15 [get_ports gpio_sw_s]
set_property PACKAGE_PIN AA12    [get_ports gpio_sw_n]      ;# GPIO_SW_N (LED page status_flags), verify UG810
set_property IOSTANDARD LVCMOS15 [get_ports gpio_sw_n]
set_property PACKAGE_PIN AG5     [get_ports gpio_sw_e]      ;# GPIO_SW_E (LED page link info), verify UG810
set_property IOSTANDARD LVCMOS15 [get_ports gpio_sw_e]
set_property PACKAGE_PIN AC6     [get_ports gpio_sw_w]      ;# GPIO_SW_W (LED page status_error), verify UG810
set_property IOSTANDARD LVCMOS15 [get_ports gpio_sw_w]

# ---------------------------------------------------------------------------
# DIP switches GPIO_DIP_SW[3:0]: [1:0] generator rate, [3:2] total length
# ---------------------------------------------------------------------------
set_property PACKAGE_PIN Y29     [get_ports {gpio_dip_sw[0]}]   ;# GPIO_DIP_SW0, verify UG810
set_property PACKAGE_PIN W29     [get_ports {gpio_dip_sw[1]}]   ;# GPIO_DIP_SW1, verify UG810
set_property PACKAGE_PIN AA28    [get_ports {gpio_dip_sw[2]}]   ;# GPIO_DIP_SW2, verify UG810
set_property PACKAGE_PIN Y28     [get_ports {gpio_dip_sw[3]}]   ;# GPIO_DIP_SW3, verify UG810
set_property IOSTANDARD LVCMOS25 [get_ports {gpio_dip_sw[*]}]

# ---------------------------------------------------------------------------
# GPIO LEDs 0..7
# ---------------------------------------------------------------------------
set_property PACKAGE_PIN AB8     [get_ports {gpio_led[0]}]      ;# GPIO_LED_0, verify UG810
set_property PACKAGE_PIN AA8     [get_ports {gpio_led[1]}]      ;# GPIO_LED_1, verify UG810
set_property PACKAGE_PIN AC9     [get_ports {gpio_led[2]}]      ;# GPIO_LED_2, verify UG810
set_property PACKAGE_PIN AB9     [get_ports {gpio_led[3]}]      ;# GPIO_LED_3, verify UG810
set_property PACKAGE_PIN AE26    [get_ports {gpio_led[4]}]      ;# GPIO_LED_4, verify UG810
set_property PACKAGE_PIN G19     [get_ports {gpio_led[5]}]      ;# GPIO_LED_5, verify UG810
set_property PACKAGE_PIN E18     [get_ports {gpio_led[6]}]      ;# GPIO_LED_6, verify UG810
set_property PACKAGE_PIN F16     [get_ports {gpio_led[7]}]      ;# GPIO_LED_7, verify UG810
set_property IOSTANDARD LVCMOS15 [get_ports {gpio_led[0] gpio_led[1] gpio_led[2] gpio_led[3]}]
set_property IOSTANDARD LVCMOS25 [get_ports {gpio_led[4] gpio_led[5] gpio_led[6] gpio_led[7]}]
set_property DRIVE 8             [get_ports {gpio_led[*]}]
set_property SLEW SLOW           [get_ports {gpio_led[*]}]

# ---------------------------------------------------------------------------
# Timing of the user I/O: buttons / switches are asynchronous (2-FF synchronisers inside),
# LEDs and PERST# are static outputs.
# ---------------------------------------------------------------------------
set_false_path -from [get_ports {cpu_reset gpio_sw_c gpio_sw_s gpio_sw_n gpio_sw_e gpio_sw_w gpio_dip_sw[*]}]
set_false_path -to   [get_ports {gpio_led[*] pcie_perst_n}]

# ---------------------------------------------------------------------------
# Board configuration
# ---------------------------------------------------------------------------
set_property CFGBVS GND                      [current_design]
set_property CONFIG_VOLTAGE 1.8              [current_design]
set_property BITSTREAM.GENERAL.COMPRESS TRUE [current_design]
