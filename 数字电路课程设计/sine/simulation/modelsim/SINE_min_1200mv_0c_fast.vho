-- Copyright (C) 1991-2015 Altera Corporation. All rights reserved.
-- Your use of Altera Corporation's design tools, logic functions 
-- and other software and tools, and its AMPP partner logic 
-- functions, and any output files from any of the foregoing 
-- (including device programming or simulation files), and any 
-- associated documentation or information are expressly subject 
-- to the terms and conditions of the Altera Program License 
-- Subscription Agreement, the Altera Quartus II License Agreement,
-- the Altera MegaCore Function License Agreement, or other 
-- applicable license agreement, including, without limitation, 
-- that your use is for the sole purpose of programming logic 
-- devices manufactured by Altera and sold by Altera or its 
-- authorized distributors.  Please refer to the applicable 
-- agreement for further details.

-- VENDOR "Altera"
-- PROGRAM "Quartus II 64-Bit"
-- VERSION "Version 15.0.0 Build 145 04/22/2015 SJ Full Version"

-- DATE "07/14/2020 14:21:14"

-- 
-- Device: Altera EP4CE6E22C8 Package TQFP144
-- 

-- 
-- This VHDL file should be used for ModelSim-Altera (VHDL) only
-- 

LIBRARY ALTERA;
LIBRARY CYCLONEIVE;
LIBRARY IEEE;
USE ALTERA.ALTERA_PRIMITIVES_COMPONENTS.ALL;
USE CYCLONEIVE.CYCLONEIVE_COMPONENTS.ALL;
USE IEEE.STD_LOGIC_1164.ALL;

ENTITY 	SINE IS
    PORT (
	P : OUT std_logic;
	CLK0 : IN std_logic;
	Q : OUT std_logic_vector(7 DOWNTO 0)
	);
END SINE;

-- Design Ports Information
-- P	=>  Location: PIN_86,	 I/O Standard: 2.5 V,	 Current Strength: Default
-- Q[7]	=>  Location: PIN_85,	 I/O Standard: 2.5 V,	 Current Strength: Default
-- Q[6]	=>  Location: PIN_84,	 I/O Standard: 2.5 V,	 Current Strength: Default
-- Q[5]	=>  Location: PIN_83,	 I/O Standard: 2.5 V,	 Current Strength: Default
-- Q[4]	=>  Location: PIN_80,	 I/O Standard: 2.5 V,	 Current Strength: Default
-- Q[3]	=>  Location: PIN_77,	 I/O Standard: 2.5 V,	 Current Strength: Default
-- Q[2]	=>  Location: PIN_76,	 I/O Standard: 2.5 V,	 Current Strength: Default
-- Q[1]	=>  Location: PIN_75,	 I/O Standard: 2.5 V,	 Current Strength: Default
-- Q[0]	=>  Location: PIN_74,	 I/O Standard: 2.5 V,	 Current Strength: Default
-- CLK0	=>  Location: PIN_24,	 I/O Standard: 2.5 V,	 Current Strength: Default


ARCHITECTURE structure OF SINE IS
SIGNAL gnd : std_logic := '0';
SIGNAL vcc : std_logic := '1';
SIGNAL unknown : std_logic := 'X';
SIGNAL devoe : std_logic := '1';
SIGNAL devclrn : std_logic := '1';
SIGNAL devpor : std_logic := '1';
SIGNAL ww_devoe : std_logic;
SIGNAL ww_devclrn : std_logic;
SIGNAL ww_devpor : std_logic;
SIGNAL ww_P : std_logic;
SIGNAL ww_CLK0 : std_logic;
SIGNAL ww_Q : std_logic_vector(7 DOWNTO 0);
SIGNAL \inst2|altpll_component|auto_generated|pll1_INCLK_bus\ : std_logic_vector(1 DOWNTO 0);
SIGNAL \inst2|altpll_component|auto_generated|pll1_CLK_bus\ : std_logic_vector(4 DOWNTO 0);
SIGNAL \inst3|altsyncram_component|auto_generated|ram_block1a0_PORTAADDR_bus\ : std_logic_vector(7 DOWNTO 0);
SIGNAL \inst3|altsyncram_component|auto_generated|ram_block1a0_PORTADATAOUT_bus\ : std_logic_vector(17 DOWNTO 0);
SIGNAL \inst2|altpll_component|auto_generated|wire_pll1_clk[0]~clkctrl_INCLK_bus\ : std_logic_vector(3 DOWNTO 0);
SIGNAL \CLK0~input_o\ : std_logic;
SIGNAL \inst2|altpll_component|auto_generated|wire_pll1_fbout\ : std_logic;
SIGNAL \inst2|altpll_component|auto_generated|wire_pll1_clk[0]~clkctrl_outclk\ : std_logic;
SIGNAL \inst|Q[0]~21_combout\ : std_logic;
SIGNAL \inst|Q[1]~7_combout\ : std_logic;
SIGNAL \inst|Q[1]~8\ : std_logic;
SIGNAL \inst|Q[2]~9_combout\ : std_logic;
SIGNAL \inst|Q[2]~10\ : std_logic;
SIGNAL \inst|Q[3]~11_combout\ : std_logic;
SIGNAL \inst|Q[3]~12\ : std_logic;
SIGNAL \inst|Q[4]~13_combout\ : std_logic;
SIGNAL \inst|Q[4]~14\ : std_logic;
SIGNAL \inst|Q[5]~15_combout\ : std_logic;
SIGNAL \inst|Q[5]~16\ : std_logic;
SIGNAL \inst|Q[6]~17_combout\ : std_logic;
SIGNAL \inst|Q[6]~18\ : std_logic;
SIGNAL \inst|Q[7]~19_combout\ : std_logic;
SIGNAL \inst2|altpll_component|auto_generated|wire_pll1_clk\ : std_logic_vector(4 DOWNTO 0);
SIGNAL \inst3|altsyncram_component|auto_generated|q_a\ : std_logic_vector(7 DOWNTO 0);
SIGNAL \inst|Q\ : std_logic_vector(7 DOWNTO 0);
SIGNAL \inst2|altpll_component|auto_generated|ALT_INV_wire_pll1_clk[0]~clkctrl_outclk\ : std_logic;

BEGIN

P <= ww_P;
ww_CLK0 <= CLK0;
Q <= ww_Q;
ww_devoe <= devoe;
ww_devclrn <= devclrn;
ww_devpor <= devpor;

\inst2|altpll_component|auto_generated|pll1_INCLK_bus\ <= (gnd & \CLK0~input_o\);

\inst2|altpll_component|auto_generated|wire_pll1_clk\(0) <= \inst2|altpll_component|auto_generated|pll1_CLK_bus\(0);
\inst2|altpll_component|auto_generated|wire_pll1_clk\(1) <= \inst2|altpll_component|auto_generated|pll1_CLK_bus\(1);
\inst2|altpll_component|auto_generated|wire_pll1_clk\(2) <= \inst2|altpll_component|auto_generated|pll1_CLK_bus\(2);
\inst2|altpll_component|auto_generated|wire_pll1_clk\(3) <= \inst2|altpll_component|auto_generated|pll1_CLK_bus\(3);
\inst2|altpll_component|auto_generated|wire_pll1_clk\(4) <= \inst2|altpll_component|auto_generated|pll1_CLK_bus\(4);

\inst3|altsyncram_component|auto_generated|ram_block1a0_PORTAADDR_bus\ <= (\inst|Q\(7) & \inst|Q\(6) & \inst|Q\(5) & \inst|Q\(4) & \inst|Q\(3) & \inst|Q\(2) & \inst|Q\(1) & \inst|Q\(0));

\inst3|altsyncram_component|auto_generated|q_a\(0) <= \inst3|altsyncram_component|auto_generated|ram_block1a0_PORTADATAOUT_bus\(0);
\inst3|altsyncram_component|auto_generated|q_a\(1) <= \inst3|altsyncram_component|auto_generated|ram_block1a0_PORTADATAOUT_bus\(1);
\inst3|altsyncram_component|auto_generated|q_a\(2) <= \inst3|altsyncram_component|auto_generated|ram_block1a0_PORTADATAOUT_bus\(2);
\inst3|altsyncram_component|auto_generated|q_a\(3) <= \inst3|altsyncram_component|auto_generated|ram_block1a0_PORTADATAOUT_bus\(3);
\inst3|altsyncram_component|auto_generated|q_a\(4) <= \inst3|altsyncram_component|auto_generated|ram_block1a0_PORTADATAOUT_bus\(4);
\inst3|altsyncram_component|auto_generated|q_a\(5) <= \inst3|altsyncram_component|auto_generated|ram_block1a0_PORTADATAOUT_bus\(5);
\inst3|altsyncram_component|auto_generated|q_a\(6) <= \inst3|altsyncram_component|auto_generated|ram_block1a0_PORTADATAOUT_bus\(6);
\inst3|altsyncram_component|auto_generated|q_a\(7) <= \inst3|altsyncram_component|auto_generated|ram_block1a0_PORTADATAOUT_bus\(7);

\inst2|altpll_component|auto_generated|wire_pll1_clk[0]~clkctrl_INCLK_bus\ <= (vcc & vcc & vcc & \inst2|altpll_component|auto_generated|wire_pll1_clk\(0));
\inst2|altpll_component|auto_generated|ALT_INV_wire_pll1_clk[0]~clkctrl_outclk\ <= NOT \inst2|altpll_component|auto_generated|wire_pll1_clk[0]~clkctrl_outclk\;

-- Location: IOOBUF_X34_Y9_N2
\P~output\ : cycloneive_io_obuf
-- pragma translate_off
GENERIC MAP (
	bus_hold => "false",
	open_drain_output => "false")
-- pragma translate_on
PORT MAP (
	i => \inst2|altpll_component|auto_generated|ALT_INV_wire_pll1_clk[0]~clkctrl_outclk\,
	devoe => ww_devoe,
	o => ww_P);

-- Location: IOOBUF_X34_Y9_N9
\Q[7]~output\ : cycloneive_io_obuf
-- pragma translate_off
GENERIC MAP (
	bus_hold => "false",
	open_drain_output => "false")
-- pragma translate_on
PORT MAP (
	i => \inst3|altsyncram_component|auto_generated|q_a\(7),
	devoe => ww_devoe,
	o => ww_Q(7));

-- Location: IOOBUF_X34_Y9_N16
\Q[6]~output\ : cycloneive_io_obuf
-- pragma translate_off
GENERIC MAP (
	bus_hold => "false",
	open_drain_output => "false")
-- pragma translate_on
PORT MAP (
	i => \inst3|altsyncram_component|auto_generated|q_a\(6),
	devoe => ww_devoe,
	o => ww_Q(6));

-- Location: IOOBUF_X34_Y9_N23
\Q[5]~output\ : cycloneive_io_obuf
-- pragma translate_off
GENERIC MAP (
	bus_hold => "false",
	open_drain_output => "false")
-- pragma translate_on
PORT MAP (
	i => \inst3|altsyncram_component|auto_generated|q_a\(5),
	devoe => ww_devoe,
	o => ww_Q(5));

-- Location: IOOBUF_X34_Y7_N9
\Q[4]~output\ : cycloneive_io_obuf
-- pragma translate_off
GENERIC MAP (
	bus_hold => "false",
	open_drain_output => "false")
-- pragma translate_on
PORT MAP (
	i => \inst3|altsyncram_component|auto_generated|q_a\(4),
	devoe => ww_devoe,
	o => ww_Q(4));

-- Location: IOOBUF_X34_Y4_N16
\Q[3]~output\ : cycloneive_io_obuf
-- pragma translate_off
GENERIC MAP (
	bus_hold => "false",
	open_drain_output => "false")
-- pragma translate_on
PORT MAP (
	i => \inst3|altsyncram_component|auto_generated|q_a\(3),
	devoe => ww_devoe,
	o => ww_Q(3));

-- Location: IOOBUF_X34_Y4_N23
\Q[2]~output\ : cycloneive_io_obuf
-- pragma translate_off
GENERIC MAP (
	bus_hold => "false",
	open_drain_output => "false")
-- pragma translate_on
PORT MAP (
	i => \inst3|altsyncram_component|auto_generated|q_a\(2),
	devoe => ww_devoe,
	o => ww_Q(2));

-- Location: IOOBUF_X34_Y3_N23
\Q[1]~output\ : cycloneive_io_obuf
-- pragma translate_off
GENERIC MAP (
	bus_hold => "false",
	open_drain_output => "false")
-- pragma translate_on
PORT MAP (
	i => \inst3|altsyncram_component|auto_generated|q_a\(1),
	devoe => ww_devoe,
	o => ww_Q(1));

-- Location: IOOBUF_X34_Y2_N16
\Q[0]~output\ : cycloneive_io_obuf
-- pragma translate_off
GENERIC MAP (
	bus_hold => "false",
	open_drain_output => "false")
-- pragma translate_on
PORT MAP (
	i => \inst3|altsyncram_component|auto_generated|q_a\(0),
	devoe => ww_devoe,
	o => ww_Q(0));

-- Location: IOIBUF_X0_Y11_N15
\CLK0~input\ : cycloneive_io_ibuf
-- pragma translate_off
GENERIC MAP (
	bus_hold => "false",
	simulate_z_as => "z")
-- pragma translate_on
PORT MAP (
	i => ww_CLK0,
	o => \CLK0~input_o\);

-- Location: PLL_1
\inst2|altpll_component|auto_generated|pll1\ : cycloneive_pll
-- pragma translate_off
GENERIC MAP (
	auto_settings => "false",
	bandwidth_type => "medium",
	c0_high => 15,
	c0_initial => 1,
	c0_low => 15,
	c0_mode => "even",
	c0_ph => 0,
	c1_high => 0,
	c1_initial => 0,
	c1_low => 0,
	c1_mode => "bypass",
	c1_ph => 0,
	c1_use_casc_in => "off",
	c2_high => 0,
	c2_initial => 0,
	c2_low => 0,
	c2_mode => "bypass",
	c2_ph => 0,
	c2_use_casc_in => "off",
	c3_high => 0,
	c3_initial => 0,
	c3_low => 0,
	c3_mode => "bypass",
	c3_ph => 0,
	c3_use_casc_in => "off",
	c4_high => 0,
	c4_initial => 0,
	c4_low => 0,
	c4_mode => "bypass",
	c4_ph => 0,
	c4_use_casc_in => "off",
	charge_pump_current_bits => 1,
	clk0_counter => "c0",
	clk0_divide_by => 5,
	clk0_duty_cycle => 50,
	clk0_multiply_by => 4,
	clk0_phase_shift => "0",
	clk1_counter => "unused",
	clk1_divide_by => 0,
	clk1_duty_cycle => 50,
	clk1_multiply_by => 0,
	clk1_phase_shift => "0",
	clk2_counter => "unused",
	clk2_divide_by => 0,
	clk2_duty_cycle => 50,
	clk2_multiply_by => 0,
	clk2_phase_shift => "0",
	clk3_counter => "unused",
	clk3_divide_by => 0,
	clk3_duty_cycle => 50,
	clk3_multiply_by => 0,
	clk3_phase_shift => "0",
	clk4_counter => "unused",
	clk4_divide_by => 0,
	clk4_duty_cycle => 50,
	clk4_multiply_by => 0,
	clk4_phase_shift => "0",
	compensate_clock => "clock0",
	inclk0_input_frequency => 40000,
	inclk1_input_frequency => 0,
	loop_filter_c_bits => 0,
	loop_filter_r_bits => 24,
	m => 24,
	m_initial => 1,
	m_ph => 0,
	n => 1,
	operation_mode => "normal",
	pfd_max => 200000,
	pfd_min => 3076,
	pll_compensation_delay => 3267,
	self_reset_on_loss_lock => "off",
	simulation_type => "timing",
	switch_over_type => "auto",
	vco_center => 1538,
	vco_divide_by => 0,
	vco_frequency_control => "auto",
	vco_max => 3333,
	vco_min => 1538,
	vco_multiply_by => 0,
	vco_phase_shift_step => 208,
	vco_post_scale => 2)
-- pragma translate_on
PORT MAP (
	areset => GND,
	fbin => \inst2|altpll_component|auto_generated|wire_pll1_fbout\,
	inclk => \inst2|altpll_component|auto_generated|pll1_INCLK_bus\,
	fbout => \inst2|altpll_component|auto_generated|wire_pll1_fbout\,
	clk => \inst2|altpll_component|auto_generated|pll1_CLK_bus\);

-- Location: CLKCTRL_G3
\inst2|altpll_component|auto_generated|wire_pll1_clk[0]~clkctrl\ : cycloneive_clkctrl
-- pragma translate_off
GENERIC MAP (
	clock_type => "global clock",
	ena_register_mode => "none")
-- pragma translate_on
PORT MAP (
	inclk => \inst2|altpll_component|auto_generated|wire_pll1_clk[0]~clkctrl_INCLK_bus\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	outclk => \inst2|altpll_component|auto_generated|wire_pll1_clk[0]~clkctrl_outclk\);

-- Location: LCCOMB_X28_Y4_N24
\inst|Q[0]~21\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst|Q[0]~21_combout\ = !\inst|Q\(0)

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0000111100001111",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	datac => \inst|Q\(0),
	combout => \inst|Q[0]~21_combout\);

-- Location: FF_X28_Y4_N25
\inst|Q[0]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \inst2|altpll_component|auto_generated|wire_pll1_clk[0]~clkctrl_outclk\,
	d => \inst|Q[0]~21_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst|Q\(0));

-- Location: LCCOMB_X28_Y4_N6
\inst|Q[1]~7\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst|Q[1]~7_combout\ = (\inst|Q\(1) & (\inst|Q\(0) $ (VCC))) # (!\inst|Q\(1) & (\inst|Q\(0) & VCC))
-- \inst|Q[1]~8\ = CARRY((\inst|Q\(1) & \inst|Q\(0)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0110011010001000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst|Q\(1),
	datab => \inst|Q\(0),
	datad => VCC,
	combout => \inst|Q[1]~7_combout\,
	cout => \inst|Q[1]~8\);

-- Location: FF_X28_Y4_N7
\inst|Q[1]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \inst2|altpll_component|auto_generated|wire_pll1_clk[0]~clkctrl_outclk\,
	d => \inst|Q[1]~7_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst|Q\(1));

-- Location: LCCOMB_X28_Y4_N8
\inst|Q[2]~9\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst|Q[2]~9_combout\ = (\inst|Q\(2) & (!\inst|Q[1]~8\)) # (!\inst|Q\(2) & ((\inst|Q[1]~8\) # (GND)))
-- \inst|Q[2]~10\ = CARRY((!\inst|Q[1]~8\) # (!\inst|Q\(2)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0011110000111111",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	datab => \inst|Q\(2),
	datad => VCC,
	cin => \inst|Q[1]~8\,
	combout => \inst|Q[2]~9_combout\,
	cout => \inst|Q[2]~10\);

-- Location: FF_X28_Y4_N9
\inst|Q[2]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \inst2|altpll_component|auto_generated|wire_pll1_clk[0]~clkctrl_outclk\,
	d => \inst|Q[2]~9_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst|Q\(2));

-- Location: LCCOMB_X28_Y4_N10
\inst|Q[3]~11\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst|Q[3]~11_combout\ = (\inst|Q\(3) & (\inst|Q[2]~10\ $ (GND))) # (!\inst|Q\(3) & (!\inst|Q[2]~10\ & VCC))
-- \inst|Q[3]~12\ = CARRY((\inst|Q\(3) & !\inst|Q[2]~10\))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1010010100001010",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst|Q\(3),
	datad => VCC,
	cin => \inst|Q[2]~10\,
	combout => \inst|Q[3]~11_combout\,
	cout => \inst|Q[3]~12\);

-- Location: FF_X28_Y4_N11
\inst|Q[3]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \inst2|altpll_component|auto_generated|wire_pll1_clk[0]~clkctrl_outclk\,
	d => \inst|Q[3]~11_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst|Q\(3));

-- Location: LCCOMB_X28_Y4_N12
\inst|Q[4]~13\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst|Q[4]~13_combout\ = (\inst|Q\(4) & (!\inst|Q[3]~12\)) # (!\inst|Q\(4) & ((\inst|Q[3]~12\) # (GND)))
-- \inst|Q[4]~14\ = CARRY((!\inst|Q[3]~12\) # (!\inst|Q\(4)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0101101001011111",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst|Q\(4),
	datad => VCC,
	cin => \inst|Q[3]~12\,
	combout => \inst|Q[4]~13_combout\,
	cout => \inst|Q[4]~14\);

-- Location: FF_X28_Y4_N13
\inst|Q[4]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \inst2|altpll_component|auto_generated|wire_pll1_clk[0]~clkctrl_outclk\,
	d => \inst|Q[4]~13_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst|Q\(4));

-- Location: LCCOMB_X28_Y4_N14
\inst|Q[5]~15\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst|Q[5]~15_combout\ = (\inst|Q\(5) & (\inst|Q[4]~14\ $ (GND))) # (!\inst|Q\(5) & (!\inst|Q[4]~14\ & VCC))
-- \inst|Q[5]~16\ = CARRY((\inst|Q\(5) & !\inst|Q[4]~14\))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1100001100001100",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	datab => \inst|Q\(5),
	datad => VCC,
	cin => \inst|Q[4]~14\,
	combout => \inst|Q[5]~15_combout\,
	cout => \inst|Q[5]~16\);

-- Location: FF_X28_Y4_N15
\inst|Q[5]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \inst2|altpll_component|auto_generated|wire_pll1_clk[0]~clkctrl_outclk\,
	d => \inst|Q[5]~15_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst|Q\(5));

-- Location: LCCOMB_X28_Y4_N16
\inst|Q[6]~17\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst|Q[6]~17_combout\ = (\inst|Q\(6) & (!\inst|Q[5]~16\)) # (!\inst|Q\(6) & ((\inst|Q[5]~16\) # (GND)))
-- \inst|Q[6]~18\ = CARRY((!\inst|Q[5]~16\) # (!\inst|Q\(6)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0011110000111111",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	datab => \inst|Q\(6),
	datad => VCC,
	cin => \inst|Q[5]~16\,
	combout => \inst|Q[6]~17_combout\,
	cout => \inst|Q[6]~18\);

-- Location: FF_X28_Y4_N17
\inst|Q[6]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \inst2|altpll_component|auto_generated|wire_pll1_clk[0]~clkctrl_outclk\,
	d => \inst|Q[6]~17_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst|Q\(6));

-- Location: LCCOMB_X28_Y4_N18
\inst|Q[7]~19\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst|Q[7]~19_combout\ = \inst|Q[6]~18\ $ (!\inst|Q\(7))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1111000000001111",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	datad => \inst|Q\(7),
	cin => \inst|Q[6]~18\,
	combout => \inst|Q[7]~19_combout\);

-- Location: FF_X28_Y4_N19
\inst|Q[7]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \inst2|altpll_component|auto_generated|wire_pll1_clk[0]~clkctrl_outclk\,
	d => \inst|Q[7]~19_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst|Q\(7));

-- Location: M9K_X27_Y4_N0
\inst3|altsyncram_component|auto_generated|ram_block1a0\ : cycloneive_ram_block
-- pragma translate_off
GENERIC MAP (
	mem_init2 => X"001F00079001D80073001BC006C001A400660018C006000174005A0015C005400144004E0012C004800118004300100003D000EC0038000D80033000C4002E00",
	mem_init1 => X"0B000290009C00250008C002100078001C0006800190005C00150004C001200040000F00034000C0002800090002000070001800050001000030000C00020000400010000000000000000000000000000000000000000000000000000010000400020000C000300010000500018000700020000900028000C00034000F0004000120004C00150005C001900068001C0007800210008C00250009C0029000B0002E000C40033000D80038000EC003D0010000430011800480012C004E0014400540015C005A0017400600018C0066001A4006C001BC0073001D80079001F0007F00208008500220008B0023C00920025400980026C009E0028400A40029C00AA0",
	mem_init0 => X"02B400B0002CC00B6002E000BB002F800C10030C00C60032000CB0033400D00034800D50035C00D90036C00DD0038000E20039000E50039C00E9003AC00EC003B800EF003C400F2003D000F5003D800F7003E000F9003E800FB003EC00FC003F400FD003F800FE003F800FE003F800FF003F800FE003F800FE003F800FD003F400FC003EC00FB003E800F9003E000F7003D800F5003D000F2003C400EF003B800EC003AC00E90039C00E50039000E20038000DD0036C00D90035C00D50034800D00033400CB0032000C60030C00C1002F800BB002E000B6002CC00B0002B400AA0029C00A400284009E0026C00980025400920023C008B00220008500208007F",
	data_interleave_offset_in_bits => 1,
	data_interleave_width_in_bits => 1,
	init_file => "sindat.hex",
	init_file_layout => "port_a",
	logical_ram_name => "ROM:inst3|altsyncram:altsyncram_component|altsyncram_v6r3:auto_generated|ALTSYNCRAM",
	operation_mode => "rom",
	port_a_address_clear => "none",
	port_a_address_width => 8,
	port_a_byte_enable_clock => "none",
	port_a_data_out_clear => "none",
	port_a_data_out_clock => "clock0",
	port_a_data_width => 18,
	port_a_first_address => 0,
	port_a_first_bit_number => 0,
	port_a_last_address => 255,
	port_a_logical_ram_depth => 256,
	port_a_logical_ram_width => 8,
	port_a_read_during_write_mode => "new_data_with_nbe_read",
	port_a_write_enable_clock => "none",
	port_b_address_width => 8,
	port_b_data_width => 18,
	ram_block_type => "M9K")
-- pragma translate_on
PORT MAP (
	portare => VCC,
	clk0 => VCC,
	portaaddr => \inst3|altsyncram_component|auto_generated|ram_block1a0_PORTAADDR_bus\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	portadataout => \inst3|altsyncram_component|auto_generated|ram_block1a0_PORTADATAOUT_bus\);
END structure;


