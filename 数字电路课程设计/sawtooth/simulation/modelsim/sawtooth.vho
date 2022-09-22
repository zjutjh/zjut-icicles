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

-- DATE "07/14/2020 08:18:20"

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

ENTITY 	sawtooth IS
    PORT (
	Z : OUT std_logic;
	CLK0 : IN std_logic;
	A : OUT std_logic_vector(7 DOWNTO 0)
	);
END sawtooth;

-- Design Ports Information
-- Z	=>  Location: PIN_86,	 I/O Standard: 2.5 V,	 Current Strength: Default
-- A[7]	=>  Location: PIN_85,	 I/O Standard: 2.5 V,	 Current Strength: Default
-- A[6]	=>  Location: PIN_84,	 I/O Standard: 2.5 V,	 Current Strength: Default
-- A[5]	=>  Location: PIN_83,	 I/O Standard: 2.5 V,	 Current Strength: Default
-- A[4]	=>  Location: PIN_80,	 I/O Standard: 2.5 V,	 Current Strength: Default
-- A[3]	=>  Location: PIN_77,	 I/O Standard: 2.5 V,	 Current Strength: Default
-- A[2]	=>  Location: PIN_76,	 I/O Standard: 2.5 V,	 Current Strength: Default
-- A[1]	=>  Location: PIN_75,	 I/O Standard: 2.5 V,	 Current Strength: Default
-- A[0]	=>  Location: PIN_74,	 I/O Standard: 2.5 V,	 Current Strength: Default
-- CLK0	=>  Location: PIN_24,	 I/O Standard: 2.5 V,	 Current Strength: Default


ARCHITECTURE structure OF sawtooth IS
SIGNAL gnd : std_logic := '0';
SIGNAL vcc : std_logic := '1';
SIGNAL unknown : std_logic := 'X';
SIGNAL devoe : std_logic := '1';
SIGNAL devclrn : std_logic := '1';
SIGNAL devpor : std_logic := '1';
SIGNAL ww_devoe : std_logic;
SIGNAL ww_devclrn : std_logic;
SIGNAL ww_devpor : std_logic;
SIGNAL ww_Z : std_logic;
SIGNAL ww_CLK0 : std_logic;
SIGNAL ww_A : std_logic_vector(7 DOWNTO 0);
SIGNAL \inst|Equal0~clkctrl_INCLK_bus\ : std_logic_vector(3 DOWNTO 0);
SIGNAL \Z~output_o\ : std_logic;
SIGNAL \A[7]~output_o\ : std_logic;
SIGNAL \A[6]~output_o\ : std_logic;
SIGNAL \A[5]~output_o\ : std_logic;
SIGNAL \A[4]~output_o\ : std_logic;
SIGNAL \A[3]~output_o\ : std_logic;
SIGNAL \A[2]~output_o\ : std_logic;
SIGNAL \A[1]~output_o\ : std_logic;
SIGNAL \A[0]~output_o\ : std_logic;
SIGNAL \CLK0~input_o\ : std_logic;
SIGNAL \inst|Q~2_combout\ : std_logic;
SIGNAL \inst|Q[0]~feeder_combout\ : std_logic;
SIGNAL \inst|Q~1_combout\ : std_logic;
SIGNAL \inst|Q~0_combout\ : std_logic;
SIGNAL \inst|Equal0~combout\ : std_logic;
SIGNAL \inst|Equal0~clkctrl_outclk\ : std_logic;
SIGNAL \inst2|Q[0]~8_combout\ : std_logic;
SIGNAL \inst2|Q[0]~9\ : std_logic;
SIGNAL \inst2|Q[1]~10_combout\ : std_logic;
SIGNAL \inst2|Q[1]~11\ : std_logic;
SIGNAL \inst2|Q[2]~12_combout\ : std_logic;
SIGNAL \inst2|Q[2]~13\ : std_logic;
SIGNAL \inst2|Q[3]~14_combout\ : std_logic;
SIGNAL \inst2|Q[3]~15\ : std_logic;
SIGNAL \inst2|Q[4]~16_combout\ : std_logic;
SIGNAL \inst2|Q[4]~17\ : std_logic;
SIGNAL \inst2|Q[5]~18_combout\ : std_logic;
SIGNAL \inst2|Q[5]~19\ : std_logic;
SIGNAL \inst2|Q[6]~20_combout\ : std_logic;
SIGNAL \inst2|Q[6]~21\ : std_logic;
SIGNAL \inst2|Q[7]~22_combout\ : std_logic;
SIGNAL \inst2|Q\ : std_logic_vector(7 DOWNTO 0);
SIGNAL \inst|Q\ : std_logic_vector(2 DOWNTO 0);
SIGNAL \inst|ALT_INV_Equal0~combout\ : std_logic;

BEGIN

Z <= ww_Z;
ww_CLK0 <= CLK0;
A <= ww_A;
ww_devoe <= devoe;
ww_devclrn <= devclrn;
ww_devpor <= devpor;

\inst|Equal0~clkctrl_INCLK_bus\ <= (vcc & vcc & vcc & \inst|Equal0~combout\);
\inst|ALT_INV_Equal0~combout\ <= NOT \inst|Equal0~combout\;

-- Location: IOOBUF_X34_Y9_N2
\Z~output\ : cycloneive_io_obuf
-- pragma translate_off
GENERIC MAP (
	bus_hold => "false",
	open_drain_output => "false")
-- pragma translate_on
PORT MAP (
	i => \inst|ALT_INV_Equal0~combout\,
	devoe => ww_devoe,
	o => \Z~output_o\);

-- Location: IOOBUF_X34_Y9_N9
\A[7]~output\ : cycloneive_io_obuf
-- pragma translate_off
GENERIC MAP (
	bus_hold => "false",
	open_drain_output => "false")
-- pragma translate_on
PORT MAP (
	i => \inst2|Q\(7),
	devoe => ww_devoe,
	o => \A[7]~output_o\);

-- Location: IOOBUF_X34_Y9_N16
\A[6]~output\ : cycloneive_io_obuf
-- pragma translate_off
GENERIC MAP (
	bus_hold => "false",
	open_drain_output => "false")
-- pragma translate_on
PORT MAP (
	i => \inst2|Q\(6),
	devoe => ww_devoe,
	o => \A[6]~output_o\);

-- Location: IOOBUF_X34_Y9_N23
\A[5]~output\ : cycloneive_io_obuf
-- pragma translate_off
GENERIC MAP (
	bus_hold => "false",
	open_drain_output => "false")
-- pragma translate_on
PORT MAP (
	i => \inst2|Q\(5),
	devoe => ww_devoe,
	o => \A[5]~output_o\);

-- Location: IOOBUF_X34_Y7_N9
\A[4]~output\ : cycloneive_io_obuf
-- pragma translate_off
GENERIC MAP (
	bus_hold => "false",
	open_drain_output => "false")
-- pragma translate_on
PORT MAP (
	i => \inst2|Q\(4),
	devoe => ww_devoe,
	o => \A[4]~output_o\);

-- Location: IOOBUF_X34_Y4_N16
\A[3]~output\ : cycloneive_io_obuf
-- pragma translate_off
GENERIC MAP (
	bus_hold => "false",
	open_drain_output => "false")
-- pragma translate_on
PORT MAP (
	i => \inst2|Q\(3),
	devoe => ww_devoe,
	o => \A[3]~output_o\);

-- Location: IOOBUF_X34_Y4_N23
\A[2]~output\ : cycloneive_io_obuf
-- pragma translate_off
GENERIC MAP (
	bus_hold => "false",
	open_drain_output => "false")
-- pragma translate_on
PORT MAP (
	i => \inst2|Q\(2),
	devoe => ww_devoe,
	o => \A[2]~output_o\);

-- Location: IOOBUF_X34_Y3_N23
\A[1]~output\ : cycloneive_io_obuf
-- pragma translate_off
GENERIC MAP (
	bus_hold => "false",
	open_drain_output => "false")
-- pragma translate_on
PORT MAP (
	i => \inst2|Q\(1),
	devoe => ww_devoe,
	o => \A[1]~output_o\);

-- Location: IOOBUF_X34_Y2_N16
\A[0]~output\ : cycloneive_io_obuf
-- pragma translate_off
GENERIC MAP (
	bus_hold => "false",
	open_drain_output => "false")
-- pragma translate_on
PORT MAP (
	i => \inst2|Q\(0),
	devoe => ww_devoe,
	o => \A[0]~output_o\);

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

-- Location: LCCOMB_X1_Y11_N4
\inst|Q~2\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst|Q~2_combout\ = (!\inst|Q\(0) & !\inst|Equal0~combout\)

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0000010100000101",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst|Q\(0),
	datac => \inst|Equal0~combout\,
	combout => \inst|Q~2_combout\);

-- Location: LCCOMB_X1_Y11_N30
\inst|Q[0]~feeder\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst|Q[0]~feeder_combout\ = \inst|Q~2_combout\

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1111000011110000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	datac => \inst|Q~2_combout\,
	combout => \inst|Q[0]~feeder_combout\);

-- Location: FF_X1_Y11_N31
\inst|Q[0]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \CLK0~input_o\,
	d => \inst|Q[0]~feeder_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst|Q\(0));

-- Location: LCCOMB_X1_Y11_N8
\inst|Q~1\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst|Q~1_combout\ = (!\inst|Equal0~combout\ & (\inst|Q\(1) $ (\inst|Q\(0))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0000001100110000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	datab => \inst|Equal0~combout\,
	datac => \inst|Q\(1),
	datad => \inst|Q\(0),
	combout => \inst|Q~1_combout\);

-- Location: FF_X1_Y11_N9
\inst|Q[1]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \CLK0~input_o\,
	d => \inst|Q~1_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst|Q\(1));

-- Location: LCCOMB_X1_Y11_N20
\inst|Q~0\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst|Q~0_combout\ = (!\inst|Equal0~combout\ & (\inst|Q\(2) $ (((\inst|Q\(1) & \inst|Q\(0))))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0001001000110000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst|Q\(1),
	datab => \inst|Equal0~combout\,
	datac => \inst|Q\(2),
	datad => \inst|Q\(0),
	combout => \inst|Q~0_combout\);

-- Location: FF_X1_Y11_N21
\inst|Q[2]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \CLK0~input_o\,
	d => \inst|Q~0_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst|Q\(2));

-- Location: LCCOMB_X1_Y11_N24
\inst|Equal0\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst|Equal0~combout\ = LCELL((!\inst|Q\(0) & (!\inst|Q\(1) & \inst|Q\(2))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0000010100000000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst|Q\(0),
	datac => \inst|Q\(1),
	datad => \inst|Q\(2),
	combout => \inst|Equal0~combout\);

-- Location: CLKCTRL_G4
\inst|Equal0~clkctrl\ : cycloneive_clkctrl
-- pragma translate_off
GENERIC MAP (
	clock_type => "global clock",
	ena_register_mode => "none")
-- pragma translate_on
PORT MAP (
	inclk => \inst|Equal0~clkctrl_INCLK_bus\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	outclk => \inst|Equal0~clkctrl_outclk\);

-- Location: LCCOMB_X33_Y7_N10
\inst2|Q[0]~8\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst2|Q[0]~8_combout\ = \inst2|Q\(0) $ (VCC)
-- \inst2|Q[0]~9\ = CARRY(\inst2|Q\(0))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0101010110101010",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst2|Q\(0),
	datad => VCC,
	combout => \inst2|Q[0]~8_combout\,
	cout => \inst2|Q[0]~9\);

-- Location: FF_X33_Y7_N11
\inst2|Q[0]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \inst|Equal0~clkctrl_outclk\,
	d => \inst2|Q[0]~8_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst2|Q\(0));

-- Location: LCCOMB_X33_Y7_N12
\inst2|Q[1]~10\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst2|Q[1]~10_combout\ = (\inst2|Q\(1) & (!\inst2|Q[0]~9\)) # (!\inst2|Q\(1) & ((\inst2|Q[0]~9\) # (GND)))
-- \inst2|Q[1]~11\ = CARRY((!\inst2|Q[0]~9\) # (!\inst2|Q\(1)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0101101001011111",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst2|Q\(1),
	datad => VCC,
	cin => \inst2|Q[0]~9\,
	combout => \inst2|Q[1]~10_combout\,
	cout => \inst2|Q[1]~11\);

-- Location: FF_X33_Y7_N13
\inst2|Q[1]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \inst|Equal0~clkctrl_outclk\,
	d => \inst2|Q[1]~10_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst2|Q\(1));

-- Location: LCCOMB_X33_Y7_N14
\inst2|Q[2]~12\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst2|Q[2]~12_combout\ = (\inst2|Q\(2) & (\inst2|Q[1]~11\ $ (GND))) # (!\inst2|Q\(2) & (!\inst2|Q[1]~11\ & VCC))
-- \inst2|Q[2]~13\ = CARRY((\inst2|Q\(2) & !\inst2|Q[1]~11\))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1100001100001100",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	datab => \inst2|Q\(2),
	datad => VCC,
	cin => \inst2|Q[1]~11\,
	combout => \inst2|Q[2]~12_combout\,
	cout => \inst2|Q[2]~13\);

-- Location: FF_X33_Y7_N15
\inst2|Q[2]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \inst|Equal0~clkctrl_outclk\,
	d => \inst2|Q[2]~12_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst2|Q\(2));

-- Location: LCCOMB_X33_Y7_N16
\inst2|Q[3]~14\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst2|Q[3]~14_combout\ = (\inst2|Q\(3) & (!\inst2|Q[2]~13\)) # (!\inst2|Q\(3) & ((\inst2|Q[2]~13\) # (GND)))
-- \inst2|Q[3]~15\ = CARRY((!\inst2|Q[2]~13\) # (!\inst2|Q\(3)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0011110000111111",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	datab => \inst2|Q\(3),
	datad => VCC,
	cin => \inst2|Q[2]~13\,
	combout => \inst2|Q[3]~14_combout\,
	cout => \inst2|Q[3]~15\);

-- Location: FF_X33_Y7_N17
\inst2|Q[3]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \inst|Equal0~clkctrl_outclk\,
	d => \inst2|Q[3]~14_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst2|Q\(3));

-- Location: LCCOMB_X33_Y7_N18
\inst2|Q[4]~16\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst2|Q[4]~16_combout\ = (\inst2|Q\(4) & (\inst2|Q[3]~15\ $ (GND))) # (!\inst2|Q\(4) & (!\inst2|Q[3]~15\ & VCC))
-- \inst2|Q[4]~17\ = CARRY((\inst2|Q\(4) & !\inst2|Q[3]~15\))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1100001100001100",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	datab => \inst2|Q\(4),
	datad => VCC,
	cin => \inst2|Q[3]~15\,
	combout => \inst2|Q[4]~16_combout\,
	cout => \inst2|Q[4]~17\);

-- Location: FF_X33_Y7_N19
\inst2|Q[4]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \inst|Equal0~clkctrl_outclk\,
	d => \inst2|Q[4]~16_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst2|Q\(4));

-- Location: LCCOMB_X33_Y7_N20
\inst2|Q[5]~18\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst2|Q[5]~18_combout\ = (\inst2|Q\(5) & (!\inst2|Q[4]~17\)) # (!\inst2|Q\(5) & ((\inst2|Q[4]~17\) # (GND)))
-- \inst2|Q[5]~19\ = CARRY((!\inst2|Q[4]~17\) # (!\inst2|Q\(5)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0011110000111111",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	datab => \inst2|Q\(5),
	datad => VCC,
	cin => \inst2|Q[4]~17\,
	combout => \inst2|Q[5]~18_combout\,
	cout => \inst2|Q[5]~19\);

-- Location: FF_X33_Y7_N21
\inst2|Q[5]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \inst|Equal0~clkctrl_outclk\,
	d => \inst2|Q[5]~18_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst2|Q\(5));

-- Location: LCCOMB_X33_Y7_N22
\inst2|Q[6]~20\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst2|Q[6]~20_combout\ = (\inst2|Q\(6) & (\inst2|Q[5]~19\ $ (GND))) # (!\inst2|Q\(6) & (!\inst2|Q[5]~19\ & VCC))
-- \inst2|Q[6]~21\ = CARRY((\inst2|Q\(6) & !\inst2|Q[5]~19\))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1010010100001010",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst2|Q\(6),
	datad => VCC,
	cin => \inst2|Q[5]~19\,
	combout => \inst2|Q[6]~20_combout\,
	cout => \inst2|Q[6]~21\);

-- Location: FF_X33_Y7_N23
\inst2|Q[6]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \inst|Equal0~clkctrl_outclk\,
	d => \inst2|Q[6]~20_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst2|Q\(6));

-- Location: LCCOMB_X33_Y7_N24
\inst2|Q[7]~22\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst2|Q[7]~22_combout\ = \inst2|Q[6]~21\ $ (\inst2|Q\(7))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0000111111110000",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	datad => \inst2|Q\(7),
	cin => \inst2|Q[6]~21\,
	combout => \inst2|Q[7]~22_combout\);

-- Location: FF_X33_Y7_N25
\inst2|Q[7]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \inst|Equal0~clkctrl_outclk\,
	d => \inst2|Q[7]~22_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst2|Q\(7));

ww_Z <= \Z~output_o\;

ww_A(7) <= \A[7]~output_o\;

ww_A(6) <= \A[6]~output_o\;

ww_A(5) <= \A[5]~output_o\;

ww_A(4) <= \A[4]~output_o\;

ww_A(3) <= \A[3]~output_o\;

ww_A(2) <= \A[2]~output_o\;

ww_A(1) <= \A[1]~output_o\;

ww_A(0) <= \A[0]~output_o\;
END structure;


