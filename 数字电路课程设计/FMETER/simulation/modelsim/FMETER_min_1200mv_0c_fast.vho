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

-- DATE "07/13/2020 15:32:03"

-- 
-- Device: Altera EP4CE6E22C8 Package TQFP144
-- 

-- 
-- This VHDL file should be used for ModelSim-Altera (VHDL) only
-- 

LIBRARY CYCLONEIVE;
LIBRARY IEEE;
USE CYCLONEIVE.CYCLONEIVE_COMPONENTS.ALL;
USE IEEE.STD_LOGIC_1164.ALL;

ENTITY 	LATCH4 IS
    PORT (
	le : IN std_logic;
	d : IN std_logic_vector(3 DOWNTO 0);
	q : OUT std_logic_vector(3 DOWNTO 0)
	);
END LATCH4;

-- Design Ports Information
-- q[0]	=>  Location: PIN_143,	 I/O Standard: 2.5 V,	 Current Strength: Default
-- q[1]	=>  Location: PIN_142,	 I/O Standard: 2.5 V,	 Current Strength: Default
-- q[2]	=>  Location: PIN_10,	 I/O Standard: 2.5 V,	 Current Strength: Default
-- q[3]	=>  Location: PIN_7,	 I/O Standard: 2.5 V,	 Current Strength: Default
-- d[0]	=>  Location: PIN_144,	 I/O Standard: 2.5 V,	 Current Strength: Default
-- le	=>  Location: PIN_23,	 I/O Standard: 2.5 V,	 Current Strength: Default
-- d[1]	=>  Location: PIN_1,	 I/O Standard: 2.5 V,	 Current Strength: Default
-- d[2]	=>  Location: PIN_2,	 I/O Standard: 2.5 V,	 Current Strength: Default
-- d[3]	=>  Location: PIN_3,	 I/O Standard: 2.5 V,	 Current Strength: Default


ARCHITECTURE structure OF LATCH4 IS
SIGNAL gnd : std_logic := '0';
SIGNAL vcc : std_logic := '1';
SIGNAL unknown : std_logic := 'X';
SIGNAL devoe : std_logic := '1';
SIGNAL devclrn : std_logic := '1';
SIGNAL devpor : std_logic := '1';
SIGNAL ww_devoe : std_logic;
SIGNAL ww_devclrn : std_logic;
SIGNAL ww_devpor : std_logic;
SIGNAL ww_le : std_logic;
SIGNAL ww_d : std_logic_vector(3 DOWNTO 0);
SIGNAL ww_q : std_logic_vector(3 DOWNTO 0);
SIGNAL \le~inputclkctrl_INCLK_bus\ : std_logic_vector(3 DOWNTO 0);
SIGNAL \q[0]~output_o\ : std_logic;
SIGNAL \q[1]~output_o\ : std_logic;
SIGNAL \q[2]~output_o\ : std_logic;
SIGNAL \q[3]~output_o\ : std_logic;
SIGNAL \d[0]~input_o\ : std_logic;
SIGNAL \le~input_o\ : std_logic;
SIGNAL \le~inputclkctrl_outclk\ : std_logic;
SIGNAL \q[0]$latch~combout\ : std_logic;
SIGNAL \d[1]~input_o\ : std_logic;
SIGNAL \q[1]$latch~combout\ : std_logic;
SIGNAL \d[2]~input_o\ : std_logic;
SIGNAL \q[2]$latch~combout\ : std_logic;
SIGNAL \d[3]~input_o\ : std_logic;
SIGNAL \q[3]$latch~combout\ : std_logic;

BEGIN

ww_le <= le;
ww_d <= d;
q <= ww_q;
ww_devoe <= devoe;
ww_devclrn <= devclrn;
ww_devpor <= devpor;

\le~inputclkctrl_INCLK_bus\ <= (vcc & vcc & vcc & \le~input_o\);

-- Location: IOOBUF_X1_Y24_N2
\q[0]~output\ : cycloneive_io_obuf
-- pragma translate_off
GENERIC MAP (
	bus_hold => "false",
	open_drain_output => "false")
-- pragma translate_on
PORT MAP (
	i => \q[0]$latch~combout\,
	devoe => ww_devoe,
	o => \q[0]~output_o\);

-- Location: IOOBUF_X3_Y24_N23
\q[1]~output\ : cycloneive_io_obuf
-- pragma translate_off
GENERIC MAP (
	bus_hold => "false",
	open_drain_output => "false")
-- pragma translate_on
PORT MAP (
	i => \q[1]$latch~combout\,
	devoe => ww_devoe,
	o => \q[1]~output_o\);

-- Location: IOOBUF_X0_Y18_N16
\q[2]~output\ : cycloneive_io_obuf
-- pragma translate_off
GENERIC MAP (
	bus_hold => "false",
	open_drain_output => "false")
-- pragma translate_on
PORT MAP (
	i => \q[2]$latch~combout\,
	devoe => ww_devoe,
	o => \q[2]~output_o\);

-- Location: IOOBUF_X0_Y21_N9
\q[3]~output\ : cycloneive_io_obuf
-- pragma translate_off
GENERIC MAP (
	bus_hold => "false",
	open_drain_output => "false")
-- pragma translate_on
PORT MAP (
	i => \q[3]$latch~combout\,
	devoe => ww_devoe,
	o => \q[3]~output_o\);

-- Location: IOIBUF_X1_Y24_N8
\d[0]~input\ : cycloneive_io_ibuf
-- pragma translate_off
GENERIC MAP (
	bus_hold => "false",
	simulate_z_as => "z")
-- pragma translate_on
PORT MAP (
	i => ww_d(0),
	o => \d[0]~input_o\);

-- Location: IOIBUF_X0_Y11_N8
\le~input\ : cycloneive_io_ibuf
-- pragma translate_off
GENERIC MAP (
	bus_hold => "false",
	simulate_z_as => "z")
-- pragma translate_on
PORT MAP (
	i => ww_le,
	o => \le~input_o\);

-- Location: CLKCTRL_G2
\le~inputclkctrl\ : cycloneive_clkctrl
-- pragma translate_off
GENERIC MAP (
	clock_type => "global clock",
	ena_register_mode => "none")
-- pragma translate_on
PORT MAP (
	inclk => \le~inputclkctrl_INCLK_bus\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	outclk => \le~inputclkctrl_outclk\);

-- Location: LCCOMB_X2_Y23_N0
\q[0]$latch\ : cycloneive_lcell_comb
-- Equation(s):
-- \q[0]$latch~combout\ = (GLOBAL(\le~inputclkctrl_outclk\) & (\d[0]~input_o\)) # (!GLOBAL(\le~inputclkctrl_outclk\) & ((\q[0]$latch~combout\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1100111111000000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	datab => \d[0]~input_o\,
	datac => \le~inputclkctrl_outclk\,
	datad => \q[0]$latch~combout\,
	combout => \q[0]$latch~combout\);

-- Location: IOIBUF_X0_Y23_N1
\d[1]~input\ : cycloneive_io_ibuf
-- pragma translate_off
GENERIC MAP (
	bus_hold => "false",
	simulate_z_as => "z")
-- pragma translate_on
PORT MAP (
	i => ww_d(1),
	o => \d[1]~input_o\);

-- Location: LCCOMB_X3_Y23_N0
\q[1]$latch\ : cycloneive_lcell_comb
-- Equation(s):
-- \q[1]$latch~combout\ = (GLOBAL(\le~inputclkctrl_outclk\) & (\d[1]~input_o\)) # (!GLOBAL(\le~inputclkctrl_outclk\) & ((\q[1]$latch~combout\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1010111110100000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \d[1]~input_o\,
	datac => \le~inputclkctrl_outclk\,
	datad => \q[1]$latch~combout\,
	combout => \q[1]$latch~combout\);

-- Location: IOIBUF_X0_Y23_N8
\d[2]~input\ : cycloneive_io_ibuf
-- pragma translate_off
GENERIC MAP (
	bus_hold => "false",
	simulate_z_as => "z")
-- pragma translate_on
PORT MAP (
	i => ww_d(2),
	o => \d[2]~input_o\);

-- Location: LCCOMB_X1_Y23_N24
\q[2]$latch\ : cycloneive_lcell_comb
-- Equation(s):
-- \q[2]$latch~combout\ = (GLOBAL(\le~inputclkctrl_outclk\) & (\d[2]~input_o\)) # (!GLOBAL(\le~inputclkctrl_outclk\) & ((\q[2]$latch~combout\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1010111110100000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \d[2]~input_o\,
	datac => \le~inputclkctrl_outclk\,
	datad => \q[2]$latch~combout\,
	combout => \q[2]$latch~combout\);

-- Location: IOIBUF_X0_Y23_N15
\d[3]~input\ : cycloneive_io_ibuf
-- pragma translate_off
GENERIC MAP (
	bus_hold => "false",
	simulate_z_as => "z")
-- pragma translate_on
PORT MAP (
	i => ww_d(3),
	o => \d[3]~input_o\);

-- Location: LCCOMB_X1_Y23_N10
\q[3]$latch\ : cycloneive_lcell_comb
-- Equation(s):
-- \q[3]$latch~combout\ = (GLOBAL(\le~inputclkctrl_outclk\) & (\d[3]~input_o\)) # (!GLOBAL(\le~inputclkctrl_outclk\) & ((\q[3]$latch~combout\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1100111111000000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	datab => \d[3]~input_o\,
	datac => \le~inputclkctrl_outclk\,
	datad => \q[3]$latch~combout\,
	combout => \q[3]$latch~combout\);

ww_q(0) <= \q[0]~output_o\;

ww_q(1) <= \q[1]~output_o\;

ww_q(2) <= \q[2]~output_o\;

ww_q(3) <= \q[3]~output_o\;
END structure;


