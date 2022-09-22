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

-- DATE "07/14/2020 13:32:51"

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

ENTITY 	mydds IS
    PORT (
	L : OUT std_logic;
	CLK0 : IN std_logic;
	LEDA : OUT std_logic_vector(6 DOWNTO 0);
	KEY0 : IN std_logic;
	LEDB : OUT std_logic_vector(6 DOWNTO 0);
	Q : OUT std_logic_vector(7 DOWNTO 0)
	);
END mydds;

-- Design Ports Information
-- L	=>  Location: PIN_86,	 I/O Standard: 2.5 V,	 Current Strength: Default
-- LEDA[6]	=>  Location: PIN_124,	 I/O Standard: 2.5 V,	 Current Strength: Default
-- LEDA[5]	=>  Location: PIN_121,	 I/O Standard: 2.5 V,	 Current Strength: Default
-- LEDA[4]	=>  Location: PIN_120,	 I/O Standard: 2.5 V,	 Current Strength: Default
-- LEDA[3]	=>  Location: PIN_119,	 I/O Standard: 2.5 V,	 Current Strength: Default
-- LEDA[2]	=>  Location: PIN_115,	 I/O Standard: 2.5 V,	 Current Strength: Default
-- LEDA[1]	=>  Location: PIN_114,	 I/O Standard: 2.5 V,	 Current Strength: Default
-- LEDA[0]	=>  Location: PIN_113,	 I/O Standard: 2.5 V,	 Current Strength: Default
-- LEDB[6]	=>  Location: PIN_133,	 I/O Standard: 2.5 V,	 Current Strength: Default
-- LEDB[5]	=>  Location: PIN_132,	 I/O Standard: 2.5 V,	 Current Strength: Default
-- LEDB[4]	=>  Location: PIN_129,	 I/O Standard: 2.5 V,	 Current Strength: Default
-- LEDB[3]	=>  Location: PIN_128,	 I/O Standard: 2.5 V,	 Current Strength: Default
-- LEDB[2]	=>  Location: PIN_127,	 I/O Standard: 2.5 V,	 Current Strength: Default
-- LEDB[1]	=>  Location: PIN_126,	 I/O Standard: 2.5 V,	 Current Strength: Default
-- LEDB[0]	=>  Location: PIN_125,	 I/O Standard: 2.5 V,	 Current Strength: Default
-- Q[7]	=>  Location: PIN_85,	 I/O Standard: 2.5 V,	 Current Strength: Default
-- Q[6]	=>  Location: PIN_84,	 I/O Standard: 2.5 V,	 Current Strength: Default
-- Q[5]	=>  Location: PIN_83,	 I/O Standard: 2.5 V,	 Current Strength: Default
-- Q[4]	=>  Location: PIN_80,	 I/O Standard: 2.5 V,	 Current Strength: Default
-- Q[3]	=>  Location: PIN_77,	 I/O Standard: 2.5 V,	 Current Strength: Default
-- Q[2]	=>  Location: PIN_76,	 I/O Standard: 2.5 V,	 Current Strength: Default
-- Q[1]	=>  Location: PIN_75,	 I/O Standard: 2.5 V,	 Current Strength: Default
-- Q[0]	=>  Location: PIN_74,	 I/O Standard: 2.5 V,	 Current Strength: Default
-- CLK0	=>  Location: PIN_24,	 I/O Standard: 2.5 V,	 Current Strength: Default
-- KEY0	=>  Location: PIN_90,	 I/O Standard: 2.5 V,	 Current Strength: Default


ARCHITECTURE structure OF mydds IS
SIGNAL gnd : std_logic := '0';
SIGNAL vcc : std_logic := '1';
SIGNAL unknown : std_logic := 'X';
SIGNAL devoe : std_logic := '1';
SIGNAL devclrn : std_logic := '1';
SIGNAL devpor : std_logic := '1';
SIGNAL ww_devoe : std_logic;
SIGNAL ww_devclrn : std_logic;
SIGNAL ww_devpor : std_logic;
SIGNAL ww_L : std_logic;
SIGNAL ww_CLK0 : std_logic;
SIGNAL ww_LEDA : std_logic_vector(6 DOWNTO 0);
SIGNAL ww_KEY0 : std_logic;
SIGNAL ww_LEDB : std_logic_vector(6 DOWNTO 0);
SIGNAL ww_Q : std_logic_vector(7 DOWNTO 0);
SIGNAL \inst|altsyncram_component|auto_generated|ram_block1a0_PORTAADDR_bus\ : std_logic_vector(7 DOWNTO 0);
SIGNAL \inst|altsyncram_component|auto_generated|ram_block1a0_PORTADATAOUT_bus\ : std_logic_vector(17 DOWNTO 0);
SIGNAL \inst4~clkctrl_INCLK_bus\ : std_logic_vector(3 DOWNTO 0);
SIGNAL \CLK0~inputclkctrl_INCLK_bus\ : std_logic_vector(3 DOWNTO 0);
SIGNAL \L~output_o\ : std_logic;
SIGNAL \LEDA[6]~output_o\ : std_logic;
SIGNAL \LEDA[5]~output_o\ : std_logic;
SIGNAL \LEDA[4]~output_o\ : std_logic;
SIGNAL \LEDA[3]~output_o\ : std_logic;
SIGNAL \LEDA[2]~output_o\ : std_logic;
SIGNAL \LEDA[1]~output_o\ : std_logic;
SIGNAL \LEDA[0]~output_o\ : std_logic;
SIGNAL \LEDB[6]~output_o\ : std_logic;
SIGNAL \LEDB[5]~output_o\ : std_logic;
SIGNAL \LEDB[4]~output_o\ : std_logic;
SIGNAL \LEDB[3]~output_o\ : std_logic;
SIGNAL \LEDB[2]~output_o\ : std_logic;
SIGNAL \LEDB[1]~output_o\ : std_logic;
SIGNAL \LEDB[0]~output_o\ : std_logic;
SIGNAL \Q[7]~output_o\ : std_logic;
SIGNAL \Q[6]~output_o\ : std_logic;
SIGNAL \Q[5]~output_o\ : std_logic;
SIGNAL \Q[4]~output_o\ : std_logic;
SIGNAL \Q[3]~output_o\ : std_logic;
SIGNAL \Q[2]~output_o\ : std_logic;
SIGNAL \Q[1]~output_o\ : std_logic;
SIGNAL \Q[0]~output_o\ : std_logic;
SIGNAL \CLK0~input_o\ : std_logic;
SIGNAL \CLK0~inputclkctrl_outclk\ : std_logic;
SIGNAL \KEY0~input_o\ : std_logic;
SIGNAL \inst4~feeder_combout\ : std_logic;
SIGNAL \inst4~q\ : std_logic;
SIGNAL \inst4~clkctrl_outclk\ : std_logic;
SIGNAL \inst2|Add0~0_combout\ : std_logic;
SIGNAL \inst2|Add0~3\ : std_logic;
SIGNAL \inst2|Add0~4_combout\ : std_logic;
SIGNAL \inst2|Add0~5\ : std_logic;
SIGNAL \inst2|Add0~6_combout\ : std_logic;
SIGNAL \inst2|Add0~7\ : std_logic;
SIGNAL \inst2|Add0~8_combout\ : std_logic;
SIGNAL \inst2|Equal0~0_combout\ : std_logic;
SIGNAL \inst2|q~0_combout\ : std_logic;
SIGNAL \inst2|Add0~1\ : std_logic;
SIGNAL \inst2|Add0~2_combout\ : std_logic;
SIGNAL \inst2|Mux0~0_combout\ : std_logic;
SIGNAL \inst2|Mux0~1_combout\ : std_logic;
SIGNAL \inst2|Mux0~2_combout\ : std_logic;
SIGNAL \inst2|Mux1~0_combout\ : std_logic;
SIGNAL \inst2|Mux1~1_combout\ : std_logic;
SIGNAL \inst2|Mux1~2_combout\ : std_logic;
SIGNAL \inst2|Mux2~0_combout\ : std_logic;
SIGNAL \inst2|Mux2~1_combout\ : std_logic;
SIGNAL \inst2|Mux3~0_combout\ : std_logic;
SIGNAL \inst2|Mux4~0_combout\ : std_logic;
SIGNAL \inst2|Mux5~0_combout\ : std_logic;
SIGNAL \inst2|Mux6~0_combout\ : std_logic;
SIGNAL \inst2|Mux6~1_combout\ : std_logic;
SIGNAL \inst2|Mux7~0_combout\ : std_logic;
SIGNAL \inst2|Mux7~1_combout\ : std_logic;
SIGNAL \inst2|Mux8~0_combout\ : std_logic;
SIGNAL \inst2|Mux8~1_combout\ : std_logic;
SIGNAL \inst2|Mux9~0_combout\ : std_logic;
SIGNAL \inst2|Mux9~1_combout\ : std_logic;
SIGNAL \inst2|Mux10~0_combout\ : std_logic;
SIGNAL \inst2|Mux11~0_combout\ : std_logic;
SIGNAL \inst2|Mux11~1_combout\ : std_logic;
SIGNAL \inst2|Equal0~1_combout\ : std_logic;
SIGNAL \inst2|Ram0~0_combout\ : std_logic;
SIGNAL \inst2|Ram0~1_combout\ : std_logic;
SIGNAL \inst2|Ram0~3_combout\ : std_logic;
SIGNAL \inst2|Ram0~2_combout\ : std_logic;
SIGNAL \inst2|Ram0~4_combout\ : std_logic;
SIGNAL \inst2|Ram0~6_combout\ : std_logic;
SIGNAL \inst2|Ram0~5_combout\ : std_logic;
SIGNAL \inst2|Ram0~7_combout\ : std_logic;
SIGNAL \inst2|Ram0~8_combout\ : std_logic;
SIGNAL \inst2|Ram0~9_combout\ : std_logic;
SIGNAL \inst2|Ram0~10_combout\ : std_logic;
SIGNAL \inst2|Ram0~11_combout\ : std_logic;
SIGNAL \inst2|Ram0~12_combout\ : std_logic;
SIGNAL \inst2|Ram0~13_combout\ : std_logic;
SIGNAL \inst2|Ram0~14_combout\ : std_logic;
SIGNAL \inst2|Ram0~15_combout\ : std_logic;
SIGNAL \inst2|Ram0~16_combout\ : std_logic;
SIGNAL \inst2|Ram0~17_combout\ : std_logic;
SIGNAL \inst2|Ram0~18_combout\ : std_logic;
SIGNAL \inst2|Ram0~30_combout\ : std_logic;
SIGNAL \inst2|Ram0~31_combout\ : std_logic;
SIGNAL \inst2|Ram0~28_combout\ : std_logic;
SIGNAL \inst2|Ram0~29_combout\ : std_logic;
SIGNAL \inst2|Ram0~19_combout\ : std_logic;
SIGNAL \inst2|Ram0~20_combout\ : std_logic;
SIGNAL \inst2|Ram0~21_combout\ : std_logic;
SIGNAL \inst2|Ram0~22_combout\ : std_logic;
SIGNAL \inst2|Ram0~23_combout\ : std_logic;
SIGNAL \inst2|Ram0~24_combout\ : std_logic;
SIGNAL \inst2|Ram0~25_combout\ : std_logic;
SIGNAL \inst2|Ram0~26_combout\ : std_logic;
SIGNAL \inst2|Ram0~27_combout\ : std_logic;
SIGNAL \inst3|acc[0]~32_combout\ : std_logic;
SIGNAL \inst3|acc[0]~33\ : std_logic;
SIGNAL \inst3|acc[1]~34_combout\ : std_logic;
SIGNAL \inst3|acc[1]~35\ : std_logic;
SIGNAL \inst3|acc[2]~36_combout\ : std_logic;
SIGNAL \inst3|acc[2]~37\ : std_logic;
SIGNAL \inst3|acc[3]~38_combout\ : std_logic;
SIGNAL \inst3|acc[3]~39\ : std_logic;
SIGNAL \inst3|acc[4]~40_combout\ : std_logic;
SIGNAL \inst3|acc[4]~41\ : std_logic;
SIGNAL \inst3|acc[5]~42_combout\ : std_logic;
SIGNAL \inst3|acc[5]~43\ : std_logic;
SIGNAL \inst3|acc[6]~44_combout\ : std_logic;
SIGNAL \inst3|acc[6]~45\ : std_logic;
SIGNAL \inst3|acc[7]~46_combout\ : std_logic;
SIGNAL \inst3|acc[7]~47\ : std_logic;
SIGNAL \inst3|acc[8]~48_combout\ : std_logic;
SIGNAL \inst3|acc[8]~49\ : std_logic;
SIGNAL \inst3|acc[9]~50_combout\ : std_logic;
SIGNAL \inst3|acc[9]~51\ : std_logic;
SIGNAL \inst3|acc[10]~52_combout\ : std_logic;
SIGNAL \inst3|acc[10]~53\ : std_logic;
SIGNAL \inst3|acc[11]~54_combout\ : std_logic;
SIGNAL \inst3|acc[11]~55\ : std_logic;
SIGNAL \inst3|acc[12]~56_combout\ : std_logic;
SIGNAL \inst3|acc[12]~57\ : std_logic;
SIGNAL \inst3|acc[13]~58_combout\ : std_logic;
SIGNAL \inst3|acc[13]~59\ : std_logic;
SIGNAL \inst3|acc[14]~60_combout\ : std_logic;
SIGNAL \inst3|acc[14]~61\ : std_logic;
SIGNAL \inst3|acc[15]~62_combout\ : std_logic;
SIGNAL \inst3|acc[15]~63\ : std_logic;
SIGNAL \inst3|acc[16]~64_combout\ : std_logic;
SIGNAL \inst3|acc[16]~65\ : std_logic;
SIGNAL \inst3|acc[17]~66_combout\ : std_logic;
SIGNAL \inst3|acc[17]~67\ : std_logic;
SIGNAL \inst3|acc[18]~68_combout\ : std_logic;
SIGNAL \inst3|acc[18]~69\ : std_logic;
SIGNAL \inst3|acc[19]~70_combout\ : std_logic;
SIGNAL \inst3|acc[19]~71\ : std_logic;
SIGNAL \inst3|acc[20]~72_combout\ : std_logic;
SIGNAL \inst3|acc[20]~73\ : std_logic;
SIGNAL \inst3|acc[21]~74_combout\ : std_logic;
SIGNAL \inst3|acc[21]~75\ : std_logic;
SIGNAL \inst3|acc[22]~76_combout\ : std_logic;
SIGNAL \inst3|acc[22]~77\ : std_logic;
SIGNAL \inst3|acc[23]~78_combout\ : std_logic;
SIGNAL \inst3|acc[23]~79\ : std_logic;
SIGNAL \inst3|acc[24]~80_combout\ : std_logic;
SIGNAL \inst3|acc[24]~81\ : std_logic;
SIGNAL \inst3|acc[25]~82_combout\ : std_logic;
SIGNAL \inst3|acc[25]~83\ : std_logic;
SIGNAL \inst3|acc[26]~84_combout\ : std_logic;
SIGNAL \inst3|acc[26]~85\ : std_logic;
SIGNAL \inst3|acc[27]~86_combout\ : std_logic;
SIGNAL \inst3|acc[27]~87\ : std_logic;
SIGNAL \inst3|acc[28]~88_combout\ : std_logic;
SIGNAL \inst3|acc[28]~89\ : std_logic;
SIGNAL \inst3|acc[29]~90_combout\ : std_logic;
SIGNAL \inst3|acc[29]~91\ : std_logic;
SIGNAL \inst3|acc[30]~92_combout\ : std_logic;
SIGNAL \inst3|acc[30]~93\ : std_logic;
SIGNAL \inst3|acc[31]~94_combout\ : std_logic;
SIGNAL \inst|altsyncram_component|auto_generated|q_a\ : std_logic_vector(7 DOWNTO 0);
SIGNAL \inst3|acc\ : std_logic_vector(31 DOWNTO 0);
SIGNAL \inst2|q\ : std_logic_vector(4 DOWNTO 0);
SIGNAL \ALT_INV_inst4~clkctrl_outclk\ : std_logic;
SIGNAL \ALT_INV_CLK0~input_o\ : std_logic;
SIGNAL \inst2|ALT_INV_Mux9~1_combout\ : std_logic;
SIGNAL \inst2|ALT_INV_Mux8~1_combout\ : std_logic;
SIGNAL \inst2|ALT_INV_Mux5~0_combout\ : std_logic;

BEGIN

L <= ww_L;
ww_CLK0 <= CLK0;
LEDA <= ww_LEDA;
ww_KEY0 <= KEY0;
LEDB <= ww_LEDB;
Q <= ww_Q;
ww_devoe <= devoe;
ww_devclrn <= devclrn;
ww_devpor <= devpor;

\inst|altsyncram_component|auto_generated|ram_block1a0_PORTAADDR_bus\ <= (\inst3|acc\(31) & \inst3|acc\(30) & \inst3|acc\(29) & \inst3|acc\(28) & \inst3|acc\(27) & \inst3|acc\(26) & \inst3|acc\(25) & \inst3|acc\(24));

\inst|altsyncram_component|auto_generated|q_a\(0) <= \inst|altsyncram_component|auto_generated|ram_block1a0_PORTADATAOUT_bus\(0);
\inst|altsyncram_component|auto_generated|q_a\(1) <= \inst|altsyncram_component|auto_generated|ram_block1a0_PORTADATAOUT_bus\(1);
\inst|altsyncram_component|auto_generated|q_a\(2) <= \inst|altsyncram_component|auto_generated|ram_block1a0_PORTADATAOUT_bus\(2);
\inst|altsyncram_component|auto_generated|q_a\(3) <= \inst|altsyncram_component|auto_generated|ram_block1a0_PORTADATAOUT_bus\(3);
\inst|altsyncram_component|auto_generated|q_a\(4) <= \inst|altsyncram_component|auto_generated|ram_block1a0_PORTADATAOUT_bus\(4);
\inst|altsyncram_component|auto_generated|q_a\(5) <= \inst|altsyncram_component|auto_generated|ram_block1a0_PORTADATAOUT_bus\(5);
\inst|altsyncram_component|auto_generated|q_a\(6) <= \inst|altsyncram_component|auto_generated|ram_block1a0_PORTADATAOUT_bus\(6);
\inst|altsyncram_component|auto_generated|q_a\(7) <= \inst|altsyncram_component|auto_generated|ram_block1a0_PORTADATAOUT_bus\(7);

\inst4~clkctrl_INCLK_bus\ <= (vcc & vcc & vcc & \inst4~q\);

\CLK0~inputclkctrl_INCLK_bus\ <= (vcc & vcc & vcc & \CLK0~input_o\);
\ALT_INV_inst4~clkctrl_outclk\ <= NOT \inst4~clkctrl_outclk\;
\ALT_INV_CLK0~input_o\ <= NOT \CLK0~input_o\;
\inst2|ALT_INV_Mux9~1_combout\ <= NOT \inst2|Mux9~1_combout\;
\inst2|ALT_INV_Mux8~1_combout\ <= NOT \inst2|Mux8~1_combout\;
\inst2|ALT_INV_Mux5~0_combout\ <= NOT \inst2|Mux5~0_combout\;

-- Location: IOOBUF_X34_Y9_N2
\L~output\ : cycloneive_io_obuf
-- pragma translate_off
GENERIC MAP (
	bus_hold => "false",
	open_drain_output => "false")
-- pragma translate_on
PORT MAP (
	i => \ALT_INV_CLK0~input_o\,
	devoe => ww_devoe,
	o => \L~output_o\);

-- Location: IOOBUF_X18_Y24_N16
\LEDA[6]~output\ : cycloneive_io_obuf
-- pragma translate_off
GENERIC MAP (
	bus_hold => "false",
	open_drain_output => "false")
-- pragma translate_on
PORT MAP (
	i => \inst2|Mux0~2_combout\,
	devoe => ww_devoe,
	o => \LEDA[6]~output_o\);

-- Location: IOOBUF_X23_Y24_N16
\LEDA[5]~output\ : cycloneive_io_obuf
-- pragma translate_off
GENERIC MAP (
	bus_hold => "false",
	open_drain_output => "false")
-- pragma translate_on
PORT MAP (
	i => \inst2|Mux1~2_combout\,
	devoe => ww_devoe,
	o => \LEDA[5]~output_o\);

-- Location: IOOBUF_X23_Y24_N9
\LEDA[4]~output\ : cycloneive_io_obuf
-- pragma translate_off
GENERIC MAP (
	bus_hold => "false",
	open_drain_output => "false")
-- pragma translate_on
PORT MAP (
	i => \inst2|Mux2~1_combout\,
	devoe => ww_devoe,
	o => \LEDA[4]~output_o\);

-- Location: IOOBUF_X23_Y24_N2
\LEDA[3]~output\ : cycloneive_io_obuf
-- pragma translate_off
GENERIC MAP (
	bus_hold => "false",
	open_drain_output => "false")
-- pragma translate_on
PORT MAP (
	i => \inst2|Mux3~0_combout\,
	devoe => ww_devoe,
	o => \LEDA[3]~output_o\);

-- Location: IOOBUF_X28_Y24_N23
\LEDA[2]~output\ : cycloneive_io_obuf
-- pragma translate_off
GENERIC MAP (
	bus_hold => "false",
	open_drain_output => "false")
-- pragma translate_on
PORT MAP (
	i => \inst2|Mux4~0_combout\,
	devoe => ww_devoe,
	o => \LEDA[2]~output_o\);

-- Location: IOOBUF_X28_Y24_N16
\LEDA[1]~output\ : cycloneive_io_obuf
-- pragma translate_off
GENERIC MAP (
	bus_hold => "false",
	open_drain_output => "false")
-- pragma translate_on
PORT MAP (
	i => \inst2|ALT_INV_Mux5~0_combout\,
	devoe => ww_devoe,
	o => \LEDA[1]~output_o\);

-- Location: IOOBUF_X28_Y24_N9
\LEDA[0]~output\ : cycloneive_io_obuf
-- pragma translate_off
GENERIC MAP (
	bus_hold => "false",
	open_drain_output => "false")
-- pragma translate_on
PORT MAP (
	i => \inst2|Mux6~1_combout\,
	devoe => ww_devoe,
	o => \LEDA[0]~output_o\);

-- Location: IOOBUF_X13_Y24_N23
\LEDB[6]~output\ : cycloneive_io_obuf
-- pragma translate_off
GENERIC MAP (
	bus_hold => "false",
	open_drain_output => "false")
-- pragma translate_on
PORT MAP (
	i => \inst2|Mux7~1_combout\,
	devoe => ww_devoe,
	o => \LEDB[6]~output_o\);

-- Location: IOOBUF_X13_Y24_N16
\LEDB[5]~output\ : cycloneive_io_obuf
-- pragma translate_off
GENERIC MAP (
	bus_hold => "false",
	open_drain_output => "false")
-- pragma translate_on
PORT MAP (
	i => \inst2|ALT_INV_Mux8~1_combout\,
	devoe => ww_devoe,
	o => \LEDB[5]~output_o\);

-- Location: IOOBUF_X16_Y24_N23
\LEDB[4]~output\ : cycloneive_io_obuf
-- pragma translate_off
GENERIC MAP (
	bus_hold => "false",
	open_drain_output => "false")
-- pragma translate_on
PORT MAP (
	i => \inst2|ALT_INV_Mux9~1_combout\,
	devoe => ww_devoe,
	o => \LEDB[4]~output_o\);

-- Location: IOOBUF_X16_Y24_N16
\LEDB[3]~output\ : cycloneive_io_obuf
-- pragma translate_off
GENERIC MAP (
	bus_hold => "false",
	open_drain_output => "false")
-- pragma translate_on
PORT MAP (
	i => \inst2|Mux10~0_combout\,
	devoe => ww_devoe,
	o => \LEDB[3]~output_o\);

-- Location: IOOBUF_X16_Y24_N9
\LEDB[2]~output\ : cycloneive_io_obuf
-- pragma translate_off
GENERIC MAP (
	bus_hold => "false",
	open_drain_output => "false")
-- pragma translate_on
PORT MAP (
	i => \inst2|Mux11~1_combout\,
	devoe => ww_devoe,
	o => \LEDB[2]~output_o\);

-- Location: IOOBUF_X16_Y24_N2
\LEDB[1]~output\ : cycloneive_io_obuf
-- pragma translate_off
GENERIC MAP (
	bus_hold => "false",
	open_drain_output => "false")
-- pragma translate_on
PORT MAP (
	i => VCC,
	devoe => ww_devoe,
	o => \LEDB[1]~output_o\);

-- Location: IOOBUF_X18_Y24_N23
\LEDB[0]~output\ : cycloneive_io_obuf
-- pragma translate_off
GENERIC MAP (
	bus_hold => "false",
	open_drain_output => "false")
-- pragma translate_on
PORT MAP (
	i => \inst2|Mux10~0_combout\,
	devoe => ww_devoe,
	o => \LEDB[0]~output_o\);

-- Location: IOOBUF_X34_Y9_N9
\Q[7]~output\ : cycloneive_io_obuf
-- pragma translate_off
GENERIC MAP (
	bus_hold => "false",
	open_drain_output => "false")
-- pragma translate_on
PORT MAP (
	i => \inst|altsyncram_component|auto_generated|q_a\(7),
	devoe => ww_devoe,
	o => \Q[7]~output_o\);

-- Location: IOOBUF_X34_Y9_N16
\Q[6]~output\ : cycloneive_io_obuf
-- pragma translate_off
GENERIC MAP (
	bus_hold => "false",
	open_drain_output => "false")
-- pragma translate_on
PORT MAP (
	i => \inst|altsyncram_component|auto_generated|q_a\(6),
	devoe => ww_devoe,
	o => \Q[6]~output_o\);

-- Location: IOOBUF_X34_Y9_N23
\Q[5]~output\ : cycloneive_io_obuf
-- pragma translate_off
GENERIC MAP (
	bus_hold => "false",
	open_drain_output => "false")
-- pragma translate_on
PORT MAP (
	i => \inst|altsyncram_component|auto_generated|q_a\(5),
	devoe => ww_devoe,
	o => \Q[5]~output_o\);

-- Location: IOOBUF_X34_Y7_N9
\Q[4]~output\ : cycloneive_io_obuf
-- pragma translate_off
GENERIC MAP (
	bus_hold => "false",
	open_drain_output => "false")
-- pragma translate_on
PORT MAP (
	i => \inst|altsyncram_component|auto_generated|q_a\(4),
	devoe => ww_devoe,
	o => \Q[4]~output_o\);

-- Location: IOOBUF_X34_Y4_N16
\Q[3]~output\ : cycloneive_io_obuf
-- pragma translate_off
GENERIC MAP (
	bus_hold => "false",
	open_drain_output => "false")
-- pragma translate_on
PORT MAP (
	i => \inst|altsyncram_component|auto_generated|q_a\(3),
	devoe => ww_devoe,
	o => \Q[3]~output_o\);

-- Location: IOOBUF_X34_Y4_N23
\Q[2]~output\ : cycloneive_io_obuf
-- pragma translate_off
GENERIC MAP (
	bus_hold => "false",
	open_drain_output => "false")
-- pragma translate_on
PORT MAP (
	i => \inst|altsyncram_component|auto_generated|q_a\(2),
	devoe => ww_devoe,
	o => \Q[2]~output_o\);

-- Location: IOOBUF_X34_Y3_N23
\Q[1]~output\ : cycloneive_io_obuf
-- pragma translate_off
GENERIC MAP (
	bus_hold => "false",
	open_drain_output => "false")
-- pragma translate_on
PORT MAP (
	i => \inst|altsyncram_component|auto_generated|q_a\(1),
	devoe => ww_devoe,
	o => \Q[1]~output_o\);

-- Location: IOOBUF_X34_Y2_N16
\Q[0]~output\ : cycloneive_io_obuf
-- pragma translate_off
GENERIC MAP (
	bus_hold => "false",
	open_drain_output => "false")
-- pragma translate_on
PORT MAP (
	i => \inst|altsyncram_component|auto_generated|q_a\(0),
	devoe => ww_devoe,
	o => \Q[0]~output_o\);

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

-- Location: CLKCTRL_G4
\CLK0~inputclkctrl\ : cycloneive_clkctrl
-- pragma translate_off
GENERIC MAP (
	clock_type => "global clock",
	ena_register_mode => "none")
-- pragma translate_on
PORT MAP (
	inclk => \CLK0~inputclkctrl_INCLK_bus\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	outclk => \CLK0~inputclkctrl_outclk\);

-- Location: IOIBUF_X34_Y12_N8
\KEY0~input\ : cycloneive_io_ibuf
-- pragma translate_off
GENERIC MAP (
	bus_hold => "false",
	simulate_z_as => "z")
-- pragma translate_on
PORT MAP (
	i => ww_KEY0,
	o => \KEY0~input_o\);

-- Location: LCCOMB_X33_Y12_N2
\inst4~feeder\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst4~feeder_combout\ = \KEY0~input_o\

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1111111100000000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	datad => \KEY0~input_o\,
	combout => \inst4~feeder_combout\);

-- Location: FF_X33_Y12_N3
inst4 : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \CLK0~inputclkctrl_outclk\,
	d => \inst4~feeder_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst4~q\);

-- Location: CLKCTRL_G8
\inst4~clkctrl\ : cycloneive_clkctrl
-- pragma translate_off
GENERIC MAP (
	clock_type => "global clock",
	ena_register_mode => "none")
-- pragma translate_on
PORT MAP (
	inclk => \inst4~clkctrl_INCLK_bus\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	outclk => \inst4~clkctrl_outclk\);

-- Location: LCCOMB_X25_Y23_N6
\inst2|Add0~0\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst2|Add0~0_combout\ = \inst2|q\(0) $ (VCC)
-- \inst2|Add0~1\ = CARRY(\inst2|q\(0))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0011001111001100",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	datab => \inst2|q\(0),
	datad => VCC,
	combout => \inst2|Add0~0_combout\,
	cout => \inst2|Add0~1\);

-- Location: LCCOMB_X25_Y23_N8
\inst2|Add0~2\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst2|Add0~2_combout\ = (\inst2|q\(1) & (\inst2|Add0~1\ & VCC)) # (!\inst2|q\(1) & (!\inst2|Add0~1\))
-- \inst2|Add0~3\ = CARRY((!\inst2|q\(1) & !\inst2|Add0~1\))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1100001100000011",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	datab => \inst2|q\(1),
	datad => VCC,
	cin => \inst2|Add0~1\,
	combout => \inst2|Add0~2_combout\,
	cout => \inst2|Add0~3\);

-- Location: LCCOMB_X25_Y23_N10
\inst2|Add0~4\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst2|Add0~4_combout\ = (\inst2|q\(2) & (\inst2|Add0~3\ $ (GND))) # (!\inst2|q\(2) & (!\inst2|Add0~3\ & VCC))
-- \inst2|Add0~5\ = CARRY((\inst2|q\(2) & !\inst2|Add0~3\))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1010010100001010",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst2|q\(2),
	datad => VCC,
	cin => \inst2|Add0~3\,
	combout => \inst2|Add0~4_combout\,
	cout => \inst2|Add0~5\);

-- Location: FF_X25_Y23_N11
\inst2|q[2]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \ALT_INV_inst4~clkctrl_outclk\,
	d => \inst2|Add0~4_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst2|q\(2));

-- Location: LCCOMB_X25_Y23_N12
\inst2|Add0~6\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst2|Add0~6_combout\ = (\inst2|q\(3) & (!\inst2|Add0~5\)) # (!\inst2|q\(3) & ((\inst2|Add0~5\) # (GND)))
-- \inst2|Add0~7\ = CARRY((!\inst2|Add0~5\) # (!\inst2|q\(3)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0101101001011111",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst2|q\(3),
	datad => VCC,
	cin => \inst2|Add0~5\,
	combout => \inst2|Add0~6_combout\,
	cout => \inst2|Add0~7\);

-- Location: FF_X25_Y23_N13
\inst2|q[3]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \ALT_INV_inst4~clkctrl_outclk\,
	d => \inst2|Add0~6_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst2|q\(3));

-- Location: LCCOMB_X25_Y23_N14
\inst2|Add0~8\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst2|Add0~8_combout\ = \inst2|Add0~7\ $ (!\inst2|q\(4))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1111000000001111",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	datad => \inst2|q\(4),
	cin => \inst2|Add0~7\,
	combout => \inst2|Add0~8_combout\);

-- Location: FF_X25_Y23_N15
\inst2|q[4]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \ALT_INV_inst4~clkctrl_outclk\,
	d => \inst2|Add0~8_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst2|q\(4));

-- Location: LCCOMB_X25_Y23_N18
\inst2|Equal0~0\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst2|Equal0~0_combout\ = (\inst2|q\(2) & (\inst2|q\(4) & (\inst2|q\(1) & \inst2|q\(3))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1000000000000000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst2|q\(2),
	datab => \inst2|q\(4),
	datac => \inst2|q\(1),
	datad => \inst2|q\(3),
	combout => \inst2|Equal0~0_combout\);

-- Location: LCCOMB_X25_Y23_N4
\inst2|q~0\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst2|q~0_combout\ = \inst2|Add0~0_combout\ $ (((!\inst2|q\(0) & \inst2|Equal0~0_combout\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1010010110101010",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst2|Add0~0_combout\,
	datac => \inst2|q\(0),
	datad => \inst2|Equal0~0_combout\,
	combout => \inst2|q~0_combout\);

-- Location: FF_X25_Y23_N5
\inst2|q[0]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \ALT_INV_inst4~clkctrl_outclk\,
	d => \inst2|q~0_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst2|q\(0));

-- Location: FF_X25_Y23_N9
\inst2|q[1]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \ALT_INV_inst4~clkctrl_outclk\,
	d => \inst2|Add0~2_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst2|q\(1));

-- Location: LCCOMB_X26_Y23_N16
\inst2|Mux0~0\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst2|Mux0~0_combout\ = (\inst2|q\(1) & (\inst2|q\(3) $ (((\inst2|q\(4)) # (!\inst2|q\(2)))))) # (!\inst2|q\(1) & ((\inst2|q\(3)) # ((!\inst2|q\(4) & \inst2|q\(2)))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0111100101011010",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst2|q\(1),
	datab => \inst2|q\(4),
	datac => \inst2|q\(3),
	datad => \inst2|q\(2),
	combout => \inst2|Mux0~0_combout\);

-- Location: LCCOMB_X26_Y23_N18
\inst2|Mux0~1\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst2|Mux0~1_combout\ = (\inst2|q\(1) & (\inst2|q\(4) & (!\inst2|q\(3) & !\inst2|q\(2)))) # (!\inst2|q\(1) & (\inst2|q\(3) & (\inst2|q\(4) $ (!\inst2|q\(2)))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0100000000011000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst2|q\(1),
	datab => \inst2|q\(4),
	datac => \inst2|q\(3),
	datad => \inst2|q\(2),
	combout => \inst2|Mux0~1_combout\);

-- Location: LCCOMB_X26_Y23_N0
\inst2|Mux0~2\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst2|Mux0~2_combout\ = (\inst2|q\(0) & ((!\inst2|Mux0~1_combout\))) # (!\inst2|q\(0) & (\inst2|Mux0~0_combout\))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0000110011111100",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	datab => \inst2|Mux0~0_combout\,
	datac => \inst2|q\(0),
	datad => \inst2|Mux0~1_combout\,
	combout => \inst2|Mux0~2_combout\);

-- Location: LCCOMB_X26_Y23_N26
\inst2|Mux1~0\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst2|Mux1~0_combout\ = (\inst2|q\(1) & ((\inst2|q\(4) $ (\inst2|q\(2))) # (!\inst2|q\(3)))) # (!\inst2|q\(1) & ((\inst2|q\(3)) # (\inst2|q\(4) $ (\inst2|q\(2)))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0111101111011110",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst2|q\(1),
	datab => \inst2|q\(4),
	datac => \inst2|q\(3),
	datad => \inst2|q\(2),
	combout => \inst2|Mux1~0_combout\);

-- Location: LCCOMB_X26_Y23_N8
\inst2|Mux1~1\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst2|Mux1~1_combout\ = (\inst2|q\(1) & ((\inst2|q\(4) & (!\inst2|q\(3) & !\inst2|q\(2))) # (!\inst2|q\(4) & (\inst2|q\(3) & \inst2|q\(2))))) # (!\inst2|q\(1) & (\inst2|q\(3) $ (((!\inst2|q\(4) & \inst2|q\(2))))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0110000101011000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst2|q\(1),
	datab => \inst2|q\(4),
	datac => \inst2|q\(3),
	datad => \inst2|q\(2),
	combout => \inst2|Mux1~1_combout\);

-- Location: LCCOMB_X26_Y23_N2
\inst2|Mux1~2\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst2|Mux1~2_combout\ = (\inst2|q\(0) & (\inst2|Mux1~0_combout\)) # (!\inst2|q\(0) & ((\inst2|Mux1~1_combout\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1111001111000000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	datab => \inst2|q\(0),
	datac => \inst2|Mux1~0_combout\,
	datad => \inst2|Mux1~1_combout\,
	combout => \inst2|Mux1~2_combout\);

-- Location: LCCOMB_X26_Y23_N24
\inst2|Mux2~0\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst2|Mux2~0_combout\ = (\inst2|q\(1) & (!\inst2|q\(3) & (\inst2|q\(4) $ (!\inst2|q\(2))))) # (!\inst2|q\(1) & (!\inst2|q\(4) & (\inst2|q\(3) & \inst2|q\(2))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0001100000000010",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst2|q\(1),
	datab => \inst2|q\(4),
	datac => \inst2|q\(3),
	datad => \inst2|q\(2),
	combout => \inst2|Mux2~0_combout\);

-- Location: LCCOMB_X26_Y23_N30
\inst2|Mux2~1\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst2|Mux2~1_combout\ = (\inst2|q\(0) & !\inst2|Mux2~0_combout\)

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0000000011110000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	datac => \inst2|q\(0),
	datad => \inst2|Mux2~0_combout\,
	combout => \inst2|Mux2~1_combout\);

-- Location: LCCOMB_X26_Y23_N28
\inst2|Mux3~0\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst2|Mux3~0_combout\ = (\inst2|q\(0) & ((!\inst2|Mux2~0_combout\))) # (!\inst2|q\(0) & (\inst2|Mux0~0_combout\))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0000110011111100",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	datab => \inst2|Mux0~0_combout\,
	datac => \inst2|q\(0),
	datad => \inst2|Mux2~0_combout\,
	combout => \inst2|Mux3~0_combout\);

-- Location: LCCOMB_X26_Y23_N10
\inst2|Mux4~0\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst2|Mux4~0_combout\ = (\inst2|Mux1~0_combout\) # (!\inst2|q\(0))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1111001111110011",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	datab => \inst2|q\(0),
	datac => \inst2|Mux1~0_combout\,
	combout => \inst2|Mux4~0_combout\);

-- Location: LCCOMB_X26_Y23_N20
\inst2|Mux5~0\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst2|Mux5~0_combout\ = (\inst2|q\(4) & (!\inst2|q\(1) & (\inst2|q\(3) & !\inst2|q\(2)))) # (!\inst2|q\(4) & (\inst2|q\(2) & (\inst2|q\(1) $ (!\inst2|q\(3)))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0010000101000000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst2|q\(1),
	datab => \inst2|q\(4),
	datac => \inst2|q\(3),
	datad => \inst2|q\(2),
	combout => \inst2|Mux5~0_combout\);

-- Location: LCCOMB_X22_Y23_N28
\inst2|Mux6~0\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst2|Mux6~0_combout\ = (\inst2|q\(4) & (((\inst2|q\(3) & !\inst2|q\(1))) # (!\inst2|q\(2)))) # (!\inst2|q\(4) & (\inst2|q\(2) $ (((\inst2|q\(3) & !\inst2|q\(1))))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0101100110101110",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst2|q\(4),
	datab => \inst2|q\(3),
	datac => \inst2|q\(1),
	datad => \inst2|q\(2),
	combout => \inst2|Mux6~0_combout\);

-- Location: LCCOMB_X22_Y23_N2
\inst2|Mux6~1\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst2|Mux6~1_combout\ = (\inst2|Mux6~0_combout\) # (\inst2|q\(3) $ (\inst2|q\(0) $ (\inst2|q\(1))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1111111110010110",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst2|q\(3),
	datab => \inst2|q\(0),
	datac => \inst2|q\(1),
	datad => \inst2|Mux6~0_combout\,
	combout => \inst2|Mux6~1_combout\);

-- Location: LCCOMB_X22_Y23_N0
\inst2|Mux7~0\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst2|Mux7~0_combout\ = (\inst2|q\(3)) # ((\inst2|q\(2)) # ((\inst2|q\(0) & \inst2|q\(1))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1111111111101010",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst2|q\(3),
	datab => \inst2|q\(0),
	datac => \inst2|q\(1),
	datad => \inst2|q\(2),
	combout => \inst2|Mux7~0_combout\);

-- Location: LCCOMB_X22_Y23_N26
\inst2|Mux7~1\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst2|Mux7~1_combout\ = (\inst2|q\(4) & \inst2|Mux7~0_combout\)

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1010101000000000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst2|q\(4),
	datad => \inst2|Mux7~0_combout\,
	combout => \inst2|Mux7~1_combout\);

-- Location: LCCOMB_X24_Y23_N20
\inst2|Mux8~0\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst2|Mux8~0_combout\ = (\inst2|q\(3) & ((\inst2|q\(0)) # ((\inst2|q\(1)) # (\inst2|q\(2)))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1111111000000000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst2|q\(0),
	datab => \inst2|q\(1),
	datac => \inst2|q\(2),
	datad => \inst2|q\(3),
	combout => \inst2|Mux8~0_combout\);

-- Location: LCCOMB_X24_Y23_N18
\inst2|Mux8~1\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst2|Mux8~1_combout\ = (\inst2|q\(4)) # (\inst2|Mux8~0_combout\)

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1111111111110000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	datac => \inst2|q\(4),
	datad => \inst2|Mux8~0_combout\,
	combout => \inst2|Mux8~1_combout\);

-- Location: LCCOMB_X26_Y23_N14
\inst2|Mux9~0\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst2|Mux9~0_combout\ = (\inst2|q\(1) & ((\inst2|q\(3)) # (\inst2|q\(0)))) # (!\inst2|q\(1) & (\inst2|q\(3) & \inst2|q\(0)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1110100011101000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst2|q\(1),
	datab => \inst2|q\(3),
	datac => \inst2|q\(0),
	combout => \inst2|Mux9~0_combout\);

-- Location: LCCOMB_X26_Y23_N12
\inst2|Mux9~1\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst2|Mux9~1_combout\ = (\inst2|q\(4) & ((\inst2|Mux9~0_combout\ & (\inst2|q\(3) & \inst2|q\(2))) # (!\inst2|Mux9~0_combout\ & (!\inst2|q\(3) & !\inst2|q\(2))))) # (!\inst2|q\(4) & (\inst2|q\(3) & ((\inst2|Mux9~0_combout\) # (\inst2|q\(2)))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1101000001000010",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst2|q\(4),
	datab => \inst2|Mux9~0_combout\,
	datac => \inst2|q\(3),
	datad => \inst2|q\(2),
	combout => \inst2|Mux9~1_combout\);

-- Location: LCCOMB_X24_Y23_N8
\inst2|Mux10~0\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst2|Mux10~0_combout\ = (\inst2|q\(4) & (\inst2|Mux7~0_combout\)) # (!\inst2|q\(4) & ((!\inst2|Mux8~0_combout\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1100000011001111",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	datab => \inst2|Mux7~0_combout\,
	datac => \inst2|q\(4),
	datad => \inst2|Mux8~0_combout\,
	combout => \inst2|Mux10~0_combout\);

-- Location: LCCOMB_X26_Y23_N22
\inst2|Mux11~0\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst2|Mux11~0_combout\ = (\inst2|q\(3) & (((!\inst2|q\(1) & !\inst2|q\(0))) # (!\inst2|q\(2)))) # (!\inst2|q\(3) & ((\inst2|q\(2)) # ((\inst2|q\(1) & \inst2|q\(0)))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0011011111101100",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst2|q\(1),
	datab => \inst2|q\(3),
	datac => \inst2|q\(0),
	datad => \inst2|q\(2),
	combout => \inst2|Mux11~0_combout\);

-- Location: LCCOMB_X22_Y23_N12
\inst2|Mux11~1\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst2|Mux11~1_combout\ = (!\inst2|Mux11~0_combout\) # (!\inst2|q\(4))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0101010111111111",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst2|q\(4),
	datad => \inst2|Mux11~0_combout\,
	combout => \inst2|Mux11~1_combout\);

-- Location: LCCOMB_X24_Y23_N14
\inst2|Equal0~1\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst2|Equal0~1_combout\ = (\inst2|q\(4) & \inst2|q\(3))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1100000011000000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	datab => \inst2|q\(4),
	datac => \inst2|q\(3),
	combout => \inst2|Equal0~1_combout\);

-- Location: LCCOMB_X24_Y23_N0
\inst2|Ram0~0\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst2|Ram0~0_combout\ = (\inst2|q\(4) & ((!\inst2|q\(3)))) # (!\inst2|q\(4) & (\inst2|q\(2) & \inst2|q\(3)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0011000011001100",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	datab => \inst2|q\(4),
	datac => \inst2|q\(2),
	datad => \inst2|q\(3),
	combout => \inst2|Ram0~0_combout\);

-- Location: LCCOMB_X24_Y23_N30
\inst2|Ram0~1\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst2|Ram0~1_combout\ = (\inst2|q\(4) & ((\inst2|q\(1) & ((\inst2|q\(2)) # (!\inst2|q\(3)))) # (!\inst2|q\(1) & (\inst2|q\(2) & !\inst2|q\(3))))) # (!\inst2|q\(4) & ((\inst2|q\(2) & (\inst2|q\(1) & !\inst2|q\(3))) # (!\inst2|q\(2) & ((\inst2|q\(3))))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1000001111101000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst2|q\(1),
	datab => \inst2|q\(4),
	datac => \inst2|q\(2),
	datad => \inst2|q\(3),
	combout => \inst2|Ram0~1_combout\);

-- Location: LCCOMB_X22_Y23_N8
\inst2|Ram0~3\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst2|Ram0~3_combout\ = (\inst2|q\(4) & ((\inst2|q\(1) & (!\inst2|q\(3) & \inst2|q\(2))) # (!\inst2|q\(1) & (\inst2|q\(3) $ (!\inst2|q\(2)))))) # (!\inst2|q\(4) & ((\inst2|q\(1) & (\inst2|q\(3) & !\inst2|q\(2))) # (!\inst2|q\(1) & (!\inst2|q\(3) & 
-- \inst2|q\(2)))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0010100101000010",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst2|q\(4),
	datab => \inst2|q\(1),
	datac => \inst2|q\(3),
	datad => \inst2|q\(2),
	combout => \inst2|Ram0~3_combout\);

-- Location: LCCOMB_X22_Y23_N18
\inst2|Ram0~2\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst2|Ram0~2_combout\ = (\inst2|q\(4) & ((\inst2|q\(1) & (\inst2|q\(3) $ (\inst2|q\(2)))) # (!\inst2|q\(1) & ((\inst2|q\(2)) # (!\inst2|q\(3)))))) # (!\inst2|q\(4) & ((\inst2|q\(1) & ((\inst2|q\(3)) # (!\inst2|q\(2)))) # (!\inst2|q\(1) & (\inst2|q\(3) $ 
-- (\inst2|q\(2))))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0110101111010110",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst2|q\(4),
	datab => \inst2|q\(1),
	datac => \inst2|q\(3),
	datad => \inst2|q\(2),
	combout => \inst2|Ram0~2_combout\);

-- Location: LCCOMB_X22_Y23_N30
\inst2|Ram0~4\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst2|Ram0~4_combout\ = (\inst2|q\(0) & ((\inst2|Ram0~2_combout\))) # (!\inst2|q\(0) & (\inst2|Ram0~3_combout\))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1111110000110000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	datab => \inst2|q\(0),
	datac => \inst2|Ram0~3_combout\,
	datad => \inst2|Ram0~2_combout\,
	combout => \inst2|Ram0~4_combout\);

-- Location: LCCOMB_X22_Y23_N22
\inst2|Ram0~6\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst2|Ram0~6_combout\ = (\inst2|q\(4) & (\inst2|q\(1) $ (((!\inst2|q\(0) & !\inst2|q\(2)))))) # (!\inst2|q\(4) & ((\inst2|q\(1) & (\inst2|q\(0) & \inst2|q\(2))) # (!\inst2|q\(1) & (\inst2|q\(0) $ (\inst2|q\(2))))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1100100110010010",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst2|q\(4),
	datab => \inst2|q\(1),
	datac => \inst2|q\(0),
	datad => \inst2|q\(2),
	combout => \inst2|Ram0~6_combout\);

-- Location: LCCOMB_X22_Y23_N24
\inst2|Ram0~5\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst2|Ram0~5_combout\ = (\inst2|q\(4) & ((\inst2|q\(0) & ((\inst2|q\(1)) # (!\inst2|q\(2)))) # (!\inst2|q\(0) & ((\inst2|q\(2)) # (!\inst2|q\(1)))))) # (!\inst2|q\(4) & ((\inst2|q\(0) & ((\inst2|q\(2)) # (!\inst2|q\(1)))) # (!\inst2|q\(0) & (\inst2|q\(1) 
-- $ (\inst2|q\(2))))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1110011110011110",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst2|q\(4),
	datab => \inst2|q\(0),
	datac => \inst2|q\(1),
	datad => \inst2|q\(2),
	combout => \inst2|Ram0~5_combout\);

-- Location: LCCOMB_X22_Y23_N4
\inst2|Ram0~7\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst2|Ram0~7_combout\ = (\inst2|q\(3) & (!\inst2|Ram0~6_combout\)) # (!\inst2|q\(3) & ((\inst2|Ram0~5_combout\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0101111100001010",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst2|q\(3),
	datac => \inst2|Ram0~6_combout\,
	datad => \inst2|Ram0~5_combout\,
	combout => \inst2|Ram0~7_combout\);

-- Location: LCCOMB_X24_Y23_N16
\inst2|Ram0~8\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst2|Ram0~8_combout\ = (\inst2|q\(0) & (\inst2|q\(2) $ (((!\inst2|q\(3) & !\inst2|q\(1)))))) # (!\inst2|q\(0) & ((\inst2|q\(3) & (!\inst2|q\(2) & \inst2|q\(1))) # (!\inst2|q\(3) & (\inst2|q\(2) & !\inst2|q\(1)))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1010010010010010",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst2|q\(0),
	datab => \inst2|q\(3),
	datac => \inst2|q\(2),
	datad => \inst2|q\(1),
	combout => \inst2|Ram0~8_combout\);

-- Location: LCCOMB_X24_Y23_N6
\inst2|Ram0~9\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst2|Ram0~9_combout\ = \inst2|q\(4) $ (\inst2|Ram0~8_combout\)

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0000111111110000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	datac => \inst2|q\(4),
	datad => \inst2|Ram0~8_combout\,
	combout => \inst2|Ram0~9_combout\);

-- Location: LCCOMB_X24_Y23_N24
\inst2|Ram0~10\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst2|Ram0~10_combout\ = \inst2|q\(1) $ (\inst2|q\(3) $ (((\inst2|q\(2)) # (\inst2|q\(0)))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1010010110010110",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst2|q\(1),
	datab => \inst2|q\(2),
	datac => \inst2|q\(3),
	datad => \inst2|q\(0),
	combout => \inst2|Ram0~10_combout\);

-- Location: LCCOMB_X24_Y23_N10
\inst2|Ram0~11\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst2|Ram0~11_combout\ = \inst2|q\(2) $ (\inst2|q\(0))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0000111111110000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	datac => \inst2|q\(2),
	datad => \inst2|q\(0),
	combout => \inst2|Ram0~11_combout\);

-- Location: LCCOMB_X22_Y23_N10
\inst2|Ram0~12\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst2|Ram0~12_combout\ = (!\inst2|q\(1) & !\inst2|q\(2))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0000000000001111",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	datac => \inst2|q\(1),
	datad => \inst2|q\(2),
	combout => \inst2|Ram0~12_combout\);

-- Location: LCCOMB_X22_Y23_N16
\inst2|Ram0~13\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst2|Ram0~13_combout\ = ((!\inst2|q\(0) & (!\inst2|q\(3) & \inst2|Ram0~12_combout\))) # (!\inst2|q\(4))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0101011101010101",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst2|q\(4),
	datab => \inst2|q\(0),
	datac => \inst2|q\(3),
	datad => \inst2|Ram0~12_combout\,
	combout => \inst2|Ram0~13_combout\);

-- Location: LCCOMB_X22_Y23_N6
\inst2|Ram0~14\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst2|Ram0~14_combout\ = (\inst2|q\(4) & ((\inst2|q\(3) & ((\inst2|Ram0~12_combout\))) # (!\inst2|q\(3) & ((\inst2|q\(0)) # (!\inst2|Ram0~12_combout\))))) # (!\inst2|q\(4) & (((!\inst2|q\(3)))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1010110100001111",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst2|q\(4),
	datab => \inst2|q\(0),
	datac => \inst2|q\(3),
	datad => \inst2|Ram0~12_combout\,
	combout => \inst2|Ram0~14_combout\);

-- Location: LCCOMB_X22_Y23_N20
\inst2|Ram0~15\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst2|Ram0~15_combout\ = (\inst2|q\(0) & (!\inst2|q\(3) & \inst2|q\(4))) # (!\inst2|q\(0) & (\inst2|q\(3) & !\inst2|q\(4)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0000110000110000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	datab => \inst2|q\(0),
	datac => \inst2|q\(3),
	datad => \inst2|q\(4),
	combout => \inst2|Ram0~15_combout\);

-- Location: LCCOMB_X22_Y23_N14
\inst2|Ram0~16\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst2|Ram0~16_combout\ = (\inst2|q\(4) & (\inst2|q\(2) $ (((\inst2|q\(1)) # (\inst2|Ram0~15_combout\))))) # (!\inst2|q\(4) & (((!\inst2|q\(1) & \inst2|Ram0~15_combout\)) # (!\inst2|q\(2))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0011011100111001",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst2|q\(4),
	datab => \inst2|q\(2),
	datac => \inst2|q\(1),
	datad => \inst2|Ram0~15_combout\,
	combout => \inst2|Ram0~16_combout\);

-- Location: LCCOMB_X25_Y23_N24
\inst2|Ram0~17\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst2|Ram0~17_combout\ = (\inst2|q\(0) & (\inst2|q\(4) & !\inst2|q\(3))) # (!\inst2|q\(0) & (!\inst2|q\(4) & \inst2|q\(3)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0000001111000000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	datab => \inst2|q\(0),
	datac => \inst2|q\(4),
	datad => \inst2|q\(3),
	combout => \inst2|Ram0~17_combout\);

-- Location: LCCOMB_X24_Y23_N12
\inst2|Ram0~18\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst2|Ram0~18_combout\ = (\inst2|Ram0~17_combout\ & ((\inst2|q\(2) & ((\inst2|q\(1)) # (\inst2|q\(4)))) # (!\inst2|q\(2) & (!\inst2|q\(1))))) # (!\inst2|Ram0~17_combout\ & ((\inst2|q\(1) $ (!\inst2|q\(4)))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1011100111000011",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst2|q\(2),
	datab => \inst2|q\(1),
	datac => \inst2|q\(4),
	datad => \inst2|Ram0~17_combout\,
	combout => \inst2|Ram0~18_combout\);

-- Location: LCCOMB_X25_Y23_N20
\inst2|Ram0~30\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst2|Ram0~30_combout\ = (\inst2|q\(4) & (\inst2|q\(2) & ((\inst2|q\(3)) # (\inst2|q\(1))))) # (!\inst2|q\(4) & (((\inst2|q\(1) & !\inst2|q\(2))) # (!\inst2|q\(3))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1101100100110001",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst2|q\(3),
	datab => \inst2|q\(4),
	datac => \inst2|q\(1),
	datad => \inst2|q\(2),
	combout => \inst2|Ram0~30_combout\);

-- Location: LCCOMB_X25_Y23_N30
\inst2|Ram0~31\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst2|Ram0~31_combout\ = (\inst2|q\(0) & (\inst2|Ram0~30_combout\ $ (((\inst2|q\(2)) # (!\inst2|q\(3)))))) # (!\inst2|q\(0) & ((\inst2|Ram0~30_combout\) # ((\inst2|q\(3) & !\inst2|q\(2)))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0011101111000110",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst2|q\(3),
	datab => \inst2|q\(0),
	datac => \inst2|q\(2),
	datad => \inst2|Ram0~30_combout\,
	combout => \inst2|Ram0~31_combout\);

-- Location: LCCOMB_X25_Y23_N2
\inst2|Ram0~28\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst2|Ram0~28_combout\ = (\inst2|q\(3) & (!\inst2|q\(2) & ((!\inst2|q\(0)) # (!\inst2|q\(1))))) # (!\inst2|q\(3) & (\inst2|q\(2) & ((\inst2|q\(1)) # (\inst2|q\(0)))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0101010000101010",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst2|q\(3),
	datab => \inst2|q\(1),
	datac => \inst2|q\(0),
	datad => \inst2|q\(2),
	combout => \inst2|Ram0~28_combout\);

-- Location: LCCOMB_X24_Y23_N2
\inst2|Ram0~29\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst2|Ram0~29_combout\ = (\inst2|q\(4) & (((!\inst2|q\(1) & !\inst2|q\(3))) # (!\inst2|Ram0~28_combout\))) # (!\inst2|q\(4) & (((\inst2|Ram0~28_combout\))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0001111111110000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst2|q\(1),
	datab => \inst2|q\(3),
	datac => \inst2|q\(4),
	datad => \inst2|Ram0~28_combout\,
	combout => \inst2|Ram0~29_combout\);

-- Location: LCCOMB_X25_Y23_N0
\inst2|Ram0~19\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst2|Ram0~19_combout\ = (\inst2|q\(2) & (!\inst2|q\(1) & ((\inst2|q\(4)) # (!\inst2|q\(0))))) # (!\inst2|q\(2) & (\inst2|q\(1) & ((\inst2|q\(0)) # (!\inst2|q\(4)))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0110010000100110",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst2|q\(2),
	datab => \inst2|q\(1),
	datac => \inst2|q\(4),
	datad => \inst2|q\(0),
	combout => \inst2|Ram0~19_combout\);

-- Location: LCCOMB_X25_Y23_N22
\inst2|Ram0~20\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst2|Ram0~20_combout\ = (\inst2|q\(2) & (!\inst2|q\(1) & ((\inst2|q\(4)) # (!\inst2|q\(0))))) # (!\inst2|q\(2) & ((\inst2|q\(0) & ((\inst2|q\(1)))) # (!\inst2|q\(0) & (\inst2|q\(4) & !\inst2|q\(1)))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0100010010110010",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst2|q\(2),
	datab => \inst2|q\(0),
	datac => \inst2|q\(4),
	datad => \inst2|q\(1),
	combout => \inst2|Ram0~20_combout\);

-- Location: LCCOMB_X24_Y23_N26
\inst2|Ram0~21\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst2|Ram0~21_combout\ = (\inst2|q\(3) & ((!\inst2|Ram0~20_combout\))) # (!\inst2|q\(3) & (\inst2|Ram0~19_combout\))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0011000011111100",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	datab => \inst2|q\(3),
	datac => \inst2|Ram0~19_combout\,
	datad => \inst2|Ram0~20_combout\,
	combout => \inst2|Ram0~21_combout\);

-- Location: LCCOMB_X25_Y23_N28
\inst2|Ram0~22\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst2|Ram0~22_combout\ = (\inst2|q\(0) & (((!\inst2|q\(4) & !\inst2|q\(1))))) # (!\inst2|q\(0) & ((\inst2|q\(3) & ((\inst2|q\(4)) # (\inst2|q\(1)))) # (!\inst2|q\(3) & (\inst2|q\(4) & \inst2|q\(1)))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0011001000101100",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst2|q\(3),
	datab => \inst2|q\(0),
	datac => \inst2|q\(4),
	datad => \inst2|q\(1),
	combout => \inst2|Ram0~22_combout\);

-- Location: LCCOMB_X25_Y23_N26
\inst2|Ram0~23\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst2|Ram0~23_combout\ = \inst2|q\(2) $ (\inst2|Ram0~22_combout\)

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0000111111110000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	datac => \inst2|q\(2),
	datad => \inst2|Ram0~22_combout\,
	combout => \inst2|Ram0~23_combout\);

-- Location: LCCOMB_X24_Y23_N28
\inst2|Ram0~24\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst2|Ram0~24_combout\ = \inst2|q\(4) $ (\inst2|q\(1) $ (((\inst2|q\(0)) # (\inst2|q\(3)))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1100001110010110",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst2|q\(0),
	datab => \inst2|q\(4),
	datac => \inst2|q\(1),
	datad => \inst2|q\(3),
	combout => \inst2|Ram0~24_combout\);

-- Location: LCCOMB_X24_Y23_N22
\inst2|Ram0~25\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst2|Ram0~25_combout\ = \inst2|q\(3) $ (\inst2|q\(0))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0000111111110000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	datac => \inst2|q\(3),
	datad => \inst2|q\(0),
	combout => \inst2|Ram0~25_combout\);

-- Location: LCCOMB_X25_Y23_N16
\inst2|Ram0~26\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst2|Ram0~26_combout\ = (!\inst2|q\(3) & (!\inst2|q\(4) & (!\inst2|q\(0) & \inst2|q\(2))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0000000100000000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst2|q\(3),
	datab => \inst2|q\(4),
	datac => \inst2|q\(0),
	datad => \inst2|q\(2),
	combout => \inst2|Ram0~26_combout\);

-- Location: LCCOMB_X24_Y23_N4
\inst2|Ram0~27\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst2|Ram0~27_combout\ = (\inst2|q\(1)) # (\inst2|Ram0~26_combout\)

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1111111111110000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	datac => \inst2|q\(1),
	datad => \inst2|Ram0~26_combout\,
	combout => \inst2|Ram0~27_combout\);

-- Location: LCCOMB_X23_Y23_N0
\inst3|acc[0]~32\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst3|acc[0]~32_combout\ = (\inst2|q\(0) & ((\inst3|acc\(0)) # (GND))) # (!\inst2|q\(0) & (\inst3|acc\(0) $ (VCC)))
-- \inst3|acc[0]~33\ = CARRY((\inst2|q\(0)) # (\inst3|acc\(0)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1001100111101110",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst2|q\(0),
	datab => \inst3|acc\(0),
	datad => VCC,
	combout => \inst3|acc[0]~32_combout\,
	cout => \inst3|acc[0]~33\);

-- Location: FF_X23_Y23_N1
\inst3|acc[0]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \CLK0~inputclkctrl_outclk\,
	d => \inst3|acc[0]~32_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst3|acc\(0));

-- Location: LCCOMB_X23_Y23_N2
\inst3|acc[1]~34\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst3|acc[1]~34_combout\ = (\inst2|Ram0~27_combout\ & ((\inst3|acc\(1) & (\inst3|acc[0]~33\ & VCC)) # (!\inst3|acc\(1) & (!\inst3|acc[0]~33\)))) # (!\inst2|Ram0~27_combout\ & ((\inst3|acc\(1) & (!\inst3|acc[0]~33\)) # (!\inst3|acc\(1) & 
-- ((\inst3|acc[0]~33\) # (GND)))))
-- \inst3|acc[1]~35\ = CARRY((\inst2|Ram0~27_combout\ & (!\inst3|acc\(1) & !\inst3|acc[0]~33\)) # (!\inst2|Ram0~27_combout\ & ((!\inst3|acc[0]~33\) # (!\inst3|acc\(1)))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1001011000010111",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst2|Ram0~27_combout\,
	datab => \inst3|acc\(1),
	datad => VCC,
	cin => \inst3|acc[0]~33\,
	combout => \inst3|acc[1]~34_combout\,
	cout => \inst3|acc[1]~35\);

-- Location: FF_X23_Y23_N3
\inst3|acc[1]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \CLK0~inputclkctrl_outclk\,
	d => \inst3|acc[1]~34_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst3|acc\(1));

-- Location: LCCOMB_X23_Y23_N4
\inst3|acc[2]~36\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst3|acc[2]~36_combout\ = ((\inst2|q\(2) $ (\inst3|acc\(2) $ (!\inst3|acc[1]~35\)))) # (GND)
-- \inst3|acc[2]~37\ = CARRY((\inst2|q\(2) & ((\inst3|acc\(2)) # (!\inst3|acc[1]~35\))) # (!\inst2|q\(2) & (\inst3|acc\(2) & !\inst3|acc[1]~35\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0110100110001110",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst2|q\(2),
	datab => \inst3|acc\(2),
	datad => VCC,
	cin => \inst3|acc[1]~35\,
	combout => \inst3|acc[2]~36_combout\,
	cout => \inst3|acc[2]~37\);

-- Location: FF_X23_Y23_N5
\inst3|acc[2]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \CLK0~inputclkctrl_outclk\,
	d => \inst3|acc[2]~36_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst3|acc\(2));

-- Location: LCCOMB_X23_Y23_N6
\inst3|acc[3]~38\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst3|acc[3]~38_combout\ = (\inst3|acc\(3) & ((\inst2|Ram0~25_combout\ & (!\inst3|acc[2]~37\)) # (!\inst2|Ram0~25_combout\ & (\inst3|acc[2]~37\ & VCC)))) # (!\inst3|acc\(3) & ((\inst2|Ram0~25_combout\ & ((\inst3|acc[2]~37\) # (GND))) # 
-- (!\inst2|Ram0~25_combout\ & (!\inst3|acc[2]~37\))))
-- \inst3|acc[3]~39\ = CARRY((\inst3|acc\(3) & (\inst2|Ram0~25_combout\ & !\inst3|acc[2]~37\)) # (!\inst3|acc\(3) & ((\inst2|Ram0~25_combout\) # (!\inst3|acc[2]~37\))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0110100101001101",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst3|acc\(3),
	datab => \inst2|Ram0~25_combout\,
	datad => VCC,
	cin => \inst3|acc[2]~37\,
	combout => \inst3|acc[3]~38_combout\,
	cout => \inst3|acc[3]~39\);

-- Location: FF_X23_Y23_N7
\inst3|acc[3]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \CLK0~inputclkctrl_outclk\,
	d => \inst3|acc[3]~38_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst3|acc\(3));

-- Location: LCCOMB_X23_Y23_N8
\inst3|acc[4]~40\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst3|acc[4]~40_combout\ = ((\inst2|Ram0~24_combout\ $ (\inst3|acc\(4) $ (!\inst3|acc[3]~39\)))) # (GND)
-- \inst3|acc[4]~41\ = CARRY((\inst2|Ram0~24_combout\ & ((\inst3|acc\(4)) # (!\inst3|acc[3]~39\))) # (!\inst2|Ram0~24_combout\ & (\inst3|acc\(4) & !\inst3|acc[3]~39\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0110100110001110",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst2|Ram0~24_combout\,
	datab => \inst3|acc\(4),
	datad => VCC,
	cin => \inst3|acc[3]~39\,
	combout => \inst3|acc[4]~40_combout\,
	cout => \inst3|acc[4]~41\);

-- Location: FF_X23_Y23_N9
\inst3|acc[4]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \CLK0~inputclkctrl_outclk\,
	d => \inst3|acc[4]~40_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst3|acc\(4));

-- Location: LCCOMB_X23_Y23_N10
\inst3|acc[5]~42\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst3|acc[5]~42_combout\ = (\inst3|acc\(5) & ((\inst2|Ram0~23_combout\ & (!\inst3|acc[4]~41\)) # (!\inst2|Ram0~23_combout\ & (\inst3|acc[4]~41\ & VCC)))) # (!\inst3|acc\(5) & ((\inst2|Ram0~23_combout\ & ((\inst3|acc[4]~41\) # (GND))) # 
-- (!\inst2|Ram0~23_combout\ & (!\inst3|acc[4]~41\))))
-- \inst3|acc[5]~43\ = CARRY((\inst3|acc\(5) & (\inst2|Ram0~23_combout\ & !\inst3|acc[4]~41\)) # (!\inst3|acc\(5) & ((\inst2|Ram0~23_combout\) # (!\inst3|acc[4]~41\))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0110100101001101",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst3|acc\(5),
	datab => \inst2|Ram0~23_combout\,
	datad => VCC,
	cin => \inst3|acc[4]~41\,
	combout => \inst3|acc[5]~42_combout\,
	cout => \inst3|acc[5]~43\);

-- Location: FF_X23_Y23_N11
\inst3|acc[5]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \CLK0~inputclkctrl_outclk\,
	d => \inst3|acc[5]~42_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst3|acc\(5));

-- Location: LCCOMB_X23_Y23_N12
\inst3|acc[6]~44\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst3|acc[6]~44_combout\ = ((\inst3|acc\(6) $ (\inst2|Ram0~21_combout\ $ (\inst3|acc[5]~43\)))) # (GND)
-- \inst3|acc[6]~45\ = CARRY((\inst3|acc\(6) & ((!\inst3|acc[5]~43\) # (!\inst2|Ram0~21_combout\))) # (!\inst3|acc\(6) & (!\inst2|Ram0~21_combout\ & !\inst3|acc[5]~43\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1001011000101011",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst3|acc\(6),
	datab => \inst2|Ram0~21_combout\,
	datad => VCC,
	cin => \inst3|acc[5]~43\,
	combout => \inst3|acc[6]~44_combout\,
	cout => \inst3|acc[6]~45\);

-- Location: FF_X23_Y23_N13
\inst3|acc[6]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \CLK0~inputclkctrl_outclk\,
	d => \inst3|acc[6]~44_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst3|acc\(6));

-- Location: LCCOMB_X23_Y23_N14
\inst3|acc[7]~46\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst3|acc[7]~46_combout\ = (\inst2|Ram0~29_combout\ & ((\inst3|acc\(7) & (!\inst3|acc[6]~45\)) # (!\inst3|acc\(7) & ((\inst3|acc[6]~45\) # (GND))))) # (!\inst2|Ram0~29_combout\ & ((\inst3|acc\(7) & (\inst3|acc[6]~45\ & VCC)) # (!\inst3|acc\(7) & 
-- (!\inst3|acc[6]~45\))))
-- \inst3|acc[7]~47\ = CARRY((\inst2|Ram0~29_combout\ & ((!\inst3|acc[6]~45\) # (!\inst3|acc\(7)))) # (!\inst2|Ram0~29_combout\ & (!\inst3|acc\(7) & !\inst3|acc[6]~45\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0110100100101011",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst2|Ram0~29_combout\,
	datab => \inst3|acc\(7),
	datad => VCC,
	cin => \inst3|acc[6]~45\,
	combout => \inst3|acc[7]~46_combout\,
	cout => \inst3|acc[7]~47\);

-- Location: FF_X23_Y23_N15
\inst3|acc[7]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \CLK0~inputclkctrl_outclk\,
	d => \inst3|acc[7]~46_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst3|acc\(7));

-- Location: LCCOMB_X23_Y23_N16
\inst3|acc[8]~48\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst3|acc[8]~48_combout\ = ((\inst2|Ram0~31_combout\ $ (\inst3|acc\(8) $ (\inst3|acc[7]~47\)))) # (GND)
-- \inst3|acc[8]~49\ = CARRY((\inst2|Ram0~31_combout\ & (\inst3|acc\(8) & !\inst3|acc[7]~47\)) # (!\inst2|Ram0~31_combout\ & ((\inst3|acc\(8)) # (!\inst3|acc[7]~47\))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1001011001001101",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst2|Ram0~31_combout\,
	datab => \inst3|acc\(8),
	datad => VCC,
	cin => \inst3|acc[7]~47\,
	combout => \inst3|acc[8]~48_combout\,
	cout => \inst3|acc[8]~49\);

-- Location: FF_X23_Y23_N17
\inst3|acc[8]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \CLK0~inputclkctrl_outclk\,
	d => \inst3|acc[8]~48_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst3|acc\(8));

-- Location: LCCOMB_X23_Y23_N18
\inst3|acc[9]~50\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst3|acc[9]~50_combout\ = (\inst3|acc\(9) & ((\inst2|Ram0~18_combout\ & (!\inst3|acc[8]~49\)) # (!\inst2|Ram0~18_combout\ & (\inst3|acc[8]~49\ & VCC)))) # (!\inst3|acc\(9) & ((\inst2|Ram0~18_combout\ & ((\inst3|acc[8]~49\) # (GND))) # 
-- (!\inst2|Ram0~18_combout\ & (!\inst3|acc[8]~49\))))
-- \inst3|acc[9]~51\ = CARRY((\inst3|acc\(9) & (\inst2|Ram0~18_combout\ & !\inst3|acc[8]~49\)) # (!\inst3|acc\(9) & ((\inst2|Ram0~18_combout\) # (!\inst3|acc[8]~49\))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0110100101001101",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst3|acc\(9),
	datab => \inst2|Ram0~18_combout\,
	datad => VCC,
	cin => \inst3|acc[8]~49\,
	combout => \inst3|acc[9]~50_combout\,
	cout => \inst3|acc[9]~51\);

-- Location: FF_X23_Y23_N19
\inst3|acc[9]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \CLK0~inputclkctrl_outclk\,
	d => \inst3|acc[9]~50_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst3|acc\(9));

-- Location: LCCOMB_X23_Y23_N20
\inst3|acc[10]~52\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst3|acc[10]~52_combout\ = ((\inst2|Ram0~16_combout\ $ (\inst3|acc\(10) $ (\inst3|acc[9]~51\)))) # (GND)
-- \inst3|acc[10]~53\ = CARRY((\inst2|Ram0~16_combout\ & (\inst3|acc\(10) & !\inst3|acc[9]~51\)) # (!\inst2|Ram0~16_combout\ & ((\inst3|acc\(10)) # (!\inst3|acc[9]~51\))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1001011001001101",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst2|Ram0~16_combout\,
	datab => \inst3|acc\(10),
	datad => VCC,
	cin => \inst3|acc[9]~51\,
	combout => \inst3|acc[10]~52_combout\,
	cout => \inst3|acc[10]~53\);

-- Location: FF_X23_Y23_N21
\inst3|acc[10]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \CLK0~inputclkctrl_outclk\,
	d => \inst3|acc[10]~52_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst3|acc\(10));

-- Location: LCCOMB_X23_Y23_N22
\inst3|acc[11]~54\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst3|acc[11]~54_combout\ = (\inst3|acc\(11) & ((\inst2|Ram0~14_combout\ & (!\inst3|acc[10]~53\)) # (!\inst2|Ram0~14_combout\ & (\inst3|acc[10]~53\ & VCC)))) # (!\inst3|acc\(11) & ((\inst2|Ram0~14_combout\ & ((\inst3|acc[10]~53\) # (GND))) # 
-- (!\inst2|Ram0~14_combout\ & (!\inst3|acc[10]~53\))))
-- \inst3|acc[11]~55\ = CARRY((\inst3|acc\(11) & (\inst2|Ram0~14_combout\ & !\inst3|acc[10]~53\)) # (!\inst3|acc\(11) & ((\inst2|Ram0~14_combout\) # (!\inst3|acc[10]~53\))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0110100101001101",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst3|acc\(11),
	datab => \inst2|Ram0~14_combout\,
	datad => VCC,
	cin => \inst3|acc[10]~53\,
	combout => \inst3|acc[11]~54_combout\,
	cout => \inst3|acc[11]~55\);

-- Location: FF_X23_Y23_N23
\inst3|acc[11]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \CLK0~inputclkctrl_outclk\,
	d => \inst3|acc[11]~54_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst3|acc\(11));

-- Location: LCCOMB_X23_Y23_N24
\inst3|acc[12]~56\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst3|acc[12]~56_combout\ = ((\inst2|Ram0~13_combout\ $ (\inst3|acc\(12) $ (\inst3|acc[11]~55\)))) # (GND)
-- \inst3|acc[12]~57\ = CARRY((\inst2|Ram0~13_combout\ & (\inst3|acc\(12) & !\inst3|acc[11]~55\)) # (!\inst2|Ram0~13_combout\ & ((\inst3|acc\(12)) # (!\inst3|acc[11]~55\))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1001011001001101",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst2|Ram0~13_combout\,
	datab => \inst3|acc\(12),
	datad => VCC,
	cin => \inst3|acc[11]~55\,
	combout => \inst3|acc[12]~56_combout\,
	cout => \inst3|acc[12]~57\);

-- Location: FF_X23_Y23_N25
\inst3|acc[12]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \CLK0~inputclkctrl_outclk\,
	d => \inst3|acc[12]~56_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst3|acc\(12));

-- Location: LCCOMB_X23_Y23_N26
\inst3|acc[13]~58\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst3|acc[13]~58_combout\ = (\inst3|acc\(13) & ((\inst2|q\(0) & (!\inst3|acc[12]~57\)) # (!\inst2|q\(0) & (\inst3|acc[12]~57\ & VCC)))) # (!\inst3|acc\(13) & ((\inst2|q\(0) & ((\inst3|acc[12]~57\) # (GND))) # (!\inst2|q\(0) & (!\inst3|acc[12]~57\))))
-- \inst3|acc[13]~59\ = CARRY((\inst3|acc\(13) & (\inst2|q\(0) & !\inst3|acc[12]~57\)) # (!\inst3|acc\(13) & ((\inst2|q\(0)) # (!\inst3|acc[12]~57\))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0110100101001101",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst3|acc\(13),
	datab => \inst2|q\(0),
	datad => VCC,
	cin => \inst3|acc[12]~57\,
	combout => \inst3|acc[13]~58_combout\,
	cout => \inst3|acc[13]~59\);

-- Location: FF_X23_Y23_N27
\inst3|acc[13]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \CLK0~inputclkctrl_outclk\,
	d => \inst3|acc[13]~58_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst3|acc\(13));

-- Location: LCCOMB_X23_Y23_N28
\inst3|acc[14]~60\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst3|acc[14]~60_combout\ = ((\inst2|q\(1) $ (\inst3|acc\(14) $ (\inst3|acc[13]~59\)))) # (GND)
-- \inst3|acc[14]~61\ = CARRY((\inst2|q\(1) & (\inst3|acc\(14) & !\inst3|acc[13]~59\)) # (!\inst2|q\(1) & ((\inst3|acc\(14)) # (!\inst3|acc[13]~59\))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1001011001001101",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst2|q\(1),
	datab => \inst3|acc\(14),
	datad => VCC,
	cin => \inst3|acc[13]~59\,
	combout => \inst3|acc[14]~60_combout\,
	cout => \inst3|acc[14]~61\);

-- Location: FF_X23_Y23_N29
\inst3|acc[14]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \CLK0~inputclkctrl_outclk\,
	d => \inst3|acc[14]~60_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst3|acc\(14));

-- Location: LCCOMB_X23_Y23_N30
\inst3|acc[15]~62\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst3|acc[15]~62_combout\ = (\inst3|acc\(15) & ((\inst2|Ram0~11_combout\ & (\inst3|acc[14]~61\ & VCC)) # (!\inst2|Ram0~11_combout\ & (!\inst3|acc[14]~61\)))) # (!\inst3|acc\(15) & ((\inst2|Ram0~11_combout\ & (!\inst3|acc[14]~61\)) # 
-- (!\inst2|Ram0~11_combout\ & ((\inst3|acc[14]~61\) # (GND)))))
-- \inst3|acc[15]~63\ = CARRY((\inst3|acc\(15) & (!\inst2|Ram0~11_combout\ & !\inst3|acc[14]~61\)) # (!\inst3|acc\(15) & ((!\inst3|acc[14]~61\) # (!\inst2|Ram0~11_combout\))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1001011000010111",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst3|acc\(15),
	datab => \inst2|Ram0~11_combout\,
	datad => VCC,
	cin => \inst3|acc[14]~61\,
	combout => \inst3|acc[15]~62_combout\,
	cout => \inst3|acc[15]~63\);

-- Location: FF_X23_Y23_N31
\inst3|acc[15]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \CLK0~inputclkctrl_outclk\,
	d => \inst3|acc[15]~62_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst3|acc\(15));

-- Location: LCCOMB_X23_Y22_N0
\inst3|acc[16]~64\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst3|acc[16]~64_combout\ = ((\inst2|Ram0~10_combout\ $ (\inst3|acc\(16) $ (\inst3|acc[15]~63\)))) # (GND)
-- \inst3|acc[16]~65\ = CARRY((\inst2|Ram0~10_combout\ & (\inst3|acc\(16) & !\inst3|acc[15]~63\)) # (!\inst2|Ram0~10_combout\ & ((\inst3|acc\(16)) # (!\inst3|acc[15]~63\))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1001011001001101",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst2|Ram0~10_combout\,
	datab => \inst3|acc\(16),
	datad => VCC,
	cin => \inst3|acc[15]~63\,
	combout => \inst3|acc[16]~64_combout\,
	cout => \inst3|acc[16]~65\);

-- Location: FF_X23_Y22_N1
\inst3|acc[16]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \CLK0~inputclkctrl_outclk\,
	d => \inst3|acc[16]~64_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst3|acc\(16));

-- Location: LCCOMB_X23_Y22_N2
\inst3|acc[17]~66\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst3|acc[17]~66_combout\ = (\inst2|Ram0~9_combout\ & ((\inst3|acc\(17) & (\inst3|acc[16]~65\ & VCC)) # (!\inst3|acc\(17) & (!\inst3|acc[16]~65\)))) # (!\inst2|Ram0~9_combout\ & ((\inst3|acc\(17) & (!\inst3|acc[16]~65\)) # (!\inst3|acc\(17) & 
-- ((\inst3|acc[16]~65\) # (GND)))))
-- \inst3|acc[17]~67\ = CARRY((\inst2|Ram0~9_combout\ & (!\inst3|acc\(17) & !\inst3|acc[16]~65\)) # (!\inst2|Ram0~9_combout\ & ((!\inst3|acc[16]~65\) # (!\inst3|acc\(17)))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1001011000010111",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst2|Ram0~9_combout\,
	datab => \inst3|acc\(17),
	datad => VCC,
	cin => \inst3|acc[16]~65\,
	combout => \inst3|acc[17]~66_combout\,
	cout => \inst3|acc[17]~67\);

-- Location: FF_X23_Y22_N3
\inst3|acc[17]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \CLK0~inputclkctrl_outclk\,
	d => \inst3|acc[17]~66_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst3|acc\(17));

-- Location: LCCOMB_X23_Y22_N4
\inst3|acc[18]~68\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst3|acc[18]~68_combout\ = ((\inst2|Ram0~7_combout\ $ (\inst3|acc\(18) $ (\inst3|acc[17]~67\)))) # (GND)
-- \inst3|acc[18]~69\ = CARRY((\inst2|Ram0~7_combout\ & (\inst3|acc\(18) & !\inst3|acc[17]~67\)) # (!\inst2|Ram0~7_combout\ & ((\inst3|acc\(18)) # (!\inst3|acc[17]~67\))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1001011001001101",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst2|Ram0~7_combout\,
	datab => \inst3|acc\(18),
	datad => VCC,
	cin => \inst3|acc[17]~67\,
	combout => \inst3|acc[18]~68_combout\,
	cout => \inst3|acc[18]~69\);

-- Location: FF_X23_Y22_N5
\inst3|acc[18]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \CLK0~inputclkctrl_outclk\,
	d => \inst3|acc[18]~68_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst3|acc\(18));

-- Location: LCCOMB_X23_Y22_N6
\inst3|acc[19]~70\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst3|acc[19]~70_combout\ = (\inst3|acc\(19) & ((\inst2|Ram0~4_combout\ & (!\inst3|acc[18]~69\)) # (!\inst2|Ram0~4_combout\ & (\inst3|acc[18]~69\ & VCC)))) # (!\inst3|acc\(19) & ((\inst2|Ram0~4_combout\ & ((\inst3|acc[18]~69\) # (GND))) # 
-- (!\inst2|Ram0~4_combout\ & (!\inst3|acc[18]~69\))))
-- \inst3|acc[19]~71\ = CARRY((\inst3|acc\(19) & (\inst2|Ram0~4_combout\ & !\inst3|acc[18]~69\)) # (!\inst3|acc\(19) & ((\inst2|Ram0~4_combout\) # (!\inst3|acc[18]~69\))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0110100101001101",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst3|acc\(19),
	datab => \inst2|Ram0~4_combout\,
	datad => VCC,
	cin => \inst3|acc[18]~69\,
	combout => \inst3|acc[19]~70_combout\,
	cout => \inst3|acc[19]~71\);

-- Location: FF_X23_Y22_N7
\inst3|acc[19]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \CLK0~inputclkctrl_outclk\,
	d => \inst3|acc[19]~70_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst3|acc\(19));

-- Location: LCCOMB_X23_Y22_N8
\inst3|acc[20]~72\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst3|acc[20]~72_combout\ = ((\inst2|Ram0~1_combout\ $ (\inst3|acc\(20) $ (\inst3|acc[19]~71\)))) # (GND)
-- \inst3|acc[20]~73\ = CARRY((\inst2|Ram0~1_combout\ & (\inst3|acc\(20) & !\inst3|acc[19]~71\)) # (!\inst2|Ram0~1_combout\ & ((\inst3|acc\(20)) # (!\inst3|acc[19]~71\))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1001011001001101",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst2|Ram0~1_combout\,
	datab => \inst3|acc\(20),
	datad => VCC,
	cin => \inst3|acc[19]~71\,
	combout => \inst3|acc[20]~72_combout\,
	cout => \inst3|acc[20]~73\);

-- Location: FF_X23_Y22_N9
\inst3|acc[20]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \CLK0~inputclkctrl_outclk\,
	d => \inst3|acc[20]~72_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst3|acc\(20));

-- Location: LCCOMB_X23_Y22_N10
\inst3|acc[21]~74\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst3|acc[21]~74_combout\ = (\inst3|acc\(21) & ((\inst2|Ram0~0_combout\ & (!\inst3|acc[20]~73\)) # (!\inst2|Ram0~0_combout\ & (\inst3|acc[20]~73\ & VCC)))) # (!\inst3|acc\(21) & ((\inst2|Ram0~0_combout\ & ((\inst3|acc[20]~73\) # (GND))) # 
-- (!\inst2|Ram0~0_combout\ & (!\inst3|acc[20]~73\))))
-- \inst3|acc[21]~75\ = CARRY((\inst3|acc\(21) & (\inst2|Ram0~0_combout\ & !\inst3|acc[20]~73\)) # (!\inst3|acc\(21) & ((\inst2|Ram0~0_combout\) # (!\inst3|acc[20]~73\))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0110100101001101",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst3|acc\(21),
	datab => \inst2|Ram0~0_combout\,
	datad => VCC,
	cin => \inst3|acc[20]~73\,
	combout => \inst3|acc[21]~74_combout\,
	cout => \inst3|acc[21]~75\);

-- Location: FF_X23_Y22_N11
\inst3|acc[21]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \CLK0~inputclkctrl_outclk\,
	d => \inst3|acc[21]~74_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst3|acc\(21));

-- Location: LCCOMB_X23_Y22_N12
\inst3|acc[22]~76\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst3|acc[22]~76_combout\ = ((\inst3|acc\(22) $ (\inst2|Equal0~1_combout\ $ (\inst3|acc[21]~75\)))) # (GND)
-- \inst3|acc[22]~77\ = CARRY((\inst3|acc\(22) & ((!\inst3|acc[21]~75\) # (!\inst2|Equal0~1_combout\))) # (!\inst3|acc\(22) & (!\inst2|Equal0~1_combout\ & !\inst3|acc[21]~75\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1001011000101011",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst3|acc\(22),
	datab => \inst2|Equal0~1_combout\,
	datad => VCC,
	cin => \inst3|acc[21]~75\,
	combout => \inst3|acc[22]~76_combout\,
	cout => \inst3|acc[22]~77\);

-- Location: FF_X23_Y22_N13
\inst3|acc[22]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \CLK0~inputclkctrl_outclk\,
	d => \inst3|acc[22]~76_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst3|acc\(22));

-- Location: LCCOMB_X23_Y22_N14
\inst3|acc[23]~78\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst3|acc[23]~78_combout\ = (\inst3|acc\(23) & (\inst3|acc[22]~77\ & VCC)) # (!\inst3|acc\(23) & (!\inst3|acc[22]~77\))
-- \inst3|acc[23]~79\ = CARRY((!\inst3|acc\(23) & !\inst3|acc[22]~77\))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1100001100000011",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	datab => \inst3|acc\(23),
	datad => VCC,
	cin => \inst3|acc[22]~77\,
	combout => \inst3|acc[23]~78_combout\,
	cout => \inst3|acc[23]~79\);

-- Location: FF_X23_Y22_N15
\inst3|acc[23]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \CLK0~inputclkctrl_outclk\,
	d => \inst3|acc[23]~78_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst3|acc\(23));

-- Location: LCCOMB_X23_Y22_N16
\inst3|acc[24]~80\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst3|acc[24]~80_combout\ = (\inst3|acc\(24) & ((GND) # (!\inst3|acc[23]~79\))) # (!\inst3|acc\(24) & (\inst3|acc[23]~79\ $ (GND)))
-- \inst3|acc[24]~81\ = CARRY((\inst3|acc\(24)) # (!\inst3|acc[23]~79\))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0011110011001111",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	datab => \inst3|acc\(24),
	datad => VCC,
	cin => \inst3|acc[23]~79\,
	combout => \inst3|acc[24]~80_combout\,
	cout => \inst3|acc[24]~81\);

-- Location: FF_X23_Y22_N17
\inst3|acc[24]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \CLK0~inputclkctrl_outclk\,
	d => \inst3|acc[24]~80_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst3|acc\(24));

-- Location: LCCOMB_X23_Y22_N18
\inst3|acc[25]~82\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst3|acc[25]~82_combout\ = (\inst3|acc\(25) & (\inst3|acc[24]~81\ & VCC)) # (!\inst3|acc\(25) & (!\inst3|acc[24]~81\))
-- \inst3|acc[25]~83\ = CARRY((!\inst3|acc\(25) & !\inst3|acc[24]~81\))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1100001100000011",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	datab => \inst3|acc\(25),
	datad => VCC,
	cin => \inst3|acc[24]~81\,
	combout => \inst3|acc[25]~82_combout\,
	cout => \inst3|acc[25]~83\);

-- Location: FF_X23_Y22_N19
\inst3|acc[25]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \CLK0~inputclkctrl_outclk\,
	d => \inst3|acc[25]~82_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst3|acc\(25));

-- Location: LCCOMB_X23_Y22_N20
\inst3|acc[26]~84\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst3|acc[26]~84_combout\ = (\inst3|acc\(26) & ((GND) # (!\inst3|acc[25]~83\))) # (!\inst3|acc\(26) & (\inst3|acc[25]~83\ $ (GND)))
-- \inst3|acc[26]~85\ = CARRY((\inst3|acc\(26)) # (!\inst3|acc[25]~83\))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0011110011001111",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	datab => \inst3|acc\(26),
	datad => VCC,
	cin => \inst3|acc[25]~83\,
	combout => \inst3|acc[26]~84_combout\,
	cout => \inst3|acc[26]~85\);

-- Location: FF_X23_Y22_N21
\inst3|acc[26]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \CLK0~inputclkctrl_outclk\,
	d => \inst3|acc[26]~84_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst3|acc\(26));

-- Location: LCCOMB_X23_Y22_N22
\inst3|acc[27]~86\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst3|acc[27]~86_combout\ = (\inst3|acc\(27) & (\inst3|acc[26]~85\ & VCC)) # (!\inst3|acc\(27) & (!\inst3|acc[26]~85\))
-- \inst3|acc[27]~87\ = CARRY((!\inst3|acc\(27) & !\inst3|acc[26]~85\))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1010010100000101",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst3|acc\(27),
	datad => VCC,
	cin => \inst3|acc[26]~85\,
	combout => \inst3|acc[27]~86_combout\,
	cout => \inst3|acc[27]~87\);

-- Location: FF_X23_Y22_N23
\inst3|acc[27]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \CLK0~inputclkctrl_outclk\,
	d => \inst3|acc[27]~86_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst3|acc\(27));

-- Location: LCCOMB_X23_Y22_N24
\inst3|acc[28]~88\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst3|acc[28]~88_combout\ = (\inst3|acc\(28) & ((GND) # (!\inst3|acc[27]~87\))) # (!\inst3|acc\(28) & (\inst3|acc[27]~87\ $ (GND)))
-- \inst3|acc[28]~89\ = CARRY((\inst3|acc\(28)) # (!\inst3|acc[27]~87\))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0011110011001111",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	datab => \inst3|acc\(28),
	datad => VCC,
	cin => \inst3|acc[27]~87\,
	combout => \inst3|acc[28]~88_combout\,
	cout => \inst3|acc[28]~89\);

-- Location: FF_X23_Y22_N25
\inst3|acc[28]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \CLK0~inputclkctrl_outclk\,
	d => \inst3|acc[28]~88_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst3|acc\(28));

-- Location: LCCOMB_X23_Y22_N26
\inst3|acc[29]~90\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst3|acc[29]~90_combout\ = (\inst3|acc\(29) & (\inst3|acc[28]~89\ & VCC)) # (!\inst3|acc\(29) & (!\inst3|acc[28]~89\))
-- \inst3|acc[29]~91\ = CARRY((!\inst3|acc\(29) & !\inst3|acc[28]~89\))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1010010100000101",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst3|acc\(29),
	datad => VCC,
	cin => \inst3|acc[28]~89\,
	combout => \inst3|acc[29]~90_combout\,
	cout => \inst3|acc[29]~91\);

-- Location: FF_X23_Y22_N27
\inst3|acc[29]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \CLK0~inputclkctrl_outclk\,
	d => \inst3|acc[29]~90_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst3|acc\(29));

-- Location: LCCOMB_X23_Y22_N28
\inst3|acc[30]~92\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst3|acc[30]~92_combout\ = (\inst3|acc\(30) & ((GND) # (!\inst3|acc[29]~91\))) # (!\inst3|acc\(30) & (\inst3|acc[29]~91\ $ (GND)))
-- \inst3|acc[30]~93\ = CARRY((\inst3|acc\(30)) # (!\inst3|acc[29]~91\))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0011110011001111",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	datab => \inst3|acc\(30),
	datad => VCC,
	cin => \inst3|acc[29]~91\,
	combout => \inst3|acc[30]~92_combout\,
	cout => \inst3|acc[30]~93\);

-- Location: FF_X23_Y22_N29
\inst3|acc[30]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \CLK0~inputclkctrl_outclk\,
	d => \inst3|acc[30]~92_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst3|acc\(30));

-- Location: LCCOMB_X23_Y22_N30
\inst3|acc[31]~94\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst3|acc[31]~94_combout\ = \inst3|acc\(31) $ (!\inst3|acc[30]~93\)

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1010010110100101",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst3|acc\(31),
	cin => \inst3|acc[30]~93\,
	combout => \inst3|acc[31]~94_combout\);

-- Location: FF_X23_Y22_N31
\inst3|acc[31]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \CLK0~inputclkctrl_outclk\,
	d => \inst3|acc[31]~94_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst3|acc\(31));

-- Location: M9K_X27_Y14_N0
\inst|altsyncram_component|auto_generated|ram_block1a0\ : cycloneive_ram_block
-- pragma translate_off
GENERIC MAP (
	mem_init2 => X"001F00079001D80073001BC006C001A400660018C006000174005A0015C005400144004E0012C004800118004300100003D000EC0038000D80033000C4002E00",
	mem_init1 => X"0B000290009C00250008C002100078001C0006800190005C00150004C001200040000F00034000C0002800090002000070001800050001000030000C00020000400010000000000000000000000000000000000000000000000000000010000400020000C000300010000500018000700020000900028000C00034000F0004000120004C00150005C001900068001C0007800210008C00250009C0029000B0002E000C40033000D80038000EC003D0010000430011800480012C004E0014400540015C005A0017400600018C0066001A4006C001BC0073001D80079001F0007F00208008500220008B0023C00920025400980026C009E0028400A40029C00AA0",
	mem_init0 => X"02B400B0002CC00B6002E000BB002F800C10030C00C60032000CB0033400D00034800D50035C00D90036C00DD0038000E20039000E50039C00E9003AC00EC003B800EF003C400F2003D000F5003D800F7003E000F9003E800FB003EC00FC003F400FD003F800FE003F800FE003F800FF003F800FE003F800FE003F800FD003F400FC003EC00FB003E800F9003E000F7003D800F5003D000F2003C400EF003B800EC003AC00E90039C00E50039000E20038000DD0036C00D90035C00D50034800D00033400CB0032000C60030C00C1002F800BB002E000B6002CC00B0002B400AA0029C00A400284009E0026C00980025400920023C008B00220008500208007F",
	data_interleave_offset_in_bits => 1,
	data_interleave_width_in_bits => 1,
	init_file => "sindat.hex",
	init_file_layout => "port_a",
	logical_ram_name => "ROM:inst|altsyncram:altsyncram_component|altsyncram_v6r3:auto_generated|ALTSYNCRAM",
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
	clk0 => \CLK0~inputclkctrl_outclk\,
	portaaddr => \inst|altsyncram_component|auto_generated|ram_block1a0_PORTAADDR_bus\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	portadataout => \inst|altsyncram_component|auto_generated|ram_block1a0_PORTADATAOUT_bus\);

ww_L <= \L~output_o\;

ww_LEDA(6) <= \LEDA[6]~output_o\;

ww_LEDA(5) <= \LEDA[5]~output_o\;

ww_LEDA(4) <= \LEDA[4]~output_o\;

ww_LEDA(3) <= \LEDA[3]~output_o\;

ww_LEDA(2) <= \LEDA[2]~output_o\;

ww_LEDA(1) <= \LEDA[1]~output_o\;

ww_LEDA(0) <= \LEDA[0]~output_o\;

ww_LEDB(6) <= \LEDB[6]~output_o\;

ww_LEDB(5) <= \LEDB[5]~output_o\;

ww_LEDB(4) <= \LEDB[4]~output_o\;

ww_LEDB(3) <= \LEDB[3]~output_o\;

ww_LEDB(2) <= \LEDB[2]~output_o\;

ww_LEDB(1) <= \LEDB[1]~output_o\;

ww_LEDB(0) <= \LEDB[0]~output_o\;

ww_Q(7) <= \Q[7]~output_o\;

ww_Q(6) <= \Q[6]~output_o\;

ww_Q(5) <= \Q[5]~output_o\;

ww_Q(4) <= \Q[4]~output_o\;

ww_Q(3) <= \Q[3]~output_o\;

ww_Q(2) <= \Q[2]~output_o\;

ww_Q(1) <= \Q[1]~output_o\;

ww_Q(0) <= \Q[0]~output_o\;
END structure;


