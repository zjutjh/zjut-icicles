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

-- DATE "07/14/2020 14:01:27"

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

ENTITY 	add IS
    PORT (
	DACLK : OUT std_logic;
	CLK : IN std_logic;
	DAC : OUT std_logic_vector(7 DOWNTO 0);
	SW0 : IN std_logic;
	SW1 : IN std_logic;
	KEY0 : IN std_logic;
	LEDA : OUT std_logic_vector(6 DOWNTO 0);
	LEDB : OUT std_logic_vector(6 DOWNTO 0)
	);
END add;

-- Design Ports Information
-- DACLK	=>  Location: PIN_86,	 I/O Standard: 2.5 V,	 Current Strength: Default
-- DAC[7]	=>  Location: PIN_85,	 I/O Standard: 2.5 V,	 Current Strength: Default
-- DAC[6]	=>  Location: PIN_84,	 I/O Standard: 2.5 V,	 Current Strength: Default
-- DAC[5]	=>  Location: PIN_83,	 I/O Standard: 2.5 V,	 Current Strength: Default
-- DAC[4]	=>  Location: PIN_80,	 I/O Standard: 2.5 V,	 Current Strength: Default
-- DAC[3]	=>  Location: PIN_77,	 I/O Standard: 2.5 V,	 Current Strength: Default
-- DAC[2]	=>  Location: PIN_76,	 I/O Standard: 2.5 V,	 Current Strength: Default
-- DAC[1]	=>  Location: PIN_75,	 I/O Standard: 2.5 V,	 Current Strength: Default
-- DAC[0]	=>  Location: PIN_74,	 I/O Standard: 2.5 V,	 Current Strength: Default
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
-- CLK	=>  Location: PIN_24,	 I/O Standard: 2.5 V,	 Current Strength: Default
-- SW0	=>  Location: PIN_73,	 I/O Standard: 2.5 V,	 Current Strength: Default
-- SW1	=>  Location: PIN_72,	 I/O Standard: 2.5 V,	 Current Strength: Default
-- KEY0	=>  Location: PIN_90,	 I/O Standard: 2.5 V,	 Current Strength: Default


ARCHITECTURE structure OF add IS
SIGNAL gnd : std_logic := '0';
SIGNAL vcc : std_logic := '1';
SIGNAL unknown : std_logic := 'X';
SIGNAL devoe : std_logic := '1';
SIGNAL devclrn : std_logic := '1';
SIGNAL devpor : std_logic := '1';
SIGNAL ww_devoe : std_logic;
SIGNAL ww_devclrn : std_logic;
SIGNAL ww_devpor : std_logic;
SIGNAL ww_DACLK : std_logic;
SIGNAL ww_CLK : std_logic;
SIGNAL ww_DAC : std_logic_vector(7 DOWNTO 0);
SIGNAL ww_SW0 : std_logic;
SIGNAL ww_SW1 : std_logic;
SIGNAL ww_KEY0 : std_logic;
SIGNAL ww_LEDA : std_logic_vector(6 DOWNTO 0);
SIGNAL ww_LEDB : std_logic_vector(6 DOWNTO 0);
SIGNAL \inst4|altsyncram_component|auto_generated|ram_block1a0_PORTAADDR_bus\ : std_logic_vector(7 DOWNTO 0);
SIGNAL \inst4|altsyncram_component|auto_generated|ram_block1a0_PORTADATAOUT_bus\ : std_logic_vector(35 DOWNTO 0);
SIGNAL \inst~clkctrl_INCLK_bus\ : std_logic_vector(3 DOWNTO 0);
SIGNAL \CLK~inputclkctrl_INCLK_bus\ : std_logic_vector(3 DOWNTO 0);
SIGNAL \DACLK~output_o\ : std_logic;
SIGNAL \DAC[7]~output_o\ : std_logic;
SIGNAL \DAC[6]~output_o\ : std_logic;
SIGNAL \DAC[5]~output_o\ : std_logic;
SIGNAL \DAC[4]~output_o\ : std_logic;
SIGNAL \DAC[3]~output_o\ : std_logic;
SIGNAL \DAC[2]~output_o\ : std_logic;
SIGNAL \DAC[1]~output_o\ : std_logic;
SIGNAL \DAC[0]~output_o\ : std_logic;
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
SIGNAL \CLK~input_o\ : std_logic;
SIGNAL \CLK~inputclkctrl_outclk\ : std_logic;
SIGNAL \KEY0~input_o\ : std_logic;
SIGNAL \inst~feeder_combout\ : std_logic;
SIGNAL \inst~q\ : std_logic;
SIGNAL \inst~clkctrl_outclk\ : std_logic;
SIGNAL \inst5|Add0~0_combout\ : std_logic;
SIGNAL \inst5|Add0~7\ : std_logic;
SIGNAL \inst5|Add0~8_combout\ : std_logic;
SIGNAL \inst5|Equal0~0_combout\ : std_logic;
SIGNAL \inst5|q~0_combout\ : std_logic;
SIGNAL \inst5|Add0~1\ : std_logic;
SIGNAL \inst5|Add0~2_combout\ : std_logic;
SIGNAL \inst5|Add0~3\ : std_logic;
SIGNAL \inst5|Add0~4_combout\ : std_logic;
SIGNAL \inst5|Add0~5\ : std_logic;
SIGNAL \inst5|Add0~6_combout\ : std_logic;
SIGNAL \inst5|Equal0~1_combout\ : std_logic;
SIGNAL \inst5|Ram0~0_combout\ : std_logic;
SIGNAL \inst5|Ram0~1_combout\ : std_logic;
SIGNAL \inst5|Ram0~3_combout\ : std_logic;
SIGNAL \inst5|Ram0~2_combout\ : std_logic;
SIGNAL \inst5|Ram0~4_combout\ : std_logic;
SIGNAL \inst5|Ram0~6_combout\ : std_logic;
SIGNAL \inst5|Ram0~5_combout\ : std_logic;
SIGNAL \inst5|Ram0~7_combout\ : std_logic;
SIGNAL \inst5|Ram0~8_combout\ : std_logic;
SIGNAL \inst5|Ram0~9_combout\ : std_logic;
SIGNAL \inst5|Ram0~10_combout\ : std_logic;
SIGNAL \inst5|Ram0~11_combout\ : std_logic;
SIGNAL \inst5|Ram0~12_combout\ : std_logic;
SIGNAL \inst5|Ram0~13_combout\ : std_logic;
SIGNAL \inst5|Ram0~14_combout\ : std_logic;
SIGNAL \inst5|Ram0~15_combout\ : std_logic;
SIGNAL \inst5|Ram0~16_combout\ : std_logic;
SIGNAL \inst5|Ram0~17_combout\ : std_logic;
SIGNAL \inst5|Ram0~18_combout\ : std_logic;
SIGNAL \inst5|Ram0~30_combout\ : std_logic;
SIGNAL \inst5|Ram0~31_combout\ : std_logic;
SIGNAL \inst5|Ram0~28_combout\ : std_logic;
SIGNAL \inst5|Ram0~29_combout\ : std_logic;
SIGNAL \inst5|Ram0~20_combout\ : std_logic;
SIGNAL \inst5|Ram0~19_combout\ : std_logic;
SIGNAL \inst5|Ram0~21_combout\ : std_logic;
SIGNAL \inst5|Ram0~22_combout\ : std_logic;
SIGNAL \inst5|Ram0~23_combout\ : std_logic;
SIGNAL \inst5|Ram0~24_combout\ : std_logic;
SIGNAL \inst5|Ram0~25_combout\ : std_logic;
SIGNAL \inst5|Ram0~26_combout\ : std_logic;
SIGNAL \inst5|Ram0~27_combout\ : std_logic;
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
SIGNAL \SW0~input_o\ : std_logic;
SIGNAL \SW1~input_o\ : std_logic;
SIGNAL \inst2|OU[7]~0_combout\ : std_logic;
SIGNAL \inst2|OU[7]~1_combout\ : std_logic;
SIGNAL \inst2|OU[6]~2_combout\ : std_logic;
SIGNAL \inst2|OU[6]~3_combout\ : std_logic;
SIGNAL \inst2|OU[5]~4_combout\ : std_logic;
SIGNAL \inst2|OU[5]~5_combout\ : std_logic;
SIGNAL \inst2|OU[4]~6_combout\ : std_logic;
SIGNAL \inst2|OU[4]~7_combout\ : std_logic;
SIGNAL \inst2|OU[3]~8_combout\ : std_logic;
SIGNAL \inst2|OU[3]~9_combout\ : std_logic;
SIGNAL \inst2|OU[2]~10_combout\ : std_logic;
SIGNAL \inst2|OU[2]~11_combout\ : std_logic;
SIGNAL \inst2|OU[1]~12_combout\ : std_logic;
SIGNAL \inst2|OU[1]~13_combout\ : std_logic;
SIGNAL \inst2|OU[0]~14_combout\ : std_logic;
SIGNAL \inst2|OU[0]~15_combout\ : std_logic;
SIGNAL \inst5|Mux0~1_combout\ : std_logic;
SIGNAL \inst5|Mux0~0_combout\ : std_logic;
SIGNAL \inst5|Mux0~2_combout\ : std_logic;
SIGNAL \inst5|Mux1~1_combout\ : std_logic;
SIGNAL \inst5|Mux1~0_combout\ : std_logic;
SIGNAL \inst5|Mux1~2_combout\ : std_logic;
SIGNAL \inst5|Mux2~0_combout\ : std_logic;
SIGNAL \inst5|Mux2~1_combout\ : std_logic;
SIGNAL \inst5|Mux3~0_combout\ : std_logic;
SIGNAL \inst5|Mux4~0_combout\ : std_logic;
SIGNAL \inst5|Mux5~0_combout\ : std_logic;
SIGNAL \inst5|Mux6~0_combout\ : std_logic;
SIGNAL \inst5|Mux6~1_combout\ : std_logic;
SIGNAL \inst5|Mux7~0_combout\ : std_logic;
SIGNAL \inst5|Mux7~1_combout\ : std_logic;
SIGNAL \inst5|Mux8~0_combout\ : std_logic;
SIGNAL \inst5|Mux8~1_combout\ : std_logic;
SIGNAL \inst5|Mux9~0_combout\ : std_logic;
SIGNAL \inst5|Mux9~1_combout\ : std_logic;
SIGNAL \inst5|Mux10~0_combout\ : std_logic;
SIGNAL \inst5|Mux11~0_combout\ : std_logic;
SIGNAL \inst5|Mux11~1_combout\ : std_logic;
SIGNAL \inst6|altsyncram_component|auto_generated|q_a\ : std_logic_vector(7 DOWNTO 0);
SIGNAL \inst4|altsyncram_component|auto_generated|q_a\ : std_logic_vector(7 DOWNTO 0);
SIGNAL \inst7|altsyncram_component|auto_generated|q_a\ : std_logic_vector(7 DOWNTO 0);
SIGNAL \inst3|acc\ : std_logic_vector(31 DOWNTO 0);
SIGNAL \inst5|q\ : std_logic_vector(4 DOWNTO 0);
SIGNAL \inst5|ALT_INV_Mux9~1_combout\ : std_logic;
SIGNAL \inst5|ALT_INV_Mux8~1_combout\ : std_logic;
SIGNAL \inst5|ALT_INV_Mux5~0_combout\ : std_logic;
SIGNAL \ALT_INV_inst~clkctrl_outclk\ : std_logic;
SIGNAL \ALT_INV_CLK~input_o\ : std_logic;

BEGIN

DACLK <= ww_DACLK;
ww_CLK <= CLK;
DAC <= ww_DAC;
ww_SW0 <= SW0;
ww_SW1 <= SW1;
ww_KEY0 <= KEY0;
LEDA <= ww_LEDA;
LEDB <= ww_LEDB;
ww_devoe <= devoe;
ww_devclrn <= devclrn;
ww_devpor <= devpor;

\inst4|altsyncram_component|auto_generated|ram_block1a0_PORTAADDR_bus\ <= (\inst3|acc\(31) & \inst3|acc\(30) & \inst3|acc\(29) & \inst3|acc\(28) & \inst3|acc\(27) & \inst3|acc\(26) & \inst3|acc\(25) & \inst3|acc\(24));

\inst4|altsyncram_component|auto_generated|q_a\(0) <= \inst4|altsyncram_component|auto_generated|ram_block1a0_PORTADATAOUT_bus\(0);
\inst4|altsyncram_component|auto_generated|q_a\(1) <= \inst4|altsyncram_component|auto_generated|ram_block1a0_PORTADATAOUT_bus\(1);
\inst4|altsyncram_component|auto_generated|q_a\(2) <= \inst4|altsyncram_component|auto_generated|ram_block1a0_PORTADATAOUT_bus\(2);
\inst4|altsyncram_component|auto_generated|q_a\(3) <= \inst4|altsyncram_component|auto_generated|ram_block1a0_PORTADATAOUT_bus\(3);
\inst4|altsyncram_component|auto_generated|q_a\(4) <= \inst4|altsyncram_component|auto_generated|ram_block1a0_PORTADATAOUT_bus\(4);
\inst4|altsyncram_component|auto_generated|q_a\(5) <= \inst4|altsyncram_component|auto_generated|ram_block1a0_PORTADATAOUT_bus\(5);
\inst4|altsyncram_component|auto_generated|q_a\(6) <= \inst4|altsyncram_component|auto_generated|ram_block1a0_PORTADATAOUT_bus\(6);
\inst4|altsyncram_component|auto_generated|q_a\(7) <= \inst4|altsyncram_component|auto_generated|ram_block1a0_PORTADATAOUT_bus\(7);
\inst6|altsyncram_component|auto_generated|q_a\(0) <= \inst4|altsyncram_component|auto_generated|ram_block1a0_PORTADATAOUT_bus\(8);
\inst6|altsyncram_component|auto_generated|q_a\(1) <= \inst4|altsyncram_component|auto_generated|ram_block1a0_PORTADATAOUT_bus\(9);
\inst6|altsyncram_component|auto_generated|q_a\(2) <= \inst4|altsyncram_component|auto_generated|ram_block1a0_PORTADATAOUT_bus\(10);
\inst6|altsyncram_component|auto_generated|q_a\(3) <= \inst4|altsyncram_component|auto_generated|ram_block1a0_PORTADATAOUT_bus\(11);
\inst6|altsyncram_component|auto_generated|q_a\(4) <= \inst4|altsyncram_component|auto_generated|ram_block1a0_PORTADATAOUT_bus\(12);
\inst6|altsyncram_component|auto_generated|q_a\(5) <= \inst4|altsyncram_component|auto_generated|ram_block1a0_PORTADATAOUT_bus\(13);
\inst6|altsyncram_component|auto_generated|q_a\(6) <= \inst4|altsyncram_component|auto_generated|ram_block1a0_PORTADATAOUT_bus\(14);
\inst6|altsyncram_component|auto_generated|q_a\(7) <= \inst4|altsyncram_component|auto_generated|ram_block1a0_PORTADATAOUT_bus\(15);
\inst7|altsyncram_component|auto_generated|q_a\(0) <= \inst4|altsyncram_component|auto_generated|ram_block1a0_PORTADATAOUT_bus\(16);
\inst7|altsyncram_component|auto_generated|q_a\(1) <= \inst4|altsyncram_component|auto_generated|ram_block1a0_PORTADATAOUT_bus\(17);
\inst7|altsyncram_component|auto_generated|q_a\(2) <= \inst4|altsyncram_component|auto_generated|ram_block1a0_PORTADATAOUT_bus\(18);
\inst7|altsyncram_component|auto_generated|q_a\(3) <= \inst4|altsyncram_component|auto_generated|ram_block1a0_PORTADATAOUT_bus\(19);
\inst7|altsyncram_component|auto_generated|q_a\(4) <= \inst4|altsyncram_component|auto_generated|ram_block1a0_PORTADATAOUT_bus\(20);
\inst7|altsyncram_component|auto_generated|q_a\(5) <= \inst4|altsyncram_component|auto_generated|ram_block1a0_PORTADATAOUT_bus\(21);
\inst7|altsyncram_component|auto_generated|q_a\(6) <= \inst4|altsyncram_component|auto_generated|ram_block1a0_PORTADATAOUT_bus\(22);
\inst7|altsyncram_component|auto_generated|q_a\(7) <= \inst4|altsyncram_component|auto_generated|ram_block1a0_PORTADATAOUT_bus\(23);

\inst~clkctrl_INCLK_bus\ <= (vcc & vcc & vcc & \inst~q\);

\CLK~inputclkctrl_INCLK_bus\ <= (vcc & vcc & vcc & \CLK~input_o\);
\inst5|ALT_INV_Mux9~1_combout\ <= NOT \inst5|Mux9~1_combout\;
\inst5|ALT_INV_Mux8~1_combout\ <= NOT \inst5|Mux8~1_combout\;
\inst5|ALT_INV_Mux5~0_combout\ <= NOT \inst5|Mux5~0_combout\;
\ALT_INV_inst~clkctrl_outclk\ <= NOT \inst~clkctrl_outclk\;
\ALT_INV_CLK~input_o\ <= NOT \CLK~input_o\;

-- Location: IOOBUF_X34_Y9_N2
\DACLK~output\ : cycloneive_io_obuf
-- pragma translate_off
GENERIC MAP (
	bus_hold => "false",
	open_drain_output => "false")
-- pragma translate_on
PORT MAP (
	i => \ALT_INV_CLK~input_o\,
	devoe => ww_devoe,
	o => \DACLK~output_o\);

-- Location: IOOBUF_X34_Y9_N9
\DAC[7]~output\ : cycloneive_io_obuf
-- pragma translate_off
GENERIC MAP (
	bus_hold => "false",
	open_drain_output => "false")
-- pragma translate_on
PORT MAP (
	i => \inst2|OU[7]~1_combout\,
	devoe => ww_devoe,
	o => \DAC[7]~output_o\);

-- Location: IOOBUF_X34_Y9_N16
\DAC[6]~output\ : cycloneive_io_obuf
-- pragma translate_off
GENERIC MAP (
	bus_hold => "false",
	open_drain_output => "false")
-- pragma translate_on
PORT MAP (
	i => \inst2|OU[6]~3_combout\,
	devoe => ww_devoe,
	o => \DAC[6]~output_o\);

-- Location: IOOBUF_X34_Y9_N23
\DAC[5]~output\ : cycloneive_io_obuf
-- pragma translate_off
GENERIC MAP (
	bus_hold => "false",
	open_drain_output => "false")
-- pragma translate_on
PORT MAP (
	i => \inst2|OU[5]~5_combout\,
	devoe => ww_devoe,
	o => \DAC[5]~output_o\);

-- Location: IOOBUF_X34_Y7_N9
\DAC[4]~output\ : cycloneive_io_obuf
-- pragma translate_off
GENERIC MAP (
	bus_hold => "false",
	open_drain_output => "false")
-- pragma translate_on
PORT MAP (
	i => \inst2|OU[4]~7_combout\,
	devoe => ww_devoe,
	o => \DAC[4]~output_o\);

-- Location: IOOBUF_X34_Y4_N16
\DAC[3]~output\ : cycloneive_io_obuf
-- pragma translate_off
GENERIC MAP (
	bus_hold => "false",
	open_drain_output => "false")
-- pragma translate_on
PORT MAP (
	i => \inst2|OU[3]~9_combout\,
	devoe => ww_devoe,
	o => \DAC[3]~output_o\);

-- Location: IOOBUF_X34_Y4_N23
\DAC[2]~output\ : cycloneive_io_obuf
-- pragma translate_off
GENERIC MAP (
	bus_hold => "false",
	open_drain_output => "false")
-- pragma translate_on
PORT MAP (
	i => \inst2|OU[2]~11_combout\,
	devoe => ww_devoe,
	o => \DAC[2]~output_o\);

-- Location: IOOBUF_X34_Y3_N23
\DAC[1]~output\ : cycloneive_io_obuf
-- pragma translate_off
GENERIC MAP (
	bus_hold => "false",
	open_drain_output => "false")
-- pragma translate_on
PORT MAP (
	i => \inst2|OU[1]~13_combout\,
	devoe => ww_devoe,
	o => \DAC[1]~output_o\);

-- Location: IOOBUF_X34_Y2_N16
\DAC[0]~output\ : cycloneive_io_obuf
-- pragma translate_off
GENERIC MAP (
	bus_hold => "false",
	open_drain_output => "false")
-- pragma translate_on
PORT MAP (
	i => \inst2|OU[0]~15_combout\,
	devoe => ww_devoe,
	o => \DAC[0]~output_o\);

-- Location: IOOBUF_X18_Y24_N16
\LEDA[6]~output\ : cycloneive_io_obuf
-- pragma translate_off
GENERIC MAP (
	bus_hold => "false",
	open_drain_output => "false")
-- pragma translate_on
PORT MAP (
	i => \inst5|Mux0~2_combout\,
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
	i => \inst5|Mux1~2_combout\,
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
	i => \inst5|Mux2~1_combout\,
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
	i => \inst5|Mux3~0_combout\,
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
	i => \inst5|Mux4~0_combout\,
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
	i => \inst5|ALT_INV_Mux5~0_combout\,
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
	i => \inst5|Mux6~1_combout\,
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
	i => \inst5|Mux7~1_combout\,
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
	i => \inst5|ALT_INV_Mux8~1_combout\,
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
	i => \inst5|ALT_INV_Mux9~1_combout\,
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
	i => \inst5|Mux10~0_combout\,
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
	i => \inst5|Mux11~1_combout\,
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
	i => \inst5|Mux10~0_combout\,
	devoe => ww_devoe,
	o => \LEDB[0]~output_o\);

-- Location: IOIBUF_X0_Y11_N15
\CLK~input\ : cycloneive_io_ibuf
-- pragma translate_off
GENERIC MAP (
	bus_hold => "false",
	simulate_z_as => "z")
-- pragma translate_on
PORT MAP (
	i => ww_CLK,
	o => \CLK~input_o\);

-- Location: CLKCTRL_G4
\CLK~inputclkctrl\ : cycloneive_clkctrl
-- pragma translate_off
GENERIC MAP (
	clock_type => "global clock",
	ena_register_mode => "none")
-- pragma translate_on
PORT MAP (
	inclk => \CLK~inputclkctrl_INCLK_bus\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	outclk => \CLK~inputclkctrl_outclk\);

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
\inst~feeder\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst~feeder_combout\ = \KEY0~input_o\

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1111111100000000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	datad => \KEY0~input_o\,
	combout => \inst~feeder_combout\);

-- Location: FF_X33_Y12_N3
inst : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \CLK~inputclkctrl_outclk\,
	d => \inst~feeder_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst~q\);

-- Location: CLKCTRL_G7
\inst~clkctrl\ : cycloneive_clkctrl
-- pragma translate_off
GENERIC MAP (
	clock_type => "global clock",
	ena_register_mode => "none")
-- pragma translate_on
PORT MAP (
	inclk => \inst~clkctrl_INCLK_bus\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	outclk => \inst~clkctrl_outclk\);

-- Location: LCCOMB_X19_Y22_N16
\inst5|Add0~0\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst5|Add0~0_combout\ = \inst5|q\(0) $ (VCC)
-- \inst5|Add0~1\ = CARRY(\inst5|q\(0))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0101010110101010",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst5|q\(0),
	datad => VCC,
	combout => \inst5|Add0~0_combout\,
	cout => \inst5|Add0~1\);

-- Location: LCCOMB_X19_Y22_N22
\inst5|Add0~6\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst5|Add0~6_combout\ = (\inst5|q\(3) & (!\inst5|Add0~5\)) # (!\inst5|q\(3) & ((\inst5|Add0~5\) # (GND)))
-- \inst5|Add0~7\ = CARRY((!\inst5|Add0~5\) # (!\inst5|q\(3)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0101101001011111",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst5|q\(3),
	datad => VCC,
	cin => \inst5|Add0~5\,
	combout => \inst5|Add0~6_combout\,
	cout => \inst5|Add0~7\);

-- Location: LCCOMB_X19_Y22_N24
\inst5|Add0~8\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst5|Add0~8_combout\ = \inst5|q\(4) $ (!\inst5|Add0~7\)

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1010010110100101",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst5|q\(4),
	cin => \inst5|Add0~7\,
	combout => \inst5|Add0~8_combout\);

-- Location: FF_X19_Y22_N25
\inst5|q[4]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \ALT_INV_inst~clkctrl_outclk\,
	d => \inst5|Add0~8_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst5|q\(4));

-- Location: LCCOMB_X19_Y22_N10
\inst5|Equal0~0\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst5|Equal0~0_combout\ = (\inst5|q\(1) & (\inst5|q\(2) & (\inst5|q\(3) & \inst5|q\(4))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1000000000000000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst5|q\(1),
	datab => \inst5|q\(2),
	datac => \inst5|q\(3),
	datad => \inst5|q\(4),
	combout => \inst5|Equal0~0_combout\);

-- Location: LCCOMB_X19_Y22_N12
\inst5|q~0\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst5|q~0_combout\ = \inst5|Add0~0_combout\ $ (((!\inst5|q\(0) & \inst5|Equal0~0_combout\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1100001111001100",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	datab => \inst5|Add0~0_combout\,
	datac => \inst5|q\(0),
	datad => \inst5|Equal0~0_combout\,
	combout => \inst5|q~0_combout\);

-- Location: FF_X19_Y22_N13
\inst5|q[0]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \ALT_INV_inst~clkctrl_outclk\,
	d => \inst5|q~0_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst5|q\(0));

-- Location: LCCOMB_X19_Y22_N18
\inst5|Add0~2\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst5|Add0~2_combout\ = (\inst5|q\(1) & (\inst5|Add0~1\ & VCC)) # (!\inst5|q\(1) & (!\inst5|Add0~1\))
-- \inst5|Add0~3\ = CARRY((!\inst5|q\(1) & !\inst5|Add0~1\))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1100001100000011",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	datab => \inst5|q\(1),
	datad => VCC,
	cin => \inst5|Add0~1\,
	combout => \inst5|Add0~2_combout\,
	cout => \inst5|Add0~3\);

-- Location: FF_X19_Y22_N19
\inst5|q[1]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \ALT_INV_inst~clkctrl_outclk\,
	d => \inst5|Add0~2_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst5|q\(1));

-- Location: LCCOMB_X19_Y22_N20
\inst5|Add0~4\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst5|Add0~4_combout\ = (\inst5|q\(2) & (\inst5|Add0~3\ $ (GND))) # (!\inst5|q\(2) & (!\inst5|Add0~3\ & VCC))
-- \inst5|Add0~5\ = CARRY((\inst5|q\(2) & !\inst5|Add0~3\))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1100001100001100",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	datab => \inst5|q\(2),
	datad => VCC,
	cin => \inst5|Add0~3\,
	combout => \inst5|Add0~4_combout\,
	cout => \inst5|Add0~5\);

-- Location: FF_X19_Y22_N21
\inst5|q[2]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \ALT_INV_inst~clkctrl_outclk\,
	d => \inst5|Add0~4_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst5|q\(2));

-- Location: FF_X19_Y22_N23
\inst5|q[3]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \ALT_INV_inst~clkctrl_outclk\,
	d => \inst5|Add0~6_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst5|q\(3));

-- Location: LCCOMB_X18_Y21_N28
\inst5|Equal0~1\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst5|Equal0~1_combout\ = (\inst5|q\(3) & \inst5|q\(4))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1100110000000000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	datab => \inst5|q\(3),
	datad => \inst5|q\(4),
	combout => \inst5|Equal0~1_combout\);

-- Location: LCCOMB_X18_Y21_N10
\inst5|Ram0~0\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst5|Ram0~0_combout\ = (\inst5|q\(3) & (\inst5|q\(2) & !\inst5|q\(4))) # (!\inst5|q\(3) & ((\inst5|q\(4))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0011001111000000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	datab => \inst5|q\(3),
	datac => \inst5|q\(2),
	datad => \inst5|q\(4),
	combout => \inst5|Ram0~0_combout\);

-- Location: LCCOMB_X18_Y21_N20
\inst5|Ram0~1\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst5|Ram0~1_combout\ = (\inst5|q\(3) & ((\inst5|q\(2) & (\inst5|q\(1) & \inst5|q\(4))) # (!\inst5|q\(2) & ((!\inst5|q\(4)))))) # (!\inst5|q\(3) & ((\inst5|q\(1) & ((\inst5|q\(2)) # (\inst5|q\(4)))) # (!\inst5|q\(1) & (\inst5|q\(2) & \inst5|q\(4)))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1011001000101100",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst5|q\(1),
	datab => \inst5|q\(3),
	datac => \inst5|q\(2),
	datad => \inst5|q\(4),
	combout => \inst5|Ram0~1_combout\);

-- Location: LCCOMB_X16_Y22_N14
\inst5|Ram0~3\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst5|Ram0~3_combout\ = (\inst5|q\(4) & ((\inst5|q\(2) & (\inst5|q\(3) $ (\inst5|q\(1)))) # (!\inst5|q\(2) & (!\inst5|q\(3) & !\inst5|q\(1))))) # (!\inst5|q\(4) & ((\inst5|q\(2) & (!\inst5|q\(3) & !\inst5|q\(1))) # (!\inst5|q\(2) & (\inst5|q\(3) & 
-- \inst5|q\(1)))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0001100010000110",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst5|q\(4),
	datab => \inst5|q\(2),
	datac => \inst5|q\(3),
	datad => \inst5|q\(1),
	combout => \inst5|Ram0~3_combout\);

-- Location: LCCOMB_X16_Y22_N12
\inst5|Ram0~2\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst5|Ram0~2_combout\ = (\inst5|q\(3) & ((\inst5|q\(2) & (\inst5|q\(4) $ (\inst5|q\(1)))) # (!\inst5|q\(2) & ((\inst5|q\(1)) # (!\inst5|q\(4)))))) # (!\inst5|q\(3) & ((\inst5|q\(2) & ((\inst5|q\(4)) # (!\inst5|q\(1)))) # (!\inst5|q\(2) & (\inst5|q\(4) $ 
-- (\inst5|q\(1))))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0110101111010110",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst5|q\(3),
	datab => \inst5|q\(2),
	datac => \inst5|q\(4),
	datad => \inst5|q\(1),
	combout => \inst5|Ram0~2_combout\);

-- Location: LCCOMB_X16_Y22_N20
\inst5|Ram0~4\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst5|Ram0~4_combout\ = (\inst5|q\(0) & ((\inst5|Ram0~2_combout\))) # (!\inst5|q\(0) & (\inst5|Ram0~3_combout\))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1111110000110000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	datab => \inst5|q\(0),
	datac => \inst5|Ram0~3_combout\,
	datad => \inst5|Ram0~2_combout\,
	combout => \inst5|Ram0~4_combout\);

-- Location: LCCOMB_X18_Y22_N22
\inst5|Ram0~6\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst5|Ram0~6_combout\ = (\inst5|q\(2) & (\inst5|q\(1) $ (((!\inst5|q\(0) & !\inst5|q\(4)))))) # (!\inst5|q\(2) & ((\inst5|q\(1) & (\inst5|q\(0) & \inst5|q\(4))) # (!\inst5|q\(1) & (\inst5|q\(0) $ (\inst5|q\(4))))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1100100110010010",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst5|q\(2),
	datab => \inst5|q\(1),
	datac => \inst5|q\(0),
	datad => \inst5|q\(4),
	combout => \inst5|Ram0~6_combout\);

-- Location: LCCOMB_X18_Y22_N16
\inst5|Ram0~5\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst5|Ram0~5_combout\ = (\inst5|q\(2) & ((\inst5|q\(1) & ((\inst5|q\(0)) # (\inst5|q\(4)))) # (!\inst5|q\(1) & ((!\inst5|q\(4)) # (!\inst5|q\(0)))))) # (!\inst5|q\(2) & ((\inst5|q\(1) & (\inst5|q\(0) $ (!\inst5|q\(4)))) # (!\inst5|q\(1) & ((\inst5|q\(0)) 
-- # (\inst5|q\(4))))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1101101110110110",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst5|q\(2),
	datab => \inst5|q\(1),
	datac => \inst5|q\(0),
	datad => \inst5|q\(4),
	combout => \inst5|Ram0~5_combout\);

-- Location: LCCOMB_X18_Y22_N0
\inst5|Ram0~7\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst5|Ram0~7_combout\ = (\inst5|q\(3) & (!\inst5|Ram0~6_combout\)) # (!\inst5|q\(3) & ((\inst5|Ram0~5_combout\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0011111100001100",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	datab => \inst5|q\(3),
	datac => \inst5|Ram0~6_combout\,
	datad => \inst5|Ram0~5_combout\,
	combout => \inst5|Ram0~7_combout\);

-- Location: LCCOMB_X18_Y22_N6
\inst5|Ram0~8\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst5|Ram0~8_combout\ = (\inst5|q\(2) & (\inst5|q\(0) $ (((!\inst5|q\(1) & !\inst5|q\(3)))))) # (!\inst5|q\(2) & ((\inst5|q\(1) & (!\inst5|q\(0) & \inst5|q\(3))) # (!\inst5|q\(1) & (\inst5|q\(0) & !\inst5|q\(3)))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1010010010010010",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst5|q\(2),
	datab => \inst5|q\(1),
	datac => \inst5|q\(0),
	datad => \inst5|q\(3),
	combout => \inst5|Ram0~8_combout\);

-- Location: LCCOMB_X18_Y22_N24
\inst5|Ram0~9\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst5|Ram0~9_combout\ = \inst5|q\(4) $ (\inst5|Ram0~8_combout\)

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0011001111001100",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	datab => \inst5|q\(4),
	datad => \inst5|Ram0~8_combout\,
	combout => \inst5|Ram0~9_combout\);

-- Location: LCCOMB_X19_Y22_N4
\inst5|Ram0~10\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst5|Ram0~10_combout\ = \inst5|q\(3) $ (\inst5|q\(1) $ (((\inst5|q\(0)) # (\inst5|q\(2)))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1010010110010110",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst5|q\(3),
	datab => \inst5|q\(0),
	datac => \inst5|q\(1),
	datad => \inst5|q\(2),
	combout => \inst5|Ram0~10_combout\);

-- Location: LCCOMB_X18_Y22_N26
\inst5|Ram0~11\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst5|Ram0~11_combout\ = \inst5|q\(0) $ (\inst5|q\(2))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0011110000111100",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	datab => \inst5|q\(0),
	datac => \inst5|q\(2),
	combout => \inst5|Ram0~11_combout\);

-- Location: LCCOMB_X16_Y22_N10
\inst5|Ram0~12\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst5|Ram0~12_combout\ = (!\inst5|q\(1) & !\inst5|q\(2))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0000000000001111",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	datac => \inst5|q\(1),
	datad => \inst5|q\(2),
	combout => \inst5|Ram0~12_combout\);

-- Location: LCCOMB_X16_Y22_N16
\inst5|Ram0~13\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst5|Ram0~13_combout\ = ((!\inst5|q\(3) & (!\inst5|q\(0) & \inst5|Ram0~12_combout\))) # (!\inst5|q\(4))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0001111100001111",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst5|q\(3),
	datab => \inst5|q\(0),
	datac => \inst5|q\(4),
	datad => \inst5|Ram0~12_combout\,
	combout => \inst5|Ram0~13_combout\);

-- Location: LCCOMB_X16_Y22_N6
\inst5|Ram0~14\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst5|Ram0~14_combout\ = (\inst5|q\(3) & (((\inst5|q\(4) & \inst5|Ram0~12_combout\)))) # (!\inst5|q\(3) & ((\inst5|q\(0)) # ((!\inst5|Ram0~12_combout\) # (!\inst5|q\(4)))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1110010101010101",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst5|q\(3),
	datab => \inst5|q\(0),
	datac => \inst5|q\(4),
	datad => \inst5|Ram0~12_combout\,
	combout => \inst5|Ram0~14_combout\);

-- Location: LCCOMB_X18_Y22_N4
\inst5|Ram0~15\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst5|Ram0~15_combout\ = (\inst5|q\(3) & (!\inst5|q\(0) & !\inst5|q\(4))) # (!\inst5|q\(3) & (\inst5|q\(0) & \inst5|q\(4)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0011000000001100",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	datab => \inst5|q\(3),
	datac => \inst5|q\(0),
	datad => \inst5|q\(4),
	combout => \inst5|Ram0~15_combout\);

-- Location: LCCOMB_X18_Y22_N2
\inst5|Ram0~16\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst5|Ram0~16_combout\ = (\inst5|q\(2) & (!\inst5|q\(1) & (\inst5|q\(4) $ (\inst5|Ram0~15_combout\)))) # (!\inst5|q\(2) & (((\inst5|Ram0~15_combout\) # (\inst5|q\(1))) # (!\inst5|q\(4))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0101010101111001",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst5|q\(2),
	datab => \inst5|q\(4),
	datac => \inst5|Ram0~15_combout\,
	datad => \inst5|q\(1),
	combout => \inst5|Ram0~16_combout\);

-- Location: LCCOMB_X18_Y22_N8
\inst5|Ram0~17\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst5|Ram0~17_combout\ = (\inst5|q\(4) & (\inst5|q\(0) & !\inst5|q\(3))) # (!\inst5|q\(4) & (!\inst5|q\(0) & \inst5|q\(3)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0000001111000000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	datab => \inst5|q\(4),
	datac => \inst5|q\(0),
	datad => \inst5|q\(3),
	combout => \inst5|Ram0~17_combout\);

-- Location: LCCOMB_X18_Y22_N30
\inst5|Ram0~18\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst5|Ram0~18_combout\ = (\inst5|Ram0~17_combout\ & ((\inst5|q\(2) & ((\inst5|q\(4)) # (\inst5|q\(1)))) # (!\inst5|q\(2) & ((!\inst5|q\(1)))))) # (!\inst5|Ram0~17_combout\ & ((\inst5|q\(4) $ (!\inst5|q\(1)))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1010110011010011",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst5|q\(2),
	datab => \inst5|q\(4),
	datac => \inst5|Ram0~17_combout\,
	datad => \inst5|q\(1),
	combout => \inst5|Ram0~18_combout\);

-- Location: LCCOMB_X19_Y22_N8
\inst5|Ram0~30\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst5|Ram0~30_combout\ = (\inst5|q\(4) & (\inst5|q\(2) & ((\inst5|q\(3)) # (\inst5|q\(1))))) # (!\inst5|q\(4) & (((\inst5|q\(1) & !\inst5|q\(2))) # (!\inst5|q\(3))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1110010100001101",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst5|q\(3),
	datab => \inst5|q\(1),
	datac => \inst5|q\(4),
	datad => \inst5|q\(2),
	combout => \inst5|Ram0~30_combout\);

-- Location: LCCOMB_X19_Y22_N6
\inst5|Ram0~31\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst5|Ram0~31_combout\ = (\inst5|q\(0) & (\inst5|Ram0~30_combout\ $ (((\inst5|q\(2)) # (!\inst5|q\(3)))))) # (!\inst5|q\(0) & ((\inst5|Ram0~30_combout\) # ((\inst5|q\(3) & !\inst5|q\(2)))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0011110010110110",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst5|q\(3),
	datab => \inst5|q\(0),
	datac => \inst5|Ram0~30_combout\,
	datad => \inst5|q\(2),
	combout => \inst5|Ram0~31_combout\);

-- Location: LCCOMB_X18_Y22_N14
\inst5|Ram0~28\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst5|Ram0~28_combout\ = (\inst5|q\(2) & (!\inst5|q\(3) & ((\inst5|q\(0)) # (\inst5|q\(1))))) # (!\inst5|q\(2) & (\inst5|q\(3) & ((!\inst5|q\(1)) # (!\inst5|q\(0)))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0010011001100100",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst5|q\(2),
	datab => \inst5|q\(3),
	datac => \inst5|q\(0),
	datad => \inst5|q\(1),
	combout => \inst5|Ram0~28_combout\);

-- Location: LCCOMB_X18_Y22_N20
\inst5|Ram0~29\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst5|Ram0~29_combout\ = (\inst5|Ram0~28_combout\ & (((!\inst5|q\(3) & !\inst5|q\(1))) # (!\inst5|q\(4)))) # (!\inst5|Ram0~28_combout\ & (((\inst5|q\(4)))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0001111111110000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst5|q\(3),
	datab => \inst5|q\(1),
	datac => \inst5|Ram0~28_combout\,
	datad => \inst5|q\(4),
	combout => \inst5|Ram0~29_combout\);

-- Location: LCCOMB_X19_Y22_N30
\inst5|Ram0~20\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst5|Ram0~20_combout\ = (\inst5|q\(0) & ((\inst5|q\(1) & (!\inst5|q\(2))) # (!\inst5|q\(1) & (\inst5|q\(2) & \inst5|q\(4))))) # (!\inst5|q\(0) & (!\inst5|q\(1) & ((\inst5|q\(2)) # (\inst5|q\(4)))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0011100100011000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst5|q\(0),
	datab => \inst5|q\(1),
	datac => \inst5|q\(2),
	datad => \inst5|q\(4),
	combout => \inst5|Ram0~20_combout\);

-- Location: LCCOMB_X18_Y22_N28
\inst5|Ram0~19\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst5|Ram0~19_combout\ = (\inst5|q\(2) & (!\inst5|q\(1) & ((\inst5|q\(4)) # (!\inst5|q\(0))))) # (!\inst5|q\(2) & (\inst5|q\(1) & ((\inst5|q\(0)) # (!\inst5|q\(4)))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0110001001000110",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst5|q\(2),
	datab => \inst5|q\(1),
	datac => \inst5|q\(0),
	datad => \inst5|q\(4),
	combout => \inst5|Ram0~19_combout\);

-- Location: LCCOMB_X18_Y22_N10
\inst5|Ram0~21\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst5|Ram0~21_combout\ = (\inst5|q\(3) & (!\inst5|Ram0~20_combout\)) # (!\inst5|q\(3) & ((\inst5|Ram0~19_combout\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0011111100001100",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	datab => \inst5|q\(3),
	datac => \inst5|Ram0~20_combout\,
	datad => \inst5|Ram0~19_combout\,
	combout => \inst5|Ram0~21_combout\);

-- Location: LCCOMB_X19_Y22_N0
\inst5|Ram0~22\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst5|Ram0~22_combout\ = (\inst5|q\(4) & (!\inst5|q\(0) & ((\inst5|q\(1)) # (\inst5|q\(3))))) # (!\inst5|q\(4) & ((\inst5|q\(1) & (\inst5|q\(3) & !\inst5|q\(0))) # (!\inst5|q\(1) & ((\inst5|q\(0))))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0001000111101000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst5|q\(4),
	datab => \inst5|q\(1),
	datac => \inst5|q\(3),
	datad => \inst5|q\(0),
	combout => \inst5|Ram0~22_combout\);

-- Location: LCCOMB_X19_Y22_N26
\inst5|Ram0~23\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst5|Ram0~23_combout\ = \inst5|q\(2) $ (\inst5|Ram0~22_combout\)

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0011001111001100",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	datab => \inst5|q\(2),
	datad => \inst5|Ram0~22_combout\,
	combout => \inst5|Ram0~23_combout\);

-- Location: LCCOMB_X19_Y22_N28
\inst5|Ram0~24\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst5|Ram0~24_combout\ = \inst5|q\(4) $ (\inst5|q\(1) $ (((\inst5|q\(3)) # (\inst5|q\(0)))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1100100100110110",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst5|q\(3),
	datab => \inst5|q\(4),
	datac => \inst5|q\(0),
	datad => \inst5|q\(1),
	combout => \inst5|Ram0~24_combout\);

-- Location: LCCOMB_X18_Y22_N12
\inst5|Ram0~25\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst5|Ram0~25_combout\ = \inst5|q\(0) $ (\inst5|q\(3))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0000111111110000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	datac => \inst5|q\(0),
	datad => \inst5|q\(3),
	combout => \inst5|Ram0~25_combout\);

-- Location: LCCOMB_X19_Y22_N2
\inst5|Ram0~26\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst5|Ram0~26_combout\ = (!\inst5|q\(0) & (\inst5|q\(2) & (!\inst5|q\(3) & !\inst5|q\(4))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0000000000000100",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst5|q\(0),
	datab => \inst5|q\(2),
	datac => \inst5|q\(3),
	datad => \inst5|q\(4),
	combout => \inst5|Ram0~26_combout\);

-- Location: LCCOMB_X19_Y22_N14
\inst5|Ram0~27\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst5|Ram0~27_combout\ = (\inst5|q\(1)) # (\inst5|Ram0~26_combout\)

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1111111111110000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	datac => \inst5|q\(1),
	datad => \inst5|Ram0~26_combout\,
	combout => \inst5|Ram0~27_combout\);

-- Location: LCCOMB_X17_Y22_N0
\inst3|acc[0]~32\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst3|acc[0]~32_combout\ = (\inst5|q\(0) & (\inst3|acc\(0) & VCC)) # (!\inst5|q\(0) & (\inst3|acc\(0) $ (VCC)))
-- \inst3|acc[0]~33\ = CARRY((!\inst5|q\(0) & \inst3|acc\(0)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1001100101000100",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst5|q\(0),
	datab => \inst3|acc\(0),
	datad => VCC,
	combout => \inst3|acc[0]~32_combout\,
	cout => \inst3|acc[0]~33\);

-- Location: FF_X17_Y22_N1
\inst3|acc[0]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \CLK~inputclkctrl_outclk\,
	d => \inst3|acc[0]~32_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst3|acc\(0));

-- Location: LCCOMB_X17_Y22_N2
\inst3|acc[1]~34\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst3|acc[1]~34_combout\ = (\inst5|Ram0~27_combout\ & ((\inst3|acc\(1) & (!\inst3|acc[0]~33\)) # (!\inst3|acc\(1) & ((\inst3|acc[0]~33\) # (GND))))) # (!\inst5|Ram0~27_combout\ & ((\inst3|acc\(1) & (\inst3|acc[0]~33\ & VCC)) # (!\inst3|acc\(1) & 
-- (!\inst3|acc[0]~33\))))
-- \inst3|acc[1]~35\ = CARRY((\inst5|Ram0~27_combout\ & ((!\inst3|acc[0]~33\) # (!\inst3|acc\(1)))) # (!\inst5|Ram0~27_combout\ & (!\inst3|acc\(1) & !\inst3|acc[0]~33\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0110100100101011",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst5|Ram0~27_combout\,
	datab => \inst3|acc\(1),
	datad => VCC,
	cin => \inst3|acc[0]~33\,
	combout => \inst3|acc[1]~34_combout\,
	cout => \inst3|acc[1]~35\);

-- Location: FF_X17_Y22_N3
\inst3|acc[1]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \CLK~inputclkctrl_outclk\,
	d => \inst3|acc[1]~34_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst3|acc\(1));

-- Location: LCCOMB_X17_Y22_N4
\inst3|acc[2]~36\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst3|acc[2]~36_combout\ = ((\inst5|q\(2) $ (\inst3|acc\(2) $ (\inst3|acc[1]~35\)))) # (GND)
-- \inst3|acc[2]~37\ = CARRY((\inst5|q\(2) & (\inst3|acc\(2) & !\inst3|acc[1]~35\)) # (!\inst5|q\(2) & ((\inst3|acc\(2)) # (!\inst3|acc[1]~35\))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1001011001001101",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst5|q\(2),
	datab => \inst3|acc\(2),
	datad => VCC,
	cin => \inst3|acc[1]~35\,
	combout => \inst3|acc[2]~36_combout\,
	cout => \inst3|acc[2]~37\);

-- Location: FF_X17_Y22_N5
\inst3|acc[2]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \CLK~inputclkctrl_outclk\,
	d => \inst3|acc[2]~36_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst3|acc\(2));

-- Location: LCCOMB_X17_Y22_N6
\inst3|acc[3]~38\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst3|acc[3]~38_combout\ = (\inst3|acc\(3) & ((\inst5|Ram0~25_combout\ & (\inst3|acc[2]~37\ & VCC)) # (!\inst5|Ram0~25_combout\ & (!\inst3|acc[2]~37\)))) # (!\inst3|acc\(3) & ((\inst5|Ram0~25_combout\ & (!\inst3|acc[2]~37\)) # (!\inst5|Ram0~25_combout\ & 
-- ((\inst3|acc[2]~37\) # (GND)))))
-- \inst3|acc[3]~39\ = CARRY((\inst3|acc\(3) & (!\inst5|Ram0~25_combout\ & !\inst3|acc[2]~37\)) # (!\inst3|acc\(3) & ((!\inst3|acc[2]~37\) # (!\inst5|Ram0~25_combout\))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1001011000010111",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst3|acc\(3),
	datab => \inst5|Ram0~25_combout\,
	datad => VCC,
	cin => \inst3|acc[2]~37\,
	combout => \inst3|acc[3]~38_combout\,
	cout => \inst3|acc[3]~39\);

-- Location: FF_X17_Y22_N7
\inst3|acc[3]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \CLK~inputclkctrl_outclk\,
	d => \inst3|acc[3]~38_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst3|acc\(3));

-- Location: LCCOMB_X17_Y22_N8
\inst3|acc[4]~40\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst3|acc[4]~40_combout\ = ((\inst5|Ram0~24_combout\ $ (\inst3|acc\(4) $ (\inst3|acc[3]~39\)))) # (GND)
-- \inst3|acc[4]~41\ = CARRY((\inst5|Ram0~24_combout\ & (\inst3|acc\(4) & !\inst3|acc[3]~39\)) # (!\inst5|Ram0~24_combout\ & ((\inst3|acc\(4)) # (!\inst3|acc[3]~39\))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1001011001001101",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst5|Ram0~24_combout\,
	datab => \inst3|acc\(4),
	datad => VCC,
	cin => \inst3|acc[3]~39\,
	combout => \inst3|acc[4]~40_combout\,
	cout => \inst3|acc[4]~41\);

-- Location: FF_X17_Y22_N9
\inst3|acc[4]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \CLK~inputclkctrl_outclk\,
	d => \inst3|acc[4]~40_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst3|acc\(4));

-- Location: LCCOMB_X17_Y22_N10
\inst3|acc[5]~42\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst3|acc[5]~42_combout\ = (\inst3|acc\(5) & ((\inst5|Ram0~23_combout\ & (\inst3|acc[4]~41\ & VCC)) # (!\inst5|Ram0~23_combout\ & (!\inst3|acc[4]~41\)))) # (!\inst3|acc\(5) & ((\inst5|Ram0~23_combout\ & (!\inst3|acc[4]~41\)) # (!\inst5|Ram0~23_combout\ & 
-- ((\inst3|acc[4]~41\) # (GND)))))
-- \inst3|acc[5]~43\ = CARRY((\inst3|acc\(5) & (!\inst5|Ram0~23_combout\ & !\inst3|acc[4]~41\)) # (!\inst3|acc\(5) & ((!\inst3|acc[4]~41\) # (!\inst5|Ram0~23_combout\))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1001011000010111",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst3|acc\(5),
	datab => \inst5|Ram0~23_combout\,
	datad => VCC,
	cin => \inst3|acc[4]~41\,
	combout => \inst3|acc[5]~42_combout\,
	cout => \inst3|acc[5]~43\);

-- Location: FF_X17_Y22_N11
\inst3|acc[5]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \CLK~inputclkctrl_outclk\,
	d => \inst3|acc[5]~42_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst3|acc\(5));

-- Location: LCCOMB_X17_Y22_N12
\inst3|acc[6]~44\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst3|acc[6]~44_combout\ = ((\inst3|acc\(6) $ (\inst5|Ram0~21_combout\ $ (!\inst3|acc[5]~43\)))) # (GND)
-- \inst3|acc[6]~45\ = CARRY((\inst3|acc\(6) & ((\inst5|Ram0~21_combout\) # (!\inst3|acc[5]~43\))) # (!\inst3|acc\(6) & (\inst5|Ram0~21_combout\ & !\inst3|acc[5]~43\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0110100110001110",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst3|acc\(6),
	datab => \inst5|Ram0~21_combout\,
	datad => VCC,
	cin => \inst3|acc[5]~43\,
	combout => \inst3|acc[6]~44_combout\,
	cout => \inst3|acc[6]~45\);

-- Location: FF_X17_Y22_N13
\inst3|acc[6]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \CLK~inputclkctrl_outclk\,
	d => \inst3|acc[6]~44_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst3|acc\(6));

-- Location: LCCOMB_X17_Y22_N14
\inst3|acc[7]~46\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst3|acc[7]~46_combout\ = (\inst5|Ram0~29_combout\ & ((\inst3|acc\(7) & (\inst3|acc[6]~45\ & VCC)) # (!\inst3|acc\(7) & (!\inst3|acc[6]~45\)))) # (!\inst5|Ram0~29_combout\ & ((\inst3|acc\(7) & (!\inst3|acc[6]~45\)) # (!\inst3|acc\(7) & 
-- ((\inst3|acc[6]~45\) # (GND)))))
-- \inst3|acc[7]~47\ = CARRY((\inst5|Ram0~29_combout\ & (!\inst3|acc\(7) & !\inst3|acc[6]~45\)) # (!\inst5|Ram0~29_combout\ & ((!\inst3|acc[6]~45\) # (!\inst3|acc\(7)))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1001011000010111",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst5|Ram0~29_combout\,
	datab => \inst3|acc\(7),
	datad => VCC,
	cin => \inst3|acc[6]~45\,
	combout => \inst3|acc[7]~46_combout\,
	cout => \inst3|acc[7]~47\);

-- Location: FF_X17_Y22_N15
\inst3|acc[7]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \CLK~inputclkctrl_outclk\,
	d => \inst3|acc[7]~46_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst3|acc\(7));

-- Location: LCCOMB_X17_Y22_N16
\inst3|acc[8]~48\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst3|acc[8]~48_combout\ = ((\inst5|Ram0~31_combout\ $ (\inst3|acc\(8) $ (!\inst3|acc[7]~47\)))) # (GND)
-- \inst3|acc[8]~49\ = CARRY((\inst5|Ram0~31_combout\ & ((\inst3|acc\(8)) # (!\inst3|acc[7]~47\))) # (!\inst5|Ram0~31_combout\ & (\inst3|acc\(8) & !\inst3|acc[7]~47\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0110100110001110",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst5|Ram0~31_combout\,
	datab => \inst3|acc\(8),
	datad => VCC,
	cin => \inst3|acc[7]~47\,
	combout => \inst3|acc[8]~48_combout\,
	cout => \inst3|acc[8]~49\);

-- Location: FF_X17_Y22_N17
\inst3|acc[8]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \CLK~inputclkctrl_outclk\,
	d => \inst3|acc[8]~48_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst3|acc\(8));

-- Location: LCCOMB_X17_Y22_N18
\inst3|acc[9]~50\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst3|acc[9]~50_combout\ = (\inst5|Ram0~18_combout\ & ((\inst3|acc\(9) & (\inst3|acc[8]~49\ & VCC)) # (!\inst3|acc\(9) & (!\inst3|acc[8]~49\)))) # (!\inst5|Ram0~18_combout\ & ((\inst3|acc\(9) & (!\inst3|acc[8]~49\)) # (!\inst3|acc\(9) & 
-- ((\inst3|acc[8]~49\) # (GND)))))
-- \inst3|acc[9]~51\ = CARRY((\inst5|Ram0~18_combout\ & (!\inst3|acc\(9) & !\inst3|acc[8]~49\)) # (!\inst5|Ram0~18_combout\ & ((!\inst3|acc[8]~49\) # (!\inst3|acc\(9)))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1001011000010111",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst5|Ram0~18_combout\,
	datab => \inst3|acc\(9),
	datad => VCC,
	cin => \inst3|acc[8]~49\,
	combout => \inst3|acc[9]~50_combout\,
	cout => \inst3|acc[9]~51\);

-- Location: FF_X17_Y22_N19
\inst3|acc[9]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \CLK~inputclkctrl_outclk\,
	d => \inst3|acc[9]~50_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst3|acc\(9));

-- Location: LCCOMB_X17_Y22_N20
\inst3|acc[10]~52\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst3|acc[10]~52_combout\ = ((\inst5|Ram0~16_combout\ $ (\inst3|acc\(10) $ (!\inst3|acc[9]~51\)))) # (GND)
-- \inst3|acc[10]~53\ = CARRY((\inst5|Ram0~16_combout\ & ((\inst3|acc\(10)) # (!\inst3|acc[9]~51\))) # (!\inst5|Ram0~16_combout\ & (\inst3|acc\(10) & !\inst3|acc[9]~51\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0110100110001110",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst5|Ram0~16_combout\,
	datab => \inst3|acc\(10),
	datad => VCC,
	cin => \inst3|acc[9]~51\,
	combout => \inst3|acc[10]~52_combout\,
	cout => \inst3|acc[10]~53\);

-- Location: FF_X17_Y22_N21
\inst3|acc[10]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \CLK~inputclkctrl_outclk\,
	d => \inst3|acc[10]~52_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst3|acc\(10));

-- Location: LCCOMB_X17_Y22_N22
\inst3|acc[11]~54\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst3|acc[11]~54_combout\ = (\inst3|acc\(11) & ((\inst5|Ram0~14_combout\ & (\inst3|acc[10]~53\ & VCC)) # (!\inst5|Ram0~14_combout\ & (!\inst3|acc[10]~53\)))) # (!\inst3|acc\(11) & ((\inst5|Ram0~14_combout\ & (!\inst3|acc[10]~53\)) # 
-- (!\inst5|Ram0~14_combout\ & ((\inst3|acc[10]~53\) # (GND)))))
-- \inst3|acc[11]~55\ = CARRY((\inst3|acc\(11) & (!\inst5|Ram0~14_combout\ & !\inst3|acc[10]~53\)) # (!\inst3|acc\(11) & ((!\inst3|acc[10]~53\) # (!\inst5|Ram0~14_combout\))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1001011000010111",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst3|acc\(11),
	datab => \inst5|Ram0~14_combout\,
	datad => VCC,
	cin => \inst3|acc[10]~53\,
	combout => \inst3|acc[11]~54_combout\,
	cout => \inst3|acc[11]~55\);

-- Location: FF_X17_Y22_N23
\inst3|acc[11]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \CLK~inputclkctrl_outclk\,
	d => \inst3|acc[11]~54_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst3|acc\(11));

-- Location: LCCOMB_X17_Y22_N24
\inst3|acc[12]~56\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst3|acc[12]~56_combout\ = ((\inst5|Ram0~13_combout\ $ (\inst3|acc\(12) $ (!\inst3|acc[11]~55\)))) # (GND)
-- \inst3|acc[12]~57\ = CARRY((\inst5|Ram0~13_combout\ & ((\inst3|acc\(12)) # (!\inst3|acc[11]~55\))) # (!\inst5|Ram0~13_combout\ & (\inst3|acc\(12) & !\inst3|acc[11]~55\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0110100110001110",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst5|Ram0~13_combout\,
	datab => \inst3|acc\(12),
	datad => VCC,
	cin => \inst3|acc[11]~55\,
	combout => \inst3|acc[12]~56_combout\,
	cout => \inst3|acc[12]~57\);

-- Location: FF_X17_Y22_N25
\inst3|acc[12]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \CLK~inputclkctrl_outclk\,
	d => \inst3|acc[12]~56_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst3|acc\(12));

-- Location: LCCOMB_X17_Y22_N26
\inst3|acc[13]~58\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst3|acc[13]~58_combout\ = (\inst3|acc\(13) & ((\inst5|q\(0) & (\inst3|acc[12]~57\ & VCC)) # (!\inst5|q\(0) & (!\inst3|acc[12]~57\)))) # (!\inst3|acc\(13) & ((\inst5|q\(0) & (!\inst3|acc[12]~57\)) # (!\inst5|q\(0) & ((\inst3|acc[12]~57\) # (GND)))))
-- \inst3|acc[13]~59\ = CARRY((\inst3|acc\(13) & (!\inst5|q\(0) & !\inst3|acc[12]~57\)) # (!\inst3|acc\(13) & ((!\inst3|acc[12]~57\) # (!\inst5|q\(0)))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1001011000010111",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst3|acc\(13),
	datab => \inst5|q\(0),
	datad => VCC,
	cin => \inst3|acc[12]~57\,
	combout => \inst3|acc[13]~58_combout\,
	cout => \inst3|acc[13]~59\);

-- Location: FF_X17_Y22_N27
\inst3|acc[13]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \CLK~inputclkctrl_outclk\,
	d => \inst3|acc[13]~58_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst3|acc\(13));

-- Location: LCCOMB_X17_Y22_N28
\inst3|acc[14]~60\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst3|acc[14]~60_combout\ = ((\inst5|q\(1) $ (\inst3|acc\(14) $ (!\inst3|acc[13]~59\)))) # (GND)
-- \inst3|acc[14]~61\ = CARRY((\inst5|q\(1) & ((\inst3|acc\(14)) # (!\inst3|acc[13]~59\))) # (!\inst5|q\(1) & (\inst3|acc\(14) & !\inst3|acc[13]~59\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0110100110001110",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst5|q\(1),
	datab => \inst3|acc\(14),
	datad => VCC,
	cin => \inst3|acc[13]~59\,
	combout => \inst3|acc[14]~60_combout\,
	cout => \inst3|acc[14]~61\);

-- Location: FF_X17_Y22_N29
\inst3|acc[14]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \CLK~inputclkctrl_outclk\,
	d => \inst3|acc[14]~60_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst3|acc\(14));

-- Location: LCCOMB_X17_Y22_N30
\inst3|acc[15]~62\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst3|acc[15]~62_combout\ = (\inst3|acc\(15) & ((\inst5|Ram0~11_combout\ & (!\inst3|acc[14]~61\)) # (!\inst5|Ram0~11_combout\ & (\inst3|acc[14]~61\ & VCC)))) # (!\inst3|acc\(15) & ((\inst5|Ram0~11_combout\ & ((\inst3|acc[14]~61\) # (GND))) # 
-- (!\inst5|Ram0~11_combout\ & (!\inst3|acc[14]~61\))))
-- \inst3|acc[15]~63\ = CARRY((\inst3|acc\(15) & (\inst5|Ram0~11_combout\ & !\inst3|acc[14]~61\)) # (!\inst3|acc\(15) & ((\inst5|Ram0~11_combout\) # (!\inst3|acc[14]~61\))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0110100101001101",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst3|acc\(15),
	datab => \inst5|Ram0~11_combout\,
	datad => VCC,
	cin => \inst3|acc[14]~61\,
	combout => \inst3|acc[15]~62_combout\,
	cout => \inst3|acc[15]~63\);

-- Location: FF_X17_Y22_N31
\inst3|acc[15]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \CLK~inputclkctrl_outclk\,
	d => \inst3|acc[15]~62_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst3|acc\(15));

-- Location: LCCOMB_X17_Y21_N0
\inst3|acc[16]~64\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst3|acc[16]~64_combout\ = ((\inst3|acc\(16) $ (\inst5|Ram0~10_combout\ $ (!\inst3|acc[15]~63\)))) # (GND)
-- \inst3|acc[16]~65\ = CARRY((\inst3|acc\(16) & ((\inst5|Ram0~10_combout\) # (!\inst3|acc[15]~63\))) # (!\inst3|acc\(16) & (\inst5|Ram0~10_combout\ & !\inst3|acc[15]~63\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0110100110001110",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst3|acc\(16),
	datab => \inst5|Ram0~10_combout\,
	datad => VCC,
	cin => \inst3|acc[15]~63\,
	combout => \inst3|acc[16]~64_combout\,
	cout => \inst3|acc[16]~65\);

-- Location: FF_X17_Y21_N1
\inst3|acc[16]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \CLK~inputclkctrl_outclk\,
	d => \inst3|acc[16]~64_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst3|acc\(16));

-- Location: LCCOMB_X17_Y21_N2
\inst3|acc[17]~66\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst3|acc[17]~66_combout\ = (\inst5|Ram0~9_combout\ & ((\inst3|acc\(17) & (!\inst3|acc[16]~65\)) # (!\inst3|acc\(17) & ((\inst3|acc[16]~65\) # (GND))))) # (!\inst5|Ram0~9_combout\ & ((\inst3|acc\(17) & (\inst3|acc[16]~65\ & VCC)) # (!\inst3|acc\(17) & 
-- (!\inst3|acc[16]~65\))))
-- \inst3|acc[17]~67\ = CARRY((\inst5|Ram0~9_combout\ & ((!\inst3|acc[16]~65\) # (!\inst3|acc\(17)))) # (!\inst5|Ram0~9_combout\ & (!\inst3|acc\(17) & !\inst3|acc[16]~65\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0110100100101011",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst5|Ram0~9_combout\,
	datab => \inst3|acc\(17),
	datad => VCC,
	cin => \inst3|acc[16]~65\,
	combout => \inst3|acc[17]~66_combout\,
	cout => \inst3|acc[17]~67\);

-- Location: FF_X17_Y21_N3
\inst3|acc[17]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \CLK~inputclkctrl_outclk\,
	d => \inst3|acc[17]~66_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst3|acc\(17));

-- Location: LCCOMB_X17_Y21_N4
\inst3|acc[18]~68\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst3|acc[18]~68_combout\ = ((\inst5|Ram0~7_combout\ $ (\inst3|acc\(18) $ (!\inst3|acc[17]~67\)))) # (GND)
-- \inst3|acc[18]~69\ = CARRY((\inst5|Ram0~7_combout\ & ((\inst3|acc\(18)) # (!\inst3|acc[17]~67\))) # (!\inst5|Ram0~7_combout\ & (\inst3|acc\(18) & !\inst3|acc[17]~67\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0110100110001110",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst5|Ram0~7_combout\,
	datab => \inst3|acc\(18),
	datad => VCC,
	cin => \inst3|acc[17]~67\,
	combout => \inst3|acc[18]~68_combout\,
	cout => \inst3|acc[18]~69\);

-- Location: FF_X17_Y21_N5
\inst3|acc[18]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \CLK~inputclkctrl_outclk\,
	d => \inst3|acc[18]~68_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst3|acc\(18));

-- Location: LCCOMB_X17_Y21_N6
\inst3|acc[19]~70\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst3|acc[19]~70_combout\ = (\inst3|acc\(19) & ((\inst5|Ram0~4_combout\ & (\inst3|acc[18]~69\ & VCC)) # (!\inst5|Ram0~4_combout\ & (!\inst3|acc[18]~69\)))) # (!\inst3|acc\(19) & ((\inst5|Ram0~4_combout\ & (!\inst3|acc[18]~69\)) # (!\inst5|Ram0~4_combout\ 
-- & ((\inst3|acc[18]~69\) # (GND)))))
-- \inst3|acc[19]~71\ = CARRY((\inst3|acc\(19) & (!\inst5|Ram0~4_combout\ & !\inst3|acc[18]~69\)) # (!\inst3|acc\(19) & ((!\inst3|acc[18]~69\) # (!\inst5|Ram0~4_combout\))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1001011000010111",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst3|acc\(19),
	datab => \inst5|Ram0~4_combout\,
	datad => VCC,
	cin => \inst3|acc[18]~69\,
	combout => \inst3|acc[19]~70_combout\,
	cout => \inst3|acc[19]~71\);

-- Location: FF_X17_Y21_N7
\inst3|acc[19]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \CLK~inputclkctrl_outclk\,
	d => \inst3|acc[19]~70_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst3|acc\(19));

-- Location: LCCOMB_X17_Y21_N8
\inst3|acc[20]~72\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst3|acc[20]~72_combout\ = ((\inst5|Ram0~1_combout\ $ (\inst3|acc\(20) $ (!\inst3|acc[19]~71\)))) # (GND)
-- \inst3|acc[20]~73\ = CARRY((\inst5|Ram0~1_combout\ & ((\inst3|acc\(20)) # (!\inst3|acc[19]~71\))) # (!\inst5|Ram0~1_combout\ & (\inst3|acc\(20) & !\inst3|acc[19]~71\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0110100110001110",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst5|Ram0~1_combout\,
	datab => \inst3|acc\(20),
	datad => VCC,
	cin => \inst3|acc[19]~71\,
	combout => \inst3|acc[20]~72_combout\,
	cout => \inst3|acc[20]~73\);

-- Location: FF_X17_Y21_N9
\inst3|acc[20]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \CLK~inputclkctrl_outclk\,
	d => \inst3|acc[20]~72_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst3|acc\(20));

-- Location: LCCOMB_X17_Y21_N10
\inst3|acc[21]~74\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst3|acc[21]~74_combout\ = (\inst3|acc\(21) & ((\inst5|Ram0~0_combout\ & (\inst3|acc[20]~73\ & VCC)) # (!\inst5|Ram0~0_combout\ & (!\inst3|acc[20]~73\)))) # (!\inst3|acc\(21) & ((\inst5|Ram0~0_combout\ & (!\inst3|acc[20]~73\)) # (!\inst5|Ram0~0_combout\ 
-- & ((\inst3|acc[20]~73\) # (GND)))))
-- \inst3|acc[21]~75\ = CARRY((\inst3|acc\(21) & (!\inst5|Ram0~0_combout\ & !\inst3|acc[20]~73\)) # (!\inst3|acc\(21) & ((!\inst3|acc[20]~73\) # (!\inst5|Ram0~0_combout\))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1001011000010111",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst3|acc\(21),
	datab => \inst5|Ram0~0_combout\,
	datad => VCC,
	cin => \inst3|acc[20]~73\,
	combout => \inst3|acc[21]~74_combout\,
	cout => \inst3|acc[21]~75\);

-- Location: FF_X17_Y21_N11
\inst3|acc[21]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \CLK~inputclkctrl_outclk\,
	d => \inst3|acc[21]~74_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst3|acc\(21));

-- Location: LCCOMB_X17_Y21_N12
\inst3|acc[22]~76\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst3|acc[22]~76_combout\ = ((\inst3|acc\(22) $ (\inst5|Equal0~1_combout\ $ (!\inst3|acc[21]~75\)))) # (GND)
-- \inst3|acc[22]~77\ = CARRY((\inst3|acc\(22) & ((\inst5|Equal0~1_combout\) # (!\inst3|acc[21]~75\))) # (!\inst3|acc\(22) & (\inst5|Equal0~1_combout\ & !\inst3|acc[21]~75\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0110100110001110",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst3|acc\(22),
	datab => \inst5|Equal0~1_combout\,
	datad => VCC,
	cin => \inst3|acc[21]~75\,
	combout => \inst3|acc[22]~76_combout\,
	cout => \inst3|acc[22]~77\);

-- Location: FF_X17_Y21_N13
\inst3|acc[22]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \CLK~inputclkctrl_outclk\,
	d => \inst3|acc[22]~76_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst3|acc\(22));

-- Location: LCCOMB_X17_Y21_N14
\inst3|acc[23]~78\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst3|acc[23]~78_combout\ = (\inst3|acc\(23) & (!\inst3|acc[22]~77\)) # (!\inst3|acc\(23) & ((\inst3|acc[22]~77\) # (GND)))
-- \inst3|acc[23]~79\ = CARRY((!\inst3|acc[22]~77\) # (!\inst3|acc\(23)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0011110000111111",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	datab => \inst3|acc\(23),
	datad => VCC,
	cin => \inst3|acc[22]~77\,
	combout => \inst3|acc[23]~78_combout\,
	cout => \inst3|acc[23]~79\);

-- Location: FF_X17_Y21_N15
\inst3|acc[23]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \CLK~inputclkctrl_outclk\,
	d => \inst3|acc[23]~78_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst3|acc\(23));

-- Location: LCCOMB_X17_Y21_N16
\inst3|acc[24]~80\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst3|acc[24]~80_combout\ = (\inst3|acc\(24) & (\inst3|acc[23]~79\ $ (GND))) # (!\inst3|acc\(24) & (!\inst3|acc[23]~79\ & VCC))
-- \inst3|acc[24]~81\ = CARRY((\inst3|acc\(24) & !\inst3|acc[23]~79\))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1100001100001100",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	datab => \inst3|acc\(24),
	datad => VCC,
	cin => \inst3|acc[23]~79\,
	combout => \inst3|acc[24]~80_combout\,
	cout => \inst3|acc[24]~81\);

-- Location: FF_X17_Y21_N17
\inst3|acc[24]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \CLK~inputclkctrl_outclk\,
	d => \inst3|acc[24]~80_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst3|acc\(24));

-- Location: LCCOMB_X17_Y21_N18
\inst3|acc[25]~82\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst3|acc[25]~82_combout\ = (\inst3|acc\(25) & (!\inst3|acc[24]~81\)) # (!\inst3|acc\(25) & ((\inst3|acc[24]~81\) # (GND)))
-- \inst3|acc[25]~83\ = CARRY((!\inst3|acc[24]~81\) # (!\inst3|acc\(25)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0011110000111111",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	datab => \inst3|acc\(25),
	datad => VCC,
	cin => \inst3|acc[24]~81\,
	combout => \inst3|acc[25]~82_combout\,
	cout => \inst3|acc[25]~83\);

-- Location: FF_X17_Y21_N19
\inst3|acc[25]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \CLK~inputclkctrl_outclk\,
	d => \inst3|acc[25]~82_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst3|acc\(25));

-- Location: LCCOMB_X17_Y21_N20
\inst3|acc[26]~84\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst3|acc[26]~84_combout\ = (\inst3|acc\(26) & (\inst3|acc[25]~83\ $ (GND))) # (!\inst3|acc\(26) & (!\inst3|acc[25]~83\ & VCC))
-- \inst3|acc[26]~85\ = CARRY((\inst3|acc\(26) & !\inst3|acc[25]~83\))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1100001100001100",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	datab => \inst3|acc\(26),
	datad => VCC,
	cin => \inst3|acc[25]~83\,
	combout => \inst3|acc[26]~84_combout\,
	cout => \inst3|acc[26]~85\);

-- Location: FF_X17_Y21_N21
\inst3|acc[26]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \CLK~inputclkctrl_outclk\,
	d => \inst3|acc[26]~84_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst3|acc\(26));

-- Location: LCCOMB_X17_Y21_N22
\inst3|acc[27]~86\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst3|acc[27]~86_combout\ = (\inst3|acc\(27) & (!\inst3|acc[26]~85\)) # (!\inst3|acc\(27) & ((\inst3|acc[26]~85\) # (GND)))
-- \inst3|acc[27]~87\ = CARRY((!\inst3|acc[26]~85\) # (!\inst3|acc\(27)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0101101001011111",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst3|acc\(27),
	datad => VCC,
	cin => \inst3|acc[26]~85\,
	combout => \inst3|acc[27]~86_combout\,
	cout => \inst3|acc[27]~87\);

-- Location: FF_X17_Y21_N23
\inst3|acc[27]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \CLK~inputclkctrl_outclk\,
	d => \inst3|acc[27]~86_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst3|acc\(27));

-- Location: LCCOMB_X17_Y21_N24
\inst3|acc[28]~88\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst3|acc[28]~88_combout\ = (\inst3|acc\(28) & (\inst3|acc[27]~87\ $ (GND))) # (!\inst3|acc\(28) & (!\inst3|acc[27]~87\ & VCC))
-- \inst3|acc[28]~89\ = CARRY((\inst3|acc\(28) & !\inst3|acc[27]~87\))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1100001100001100",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	datab => \inst3|acc\(28),
	datad => VCC,
	cin => \inst3|acc[27]~87\,
	combout => \inst3|acc[28]~88_combout\,
	cout => \inst3|acc[28]~89\);

-- Location: FF_X17_Y21_N25
\inst3|acc[28]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \CLK~inputclkctrl_outclk\,
	d => \inst3|acc[28]~88_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst3|acc\(28));

-- Location: LCCOMB_X17_Y21_N26
\inst3|acc[29]~90\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst3|acc[29]~90_combout\ = (\inst3|acc\(29) & (!\inst3|acc[28]~89\)) # (!\inst3|acc\(29) & ((\inst3|acc[28]~89\) # (GND)))
-- \inst3|acc[29]~91\ = CARRY((!\inst3|acc[28]~89\) # (!\inst3|acc\(29)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0101101001011111",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst3|acc\(29),
	datad => VCC,
	cin => \inst3|acc[28]~89\,
	combout => \inst3|acc[29]~90_combout\,
	cout => \inst3|acc[29]~91\);

-- Location: FF_X17_Y21_N27
\inst3|acc[29]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \CLK~inputclkctrl_outclk\,
	d => \inst3|acc[29]~90_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst3|acc\(29));

-- Location: LCCOMB_X17_Y21_N28
\inst3|acc[30]~92\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst3|acc[30]~92_combout\ = (\inst3|acc\(30) & (\inst3|acc[29]~91\ $ (GND))) # (!\inst3|acc\(30) & (!\inst3|acc[29]~91\ & VCC))
-- \inst3|acc[30]~93\ = CARRY((\inst3|acc\(30) & !\inst3|acc[29]~91\))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1100001100001100",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	datab => \inst3|acc\(30),
	datad => VCC,
	cin => \inst3|acc[29]~91\,
	combout => \inst3|acc[30]~92_combout\,
	cout => \inst3|acc[30]~93\);

-- Location: FF_X17_Y21_N29
\inst3|acc[30]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \CLK~inputclkctrl_outclk\,
	d => \inst3|acc[30]~92_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst3|acc\(30));

-- Location: LCCOMB_X17_Y21_N30
\inst3|acc[31]~94\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst3|acc[31]~94_combout\ = \inst3|acc\(31) $ (\inst3|acc[30]~93\)

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0101101001011010",
	sum_lutc_input => "cin")
-- pragma translate_on
PORT MAP (
	dataa => \inst3|acc\(31),
	cin => \inst3|acc[30]~93\,
	combout => \inst3|acc[31]~94_combout\);

-- Location: FF_X17_Y21_N31
\inst3|acc[31]\ : dffeas
-- pragma translate_off
GENERIC MAP (
	is_wysiwyg => "true",
	power_up => "low")
-- pragma translate_on
PORT MAP (
	clk => \CLK~inputclkctrl_outclk\,
	d => \inst3|acc[31]~94_combout\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	q => \inst3|acc\(31));

-- Location: M9K_X27_Y16_N0
\inst4|altsyncram_component|auto_generated|ram_block1a0\ : cycloneive_ram_block
-- pragma translate_off
GENERIC MAP (
	mem_init4 => X"0007E7C7C0007C79790007A7676000787373000766F70000746C6D00072696A0007066670006E63630006C60600006A5D5D000685A5A000665757000645454000625151000604E4F0005E4B4C0005C48490005A4646000584343000564040000543D3E000523B3B0005038380004E36360004C33330004A3131000482E2E0004",
	mem_init3 => X"62C2C00044292A0004227270004025250003E23230003C21210003A1E1F000381C1D000361A1B0003419190003217170003015150002E13130002C12120002A1010000280F0F000260D0D000240C0C000220A0A0002009090001E08080001C07070001A06060001805050001604040001403030001203030001002020000E01010000C01010000A00000000800000000600000000400000000200000000000000000100000000300000000500000000700000000900000000B01010000D01010000F02020001103030001303030001504040001705050001906060001B07070001D08080001F0909000210A0A000230C0C000250D0D000270F0F000291010000",
	mem_init2 => X"2B12120002D13130002F1515000311717000331919000351A1B000371C1D000391E1F0003B21210003D23230003F252500041272700043292A000452C2C000472E2E0004931310004B33330004D36360004F3838000513B3B000533D3E0005540400005743430005946460005B48490005D4B4C0005F4E4F000615151000635454000655757000675A5A000695D5D0006B60600006D63630006F666700071696A000736C6D000756F700007773730007976760007B79790007D7C7C0007F7F7F000818283000838586000858889000878B8C000898F8F0008B92920008D95950008F9898000919B9C000939E9F00095A1A200097A4A500099A7A80009BAAAB00",
	mem_init1 => X"09DADAE0009FB0B0000A1B3B3000A3B6B6000A5B8B9000A7BBBC000A9BEBF000ABC1C1000ADC3C4000AFC6C7000B1C8C9000B3CBCC000B5CDCE000B7D0D1000B9D2D3000BBD5D5000BDD7D8000BFD9DA000C1DBDC000C3DDDE000C5E0E0000C7E2E2000C9E4E4000CBE5E6000CDE7E8000CFE9EA000D1EBEC000D3ECED000D5EEEF000D7EFF0000D9F1F2000DBF2F3000DDF4F5000DFF5F6000E1F6F7000E3F7F8000E5F8F9000E7F9FA000E9FAFB000EBFBFC000EDFBFC000EFFCFD000F1FDFE000F3FDFE000F5FEFF000F7FEFF000F9FEFF000FBFEFF000FDFEFF000FFFFFF000FDFEFF000FBFEFF000F9FEFF000F7FEFF000F5FEFF000F3FDFE000F1FDFE0",
	mem_init0 => X"00EFFCFD000EDFBFC000EBFBFC000E9FAFB000E7F9FA000E5F8F9000E3F7F8000E1F6F7000DFF5F6000DDF4F5000DBF2F3000D9F1F2000D7EFF0000D5EEEF000D3ECED000D1EBEC000CFE9EA000CDE7E8000CBE5E6000C9E4E4000C7E2E2000C5E0E0000C3DDDE000C1DBDC000BFD9DA000BDD7D8000BBD5D5000B9D2D3000B7D0D1000B5CDCE000B3CBCC000B1C8C9000AFC6C7000ADC3C4000ABC1C1000A9BEBF000A7BBBC000A5B8B9000A3B6B6000A1B3B30009FB0B00009DADAE0009BAAAB00099A7A800097A4A500095A1A2000939E9F000919B9C0008F98980008D95950008B9292000898F8F000878B8C0008588890008385860008182830007F7F80",
	data_interleave_offset_in_bits => 1,
	data_interleave_width_in_bits => 1,
	init_file => "afgdata.hex",
	init_file_layout => "port_a",
	logical_ram_name => "afg:inst4|altsyncram:altsyncram_component|altsyncram_49r3:auto_generated|ALTSYNCRAM",
	operation_mode => "rom",
	port_a_address_clear => "none",
	port_a_address_width => 8,
	port_a_byte_enable_clock => "none",
	port_a_data_out_clear => "none",
	port_a_data_out_clock => "clock0",
	port_a_data_width => 36,
	port_a_first_address => 0,
	port_a_first_bit_number => 0,
	port_a_last_address => 255,
	port_a_logical_ram_depth => 256,
	port_a_logical_ram_width => 8,
	port_a_read_during_write_mode => "new_data_with_nbe_read",
	port_a_write_enable_clock => "none",
	port_b_address_width => 8,
	port_b_data_width => 36,
	ram_block_type => "M9K")
-- pragma translate_on
PORT MAP (
	portare => VCC,
	clk0 => \CLK~inputclkctrl_outclk\,
	portaaddr => \inst4|altsyncram_component|auto_generated|ram_block1a0_PORTAADDR_bus\,
	devclrn => ww_devclrn,
	devpor => ww_devpor,
	portadataout => \inst4|altsyncram_component|auto_generated|ram_block1a0_PORTADATAOUT_bus\);

-- Location: IOIBUF_X34_Y2_N22
\SW0~input\ : cycloneive_io_ibuf
-- pragma translate_off
GENERIC MAP (
	bus_hold => "false",
	simulate_z_as => "z")
-- pragma translate_on
PORT MAP (
	i => ww_SW0,
	o => \SW0~input_o\);

-- Location: IOIBUF_X32_Y0_N8
\SW1~input\ : cycloneive_io_ibuf
-- pragma translate_off
GENERIC MAP (
	bus_hold => "false",
	simulate_z_as => "z")
-- pragma translate_on
PORT MAP (
	i => ww_SW1,
	o => \SW1~input_o\);

-- Location: LCCOMB_X28_Y16_N28
\inst2|OU[7]~0\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst2|OU[7]~0_combout\ = (\SW0~input_o\ & ((\SW1~input_o\ & ((\inst7|altsyncram_component|auto_generated|q_a\(7)))) # (!\SW1~input_o\ & (\inst6|altsyncram_component|auto_generated|q_a\(7)))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1010100000100000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \SW0~input_o\,
	datab => \SW1~input_o\,
	datac => \inst6|altsyncram_component|auto_generated|q_a\(7),
	datad => \inst7|altsyncram_component|auto_generated|q_a\(7),
	combout => \inst2|OU[7]~0_combout\);

-- Location: LCCOMB_X28_Y16_N30
\inst2|OU[7]~1\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst2|OU[7]~1_combout\ = (\inst2|OU[7]~0_combout\) # ((\inst4|altsyncram_component|auto_generated|q_a\(7) & !\SW0~input_o\))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1100110011101110",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst4|altsyncram_component|auto_generated|q_a\(7),
	datab => \inst2|OU[7]~0_combout\,
	datad => \SW0~input_o\,
	combout => \inst2|OU[7]~1_combout\);

-- Location: LCCOMB_X28_Y16_N8
\inst2|OU[6]~2\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst2|OU[6]~2_combout\ = (\SW0~input_o\ & ((\SW1~input_o\ & (\inst7|altsyncram_component|auto_generated|q_a\(6))) # (!\SW1~input_o\ & ((\inst6|altsyncram_component|auto_generated|q_a\(6))))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1011100000000000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst7|altsyncram_component|auto_generated|q_a\(6),
	datab => \SW1~input_o\,
	datac => \inst6|altsyncram_component|auto_generated|q_a\(6),
	datad => \SW0~input_o\,
	combout => \inst2|OU[6]~2_combout\);

-- Location: LCCOMB_X28_Y16_N18
\inst2|OU[6]~3\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst2|OU[6]~3_combout\ = (\inst2|OU[6]~2_combout\) # ((!\SW0~input_o\ & \inst4|altsyncram_component|auto_generated|q_a\(6)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1101110111001100",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \SW0~input_o\,
	datab => \inst2|OU[6]~2_combout\,
	datad => \inst4|altsyncram_component|auto_generated|q_a\(6),
	combout => \inst2|OU[6]~3_combout\);

-- Location: LCCOMB_X28_Y16_N4
\inst2|OU[5]~4\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst2|OU[5]~4_combout\ = (\SW0~input_o\ & ((\SW1~input_o\ & (\inst7|altsyncram_component|auto_generated|q_a\(5))) # (!\SW1~input_o\ & ((\inst6|altsyncram_component|auto_generated|q_a\(5))))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1011100000000000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst7|altsyncram_component|auto_generated|q_a\(5),
	datab => \SW1~input_o\,
	datac => \inst6|altsyncram_component|auto_generated|q_a\(5),
	datad => \SW0~input_o\,
	combout => \inst2|OU[5]~4_combout\);

-- Location: LCCOMB_X28_Y16_N22
\inst2|OU[5]~5\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst2|OU[5]~5_combout\ = (\inst2|OU[5]~4_combout\) # ((\inst4|altsyncram_component|auto_generated|q_a\(5) & !\SW0~input_o\))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1100110011101110",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst4|altsyncram_component|auto_generated|q_a\(5),
	datab => \inst2|OU[5]~4_combout\,
	datad => \SW0~input_o\,
	combout => \inst2|OU[5]~5_combout\);

-- Location: LCCOMB_X28_Y16_N12
\inst2|OU[4]~6\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst2|OU[4]~6_combout\ = (\SW0~input_o\ & ((\SW1~input_o\ & ((\inst7|altsyncram_component|auto_generated|q_a\(4)))) # (!\SW1~input_o\ & (\inst6|altsyncram_component|auto_generated|q_a\(4)))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1010100000001000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \SW0~input_o\,
	datab => \inst6|altsyncram_component|auto_generated|q_a\(4),
	datac => \SW1~input_o\,
	datad => \inst7|altsyncram_component|auto_generated|q_a\(4),
	combout => \inst2|OU[4]~6_combout\);

-- Location: LCCOMB_X28_Y16_N14
\inst2|OU[4]~7\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst2|OU[4]~7_combout\ = (\inst2|OU[4]~6_combout\) # ((!\SW0~input_o\ & \inst4|altsyncram_component|auto_generated|q_a\(4)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1111111101000100",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \SW0~input_o\,
	datab => \inst4|altsyncram_component|auto_generated|q_a\(4),
	datad => \inst2|OU[4]~6_combout\,
	combout => \inst2|OU[4]~7_combout\);

-- Location: LCCOMB_X28_Y16_N0
\inst2|OU[3]~8\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst2|OU[3]~8_combout\ = (\SW0~input_o\ & ((\SW1~input_o\ & ((\inst7|altsyncram_component|auto_generated|q_a\(3)))) # (!\SW1~input_o\ & (\inst6|altsyncram_component|auto_generated|q_a\(3)))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1100101000000000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst6|altsyncram_component|auto_generated|q_a\(3),
	datab => \inst7|altsyncram_component|auto_generated|q_a\(3),
	datac => \SW1~input_o\,
	datad => \SW0~input_o\,
	combout => \inst2|OU[3]~8_combout\);

-- Location: LCCOMB_X28_Y16_N26
\inst2|OU[3]~9\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst2|OU[3]~9_combout\ = (\inst2|OU[3]~8_combout\) # ((\inst4|altsyncram_component|auto_generated|q_a\(3) & !\SW0~input_o\))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1100110011101110",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst4|altsyncram_component|auto_generated|q_a\(3),
	datab => \inst2|OU[3]~8_combout\,
	datad => \SW0~input_o\,
	combout => \inst2|OU[3]~9_combout\);

-- Location: LCCOMB_X28_Y16_N24
\inst2|OU[2]~10\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst2|OU[2]~10_combout\ = (\SW0~input_o\ & ((\SW1~input_o\ & (\inst7|altsyncram_component|auto_generated|q_a\(2))) # (!\SW1~input_o\ & ((\inst6|altsyncram_component|auto_generated|q_a\(2))))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1010110000000000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst7|altsyncram_component|auto_generated|q_a\(2),
	datab => \inst6|altsyncram_component|auto_generated|q_a\(2),
	datac => \SW1~input_o\,
	datad => \SW0~input_o\,
	combout => \inst2|OU[2]~10_combout\);

-- Location: LCCOMB_X28_Y16_N6
\inst2|OU[2]~11\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst2|OU[2]~11_combout\ = (\inst2|OU[2]~10_combout\) # ((\inst4|altsyncram_component|auto_generated|q_a\(2) & !\SW0~input_o\))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1100110011101110",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst4|altsyncram_component|auto_generated|q_a\(2),
	datab => \inst2|OU[2]~10_combout\,
	datad => \SW0~input_o\,
	combout => \inst2|OU[2]~11_combout\);

-- Location: LCCOMB_X28_Y16_N20
\inst2|OU[1]~12\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst2|OU[1]~12_combout\ = (\SW0~input_o\ & ((\SW1~input_o\ & ((\inst7|altsyncram_component|auto_generated|q_a\(1)))) # (!\SW1~input_o\ & (\inst6|altsyncram_component|auto_generated|q_a\(1)))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1110001000000000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst6|altsyncram_component|auto_generated|q_a\(1),
	datab => \SW1~input_o\,
	datac => \inst7|altsyncram_component|auto_generated|q_a\(1),
	datad => \SW0~input_o\,
	combout => \inst2|OU[1]~12_combout\);

-- Location: LCCOMB_X28_Y16_N10
\inst2|OU[1]~13\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst2|OU[1]~13_combout\ = (\inst2|OU[1]~12_combout\) # ((!\SW0~input_o\ & \inst4|altsyncram_component|auto_generated|q_a\(1)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1111111101000100",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \SW0~input_o\,
	datab => \inst4|altsyncram_component|auto_generated|q_a\(1),
	datad => \inst2|OU[1]~12_combout\,
	combout => \inst2|OU[1]~13_combout\);

-- Location: LCCOMB_X28_Y16_N16
\inst2|OU[0]~14\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst2|OU[0]~14_combout\ = (\SW0~input_o\ & ((\SW1~input_o\ & (\inst7|altsyncram_component|auto_generated|q_a\(0))) # (!\SW1~input_o\ & ((\inst6|altsyncram_component|auto_generated|q_a\(0))))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1010001010000000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \SW0~input_o\,
	datab => \SW1~input_o\,
	datac => \inst7|altsyncram_component|auto_generated|q_a\(0),
	datad => \inst6|altsyncram_component|auto_generated|q_a\(0),
	combout => \inst2|OU[0]~14_combout\);

-- Location: LCCOMB_X28_Y16_N2
\inst2|OU[0]~15\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst2|OU[0]~15_combout\ = (\inst2|OU[0]~14_combout\) # ((!\SW0~input_o\ & \inst4|altsyncram_component|auto_generated|q_a\(0)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1111111101000100",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \SW0~input_o\,
	datab => \inst4|altsyncram_component|auto_generated|q_a\(0),
	datad => \inst2|OU[0]~14_combout\,
	combout => \inst2|OU[0]~15_combout\);

-- Location: LCCOMB_X16_Y22_N18
\inst5|Mux0~1\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst5|Mux0~1_combout\ = (\inst5|q\(3) & (!\inst5|q\(1) & (\inst5|q\(2) $ (!\inst5|q\(4))))) # (!\inst5|q\(3) & (!\inst5|q\(2) & (\inst5|q\(4) & \inst5|q\(1))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0001000010000010",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst5|q\(3),
	datab => \inst5|q\(2),
	datac => \inst5|q\(4),
	datad => \inst5|q\(1),
	combout => \inst5|Mux0~1_combout\);

-- Location: LCCOMB_X16_Y22_N24
\inst5|Mux0~0\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst5|Mux0~0_combout\ = (\inst5|q\(3) & (((\inst5|q\(2) & !\inst5|q\(4))) # (!\inst5|q\(1)))) # (!\inst5|q\(3) & (\inst5|q\(1) $ (((\inst5|q\(2) & !\inst5|q\(4))))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0101100110101110",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst5|q\(3),
	datab => \inst5|q\(2),
	datac => \inst5|q\(4),
	datad => \inst5|q\(1),
	combout => \inst5|Mux0~0_combout\);

-- Location: LCCOMB_X16_Y22_N28
\inst5|Mux0~2\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst5|Mux0~2_combout\ = (\inst5|q\(0) & (!\inst5|Mux0~1_combout\)) # (!\inst5|q\(0) & ((\inst5|Mux0~0_combout\)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0011111100110000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	datab => \inst5|Mux0~1_combout\,
	datac => \inst5|q\(0),
	datad => \inst5|Mux0~0_combout\,
	combout => \inst5|Mux0~2_combout\);

-- Location: LCCOMB_X21_Y22_N26
\inst5|Mux1~1\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst5|Mux1~1_combout\ = (\inst5|q\(3) & (\inst5|q\(1) $ (((\inst5|q\(4)) # (!\inst5|q\(2)))))) # (!\inst5|q\(3) & ((\inst5|q\(2) & (!\inst5|q\(1) & !\inst5|q\(4))) # (!\inst5|q\(2) & (\inst5|q\(1) & \inst5|q\(4)))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0001101010000110",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst5|q\(3),
	datab => \inst5|q\(2),
	datac => \inst5|q\(1),
	datad => \inst5|q\(4),
	combout => \inst5|Mux1~1_combout\);

-- Location: LCCOMB_X21_Y22_N28
\inst5|Mux1~0\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst5|Mux1~0_combout\ = (\inst5|q\(3) & ((\inst5|q\(2) $ (\inst5|q\(4))) # (!\inst5|q\(1)))) # (!\inst5|q\(3) & ((\inst5|q\(1)) # (\inst5|q\(2) $ (\inst5|q\(4)))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0111101111011110",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst5|q\(3),
	datab => \inst5|q\(2),
	datac => \inst5|q\(1),
	datad => \inst5|q\(4),
	combout => \inst5|Mux1~0_combout\);

-- Location: LCCOMB_X21_Y22_N24
\inst5|Mux1~2\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst5|Mux1~2_combout\ = (\inst5|q\(0) & ((\inst5|Mux1~0_combout\))) # (!\inst5|q\(0) & (\inst5|Mux1~1_combout\))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1111101000001010",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst5|Mux1~1_combout\,
	datac => \inst5|q\(0),
	datad => \inst5|Mux1~0_combout\,
	combout => \inst5|Mux1~2_combout\);

-- Location: LCCOMB_X21_Y22_N10
\inst5|Mux2~0\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst5|Mux2~0_combout\ = (\inst5|q\(3) & (\inst5|q\(2) & (!\inst5|q\(1) & !\inst5|q\(4)))) # (!\inst5|q\(3) & (\inst5|q\(1) & (\inst5|q\(2) $ (!\inst5|q\(4)))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0100000000011000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst5|q\(3),
	datab => \inst5|q\(2),
	datac => \inst5|q\(1),
	datad => \inst5|q\(4),
	combout => \inst5|Mux2~0_combout\);

-- Location: LCCOMB_X21_Y22_N20
\inst5|Mux2~1\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst5|Mux2~1_combout\ = (\inst5|q\(0) & !\inst5|Mux2~0_combout\)

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0000000011110000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	datac => \inst5|q\(0),
	datad => \inst5|Mux2~0_combout\,
	combout => \inst5|Mux2~1_combout\);

-- Location: LCCOMB_X21_Y22_N2
\inst5|Mux3~0\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst5|Mux3~0_combout\ = (\inst5|q\(0) & ((!\inst5|Mux2~0_combout\))) # (!\inst5|q\(0) & (\inst5|Mux0~0_combout\))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0000110011111100",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	datab => \inst5|Mux0~0_combout\,
	datac => \inst5|q\(0),
	datad => \inst5|Mux2~0_combout\,
	combout => \inst5|Mux3~0_combout\);

-- Location: LCCOMB_X21_Y22_N0
\inst5|Mux4~0\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst5|Mux4~0_combout\ = (\inst5|Mux1~0_combout\) # (!\inst5|q\(0))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1111111100001111",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	datac => \inst5|q\(0),
	datad => \inst5|Mux1~0_combout\,
	combout => \inst5|Mux4~0_combout\);

-- Location: LCCOMB_X21_Y22_N30
\inst5|Mux5~0\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst5|Mux5~0_combout\ = (\inst5|q\(2) & (!\inst5|q\(4) & (\inst5|q\(3) $ (!\inst5|q\(1))))) # (!\inst5|q\(2) & (\inst5|q\(3) & (!\inst5|q\(1) & \inst5|q\(4))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0000001010000100",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst5|q\(3),
	datab => \inst5|q\(2),
	datac => \inst5|q\(1),
	datad => \inst5|q\(4),
	combout => \inst5|Mux5~0_combout\);

-- Location: LCCOMB_X21_Y22_N16
\inst5|Mux6~0\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst5|Mux6~0_combout\ = (\inst5|q\(2) & (\inst5|q\(4) $ (((\inst5|q\(1)) # (!\inst5|q\(3)))))) # (!\inst5|q\(2) & ((\inst5|q\(4)) # ((\inst5|q\(3) & !\inst5|q\(1)))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0011101111000110",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst5|q\(3),
	datab => \inst5|q\(2),
	datac => \inst5|q\(1),
	datad => \inst5|q\(4),
	combout => \inst5|Mux6~0_combout\);

-- Location: LCCOMB_X21_Y22_N14
\inst5|Mux6~1\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst5|Mux6~1_combout\ = (\inst5|Mux6~0_combout\) # (\inst5|q\(3) $ (\inst5|q\(0) $ (\inst5|q\(1))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1111111110010110",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst5|q\(3),
	datab => \inst5|q\(0),
	datac => \inst5|q\(1),
	datad => \inst5|Mux6~0_combout\,
	combout => \inst5|Mux6~1_combout\);

-- Location: LCCOMB_X16_Y22_N30
\inst5|Mux7~0\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst5|Mux7~0_combout\ = (\inst5|q\(3)) # ((\inst5|q\(2)) # ((\inst5|q\(0) & \inst5|q\(1))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1111111111111000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst5|q\(0),
	datab => \inst5|q\(1),
	datac => \inst5|q\(3),
	datad => \inst5|q\(2),
	combout => \inst5|Mux7~0_combout\);

-- Location: LCCOMB_X16_Y22_N8
\inst5|Mux7~1\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst5|Mux7~1_combout\ = (\inst5|q\(4) & \inst5|Mux7~0_combout\)

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1010000010100000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst5|q\(4),
	datac => \inst5|Mux7~0_combout\,
	combout => \inst5|Mux7~1_combout\);

-- Location: LCCOMB_X16_Y22_N22
\inst5|Mux8~0\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst5|Mux8~0_combout\ = (\inst5|q\(3) & ((\inst5|q\(0)) # ((\inst5|q\(1)) # (\inst5|q\(2)))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1111000011100000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst5|q\(0),
	datab => \inst5|q\(1),
	datac => \inst5|q\(3),
	datad => \inst5|q\(2),
	combout => \inst5|Mux8~0_combout\);

-- Location: LCCOMB_X16_Y22_N4
\inst5|Mux8~1\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst5|Mux8~1_combout\ = (\inst5|q\(4)) # (\inst5|Mux8~0_combout\)

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1111101011111010",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst5|q\(4),
	datac => \inst5|Mux8~0_combout\,
	combout => \inst5|Mux8~1_combout\);

-- Location: LCCOMB_X21_Y22_N12
\inst5|Mux9~0\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst5|Mux9~0_combout\ = (\inst5|q\(3) & ((\inst5|q\(0)) # (\inst5|q\(1)))) # (!\inst5|q\(3) & (\inst5|q\(0) & \inst5|q\(1)))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1110100011101000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst5|q\(3),
	datab => \inst5|q\(0),
	datac => \inst5|q\(1),
	combout => \inst5|Mux9~0_combout\);

-- Location: LCCOMB_X21_Y22_N22
\inst5|Mux9~1\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst5|Mux9~1_combout\ = (\inst5|Mux9~0_combout\ & (\inst5|q\(3) & ((\inst5|q\(2)) # (!\inst5|q\(4))))) # (!\inst5|Mux9~0_combout\ & ((\inst5|q\(2) & (\inst5|q\(3) & !\inst5|q\(4))) # (!\inst5|q\(2) & (!\inst5|q\(3) & \inst5|q\(4)))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1000000111100000",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst5|Mux9~0_combout\,
	datab => \inst5|q\(2),
	datac => \inst5|q\(3),
	datad => \inst5|q\(4),
	combout => \inst5|Mux9~1_combout\);

-- Location: LCCOMB_X16_Y22_N26
\inst5|Mux10~0\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst5|Mux10~0_combout\ = (\inst5|q\(4) & ((\inst5|Mux7~0_combout\))) # (!\inst5|q\(4) & (!\inst5|Mux8~0_combout\))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "1010111100000101",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst5|q\(4),
	datac => \inst5|Mux8~0_combout\,
	datad => \inst5|Mux7~0_combout\,
	combout => \inst5|Mux10~0_combout\);

-- Location: LCCOMB_X21_Y22_N8
\inst5|Mux11~0\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst5|Mux11~0_combout\ = (\inst5|q\(3) & (((!\inst5|q\(0) & !\inst5|q\(1))) # (!\inst5|q\(2)))) # (!\inst5|q\(3) & ((\inst5|q\(2)) # ((\inst5|q\(0) & \inst5|q\(1)))))

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0101011111101010",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	dataa => \inst5|q\(3),
	datab => \inst5|q\(0),
	datac => \inst5|q\(1),
	datad => \inst5|q\(2),
	combout => \inst5|Mux11~0_combout\);

-- Location: LCCOMB_X21_Y22_N6
\inst5|Mux11~1\ : cycloneive_lcell_comb
-- Equation(s):
-- \inst5|Mux11~1_combout\ = (!\inst5|q\(4)) # (!\inst5|Mux11~0_combout\)

-- pragma translate_off
GENERIC MAP (
	lut_mask => "0000111111111111",
	sum_lutc_input => "datac")
-- pragma translate_on
PORT MAP (
	datac => \inst5|Mux11~0_combout\,
	datad => \inst5|q\(4),
	combout => \inst5|Mux11~1_combout\);

ww_DACLK <= \DACLK~output_o\;

ww_DAC(7) <= \DAC[7]~output_o\;

ww_DAC(6) <= \DAC[6]~output_o\;

ww_DAC(5) <= \DAC[5]~output_o\;

ww_DAC(4) <= \DAC[4]~output_o\;

ww_DAC(3) <= \DAC[3]~output_o\;

ww_DAC(2) <= \DAC[2]~output_o\;

ww_DAC(1) <= \DAC[1]~output_o\;

ww_DAC(0) <= \DAC[0]~output_o\;

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
END structure;


