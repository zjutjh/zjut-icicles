library ieee;
use ieee.std_logic_1164.all;

entity control is
	port(clk: in std_logic;
	cs, clr, le: out std_logic
	);
end control;

architecture behav of control is
signal current_state,next_state: std_logic_vector(3 downto 0);
constant st0: std_logic_vector:="0011";
constant st1: std_logic_vector:="0010";
constant st2: std_logic_vector:="0110";
constant st3: std_logic_vector:="0111";
constant st4: std_logic_vector:="0101";
constant st5: std_logic_vector:="0100";
constant st6: std_logic_vector:="1100";
constant st7: std_logic_vector:="1101";
constant st8: std_logic_vector:="1111";
constant st9: std_logic_vector:="1110";
constant st10: std_logic_vector:="1010";
constant st11: std_logic_vector:="1011";

begin
com1:process(current_state)
	begin
	case current_state is
	when st0=>next_state<=st1;clr<='1';cs<='0';le<='0';
	when st1=>next_state<=st2;clr<='0';cs<='1';le<='0';
	when st2=>next_state<=st3;clr<='0';cs<='1';le<='0';
	when st3=>next_state<=st4;clr<='0';cs<='1';le<='0';
	when st4=>next_state<=st5;clr<='0';cs<='1';le<='0';
	when st5=>next_state<=st6;clr<='0';cs<='1';le<='0';
	when st6=>next_state<=st7;clr<='0';cs<='1';le<='0';
	when st7=>next_state<=st8;clr<='0';cs<='1';le<='0';
	when st8=>next_state<=st9;clr<='0';cs<='1';le<='0';
	when st9=>next_state<=st10;clr<='0';cs<='0';le<='0';
	when st10=>next_state<=st11;clr<='0';cs<='0';le<='1';
	when st11=>next_state<=st0;clr<='0';cs<='0';le<='0';
	when others=>next_state<=st0;clr<='0';cs<='0';le<='0';
end case;
end process com1;

reg: process(clk)
	begin
	 if(clk'event and clk='1') then
	 current_state<=next_state;
	 end if;
	 end process reg;
	 end behav;