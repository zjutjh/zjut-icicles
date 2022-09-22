library ieee;
use ieee.std_logic_1164.all;

use ieee.std_logic_unsigned.all;
use ieee.std_logic_arith.all;

entity phase_acc is
	port(
			clk:in std_logic;
			freqin:in std_logic_vector(31 downto 0);
			romaddr:out std_logic_vector(7 downto 0));
end phase_acc;

architecture one of phase_acc is 
		signal acc:std_logic_vector(31 downto 0);
begin	
	process(clk)
	begin
		if(clk'event and clk='1')then
			acc<=acc-freqin;
			end if;
		end process;
		romaddr<=acc(31 downto 24);
end one;