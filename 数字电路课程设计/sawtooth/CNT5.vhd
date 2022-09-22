library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.std_logic_unsigned.all;

entity CNT5 is
	port(CLK: in std_logic;
			CO: out std_logic);
end CNT5;


architecture one of CNT5 is
signal Q:std_logic_vector(2 downto 0);
	begin
		process(CLK)
			begin
				
				if(CLK'EVENT and CLK = '1') then
					
						if(Q = 4) then
							Q <= "000";
						else
							Q <= Q + 1;
						end if;
					
				end if;
			end process;
		process (Q)
		begin
			if(Q = 4) then
				CO <= '1';
			else
				CO <= '0';
			end if;
		end process;
	end;