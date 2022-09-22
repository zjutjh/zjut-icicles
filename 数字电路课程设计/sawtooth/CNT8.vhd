library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.std_logic_unsigned.all;

entity CNT8 is
	port(CLK, CLR,CS: in std_logic;
			Q: buffer std_logic_vector(7 downto 0);
			CO: out std_logic);
end CNT8;


architecture one of CNT8 is
	begin
		process(CLK,CLR,CS)
			begin
				if(CLR = '1') then
					Q <= "00000000";	--异步清零
				elsif(CLK'EVENT and CLK = '1') then
					if(CS = '1') then
						if(Q = 255) then
							Q <= "00000000";
						else
							Q <= Q + 1;
						end if;
					end if;
				end if;
			end process;
		process (Q)
		begin
			if(Q = 255) then
				CO <= '1';
			else
				CO <= '0';
			end if;
		end process;
	end;