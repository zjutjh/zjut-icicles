library ieee;
use ieee.std_logic_1164.all;
use ieee.std_logic_unsigned.all;

entity CNT10 is
	port(CLK, CLR, CS: in std_logic;
				Q: buffer std_logic_vector(3 downto 0);
				CO: out std_logic);
end CNT10;

architecture one of CNT10 is
	begin
		process(CLK, CLR, CS)
			begin
				if(CLR='1') then
					Q <= "0000";      --异步清零
				elsif(CLK'event and CLK='0') then
				 if(CS = '1') then	--计数使能
				  if(Q = 9) then     --十进制加法计数
						Q<="0000";
				  else
						Q <= Q + 1;
				  end if;
				 end if;
			  end if;
		end process;
		process(Q)
		begin
			if(Q = 9)then
				CO <= '1';    			--进位输出
			else
				CO <= '0';
			end if;
		end process;
	end;