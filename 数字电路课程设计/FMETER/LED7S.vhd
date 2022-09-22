library ieee;
use ieee.std_logic_1164.all;

entity LED7S is
port(DIN: in std_logic_vector(3 downto 0);
	  Y: out std_logic_vector(6 downto 0)
);
end;

architecture one of LED7S is
begin
	process(DIN)
		begin
			case DIN is
				when "0000" => Y <= "0111111";  --显示0；
				when "0001" => Y <= "0000110";  --显示1；
				when "0010" => Y <= "1011011";  --显示2；
				when "0011" => Y <= "1001111";  --显示3；
				when "0100" => Y <= "1100110";  --显示4；
				when "0101" => Y <= "1101101";  --显示5；
				when "0110" => Y <= "1111101";  --显示6；
				when "0111" => Y <= "0000111";  --显示7；
				when "1000" => Y <= "1111111";  --显示8；
				when "1001" => Y <= "1101111";  --显示9；
				when "1010" => Y <= "1110111";  --显示A；
				when "1011" => Y <= "1111100";  --显示B；
				when "1100" => Y <= "0111001";  --显示C；
				when "1101" => Y <= "1011110";  --显示D；
				when "1110" => Y <= "1111001";  --显示E；
				when "1111" => Y <= "1110001";  --显示F；
				when others => Y <= null;
		end case;
	end process;
end one;