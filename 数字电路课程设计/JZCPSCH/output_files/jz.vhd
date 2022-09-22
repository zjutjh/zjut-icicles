library ieee;
use ieee.std_logic_1164.all;

entity jz is
port(
	A,B,C:in std_logic;
	Y:out std_logic);
end jz;

architecture one of jz is
begin
	Y<=(A and B) or (A and C);
end one;