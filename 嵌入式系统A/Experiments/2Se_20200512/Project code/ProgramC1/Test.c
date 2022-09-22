#define 	uint8 	unsigned char
#define 	uint32 	unsigned int

extern uint32 	Add(uint32 x, uint32 y);

uint32 		sum;

void Main(void)
{
    sum=Add(555,168);
    while(1);
}