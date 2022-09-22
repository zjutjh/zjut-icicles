#define 	uint8 	unsigned char
#define 	uint32 	unsigned int

extern uint32 	Sub(uint32 x, uint32 y);

uint32 		sum;

void Main(void)
{
    sum=Sub(666,234);
    while(1);
}