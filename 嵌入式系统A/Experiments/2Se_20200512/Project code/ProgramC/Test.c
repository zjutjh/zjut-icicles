#define uint8 	unsigned char
#define uint32 	unsigned int
#define N 		100

uint32 sum;

void Main(void){
    uint32 i;
    
    sum = 0;
    for(i=0;i<=N;i++){
        sum += i;
    }
    while(1);
}