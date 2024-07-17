int ReplaceMax1(int *a,int size) {
	int i,max = 0;
	for(i=1;i<size;i++)
		if(a[max]<a[i])
			max = i;
	return max;
}
int* ReplaceMax2(int *a,int size){
	int *p=&a[0];
	int i;
	for(i=1;i<size;i++)
		if(a[i]>*p){
			p=&a[i];
		}
	return p;
}
int& ReplaceMax3(int *a,int size){
	int j=0,max=a[0];
	for(int i=1;i<size;i++)
		if(a[i]>max){
			max = a[i] ;
			j=i;
		}
	return a[j];
}
