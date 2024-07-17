#include<iostream>
#include<cstring>
using namespace std;
bool myStrNCpy(char* to, const  char* from, unsigned startpos, unsigned len) {
	if (startpos<0 || (startpos + len)>strlen(from)) {
		cout << "copy failed!\n";
		return false;
	}
	for (int i = 0; i < len; i++) *(to + i) = *(from + i + startpos);
	*(to + len) = '\0';
	return true;
}
int main() {
	char to[200], from[200];
	cin >> from;
	myStrNCpy(to, from, 0, 0);  cout << to << endl;
	myStrNCpy(to, from, 3, 4);  cout << to << endl;
	myStrNCpy(to, from, 3, 20);  cout << to << endl;
	myStrNCpy(to, "abcdefghijklmn", 0, 0);    cout << to << endl;
	myStrNCpy(to, "abcdefghijklmn", 3, 4);    cout << to << endl;
	myStrNCpy(to, "abcdefghijklmn", 3, 20);   cout << to << endl;

}