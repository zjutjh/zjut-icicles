// C++ How to Program
// Debugging Problem (animal.hpp)
#ifndef ANIMAL_HPP
#define ANIMAL_HPP

class Animal {
public:
	Animal(const int = 0, const int = 0);
	void print() const;
	int getHeight() const;
	int getWeight() const;
	void setHeight(int);
	void setWeight(int);
	const char* getName() const;
private:
	int height;
	int weight;
	char name[30];
};

#endif
