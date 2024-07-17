// How to Program
// Debugging Problem (dog.hpp)

#ifndef DOG_HPP
#define DOG_HPP

#include "animal.hpp"


class Dog : public Animal {
public:

	Dog(const int=0, const int=0, const char* = "Toto");
	void Print() const;
	void setName(const char*);
	Dog& operator=(const Animal& right);
private:
	char name[30];
};

#endif
