// C++ How to Program
// Debugging Problem (lion.hpp)

#ifndef LION_HPP
#define LION_HPP

#include "animal.hpp"

class Lion:public Animal {
public:
	Lion(const int = 0, const int = 0);
	void print() const;
};

#endif
