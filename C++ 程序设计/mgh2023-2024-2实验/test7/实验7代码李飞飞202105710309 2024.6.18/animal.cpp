// C++ How to Program
// Debugging Problem (animal.cpp)

#include <iostream>
using std::cout;
using std::endl;

#include "animal.hpp"

Animal::Animal(const int h , const int w )
{
    height = h;
    weight = w;
}

void Animal::print() const
{
    cout << "This animal's height and weight are as follows\n"
        << "Height: " << height << "\tWeight: " << weight << endl << endl;
}

int Animal::getHeight() const { return height; }

int Animal::getWeight() const { return weight; }

void Animal::setHeight(const int h) { height = h; }

void Animal::setWeight(const int w) { weight = w; }

const char* Animal::getName() const { return name; }
