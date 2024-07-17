#include <iostream>
#include <string>
using namespace std; 
class Vehicle{
protected:
    int numberOfDoors;
    int numberOfCylinders;
    int transmissionType;
    string color;
    double fuelLevel;

public:
    Vehicle(int doors = 2, int cylinders = 6, int transmission = 3, string color_ = "blue", double fuel = 0)
        : numberOfDoors(doors), numberOfCylinders(cylinders), transmissionType(transmission), color(color_), fuelLevel(fuel) {}
    void printInfo() const {
        cout<< "Vehicle\n"
            << "    Number of doors: " << numberOfDoors << "\n"
            << "    Number of cylinders: " << numberOfCylinders << "\n"
            << "    Transmission type: " << transmissionType << "\n"
            << "    Color: " << color << "\n"
            << "    Fuel level: " << fuelLevel << "\n";
    }

    void setColor(string newColor) { color = newColor; }
    string getColor() const { return color; }
};
class Taxi : public Vehicle {
private:
    bool passenger;
public:
    Taxi(int doors = 4, int cylinders = 6, int transmission = 5, string color_ = "yellow", double fuel = 0, bool passenger = false)
        : Vehicle(doors, cylinders, transmission, color_, fuel), passenger(passenger) {}
    void printInfo() const {
        Vehicle::printInfo();
        cout << "    The taxi has " << (passenger ? "" : "no ") << "passengers.\n";
    }

    void setHasPassenger(bool status) { passenger = status; }
    bool getHasPassenger() const { return passenger; }
};
class Truck : public Vehicle {
private:
    bool cargo;

public:
    Truck(int doors = 2, int cylinders = 16, int transmission = 8, string color_ = "black", double fuel = 0, bool cargo = true)
        : Vehicle(doors, cylinders, transmission, color_, fuel), cargo(cargo) {}

    void printInfo() const {
        Vehicle::printInfo();
        cout << "    The truck is " << (cargo ? "" : "not ") << "carrying cargo.\n";
    }
    void setIsCarryingCargo(bool status) { cargo = status; }
    bool getIsCarryingCargo() const { return cargo; }
};
int main() {
    Vehicle vehicle;
    Taxi taxi;
    Truck truck;
    vehicle.printInfo();
    cout << endl;
    taxi.printInfo();
    cout << endl;
    truck.printInfo();
    return 0;
}
