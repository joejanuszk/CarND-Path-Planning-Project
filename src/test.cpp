#include <iostream>
#include "utils.hpp"

using namespace std;

int main() {
    CarState cs_loop = { 0, 0, 0, 0 };
    printCarState(cs_loop);
    for (int i = 0; i < 50; ++i) {
        cs_loop = getNextMaxSafeForwardCarState(cs_loop);
        printCarState(cs_loop);
    }

    return 0;
}
