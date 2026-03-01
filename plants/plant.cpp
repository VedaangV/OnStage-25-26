#include <iostream>
using namespace std;

#define MAX_GROWTH_STAGE 4

class Plant {

	private:

	int growth_stage;

	public:

	int x, y;
	bool fully_grown;

	Plant() {
		x = 0;
		y = 0;
		growth_stage = 0;
		fully_grown = false;
	}

	bool grow() { //returns true if the max growth has been achieved
		growth_stage++;
		if (growth_stage >= MAX_GROWTH_STAGE) {
			fully_grown = true;
			return true;
		} else {
			return false;
		}
	}

};
