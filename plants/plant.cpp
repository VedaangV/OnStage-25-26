#include <iostream>
#include <winsock.h>
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

int main() {

	Plant self = Plant();

	int serverSocket = socket(AF_INET, SOCK_STREAM, 0);

	sockaddr_in serverAddress;
	serverAddress.sin_family = AF_INET;
	serverAddress.sin_port = htons(5000);
	serverAddress.sin_addr.s_addr = INADDR_ANY;

	bind(serverSocket, (struct sockaddr*)&serverAddress, sizeof(serverAddress));

	while (self.fully_grown == false) {

		listen(serverSocket, 5);
		int client = accept(serverSocket, nullptr, nullptr);

		char data[2] = { 0 }; // either one character to grow, or two for coords
		recv(serverSocket, data, sizeof(data), 0);

		if (data[0] == 'G') self.grow();
		else { // we have received coords
			self.x = data[0];
			self.y = data[1];
		}

	}

}
