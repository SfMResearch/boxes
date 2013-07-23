
#include <iostream>

#include <boxes.h>

using namespace std;

int main(int argc, char **argv) {
	Boxes::Boxes boxes;

	cout << boxes.version_string() << endl;
}
