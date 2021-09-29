#include <iostream>
#include "theta.hpp"
using namespace std;

int main() {
	point from = { 8,10 }, to = { 1,15 };
	vector<point> result, walls = {
		{5,8}, {5,9}, { 5,10 }, { 5,11 }, { 5,12 }, { 5,13 }, {6,14}, {7,15}
	};
	theta(from, to, walls, result);
	print(from, to, walls, result);

	cout << endl << endl;
	walls = {
		{4,9}, {5,9}, { 5,10 }, { 5,11 }, { 5,12 }, { 5,13 }, {6,14}, {7,15}, {7,16}
	};
	result.clear();
	theta(from, to, walls, result);
	print(from, to, walls, result);


	return 0;
}
