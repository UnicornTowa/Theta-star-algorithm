#include <iostream>
#include "theta.h"

using namespace std;

int main() {

    point from = { 2,2 }, to = { 17,15 };
    vector<point> result, walls = {
            { 0 , 0 },
            { 0 , 1 },
            { 0 , 2 },
            { 0 , 3 },
            { 0 , 4 },
            { 0 , 5 },
            { 0 , 6 },
            { 0 , 7 },
            { 0 , 8 },
            { 0 , 9 },
            { 0 , 10 },
            { 0 , 11 },
            { 0 , 12 },
            { 0 , 13 },
            { 0 , 14 },
            { 0 , 15 },
            { 0 , 16 },
            { 0 , 17 },
            { 0 , 18 },
            { 0 , 19 },
            { 1 , 0 },
            { 1 , 6 },
            { 1 , 19 },
            { 2 , 0 },
            { 2 , 6 },
            { 2 , 19 },
            { 3 , 0 },
            { 3 , 6 },
            { 3 , 13 },
            { 3 , 19 },
            { 4 , 0 },
            { 4 , 6 },
            { 4 , 13 },
            { 4 , 19 },
            { 5 , 0 },
            { 5 , 1 },
            { 5 , 2 },
            { 5 , 3 },
            { 5 , 4 },
            { 5 , 8 },
            { 5 , 9 },
            { 5 , 10 },
            { 5 , 11 },
            { 5 , 12 },
            { 5 , 13 },
            { 5 , 14 },
            { 5 , 15 },
            { 5 , 16 },
            { 5 , 17 },
            { 5 , 19 },
            { 6 , 0 },
            { 6 , 4 },
            { 6 , 8 },
            { 6 , 16 },
            { 6 , 19 },
            { 7 , 0 },
            { 7 , 4 },
            { 7 , 5 },
            { 7 , 6 },
            { 7 , 7 },
            { 7 , 8 },
            { 7 , 9 },
            { 7 , 10 },
            { 7 , 11 },
            { 7 , 12 },
            { 7 , 13 },
            { 7 , 14 },
            { 7 , 16 },
            { 7 , 19 },
            { 8 , 0 },
            { 8 , 16 },
            { 8 , 19 },
            { 9 , 0 },
            { 9 , 16 },
            { 9 , 19 },
            { 10 , 0 },
            { 10 , 2 },
            { 10 , 3 },
            { 10 , 4 },
            { 10 , 5 },
            { 10 , 6 },
            { 10 , 7 },
            { 10 , 8 },
            { 10 , 9 },
            { 10 , 10 },
            { 10 , 16 },
            { 10 , 19 },
            { 11 , 0 },
            { 11 , 8 },
            { 11 , 10 },
            { 11 , 11 },
            { 11 , 12 },
            { 11 , 13 },
            { 11 , 14 },
            { 11 , 19 },
            { 12 , 0 },
            { 12 , 8 },
            { 12 , 14 },
            { 12 , 19 },
            { 13 , 0 },
            { 13 , 1 },
            { 13 , 2 },
            { 13 , 3 },
            { 13 , 4 },
            { 13 , 5 },
            { 13 , 6 },
            { 13 , 8 },
            { 13 , 9 },
            { 13 , 10 },
            { 13 , 11 },
            { 13 , 12 },
            { 13 , 13 },
            { 13 , 14 },
            { 13 , 15 },
            { 13 , 16 },
            { 13 , 17 },
            { 13 , 18 },
            { 13 , 19 },
            { 14 , 0 },
            { 14 , 8 },
            { 14 , 19 },
            { 15 , 0 },
            { 15 , 2 },
            { 15 , 3 },
            { 15 , 4 },
            { 15 , 5 },
            { 15 , 6 },
            { 15 , 8 },
            { 15 , 13 },
            { 15 , 14 },
            { 15 , 15 },
            { 15 , 16 },
            { 15 , 17 },
            { 15 , 19 },
            { 16 , 0 },
            { 16 , 2 },
            { 16 , 6 },
            { 15 , 7 },
            { 16 , 13 },
            { 16 , 17 },
            { 16 , 19 },
            { 17 , 0 },
            { 17 , 2 },
            { 17 , 4 },
            { 17 , 6 },
            { 17 , 8 },
            { 17 , 13 },
            { 17 , 19 },
            { 18 , 0 },
            { 18 , 4 },
            { 18 , 8 },
            { 18 , 13 },
            { 18 , 17 },
            { 18 , 19 },
            { 19 , 0 },
            { 19 , 1 },
            { 19 , 2 },
            { 19 , 3 },
            { 19 , 4 },
            { 19 , 5 },
            { 19 , 6 },
            { 19 , 7 },
            { 19 , 8 },
            { 19 , 9 },
            { 19 , 10 },
            { 19 , 11 },
            { 19 , 12 },
            { 19 , 13 },
            { 19 , 14 },
            { 19 , 15 },
            { 19 , 16 },
            { 19 , 17 },
            { 19 , 18 },
            { 19 , 19 }


    };
    theta(from, to, walls, result);
    print(from, to, walls, result);

    cout << endl << endl;



    return 0;
}
