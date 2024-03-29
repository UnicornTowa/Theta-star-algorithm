# Theta-star-algorithm on C++20

Any-angle path planning algorithm that works with cells in the coordinate plane.
#### Prepared by first-year students of Saint Petersburg State University Arzha Anton and Mihaylov Ivan
## How to use
To work with the program you need to edit the main.cpp and make adjustments to it according to your task.
```cpp
int main() {
    point from = { X, Y }; // coordinates of start point
    point to = { X, Y }; // coordinates of target
    vector<point> walls = {
    { X1, Y1 }, //coordinates of walls
    { X2, Y2 },
    ...
    { Xn, Yn}
    };
    ...
}
```
## Program output
Default program will give you output as a "image" where "@" marks start and target points, "X" marks walls and "\*" marks a point where the direction of moving changes  (connected "@" and "\*" = desired path). 
### Example
![alt text](https://raw.githubusercontent.com/UnicornTowa/Theta-star-algorythm/main/output_example.jpg)
## Tests (visualisation)
##### Test 1
![alt text](https://raw.githubusercontent.com/UnicornTowa/Theta-star-algorythm/main/new.png)
##### Test 2
![alt text](https://raw.githubusercontent.com/UnicornTowa/Theta-star-algorythm/c6a5c51703faa64ac84838857b7a54ce62699e3e/test2.svg)
##### Test 3
![alt text](https://raw.githubusercontent.com/UnicornTowa/Theta-star-algorythm/c6a5c51703faa64ac84838857b7a54ce62699e3e/last.svg)

### End
