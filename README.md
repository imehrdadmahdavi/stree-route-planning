# Street Route Planning

This project uses real map data and A* search to find a path between two points, just as you might see in a desktop or mobile mapping application. The project uses data from the [OpenStreetMap](https://www.openstreetmap.org/) open source project. Maps of this project are completely generated by individuals who volunteer to perform ground surveys of their local environment.

<img src="map.png" width="600" height="450" />

## Cloning

When cloning this project, be sure to use the `--recurse-submodules` flag to import sub modules.
```
git clone https://github.com/udacity/CppND-Route-Planning-Project.git --recurse-submodules
```


## Dependencies for Running Locally


* cmake >= 3.11.3
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 7.4.0
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same instructions as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* IO2D
  * Installation instructions for all operating systems can be found [here](https://github.com/cpp-io2d/P0267_RefImpl/blob/master/BUILDING.md)
  * This library must be built in a place where CMake `find_package` will be able to find it
  
This project was build on an Ubuntu 18.04 machine. To install all of the above dependencies in Ubuntu, run the below commands in the Home directory:
     
```
sudo apt update
sudo apt install build-essential
sudo apt install cmake
sudo apt install libcairo2-dev
sudo apt install libgraphicsmagick1-dev
sudo apt install libpng-dev

git clone --recurse-submodules https://github.com/cpp-io2d/P0267_RefImpl
cd P0267_RefImpl
mkdir Debug
cd Debug
cmake --config Debug "-DCMAKE_BUILD_TYPE=Debug" ..
cmake --build .
sudo make install
```

## Compiling and Running

### Compiling
To compile the project, first, create a `build` directory and change to that directory:
```
mkdir build && cd build
```
From within the `build` directory, then run `cmake` and `make` as follows:
```
cmake ..
make
```
### Running
The executable will be placed in the `build` directory. From within `build`, you can run the project as follows:
```
./OSM_A_star_search
```
Or to specify a map file:
```
./OSM_A_star_search -f ../<your_osm_file.osm>
```

## Testing

The testing executable is also placed in the `build` directory. From within `build`, you can run the unit tests as follows:
```
./test
```
## Code Strucutre

<img src="code-structure.png" width="600" height="450" />
![Code Structure](/code-structure.png)
