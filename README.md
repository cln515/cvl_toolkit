# CVL-ToolKit
utilities for miscellaneous computation

## how to build
### dependency
Windows 10
Eigen3(https://eigen.tuxfamily.org/index.php?title=Main_Page)

```
Download and change directory to Eigen3
$ mkdir build & cd build
```
CMake & Install

json(https://github.com/nlohmann/json)
```
$ git clone https://github.com/nlohmann/json
$ cd json
$ mkdir build & cd build
```
CMake & Install

```
$ git clone https://git.cvl.iis.u-tokyo.ac.jp/cln515/cvl-tk
$ cd cvl-tk
$ mkdir build & cd build
```
CMake & Install

### CMake & Install operation
Assuming you use CMake GUI
1. set path of source and build directory
e.g. "where is the source code" <path/to/cvl-tk>
"where to build the binaries" <path/to/cvl-tk>/build

2. CMake configure and generate
Push "Configure" button and set VS version you use. Set platform: x64
Then push "Generate"

3. Run Visual Studio as Administrator and open the project in the build directory.

4. Check the target configulation (Debug or Release) and build "INSTALL"
