Each of the following directorys need to be installed in the following way:

ceres-solver
============
Install the dependencies

    # CMake
    sudo apt-get install cmake
    # google-glog + gflags
    sudo apt-get install libgoogle-glog-dev libgflags-dev
    # BLAS & LAPACK
    sudo apt-get install libatlas-base-dev
    # Eigen3
    sudo apt-get install libeigen3-dev
    # SuiteSparse and CXSparse (optional)
    sudo apt-get install libsuitesparse-dev

Build the code and install

    cd ceres-solver
    mkdir build
    cd build
    cmake ..
    make -j3
    make test
    # Optionally install Ceres, it can also be exported using CMake which
    # allows Ceres to be used without requiring installation, see the documentation
    # for the EXPORT_BUILD_DIR option for more information.
    sudo make install

reference: http://ceres-solver.org/installation.html

rransac
=======
Dependencies

    Eigen3 (installed above)
    Boost (comes with Xavier Ubuntu)
    Ceres Solver (above)
    OpenCV4 (comes with Xavier Ubuntu)
    LieGroups (Submodule)(may need to change liegroups/cmake/googletest.cmake.in master->main)
    (may need to change cmake/googletest.cmake.in master->main)

Building the code

    cd rransac
    mkdir build && cd build
    cmake ..   # This is where you would put -D<options>
    make
    sudo make install

Some common -D options:

    -DBUILD_PY=ON or -DBUILD_PY=OFF (default: ON)
    -DPYVER=2.7 or -DPYVER=3 (default: 3.6)

parallax_cpp
============
Edit the CMAKELists.txt

    Remove -mavx from
    set(CMAKE_CXX_FLAGS "-std=c++14 -mavx -openmp") #-O2 -mfma


Build and install the code

    mkdir build
    cd build
    cmake ..
    make
    sudo make install

If multiple OpenCVs are present

    mkdir build
    cd build
    cmake -DOpenCV_DIR=/usr/local/share/OpenCV ..
    make
    sudo make install


