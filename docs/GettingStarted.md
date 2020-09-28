

# Add uorb to your project

There are two ways use uorb in your project:

* [**Use the uorb pre-built library binaries and headers**](#use-the-uorb-pre-built-library). Use this approach if you just want to use a stable version of the uorb library in your project.

* [**Build uorb from source**](#build-uorb-from-source). Use this approach if you would like to debug or want the latest modifications.

## Use the uorb pre-built library

It is recommended that you build uorb outside of the source directory, to avoid having issues with CMake generated files polluting the source directory:

```shell
mkdir uorb-build
cd uorb-build
cmake -DCMAKE_C_COMPILER=clang     \
        -DCMAKE_CXX_COMPILER=clang++ \
        -DCMAKE_INSTALL_PREFIX=./install \
        $HOME/uorb                         # we're assuming this is where uorb is.
make
make install
```

The `./install` directory contains a lib directory and an include directory, link the libraries in the lib directory, and then add the include folder to the include path of your project.

Or refer to the x.toolchain.cmake file in the toolchain directory to write a cmake toolchain file for your corresponding platform.

## Build uorb from source

The best way to build from source code is as a subdirectory of the cmake project:

Add the source code to your project.

```shell
git submodule add https://github.com/ShawnFeng0/uorb.git # If you don't use git, just download the source code and unzip it.
```

Link the uorb library in the CMakeLists.txt of your own project.

```cmake
# uorb library
add_subdirectory(uorb)
link_libraries(uorb)
include_directories(uorb/include)
```

This is an example of using this method (**recommended**): [uorb-examples](https://github.com/ShawnFeng0/uorb-examples.git).

# Create uorb topic to your project

In order to manage and modify topics in a unified way, we use a message generation tool, which uses a way similar to ROS to define topics.

TODO

# Using uorb

TODO

