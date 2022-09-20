# Image to CAD

A program that converts images to CAD readable files.

# Prerequisites

- Windows (Currently tested only on this os may work on other os like linux as well)
- cmake (version >= 3.17)
- Compiler which support cpp17 (Currently tested on msvc143 as provided by vs2022)

# Compiling and running

## clone the repo:

```bat
git clone --depth 1 --recurse-submodules --shallow-submodules https://github.com/NadavT/image_to_cad_cpp.git image_to_cad
cd image_to_cad
```

## build

Open a supported build shell (`x64 Native Tools Command Prompt` for example)  
(A shell which has `cmake` and supported compiler and generator in the path)

### using nmake - provided by MSVC by default:

```bat
mkdir install
mkdir build && cd build
cmake -DCMAKE_INSTALL_PREFIX="../install" -DCMAKE_BUILD_TYPE=Release -G "NMake Makefiles" ..
cmake --build . --target image_to_cad -j 6
cmake --install . --component image_to_cad
```

### using ninja - preferred if installed (support parallel compilation):

```bat
mkdir install
mkdir build && cd build
cmake -DCMAKE_INSTALL_PREFIX="../install" -DCMAKE_BUILD_TYPE=Release -G "Ninja" ..
cmake --build . --target image_to_cad -j 6
cmake --install . --component image_to_cad
```

## Run

Run `image_to_cad.exe` from `build\bin` dir.

### Arguments

| flags                             | description                                           |
| --------------------------------- | ----------------------------------------------------- |
| -h --help                         | shows help message and exits [default: false]         |
| -v --version                      | prints version information and exits [default: false] |
| **-i --input**                    | Input image [**required**]                            |
| -o --output_dir                   | Output directory [default: "results"]                 |
| -s --scale                        | Scale factor [default: 4]                             |
| -r --reduction_proximity          | Reduction proximity [default: 2]                      |
| -lt --hanging_leaf_threshold      | Hanging leaf threshold [default: 250]                 |
| -it --islands_threshold           | Islands threshold [default: 4]                        |
| -jt --junction_collapse_threshold | Junction collapse threshold [default: 14]             |

# Extras

## Resources

Find resources and images to use as input [here](https://drive.google.com/drive/folders/1ql_MQ4TBghVFClZZAGk84Ai-Pe-QEuif?usp=sharing).
