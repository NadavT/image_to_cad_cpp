# Image to CAD

A program that converts images to CAD readable files.

Executables:

- Image to CAD - A program that converts a given black and white image to a CAD 3d representation of the image (using Irit format `.itd`).
- Image to CAD preprocess - A program that only run the preprocessing part of the Image to CAD program.

Produced files:

- `preprocessed.png` - The result of the preprocessing step.
- `surfaces.itd` - The generated surfaces (2d) in Irit format.
- `extrusions.itd` - The generated extruded surfaces (3d) in Irit format.
- Other files - Files used to debug the program (usually have a self explanatory name).

# Prerequisites

- Windows (Currently tested only on this os may work on other os like linux as well)
- cmake (version >= 3.17)
- Compiler which support cpp17 (Currently tested on msvc143 as provided by vs2022)
- [Irit](https://csaws.cs.technion.ac.il/~gershon/GuIrit/index.html) library (Compiled using a matching compiler to the one used to compile this project), set `IRIT_ROOT`/`IRIT64_ROOT` environment variable to the irit bin directory (where the `irit.dll`/`irit64.dll`/`irit.a`/`irit.so` is located, usually under `ntbin`/`ntbin64`/`bin` directory). Match the bitness (32/64) of the compiler used to compile this project

## Soft prerequisites

- Boost (version == 1.80.0), set `BOOST_ROOT` environment variable to the boost root directory to use a local boost installation.
- OpenCV (version == 4.6.0), set `OpenCV_DIR` environment variable to the opencv root directory to use a local opencv installation.

The soft prerequisites will be downloaded and compiled automatically if not found.

# Compiling the program

## clone the repo:

```sh
git clone --depth 1 --recurse-submodules --shallow-submodules https://github.com/NadavT/image_to_cad_cpp.git image_to_cad
cd image_to_cad
```

We use `--depth 1` to only clone the latest commit and `--recurse-submodules` to clone all submodules.

## build

**Make sure to have the prerequisites installed and set the environment variables if needed.**

### Windows

Open a supported build shell (`x64 Native Tools Command Prompt`/`Developer Command Prompt for VS...`/`Developer Powershell for VS...` or any other shell which has `cmake` and supported compiler and generator in the path) and run:  
(A shell which has `cmake` and supported compiler and generator in the path)

#### using nmake - provided by MSVC by default:

```bat
mkdir install
mkdir build && cd build
cmake -DCMAKE_INSTALL_PREFIX="../install" -DCMAKE_BUILD_TYPE=Release -G "NMake Makefiles" ..
cmake --build . --target image_to_cad image_to_cad_preprocess -j 6
cmake --install . --component image_to_cad
cmake --install . --component image_to_cad_preprocess
```

#### using ninja - preferred if installed (support parallel compilation):

```bat
mkdir install
mkdir build && cd build
cmake -DCMAKE_INSTALL_PREFIX="../install" -DCMAKE_BUILD_TYPE=Release -G "Ninja" ..
cmake --build . --target image_to_cad image_to_cad_preprocess -j 6
cmake --install . --component image_to_cad
cmake --install . --component image_to_cad_preprocess
```

### Linux

Open any shell with `cmake`, supported compiler and generator in the path, and run:

```sh
mkdir install
mkdir build && cd build
cmake -DCMAKE_INSTALL_PREFIX="../install" -DCMAKE_BUILD_TYPE=Release ..
cmake --build . --target image_to_cad image_to_cad_preprocess -j 6
cmake --install . --component image_to_cad
cmake --install . --component image_to_cad_preprocess
```

# Running the program

Run `image_to_cad.exe` from `install` dir.

## Arguments

Run `image_to_cad.exe --help` to see the latest arguments and their description.

Try default arguments first and tweak them if needed, usually `--border` is needed to inputs with no black border at the edges of the image.  
If using full executable (not preprocess only) with a preprocessed image, use `--no_preprocessing` flag, make sure to match the `--border` flag when using `--no_preprocessing` with the preprocessing run which generated the preprocessed image.

### General arguments

| flags           | description                                           |
| --------------- | ----------------------------------------------------- |
| -h --help       | shows help message and exits [default: false]         |
| -v --version    | prints version information and exits [default: false] |
| **-i --input**  | Input image [**required**]                            |
| -o --output_dir | Output directory [default: "results"]                 |

### Preprocessing arguments

| flags                                   | description                                                    |
| --------------------------------------- | -------------------------------------------------------------- |
| -no_pp --no_preprocessing               | Should not apply preprocessing to input image [default: false] |
| -no_cbw --no_convert_to_black_and_white | Should not convert to black and white [default: false]         |
| -no_cft --no_crop_to_fit                | Should not crop to fit [default: false]                        |
| -ctf_pl --crop_to_fit_padding_left      | Crop to fit padding left [default: 0]                          |
| -ctf_pr --crop_to_fit_padding_right     | Crop to fit padding right [default: 0]                         |
| -ctf_pt --crop_to_fit_padding_top       | Crop to fit padding top [default: 0]                           |
| -ctf_pb --crop_to_fit_padding_bottom    | Crop to fit padding bottom [default: 0]                        |
| -it --islands_threshold                 | Islands threshold [default: 25]                                |
| -s --scale                              | Scale factor [default: 4]                                      |
| -b --border                             | Should add border [default: false]                             |

### Graph processing arguments

| flags                              | description                               |
| ---------------------------------- | ----------------------------------------- |
| -r --reduction_proximity           | Reduction proximity [default: 10]         |
| -lt --hanging_leaf_threshold       | Hanging leaf threshold [default: 250]     |
| -jct --junction_collapse_threshold | Junction collapse threshold [default: 20] |
| -jst --junction_smooth_threshold   | Junction smooth threshold [default: 10]   |

### Surface generation arguments

| flags                        | description                                                                                                |
| ---------------------------- | ---------------------------------------------------------------------------------------------------------- |
| -co --curve_order            | Assign the maximal order of the curve (B-Spline), use -1 to unlimited (Bezier curve) [default: 100]        |
| -tco --target_curve_order    | Assign the target order of the curve (B-Spline), use -1 to unlimited (Bezier curve) [default: 4]           |
| -cd --curve_density          | density of the curves (number of points per unit length of arc) [default: 0.1]                             |
| -cml --curve_min_length      | minimum control points of the curves [default: 3]                                                          |
| -jra --junction_radius_adder | The radius of the junctions will be increased by this value (for treaming the outgoing curves [default: 6] |
| -ex --extrusion              | Extrusion amount [default: 25]                                                                             |

Filter offset curves arguments (usually not needed)

| flags                                 | description                                  |
| ------------------------------------- | -------------------------------------------- |
| -foc --filter_offset_curves           | Should filter offset curves [default: false] |
| -dbs --distance_to_boundary_samples   | distance to boundary samples [default: 5]    |
| -dbt --distance_to_boundary_threshold | distance to boundary threshold [default: 2]  |
| -dbb --distance_in_boundary_backoff   | distance in boundary backoff [default: 0.1]  |
| -dbf --distance_in_boundary_factor    | distance in boundary factor [default: 10]    |

# Extras

## Resources

Find resources and images to use as input [here](https://drive.google.com/drive/folders/1ql_MQ4TBghVFClZZAGk84Ai-Pe-QEuif?usp=sharing).
