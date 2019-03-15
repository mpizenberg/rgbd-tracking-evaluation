# RGB-D Camera Tracking Evaluation

This repository aims at comparing open-source implementations of RGB-D tracking algorithms.
Our goal is to make development of new algorithms easier to compare
with other state of the art implementations.

## Datasets

We use datasets following the [TUM RGB-D SLAM dataset][tum-dataset] format.
For now, this means the TUM dataset itself, containing real sequences,
and the [ICL-NUIM dataset][icl-nuim], containing synthetic images.
For more info about the dataset format, read the `dataset-format.md` file.

[tum-dataset]: https://vision.in.tum.de/data/datasets/rgbd-dataset
[icl-nuim]: https://www.doc.ic.ac.uk/~ahanda/VaFRIC/iclnuim.html

## Algorithms

The open-source algorithms available for testing here are:

- The [OpenCV contrib RGB-D odometry][ocv-odometry]. It provides three algorithms:
  - one direct approach ([`Rgbd`][ocv-rgbd]) based on the paper
    "Real-Time Visual Odometry from Dense RGB-D Images",
    F. Steinbucker, J. Strum, D. Cremers, ICCV, 2011.
  - one point cload approach ([`ICP`][ocv-icp]) based on the paper
    "KinectFusion: Real-Time Dense Surface Mapping and Tracking",
    Richard A. Newcombe, Andrew Fitzgibbon, at al, SIGGRAPH, 2011.
  - one mixed approach ([`RgbdICP`][ocv-rgbd-icp]) minimizing
    the sum of both energy functions.
- The [fovis][fovis] visual odometry library, based on the paper
  "Visual Odometry and Mapping for Autonomous Flight Using an RGB-D Camera",
  Albert S. Huang, Abraham Bachrach, Peter Henry, Michael Krainin,
  Daniel Maturana, Dieter Fox, and Nicholas Roy.
  Int. Symposium on Robotics Research (ISRR), Flagstaff, Arizona, USA, Aug. 2011
- Dense Visual Odometry ([DVO][dvo]), based on the paper
  "Robust Odometry Estimation for RGB-D Cameras", C. Kerl, J. Sturm and D. Cremers,
  In International Conference on Robotics and Automation (ICRA), 2013.
- Visual Odometry in Rust ([vors][vors]), currently being worked on.
- TODO? Daniel G. G., RAS, 2015 : http://webdiis.unizar.es/~danielgg/code.html
- TODO? Alejo Concha, 2017 RGBDTAM : https://github.com/alejocb/rgbdtam

[ocv-odometry]: https://docs.opencv.org/4.0.1/df/ddc/classcv_1_1rgbd_1_1Odometry.html
[ocv-rgbd]: https://docs.opencv.org/4.0.1/d0/d60/classcv_1_1rgbd_1_1RgbdOdometry.html
[ocv-icp]: https://docs.opencv.org/4.0.1/d7/d83/classcv_1_1rgbd_1_1ICPOdometry.html
[ocv-rgbd-icp]: https://docs.opencv.org/4.0.1/d2/d0f/classcv_1_1rgbd_1_1RgbdICPOdometry.html
[fovis]: https://github.com/fovis/fovis
[dvo]: https://github.com/mpizenberg/dvo
[vors]: https://github.com/mpizenberg/visual-odometry-rs

## Requirements

Each algorithm has a different set of requirements.
These have been tested under the Arch linux distribution,
kernel 4.19.17-1-lts or more recent.
To make everything compile, it required:

- [OpenCV][open-cv]. Contrib also required. Tested with version 4.0.1.
- [Ceres Solver][ceres-solver]. Tested with version 1.14.0.
- [Eigen3][eigen3]. Tested with version 3.3.7.
- [Boost][boost]. Tested with version 1.69.0.
- [Rust][rust]. Tested with version 1.33.0.
- Indirect dependencies coming from those above.
- Maybe something else I had already installed.

If you are running on Ubuntu, you can find exact dependencies required
from a fresh Ubuntu install by looking at the docker files in the `dockerfiles/` directory.
And if you are not on Ubuntu, chances are you can manage ;)

[open-cv]: https://opencv.org/
[ceres-solver]: http://ceres-solver.org/
[eigen3]: http://eigen.tuxfamily.org/index.php?title=Main_Page
[boost]: https://www.boost.org/
[rust]: https://www.rust-lang.org/

## Getting Started

There are basically two ways of cloning this repository:

1. With all its submodules (cf [stackoverflow][so-clone] depending on your git version).
   This will pull the other algorithms in the `third-party/` directory.
   Useful if you don't have them already installed somewhere on your computer.
2. Without submodules: `git clone (this-repo)`.
   This is useful if you wish to keep the algorithms
   independently installed somewhere else on your system by yourself.

[so-clone]: https://stackoverflow.com/questions/3796927/how-to-git-clone-including-submodules

Then, compile the algorithms you wish to test.
For this, look at the readme of each of those.
Make sure that you compile those in Release mode (`cmake -DCMAKE_BUILD_TYPE=Release ..`).
Otherwise they might be more than 10 times slower.

Let's say, for example, that we want to test the DVO algorithm,
and that we cloned the repository with its submodules (DVO is in third-party/dvo/).
We will proceed as follows.

```sh
# Download a compatible dataset, such as one at:
# https://vision.in.tum.de/data/datasets/rgbd-dataset/download
# I let you handle that, but starting with "fr1/xyz" is a good idea.

# Generate the associations file
# More info about it in tooling/README.md
$> conda activate my-python2-scientific-virtual-env
$> cd tooling/
$> DATASET=absolute/path/to/fr1/xyz/dataset
$> python associate.py $DATASET/depth.txt $DATASET/rgb.txt > $DATASET/associations.txt

# Move to DVO directory
$> cd ../third-party/dvo

# Compile the dvo_core shared library, as explained in DVO's readme
# You might need to install DVO's dependencies
$> mkdir build
$> cd build
$> cmake ..
$> make

# Go back to this root repository
$> cd ../../..

# Optionally, comment groups of commands in the CMakeLists.txt
# that are related to other algorithms than DVO,
# to minimize the potential dependencies/linking issues.

# Compile test executables
$> mkdir build
$> cd build
$> cmake ..
$> make

# Run the tracking algorithm on the dataset (e.g. fr1/xyz)
$> bin/dvo_track fr1 $DATASET/associations.txt > $DATASET/trajectory-dvo.txt

# Run evaluations scripts (ATE with plot, and RPE without)
$> cd ../tooling
$> python evaluate_ate.py --verbose --plot $DATASET/traj-dvo.png $DATASET/groundtruth.txt $DATASET/trajectory-dvo.txt
$> python evaluate_rpe.py --verbose --fixed_delta --delta_unit s --delta 1 $DATASET/groundtruth.txt $DATASET/trajectory-dvo.txt
```

Evaluation tooling is provided by the TUM RGB-D dataset.
These are python scripts, that are gathered here under the `tooling/`
directory for ease of use.
They are python 2 scripts and may require that you have some
python libraries installed on your environment.
Our recomendation would be to manage your environment using [Conda][conda],
in the Miniconda version, but do as you wish.

[conda]: https://conda.io/projects/conda/en/latest/

## License

This repository directly depends on code with different licenses:

- OpenCV is under BSD-3-Clause
- Eigen3 is under MPL-2.0
- Boost is under the Boost license
- DVO is under GPL-3.0 or later
- Fovis is under GPL-2.0 or later
- The rust dependencies are under a mix of permissive licenses.

As a consequence, the code in this repository is distributed under GPL-3.0 or later.

## FAQ

### When compiling, I got "fovis/fovis.hpp not found"?

There might be two reasons for this to appear.

1. You might not have compiled fovis first.
   Indeed compiling the test programs in `src/` requires that you already
   compiled the corresponding algorithms to produce their shared libraries.
2. You have compiled fovis not as a submodule,
   and forgot to change the `include_directories(...)`
   and `link_idrectories(...)` commands in the `CMakeLists.txt`.

### When running the test programs, no file is created?

This is the normal behavior.
Those programs, and the python scripts in tooling,
all print their results in stdout (the console).
To retrieve them in a file, do as you would for any other command,
by redirecting the results in a file of your choosing:

```sh
some_track_program arguments > some_file.txt
```

### When running the tool scripts I get errors like "print ... SyntaxError: invalid syntax"?

Or "dict has no attribute remove", etc.
Those scripts provided by the TUM RGB-D dataset are written in python 2.
So if your main python version is python 3, they will crash.
If you don't have a python module required they will also crash.

I suggest that you use a virtual environment manager,
such as [Conda][conda], to isolate a python 2 environment with the exact
dependencies you need for those scripts to run.
