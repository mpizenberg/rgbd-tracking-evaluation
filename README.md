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
  TODO: strip ROS requirements of DVO.

[ocv-odometry]: https://docs.opencv.org/4.0.1/df/ddc/classcv_1_1rgbd_1_1Odometry.html
[ocv-rgbd]: https://docs.opencv.org/4.0.1/d0/d60/classcv_1_1rgbd_1_1RgbdOdometry.html
[ocv-icp]: https://docs.opencv.org/4.0.1/d7/d83/classcv_1_1rgbd_1_1ICPOdometry.html
[ocv-rgbd-icp]: https://docs.opencv.org/4.0.1/d2/d0f/classcv_1_1rgbd_1_1RgbdICPOdometry.html
[fovis]: https://github.com/fovis/fovis
[dvo]: https://github.com/tum-vision/dvo

## Requirements

Each algorithm have a different set of requirements.
These have been tested under the Arch linux distribution,
kernel 4.19.17-1-lts or more recent.
To make everything compile, it required:

- [OpenCV][open-cv]. Contrib also required. Tested with version 4.0.1.
- [Ceres Solver][ceres-solver]. Tested with version 1.14.0.
- [Eigen3][eigen3]. Tested with version 3.3.7.
- [Boost][boost]. Tested with version 1.69.0.
- Indirect dependencies coming from those above.
- Maybe something else I had already installed.

[open-cv]: https://opencv.org/
[ceres-solver]: http://ceres-solver.org/
[eigen3]: http://eigen.tuxfamily.org/index.php?title=Main_Page
[boost]: https://www.boost.org/

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
Otherwise (fovis for example) they might be 10 times slower.
Now let's say you want to try the fovis algorithm, and managed to compile it already.
From the root of this repository, enter the following commands:

```sh
# Create a build/ directory for build artifacts
$> mkdir build
$> cd build

# Compile all C++ test executables
$> cmake ..
$> make

# Run the fovis tracking algorithm
$> ./fovis_track path/to/associations/file.txt
```

Where `path/to/associations/file.txt` is the path to the associations file
of the TUM-compatible dataset you want to test.
For more info about this associations file, cf `tooling/README.md`.
The tracking algorithm will print to stdout (console) the trajectory
in the TUM format (`timestamp tx ty tz qx qy qz qw`).
You can redirect the results to a file as you would do with any command:

```sh
$> ./fovis_track associations.txt > trajectory-fovis.txt
```

Now we want to evaluate the quality of the tracking.
Evaluation tooling is provided by the TUM RGB-D dataset.
These are python scripts, that are gathered here under the `tooling/`
directory for ease of use.
They are python 2 scripts and may require that you have some
python libraries installed on your environment.
Our recomendation would be to manage your environment using [Conda][conda],
in the Miniconda version, but do as you wish.

[conda]: https://conda.io/projects/conda/en/latest/

So for the evaluation of the trajectory obtained,
move into the `tooling/` directory and try the absolute trajectory error script:

```sh
# Switch to your python2 environment with scientific stuff available.
# Not necessary if you have everything installed globally on your system (you will regret that!).
$> conda activate my-python2-scientific-environment

# Run the evaluation script
$> python evaluate_ate.py trajectory-groundtruth.txt trajectory-fovis.txt
compared_pose_pairs 790 pairs
absolute_translational_error.rmse 0.046804 m
...
```

## License

This repository directly depends on code with different licenses:

- OpenCV is under BSD-3-Clause
- Eigen3 is under MPL-2.0
- Boost is under the Boost license
- DVO is under GPL-3.0 or later
- Fovis is under GPL-2.0 or later

As a consequence, the code in this repository is distributed under GPL-3.0 or later.
