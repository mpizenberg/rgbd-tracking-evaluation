# Dataset Format

We focus on datasets compatible with the [TUM RGB-D SLAM dataset format][tum-format].

[tum-format]: https://vision.in.tum.de/data/datasets/rgbd-dataset/file_formats

## Associations File

Though not always present in the dataset original download,
our test programs (`fovis_track.cpp`, ...) require an associations file as argument.
This file contains lines matching depth and color images as follows:

```
depth_timestamp depth_file_path rgb_timestamp rgb_file_path
```

Notice that for our programs, depth info comes first in the associations file.
If not present, it is possible to create this file from two files,
`depth.txt` and `rgb.txt`.
Each of these contains a list of timestamp and corresponding path to image file.
The `associate.py` tool script present in the `tooling/` directory
can generate this file from the other two:

```
# beware that depth is first
python associate.py depth.txt rgb.txt
```

## Color Images

Color images are stored as 640x480 8-bit RGB images in PNG format.
They usually are in an `rgb/` directory, but their location doesn't
really matter as long as the relative path to them is available in the associations file.

## Depth Images

Depth maps are stored as 640x480 16-bit monochrome images in PNG format.
They usually are in a `depth/` directory, but, as with color images, their location doesn't
really matter as long as the relative path to them is available in the associations file.

Depth images are scaled by a factor of 5000, i.e., a pixel value of 5000
corresponds to a distance of 1 meter.
Theoretically, we thus have a max precision of 0.2 mm for the range `0 -> 13.1072 m`.
(`2^16 == 65536 == 5000 * 13.1072`).

> Pixel value of exactly 0 means that the data is missing!

## Trajectory

Ground truth trajectory must be provided in a file where each line follows the format:

```
# Comment line starting with a "#"
timestamp tx ty tz qx qy qz qw
```

The timestamp (float) is the [POSIX time][posix-time] of the frame, in seconds.

The coefficients `tx`, `ty` and `tz` (floats) are the coordinates of the optical center
of the color camera with respect to the world origin as defined by the motion capture system.
Beware that those are not directly the translation parameters of
the extrinsics camera projection matrix (but of its inverse).

The coefficients `qx`, `qy`, `qz` and `qw` are the parameters of the quaternion
discribing the orientation of the optical center of the color camera.
Same as previously, beware that those are rotation parameters of the inverse projection.
The last coefficient, `qw` is the real part of the quaternion.

[posix-time]: https://en.wikipedia.org/wiki/Unix_time

## Color Camera Calibration

Calibration images are usually provided by the dataset.
Since we are only using few different datasets for the time being,
we will put their values directly here:

| Model      | fx    | fy     | cx    | cy    | d0   | d1    | d2      | d3      | d4   |
| ---------- | ----- | ------ | ----- | ----- | ---- | ----- | ------- | ------- | ---- |
| ICL-NUIM   | 481.2 | -480.0 | 319.5 | 239.5 | 0    | 0     | 0       | 0       | 0    |
| Freiburg 1 | 517.3 | 516.5  | 318.6 | 255.3 | 0.26 | -0.95 | -0.0054 | 0.0026  | 1.16 |
| Freiburg 2 | 520.9 | 521.0  | 325.1 | 249.7 | 0.23 | -0.78 | -0.0033 | -0.0001 | 0.92 |
| Freiburg 3 | 535.4 | 539.2  | 320.1 | 247.6 | 0    | 0     | 0       | 0       | 0    |

Note that Freiburg 3 sequences are already undistorted,
thus why its distortion parameters are all zero here.

In practice, those coefficients are already hard-coded in the test programs.
These ask for the camera model id in arguments (`[fr1|fr2|fr3|icl]`).

If you wish to use your own camera parameters for some dataset,
you will typically have to add one possibility in the test programs.
This is to avoid having to parse camera files,
not standardized in the TUM RGB-D dataset format.

## Other Calibration Files

Note that other calibration files are provided by the TUM RGB-D dataset
but we will not need them.
