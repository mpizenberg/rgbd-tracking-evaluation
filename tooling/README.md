# Tooling for RGB-D Tracking Evaluation

The "tools" in this directory are python scripts [provided by the TUM RGB-D dataset][tum-tools].
Those are licensed under BSD-3-Clause.
In order to run them without issue, we suggest to use
a python virtual environment manager, such as [Conda][conda], virtualenv, pipenv, etc.

[tum-tools]: https://vision.in.tum.de/data/datasets/rgbd-dataset/tools
[conda]: https://conda.io/projects/conda/en/latest/

## Associating Color and Depth Images

In the TUM dataset format, color and depth images are stored separately,
and not necessarily synchronized.
Images are therefore listed under two files, one for color images,
we will name it `rgb.txt` and one for depth images, say `depth.txt`.
In each file, are listed both the image timestamps,
and the path to the corresponding image.

The `associate.py` scripts aims at finding matching timestamps between
the color and depth images, and associating them in a merged file,
where each line matches one color and one depth image in the following manner:

```
depth_timestamp depth_file_path rgb_timestamp rgb_file_path
```

This is the file that our test scripts want as argument.
So if not present by default in the "TUM-compatible" dataset you are using,
that is the first thing to create.

## Tracking Evaluation

And your heart starts trumpeting in your chest.
Worry not! Bad news, good news, these are merely a sign of progress!

Concretely, there are many ways of evaluating tracking algorithms.
Two of them are provided here:

1. Absolute Trajectory Error (ATE)
2. Relative Pose Error (RPE)

For both evaluations, you have to provide two trajectories,
one considered "ground truth", the "correct" one
(or at least as correct as can be achieved by the dataset creation process),
and one considered "estimated", the one that your algorithm provides.
Those trajectories are stored in files where each line is formatted as follows:

```
timestamp tx ty tz qx qy qz qw
```

More information about this trajectory file format in the `doc/dataset-format.md` file.
Know that an online version of the evaluation scripts are also available
in the [TUM web site][tum-eval].
It is restricted, however, to the TUM RGB-D dataset,
as you select the "ground truth" trajectory in a drop down menu.

[tum-eval]: https://vision.in.tum.de/data/datasets/rgbd-dataset/online_evaluation

### Absolute Trajectory Error (ATE)

The ATE measures the euclidean distance between the ground truth and estimated trajectories.
It proceeds in three steps:

1. Ground truth and estimated poses are associated from their timestamps.
2. Both trajectories are aligned using singular value decomposition (SVD).
3. Different measures are computed for each pair of poses (mean, median, standard deviation, ...).

ATE is well suited for SLAM algorithms, that are supposed to build
a coherent global map of the environment.
ATE however, is not the best candidate for visual odometry (VO) algorithms,
that often drift, due to the absence of loop closure, or pose graph optimisation.
You can use the script `evaluate_ate.py` to measure ATE.

### Relative Pose Error (RPE)

The RPE measures the relative translation and rotation error between two pairs of poses
(instead of between two poses) along the provided trajectories.
Concretely, if we have one pair of estimated poses `(Rt_e1, Rt_e2)`
and its corresponding pair of ground truth poses `(Rt_gt1, Rt_gt2)`,
we can compute each pair motion (`Rt_e12` and `Rt_gt12`)
and evaluate the relative translation and rotation error between those:

```
Rt_e12 = inv(Rt_e2) * Rt_e1
Rt_gt12 = inv(Rt_gt2) * Rt_gt1
Rt_relative_12 = inv(Rt_e12) * Rt_gt12
translation_error = translation(Rt_relative_12)
rotation_error = rotation(Rt_relative_12)
```

The only question left is "which pairs do we choose?".
Well the provided script `evaluate_rpe.py` has two modes:

1. Default mode: all pairs possible are evaluated.
   The complexity grows quadratically so be carefull.
   Moreover, it does not make sense to continue evaluating pairs
   where drift plays too big of a role.
2. `--fixed_delta` mode: you provide a fixed "delta",
   whether the delta is in seconds, meters, radians, or frames,
   and it will evaluate all pairs with this delta
   along the trajectory.
   Evaluate RPE at 1 second usually gives a good idea of the quality
   of the VO tracking algorithm.

In the end, the script computes the rmse of those errors.
It can provide some other info, use `--help` for more about it.
