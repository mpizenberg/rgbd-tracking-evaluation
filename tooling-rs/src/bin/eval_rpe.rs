use nalgebra::UnitQuaternion;
use std::collections::HashMap;
use std::fmt::{self, Debug};
use std::{env, error::Error, fs, io::BufReader, io::Read, path::Path, process::exit};
use visual_odometry_rs::dataset::tum_rgbd::{self, Frame};

fn main() {
    let args: Vec<String> = env::args().collect();
    if let Err(error) = run(args) {
        eprintln!("{:?}", error);
        eprintln!("{}", USAGE);
        exit(1);
    }
}

const USAGE: &str = "Usage: cargo run --release --bin eval_rpe traj_gt traj_est";

fn run(args: Vec<String>) -> Result<(), Box<Error>> {
    // Retrieve all trajectories passed as arguments.
    let trajectories = args.iter().skip(1).map(|f| (f, parse_trajectory(f)));

    // Put each trajectory either in the ground truth or estimation hash maps.
    let mut ground_truth_trajectories = HashMap::with_capacity(args.len());
    let mut estimated_trajectories = HashMap::with_capacity(args.len());
    for (file_path, potential_trajectory) in trajectories {
        let traj = potential_trajectory?;
        let traj_metadata = metadata(file_path);
        match &traj_metadata.algorithm {
            None => ground_truth_trajectories.insert(file_path, (traj_metadata, traj)),
            Some(_) => estimated_trajectories.insert(file_path, (traj_metadata, traj)),
        };
    }

    // For each ground truth, evaluate associated estimated trajectories.
    for (meta_gt, traj_gt) in ground_truth_trajectories.values() {
        let estimated_same_seq = estimated_trajectories
            .values()
            .filter(|(meta, _)| meta.sequence == meta_gt.sequence);
        for (meta, traj) in estimated_same_seq {
            let traj_rpe = relative_pose_error(traj_gt, traj);
        }
    }

    // // Build a vector of frames, containing timestamps and camera poses.
    // let trajectory_gt = parse_trajectory(trajectory_gt_file)?;
    // let trajectory_est = parse_trajectory(trajectory_est_file)?;
    // // eprintln!("Number of ground truth frames: {}", trajectory_gt.len());
    // // eprintln!("Number of estimated frames:    {}", trajectory_est.len());
    //
    // // Compute relative pose error at 1 frame and 1 second.
    // let rpe = relative_pose_error(&trajectory_gt, &trajectory_est);
    // // eprintln!("{:?}", rpe);
    //
    // let traj_metadata = metadata(trajectory_gt_file);

    Ok(())
}

#[derive(Debug)]
struct Metadata {
    sequence: String,
    camera: String,
    algorithm: Option<String>,
}

fn metadata<P: AsRef<Path>>(path: P) -> Metadata {
    let algorithms = vec![
        "dvo",
        "fovis",
        "vors",
        "ocv-rgbd",
        "ocv-icp",
        "ocv-rgbd-icp",
    ];
    let stem = &path.as_ref().file_stem().unwrap().to_string_lossy();
    let algo = algorithms.iter().find(|&a| stem.ends_with(a));
    let sequence = match algo {
        None => stem,
        Some(algo_name) => &stem[0..(stem.len() - algo_name.len() - 1)],
    };
    let camera = if sequence.contains("freiburg1") {
        "fr1"
    } else if sequence.contains("freiburg2") {
        "fr2"
    } else if sequence.contains("freiburg3") {
        "fr3"
    } else if sequence.contains("frei_png") {
        "icl"
    } else {
        "unknown"
    };
    Metadata {
        sequence: sequence.to_string(),
        camera: camera.to_string(),
        algorithm: algo.map(|a| a.to_string()),
    }
}

fn relative_pose_error(trajectory_gt: &Vec<Frame>, trajectory_est: &Vec<Frame>) -> Rpe {
    // Define a matching time distance threshold between estimated and ground truth frames.
    let threshold_gt = timestamps_threshold(trajectory_gt);

    // Match ground truth frames with estimated ones.
    let matches = match_gt_est(trajectory_gt, trajectory_est, threshold_gt);

    // Compute relative pose error at one frame distance.
    let (translation_errors, rotation_errors) = rpe_1_frame(&matches);
    let (trans_error_rmse, trans_error_median) = statistics(&translation_errors);
    let (rot_error_rmse, rot_error_median) = statistics(&rotation_errors);

    // Define a matching time distance threshold for the later frame (1s later).
    let threshold_est = timestamps_threshold(trajectory_est);

    // Compute relative pose error at 1 second later.
    let (translation_errors_1s, rotation_errors_1s) = rpe_1_second(&matches, threshold_est);
    let (trans_error_rmse_1s, trans_error_median_1s) = statistics(&translation_errors_1s);
    let (rot_error_rmse_1s, rot_error_median_1s) = statistics(&rotation_errors_1s);

    Rpe {
        // rpe at 1 frame.
        translation_error_rmse_1f: trans_error_rmse,
        translation_error_median_1f: trans_error_median,
        rotation_error_rmse_1f: rot_error_rmse,
        rotation_error_median_1f: rot_error_median,
        // rpe at 1 second.
        translation_error_rmse_1s: trans_error_rmse_1s,
        translation_error_median_1s: trans_error_median_1s,
        rotation_error_rmse_1s: rot_error_rmse_1s,
        rotation_error_median_1s: rot_error_median_1s,
    }
}

struct Rpe {
    // rpe at 1 frame.
    translation_error_rmse_1f: f32,
    translation_error_median_1f: f32,
    rotation_error_rmse_1f: f32,
    rotation_error_median_1f: f32,
    // rpe at 1 second.
    translation_error_rmse_1s: f32,
    translation_error_median_1s: f32,
    rotation_error_rmse_1s: f32,
    rotation_error_median_1s: f32,
}

impl Debug for Rpe {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "Relative pose error:\n")?;
        write!(
            f,
            "  Translation error (1 frame) - rmse:   {} m\n",
            self.translation_error_rmse_1f
        )?;
        write!(
            f,
            "  Translation error (1 frame) - median: {} m\n",
            self.translation_error_median_1f
        )?;
        write!(
            f,
            "  Rotation error    (1 frame) - rmse: {} deg\n",
            degrees(self.rotation_error_rmse_1f)
        )?;
        write!(
            f,
            "  Rotation error    (1 frame) - median: {} deg\n",
            degrees(self.rotation_error_median_1f)
        )?;
        write!(
            f,
            "  Translation error (1 second) - rmse:   {} m\n",
            self.translation_error_rmse_1s
        )?;
        write!(
            f,
            "  Translation error (1 second) - median: {} m\n",
            self.translation_error_median_1s
        )?;
        write!(
            f,
            "  Rotation error    (1 second) - rmse: {} deg\n",
            degrees(self.rotation_error_rmse_1s)
        )?;
        write!(
            f,
            "  Rotation error    (1 second) - median: {} deg\n",
            degrees(self.rotation_error_median_1s)
        )
    }
}

fn rpe_1_second(matches: &Vec<(&Frame, &Frame)>, threshold: f64) -> (Vec<f32>, Vec<f32>) {
    let mut translation_errors = Vec::with_capacity(matches.len());
    let mut rotation_errors = Vec::with_capacity(matches.len());
    let mut start_search = 0;
    for (frame_gt, frame_est) in matches.iter() {
        let target = frame_gt.timestamp + 1.0; // 1 second later.
        match find_nearest_right(target, matches, |m| m.1.timestamp, start_search) {
            None => break,
            Some(((later_frame_gt, later_frame_est), index)) => {
                start_search = index;
                if (target - later_frame_est.timestamp).abs() < threshold {
                    let motion_gt = later_frame_gt.pose.inverse() * frame_gt.pose;
                    let motion_est = later_frame_est.pose.inverse() * frame_est.pose;
                    let motion_error = motion_gt.inverse() * motion_est;
                    translation_errors.push(motion_error.translation.vector.norm());
                    rotation_errors.push(angle(motion_error.rotation));
                }
            }
        }
    }
    (translation_errors, rotation_errors)
}

fn degrees(radians: f32) -> f32 {
    180.0 / std::f32::consts::PI * radians
}

/// Return RMSE and median of a vector of values.
fn statistics(data: &Vec<f32>) -> (f32, f32) {
    let sum: f32 = data.iter().map(|x| x * x).sum();
    let rmse = (sum / data.len() as f32).sqrt();
    (rmse, fast_median(data.clone()))
}

fn rpe_1_frame(matches: &Vec<(&Frame, &Frame)>) -> (Vec<f32>, Vec<f32>) {
    let mut translation_errors = Vec::with_capacity(matches.len() - 1);
    let mut rotation_errors = Vec::with_capacity(matches.len() - 1);
    for (new, old) in matches.iter().skip(1).zip(matches.iter()) {
        let motion_gt = new.0.pose.inverse() * old.0.pose;
        let motion_est = new.1.pose.inverse() * old.1.pose;
        let motion_error = motion_gt.inverse() * motion_est;
        translation_errors.push(motion_error.translation.vector.norm());
        rotation_errors.push(angle(motion_error.rotation));
    }
    (translation_errors, rotation_errors)
}

fn angle(uq: UnitQuaternion<f32>) -> f32 {
    let w = uq.into_inner().scalar();
    2.0 * uq.into_inner().vector().norm().atan2(w)
}

fn match_gt_est<'a>(
    gt: &'a Vec<Frame>,
    est: &'a Vec<Frame>,
    threshold: f64,
) -> Vec<(&'a Frame, &'a Frame)> {
    if gt.is_empty() || est.is_empty() {
        return Vec::new();
    }
    let mut matches = Vec::with_capacity(est.len());
    let mut start_gt = 0;
    let (_, start_est) = find_nearest_right(gt[0].timestamp, est, |f| f.timestamp, 0).unwrap();
    for frame in est.iter().skip(start_est) {
        match find_nearest_right(frame.timestamp, gt, |f| f.timestamp, start_gt) {
            None => break,
            Some((gt_match, index)) => {
                start_gt = index;
                if (gt_match.timestamp - frame.timestamp).abs() < threshold {
                    matches.push((gt_match, frame));
                }
            }
        }
    }
    matches
}

fn find_nearest_right<T, F: Fn(&T) -> f64>(
    target: f64,
    vec: &Vec<T>,
    f: F,
    start: usize,
) -> Option<(&T, usize)> {
    if start >= vec.len() {
        return None;
    }
    let mut index = start;
    while index < vec.len() && f(&vec[index]) < target {
        index = index + 1;
    }
    if index == vec.len() {
        Some((&vec[index - 1], index - 1))
    } else if index == 0 {
        Some((&vec[0], 0))
    } else {
        let left = f(&vec[index - 1]);
        let right = f(&vec[index]);
        let index_found = if (target - left) < (right - target) {
            index - 1
        } else {
            index
        };
        Some((&vec[index_found], index_found))
    }
}

fn timestamps_threshold(trajectory: &Vec<Frame>) -> f64 {
    let timestamps_iter = trajectory.iter().map(|frame| frame.timestamp);
    let timestamps_iter_later = timestamps_iter.clone().skip(1);
    let delta_timestamps: Vec<_> = timestamps_iter_later
        .zip(timestamps_iter)
        .map(|(t1, t2)| (t1 - t2).abs())
        .collect();
    2.0 * fast_median(delta_timestamps)
}

fn fast_median<T: PartialOrd + Copy>(mut floats: Vec<T>) -> T {
    floats.sort_unstable_by(|a, b| a.partial_cmp(b).unwrap());
    floats[floats.len() / 2]
}

/// Open a trajectory file and parse it into a vector of Frames.
fn parse_trajectory<P: AsRef<Path>>(file_path: P) -> Result<Vec<Frame>, Box<Error>> {
    let file = fs::File::open(file_path)?;
    let mut file_reader = BufReader::new(file);
    let mut content = String::new();
    file_reader.read_to_string(&mut content)?;
    tum_rgbd::parse::trajectory(content).map_err(|s| s.into())
}
