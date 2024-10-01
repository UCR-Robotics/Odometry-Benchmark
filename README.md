# Odometry-Benchmark
This repository hosts the benchmark scripts used in LiDAR or LiDAR-inertial odometry research.
It is also possible to generalize this framework for the evaluation of other types of odometry algorithms.

## Usage
### Auto runner
1) Make sure `algorithms.yaml` and `datasets.yaml` in this folder are properly configured,
2) run `roscore` in a separate terminal (unless you don't use ROS),
3) then just run
```
python3 autorun.py
```
Results will be saved to a folder named `results_xxxx`, which is named after date and time (Month-Date-Hour-Minute).

### Evaluation
Run the evaluation script with the previous result folder as an argument:
```
python3 evaluation.py results_xxxx/
```

More information to be updated.

## TODO
- [x] record timing and cpu usage
- [ ] add relative pose error (RPE) metric
- [ ] add KITTI odometry metric
- [ ] add visualization support to nav_msgs/Path
