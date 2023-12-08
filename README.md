# Odometry-Benchmark
This repository hosts the benchmark scripts used in LiDAR or LiDAR-inertial odometry research.
It is also possible to generalize this framework for the evaluation of other types of odometry algorithms.

## Usage
### Auto runner
Make sure `algorithms.yaml` and `datasets.yaml` in this folder are properly configured, and then just run
```
python3 autorun.py
```
Results will be saved into a folder named `results_xxxx`, which is named after date and time.
### Evaluation
Run the evaluation script with the previous result folder as an argument:
```
python3 evaluation.py results_xxxx/
```

More information to be updated.
