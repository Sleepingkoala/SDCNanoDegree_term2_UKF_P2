# Unscented Kalman Filter Project 
---
This is my Project 2 in term 2 of the Ucatity Self-Driving Car Engineer Nanodegree Program, which aims to utilize an uscented kalman filter to estimate the state of a moving vehicle based on a CTRV model, with fused noisy lidar and radar measurements using C++. In this project I chose Ubuntu 14.04 LTS for implementation. 

## Content of my project
---
This project origins from the [Udacity Starter Code repository](https://github.com/udacity/CarND-Unscented-Kalman-Filter-Project),which includes [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) installation files __install-ubuntu.sh__ and several useful files as follows:

- **src folder**: with the project code.
  - **main.cpp** - reads data, calls functions to run the unscented Kalman filter and to calculate RMSE, and keep communication between simulator and uWebSocketIO.
  - **ukf.cpp** and **ukf.h** - initializes the filter, defines and calls the predict function, and the update functions for lidar and radar, respectively. Then calculate the NIS.
  - **tools.cpp** and **tools.h** - a function to calculate RMSE.
  - **measurement_package.h** - defines the data structure to process.
  - **json.hpp** - gives some communication protocols between simulator and uWebSocketIO.
- **README.md**: i.e. this project report you are seeing.

Note that the only files I need to modify are **ukf.cpp** and **tools.cpp** for basic requirements in [project rubrics](https://review.udacity.com/#!/rubrics/783/view). And the main protcol that main.cpp uses is the web socket server **uWebSocketIO** acting as a host, connecting the C++ programs to the Unity simulator, where the input values is provided by the simulator to the c++ program, and output values provided by the c++ program to the simulator.

The flow of UnscentedKF is Prediction -> LIDAR update -> RADAR update on this synthetic dataset, i.e. the dataset 1 in simulator, so I can narrow down the source of error when run into some problems.

## Simulation results
---
The laser measurements are denoted as red circles. The radar measurements are denoted as blue circles with an arrow pointing in the direction of the observed angle. And my estimation markers are denoted as green triangles as follows.

### Debug log
---
There is lots of parameter tuning work in UKF, which is different from EKF. And the tuned parameters mainly include process noise covariance (like tangitudinal acceleration and yaw acceleration) and the process state convariance matrix. Other important parameters like measurement noise covariance matrix are provided by sensor manufactors.  

For the new data set, my algorithm will be run against "obj_pose-laser-radar-synthetic-input.txt". Then I will collect the positions that my algorithm outputs and compare them to ground truth data. MY px, py, vx, and vy RMSE should be less than or equal to the values [.09, .10, .40, .30]. 

### The results

Here are my debugging history log on dataset 1 as follows:

-  std_a_ = 30, std_yawdd_ = 30, P_ = MatrixXd::Identity(5,5)   RMSE = [px,py,vx,vy] = [1.1938, 1.5831, 13.4151, 5.5681]

-  std_a_ = 3, std_yawdd_ = 3, P_ = MatrixXd::Identity(5,5) RMSE = [1.3741, 0.9463, 3.2221, 3.3153]

-  std_a_ = 3, std_yawdd_ = 3, P_(0,0) =0.15, P_(1,1) = 0.15;   RMSE = [1.3709, 0.9450, 3.2227, 3.3152]

-  std_a_ = 3, std_yawdd_ = 0.3, P_(0.0) = 0.15, P_(1,1) = 0.15; RMSE = [3.0633,3.3002, 2.4796, 2.8060]

-  std_a_ = 2, std_yawdd_ = 0.3, P_(0.0) = 0.15, P_(1,1) = 0.15; RMSE = [3.5316,3.2543, 2.7467, 2.7141]
   - use_laser_ = true, use_radar_ = false RMSE = [0.9162, 0.8822, 2.8584, 4.1300]
   - use_radar_ = true, use_laser_ = false RMSE = [3.5765, 5.3804, 2.4369, 2.9423]
   
-  std_a_ = 2, std_yawdd_ = M_PI/8, P_(0.0) = 0.15, P_(1,1) = 0.15; RMSE = [3.1767, 2.1980, 2.9290, 2.7359]

-  std_a_ = 1, std_yawdd_ = M_PI/8, P_(0.0) = 0.15, P_(1,1) = 0.15; RMSE = [2.4283, 2.0836, 2.8018, 3.6228]

So far the RMSE evaluation metrics are all beyond the rubric required threshold. So I debug my source code following [the guide in the forum](https://discussions.udacity.com/t/how-to-use-predicted-sigma-points-from-predict-function-in-update-steps/322286/22) . And I found the bug in Prediction() about the variable yaw update. After debugging I run the simulation on dataset 1 and dataset 2:

-  std_a_ = 1, std_yawdd_ = M_PI/8, P_(0.0) = 0.15, P_(1,1) = 0.15
   - dataset 1: RMSE = [0.0688, 0.0807, 0.2922, 0.2702]
   - dataset 2: RMSE = [0.0723, 0.0671, 0.4880, 0.2646]

-  std_a_ = 2.0, std_yawdd_ = M_PI/8, P_(0.0) = 0.15, P_(1,1) = 0.15
   - dataset 1: RMSE = [0.0738, 0.0819, 0.3052, 0.2841]
   - dataset 2: RMSE = [0.0766, 0.0745, 0.4950, 0.2916]

-  std_a_ = 2.0, std_yawdd_ = M_PI/10, P_(0.0) = 0.15, P_(1,1) = 0.15
   - dataset 1: RMSE = [0.0757, 0.0824, 0.3109, 0.2899]
   - dataset 2: RMSE = [0.0783, 0.0773, 0.4973, 0.2990]
   
-  std_a_ = 1.0, std_yawdd_ = M_PI/10, P_(0.0) = 0.15, P_(1,1) = 0.15
   - dataset 1: RMSE = [0.0708, 0.0808, 0.2976, 0.2990]
   - dataset 2: RMSE = [0.0739, 0.0697, 0.4902, 0.2712]
   
-  std_a_ = 0.8, std_yawdd_ = M_PI/10, P_(0.0) = 0.15, P_(1,1) = 0.15
   - dataset 1: RMSE = [0.0693, 0.0811, 0.2955, 0.2724]
   - dataset 2: RMSE = [0.0726, 0.0674, 0.4886, 0.2656] 

-  std_a_ = 0.8, std_yawdd_ = M_PI/8, P_(0.0) = 0.15, P_(1,1) = 0.15
   - dataset 1: RMSE = [0.0672, 0.0811, 0.2905, 0.2680]
   - dataset 2: RMSE = [0.0711, 0.0650, 0.4870, 0.2595]
   
-  std_a_ = 0.8, std_yawdd_ = M_PI/8, P_(0.0) = 0.15, P_(1,1) = 0.15, P_(2,2) = 0.15, P_(3,3) = 0.15, P_(4,4) = 0.15
   - dataset 1: RMSE =  [0.0698, 0.0823 0.3629 0.2611]
   - dataset 2: RMSE = [0.1055, 0.0655, 0.6050, 0.2512]

-  std_a_ = 0.75, std_yawdd_ = M_PI/8, P_(0.0) = 0.15, P_(1,1) = 0.15
   - dataset 1: RMSE = [0.0668, 0.0813 0.2902, 0.2675]
   - dataset 2: RMSE = [0.0708, 0.0644, 0.4868, 0.2583]

Evaluation Metrics - the lowest RMSE for now : [x,y,vx,vy] = [.0668, .0813, .2902, .2675], the same parameter configuration in my source code.

The threshold in program rubics: RMSE <=  [.09, .10, .40, .30].

Here the RMSE on dataset 1 meets this project rubric requirements. And I will improve the RMSE on dataset 2 in the future. 

---

## How to run this project
---

1. Clone the starter code repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make` 
4. Run it: `./UnscentedKF `
5. Open the Unity Simulator 'term2_sim_x86_64', select the 'Project 1/2: EKF and UKF'. Terminal shows connection success. Choose dataset 1 then dataset 2.
6. Tuning the UKF parameters such as 'std_a'  'std_yawdd', 'P__', and so on. See the RMSE and NIS.


## Code Style
---
Here I stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html). And I use the ubuntu file editor gedit with the similar user interface with emacs. 


## Project Instructions and Rubric
---

Note: regardless of the changes you make, your project must be buildable using cmake and make! Passing the project requires obtaining RMSE values that are lower that the tolerance outlined in the project rubric. See [the project resources page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/daf3dee8-7117-48e8-a27a-fc4769d2b954/concepts/bbeb991a-bebd-4c1f-a590-0ef0ccdd48d6) for instructions and the project rubric.
