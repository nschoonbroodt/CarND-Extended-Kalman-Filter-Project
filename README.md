# Extended Kalman Filter Project Starter Code
Self-Driving Car Engineer Nanodegree Program

---

## Technical choices
There were few technical choices to be made in this project as the skeleton of code was already large, and the EKF was well described in the class lesson. But there were still a few decisions to be made.

### Filter initialization
To initialize the filter, there are two distincts cases. 

The first one is when we get the first data from the lidar. The lidar gives us a good position measurement, but no speed data. 
In this case, I set the position parts of the initial state to the measured value, with a small value for the position elements of the state covariance matrix, and the initial speed to 0 with a large value for the covariance of the speed, indicating that this speed of 0 is not good.

The second case is when we get the first data from the radar. In this case, there is a position available (that I convert from polar to cartesian) and a partial speed measurement, as we only get information about the radial speed. Even if this speed measurement is partial, it's a better estimation of the speed than just putting a zero speed in the state, so I use this value (again, transformed from polar to cartesian). For the state covariance matrix, I set a covariance small (but not as small as for the lidar) for the position part, and a larger (but much smaller than the lidar one) coefficient for the speed covariance.

### Avoid prediction step if small dt
If the time between two measurements is small (defined as less than 1ms in my code. For reference, the dt in the given data is always 50ms), I skip the prediction step and keep the previous state.

If we had synchronized data from the lidar and radar (instead of offseted), this would lead to the following sequence: `predict(), update(lidar), update(radar)` instead of `predict(), update(lidar), update(radar)`. This avoid some unecessary computations, as the second predict would be with a dt -> 0, so not propagation very far.

### Minor optimization
I also used a common function in the `KalmanFilter::Update()` and `KalmanFilter::UpdateEKF`, as the equations are identical, except for the computation of the predicted measurement.

## Results

On the Udacity data, I get a RMSE of (0.097, 0.085, 0.45, 0.44), below the demanded target. Here is a plot of the results:

![Results](/images/result.png)

On this image, it's difficult to see the difference between the ground thruth and the estimated value, as they overlap. Here is a zoom on the start of the sequence, where we see that the initial value is exactly as the first measurement (expected, as we set it to this value), and then goes closer to the ground thruth, as more measurements comes in:

![Zoom lidar](/images/zoom_in_lidar.png)

I've also used the same data without the first measurement, to see what happens when the first measurement is a radar measurement.

![Zoom lidar](/images/zoom_in_radar.png)

You should also be able to find thesi results [here (lidar start)](https://plot.ly/create/?fid=2PetitsVerres:1) and [here (radar start)](https://plot.ly/create/?fid=2PetitsVerres:3), with interactive charts.