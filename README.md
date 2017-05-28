# SelfDrivingCar2-PID

# A PID controller for vehicle steering in a virtual track

## The PID class: Brief overview
I controller is implemented by the `PID` class. The class uses method `init` to initialize the gains and the mode of operation (i.e., _adaptive_ or _non-adaptive_ with respefct to the gains). If the _adapting_ flag is set, then the controller will use the _twiddle_ heuristic to adapt the gains according to the average cte error in a sequence of feedback loops. The `UpdateError` updates and stores the _new error_, the _finite difference of the error_ and _cumulative error_ in order to produce the next feedback, which is obtained as the weigted sum of the gains-times-errors using the `Feedback` method.

### Compiling and Executing
Compling and running should be straightforward:
```

./install_ubuntu.sh
mkdir build
cd build
cmake ..
make
./pid
```
Then run the simulator in auto-driving mode. The simulator can be dowloaded from [here](https://github.com/udacity/CarND-PID-Control-Project/releases).

## PID gains
The proportional gain is the necessary gain to produce an immediate reaction to error. However, this reaction produces oscillatory motion around the _putative_ trajectory as shown [in this video](https://1drv.ms/u/s!AtmapBHRVgqWgWAip2EykUtDHrOU), especially when speed is over or in the range of 20 miles. High frequency oscillatory motion can be erradicated (or at least, minimized) by the _derivative_ gain as shown [in this video](https://1drv.ms/u/s!AtmapBHRVgqWgV_X7BhqyduXWsy1). With the use of the derivative term, oscillation is completely removed for speeds between 20 and 30 miles, while it is significantly reduced when crusing above 30 miles. However, it becomes evident that acumulated error "catches up" with the vehicle and throws it off the track shortly before or after the bridge. To avoid such issues, the _intergral_ gain compensates for low-frequency errors which introduce themselves as long-term offsets from the putative trajectory. Using an integral term, these displacements are eliminated. In practice, it can be observed that low-frequency errors are small, yet accumulate through time thereby making the vehicle unstable at about after or during it has taken its course through the bridge. Thus, compensation with a small gain is necessary to allow the controller to gently compensate for small, persistent offset errors which acculuate through time.

### PID Controllers employed
There are two PID controllers: One for steering and one for throttle. Of course, the steering controller is the important one, while the throttle controller is simply used to allow the vehicle to pick-up speed when the road is straight. They both have similar gain proportions with emphasis on transient oscillatory motion prevention (i.e., large derivative gain in comparison to the rest).

To make the steering controller sensitive to speed (in other words, make it more _aggressive_ when speed is high), the PID feedback is multiplied with a factor `(0.01 + speed)` which seems to be working well in practice. To pass-on the scaled steering PID weighted sum to the simulator input in the range of `[-1, 1]`, simple thresholding is used. My original idea was to warp the feedback `y` with the logistic function, `-1 + 1 / ( 1 + exp(-y) )`, which in comparison to simple thresholding caused a little bit of additional gidder, so it was dropped. On the other hand, the logistic function warping was used in the case of the throttle controller. The throttle PID feedback is warped through `0.1 + 0.7 / (1 + epx(-y) )` in order to yield a value in `[0.1, 0.8]` for the simulator.  

### PID Tuning
Clearly, the most important problem in tuning a PID controller in this case is that it would have to be done through some type of _proactive_ heuristic, which would have to account for large errors that can throw the car off-track (thereby forcing us to reset the simulation in most cases). Thus, the "twiddle" heuristic was modified and incorporated in the PID in such a way as to be able to run on batches of feedback loops as the vehicle is crusing relatively safely through the track. The twiddle works only when the _adaptive_ flag has been set duing the PID initialization.

In practice, using _twiddle_ here is usefull only to _refine_ the gains as the vehicle is moving through the track, provided that the initial values were **fairly robust** to keep use safely on the road. For this reason, the increments/decrements in the gains are set to be relatively low, to avoid track "accidents" and thereefore, significant improvements in performance will take a lot of time. For the aforementioned reasons, tuning of both PID controllers was primarily done _empirically_ for the most part. In particular, pivoting around a proportional gain of 1, and a similar derivative gain and with some twiddling, it was observed that `(Kp, Kd, Ki) = (.1 , 1.55 , 0.00045)` does a very good job for constant throttle at 0.3 or PID-controlled throttle in `[0.1, 0.8]`. Further fine-tuning can be done using "twiddle" but it may take a while to make significant difference.     
