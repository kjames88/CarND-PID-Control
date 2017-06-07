##CarND-PID-Control Project##

###Purpose###

This project implements a PID controller for steering a car around a racetrack based on cross-track-error (CTE).  The PID controller uses a sum of three weighted metrics to derive a response.  CTE is used directly for the *proportional* component, the *differential* CTE from step to step is used as a damping factor to limit overshoot due to the proportional component, and the *integral* of CTE is used to eliminate drift or persistent offset.

####Proportional Component####

The proportional component of the PID controller acts directly against the CTE.  If the car is on the right side of where it should be, the controller steers left.  If the car is offset farther to the right, steering angle is increased proportionately.

####Differential Component####

The differential component of the PID controller counterbalances the proportional component.  If the car steers left to correct a deviation to the right, the steering angle will tend to be large enough for a sufficient duration to carry the car across the line and into a left deviation.  The differential term captures the reduction in CTE resulting from previous control inputs and effectively counter-steers so that the CTE approaches 0 smoothly.  Of course, overweighting this component can overcompensate and become the cause of oscillation.

####Integral Component####

The integral component of the PID controller compensates for drift.  In the case of the racetrack, the curvature of the road causes the error to increase when the car is otherwise driving straight and centered.  Without this integral component, the P and D components would adjust the steering, but with a moving target a large steering oscillation would result.

###Caveats###

The PID controller performs reasonably well at modest speed, such as the default 0.3 throttle.  However, when the speed is increased, significant oscillation develops with the parameters set for slower speed.  This is, presumably, due to delay in the control path.  Assuming the simulator is physically accurate, steering cannot have an instantaneous effect, and increased speed results in increased offsets that overwhelm the controller responses.  The situation can be improved by re-tuning the parameters, but that would add speed as a second input parameter to the controller.

###Speed Regulation###

As a first step to enabling higher average speed, a second PD controller is used to regulate a target speed.  The target speed is set relative to the *integral* error of the steering controller.  A minimum of 10MPH if enforced so that the car continues moving even when significant error accumulates.  The resulting speed is considerably higher than the default, but drops as the car deviates from the center line.  In particular, the intent is to reduce speed around curves.  Since the controller is only reactive, the driving quality is not especially good; a better system would at least anticipate the curve and slow down in advance.

###Parameter Tuning###

Key to successful operation of a PID controller is good selection of the weights applied to the P, I, and D components.  I initially tried and adjusted values based on the magnitudes used in the lessons.  This got the car around the track fairly quickly but needed improving.  I wanted to determine the numbers automatically using twiddle, but the simulator presents some challenges.  I was able to reset the simulator for repeated attempts over the same region while adjusting parameters, but I needed a pipeline flush ability to properly isolate one round from the next.  In addition, looking forward, I would have needed a position indicator or a track cycle complete flag to evaluate the whole track each iteration.  Otherwise, the parameters would be tuned based on only one section of track and might therefore perform poorly around some of the later curves.

I wanted to see what twiddle would do so I spent some hours manually applying twiddle adjustments.  This painful process did reveal a surprise early success long before the *I* parameter magnitude was reduced to a usable level, but targeted in the same realm as my quickly selected manually tuned parameters.  I did not continue to very high resolution, which might have accomplished greater refinement, because the process is quite tedious without automation.

The final parameters were tuned manually.

###Testing Results###

**The simulator version matters**

The controller can be started with a speed target parameter:  *./pid 50* increases the target from the default of 45MPH to 50MPH.  Using the simulator from 1-2 weeks ago (single mode with two tracks available) the car goes off the track at 70-75MPH.  At 65MPH the steering oscillates considerably but the car stays on the track.  Up to 55MPH the driving should be "safe."

The v1.3 simulator drives reasonably well at 50MPH, whereas the v1.4 simulator oscillates too much, but stays on the track.  Overall, the current parameters, which are tuned for the older simulator, are too strong for the v1.4 and possibly also for the v1.3 simulators.  I had previously reduced the parameters for v1.4 but that may result in inadequate control response on the older simulator.
   

