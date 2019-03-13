# hybrid-quad-cord-ball-control
Trajectory Optimization Control and Robustness of an Hybrid system, a 2D Quadcopter with a cord attached to a ball.

In this control problem we have a 2D Quadrotor with a load attached through a chord, which is a Hybrid System. The system is very similar to a 2D Quadrotor with a pendulum, with the difference that, when the inner tension forces inside the “pendulum” (chord in this case) are negative, the system will behave in a very different way. In fact, the load will be “detached” from the quadrotor and will behave as a free-falling ball, while the quadrotor will behave as a single and simple Quad. This behavior will take place as long as the distance from the load to the quad is less than *l*, the length of the chord. When the equality takes place, the instantaneous tension of the chord, will be modeled as a common impact between objects. If the collision is inelastic, the two bodies will behave as a quad with pendulum again, this is the first mode of the hybrid system. If the collision is elastic, the load will bounce, going back to the second mode (load “detached” of “flying”), but with a reset map applied.


<p align="center">
<img src="graphics/HybrSystem.png">
</p>

The window has a height of a, with a<l, so we have now the control problem: Let’s make the quadrotor cross the window in order to take the system to the other side of the window.

After performing Hybrid Trajectory Optimization, specifically Direct Collocation, we get the following local optimal trajectory for a LQR cost function:

<p align="center">
<img src="graphics/orig.png">
</p>

But Trajectory Optimization gives a control signal in time, not feedback, open loop, which are in general not robust. In order to make it robust we perform an Hybrid TV-LQR, which is the same as TV-LQR but with a "Jump Riccate Equation" associated to the transitions between modes in the Hybrid System. We got an robust controller for the trajectory computed before.

<p align="center">
<img src="graphics/TVLQR2.gif">
</p>

Note that initial conditions here are
