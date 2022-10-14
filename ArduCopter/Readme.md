# INDI angular acceleration compensation

Basic Equation.

$\tau _{d}$ is the angular acceleration send to motors mixer.

$\tau _{f}$ is the feed back angular acceleration computed from the rotors motor speed.

$I_{v}$ is the inertia of the rotor.

$\dot{\Omega }_{f}^{B}$ is the angular accleration of the copter.

$w_{f}$ is the speed of the motors(RPM or rad/s). $\dot{w_{f}}$ is the acceleration of the motors.

$I_{v}\widehat{\dot{\Omega }}_{d}^{B}$ is the command form angular rate PID.

$I_{p}$ is the propeller's inertia.

$l$ is the arm length.

$$ \tau _{d} =\ \tau _{f} +I_{v}\left(\widehat{\dot{\Omega }}_{d}^{B} -\dot{\Omega }_{f}^{B}\right) =I_{v}\widehat{\dot{\Omega }}_{d}^{B} +\left( \tau _{f} -I_{v}\dot{\Omega }_{f}^{B}\right) $$

$$ \tau _{f} =G_{1} w_{f}^{2} +G_{2}\dot{w_{f}} $$

$$ G_{1} =\left[ -\begin{matrix}
K_{f} l\sin \alpha  & -K_{f} l\sin \alpha  & -K_{f} l\sin \alpha  & K_{f} l\sin \alpha \\
K_{f} l\cos \alpha  & -K_{f} l\cos \alpha  & K_{f} l\cos \alpha  & K_{f} l\cos \alpha \\
K_{M} & -K_{M} & K_{M} & -K_{M}
\end{matrix}\right] $$

$$ G_{2} \ =\ \left[\begin{matrix}
0 & 0 & 0 & 0\\
0 & 0 & 0 & 0\\
I_{p} & -I_{p} & I_{p} & -I_{p}
\end{matrix}\right] $$

## Parameters

`ATC_USE_INDI` Enable INDI. 0 means close. 1 means enable estimation and compensation. 2 means means start estimation and send data but disable compensation.

`ATC_MIN_INDI_SPD` The lowest speed start compensation and estimation. (RPM)

`ATC_INDI_KF` The $K_f$ of the motor. 

`ATC_INDI_KM` The $K_m$ of the motor. 

`ATC_PROP_INERTIA` The $I_p$. $I_p$ is the propeller's inertia.

`ATC_CUTOFF_RPM` Cutoff freqency of the RPM ( $w_{f}$ ) filter. Shoud be same as the `ATC_WACC_CUTOFF`

`ATC_WACC_CUTOFF` Cutoff freqency of the angular acceleration $\dot{\Omega }_{f}^{B}$ filter. Shoud be same as the `ATC_CUTOFF_RPM`

`ATC_DRPM_CUTOFF`Cutoff freqency of the $\dot{w_{f}}$ filter. 

`ATC_CUTOFF_MT` Cutoff freqency of filter of final moment $\left( \tau _{f} -I_{v}\dot{\Omega }_{f}^{B}\right)$. 

`ATC_INDI_ARM_S` The scale of the INDI. ($l*sin45 = l*cos45$).  For `X` frame this value is $l*\sqrt{2}$ . (Due to the motor factor in ardupilot is normalized into 0.5. Here we use the $l*\sqrt{2}$ rather than $l*\sqrt{2}/2$ ). For `+` Frame this value is $l*2$. (We haven't tested the `+` frame.)

`MAX_MT_XY` Max moment compensation in XY (The moment compensation value equals to the moment mutiplied by the INDI_K_XY).

`MAX_MT_Z` Max moment compensation in Z.

`INDI_K_XY`  The scale convert the moment to the command in XY (Roll pitch). 

`INDI_K_Z`  The scale convert the moment to the command in Z(Yaw). 

`COP_IXX - COP_IZZ` the inertia of the copter.