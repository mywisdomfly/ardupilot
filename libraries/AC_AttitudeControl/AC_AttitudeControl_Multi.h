#pragma once

/// @file    AC_AttitudeControl_Multi.h
/// @brief   ArduCopter attitude control library

#include "AC_AttitudeControl.h"
#include <AP_Motors/AP_MotorsMulticopter.h>

// default rate controller PID gains
#ifndef AC_ATC_MULTI_RATE_RP_P
  # define AC_ATC_MULTI_RATE_RP_P           0.135f
#endif
#ifndef AC_ATC_MULTI_RATE_RP_I
  # define AC_ATC_MULTI_RATE_RP_I           0.135f
#endif
#ifndef AC_ATC_MULTI_RATE_RP_D
  # define AC_ATC_MULTI_RATE_RP_D           0.0036f
#endif
#ifndef AC_ATC_MULTI_RATE_RP_IMAX
 # define AC_ATC_MULTI_RATE_RP_IMAX         0.5f
#endif
#ifndef AC_ATC_MULTI_RATE_RP_FILT_HZ
 # define AC_ATC_MULTI_RATE_RP_FILT_HZ      20.0f
#endif
#ifndef AC_ATC_MULTI_RATE_YAW_P
 # define AC_ATC_MULTI_RATE_YAW_P           0.180f
#endif
#ifndef AC_ATC_MULTI_RATE_YAW_I
 # define AC_ATC_MULTI_RATE_YAW_I           0.018f
#endif
#ifndef AC_ATC_MULTI_RATE_YAW_D
 # define AC_ATC_MULTI_RATE_YAW_D           0.0f
#endif
#ifndef AC_ATC_MULTI_RATE_YAW_IMAX
 # define AC_ATC_MULTI_RATE_YAW_IMAX        0.5f
#endif
#ifndef AC_ATC_MULTI_RATE_YAW_FILT_HZ
 # define AC_ATC_MULTI_RATE_YAW_FILT_HZ     2.5f
#endif
#ifndef AC_ATC_USE_INDI
 # define AC_ATC_USE_INDI     0
#endif
#ifndef AC_ATC_MIN_INDI_SPEED
 # define AC_ATC_MIN_INDI_SPEED     3000.0f
#endif
#ifndef AC_ATC_INDI_KF
 # define AC_ATC_INDI_KF     1.0e-8f
#endif
#ifndef AC_ATC_INDI_KM
 # define AC_ATC_INDI_KM     1.5e-10f
#endif
#ifndef AC_ATC_PROPELLER_INERTIA
 # define AC_ATC_PROPELLER_INERTIA     1.0e-8f
#endif
#ifndef AC_ATC_QINERTIA_IXX
 # define AC_ATC_QINERTIA_IXX     0.001f
#endif
#ifndef AC_ATC_QINERTIA_IXY
 # define AC_ATC_QINERTIA_IXY     0.0f
#endif
#ifndef AC_ATC_QINERTIA_IXZ
 # define AC_ATC_QINERTIA_IXZ     0.0f
#endif
#ifndef AC_ATC_QINERTIA_IYX
 # define AC_ATC_QINERTIA_IYX     0.0f
#endif
#ifndef AC_ATC_QINERTIA_IYY
 # define AC_ATC_QINERTIA_IYY     0.001f
#endif
#ifndef AC_ATC_QINERTIA_IYZ
 # define AC_ATC_QINERTIA_IYZ     0.0f
#endif
#ifndef AC_ATC_QINERTIA_IZX
 # define AC_ATC_QINERTIA_IZX     0.0f
#endif
#ifndef AC_ATC_QINERTIA_IZY
 # define AC_ATC_QINERTIA_IZY     0.0f
#endif
#ifndef AC_ATC_QINERTIA_IZZ
 # define AC_ATC_QINERTIA_IZZ     0.001f
#endif
#ifndef AC_ATC_CUT_OFF_RPM_FILTER
 # define AC_ATC_CUT_OFF_RPM_FILTER    12.0f
#endif
#ifndef AC_ATC_CUT_OFF_MOMENT_FILTER
 # define AC_ATC_CUT_OFF_MOMENT_FILTER    12.0f
#endif
#ifndef AC_ATC_MAX_MOMENT_XY
 # define AC_ATC_MAX_MOMENT_XY    0.2f
#endif
#ifndef AC_ATC_MAX_MOMENT_Z
 # define AC_ATC_MAX_MOMENT_Z    0.1f
#endif
#ifndef AC_ATC_ARM_SCALE
 # define AC_ATC_ARM_SCALE    0.17677669529f
#endif
#ifndef AC_ATC_INDI_SCALE
 # define AC_ATC_INDI_SCALE    1.0f
#endif
#ifndef AC_ATC_ANGACC_CUTOFF
 # define AC_ATC_ANGACC_CUTOFF    12.0f
#endif
#ifndef AC_ATC_DRPM_CUTOFF
 # define AC_ATC_DRPM_CUTOFF    12.0f
#endif
class QInertia
{
  public:
  AP_Float Ixx,Ixy,Ixz;
  AP_Float Iyx,Iyy,Iyz;
  AP_Float Izx,Izy,Izz;
  // static const struct AP_Param::GroupInfo var_info[];
};
class AC_AttitudeControl_Multi : public AC_AttitudeControl {
public:
	AC_AttitudeControl_Multi(AP_AHRS_View &ahrs, const AP_Vehicle::MultiCopter &aparm, AP_MotorsMulticopter& motors, float dt);

	// empty destructor to suppress compiler warning
	virtual ~AC_AttitudeControl_Multi() {}

    // pid accessors
    AC_PID& get_rate_roll_pid() override { return _pid_rate_roll; }
    AC_PID& get_rate_pitch_pid() override { return _pid_rate_pitch; }
    AC_PID& get_rate_yaw_pid() override { return _pid_rate_yaw; }

    // Update Alt_Hold angle maximum
    void update_althold_lean_angle_max(float throttle_in) override;

    // Set output throttle
    void set_throttle_out(float throttle_in, bool apply_angle_boost, float filt_cutoff) override;

    // calculate total body frame throttle required to produce the given earth frame throttle
    float get_throttle_boosted(float throttle_in);

    // set desired throttle vs attitude mixing (actual mix is slewed towards this value over 1~2 seconds)
    //  low values favour pilot/autopilot throttle over attitude control, high values favour attitude control over throttle
    //  has no effect when throttle is above hover throttle
    void set_throttle_mix_min() override { _throttle_rpy_mix_desired = _thr_mix_min; }
    void set_throttle_mix_man() override { _throttle_rpy_mix_desired = _thr_mix_man; }
    void set_throttle_mix_max(float ratio) override;
    void set_throttle_mix_value(float value) override { _throttle_rpy_mix_desired = _throttle_rpy_mix = value; }
    float get_throttle_mix(void) const override { return _throttle_rpy_mix; }

    // are we producing min throttle?
    bool is_throttle_mix_min() const override { return (_throttle_rpy_mix < 1.25f * _thr_mix_min); }

    // run lowest level body-frame rate controller and send outputs to the motors
    void rate_controller_run() override;

    // sanity check parameters.  should be called once before take-off
    void parameter_sanity_check() override;
    
    void init_indi();
    void update_indi_moment(); //update INDI info
    Vector3f get_indi_moment(){return _compensation_moment;}
    Vector3f get_indi_angular_acceleration(){return _angular_acceleration;}
    VectorN<float,4> get_indi_rpm_info(){return _rpm_filtered;}
    VectorN<float,4> get_indi_delta_rpm_info(){return _delta_rpm;}

    //Param of INDI
    AP_Int8               _use_indi;
    // user settable parameters
    static const struct AP_Param::GroupInfo var_info[];

protected:

    // update_throttle_rpy_mix - updates thr_low_comp value towards the target
    void update_throttle_rpy_mix();

    // get maximum value throttle can be raised to based on throttle vs attitude prioritisation
    float get_throttle_avg_max(float throttle_in);

    AP_MotorsMulticopter& _motors_multi;
    AC_PID                _pid_rate_roll;
    AC_PID                _pid_rate_pitch;
    AC_PID                _pid_rate_yaw;

    AP_Float              _thr_mix_man;     // throttle vs attitude control prioritisation used when using manual throttle (higher values mean we prioritise attitude control over throttle)
    AP_Float              _thr_mix_min;     // throttle vs attitude control prioritisation used when landing (higher values mean we prioritise attitude control over throttle)
    AP_Float              _thr_mix_max;     // throttle vs attitude control prioritisation used during active flight (higher values mean we prioritise attitude control over throttle)

    //Param of INDI
    AP_Float              _MIN_INDI_SPEED;
    AP_Float              _INDI_KF;
    AP_Float              _INDI_KM;
    AP_Float              _PROPELLER_INERTIA;
    AP_Float              _CUT_OFF_RPM_FILTER;
    AP_Float              _CUT_OFF_MOMENT_FILTER;
    AP_Float              _MAX_MOMENT_XY;
    AP_Float              _MAX_MOMENT_Z;
    QInertia              _qinertia;
    AP_Float              _INDI_K_XY;
    AP_Float              _INDI_K_Z;
    AP_Float              _ARM_SCALE;
    AP_Float              _DRPM_CUTOFF;
    AP_Float              _ANGACC_CUTOFF;
    AP_Float              _INDI_BIAS_X;
    AP_Float              _INDI_BIAS_Y;
    AP_Float              _INDI_BIAS_Z;

    LowPassFilter2p<VectorN<float,4>> _rpm_filter;
    LowPassFilter2pVector3f _moment_filter;
    LowPassFilterVector3f _ang_acc_filter;
    LowPassFilter<VectorN<float,4>> _drpm_filter;
    VectorN<float,4> _rpm_filtered;//Rad/s

    VectorN<float,4> _delta_rpm; //Rad/s^2

    Vector3f _angular_acceleration;
    Vector3f _compensation_moment;
    Vector3f _compensation_moment_raw;


    typedef DerivativeFilterFloat_Size5 DerivativeFilterUSED;
    struct 
    {
      DerivativeFilterUSED ang_acc_x_derivative_filter;
      DerivativeFilterUSED ang_acc_y_derivative_filter;
      DerivativeFilterUSED ang_acc_z_derivative_filter;
      void update(Vector3f &gyro, uint32_t now)
      {
        ang_acc_x_derivative_filter.update(gyro.x,now);
        ang_acc_y_derivative_filter.update(gyro.y,now);
        ang_acc_z_derivative_filter.update(gyro.z,now);
      }
      Vector3f get_derivative()
      {
        Vector3f ang_acc;
        ang_acc.x=ang_acc_x_derivative_filter.slope()*1.0e+6;
        ang_acc.y=ang_acc_y_derivative_filter.slope()*1.0e+6;
        ang_acc.z=ang_acc_z_derivative_filter.slope()*1.0e+6;
        return ang_acc;
      }
    } ang_acc_derivativtor;
    struct 
    {
      DerivativeFilterUSED rpm1_derivative_filter;
      DerivativeFilterUSED rpm2_derivative_filter;
      DerivativeFilterUSED rpm3_derivative_filter;
      DerivativeFilterUSED rpm4_derivative_filter;
      void update(VectorN<float,4> &rpm, uint32_t now)
      {
        rpm1_derivative_filter.update(rpm[0],now);
        rpm2_derivative_filter.update(rpm[1],now);
        rpm3_derivative_filter.update(rpm[2],now);
        rpm4_derivative_filter.update(rpm[3],now);
      }
      VectorN<float,4> get_derivative()
      {
        VectorN<float,4> drpm;
        drpm[0]=rpm1_derivative_filter.slope()*1.0e+6;
        drpm[1]=rpm2_derivative_filter.slope()*1.0e+6;
        drpm[2]=rpm3_derivative_filter.slope()*1.0e+6;
        drpm[3]=rpm4_derivative_filter.slope()*1.0e+6;
        return drpm;
      }
    } rpm_derivativtor;


};
