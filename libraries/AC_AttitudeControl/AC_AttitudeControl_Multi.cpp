#include "AC_AttitudeControl_Multi.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>

// table of user settable parameters
const AP_Param::GroupInfo AC_AttitudeControl_Multi::var_info[] = {
    // parameters from parent vehicle
    AP_NESTEDGROUPINFO(AC_AttitudeControl, 0),

    // @Param: RAT_RLL_P
    // @DisplayName: Roll axis rate controller P gain
    // @Description: Roll axis rate controller P gain.  Converts the difference between desired roll rate and actual roll rate into a motor speed output
    // @Range: 0.01 0.5
    // @Increment: 0.005
    // @User: Standard

    // @Param: RAT_RLL_I
    // @DisplayName: Roll axis rate controller I gain
    // @Description: Roll axis rate controller I gain.  Corrects long-term difference in desired roll rate vs actual roll rate
    // @Range: 0.01 2.0
    // @Increment: 0.01
    // @User: Standard

    // @Param: RAT_RLL_IMAX
    // @DisplayName: Roll axis rate controller I gain maximum
    // @Description: Roll axis rate controller I gain maximum.  Constrains the maximum motor output that the I gain will output
    // @Range: 0 1
    // @Increment: 0.01
    // @User: Standard

    // @Param: RAT_RLL_D
    // @DisplayName: Roll axis rate controller D gain
    // @Description: Roll axis rate controller D gain.  Compensates for short-term change in desired roll rate vs actual roll rate
    // @Range: 0.0 0.05
    // @Increment: 0.001
    // @User: Standard

    // @Param: RAT_RLL_FF
    // @DisplayName: Roll axis rate controller feed forward
    // @Description: Roll axis rate controller feed forward
    // @Range: 0 0.5
    // @Increment: 0.001
    // @User: Standard

    // @Param: RAT_RLL_FLTT
    // @DisplayName: Roll axis rate controller target frequency in Hz
    // @Description: Roll axis rate controller target frequency in Hz
    // @Range: 5 100
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: RAT_RLL_FLTE
    // @DisplayName: Roll axis rate controller error frequency in Hz
    // @Description: Roll axis rate controller error frequency in Hz
    // @Range: 0 100
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: RAT_RLL_FLTD
    // @DisplayName: Roll axis rate controller derivative frequency in Hz
    // @Description: Roll axis rate controller derivative frequency in Hz
    // @Range: 5 100
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: RAT_RLL_SMAX
    // @DisplayName: Roll slew rate limit
    // @Description: Sets an upper limit on the slew rate produced by the combined P and D gains. If the amplitude of the control action produced by the rate feedback exceeds this value, then the D+P gain is reduced to respect the limit. This limits the amplitude of high frequency oscillations caused by an excessive gain. The limit should be set to no more than 25% of the actuators maximum slew rate to allow for load effects. Note: The gain will not be reduced to less than 10% of the nominal value. A value of zero will disable this feature.
    // @Range: 0 200
    // @Increment: 0.5
    // @User: Advanced

    AP_SUBGROUPINFO(_pid_rate_roll, "RAT_RLL_", 1, AC_AttitudeControl_Multi, AC_PID),

    // @Param: RAT_PIT_P
    // @DisplayName: Pitch axis rate controller P gain
    // @Description: Pitch axis rate controller P gain.  Converts the difference between desired pitch rate and actual pitch rate into a motor speed output
    // @Range: 0.01 0.50
    // @Increment: 0.005
    // @User: Standard

    // @Param: RAT_PIT_I
    // @DisplayName: Pitch axis rate controller I gain
    // @Description: Pitch axis rate controller I gain.  Corrects long-term difference in desired pitch rate vs actual pitch rate
    // @Range: 0.01 2.0
    // @Increment: 0.01
    // @User: Standard

    // @Param: RAT_PIT_IMAX
    // @DisplayName: Pitch axis rate controller I gain maximum
    // @Description: Pitch axis rate controller I gain maximum.  Constrains the maximum motor output that the I gain will output
    // @Range: 0 1
    // @Increment: 0.01
    // @User: Standard

    // @Param: RAT_PIT_D
    // @DisplayName: Pitch axis rate controller D gain
    // @Description: Pitch axis rate controller D gain.  Compensates for short-term change in desired pitch rate vs actual pitch rate
    // @Range: 0.0 0.05
    // @Increment: 0.001
    // @User: Standard

    // @Param: RAT_PIT_FF
    // @DisplayName: Pitch axis rate controller feed forward
    // @Description: Pitch axis rate controller feed forward
    // @Range: 0 0.5
    // @Increment: 0.001
    // @User: Standard

    // @Param: RAT_PIT_FLTT
    // @DisplayName: Pitch axis rate controller target frequency in Hz
    // @Description: Pitch axis rate controller target frequency in Hz
    // @Range: 5 100
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: RAT_PIT_FLTE
    // @DisplayName: Pitch axis rate controller error frequency in Hz
    // @Description: Pitch axis rate controller error frequency in Hz
    // @Range: 0 100
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: RAT_PIT_FLTD
    // @DisplayName: Pitch axis rate controller derivative frequency in Hz
    // @Description: Pitch axis rate controller derivative frequency in Hz
    // @Range: 5 100
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: RAT_PIT_SMAX
    // @DisplayName: Pitch slew rate limit
    // @Description: Sets an upper limit on the slew rate produced by the combined P and D gains. If the amplitude of the control action produced by the rate feedback exceeds this value, then the D+P gain is reduced to respect the limit. This limits the amplitude of high frequency oscillations caused by an excessive gain. The limit should be set to no more than 25% of the actuators maximum slew rate to allow for load effects. Note: The gain will not be reduced to less than 10% of the nominal value. A value of zero will disable this feature.
    // @Range: 0 200
    // @Increment: 0.5
    // @User: Advanced

    AP_SUBGROUPINFO(_pid_rate_pitch, "RAT_PIT_", 2, AC_AttitudeControl_Multi, AC_PID),

    // @Param: RAT_YAW_P
    // @DisplayName: Yaw axis rate controller P gain
    // @Description: Yaw axis rate controller P gain.  Converts the difference between desired yaw rate and actual yaw rate into a motor speed output
    // @Range: 0.10 2.50
    // @Increment: 0.005
    // @User: Standard

    // @Param: RAT_YAW_I
    // @DisplayName: Yaw axis rate controller I gain
    // @Description: Yaw axis rate controller I gain.  Corrects long-term difference in desired yaw rate vs actual yaw rate
    // @Range: 0.010 1.0
    // @Increment: 0.01
    // @User: Standard

    // @Param: RAT_YAW_IMAX
    // @DisplayName: Yaw axis rate controller I gain maximum
    // @Description: Yaw axis rate controller I gain maximum.  Constrains the maximum motor output that the I gain will output
    // @Range: 0 1
    // @Increment: 0.01
    // @User: Standard

    // @Param: RAT_YAW_D
    // @DisplayName: Yaw axis rate controller D gain
    // @Description: Yaw axis rate controller D gain.  Compensates for short-term change in desired yaw rate vs actual yaw rate
    // @Range: 0.000 0.02
    // @Increment: 0.001
    // @User: Standard

    // @Param: RAT_YAW_FF
    // @DisplayName: Yaw axis rate controller feed forward
    // @Description: Yaw axis rate controller feed forward
    // @Range: 0 0.5
    // @Increment: 0.001
    // @User: Standard

    // @Param: RAT_YAW_FLTT
    // @DisplayName: Yaw axis rate controller target frequency in Hz
    // @Description: Yaw axis rate controller target frequency in Hz
    // @Range: 1 50
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: RAT_YAW_FLTE
    // @DisplayName: Yaw axis rate controller error frequency in Hz
    // @Description: Yaw axis rate controller error frequency in Hz
    // @Range: 0 20
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: RAT_YAW_FLTD
    // @DisplayName: Yaw axis rate controller derivative frequency in Hz
    // @Description: Yaw axis rate controller derivative frequency in Hz
    // @Range: 5 50
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: RAT_YAW_SMAX
    // @DisplayName: Yaw slew rate limit
    // @Description: Sets an upper limit on the slew rate produced by the combined P and D gains. If the amplitude of the control action produced by the rate feedback exceeds this value, then the D+P gain is reduced to respect the limit. This limits the amplitude of high frequency oscillations caused by an excessive gain. The limit should be set to no more than 25% of the actuators maximum slew rate to allow for load effects. Note: The gain will not be reduced to less than 10% of the nominal value. A value of zero will disable this feature.
    // @Range: 0 200
    // @Increment: 0.5
    // @User: Advanced

    AP_SUBGROUPINFO(_pid_rate_yaw, "RAT_YAW_", 3, AC_AttitudeControl_Multi, AC_PID),

    // @Param: THR_MIX_MIN
    // @DisplayName: Throttle Mix Minimum
    // @Description: Throttle vs attitude control prioritisation used when landing (higher values mean we prioritise attitude control over throttle)
    // @Range: 0.1 0.25
    // @User: Advanced
    AP_GROUPINFO("THR_MIX_MIN", 4, AC_AttitudeControl_Multi, _thr_mix_min, AC_ATTITUDE_CONTROL_MIN_DEFAULT),

    // @Param: THR_MIX_MAX
    // @DisplayName: Throttle Mix Maximum
    // @Description: Throttle vs attitude control prioritisation used during active flight (higher values mean we prioritise attitude control over throttle)
    // @Range: 0.5 0.9
    // @User: Advanced
    AP_GROUPINFO("THR_MIX_MAX", 5, AC_AttitudeControl_Multi, _thr_mix_max, AC_ATTITUDE_CONTROL_MAX_DEFAULT),

    // @Param: THR_MIX_MAN
    // @DisplayName: Throttle Mix Manual
    // @Description: Throttle vs attitude control prioritisation used during manual flight (higher values mean we prioritise attitude control over throttle)
    // @Range: 0.1 0.9
    // @User: Advanced
    AP_GROUPINFO("THR_MIX_MAN", 6, AC_AttitudeControl_Multi, _thr_mix_man, AC_ATTITUDE_CONTROL_MAN_DEFAULT),

    //Param name total length must smaller than 16

    // @Param: USE_INDI
    // @DisplayName: USE_INDI
    // @Description: Enable INDI feedforward 0 means disable
    //          1 means enable
    //          2 means start estimation but disable compensation
    // @Range: 0 1 2
    // @User: Advanced 
    AP_GROUPINFO("USE_INDI", 7, AC_AttitudeControl_Multi, _use_indi, AC_ATC_USE_INDI),

    // @Param: MIN_INDI_SPEED
    // @DisplayName: MIN_INDI_SPEED
    // @Description: Minimum speed to enable INDI
    // @Range: 100.0-60000.0
    // @User: Advanced
    AP_GROUPINFO("MIN_INDI_SPD", 8, AC_AttitudeControl_Multi, _MIN_INDI_SPEED, AC_ATC_MIN_INDI_SPEED),

    // @Param: INDI_KF
    // @DisplayName: INDI_KF
    // @Description: RPM to Thrust
    // @Range: (value*1.0E-9.0)
    // @User: Advanced
    AP_GROUPINFO("INDI_KF", 9, AC_AttitudeControl_Multi, _INDI_KF, AC_ATC_INDI_KF),

    // @Param: INDI_KM
    // @DisplayName: INDI_KM
    // @Description: RPM to moment (RPM to yaw acceleration)
    // @Range: (value*1.0E-9.0)
    // @User: Advanced
    AP_GROUPINFO("INDI_KM", 10, AC_AttitudeControl_Multi, _INDI_KM, AC_ATC_INDI_KM),

    // @Param: PROPELLER_INERTIA
    // @DisplayName: PROPELLER_INERTIA
    // @Description: PROPELLER_INERTIA 
    // @Range: (value*1.0E-9.0)
    // @User: Advanced
    AP_GROUPINFO("PROP_INERTIA", 11, AC_AttitudeControl_Multi, _PROPELLER_INERTIA, AC_ATC_PROPELLER_INERTIA),
    
    // @Param: CUT_OFF_RPM_FILTER
    // @DisplayName: CUTOFF_RPM
    // @Description: CUT_OFF_RPM_FILTER
    // @Range: 0-Sample_rate_rpm/2
    // @User: Advanced
    AP_GROUPINFO("CUTOFF_RPM", 12, AC_AttitudeControl_Multi, _CUT_OFF_RPM_FILTER, AC_ATC_CUT_OFF_RPM_FILTER),   
    // @Param: CUT_OFF_MOMENT_FILTER
    // @DisplayName: CUTOFF_MOMENT
    // @Description: CUT_OFF_MOMENT_FILTER
    // @Range: 0-controller_rate/2
    // @User: Advanced
    AP_GROUPINFO("CUTOFF_MT", 13, AC_AttitudeControl_Multi, _CUT_OFF_MOMENT_FILTER, AC_ATC_CUT_OFF_MOMENT_FILTER),  

    // @Param: MAX_MOMENT_XY
    // @DisplayName: MAX_MOMENT_XY
    // @Description: MAX_MOMENT_XY of INDI
    // @Range: 0~1
    // @User: Advanced
    AP_GROUPINFO("MAX_MT_XY", 14, AC_AttitudeControl_Multi, _MAX_MOMENT_XY, AC_ATC_MAX_MOMENT_XY),
    // @Param: MAX_MOMENT_Z
    // @DisplayName: MAX_MOMENT_Z
    // @Description: MAX_MOMENT_Z of INDI
    // @Range: 0~1
    // @User: Advanced
    AP_GROUPINFO("MAX_MT_Z", 15, AC_AttitudeControl_Multi, _MAX_MOMENT_Z, AC_ATC_MAX_MOMENT_Z),

    // @Param: INERTIA of Copter
    // @DisplayName: Copter_INERTIA
    // @Description:  INERTIA of Copter
    // @Range: (value*1.0E-3.0)
    // @User: Advanced
    AP_GROUPINFO("COP_IXX", 16, AC_AttitudeControl_Multi, _qinertia.Ixx, AC_ATC_QINERTIA_IXX),
    AP_GROUPINFO("COP_IXY", 17, AC_AttitudeControl_Multi, _qinertia.Ixy, AC_ATC_QINERTIA_IXY),
    AP_GROUPINFO("COP_IXZ", 18, AC_AttitudeControl_Multi, _qinertia.Ixz, AC_ATC_QINERTIA_IXZ),
    AP_GROUPINFO("COP_IYX", 19, AC_AttitudeControl_Multi, _qinertia.Iyx, AC_ATC_QINERTIA_IYX),
    AP_GROUPINFO("COP_IYY", 20, AC_AttitudeControl_Multi, _qinertia.Iyy, AC_ATC_QINERTIA_IYY),
    AP_GROUPINFO("COP_IYZ", 21, AC_AttitudeControl_Multi, _qinertia.Iyz, AC_ATC_QINERTIA_IYZ),    
    AP_GROUPINFO("COP_IZX", 22, AC_AttitudeControl_Multi, _qinertia.Izx, AC_ATC_QINERTIA_IZX),
    AP_GROUPINFO("COP_IZY", 23, AC_AttitudeControl_Multi, _qinertia.Izy, AC_ATC_QINERTIA_IZY),
    AP_GROUPINFO("COP_IZZ", 24, AC_AttitudeControl_Multi, _qinertia.Izz, AC_ATC_QINERTIA_IZZ), 
    
    // @Param: INDI_ARM_S
    // @DisplayName: INDI_ARM_S
    // @Description:  if QUAD/X INDI_ARM_S=sqrt(2)*arm length. if if QUAD/PLUS INDI_ARM_S=2*arm length
    // @Range: 0~1
    // @User: Advanced  
    AP_GROUPINFO("INDI_ARM_S", 25, AC_AttitudeControl_Multi, _ARM_SCALE, AC_ATC_ARM_SCALE),   
    
    // @Param: INDI_K_XY
    // @DisplayName: INDI_K_XY
    // @Description:  INDI_K_XY(Scale on roll and pitch)
    // @Range: 0~1
    // @User: Advanced  
    AP_GROUPINFO("INDI_K_XY", 26, AC_AttitudeControl_Multi, _INDI_K_XY, AC_ATC_INDI_SCALE),   

    // @Param: INDI_K_Z
    // @DisplayName: INDI_K_Z
    // @Description:  INDI_K_Z (Scale on yaw)
    // @Range: 0~1
    // @User: Advanced  
    AP_GROUPINFO("INDI_K_Z", 27, AC_AttitudeControl_Multi, _INDI_K_Z, AC_ATC_INDI_SCALE),
    
    AP_GROUPINFO("WACC_CUTOFF", 28, AC_AttitudeControl_Multi, _ANGACC_CUTOFF, AC_ATC_ANGACC_CUTOFF),

    AP_GROUPINFO("DRPM_CUTOFF", 29, AC_AttitudeControl_Multi, _DRPM_CUTOFF, AC_ATC_DRPM_CUTOFF),
    
    // @Param: INDI_BIAS_{X,Y,Z}
    // @DisplayName: INDI_BIAS_{X,Y,Z}
    // @Description:  Minus a bias on X Y Z(*1.0E-3).
    // @Range: 0~1
    // @User: Advanced  
    AP_GROUPINFO("INDI_BIAS_X", 30, AC_AttitudeControl_Multi, _INDI_BIAS_X, 0.0f), 
    AP_GROUPINFO("INDI_BIAS_Y", 31, AC_AttitudeControl_Multi, _INDI_BIAS_Y, 0.0f), 
    AP_GROUPINFO("INDI_BIAS_Z", 32, AC_AttitudeControl_Multi, _INDI_BIAS_Z, 0.0f), 
    AP_GROUPEND
};

AC_AttitudeControl_Multi::AC_AttitudeControl_Multi(AP_AHRS_View &ahrs, const AP_Vehicle::MultiCopter &aparm, AP_MotorsMulticopter& motors, float dt) :
    AC_AttitudeControl(ahrs, aparm, motors, dt),
    _motors_multi(motors),
    _pid_rate_roll(AC_ATC_MULTI_RATE_RP_P, AC_ATC_MULTI_RATE_RP_I, AC_ATC_MULTI_RATE_RP_D, 0.0f, AC_ATC_MULTI_RATE_RP_IMAX, AC_ATC_MULTI_RATE_RP_FILT_HZ, 0.0f, AC_ATC_MULTI_RATE_RP_FILT_HZ, dt),
    _pid_rate_pitch(AC_ATC_MULTI_RATE_RP_P, AC_ATC_MULTI_RATE_RP_I, AC_ATC_MULTI_RATE_RP_D, 0.0f, AC_ATC_MULTI_RATE_RP_IMAX, AC_ATC_MULTI_RATE_RP_FILT_HZ, 0.0f, AC_ATC_MULTI_RATE_RP_FILT_HZ, dt),
    _pid_rate_yaw(AC_ATC_MULTI_RATE_YAW_P, AC_ATC_MULTI_RATE_YAW_I, AC_ATC_MULTI_RATE_YAW_D, 0.0f, AC_ATC_MULTI_RATE_YAW_IMAX, AC_ATC_MULTI_RATE_RP_FILT_HZ, AC_ATC_MULTI_RATE_YAW_FILT_HZ, 0.0f, dt)
{
    AP_Param::setup_object_defaults(this, var_info);
    init_indi();
}

// Update Alt_Hold angle maximum
void AC_AttitudeControl_Multi::update_althold_lean_angle_max(float throttle_in)
{
    // calc maximum tilt angle based on throttle
    float thr_max = _motors_multi.get_throttle_thrust_max();

    // divide by zero check
    if (is_zero(thr_max)) {
        _althold_lean_angle_max = 0.0f;
        return;
    }

    float althold_lean_angle_max = acosf(constrain_float(throttle_in / (AC_ATTITUDE_CONTROL_ANGLE_LIMIT_THROTTLE_MAX * thr_max), 0.0f, 1.0f));
    _althold_lean_angle_max = _althold_lean_angle_max + (_dt / (_dt + _angle_limit_tc)) * (althold_lean_angle_max - _althold_lean_angle_max);
}

void AC_AttitudeControl_Multi::set_throttle_out(float throttle_in, bool apply_angle_boost, float filter_cutoff)
{
    _throttle_in = throttle_in;
    update_althold_lean_angle_max(throttle_in);
    _motors.set_throttle_filter_cutoff(filter_cutoff);
    if (apply_angle_boost) {
        // Apply angle boost
        throttle_in = get_throttle_boosted(throttle_in);
    } else {
        // Clear angle_boost for logging purposes
        _angle_boost = 0.0f;
    }
    _motors.set_throttle(throttle_in);
    _motors.set_throttle_avg_max(get_throttle_avg_max(MAX(throttle_in, _throttle_in)));
}

void AC_AttitudeControl_Multi::set_throttle_mix_max(float ratio)
{
    ratio = constrain_float(ratio, 0.0f, 1.0f);
    _throttle_rpy_mix_desired = (1.0f - ratio) * _thr_mix_min + ratio * _thr_mix_max;
}

// returns a throttle including compensation for roll/pitch angle
// throttle value should be 0 ~ 1
float AC_AttitudeControl_Multi::get_throttle_boosted(float throttle_in)
{
    if (!_angle_boost_enabled) {
        _angle_boost = 0;
        return throttle_in;
    }
    // inverted_factor is 1 for tilt angles below 60 degrees
    // inverted_factor reduces from 1 to 0 for tilt angles between 60 and 90 degrees

    float cos_tilt = _ahrs.cos_pitch() * _ahrs.cos_roll();
    float inverted_factor = constrain_float(10.0f * cos_tilt, 0.0f, 1.0f);
    float cos_tilt_target = cosf(_thrust_angle);
    float boost_factor = 1.0f / constrain_float(cos_tilt_target, 0.1f, 1.0f);

    float throttle_out = throttle_in * inverted_factor * boost_factor;
    _angle_boost = constrain_float(throttle_out - throttle_in, -1.0f, 1.0f);
    return throttle_out;
}

// returns a throttle including compensation for roll/pitch angle
// throttle value should be 0 ~ 1
float AC_AttitudeControl_Multi::get_throttle_avg_max(float throttle_in)
{
    throttle_in = constrain_float(throttle_in, 0.0f, 1.0f);
    return MAX(throttle_in, throttle_in * MAX(0.0f, 1.0f - _throttle_rpy_mix) + _motors.get_throttle_hover() * _throttle_rpy_mix);
}

// update_throttle_rpy_mix - slew set_throttle_rpy_mix to requested value
void AC_AttitudeControl_Multi::update_throttle_rpy_mix()
{
    // slew _throttle_rpy_mix to _throttle_rpy_mix_desired
    if (_throttle_rpy_mix < _throttle_rpy_mix_desired) {
        // increase quickly (i.e. from 0.1 to 0.9 in 0.4 seconds)
        _throttle_rpy_mix += MIN(2.0f * _dt, _throttle_rpy_mix_desired - _throttle_rpy_mix);
    } else if (_throttle_rpy_mix > _throttle_rpy_mix_desired) {
        // reduce more slowly (from 0.9 to 0.1 in 1.6 seconds)
        _throttle_rpy_mix -= MIN(0.5f * _dt, _throttle_rpy_mix - _throttle_rpy_mix_desired);

        // if the mix is still higher than that being used, reset immediately
        const float throttle_hover = _motors.get_throttle_hover();
        const float throttle_in = _motors.get_throttle();
        const float throttle_out = MAX(_motors.get_throttle_out(), throttle_in);
        float mix_used;
        // since throttle_out >= throttle_in at this point we don't need to check throttle_in < throttle_hover
        if (throttle_out < throttle_hover) {
            mix_used = (throttle_out - throttle_in) / (throttle_hover - throttle_in);
        } else {
            mix_used = throttle_out / throttle_hover;
        }

        _throttle_rpy_mix = MIN(_throttle_rpy_mix, MAX(mix_used, _throttle_rpy_mix_desired));
    }
    _throttle_rpy_mix = constrain_float(_throttle_rpy_mix, 0.1f, AC_ATTITUDE_CONTROL_MAX);
}

void AC_AttitudeControl_Multi::init_indi()
{
    _rpm_filter.set_cutoff_frequency(1.0f/_dt,_CUT_OFF_RPM_FILTER);
    _moment_filter.set_cutoff_frequency(1.0f/_dt,_CUT_OFF_MOMENT_FILTER);
    _drpm_filter.set_cutoff_frequency(1.0f/_dt,_DRPM_CUTOFF);
    _ang_acc_filter.set_cutoff_frequency(1.0f/_dt,_ANGACC_CUTOFF);
}

void AC_AttitudeControl_Multi::update_indi_moment()
{
    if (_use_indi != 1 && _use_indi != 2)
    {
        return;
    }
    
    //step 0 update angular acceleration
    Vector3f gyro_indi = _ahrs.get_gyro_latest();
    uint32_t gyro_time = AP::ins().get_singleton()->get_last_update_usec();
    ang_acc_derivativtor.update(gyro_indi,gyro_time);

    Vector3f _angular_acceleration_d = ang_acc_derivativtor.get_derivative();
    _angular_acceleration = _ang_acc_filter.apply(_angular_acceleration_d);

    //Step 1 Update RPM data 
    VectorN<float,4> rpm_current;
    const uint32_t now = AP_HAL::micros();
    AP::esc_telem().get_rpm(0,rpm_current[0]);
    AP::esc_telem().get_rpm(1,rpm_current[1]);
    AP::esc_telem().get_rpm(2,rpm_current[2]);
    AP::esc_telem().get_rpm(3,rpm_current[3]);
    _rpm_filtered = _rpm_filter.apply(rpm_current);
    // if (_use_indi == 1)
    // {
        if (_rpm_filtered[0] < _MIN_INDI_SPEED ||
            _rpm_filtered[1] < _MIN_INDI_SPEED ||
            _rpm_filtered[2] < _MIN_INDI_SPEED ||
            _rpm_filtered[3] < _MIN_INDI_SPEED) //
        {
            _compensation_moment.zero();
            return;
        }
    // }

    rpm_derivativtor.update(_rpm_filtered,now);
    auto _delta_rpm_d = rpm_derivativtor.get_derivative();
    _delta_rpm = _drpm_filter.apply(_delta_rpm_d);

    ////Step 2 Update moment from inverse dynamic
    Vector3f inverse_moment;
    inverse_moment.zero();
    for (int i = 0; i < 4; i++)
    {
        inverse_moment.x += _ARM_SCALE*_INDI_KF*1.0e-9f*_rpm_filtered[i]*_rpm_filtered[i]*_motors.get_roll_factor(i);
        inverse_moment.y += _ARM_SCALE*_INDI_KF*1.0e-9f*_rpm_filtered[i]*_rpm_filtered[i]*_motors.get_pitch_factor(i);
        inverse_moment.z += 1.0e-9f*_motors.get_yaw_factor(i)*(_INDI_KM*_rpm_filtered[i]*_rpm_filtered[i] + _PROPELLER_INERTIA*_delta_rpm[i]);
    }
    ////Step 3 Update moment from gyro
    Vector3f moment;
    moment.x = (_angular_acceleration.x * _qinertia.Ixx + _angular_acceleration.y*_qinertia.Ixy + _angular_acceleration.z * _qinertia.Ixz)*1.0e-3f;
    moment.y = (_angular_acceleration.x * _qinertia.Iyx + _angular_acceleration.y*_qinertia.Iyy + _angular_acceleration.z * _qinertia.Iyz)*1.0e-3f;
    moment.z = (_angular_acceleration.x * _qinertia.Izx + _angular_acceleration.y*_qinertia.Izy + _angular_acceleration.z * _qinertia.Izz)*1.0e-3f;
    ////Step 4 Compute the compensation_moment
    _compensation_moment_raw = ( inverse_moment - moment);
    _compensation_moment_raw.x = _compensation_moment_raw.x*_INDI_K_XY - _INDI_BIAS_X*1.0e-3;
    _compensation_moment_raw.y = _compensation_moment_raw.y*_INDI_K_XY - _INDI_BIAS_Y*1.0e-3;
    _compensation_moment_raw.z = _compensation_moment_raw.z*_INDI_K_Z - _INDI_BIAS_Z*1.0e-3;;
    _compensation_moment_raw.x = constrain_float(_compensation_moment_raw.x ,-_MAX_MOMENT_XY,_MAX_MOMENT_XY);
    _compensation_moment_raw.y = constrain_float(_compensation_moment_raw.y ,-_MAX_MOMENT_XY,_MAX_MOMENT_XY);
    _compensation_moment_raw.z = constrain_float(_compensation_moment_raw.z ,-_MAX_MOMENT_Z,_MAX_MOMENT_Z);
    _compensation_moment=_moment_filter.apply(_compensation_moment_raw);
    
}

void AC_AttitudeControl_Multi::rate_controller_run()
{
    // move throttle vs attitude mixing towards desired (called from here because this is conveniently called on every iteration)
    update_throttle_rpy_mix();

    _ang_vel_body += _sysid_ang_vel_body;
    
    update_indi_moment();
    Vector3f gyro_latest = _ahrs.get_gyro_latest();

    _motors.set_roll(get_rate_roll_pid().update_all(_ang_vel_body.x, gyro_latest.x, _motors.limit.roll) + _actuator_sysid.x);

    _motors.set_pitch(get_rate_pitch_pid().update_all(_ang_vel_body.y, gyro_latest.y, _motors.limit.pitch) + _actuator_sysid.y);

    _motors.set_yaw(get_rate_yaw_pid().update_all(_ang_vel_body.z, gyro_latest.z, _motors.limit.yaw) + _actuator_sysid.z);
    
    if (_use_indi == 1)
    {
        _motors.set_roll_ff(get_rate_roll_pid().get_ff() + _compensation_moment.x);
        _motors.set_pitch_ff(get_rate_pitch_pid().get_ff()+_compensation_moment.y);
        _motors.set_yaw_ff(get_rate_yaw_pid().get_ff()*_feedforward_scalar+_compensation_moment.z);
    }
    else
    {
        _motors.set_roll_ff(get_rate_roll_pid().get_ff());
        _motors.set_pitch_ff(get_rate_pitch_pid().get_ff());
        _motors.set_yaw_ff(get_rate_yaw_pid().get_ff()*_feedforward_scalar);
    }

    _sysid_ang_vel_body.zero();
    _actuator_sysid.zero();

    control_monitor_update();
}

// sanity check parameters.  should be called once before takeoff
void AC_AttitudeControl_Multi::parameter_sanity_check()
{
    // sanity check throttle mix parameters
    if (_thr_mix_man < 0.1f || _thr_mix_man > AC_ATTITUDE_CONTROL_MAN_LIMIT) {
        // parameter description recommends thr-mix-man be no higher than 0.9 but we allow up to 4.0
        // which can be useful for very high powered copters with very low hover throttle
        _thr_mix_man.set_and_save(constrain_float(_thr_mix_man, 0.1, AC_ATTITUDE_CONTROL_MAN_LIMIT));
    }
    if (_thr_mix_min < 0.1f || _thr_mix_min > AC_ATTITUDE_CONTROL_MIN_LIMIT) {
        _thr_mix_min.set_and_save(constrain_float(_thr_mix_min, 0.1, AC_ATTITUDE_CONTROL_MIN_LIMIT));
    }
    if (_thr_mix_max < 0.5f || _thr_mix_max > AC_ATTITUDE_CONTROL_MAX) {
        // parameter description recommends thr-mix-max be no higher than 0.9 but we allow up to 5.0
        // which can be useful for very high powered copters with very low hover throttle
        _thr_mix_max.set_and_save(constrain_float(_thr_mix_max, 0.5, AC_ATTITUDE_CONTROL_MAX));
    }
    if (_thr_mix_min > _thr_mix_max) {
        _thr_mix_min.set_and_save(AC_ATTITUDE_CONTROL_MIN_DEFAULT);
        _thr_mix_max.set_and_save(AC_ATTITUDE_CONTROL_MAX_DEFAULT);
    }
}
