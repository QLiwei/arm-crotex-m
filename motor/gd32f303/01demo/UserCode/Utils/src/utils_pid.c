#include "utils_pid.h"

#include "protocol.h"

void utils_pid_param_init(utils_pid_t *_pid, utils_pid_t _pid_para_config) {
    _pid->target_value = _pid_para_config.target_value;
    _pid->kp = _pid_para_config.kp;
    _pid->ki = _pid_para_config.ki;
    _pid->kd = _pid_para_config.kd;
    _pid->actual_value = 0.0f;
    _pid->err = 0.0f;
    _pid->err_last = 0.0f;
    _pid->integral = 0.0f;
    _pid->output_value = 0.0f;
}

void utils_pid_set_target_value(utils_pid_t *_pid, float _target_value) {
    _pid->target_value = _target_value;
}

float utils_pid_get_target_value(utils_pid_t _pid) {
    return _pid.target_value;
}

void utils_pid_set_pid_value(utils_pid_t *_pid, float _kp, float _ki, float _kd) {
    _pid->kp = _kp;
    _pid->ki = _ki;
    _pid->kd = _kd;
}

float utils_pid_realize(utils_pid_t *_pid, float _actual) {
    _pid->actual_value = _actual;
    _pid->err = _pid->target_value - _pid->actual_value;
    _pid->integral += _pid->err;
    _pid->integral = _pid->integral > INTEGRAL_MAX ? INTEGRAL_MAX : _pid->integral;
    _pid->output_value = _pid->kp * _pid->err + _pid->ki * _pid->integral + _pid->kd * (_pid->err - _pid->err_last);
    _pid->err_last = _pid->err;
    return _pid->output_value;
}



