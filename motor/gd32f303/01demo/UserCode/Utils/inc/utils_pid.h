#ifndef __UTILS_PID_H__
#define __UTILS_PID_H__

#define INTEGRAL_MAX  5000

typedef struct {
    float target_value;
    float actual_value;
    float err;
    float err_last;
    float kp,ki,kd;
    float integral;
    float output_value;
}utils_pid_t;

void utils_pid_param_init(utils_pid_t *_pid, utils_pid_t _pid_para_config);
void utils_pid_set_target_value(utils_pid_t *_pid, float _target_value);
float utils_pid_get_target_value(utils_pid_t _pid);
void utils_pid_set_pid_value(utils_pid_t *_pid, float _kp, float _ki, float _kd);
float utils_pid_realize(utils_pid_t *_pid, float _actual);

#endif /* __UTILS_PID_H__ */

