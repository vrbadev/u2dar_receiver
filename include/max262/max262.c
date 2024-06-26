#include "max262.h"

#include <math.h>
#include <stdio.h>
#include <time.h>
#include <errno.h>    

int msleep(long msec)
{
    struct timespec ts;
    int res;

    if (msec < 0)
    {
        errno = EINVAL;
        return -1;
    }

    ts.tv_sec = msec / 1000;
    ts.tv_nsec = (msec % 1000) * 1000000;

    do {
        res = nanosleep(&ts, &ts);
    } while (res && errno == EINTR);

    return res;
}

/*static const float max262_fclkf0_ratios[64][2] = {{40.84, 28.88}, {42.41, 29.99}, {43.98, 31.10}, {45.55, 32.21}, {47.12, 33.32}, {48.69, 34.43}, {50.27, 35.54}, {51.84, 36.65}, {53.41, 37.76}, {54.98, 38.87}, {56.55, 39.99}, {58.12, 41.10}, {59.69, 42.21}, {61.26, 43.32}, {62.83, 44.43}, {64.40, 45.54}, {65.97, 46.65}, {67.54, 47.76}, {69.12, 48.87}, {70.69, 49.98}, {72.26, 51.10}, {73.83, 52.20}, {75.40, 53.31}, {76.97, 54.43}, {78.53, 55.54}, {80.11, 56.65}, {81.68, 57.76}, {83.25, 58.87}, {84.82, 59.98}, {86.39, 61.09}, {87.96, 62.20}, {89.54, 63.31}, {91.11, 64.42}, {92.68, 65.53}, {94.25, 66.64}, {95.82, 67.75}, {97.39, 68.86}, {98.96, 69.98}, {100.53, 71.09}, {102.10, 72.20}, {102.67, 73.31}, {105.24, 74.42}, {106.81, 75.53}, {108.38, 76.64}, {109.96, 77.75}, {111.53, 78.86}, {113.10, 79.97}, {114.66, 81.08}, {11624, 82.19}, {117.81, 83.30}, {119.38, 84.41}, {120.95, 85.53}, {122.52, 86.64}, {124.09, 87.75}, {125.66, 88.86}, {127.23, 89.97}, {128.81, 91.08}, {130.38, 92.19}, {131.95, 93.30}, {133.52, 94.41}, {135.09, 95.52}, {136.66, 96.63}, {138.23, 97.74}, {139.80, 98.85}};

static const float max262_prog_q[128][2] = {{0.500, 0.707}, {0.504, 0.713}, {0.508, 0.718}, {0.512, 0.724}, {0.516, 0.730}, {0.520, 0.736}, {0.525, 0.742}, {0.529, 0.748}, {0.533, 0.754}, {0.538, 0.761}, {0.542, 0.767}, {0.547, 0.774}, {0.552, 0.780}, {0.556, 0.787}, {0.561, 0.794}, {0.566, 0.801}, {0.571, 0.808}, {0.577, 0.815}, {0.582, 0.823}, {0.587, 0.830}, {0.593, 0.838}, {0.598, 0.646}, {0.604, 0.854}, {0.609, 0.862}, {0.615, 0.870}, {0.621, 0.879}, {0.627, 0.887}, {0.634, 0.896}, {0.640, 0.905}, {0.646, 0.914}, {0.653, 0.924}, {0.660, 0.933}, {0.667, 0.943}, {0.674, 0.953}, {0.681, 0.963}, {0.688, 0.973}, {0.696, 0.984}, {0.703, 0.995}, {0.711, 1.01}, {0.719, 1.02}, {0.727, 1.03}, {0.736, 1.04}, {0.744, 1.05}, {0.753, 1.06}, {0.762, 1.08}, {0.771, 1.09}, {0.780, 1.10}, {0.790, 1.12}, {0.800, 1.13}, {0.810, 1.15}, {0.821, 1.16}, {0.831, 1.18}, {0.842, 1.19}, {0.853, 1.21}, {0.865, 1.22}, {0.877, 1.24}, {0.889, 1.26}, {0.901, 1.27}, {0.914, 1.29}, {0.928, 1.31}, {0.941, 1.33}, {0.955, 1.35}, {0.969, 1.37}, {0.985, 1.39}, {1.00, 1.41}, {1.02, 1.44}, {1.03, 1.46}, {1.05, 1.48}, {1.07, 1.51}, {1.08, 1.53}, {1.10, 1.56}, {1.12, 1.59}, {1.14, 1.62}, {1.16, 1.65}, {1.19, 1.68}, {1.21, 1.71}, {1.23, 1.74}, {1.25, 1.77}, {1.28, 1.81}, {1.31, 1.85}, {1.33, 1.89}, {1.36, 1.93}, {1.39, 1.97}, {1.42, 2.01}, {1.45, 2.06}, {1.49, 2.10}, {1.52, 2.16}, {1.56, 2.21}, {1.60, 2.26}, {1.64, 2.32}, {1.68, 2.40}, {1.73, 2.45}, {1.78, 2.51}, {1.83, 2.59}, {1.88, 2.66}, {1.94, 2.74}, {2.00, 2.83}, {2.06, 2.92}, {2.13, 3.02}, {2.21, 3.12}, {2.29, 3.23}, {2.37, 3.35}, {2.46, 3.48}, {2.56, 3.62}, {2.67, 3.77}, {2.78, 3.96}, {2.91, 4.11}, {3.05, 4.31}, {3.20, 4.53}, {3.37, 4.76}, {3.56, 5.03}, {3.76, 5.32}, {4.00, 5.66}, {4.27, 6.03}, {4.57, 6.46}, {4.92, 6.96}, {5.33, 7.54}, {5.82, 8.23}, {6.40, 9.05}, {7.11, 10.1}, {8.00, 11.3}, {9.14, 12.9}, {10.7, 15.1}, {12.8, 18.1}, {16.0, 22.6}, {21.3, 30.2}, {32.0, 45.3}, {64.0, 90.5}};*/

typedef struct {
    uint8_t N_F;
    uint8_t N_Q;
    uint8_t mode;
} max262_config_t;

max262_config_t max262_closest_conf(double fclk, double f0, double Q, uint8_t mode)
{
    max262_config_t res = { .N_F = 0xFF, .N_Q = 0xFF, .mode = mode };

    double N;
    double d = (fclk / f0) / (M_PI / 2);
    double d2 = 64 / Q;
    if (mode != 2) {
        N = round(d - 26);
        if (N >= 0 && N <= 63) {
            res.N_F = (uint8_t) N;
        }
        N = round(128 - d2);
        if (N >= 0 && N <= 127) {
            res.N_Q = (uint8_t) N;
        }
    } else {
        N = round((d / sqrt(2.0)) - 26);
        if (N >= 0 && N <= 63) {
            res.N_F = (uint8_t) N;
        } 
        N = round(128 - (sqrt(2.0) * d2));
        if (N >= 0 && N <= 127) {
            res.N_Q = (uint8_t) N;
        }
    }

    return res;
}

void rio_write(max262_t* handle, uint32_t gpio, uint8_t val) 
{
    if (val) {
        rp1_sys_rio_out_set(handle->rp1_handle, gpio);
    } else {
        rp1_sys_rio_out_clr(handle->rp1_handle, gpio);
    }
}

void max262_write(max262_t* handle, uint8_t d0, uint8_t d1, uint8_t A)
{
    for (uint8_t i = 0; i < 4; i++) {
        rio_write(handle, handle->gpios_a[i], A & (1 << i));
    }
    rio_write(handle, handle->gpios_d[0], d0);
    rio_write(handle, handle->gpios_d[1], d1);
}

void max262_set_conf(max262_t* handle, max262_config_t* conf, uint8_t filt_sel)
{
    uint8_t mode = conf->mode - 1;
    uint8_t d_pairs[8][2] = {
        {mode & (1 << 0), mode & (1 << 1)},
        {conf->N_F & (1 << 0), conf->N_F & (1 << 1)},
        {conf->N_F & (1 << 2), conf->N_F & (1 << 3)},
        {conf->N_F & (1 << 4), conf->N_F & (1 << 5)},
        {conf->N_Q & (1 << 0), conf->N_Q & (1 << 1)},
        {conf->N_Q & (1 << 2), conf->N_Q & (1 << 3)},
        {conf->N_Q & (1 << 4), conf->N_Q & (1 << 5)},
        {conf->N_Q & (1 << 6), 0},
    };

    for (uint8_t A = 0; A < 8; A++) {
        max262_write(handle, d_pairs[A][0], d_pairs[A][1], A | (filt_sel << 3));
        rp1_sys_rio_out_clr(handle->rp1_handle, handle->gpio_nwr);
        msleep(10);
        rp1_sys_rio_out_set(handle->rp1_handle, handle->gpio_nwr);
        msleep(10);
    }
}


int max262_init(max262_t* handle)
{
    for (uint32_t i = 0; i < 4; i++) {
        rp1_gpio_funcsel(handle->rp1_handle, handle->gpios_a[i], 5);
        rp1_sys_rio_config_output(handle->rp1_handle, handle->gpios_a[i]);
        rp1_sys_rio_out_clr(handle->rp1_handle, handle->gpios_a[i]);
    }

    for (uint32_t i = 0; i < 2; i++) {
        rp1_gpio_funcsel(handle->rp1_handle, handle->gpios_d[i], 5);
        rp1_sys_rio_config_output(handle->rp1_handle, handle->gpios_d[i]);
        rp1_sys_rio_out_clr(handle->rp1_handle, handle->gpios_d[i]);
    }

    for (uint32_t i = 0; i < 2; i++) {
        rp1_gpio_funcsel(handle->rp1_handle, handle->gpios_clk[i], 0);
        rp1_gpio_config_nopull(handle->rp1_handle, handle->gpios_clk[i]);
    }

    rp1_gpio_funcsel(handle->rp1_handle, handle->gpio_nwr, 5);
    rp1_sys_rio_config_output(handle->rp1_handle, handle->gpio_nwr);
    rp1_sys_rio_out_set(handle->rp1_handle, handle->gpio_nwr);

    // set PWM0_CH0 to 1.5MHz (PWM0 runs at 30MHz)
    rp1_pwm_chan_config(handle->rp1_handle->pwm0, 0, 1, 20, 0, 10);
    rp1_pwm_chan_enable(handle->rp1_handle->pwm0, 0, 1);

    // set PWM0_CH1 to 1.5MHz (PWM0 runs at 30MHz)
    rp1_pwm_chan_config(handle->rp1_handle->pwm0, 1, 1, 20, 0, 10);
    rp1_pwm_chan_enable(handle->rp1_handle->pwm0, 1, 1);

    max262_config_t conf_a = max262_closest_conf(1.5e6, handle->swcf_a_f0, handle->swcf_a_q, handle->swcf_a_mode);
    max262_config_t conf_b = max262_closest_conf(1.5e6, handle->swcf_b_f0, handle->swcf_b_q, handle->swcf_b_mode);

    if (conf_a.N_F == 0xFF) {
        fprintf(stderr, "[WARN] Failed to set MAX262 (ch A) N value for mode %d and center freq = %f\n", handle->swcf_a_mode, handle->swcf_a_f0);
    } else if (conf_a.N_Q == 0xFF) {
        fprintf(stderr, "[WARN] Failed to set MAX262 (ch A) N value for mode %d and Q = %f\n", handle->swcf_a_mode, handle->swcf_a_q);
    } else {
        max262_set_conf(handle, &conf_a, 0);
        fprintf(stderr, "[WARN] MAX262 (ch A) configured for mode %d: f0 = %f Hz, Q = %f\n", handle->swcf_a_mode, handle->swcf_a_f0, handle->swcf_a_q);
    }

    if (conf_b.N_F == 0xFF) {
        fprintf(stderr, "[WARN] Failed to set MAX262 (ch B) N value for mode %d and center freq = %f\n", handle->swcf_b_mode, handle->swcf_b_f0);
    } else if (conf_b.N_Q == 0xFF) {
        fprintf(stderr, "[WARN] Failed to set MAX262 (ch B) N value for mode %d and Q = %f\n", handle->swcf_b_mode, handle->swcf_b_q);
    } else {
        max262_set_conf(handle, &conf_b, 1);
        fprintf(stderr, "[WARN] MAX262 (ch B) configured for mode %d: f0 = %f Hz, Q = %f\n", handle->swcf_b_mode, handle->swcf_b_f0, handle->swcf_b_q);
    }
}

void max262_deinit(max262_t* handle)
{
    rp1_pwm_chan_enable(handle->rp1_handle->pwm0, 0, 0);
    rp1_pwm_chan_enable(handle->rp1_handle->pwm0, 1, 0);
}
