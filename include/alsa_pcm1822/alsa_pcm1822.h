#ifndef ALSA_PCM1822_H_
#define ALSA_PCM1822_H_

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <alsa/asoundlib.h>
#include <pthread.h>
#include <time.h>
#include <unistd.h>
#include "../rpi5_periph/rp1.h"

#define NUM_CHANNELS 2
#define PCM_FORMAT SND_PCM_FORMAT_S32_LE

typedef struct {
    const char* dev_name;
    uint32_t sample_rate;
    uint32_t buffer_frames_num;
    rp1_t* rp1_handle;
    uint32_t gpio_sync;
    uint32_t gpio_pcm_ws;
    struct timespec stamp_sync;
    snd_pcm_t *capture_handle;
    snd_pcm_status_t* capture_status;
    int32_t* buffer;
    uint32_t buffer_size;
    snd_htimestamp_t hstamp;
    snd_htimestamp_t hstamp_trigger;
    snd_htimestamp_t hstamp_audio;
    snd_htimestamp_t hstamp_driver;
    snd_pcm_audio_tstamp_report_t hstamp_report;
    snd_pcm_uframes_t frames_avail;
    snd_pcm_sframes_t frames_delay;
} alsa_pcm1822_t;


int alsa_pcm1822_init(alsa_pcm1822_t* handle);
void alsa_pcm1822_deinit(alsa_pcm1822_t* handle);
int alsa_pcm1822_read(alsa_pcm1822_t* handle);

#endif
