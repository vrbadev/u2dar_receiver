#include "alsa_pcm1822.h"

/*
ALSA hw ranges:
    rate min: 8000
    rate max: 384000
    period time min: 10
    period time max: 128000
    period size min: 4
    period size max: 1024
    periods min: 2
    periods max: 32768
    buffer time min: 20
    buffer time max: 16384000
    buffer size min: 8
    buffer size max: 131072
*/

static snd_pcm_audio_tstamp_config_t audio_tstamp_config = { .type_requested = 0, .report_delay = 0 };

static pthread_t th;

void* alsa_sync_thread(void* ptr) 
{
    alsa_pcm1822_t* handle = (alsa_pcm1822_t*) ptr;

    uint32_t prev_state = rp1_sys_rio_in_get(handle->rp1_handle, handle->gpio_pcm_ws);
    uint32_t curr_state;
    uint32_t period = 2*handle->sample_rate;
    uint32_t counter = 0;
    while (1) {
        if ((curr_state = (handle->rp1_handle, handle->gpio_pcm_ws)) != prev_state) {
            if (counter == 0) {
                timespec_get(&handle->stamp_sync, TIME_UTC);
            }
            if (counter % period == 0) {
                rp1_sys_rio_out_xor(handle->rp1_handle, handle->gpio_sync);
            } 
            counter++;
        }
    }

    return NULL;
}


int alsa_pcm1822_init(alsa_pcm1822_t* handle)
{
    // sync out as RIO
    rp1_gpio_funcsel(handle->rp1_handle, handle->gpio_sync, 5);
    rp1_sys_rio_config_output(handle->rp1_handle, handle->gpio_sync);
    rp1_gpio_config_nopull(handle->rp1_handle, handle->gpio_sync);
    
    // sync in as RIO
    rp1_gpio_config_input(handle->rp1_handle, handle->gpio_pcm_ws);
    rp1_gpio_funcsel(handle->rp1_handle, handle->gpio_pcm_ws, 5);

    rp1_sys_rio_out_clr(handle->rp1_handle, handle->gpio_sync);
    pthread_create(&th, NULL, alsa_sync_thread, (void*) handle);

    snd_pcm_hw_params_t *hw_params;
    snd_pcm_sw_params_t *sw_params;

    int err;
    // open audio interface
    if ((err = snd_pcm_open(&handle->capture_handle, handle->dev_name, SND_PCM_STREAM_CAPTURE, 0)) < 0) {
        fprintf(stderr, "[ERROR] cannot open audio device %s (%s)\n", handle->dev_name, snd_strerror(err));
        return -1;
    }

    // setup hw params
    if ((err = snd_pcm_hw_params_malloc(&hw_params)) < 0) {
        fprintf(stderr, "[ERROR] cannot allocate hardware parameter structure (%s)\n", snd_strerror(err));
        return -2;
    }

    if ((err = snd_pcm_hw_params_any(handle->capture_handle, hw_params)) < 0) {
        fprintf(stderr, "[ERROR] cannot initialize hardware parameter structure (%s)\n", snd_strerror(err));
        return -3;
    }

    if ((err = snd_pcm_hw_params_set_access(handle->capture_handle, hw_params, SND_PCM_ACCESS_RW_INTERLEAVED)) < 0) {
        fprintf(stderr, "[ERROR] cannot set access type (%s)\n", snd_strerror(err));
        return -4;
    }

    if ((err = snd_pcm_hw_params_set_format(handle->capture_handle, hw_params, PCM_FORMAT)) < 0) {
        fprintf(stderr, "[ERROR] cannot set sample format (%s)\n", snd_strerror(err));
        return -5;
    }
    if ((err = snd_pcm_hw_params_set_rate(handle->capture_handle, hw_params, handle->sample_rate, 0)) < 0) {
        fprintf(stderr, "[ERROR] cannot set sample rate (%s)\n", snd_strerror(err));
        return -6;
    }

    if ((err = snd_pcm_hw_params_set_channels(handle->capture_handle, hw_params, NUM_CHANNELS)) < 0) {
        fprintf(stderr, "[ERROR] cannot set channel count (%s)\n", snd_strerror(err));
        return -7;
    }

    if ((err = snd_pcm_hw_params_set_periods(handle->capture_handle, hw_params, 64, 0)) < 0) {
        fprintf(stderr, "[ERROR] cannot set periods (%s)\n", snd_strerror(err));
        return -8;
    }

    snd_pcm_uframes_t period_size;
    snd_pcm_hw_params_get_period_size_max(hw_params, &period_size, 0);
    if ((err = snd_pcm_hw_params_set_period_size_last(handle->capture_handle, hw_params, &period_size, 0)) < 0) {
        fprintf(stderr, "[ERROR] cannot set period size (%s)\n", snd_strerror(err));
        return -8;
    }
    //fprintf(stderr, "[WARN] period size set to %u\n", period_size);

    unsigned long buffer_size;
    snd_pcm_hw_params_get_buffer_size_max(hw_params, &buffer_size);
    if ((err = snd_pcm_hw_params_set_buffer_size_near(handle->capture_handle, hw_params, &buffer_size)) < 0) {
        fprintf(stderr, "[ERROR] cannot set buffer size (%s)\n", snd_strerror(err));
        return -9;
    }
    //fprintf(stderr, "[WARN] buffer size set to %u\n", buffer_size);

    if ((err = snd_pcm_hw_params(handle->capture_handle, hw_params)) < 0) {
        fprintf(stderr, "[ERROR] cannot set hw_params (%s)\n", snd_strerror(err));
        return -10;
    }

    snd_pcm_hw_params_free(hw_params);

    // setup sw params
    if ((err = snd_pcm_sw_params_malloc(&sw_params)) < 0) {
        fprintf(stderr, "[ERROR] cannot allocate software parameter structure (%s)\n", snd_strerror(err));
        return -11;
    }

    if ((err = snd_pcm_sw_params_current(handle->capture_handle, sw_params)) < 0) {
        fprintf(stderr, "[ERROR] cannot initialize sw_params (%s)\n", snd_strerror(err));
        return -12;
    }

    if ((err = snd_pcm_sw_params_set_tstamp_mode(handle->capture_handle, sw_params, SND_PCM_TSTAMP_ENABLE)) < 0) {
        fprintf(stderr, "[ERROR] cannot set tstamp mode (%s)\n", snd_strerror(err));
        return -13;
    }

    if ((err = snd_pcm_sw_params_set_tstamp_type(handle->capture_handle, sw_params, SND_PCM_TSTAMP_TYPE_GETTIMEOFDAY)) < 0) {
        fprintf(stderr, "[ERROR] cannot set tstamp type (%s)\n", snd_strerror(err));
        return -14;
    }

    if ((err = snd_pcm_sw_params(handle->capture_handle, sw_params)) < 0) {
        fprintf(stderr, "[ERROR] cannot set sw_params (%s)\n", snd_strerror(err));
        return -15;
    }

    snd_pcm_sw_params_free(sw_params);

    if ((err = snd_pcm_status_malloc(&handle->capture_status)) < 0)
    {
        fprintf(stderr, "[ERROR] cannot allocate status structure (%s)\n", snd_strerror(err));
        return -16;
    }

    // prepare the interface
    if ((err = snd_pcm_prepare(handle->capture_handle)) < 0)
    {
        fprintf(stderr, "[ERROR] cannot prepare audio interface for use (%s)\n", snd_strerror(err));
        return -17;
    }

    handle->buffer_size = handle->buffer_frames_num * (snd_pcm_format_width(PCM_FORMAT) / 8) * NUM_CHANNELS;
    handle->buffer = (int32_t*) malloc(handle->buffer_size);

    //void* ret;
    //pthread_join(th, &ret);

    return 0;
}


void alsa_pcm1822_deinit(alsa_pcm1822_t* handle)
{
    pthread_cancel(th);
    rp1_sys_rio_out_clr(handle->rp1_handle, handle->gpio_sync);

    free(handle->buffer);
    snd_pcm_status_free(handle->capture_status);
    snd_pcm_close(handle->capture_handle);
}

int alsa_pcm1822_read(alsa_pcm1822_t* handle)
{
    int err;
    if ((err = snd_pcm_readi(handle->capture_handle, handle->buffer, handle->buffer_frames_num)) != handle->buffer_frames_num) {
        //fprintf(stderr, "[ERROR] read from audio interface failed (%s)\n", snd_strerror(err));
        snd_pcm_recover(handle->capture_handle, err, 0);
        return -1;
    } 
    //rp1_sys_rio_out_xor(handle->rp1_handle, handle->gpio_sync);

    snd_pcm_status(handle->capture_handle, handle->capture_status);
    snd_pcm_status_set_audio_htstamp_config(handle->capture_status, &audio_tstamp_config);

    snd_pcm_status_get_htstamp(handle->capture_status, &handle->hstamp);
    snd_pcm_status_get_trigger_htstamp(handle->capture_status, &handle->hstamp_trigger);
    snd_pcm_status_get_audio_htstamp(handle->capture_status, &handle->hstamp_audio);
    snd_pcm_status_get_driver_htstamp(handle->capture_status, &handle->hstamp_driver);

    snd_pcm_status_get_audio_htstamp_report(handle->capture_status, &handle->hstamp_report);

    handle->frames_avail = snd_pcm_status_get_avail(handle->capture_status);
    handle->frames_delay = snd_pcm_status_get_delay(handle->capture_status);

    return 0;
}

