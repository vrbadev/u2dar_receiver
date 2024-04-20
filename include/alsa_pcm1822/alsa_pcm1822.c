#include "alsa_pcm1822.h"


static snd_pcm_audio_tstamp_config_t audio_tstamp_config = { .type_requested = 0, .report_delay = 0 };

int alsa_pcm1822_init(alsa_pcm1822_t* handle)
{
    snd_pcm_hw_params_t *hw_params;
    snd_pcm_sw_params_t *sw_params;
    snd_pcm_format_t format = SND_PCM_FORMAT_S32_LE;

    int err;
    // open audio interface
    if ((err = snd_pcm_open(&handle->capture_handle, handle->dev_name, SND_PCM_STREAM_CAPTURE, 0)) < 0) {
        fprintf(stderr, "cannot open audio device %s (%s)\n", handle->dev_name, snd_strerror(err));
        return -1;
    }

    // setup hw params
    if ((err = snd_pcm_hw_params_malloc(&hw_params)) < 0) {
        fprintf(stderr, "cannot allocate hardware parameter structure (%s)\n", snd_strerror(err));
        return -2;
    }

    if ((err = snd_pcm_hw_params_any(handle->capture_handle, hw_params)) < 0) {
        fprintf(stderr, "cannot initialize hardware parameter structure (%s)\n", snd_strerror(err));
        return -3;
    }

    if ((err = snd_pcm_hw_params_set_access(handle->capture_handle, hw_params, SND_PCM_ACCESS_RW_INTERLEAVED)) < 0) {
        fprintf(stderr, "cannot set access type (%s)\n", snd_strerror(err));
        return -4;
    }

    if ((err = snd_pcm_hw_params_set_format(handle->capture_handle, hw_params, format)) < 0) {
        fprintf(stderr, "cannot set sample format (%s)\n", snd_strerror(err));
        return -5;
    }

    if ((err = snd_pcm_hw_params_set_rate_near(handle->capture_handle, hw_params, &handle->sample_rate, 0)) < 0) {
        fprintf(stderr, "cannot set sample rate (%s)\n", snd_strerror(err));
        return -6;
    }

    if ((err = snd_pcm_hw_params_set_channels(handle->capture_handle, hw_params, 2)) < 0) {
        fprintf(stderr, "cannot set channel count (%s)\n", snd_strerror(err));
        return -7;
    }

    if ((err = snd_pcm_hw_params(handle->capture_handle, hw_params)) < 0) {
        fprintf(stderr, "cannot set hw_params (%s)\n", snd_strerror(err));
        return -8;
    }

    snd_pcm_hw_params_free(hw_params);

    // setup sw params
    if ((err = snd_pcm_sw_params_malloc(&sw_params)) < 0) {
        fprintf(stderr, "cannot allocate software parameter structure (%s)\n", snd_strerror(err));
        return -9;
    }

    if ((err = snd_pcm_sw_params_current(handle->capture_handle, sw_params)) < 0) {
        fprintf(stderr, "cannot initialize sw_params (%s)\n", snd_strerror(err));
        return -10;
    }

    if ((err = snd_pcm_sw_params_set_tstamp_mode(handle->capture_handle, sw_params, SND_PCM_TSTAMP_ENABLE)) < 0) {
        fprintf(stderr, "cannot set tstamp mode (%s)\n", snd_strerror(err));
        return -11;
    }

    if ((err = snd_pcm_sw_params_set_tstamp_type(handle->capture_handle, sw_params, SND_PCM_TSTAMP_TYPE_MONOTONIC)) < 0) {
        fprintf(stderr, "cannot set tstamp type (%s)\n", snd_strerror(err));
        return -12;
    }

    if ((err = snd_pcm_sw_params(handle->capture_handle, sw_params)) < 0) {
        fprintf(stderr, "cannot set sw_params (%s)\n", snd_strerror(err));
        return -13;
    }

    snd_pcm_sw_params_free(sw_params);

    // prepare the interface
    if ((err = snd_pcm_prepare(handle->capture_handle)) < 0)
    {
        fprintf(stderr, "cannot prepare audio interface for use (%s)\n", snd_strerror(err));
        return -14;
    }

    if ((err = snd_pcm_status_malloc(&handle->capture_status)) < 0)
    {
        fprintf(stderr, "cannot allocate status structure (%s)\n", snd_strerror(err));
        return -15;
    }

    int buffer_bytes = handle->buffer_frames_num * (snd_pcm_format_width(format) / 8) * 2;
    handle->buffer = (int32_t*) malloc(buffer_bytes);

    return 0;
}


void alsa_pcm1822_deinit(alsa_pcm1822_t* handle)
{
    free(handle->buffer);
    snd_pcm_status_free(handle->capture_status);
    snd_pcm_close(handle->capture_handle);
}

int alsa_pcm1822_read(alsa_pcm1822_t* handle)
{
    int err;
    if ((err = snd_pcm_readi(handle->capture_handle, handle->buffer, handle->buffer_frames_num)) != handle->buffer_frames_num) {
        fprintf(stderr, "read from audio interface failed (%s)\n", err, snd_strerror(err));
        return -1;
    }

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

