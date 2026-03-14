/*********************************************************************************************
 *   Filename        : ble_ai_earbuds.c
 *
 *   Description     : AI Earbuds Example — Dual Communication During Cellular Calls
 *
 *   This example implements Bluetooth earbuds that:
 *
 *   1. Handle cellular phone calls via HFP (Hands-Free Profile) over Classic Bluetooth.
 *      - The phone routes call audio to/from the earbuds automatically via SCO.
 *      - You hear the remote caller through the earbuds speaker.
 *      - Your voice goes from the earbuds mic to the remote caller.
 *
 *   2. Simultaneously maintain a BLE (Bluetooth Low Energy) data connection to a custom
 *      phone app. The app relays audio to/from a cloud AI service.
 *
 *   3. Mix AI audio (received over BLE) into the call so that:
 *      - The remote caller hears the AI speaking (mixed into SCO uplink).
 *      - You also hear the AI through the earbuds speaker.
 *
 *   4. Send call audio (your mic + remote caller mixed) over BLE to the phone app,
 *      which forwards it to the cloud AI so it can listen to the conversation.
 *
 *   ┌─────────────────────────────────────────────────────────┐
 *   │                    EARBUDS (this firmware)              │
 *   │                                                         │
 *   │  Mic ──► your voice ──┬──► ESCO encoder ──► SCO uplink  │
 *   │                       │         ▲          (to phone)   │
 *   │                       │         │                       │
 *   │  BLE RX (AI audio) ──►┘  recorder_mix                  │
 *   │                         (mixes AI into uplink)          │
 *   │                                                         │
 *   │  SCO downlink (remote voice) ──► DAC speaker            │
 *   │  BLE RX (AI audio) ──────────► DAC speaker (mixed)      │
 *   │                                                         │
 *   │  Mic + SCO downlink ──► encode ──► BLE TX to app        │
 *   └─────────────────────────────────────────────────────────┘
 *
 *   PREREQUISITES:
 *   - Board config must enable: TCFG_USER_EDR_ENABLE=1, TCFG_USER_BLE_ENABLE=1
 *   - Profile config must enable: USER_SUPPORT_PROFILE_HFP=1
 *   - Audio must be enabled: TCFG_AUDIO_ENABLE=1, TCFG_AUDIO_ADC_ENABLE, TCFG_AUDIO_DAC_ENABLE
 *   - Mixer must be enabled: RECORDER_MIX_EN=1
 *   - See ble_ai_earbuds.h for required board config changes.
 *
 *   Copyright:(c)JIELI  2011-2026  @ , All Rights Reserved.
 *********************************************************************************************/

#include "system/app_core.h"
#include "system/includes.h"
#include "app_config.h"
#include "app_action.h"
#include "btstack/btstack_task.h"
#include "btstack/bluetooth.h"
#include "btstack/btstack_event.h"
#include "btcontroller_modules.h"
#include "bt_common.h"
#include "le_common.h"
#include "user_cfg.h"
#include "vm.h"
#include "gatt_common/le_gatt_common.h"
#include "btstack/avctp_user.h"

/* Include audio headers for the mixer and ESCO encoder */
#if TCFG_AUDIO_ENABLE
#include "audio_enc.h"
#include "audio_config.h"
#ifndef CONFIG_LITE_AUDIO
#include "audio_recorder_mix.h"
#endif
#endif

/* In-ear detection: detects when user puts earbuds on/off */
#if TCFG_EARTCH_EVENT_HANDLE_ENABLE
#include "in_ear_detect/in_ear_manage.h"
#endif

/* Keyword spotting (wake word): detects when user says a trigger phrase */
#if TCFG_KWS_VOICE_RECOGNITION_ENABLE
#include "jl_kws/jl_kws_api.h"
#endif

/* Audio energy detection: detects silence for auto-stop */
#include "application/audio_energy_detect.h"

/* System timers: used for 5-minute silence timeout */
#include "system/timer.h"

/* Only compile this example when it is selected in app_config.h */
#if CONFIG_APP_SPP_LE

/*
 * ============================================================================
 *  SECTION 1: Logging
 * ============================================================================
 */
#define LOG_TAG     "[AI_EARBUDS]"
#define LOG_ERROR_ENABLE
#define LOG_DEBUG_ENABLE
#define LOG_INFO_ENABLE
#include "debug.h"

#define log_info(x, ...)  printf("[AI_EARBUDS] " x " ", ## __VA_ARGS__)


/*
 * ============================================================================
 *  SECTION 2: BLE Configuration Constants
 * ============================================================================
 *
 *  These control the BLE GATT server behavior. The earbuds act as a BLE
 *  peripheral (slave). Your phone app connects as the central (master).
 */

/* Maximum Transmission Unit — largest single BLE packet payload.
 * 512 bytes allows efficient audio chunk transfers. */
#define ATT_LOCAL_MTU_SIZE        (512)

/* How many BLE packets we buffer before blocking. */
#define ATT_PACKET_NUMS_MAX       (2)

/* Total send buffer size. Each packet needs a header + MTU payload. */
#define ATT_SEND_CBUF_SIZE        (ATT_PACKET_NUMS_MAX * (ATT_PACKET_HEAD_SIZE + ATT_LOCAL_MTU_SIZE))

/* BLE advertising interval: 500ms (800 * 0.625ms).
 * Shorter = faster discovery, higher power consumption. */
#define ADV_INTERVAL_MIN          (160 * 5)


/*
 * ============================================================================
 *  SECTION 3: Call State Tracking
 * ============================================================================
 *
 *  We need to know when a phone call is active so we can:
 *  - Start/stop the audio mixer
 *  - Begin/end streaming audio over BLE
 */
typedef enum {
    AI_CALL_STATE_IDLE = 0,     /* No call active */
    AI_CALL_STATE_INCOMING,     /* Phone is ringing, call not yet answered */
    AI_CALL_STATE_ACTIVE,       /* Call is in progress — SCO audio is flowing */
} ai_call_state_t;

static ai_call_state_t current_call_state = AI_CALL_STATE_IDLE;


/*
 * ============================================================================
 *  SECTION 3B: AI Streaming State Machine
 * ============================================================================
 *
 *  Streaming is INDEPENDENT from the call state. A call can be active but
 *  AI streaming may be off (user hasn't triggered it yet).
 *
 *  AI streaming = sending call audio to the phone app over BLE, and accepting
 *  AI audio back. The user must explicitly start it via one of:
 *
 *  START triggers:
 *    1. In-ear detection  — user puts earbuds on (hardware sensor)
 *    2. Wake word          — user says a trigger phrase ("Hey AI")
 *    3. App button         — user taps Start in the phone app (BLE command)
 *
 *  STOP triggers:
 *    1. Ear removal        — user takes earbuds off (hardware sensor)
 *    2. 5-min silence      — no voice detected for 5 minutes
 *    3. App button         — user taps Stop in the phone app (BLE command)
 */
typedef enum {
    AI_STREAM_OFF = 0,   /* Not streaming — BLE audio is silent */
    AI_STREAM_ON  = 1,   /* Actively streaming call audio over BLE */
} ai_stream_state_t;

static ai_stream_state_t ai_stream_state = AI_STREAM_OFF;

/* 5-minute silence timeout timer ID. 0 = no timer running. */
static u16 ai_silence_timer_id = 0;

/* Energy detector handle for silence detection */
static void *ai_energy_detector = NULL;

/* Silence timeout: 5 minutes = 300,000 ms */
#define AI_SILENCE_TIMEOUT_MS   (5 * 60 * 1000)


/*--- Forward declarations for trigger callbacks ---*/
static void ai_stream_start(u8 reason);
static void ai_stream_stop(u8 reason);
static void ai_notify_stream_state(void);
static void ai_silence_event_handler(u8 event, u8 ch);
static void ai_silence_timeout_callback(void *priv);

/* Reason codes — tells the phone app WHY streaming started/stopped */
#define AI_REASON_EAR_ON        0x01  /* Earbuds put on ear */
#define AI_REASON_WAKE_WORD     0x02  /* Wake word detected */
#define AI_REASON_APP_CMD       0x03  /* Phone app sent start/stop command */
#define AI_REASON_EAR_OFF       0x11  /* Earbuds removed from ear */
#define AI_REASON_SILENCE       0x12  /* 5 minutes of silence */
#define AI_REASON_APP_CMD_STOP  0x13  /* Phone app sent stop command */


/**
 * Start AI streaming.
 *
 * After this, call audio (mic + remote caller) is sent to the phone app,
 * and AI audio from the app is accepted and mixed into the call.
 *
 * @param reason  Why streaming started (AI_REASON_xxx) — sent to phone app
 */
static void ai_stream_start(u8 reason)
{
    if (ai_stream_state == AI_STREAM_ON) {
        log_info("Stream already ON, ignoring start (reason=%02x)\n", reason);
        return;
    }

    log_info(">>> AI STREAM START (reason=%02x)\n", reason);
    ai_stream_state = AI_STREAM_ON;

    /* Cancel any pending silence timer from a previous session */
    if (ai_silence_timer_id) {
        sys_timeout_del(ai_silence_timer_id);
        ai_silence_timer_id = 0;
    }

    /*
     * Start the silence detector.
     *
     * audio_energy_detect monitors the PCM audio stream and fires a callback
     * when silence (MUTE) or voice (UNMUTE) is detected.
     *
     * We use this to implement the "5 minutes of silence" auto-stop:
     * - On MUTE: start a 5-minute timer
     * - On UNMUTE: cancel the timer (someone is talking)
     * - If the timer fires: stop streaming
     */
    if (!ai_energy_detector) {
        audio_energy_detect_param e_param = {0};
        e_param.mute_energy   = 5;       /* Threshold below which = silence */
        e_param.unmute_energy = 10;      /* Threshold above which = voice  */
        e_param.mute_time_ms  = 3000;    /* 3s continuous silence to confirm */
        e_param.unmute_time_ms = 500;    /* 0.5s of voice to confirm        */
        e_param.sample_rate   = 8000;    /* Match SCO sample rate            */
        e_param.count_cycle_ms = 500;    /* Check every 500ms                */
        e_param.event_handler = ai_silence_event_handler;
        e_param.ch_total      = 1;       /* Mono                             */
        e_param.dcc           = 1;       /* DC offset removal                */
        e_param.onoff         = 1;
        ai_energy_detector = audio_energy_detect_open(&e_param);
    }

    /* Notify phone app that streaming has started + reason */
    ai_notify_stream_state();
}


/**
 * Stop AI streaming.
 *
 * After this, no more call audio is sent to the phone app,
 * and incoming AI audio from BLE is discarded.
 *
 * @param reason  Why streaming stopped (AI_REASON_xxx) — sent to phone app
 */
static void ai_stream_stop(u8 reason)
{
    if (ai_stream_state == AI_STREAM_OFF) {
        log_info("Stream already OFF, ignoring stop (reason=%02x)\n", reason);
        return;
    }

    log_info(">>> AI STREAM STOP (reason=%02x)\n", reason);
    ai_stream_state = AI_STREAM_OFF;

    /* Cancel the silence timeout timer */
    if (ai_silence_timer_id) {
        sys_timeout_del(ai_silence_timer_id);
        ai_silence_timer_id = 0;
    }

    /* Close the energy detector */
    if (ai_energy_detector) {
        audio_energy_detect_close(ai_energy_detector);
        ai_energy_detector = NULL;
    }

    /* Notify phone app */
    ai_notify_stream_state();
}


/**
 * Send current streaming state to the phone app over BLE (characteristic 0xAE04).
 * Byte format: [stream_state, reason]
 */
static void ai_notify_stream_state(void)
{
    if (!ai_ble_con_handle) {
        return;
    }
    u8 payload[2];
    payload[0] = (u8)ai_stream_state;       /* 0=OFF, 1=ON */
    payload[1] = (u8)current_call_state;    /* 0=idle, 1=incoming, 2=active */

    if (ble_gatt_server_characteristic_ccc_get(ai_ble_con_handle,
            ATT_CHARACTERISTIC_ae04_01_CLIENT_CONFIGURATION_HANDLE)) {
        ble_comm_att_send_data(ai_ble_con_handle,
            ATT_CHARACTERISTIC_ae04_01_VALUE_HANDLE,
            payload, sizeof(payload), ATT_OP_AUTO_READ_CCC);
    }
}


/*
 * ============================================================================
 *  SECTION 3C: Trigger 1 — In-Ear Detection
 * ============================================================================
 *
 *  When the user puts the earbuds in their ear, a hardware sensor
 *  (capacitive touch or IR proximity) detects it.
 *
 *  The SDK's ear detection module calls ear_detect_event_handle() which
 *  posts EAR_DETECT_EVENT_IN / EAR_DETECT_EVENT_OUT events.
 *
 *  We hook into this to start/stop AI streaming.
 *
 *  HARDWARE REQUIREMENT:
 *  - Board must have an ear detection sensor wired up
 *  - Board config must set TCFG_LP_EARTCH_KEY_ENABLE = 1
 *  - Board config must set TCFG_EARTCH_EVENT_HANDLE_ENABLE = 1
 *  - If no sensor is available, this trigger is simply inactive
 */

/**
 * Called by the ear detection module when ear state changes.
 * Hook this in your board's ear detection config.
 */
void ai_ear_detect_handler(u8 event)
{
    switch (event) {
    case 1: /* EAR_DETECT_EVENT_IN — earbuds inserted into ear */
        log_info("Ear: IN\n");
        ai_stream_start(AI_REASON_EAR_ON);
        break;

    case 2: /* EAR_DETECT_EVENT_OUT — earbuds removed from ear */
        log_info("Ear: OUT\n");
        ai_stream_stop(AI_REASON_EAR_OFF);
        break;

    default:
        break;
    }
}


/*
 * ============================================================================
 *  SECTION 3D: Trigger 2 — Wake Word Detection (KWS)
 * ============================================================================
 *
 *  The SDK includes a keyword spotting engine (jl_kws) that runs on a
 *  dedicated RTOS thread. It continuously analyzes mic audio and fires
 *  a callback when it detects a pre-trained keyword.
 *
 *  Default keywords are "Yes" / "No" but custom keywords can be trained
 *  with Jieli's KWS tool.
 *
 *  We map keyword detection to: start streaming.
 *  (Stopping via wake word is unusual — use silence or app button instead.)
 *
 *  REQUIREMENT:
 *  - Board config: TCFG_KWS_VOICE_RECOGNITION_ENABLE = 1
 *  - Allocate KWS task stack in app_main.c task_info_table[]
 *  - CPU frequency should be >= 48MHz when KWS is active
 */

/**
 * Called by the KWS engine when a keyword is detected.
 * Register this in jl_kws_event.c or via the KWS callback mechanism.
 *
 * @param voice_event  KWS_VOICE_EVENT_YES (2), KWS_VOICE_EVENT_NO (3), etc.
 */
void ai_kws_event_handler(u8 voice_event)
{
    log_info("KWS event: %d\n", voice_event);

    /*
     * Any keyword detection triggers streaming start.
     *
     * You can map different keywords to different behaviors:
     *   YES (2) = start streaming
     *   NO  (3) = stop streaming
     * Or use a single custom wake word like "Hey AI" mapped to event 2.
     */
    if (voice_event == 2) {  /* KWS_VOICE_EVENT_YES */
        ai_stream_start(AI_REASON_WAKE_WORD);
    }
}


/*
 * ============================================================================
 *  SECTION 3E: Trigger 3 — App Button (BLE Command)
 * ============================================================================
 *
 *  The phone app writes a command byte to characteristic 0xAE10:
 *    0x01 = Start streaming
 *    0x00 = Stop streaming
 *
 *  This is handled in Section 11 (ATT Write Callback) below.
 *  No additional code needed here — it just calls ai_stream_start/stop.
 */


/*
 * ============================================================================
 *  SECTION 3F: Auto-Stop — 5 Minutes of Silence
 * ============================================================================
 *
 *  When streaming is active, we monitor the audio level.
 *  If nobody speaks for 5 continuous minutes, we auto-stop streaming
 *  to save battery.
 *
 *  How it works:
 *  1. audio_energy_detect monitors each PCM frame for voice/silence.
 *  2. On silence detected → start a 5-minute one-shot timer.
 *  3. If voice resumes → cancel the timer, restart silence monitoring.
 *  4. If the 5-min timer fires → call ai_stream_stop().
 *
 *  The energy detector distinguishes real silence from background noise
 *  because the threshold (mute_energy=5) is calibrated above typical
 *  environmental noise floor. For noisy environments, increase this value.
 */

/**
 * Called by audio_energy_detect when silence/voice state changes.
 *
 * @param event   AUDIO_E_DET_MUTE (0x01) = silence, AUDIO_E_DET_UNMUTE (0x00) = voice
 * @param ch      Audio channel (0 for mono)
 */
static void ai_silence_event_handler(u8 event, u8 ch)
{
    if (ai_stream_state != AI_STREAM_ON) {
        return; /* Only track silence while streaming */
    }

    if (event == 0x01) {
        /* MUTE — silence detected. Start the 5-minute countdown. */
        if (!ai_silence_timer_id) {
            log_info("Silence detected — starting %d min timer\n",
                AI_SILENCE_TIMEOUT_MS / 60000);
            ai_silence_timer_id = sys_timeout_add(
                NULL,
                ai_silence_timeout_callback,
                AI_SILENCE_TIMEOUT_MS
            );
        }
    } else {
        /* UNMUTE — voice detected. Cancel the silence timer. */
        if (ai_silence_timer_id) {
            log_info("Voice detected — canceling silence timer\n");
            sys_timeout_del(ai_silence_timer_id);
            ai_silence_timer_id = 0;
        }
    }
}

/**
 * Called when 5 minutes of silence have elapsed.
 * One-shot timer — fires once then auto-deletes.
 */
static void ai_silence_timeout_callback(void *priv)
{
    log_info(">>> 5-minute silence timeout — auto-stopping stream\n");
    ai_silence_timer_id = 0; /* Timer has fired and self-deleted */
    ai_stream_stop(AI_REASON_SILENCE);
}


/*
 * ============================================================================
 *  SECTION 4: BLE Connection State
 * ============================================================================
 */

/* Handle to the current BLE connection. 0 means not connected. */
static u16 ai_ble_con_handle = 0;

/* Whether to request a connection parameter update after connecting. */
static u8 ai_connection_update_enable = 1;

/* Preferred BLE connection parameters.
 * The phone (central) may accept or reject these.
 * Format: { min_interval, max_interval, latency, timeout }
 * Intervals are in units of 1.25ms. Timeout in units of 10ms. */
static const struct conn_update_param_t ai_connection_param_table[] = {
    {16, 24, 0, 600},   /* 20-30ms interval, no latency skipping, 6s timeout */
    {12, 28, 0, 600},   /* Slightly faster alternative */
};
#define CONN_PARAM_TABLE_CNT  (sizeof(ai_connection_param_table)/sizeof(struct conn_update_param_t))


/*
 * ============================================================================
 *  SECTION 5: GATT Profile Definition
 * ============================================================================
 *
 *  This defines the BLE services and characteristics that the phone app sees
 *  when it connects and discovers services.
 *
 *  We define a custom service with UUID 0xAE30 containing:
 *
 *  ┌─────────────────────────────────────────────────────────────┐
 *  │ Service: 0xAE30 (AI Audio Service)                          │
 *  │                                                             │
 *  │ Characteristic 0xAE01 — WRITE_WITHOUT_RESPONSE | DYNAMIC    │
 *  │   → Phone app writes AI audio data here (AI TTS → earbuds) │
 *  │                                                             │
 *  │ Characteristic 0xAE02 — NOTIFY | DYNAMIC                    │
 *  │   → Earbuds push call audio here (earbuds → phone app)     │
 *  │   → Has a Client Configuration Descriptor (CCD) for        │
 *  │     enabling/disabling notifications                        │
 *  │                                                             │
 *  │ Characteristic 0xAE10 — READ | WRITE | DYNAMIC              │
 *  │   → Control channel for commands (start/stop AI, status)    │
 *  └─────────────────────────────────────────────────────────────┘
 *
 *  The profile is defined as raw bytes because the SDK's GATT server
 *  expects a pre-compiled attribute database. You can generate this using
 *  Jieli's "BLE Profile Tool" in sdk_tools/BLE Profile工具/.
 */

/* Include the standard trans_data profile — it already defines the services
 * we need (0xAE30 with ae01 write + ae02 notify + ae10 read/write).
 * For a production product, you'd generate a custom profile. */
#include "ble_trans_profile.h"


/*
 * ============================================================================
 *  SECTION 6: Audio Buffer for BLE ↔ Audio Bridge
 * ============================================================================
 *
 *  Audio data received from BLE (AI speech) is written into this ring buffer.
 *  The audio mixer reads from it and injects into the SCO uplink.
 *
 *  Audio from the mic + SCO downlink is written here before sending over BLE.
 *
 *  Buffer sizing:
 *  - At 8kHz mono 16-bit PCM, 1 second = 16,000 bytes
 *  - We keep 100ms of buffering = 1,600 bytes
 *  - Round up to 2048 for alignment
 */
#define AI_AUDIO_BUF_SIZE   2048

/* Ring buffer for AI audio received from BLE, waiting to be mixed into call */
static u8 ai_rx_audio_buf[AI_AUDIO_BUF_SIZE];
static volatile u16 ai_rx_write_pos = 0;
static volatile u16 ai_rx_read_pos = 0;

/* Ring buffer for call audio to send to phone app over BLE */
static u8 ai_tx_audio_buf[AI_AUDIO_BUF_SIZE];
static volatile u16 ai_tx_write_pos = 0;
static volatile u16 ai_tx_read_pos = 0;


/*
 * ============================================================================
 *  SECTION 7: Ring Buffer Helpers
 * ============================================================================
 *
 *  Simple lock-free single-producer single-consumer ring buffer.
 *  Safe because only one ISR/task writes and one reads.
 */

static u16 ring_buf_data_len(volatile u16 *wr, volatile u16 *rd, u16 size)
{
    u16 w = *wr;
    u16 r = *rd;
    if (w >= r) {
        return w - r;
    }
    return size - r + w;
}

static u16 ring_buf_free_len(volatile u16 *wr, volatile u16 *rd, u16 size)
{
    return size - 1 - ring_buf_data_len(wr, rd, size);
}

static u16 ring_buf_write(u8 *buf, u16 buf_size, volatile u16 *wr, const u8 *data, u16 len)
{
    u16 w = *wr;
    for (u16 i = 0; i < len; i++) {
        buf[w] = data[i];
        w++;
        if (w >= buf_size) {
            w = 0;
        }
    }
    *wr = w;
    return len;
}

static u16 ring_buf_read(u8 *buf, u16 buf_size, volatile u16 *rd, u8 *out, u16 len)
{
    u16 r = *rd;
    for (u16 i = 0; i < len; i++) {
        out[i] = buf[r];
        r++;
        if (r >= buf_size) {
            r = 0;
        }
    }
    *rd = r;
    return len;
}


/*
 * ============================================================================
 *  SECTION 8: AI Audio ↔ Call Audio Bridge Functions
 * ============================================================================
 *
 *  These functions connect the BLE data channel to the audio subsystem.
 */

/**
 * Called when AI audio PCM data arrives from BLE (phone app wrote to 0xAE01).
 *
 * This data is the AI's spoken response (TTS audio) that needs to be:
 * 1. Mixed into the SCO uplink (so remote caller hears it)
 * 2. Played on the local speaker (so the user hears it)
 *
 * @param data   Raw PCM audio bytes from the phone app
 * @param len    Number of bytes
 */
static void ai_audio_receive_from_ble(u8 *data, u16 len)
{
    if (current_call_state != AI_CALL_STATE_ACTIVE) {
        /* No active call — discard AI audio. Could also play locally. */
        return;
    }

    if (ai_stream_state != AI_STREAM_ON) {
        /* Streaming not started — user hasn't triggered it yet. */
        return;
    }

    /*
     * Feed AI audio into the recorder mixer.
     *
     * recorder_mix_pcm_write() is provided by the SDK's pre-compiled library.
     * It writes PCM data into the audio mixer's input channel. The mixer
     * combines this with the microphone input, and the combined signal
     * goes into the ESCO encoder → SCO uplink → remote caller hears it.
     *
     * The SDK's audio pipeline (in audio_enc.c) calls recorder_mix_sco_data_write()
     * for mic data. The mixer combines both streams automatically.
     */
#if TCFG_AUDIO_ENABLE && RECORDER_MIX_EN
    recorder_mix_pcm_write(data, len);
#endif

    /*
     * Also buffer AI audio for local playback.
     * In a full implementation, you would write this to a DAC mixer channel
     * so the user hears the AI alongside the remote caller's voice.
     * The SDK's audio_mixer (in audio_dec.c) supports multiple input channels.
     *
     * For now, we buffer it — a production implementation would use:
     *   audio_mixer_ch_write(&dac_mixer_ch, data, len);
     */
    u16 free = ring_buf_free_len(&ai_rx_write_pos, &ai_rx_read_pos, AI_AUDIO_BUF_SIZE);
    if (free >= len) {
        ring_buf_write(ai_rx_audio_buf, AI_AUDIO_BUF_SIZE, &ai_rx_write_pos, data, len);
    } else {
        log_info("AI RX buffer full, dropping %d bytes\n", len);
    }
}


/**
 * Called periodically (or from the audio ISR) to send call audio to the phone app.
 *
 * This takes mixed audio (your mic + remote caller) and sends it over BLE
 * so the cloud AI can hear both sides of the conversation.
 *
 * In a production implementation, you would hook this into the audio_enc.c
 * mic output callback (adc_mic_output_handler) to capture mic data, and
 * the ESCO decoder output callback to capture the remote caller's voice.
 */
static void ai_audio_send_to_ble(u8 *pcm_data, u16 len)
{
    if (!ai_ble_con_handle) {
        return; /* No BLE connection — nowhere to send */
    }

    if (current_call_state != AI_CALL_STATE_ACTIVE) {
        return; /* No active call */
    }

    if (ai_stream_state != AI_STREAM_ON) {
        return; /* Streaming not started by user */
    }

    /*
     * Feed PCM data into the energy detector so it can track
     * silence vs. voice and trigger the 5-minute auto-stop.
     */
    if (ai_energy_detector) {
        audio_energy_detect_run(ai_energy_detector, (s16 *)pcm_data, len / 2);
    }

    /*
     * Check if the BLE send buffer has space and the phone app has
     * subscribed to notifications on characteristic 0xAE02.
     *
     * ble_comm_att_check_send() returns true if the BLE stack can accept
     * a packet of the given size right now.
     *
     * ble_gatt_server_characteristic_ccc_get() checks if the phone app
     * has written 0x0001 to the Client Characteristic Configuration
     * descriptor, which means "I want to receive notifications".
     */
    if (ble_comm_att_check_send(ai_ble_con_handle, len) &&
        ble_gatt_server_characteristic_ccc_get(ai_ble_con_handle,
            ATT_CHARACTERISTIC_ae02_01_CLIENT_CONFIGURATION_HANDLE)) {

        /*
         * Send the audio data as a BLE notification.
         *
         * ATT_CHARACTERISTIC_ae02_01_VALUE_HANDLE is the attribute handle
         * for the 0xAE02 characteristic value. The phone app receives this
         * as a notification callback.
         *
         * ATT_OP_AUTO_READ_CCC means "automatically check if notifications
         * are enabled before sending".
         */
        ble_comm_att_send_data(ai_ble_con_handle,
            ATT_CHARACTERISTIC_ae02_01_VALUE_HANDLE,
            pcm_data, len, ATT_OP_AUTO_READ_CCC);
    }
}


/*
 * ============================================================================
 *  SECTION 9: HFP Call Event Handling
 * ============================================================================
 *
 *  These functions respond to phone call lifecycle events.
 *  The Bluetooth stack calls our EDR status handler (Section 13) which
 *  dispatches to these functions.
 */

/**
 * Called when the phone reports an incoming call (ringing).
 * At this point, no audio channel exists yet.
 */
static void ai_on_call_incoming(void)
{
    log_info(">>> Incoming call\n");
    current_call_state = AI_CALL_STATE_INCOMING;

    /* Notify phone app that a call is incoming (optional).
     * You could send a BLE notification with a status byte. */
    u8 status = 0x01; /* 0x01 = incoming call */
    if (ai_ble_con_handle &&
        ble_gatt_server_characteristic_ccc_get(ai_ble_con_handle,
            ATT_CHARACTERISTIC_ae04_01_CLIENT_CONFIGURATION_HANDLE)) {
        ble_comm_att_send_data(ai_ble_con_handle,
            ATT_CHARACTERISTIC_ae04_01_VALUE_HANDLE,
            &status, 1, ATT_OP_AUTO_READ_CCC);
    }
}


/**
 * Called when a call becomes active (answered).
 * The phone has established an SCO audio link — audio is now flowing.
 *
 * We start the audio mixer so that AI audio can be injected into the call.
 */
static void ai_on_call_active(void)
{
    log_info(">>> Call active — starting audio mixer\n");
    current_call_state = AI_CALL_STATE_ACTIVE;

    /* Reset audio buffers */
    ai_rx_write_pos = 0;
    ai_rx_read_pos = 0;
    ai_tx_write_pos = 0;
    ai_tx_read_pos = 0;

    /*
     * Start the recorder mixer.
     *
     * recorder_mix_call_status_change(1) tells the mixer that a call is
     * now active. This affects how it routes audio internally.
     *
     * recorder_mix_start() actually begins the mixing operation.
     * After this, any data written via recorder_mix_pcm_write() will be
     * combined with the microphone audio in the ESCO encoder.
     */
#if TCFG_AUDIO_ENABLE && RECORDER_MIX_EN
    recorder_mix_call_status_change(1);
    recorder_mix_start();
#endif

    /* Notify phone app that a call is now active */
    u8 status = 0x02; /* 0x02 = call active, ready for AI audio */
    if (ai_ble_con_handle &&
        ble_gatt_server_characteristic_ccc_get(ai_ble_con_handle,
            ATT_CHARACTERISTIC_ae04_01_CLIENT_CONFIGURATION_HANDLE)) {
        ble_comm_att_send_data(ai_ble_con_handle,
            ATT_CHARACTERISTIC_ae04_01_VALUE_HANDLE,
            &status, 1, ATT_OP_AUTO_READ_CCC);
    }
}


/**
 * Called when a call ends (local or remote hangup).
 * The SCO audio link is torn down.
 */
static void ai_on_call_hangup(void)
{
    log_info(">>> Call ended — stopping audio mixer\n");
    current_call_state = AI_CALL_STATE_IDLE;

    /* Stop AI streaming if it was running — can't stream without a call */
    if (ai_stream_state == AI_STREAM_ON) {
        ai_stream_stop(AI_REASON_EAR_OFF);
    }

    /*
     * Stop the mixer. After this, recorder_mix_pcm_write() has no effect.
     */
#if TCFG_AUDIO_ENABLE && RECORDER_MIX_EN
    recorder_mix_call_status_change(0);
    recorder_mix_stop();
#endif

    /* Notify phone app that the call has ended */
    u8 status = 0x00; /* 0x00 = idle, no call */
    if (ai_ble_con_handle &&
        ble_gatt_server_characteristic_ccc_get(ai_ble_con_handle,
            ATT_CHARACTERISTIC_ae04_01_CLIENT_CONFIGURATION_HANDLE)) {
        ble_comm_att_send_data(ai_ble_con_handle,
            ATT_CHARACTERISTIC_ae04_01_VALUE_HANDLE,
            &status, 1, ATT_OP_AUTO_READ_CCC);
    }
}


/*
 * ============================================================================
 *  SECTION 10: BLE ATT Read Callback
 * ============================================================================
 *
 *  Called by the BLE stack when the phone app reads a characteristic.
 *  We return the requested data.
 */

static u8 ai_read_write_ctrl_buf[4]; /* Control register, read/written by app */

static uint16_t ai_att_read_callback(hci_con_handle_t connection_handle,
    uint16_t att_handle, uint16_t offset,
    uint8_t *buffer, uint16_t buffer_size)
{
    uint16_t att_value_len = 0;
    uint16_t handle = att_handle;

    switch (handle) {

    /* 0x2A00 — GAP Device Name: return the Bluetooth name */
    case ATT_CHARACTERISTIC_2a00_01_VALUE_HANDLE: {
        char *gap_name = ble_comm_get_gap_name();
        att_value_len = strlen(gap_name);
        if ((offset >= att_value_len) || (offset + buffer_size) > att_value_len) {
            break;
        }
        if (buffer) {
            memcpy(buffer, &gap_name[offset], buffer_size);
            att_value_len = buffer_size;
        }
    }
    break;

    /* 0xAE10 — Control register: phone app reads current state */
    case ATT_CHARACTERISTIC_ae10_01_VALUE_HANDLE:
        /* Byte 0: streaming state (0=OFF, 1=ON)
         * Byte 1: call state (0=idle, 1=incoming, 2=active)
         * Byte 2: reserved
         * Byte 3: reserved */
        ai_read_write_ctrl_buf[0] = (u8)ai_stream_state;
        ai_read_write_ctrl_buf[1] = (u8)current_call_state;
        att_value_len = sizeof(ai_read_write_ctrl_buf);
        if ((offset >= att_value_len) || (offset + buffer_size) > att_value_len) {
            break;
        }
        if (buffer) {
            memcpy(buffer, &ai_read_write_ctrl_buf[offset], buffer_size);
            att_value_len = buffer_size;
        }
        break;

    /* CCC descriptors — return current notification/indication enable state */
    case ATT_CHARACTERISTIC_ae02_01_CLIENT_CONFIGURATION_HANDLE:
    case ATT_CHARACTERISTIC_ae04_01_CLIENT_CONFIGURATION_HANDLE:
    case ATT_CHARACTERISTIC_ae05_01_CLIENT_CONFIGURATION_HANDLE:
    case ATT_CHARACTERISTIC_2a05_01_CLIENT_CONFIGURATION_HANDLE:
        if (buffer) {
            buffer[0] = ble_gatt_server_characteristic_ccc_get(connection_handle, handle);
            buffer[1] = 0;
        }
        att_value_len = 2;
        break;

    default:
        break;
    }
    return att_value_len;
}


/*
 * ============================================================================
 *  SECTION 11: BLE ATT Write Callback
 * ============================================================================
 *
 *  Called by the BLE stack when the phone app writes to a characteristic.
 *  This is where we receive AI audio data and control commands.
 */
static int ai_att_write_callback(hci_con_handle_t connection_handle,
    uint16_t att_handle, uint16_t transaction_mode,
    uint16_t offset, uint8_t *buffer, uint16_t buffer_size)
{
    u16 handle = att_handle;

    switch (handle) {

    /*
     * 0xAE01 — AI Audio Data Write (app → earbuds)
     *
     * The phone app writes AI TTS audio here. This is the core data path
     * for getting AI speech into the earbuds.
     *
     * Audio format should be agreed upon between firmware and app:
     * - Raw PCM: 8kHz, 16-bit, mono (matches SCO CVSD codec)
     * - Or compressed (Opus/SBC) — would need decoding here first
     *
     * For this example we assume raw PCM for simplicity.
     */
    case ATT_CHARACTERISTIC_ae01_01_VALUE_HANDLE:
        log_info("AI audio RX: %d bytes\n", buffer_size);
        ai_audio_receive_from_ble(buffer, buffer_size);
        break;

    /*
     * CCC (Client Characteristic Configuration) writes.
     *
     * When the phone app writes 0x0001 here, it means "enable notifications"
     * for the corresponding characteristic. This is how the app subscribes
     * to receive data pushed from the earbuds.
     *
     * 0xAE02 CCC: subscribe to call audio stream
     * 0xAE04 CCC: subscribe to call status notifications
     */
    case ATT_CHARACTERISTIC_ae02_01_CLIENT_CONFIGURATION_HANDLE:
    case ATT_CHARACTERISTIC_ae04_01_CLIENT_CONFIGURATION_HANDLE:
    case ATT_CHARACTERISTIC_ae05_01_CLIENT_CONFIGURATION_HANDLE:
    case ATT_CHARACTERISTIC_2a05_01_CLIENT_CONFIGURATION_HANDLE:
        log_info("CCC write: handle=%04x, value=%02x\n", handle, buffer[0]);
        ble_gatt_server_characteristic_ccc_set(connection_handle, handle, buffer[0]);
        break;

    /*
     * 0xAE10 — Control Register Write (app → earbuds)
     *
     * The phone app can send commands here:
     *   0x01 = Start AI listening (begin streaming call audio to app)
     *   0x00 = Stop AI listening
     *   0x02 = Query status
     *
     * You can extend this protocol as needed for your AI integration.
     */
    /*
     * 0xAE10 — Control Register Write (app → earbuds)
     *
     * This is TRIGGER 3: the phone app's Start/Stop button.
     *
     * Commands:
     *   0x01 = Start AI streaming (user tapped Start button in app)
     *   0x00 = Stop AI streaming (user tapped Stop button in app)
     *   0x02 = Query current state (earbuds will send a notification back)
     */
    case ATT_CHARACTERISTIC_ae10_01_VALUE_HANDLE: {
        u16 tmp16 = sizeof(ai_read_write_ctrl_buf);
        if ((offset >= tmp16) || (offset + buffer_size) > tmp16) {
            break;
        }
        memcpy(&ai_read_write_ctrl_buf[offset], buffer, buffer_size);
        log_info("Control write: ");
        put_buf(buffer, buffer_size);

        u8 cmd = buffer[0];
        switch (cmd) {
        case 0x01:
            /* TRIGGER 3 START: App button pressed "Start" */
            log_info("App command: START streaming\n");
            ai_stream_start(AI_REASON_APP_CMD);
            break;

        case 0x00:
            /* TRIGGER 3 STOP: App button pressed "Stop" */
            log_info("App command: STOP streaming\n");
            ai_stream_stop(AI_REASON_APP_CMD_STOP);
            break;

        case 0x02:
            /* Query: send current state back */
            ai_notify_stream_state();
            break;

        default:
            break;
        }
    }
    break;

    default:
        break;
    }
    return 0;
}


/*
 * ============================================================================
 *  SECTION 12: BLE Event Handler
 * ============================================================================
 *
 *  Handles BLE connection lifecycle events (connect, disconnect, MTU exchange).
 */
static void ai_send_connection_update(u16 conn_handle)
{
    if (ai_connection_update_enable) {
        if (0 == ble_gatt_server_connetion_update_request(conn_handle,
                ai_connection_param_table, CONN_PARAM_TABLE_CNT)) {
            ai_connection_update_enable = 0;
        }
    }
}

static void ai_resume_all_ccc_enable(u16 conn_handle, u8 update_request)
{
    /* Re-enable notifications after reconnection with a bonded device */
    ble_gatt_server_characteristic_ccc_set(conn_handle,
        ATT_CHARACTERISTIC_ae02_01_CLIENT_CONFIGURATION_HANDLE, ATT_OP_NOTIFY);
    ble_gatt_server_characteristic_ccc_set(conn_handle,
        ATT_CHARACTERISTIC_ae04_01_CLIENT_CONFIGURATION_HANDLE, ATT_OP_NOTIFY);
    ble_gatt_server_characteristic_ccc_set(conn_handle,
        ATT_CHARACTERISTIC_ae05_01_CLIENT_CONFIGURATION_HANDLE, ATT_OP_INDICATE);

    if (update_request) {
        ai_send_connection_update(conn_handle);
    }
}


static int ai_event_packet_handler(int event, u8 *packet, u16 size, u8 *ext_param)
{
    switch (event) {

    /*
     * BLE connection established.
     * The phone app has connected. We can now exchange data.
     */
    case GATT_COMM_EVENT_CONNECTION_COMPLETE:
        ai_ble_con_handle = little_endian_read_16(packet, 0);
        ai_connection_update_enable = 1;
        log_info("BLE connected: handle=%04x\n", ai_ble_con_handle);
        log_info("  RSSI = %d dBm\n", ble_vendor_get_peer_rssi(ai_ble_con_handle));

        /* Request preferred connection parameters (faster interval for audio) */
        ai_send_connection_update(ai_ble_con_handle);
        break;

    /*
     * BLE disconnected.
     * Clean up — AI audio streaming stops.
     */
    case GATT_COMM_EVENT_DISCONNECT_COMPLETE:
        log_info("BLE disconnected: handle=%04x, reason=%02x\n",
            little_endian_read_16(packet, 0), packet[2]);
        if (ai_ble_con_handle == little_endian_read_16(packet, 0)) {
            ai_ble_con_handle = 0;
            /* BLE gone — stop streaming (app can't receive data anymore) */
            if (ai_stream_state == AI_STREAM_ON) {
                ai_stream_stop(AI_REASON_APP_CMD_STOP);
            }
        }
        break;

    /*
     * Encryption changed (re-connection with bonded device).
     * Re-enable notification subscriptions.
     */
    case GATT_COMM_EVENT_ENCRYPTION_CHANGE:
        log_info("Encryption change: handle=%04x, state=%d\n",
            little_endian_read_16(packet, 0), packet[2]);
        if (packet[3] == LINK_ENCRYPTION_RECONNECT) {
            ai_resume_all_ccc_enable(little_endian_read_16(packet, 0), 1);
        }
        break;

    /*
     * MTU exchange completed.
     * A larger MTU means we can send bigger audio chunks per BLE packet,
     * which is more efficient.
     */
    case GATT_COMM_EVENT_MTU_EXCHANGE_COMPLETE:
        log_info("MTU = %u bytes\n", little_endian_read_16(packet, 2));
        break;

    case GATT_COMM_EVENT_CONNECTION_UPDATE_COMPLETE:
        log_info("Conn params updated: interval=%d, latency=%d, timeout=%d\n",
            little_endian_read_16(ext_param, 6 + 0),
            little_endian_read_16(ext_param, 6 + 2),
            little_endian_read_16(ext_param, 6 + 4));
        break;

    case GATT_COMM_EVENT_CAN_SEND_NOW:
        /* BLE stack is ready to accept more data — resume sending if queued */
        break;

    default:
        break;
    }
    return 0;
}


/*
 * ============================================================================
 *  SECTION 13: EDR (Classic Bluetooth) Event Handler
 * ============================================================================
 *
 *  Handles HFP call events and Classic BT connection state.
 *  This is where phone call lifecycle events arrive.
 */
static int ai_bt_edr_status_handler(struct bt_event *bt)
{
    log_info("EDR event: %d\n", bt->event);

    switch (bt->event) {

    /*
     * BT_STATUS_PHONE_INCOME — Incoming phone call.
     * The phone is ringing. No audio yet.
     */
    case BT_STATUS_PHONE_INCOME:
        ai_on_call_incoming();
        break;

    /*
     * BT_STATUS_PHONE_OUT — Outgoing call dialed.
     * Call is being placed but not yet connected.
     */
    case BT_STATUS_PHONE_OUT:
        log_info("Outgoing call\n");
        current_call_state = AI_CALL_STATE_INCOMING; /* Treat as pre-active */
        break;

    /*
     * BT_STATUS_PHONE_ACTIVE — Call answered / connected.
     * SCO audio link is now established. This is the critical moment
     * where we start the audio mixer.
     */
    case BT_STATUS_PHONE_ACTIVE:
        ai_on_call_active();
        break;

    /*
     * BT_STATUS_PHONE_HANGUP — Call ended.
     * SCO link is torn down.
     */
    case BT_STATUS_PHONE_HANGUP:
        ai_on_call_hangup();
        break;

    /*
     * BT_STATUS_PHONE_NUMBER — Caller ID received.
     * bt->value contains the phone number string.
     */
    case BT_STATUS_PHONE_NUMBER:
        log_info("Caller number received\n");
        break;

    /* Classic BT connected to phone */
    case BT_STATUS_FIRST_CONNECTED:
    case BT_STATUS_SECOND_CONNECTED:
        log_info("EDR connected\n");
        break;

    /* Classic BT disconnected */
    case BT_STATUS_FIRST_DISCONNECT:
    case BT_STATUS_SECOND_DISCONNECT:
        log_info("EDR disconnected\n");
        if (current_call_state != AI_CALL_STATE_IDLE) {
            ai_on_call_hangup(); /* Force cleanup */
        }
        break;

    default:
        break;
    }
    return 0;
}


/*
 * ============================================================================
 *  SECTION 14: BLE Advertising Setup
 * ============================================================================
 *
 *  Configure how the earbuds advertise themselves so the phone app can find them.
 */
static u8 ai_adv_data[ADV_RSP_PACKET_MAX];
static u8 ai_scan_rsp_data[ADV_RSP_PACKET_MAX];
static adv_cfg_t ai_server_adv_config;

static int ai_make_set_adv_data(void)
{
    u8 offset = 0;
    u8 *buf = ai_adv_data;

    /*
     * Flags field — required in BLE advertising packets.
     * FLAGS_GENERAL_DISCOVERABLE_MODE: device is always discoverable
     * FLAGS_LE_AND_EDR_SAME_CONTROLLER: we support both BLE and Classic BT
     *   (important! this tells the phone we have EDR for HFP too)
     */
    offset += make_eir_packet_val(&buf[offset], offset,
        HCI_EIR_DATATYPE_FLAGS,
        FLAGS_GENERAL_DISCOVERABLE_MODE | FLAGS_LE_AND_EDR_SAME_CONTROLLER, 1);

    /* Advertise our custom service UUID 0xAF30 (recognized by your phone app) */
    offset += make_eir_packet_val(&buf[offset], offset,
        HCI_EIR_DATATYPE_COMPLETE_16BIT_SERVICE_UUIDS, 0xAF30, 2);

    /* Add device name if it fits */
    char *gap_name = ble_comm_get_gap_name();
    u8 name_len = strlen(gap_name);
    u8 avail = ADV_RSP_PACKET_MAX - (offset + 2);
    if (name_len <= avail) {
        offset += make_eir_packet_data(&buf[offset], offset,
            HCI_EIR_DATATYPE_COMPLETE_LOCAL_NAME, (void *)gap_name, name_len);
    } else if (avail > 0) {
        offset += make_eir_packet_data(&buf[offset], offset,
            HCI_EIR_DATATYPE_SHORTENED_LOCAL_NAME, (void *)gap_name, avail);
    }

    if (offset > ADV_RSP_PACKET_MAX) {
        log_info("ERROR: adv data too long (%d)\n", offset);
        return -1;
    }

    log_info("ADV data (%d bytes):\n", offset);
    ai_server_adv_config.adv_data_len = offset;
    ai_server_adv_config.adv_data = ai_adv_data;
    return 0;
}

static int ai_make_set_rsp_data(void)
{
    u8 offset = 0;
    u8 *buf = ai_scan_rsp_data;

    /* Put the full device name in the scan response */
    char *gap_name = ble_comm_get_gap_name();
    u8 name_len = strlen(gap_name);
    offset += make_eir_packet_data(&buf[offset], offset,
        HCI_EIR_DATATYPE_COMPLETE_LOCAL_NAME, (void *)gap_name, name_len);

    if (offset > ADV_RSP_PACKET_MAX) {
        return -1;
    }

    ai_server_adv_config.rsp_data_len = offset;
    ai_server_adv_config.rsp_data = ai_scan_rsp_data;
    return 0;
}


/*
 * ============================================================================
 *  SECTION 15: GATT Server Configuration Structures
 * ============================================================================
 *
 *  These structures are passed to the BLE stack at initialization.
 *  They register our callback functions for ATT read/write/events.
 */

/* No encryption for now — simplifies development.
 * For production, enable SM (Security Manager) for pairing/bonding. */
static const sm_cfg_t ai_sm_init_config = {
    .slave_security_auto_req = 0,
    .slave_set_wait_security = 0,
    .io_capabilities = IO_CAPABILITY_NO_INPUT_NO_OUTPUT,
    .authentication_req_flags = SM_AUTHREQ_BONDING | SM_AUTHREQ_MITM_PROTECTION,
    .min_key_size = 7,
    .max_key_size = 16,
    .sm_cb_packet_handler = NULL,
};

const gatt_server_cfg_t ai_server_init_cfg = {
    .att_read_cb = &ai_att_read_callback,
    .att_write_cb = &ai_att_write_callback,
    .event_packet_handler = &ai_event_packet_handler,
};

static gatt_ctrl_t ai_gatt_control_block = {
    .mtu_size = ATT_LOCAL_MTU_SIZE,
    .cbuffer_size = ATT_SEND_CBUF_SIZE,
    .multi_dev_flag = 0,

    .server_config = &ai_server_init_cfg,
    .client_config = NULL,

#if CONFIG_BT_SM_SUPPORT_ENABLE
    .sm_config = &ai_sm_init_config,
#else
    .sm_config = NULL,
#endif

    .hci_cb_packet_handler = NULL,
};


/*
 * ============================================================================
 *  SECTION 16: BLE Initialization and Advertising Start
 * ============================================================================
 */

void bt_ble_init(void)
{
    log_info("AI Earbuds BLE init\n");

    /* Register the GATT profile (service + characteristics table) */
    ble_gatt_server_set_profile(trans_profile_data, sizeof(trans_profile_data));

    /* Initialize the GATT controller with our config */
    ble_gatt_server_init(&ai_gatt_control_block);

    /* Build advertising and scan response data */
    ai_make_set_adv_data();
    ai_make_set_rsp_data();

    /* Set advertising parameters */
    ai_server_adv_config.adv_interval = ADV_INTERVAL_MIN;
    ai_server_adv_config.adv_auto_do = 1; /* Auto-restart advertising after disconnect */
    ai_server_adv_config.adv_channel = ADV_CHANNEL_ALL; /* Advertise on all 3 channels */

    /* Start advertising — phone app can now discover and connect */
    ble_gatt_server_adv_enable(1);
    ble_gatt_server_set_adv_config(&ai_server_adv_config);
}


void bt_ble_exit(void)
{
    log_info("AI Earbuds BLE exit\n");
    ble_gatt_server_adv_enable(0);
    ble_gatt_server_exit();
}


/*
 * ============================================================================
 *  SECTION 17: EDR (Classic Bluetooth) Event Registration
 * ============================================================================
 *
 *  The bt_comm_edr_status_event_handler in app_comm_edr.c dispatches events.
 *  For the AI earbuds use case, the existing handler in app_comm_edr.c needs
 *  to call ai_bt_edr_status_handler() for phone call events.
 *
 *  Alternatively, you can modify app_comm_edr.c directly.
 *  Below is a convenience function to handle the events.
 */

/**
 * Call this from bt_comm_edr_status_event_handler() in app_comm_edr.c
 * for phone-related events. Example modification:
 *
 *   case BT_STATUS_PHONE_INCOME:
 *   case BT_STATUS_PHONE_OUT:
 *   case BT_STATUS_PHONE_ACTIVE:
 *   case BT_STATUS_PHONE_HANGUP:
 *       ai_bt_edr_status_handler(bt);  // <-- add this call
 *       break;
 */
int ai_handle_edr_event(struct bt_event *bt)
{
    return ai_bt_edr_status_handler(bt);
}


/*
 * ============================================================================
 *  SECTION 18: Integration Hook for Audio Pipeline
 * ============================================================================
 *
 *  To stream mic + remote caller audio to the phone app over BLE,
 *  you need to hook into the audio pipeline.
 *
 *  In cpu/br23/audio_enc/audio_enc.c, the function adc_mic_output_handler()
 *  is called every time a mic PCM frame is ready. Add a call to
 *  ai_audio_send_to_ble() there:
 *
 *  static void adc_mic_output_handler(void *priv, s16 *data, int len)
 *  {
 *      #if (RECORDER_MIX_EN)
 *          recorder_mix_sco_data_write(data, len);
 *      #endif
 *
 *      // >>> ADD THIS: Send mic audio to BLE for cloud AI <<<
 *      extern void ai_audio_send_to_ble(u8 *pcm_data, u16 len);
 *      ai_audio_send_to_ble((u8 *)data, len);
 *
 *      audio_aec_inbuf(data, len);
 *  }
 *
 *  This captures raw mic data. To also capture the remote caller's voice,
 *  you would additionally hook the ESCO decoder output. The recorder_mix
 *  already combines both streams — check recorder_mix_sco_data_write() calls.
 */


/*
 * ============================================================================
 *  SECTION 19: Summary — What to Change in Existing SDK Files
 * ============================================================================
 *
 *  To use this example, you need to make these changes to existing files:
 *
 *  1. Board config (e.g., board_ac635n_demo_cfg.h):
 *     - #define TCFG_USER_EDR_ENABLE          1    // Was 0
 *     - #define TCFG_USER_BLE_ENABLE          1    // Already 1
 *     - #define USER_SUPPORT_PROFILE_HFP      1    // Was 0
 *     - #define USER_SUPPORT_PROFILE_SPP      1    // Optional, for extra data
 *     - #define TCFG_AUDIO_ENABLE             1
 *     - #define TCFG_AUDIO_ADC_ENABLE         ENABLE_THIS_MOUDLE
 *     - #define TCFG_AUDIO_DAC_ENABLE         ENABLE_THIS_MOUDLE
 *
 *  2. App config or board config:
 *     - #define RECORDER_MIX_EN               1    // Enable audio mixer
 *
 *  3. app_comm_edr.c — In bt_comm_edr_status_event_handler(), add:
 *     - extern int ai_handle_edr_event(struct bt_event *bt);
 *     - Call ai_handle_edr_event(bt) for phone call events.
 *
 *  4. audio_enc.c — In adc_mic_output_handler(), add:
 *     - extern void ai_audio_send_to_ble(u8 *pcm_data, u16 len);
 *     - ai_audio_send_to_ble((u8 *)data, len);
 *
 *  5. Build: make ac635n_spp_and_le
 *
 *  For the phone app (iOS):
 *  - Use CoreBluetooth to connect to the earbuds
 *  - Discover service 0xAE30
 *  - Subscribe to 0xAE02 notifications (receive call audio)
 *  - Write AI audio to 0xAE01 (send AI speech to earbuds)
 *  - Read/write 0xAE10 for status and commands
 *  - Forward audio to/from your cloud AI via WebSocket
 */

#endif /* CONFIG_APP_SPP_LE */
