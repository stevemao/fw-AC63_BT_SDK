/*********************************************************************************************
 *   Filename        : ble_ai_earbuds.h
 *
 *   Description     : Header for AI Earbuds example.
 *                     Includes required board configuration reference.
 *
 *   Copyright:(c)JIELI  2011-2026  @ , All Rights Reserved.
 *********************************************************************************************/

#ifndef _BLE_AI_EARBUDS_H
#define _BLE_AI_EARBUDS_H

#include <stdint.h>
#include "app_config.h"
#include "gatt_common/le_gatt_common.h"

/*
 * ============================================================================
 *  Required Board Configuration Changes
 * ============================================================================
 *
 *  In your board config header (e.g., board_ac635n_demo_cfg.h), set:
 *
 *  // Enable both Classic BT (for HFP phone calls) and BLE (for app data)
 *  #define TCFG_USER_EDR_ENABLE                1
 *  #define TCFG_USER_BLE_ENABLE                1
 *
 *  // Enable HFP so the earbuds can handle phone calls
 *  #if TCFG_USER_EDR_ENABLE
 *  #define USER_SUPPORT_PROFILE_HFP            1
 *  #define USER_SUPPORT_PROFILE_SPP            1  // Optional data channel
 *  #define USER_SUPPORT_PROFILE_A2DP           0  // No music streaming needed
 *  #define USER_SUPPORT_PROFILE_AVCTP          0
 *  #define USER_SUPPORT_PROFILE_HID            0
 *  #endif
 *
 *  // Enable audio hardware (mic + speaker)
 *  #define TCFG_AUDIO_ENABLE                   1
 *  #define TCFG_AUDIO_ADC_ENABLE               ENABLE_THIS_MOUDLE  // Mic input
 *  #define TCFG_AUDIO_DAC_ENABLE               ENABLE_THIS_MOUDLE  // Speaker output
 *
 *  // Enable the audio mixer (critical for AI audio injection into calls)
 *  #define RECORDER_MIX_EN                     1
 *
 *  // Enable in-ear detection (TRIGGER 1: start on ear-in, stop on ear-out)
 *  #define TCFG_LP_EARTCH_KEY_ENABLE           1
 *  #define TCFG_EARTCH_EVENT_HANDLE_ENABLE     1
 *
 *  // Enable keyword spotting / wake word (TRIGGER 2: start on "Hey AI")
 *  #define TCFG_KWS_VOICE_RECOGNITION_ENABLE   1
 *
 *  // Optional: OTA update support
 *  #define CONFIG_APP_OTA_ENABLE               1
 *  #define CONFIG_DOUBLE_BANK_ENABLE           1
 */


/*
 * ============================================================================
 *  BLE ↔ Phone App Protocol Definition
 * ============================================================================
 *
 *  GATT Service UUID: 0xAE30
 *
 *  Characteristics:
 *
 *  ┌────────┬──────────────────────┬────────────────────────────────┐
 *  │ UUID   │ Properties           │ Purpose                        │
 *  ├────────┼──────────────────────┼────────────────────────────────┤
 *  │ 0xAE01 │ WRITE_WITHOUT_RESP   │ App → Earbuds: AI audio (PCM)  │
 *  │ 0xAE02 │ NOTIFY               │ Earbuds → App: Call audio      │
 *  │ 0xAE04 │ NOTIFY               │ Earbuds → App: State change    │
 *  │ 0xAE10 │ READ | WRITE         │ Control register               │
 *  └────────┴──────────────────────┴────────────────────────────────┘
 *
 *  Audio Format (recommended):
 *  - Sample rate: 8000 Hz (matches SCO CVSD)
 *  - Bit depth:   16-bit signed, little-endian
 *  - Channels:    1 (mono)
 *  - Encoding:    Raw PCM (or Opus 16kbps for bandwidth efficiency)
 *
 *  Control Register (0xAE10):
 *    Write byte 0:
 *      0x01 = Start AI streaming (TRIGGER 3: app button)
 *      0x00 = Stop AI streaming  (TRIGGER 3: app button)
 *      0x02 = Query current state
 *    Read bytes:
 *      [0] = streaming state (0=OFF, 1=ON)
 *      [1] = call state      (0=idle, 1=incoming, 2=active)
 *
 *  State Notifications (0xAE04) — push notification on state change:
 *      [0] = streaming state (0=OFF, 1=ON)
 *      [1] = call state      (0=idle, 1=incoming, 2=active)
 *
 *  Start/Stop Triggers Summary:
 *
 *  START triggers (any one activates streaming):
 *    1. In-ear detection — hardware sensor detects earbuds placed in ear
 *    2. Wake word        — user says keyword (e.g., "Hey AI")
 *    3. App button       — app writes 0x01 to 0xAE10
 *
 *  STOP triggers (any one deactivates streaming):
 *    1. Ear removal      — sensor detects earbuds removed from ear
 *    2. 5-min silence    — no voice detected for 5 minutes
 *    3. App button       — app writes 0x00 to 0xAE10
 *    4. Call hangup      — phone call ends (automatic)
 *    5. BLE disconnect   — phone app disconnects (automatic)
 */

/* Streaming state values */
#define AI_STREAM_STATE_OFF   0x00
#define AI_STREAM_STATE_ON    0x01

/* Call state values sent over BLE */
#define AI_STATUS_IDLE       0x00
#define AI_STATUS_INCOMING   0x01
#define AI_STATUS_ACTIVE     0x02

/* Control commands received from phone app */
#define AI_CMD_STOP          0x00
#define AI_CMD_START         0x01
#define AI_CMD_QUERY         0x02

/* Reason codes — tells the phone app WHY streaming started/stopped */
#define AI_REASON_EAR_ON        0x01  /* Earbuds put on ear */
#define AI_REASON_WAKE_WORD     0x02  /* Wake word detected */
#define AI_REASON_APP_CMD       0x03  /* Phone app sent start command */
#define AI_REASON_EAR_OFF       0x11  /* Earbuds removed from ear */
#define AI_REASON_SILENCE       0x12  /* 5 minutes of silence */
#define AI_REASON_APP_CMD_STOP  0x13  /* Phone app sent stop command */


/*
 * ============================================================================
 *  Public API
 * ============================================================================
 */

/* Initialize BLE GATT server and start advertising */
extern void bt_ble_init(void);
extern void bt_ble_exit(void);

/* Call from app_comm_edr.c for HFP phone call events */
extern int ai_handle_edr_event(struct bt_event *bt);

/* Call from audio_enc.c to stream mic audio to BLE */
extern void ai_audio_send_to_ble(u8 *pcm_data, u16 len);

/* Call from ear detection module when ear state changes */
extern void ai_ear_detect_handler(u8 event);

/* Call from KWS module when a keyword is detected */
extern void ai_kws_event_handler(u8 voice_event);


#endif /* _BLE_AI_EARBUDS_H */
