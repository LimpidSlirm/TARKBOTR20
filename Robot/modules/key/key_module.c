#include "key_module.h"

/* 由板级 GPIO 驱动提供。 */
extern uint8_t gpio_get_level(uint8_t pin);

/* 板级适配的通道号，当前仅通道 0 对应板载 PE0。 */
#define KEY_1_PIN                 0u
#define KEY_2_PIN                 1u
#define KEY_3_PIN                 2u
#define KEY_4_PIN                 3u
#define KEY_ACTIVE_LEVEL          0u

/* 调用间隔需与主程序的实际调度周期一致。 */
#define KEY_CALL_PERIOD_MS        1u
#define KEY_SCAN_PERIOD_MS        5u
#define KEY_DEBOUNCE_MS          20u
#define KEY_LONG_PRESS_MS       800u

#if KEY_CALL_PERIOD_MS == 0u || KEY_SCAN_PERIOD_MS == 0u
#error "Key call and scan periods must be positive"
#endif

#if (KEY_SCAN_PERIOD_MS % KEY_CALL_PERIOD_MS) != 0u
#error "Key scan period must be a multiple of the call period"
#endif

#if KEY_DEBOUNCE_MS == 0u || (KEY_DEBOUNCE_MS % KEY_SCAN_PERIOD_MS) != 0u
#error "Key debounce time must be a positive multiple of the scan period"
#endif

#if KEY_LONG_PRESS_MS == 0u || (KEY_LONG_PRESS_MS % KEY_SCAN_PERIOD_MS) != 0u
#error "Key long-press time must be a positive multiple of the scan period"
#endif

#define KEY_SCAN_CALLS        (KEY_SCAN_PERIOD_MS / KEY_CALL_PERIOD_MS)
#define KEY_DEBOUNCE_SAMPLES  (KEY_DEBOUNCE_MS / KEY_SCAN_PERIOD_MS)
#define KEY_LONG_SAMPLES      (KEY_LONG_PRESS_MS / KEY_SCAN_PERIOD_MS)

static const uint8_t key_pins[KEY_COUNT] = {
    [KEY_1] = KEY_1_PIN,
    [KEY_2] = KEY_2_PIN,
    [KEY_3] = KEY_3_PIN,
    [KEY_4] = KEY_4_PIN
};

typedef struct {
    uint8_t stable_pressed;
    uint8_t long_reported;
    uint8_t pending_event;
    uint32_t debounce_count;
    uint32_t hold_count;
} key_state_t;

/* 静态变量初始为零，各键从“稳定松开、无待取事件”开始。 */
static key_state_t key_states[KEY_COUNT];
static uint32_t scan_call_count;

static void key_publish_event(key_state_t *state, uint8_t event)
{
    /* 每键保留一个待取事件，已有事件时保留较早的一条。 */
    if (state->pending_event == KEY_EVT_NONE) {
        state->pending_event = event;
    }
}

void key_module_scan(void)
{
    uint8_t key_id;

    ++scan_call_count;
    if (scan_call_count < KEY_SCAN_CALLS) {
        return;
    }
    scan_call_count = 0u;

    for (key_id = 0u; key_id < KEY_COUNT; ++key_id) {
        key_state_t *state = &key_states[key_id];
        uint8_t raw_pressed =
            (gpio_get_level(key_pins[key_id]) == KEY_ACTIVE_LEVEL);

        /*
         * 双边消抖：原始状态连续多次与稳定状态相反，
         * 才确认按下或松开。中途恢复则清零消抖计数。
         */
        if (raw_pressed == state->stable_pressed) {
            state->debounce_count = 0u;
        } else {
            ++state->debounce_count;
            if (state->debounce_count >= KEY_DEBOUNCE_SAMPLES) {
                state->debounce_count = 0u;
                state->stable_pressed = raw_pressed;

                if (raw_pressed) {
                    /* 稳定松开 → 稳定按下：开始新一轮长按计时。 */
                    state->hold_count = 0u;
                    state->long_reported = 0u;
                } else if (!state->long_reported) {
                    /* 稳定按下 → 稳定松开：本轮未触发长按，产生短按。 */
                    key_publish_event(state, KEY_EVT_SHORT);
                }
                continue;
            }
        }

        if (state->stable_pressed && !state->long_reported) {
            if (state->hold_count < KEY_LONG_SAMPLES) {
                ++state->hold_count;
            }

            /*
             * 长按到时且当前采样仍为按下，才产生长按事件。
             * 松开消抖期间若读到松开，暂缓判定。
             * 触发后锁存，直到下一次确认按下时重新启用。
             */
            if (raw_pressed && state->hold_count >= KEY_LONG_SAMPLES) {
                state->long_reported = 1u;
                key_publish_event(state, KEY_EVT_LONG);
            }
        }
    }
}

uint8_t key_module_get_event(uint8_t key_id)
{
    uint8_t event;

    if (key_id >= KEY_COUNT) {
        return KEY_EVT_NONE;
    }

    event = key_states[key_id].pending_event;
    key_states[key_id].pending_event = KEY_EVT_NONE;
    return event;
}
