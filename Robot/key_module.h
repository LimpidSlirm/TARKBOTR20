#ifndef KEY_MODULE_H
#define KEY_MODULE_H

#include <stdint.h>

typedef enum {
    KEY_1 = 0,
    KEY_2,
    KEY_3,
    KEY_4,
    KEY_COUNT
} key_id_t;

typedef enum {
    KEY_EVT_NONE = 0,
    KEY_EVT_SHORT,
    KEY_EVT_LONG
} key_event_t;

/* 每 1 ms 调用一次。 */
void key_module_scan(void);

/* 读取并清除该键的待取事件。无事件或 ID 无效时返回 KEY_EVT_NONE。 */
uint8_t key_module_get_event(uint8_t key_id);

#endif
