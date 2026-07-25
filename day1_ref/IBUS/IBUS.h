#ifndef IBUS_H_
#define IBUS_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

//
typedef struct
{

    int16_t ch0;
    int16_t ch1;
    int16_t ch2;
    int16_t ch3;


    uint8_t s1;
    uint8_t s2;
    

}Remote_t;


extern volatile Remote_t Remote;
extern uint8_t DBUS_RX_Buffer[18];

//IBUS 解析函数（回调函数调用）
void Remote_DBUS_to_RC(uint8_t *data);

#ifdef __cplusplus
}
#endif

#endif
