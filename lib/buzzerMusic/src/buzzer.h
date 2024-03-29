#include <Arduino.h>

#define BUZZER_H

#define NTD0 -1
#define NTD1 294
#define NTD2 330
#define NTD3 350
#define NTD4 393
#define NTD5 441
#define NTD6 495
#define NTD7 556

#define NTDL1 147
#define NTDL2 165
#define NTDL3 175
#define NTDL4 196
#define NTDL5 221
#define NTDL6 248
#define NTDL7 278

#define NTDH1 589
#define NTDH2 661
#define NTDH3 700
#define NTDH4 786
#define NTDH5 882
#define NTDH6 990
#define NTDH7 112
//列出全部D调的频率
#define WHOLE 1
#define HALF 0.5
#define QUARTER 0.25
#define EIGHTH 0.25
#define SIXTEENTH 0.625

//列出所有节拍
int tune[] = {              //根据简谱列出各频率
    NTD3,NTD3,NTD4,NTD5,
    NTD5,NTD4,NTD3,NTD2,
    NTD1,NTD1,NTD2,NTD3,
    NTD3,NTD2,NTD2,
    NTD3,NTD3,NTD4,NTD5,
    NTD5,NTD4,NTD3,NTD2,
    NTD1,NTD1,NTD2,NTD3,
    NTD2,NTD1,NTD1,
    NTD2,NTD2,NTD3,NTD1,
    NTD2,NTD3,NTD4,NTD3,NTD1,
    NTD2,NTD3,NTD4,NTD3,NTD2,
    NTD1,NTD2,NTDL5,NTD0,
    NTD3,NTD3,NTD4,NTD5,
    NTD5,NTD4,NTD3,NTD4,NTD2,
    NTD1,NTD1,NTD2,NTD3,
    NTD2,NTD1,NTD1
};

float durt[]=              //根据简谱列出各节拍
{
    1,1,1,1,
    1,1,1,1,
    1,1,1,1,
    1+0.5,0.5,1+1,
    1,1,1,1,
    1,1,1,1,
    1,1,1,1,
    1+0.5,0.5,1+1,
    1,1,1,1,
    1,0.5,0.5,1,1,
    1,0.5,0.5,1,1,
    1,1,1,1,
    1,1,1,1,
    1,1,1,0.5,0.5,
    1,1,1,1,
    1+0.5,0.5,1+1,
};

bool buzzerStopFlag = false;

// BUG 可以接收创建任务时传递的参数，但是不能使用
void buzzerLoop(void *ptParam) {
    // uint8_t _pin = *((uint8_t *)ptParam);
    // Serial.printf("buzzer pin: %d", pin);
    int length = sizeof(tune) / sizeof(tune[0]);   //计算长度
    ledcAttachPin(4, 1);
    while(true) {
        for(int x = 0; x < length; x++) {
            if (buzzerStopFlag) {
                // ledcWriteTone(1, 0);  
                break;
            }
            ledcWriteTone(1, tune[x]);  
            // tone(tonepin,tune[x]);
            vTaskDelay(500 * durt[x]);   //这里用来根据节拍调节延时，500这个指数可以自己调整，在该音乐中，我发现用500比较合适。
            // noTone(tonepin);
        }
        buzzerStopFlag = true;
    }
}

