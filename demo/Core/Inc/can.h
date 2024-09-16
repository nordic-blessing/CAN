//
// Created by 30958 on 2024/9/16.
//

#ifndef DEMO_CAN_H
#define DEMO_CAN_H

#include "stm32f1xx.h"

// 滤波器编号
#define CAN_FILTER(x) ((x) << 3)

// 接收队列
#define CAN_FIFO_0 (0 << 2)
#define CAN_FIFO_1 (1 << 2)

//标准帧或扩展帧
#define CAN_STDID (0 << 1)
#define CAN_EXTID (1 << 1)

// 数据帧或遥控帧
#define CAN_DATA_TYPE (0 << 0)
#define CAN_REMOTE_TYPE (1 << 0)

void CAN_Init(CAN_HandleTypeDef *hcan);
#endif //DEMO_CAN_H
