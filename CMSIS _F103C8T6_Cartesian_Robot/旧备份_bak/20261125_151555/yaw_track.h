/******************************************************************************
 * yaw_track.h  --  视觉极角闭环：把画面方位角积分成 yaw 目标角（F103 侧）
 *
 * 摄像头俯视、yaw 绕垂直轴旋转时，目标在画面里绕圈，dx/dy 都会变，
 * 所以误差取极角 phi = atan2(dx, -dy)，而不是单独的 dx 或 dy。
 * phi = 0 表示目标在画面正上方；输出累加到 yaw 目标角，交给 Link 层发 <<Y,..>>。
 * 全部使用定点整数（无浮点、无 libm），10ms 调用一次。
 ******************************************************************************/
#ifndef __YAW_TRACK_H
#define __YAW_TRACK_H

#include "stm32f10x.h"

void    YawTrack_Init(void);
void    YawTrack_Update(int16_t dx, int16_t dy, uint8_t valid);
int32_t YawTrack_GetTargetMrad(void);

#endif
