#ifndef __ASTAR_H__
#define __ASTAR_H__

#include "main.h"

/*==========================================================================
 * 工创赛 48x48 地图 (每格 50 mm, 整图 2400x2400 mm)
 *   0 可走    1 实体障碍(不可走)   2 启停区(可走，2 选 1)
 *   3 物料放置区(不可走)           4 物料夹取区(不可走)   5 扫码区(不可走)
 * 障碍膨胀不用"整格膨胀"，而用真实间隙判据:
 *   路径中心到实体障碍(1)的距离 >= 车半宽 + 余量，车才过得去
 *==========================================================================*/

#define MAP_N          48
#define MAP_CELLS      (MAP_N * MAP_N)
#define MAP_CELL_MM    50.0f

extern const char MAP_DATA[MAP_N][MAP_N + 1];

/* 路径中心离实体障碍的最小格数 (切比雪夫)，由车宽决定，见 Nav.h */
extern uint8_t g_map_clear_cells;

void    Map_Init(void);
uint8_t Map_Free(int r, int c);          /* 1 = 可规划 */
uint8_t Map_Kind(int r, int c);          /* 原始 0..5 */
uint8_t Map_Dist(int r, int c);          /* 到实体障碍的切比雪夫格数 */

/* 返回路径格数(含起终点), 结果写入 out(成对 r,c); 失败返回 -1 */
int     AStar_Search(int sr, int sc, int gr, int gc, uint8_t *out, int max_cells);

#endif
