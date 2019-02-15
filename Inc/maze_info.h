#ifndef __maze_info_H
#define __maze_info_H
#ifdef __cplusplus
 extern "C" {
#endif

#define WALL_NUM 16

typedef struct {
    unsigned char goal_x;
    unsigned char goal_y;

	unsigned int wall_vertical[WALL_NUM];
	unsigned int wall_horizontal[WALL_NUM];
    
	unsigned int wall_ver_search[WALL_NUM];
	unsigned int wall_hor_search[WALL_NUM];

	unsigned char step[WALL_NUM][WALL_NUM];
}maze_t;


void Maze_Set(void);
void Maze_Init(maze_t *maze);
void Maze_Get_Wall(void);
void Maze_Printf(maze_t maze);



#ifdef __cplusplus
}
#endif
#endif /*__maze_info_H */