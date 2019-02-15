#ifndef __maze_analysis_H
#define __maze_analysis_H
#ifdef __cplusplus
 extern "C" {
#endif


#include "maze_info.h"

void Maze_CreateMap(maze_t *maze);
void Maze_CreateFastMap(maze_t *maze);

#ifdef __cplusplus
}
#endif
#endif /*__maze_analysis_H */