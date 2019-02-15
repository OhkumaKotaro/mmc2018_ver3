#ifndef __maze_plan_H
#define __maze_plan_H
#ifdef __cplusplus
 extern "C" {
#endif

//motion 
#define LEFT 0
#define FRONT 1
#define RIGHT 2
#define UTURN 3
#define KABEATE 4


void Update_Position(unsigned char next_motion);
unsigned char Maze_Next_Motion(void);
void Plan_Adachi(void);
void Plan_Root(void);
void Plan_Fast(void);



#ifdef __cplusplus
}
#endif
#endif /*__maze_info_H */