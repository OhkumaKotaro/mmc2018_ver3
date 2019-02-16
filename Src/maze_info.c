#include "maze_info.h"
#include "filter.h"
#include "usart.h"

#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */


#define TRUE 1
#define FALSE 0

#define MAX_STEP 255

//direction
#define NORTH 0
#define EAST 1
#define SOUTH 2
#define WEST 3

extern sensor_t sen_l;
extern sensor_t sen_front;
extern sensor_t sen_r;

maze_t maze;
unsigned char position;//1~4bit x , 5~8bit y
unsigned char direction;

/****************************************************************************************************
 * outline : input maze size , goal point
 * argument : void
 * return : void
******************************************************************************************************/
void Maze_Set(void){
    maze.goal_x = 4;
    maze.goal_y = 7;
}



/****************************************************************************************************
 * outline : initialize maze date
 * argument : void
 * return : void
******************************************************************************************************/

void Maze_Init(maze_t *maze) {
	for (unsigned char i = 0; i < WALL_NUM; i++)
	{
		maze->wall_vertical[i] = 0;
		maze->wall_horizontal[i] = 0;
		maze->wall_ver_search[i] = 0;
		maze->wall_hor_search[i] = 0;
		for (unsigned char j = 0; j < WALL_NUM; j++)
		{
			maze->step[i][j] = MAX_STEP;
		}
	}
	maze->wall_vertical[WALL_NUM - 1] = 0xffff;
	maze->wall_ver_search[WALL_NUM - 1] = 0xffff;

	maze->wall_horizontal[WALL_NUM - 1] = 0xffff;
	maze->wall_hor_search[WALL_NUM - 1] = 0xffff;

	maze->wall_vertical[0] += 0b1;
	maze->wall_ver_search[0] += 0b1;
}




void Maze_Get_Wall(void){
	unsigned char n_wall=0, e_wall=0, s_wall=0, w_wall=0;

    unsigned char x=position>>4&0b1111;
    unsigned char y=position&0b1111;

	// 方向別に壁の状態を取得
	switch(direction){
	case NORTH:
		n_wall = sen_front.is_wall;
		e_wall = sen_r.is_wall;
		w_wall = sen_l.is_wall;
		s_wall = 0;
		break;
	case EAST:
		e_wall = sen_front.is_wall;
		s_wall = sen_r.is_wall;
		n_wall = sen_l.is_wall;
		w_wall = 0;
		break;
	case SOUTH:
		s_wall = sen_front.is_wall;
		w_wall = sen_r.is_wall;
		e_wall = sen_l.is_wall;
		n_wall = 0;
		break;
	case WEST:
		w_wall = sen_front.is_wall;
		n_wall = sen_r.is_wall;
		s_wall = sen_l.is_wall;
		e_wall = 0;
		break;
	default:
		break;
	}
	// 方向別に保存
	maze.wall_horizontal[y] = maze.wall_horizontal[y]|(n_wall<<x);
    if(y>0){
        maze.wall_horizontal[y-1] = maze.wall_horizontal[y-1]|(s_wall<<x);
    }
	maze.wall_vertical[x] = maze.wall_vertical[x]|(e_wall<<y);
    if(x>0){
	    maze.wall_vertical[x-1] = maze.wall_vertical[x-1]|(w_wall<<y);
    }
	
	maze.wall_hor_search[y] = maze.wall_hor_search[y] | (1 << x);
	if (y > 0) {
		maze.wall_hor_search[y - 1] = maze.wall_hor_search[y - 1] | (1 << x);
	}
	maze.wall_ver_search[x] = maze.wall_ver_search[x] | (1 << y);
	if (x > 0) {
		maze.wall_ver_search[x - 1] = maze.wall_ver_search[x - 1] | (1 << y);
	}
}


void Maze_Printf(maze_t maze) {
	printf("\r\n");
	for (unsigned char j = WALL_NUM; j > 0; j--)
	{
		for (unsigned char i = 0; i < WALL_NUM; i++)
		{
			printf("+");
			if (maze.wall_horizontal[j-1] & 0b1 << i)
			{
				printf("---");
			}
			else
			{
				printf("   ");
			}
		}
		printf("+\r\n");
		printf("|");
		for (unsigned char i = 0; i < WALL_NUM; i++)
		{
			printf("%3d", maze.step[i][j-1]);
			if (maze.wall_vertical[i] & 0b1 << (j - 1)) {
				printf("|");
			}
			else
			{
				printf(" ");
			}
		}
		printf("\r\n");

	}
	for (unsigned char i = 0; i < WALL_NUM; i++)
	{
		printf("+");
		printf("---");
	}
	printf("+\r\n");
}