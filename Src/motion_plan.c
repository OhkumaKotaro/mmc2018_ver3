#include "motion_plan.h"
#include "maze_info.h"
#include "maze_analysis.h"
#include "motion.h"
#include "filter.h"
#include "stm32f4xx_hal.h"
#include "spi.h"
#include "interface.h"
#include "tim.h"

#define TRUE 1
#define FALSE 0

#define MAX_STEP 255
#define MAZE_SIZE 16

//direction
#define NORTH 0
#define EAST 1
#define SOUTH 2
#define WEST 3

extern unsigned char position; //1~4bit x , 5~8bit y
extern unsigned char direction;

extern maze_t maze;

extern sensor_t sen_front;

extern uint8_t position;
extern uint8_t direction;

unsigned char flag_search = FALSE;

extern uint8_t flag_motor;

extern enc_t enc;

typedef struct
{
	unsigned char data[MAX_STEP];
	unsigned int head;
	unsigned int tail;
} queue_motion_t;
queue_motion_t q_motion;

void Que_init_motion(queue_motion_t *q)
{
	int i;

	q->head = 0;
	q->tail = 0;
	for (i = 0; i < MAX_STEP; ++i)
	{
		q->data[i] = 0;
	}
}

unsigned char enq_motion(queue_motion_t *q, unsigned char item)
{
	if (q->tail >= MAX_STEP)
	{
		return MAX_STEP;
	}
	else
	{
		q->data[q->tail] = item;
		q->tail++;
	}
	return 1;
}

unsigned char deq_motion(queue_motion_t *q)
{
	unsigned char tmp;
	tmp = q->data[q->head];
	q->head++;
	return tmp;
}

void Update_Position(unsigned char next_motion)
{
	switch (next_motion)
	{
	case LEFT:
		switch (direction)
		{
		case NORTH:
			position -= 1 << 4;
			direction = WEST;
			break;
		case EAST:
			position += 1;
			direction = NORTH;
			break;
		case SOUTH:
			position += 1 << 4;
			direction = EAST;
			break;
		case WEST:
			position -= 1;
			direction = SOUTH;
			break;
		}
		break;
	case FRONT:
		switch (direction)
		{
		case NORTH:
			position += 1;
			break;
		case EAST:
			position += 1 << 4;
			break;
		case SOUTH:
			position -= 1;
			break;
		case WEST:
			position -= 1 << 4;
			break;
		}
		break;
	case RIGHT:
		switch (direction)
		{
		case NORTH:
			position += 1 << 4;
			direction = EAST;
			break;
		case EAST:
			position -= 1;
			direction = SOUTH;
			break;
		case SOUTH:
			position -= 1 << 4;
			direction = WEST;
			break;
		case WEST:
			position += 1;
			direction = NORTH;
			break;
		}
		break;
	case UTURN:
		switch (direction)
		{
		case NORTH:
			position -= 1;
			direction = SOUTH;
			break;
		case EAST:
			position -= 1 << 4;
			direction = WEST;
			break;
		case SOUTH:
			position += 1;
			direction = NORTH;
			break;
		case WEST:
			position += 1 << 4;
			direction = EAST;
			break;
		}
		break;
	case KABEATE:
		switch (direction)
		{
		case NORTH:
			position -= 1;
			direction = SOUTH;
			break;
		case EAST:
			position -= 1 << 4;
			direction = WEST;
			break;
		case SOUTH:
			position += 1;
			direction = NORTH;
			break;
		case WEST:
			position += 1 << 4;
			direction = EAST;
			break;
		}
		break;
	}
}

unsigned char Maze_Next_Motion(void)
{
	/* <概要>   : 次の動作を決定
	* <引数>   : なし
	* <戻り値> : 次の動作
	*/
	unsigned char tmp_step = MAX_STEP; // 歩数
	unsigned char tmp_dir = UTURN;	 // 方向
									   // 現在の向きに応じて場合分けし、 歩数が少ない方向を判断
									   // 迷路外に進むのとゴールがスタートマス以外の場合(0,0)に進むのを阻止

	unsigned char x = position >> 4 & 0b1111;
	unsigned char y = position & 0b1111;

	switch (direction)
	{
	case NORTH:
		if (maze.step[x][y + 1] < tmp_step)
		{
			if (y < MAZE_SIZE - 1)
			{
				if ((maze.wall_horizontal[y] >> x & 0b1) == FALSE)
				{
					tmp_step = maze.step[x][y + 1];
					tmp_dir = FRONT;
				}
			}
		}
		if (maze.step[x - 1][y] < tmp_step)
		{
			if (x > 0)
			{
				if ((maze.wall_vertical[x - 1] >> y & 0b1) == FALSE)
				{
					tmp_step = maze.step[x - 1][y];
					tmp_dir = LEFT;
				}
			}
		}
		if (maze.step[x + 1][y] < tmp_step)
		{
			if (x < MAZE_SIZE - 1)
			{
				if ((maze.wall_vertical[x] >> y & 0b1) == FALSE)
				{
					tmp_step = maze.step[x + 1][y];
					tmp_dir = RIGHT;
				}
			}
		}
		if (tmp_step == MAX_STEP)
		{
			if ((maze.wall_horizontal[y] >> x & 0b1) == FALSE)
			{
				tmp_dir = UTURN;
			}
			else
			{
				tmp_dir = KABEATE;
			}
		}
		break;
	case EAST:
		if (maze.step[x + 1][y] < tmp_step)
		{
			if (x < MAZE_SIZE - 1)
			{
				if ((maze.wall_vertical[x] >> y & 0b1) == FALSE)
				{
					tmp_step = maze.step[x + 1][y];
					tmp_dir = FRONT;
				}
			}
		}
		if (maze.step[x][y + 1] < tmp_step)
		{
			if (y < MAZE_SIZE - 1)
			{
				if ((maze.wall_horizontal[y] >> x & 0b1) == FALSE)
				{
					tmp_step = maze.step[x][y + 1];
					tmp_dir = LEFT;
				}
			}
		}
		if (maze.step[x][y - 1] < tmp_step)
		{
			if (y > 0)
			{
				if ((maze.wall_horizontal[y - 1] >> x & 0b1) == FALSE)
				{
					tmp_step = maze.step[x][y - 1];
					tmp_dir = RIGHT;
				}
			}
		}
		if (tmp_step == MAX_STEP)
		{
			if ((maze.wall_vertical[x] >> y & 0b1) == FALSE)
			{
				tmp_dir = UTURN;
			}
			else
			{
				tmp_dir = KABEATE;
			}
		}
		break;
	case SOUTH:
		if (maze.step[x][y - 1] < tmp_step)
		{
			if (y > 0)
			{
				if ((maze.wall_horizontal[y - 1] >> x & 0b1) == FALSE)
				{
					tmp_step = maze.step[x][y - 1];
					tmp_dir = FRONT;
				}
			}
		}
		if (maze.step[x + 1][y] < tmp_step)
		{
			if (x < MAZE_SIZE - 1)
			{
				if ((maze.wall_vertical[x] >> y & 0b1) == FALSE)
				{
					tmp_step = maze.step[x + 1][y];
					tmp_dir = LEFT;
				}
			}
		}
		if (maze.step[x - 1][y] < tmp_step)
		{
			if (x > 0)
			{
				if ((maze.wall_vertical[x - 1] >> y & 0b1) == FALSE)
				{
					tmp_step = maze.step[x - 1][y];
					tmp_dir = RIGHT;
				}
			}
		}
		if (tmp_step == MAX_STEP)
		{
			if ((maze.wall_horizontal[y - 1] >> x & 0b1) == FALSE)
			{
				tmp_dir = UTURN;
			}
			else
			{
				tmp_dir = KABEATE;
			}
		}
		break;
	case WEST:
		if (maze.step[x - 1][y] < tmp_step)
		{
			if (x > 0)
			{
				if ((maze.wall_vertical[x - 1] >> y & 0b1) == FALSE)
				{
					tmp_step = maze.step[x - 1][y];
					tmp_dir = FRONT;
				}
			}
		}
		if (maze.step[x][y - 1] < tmp_step)
		{
			if (y > 0)
			{
				if ((maze.wall_horizontal[y - 1] >> x & 0b1) == FALSE)
				{
					tmp_step = maze.step[x][y - 1];
					tmp_dir = LEFT;
				}
			}
		}
		if (maze.step[x][y + 1] < tmp_step)
		{
			if (y < MAZE_SIZE - 1)
			{
				if ((maze.wall_horizontal[y] >> x & 0b1) == FALSE)
				{
					tmp_step = maze.step[x][y + 1];
					tmp_dir = RIGHT;
				}
			}
		}
		if (tmp_step == MAX_STEP)
		{
			if ((maze.wall_vertical[x - 1] >> y & 0b1) == FALSE)
			{
				tmp_dir = UTURN;
			}
			else
			{
				tmp_dir = KABEATE;
			}
		}
		break;
	default:
		break;
	}
	return tmp_dir;
}

void Plan_Adachi(void)
{
	unsigned char flag_goal_is = FALSE;
	unsigned char next_dir;

	adcStart();
	while (sen_front.is_wall == FALSE)
	{
	}
	Output_Buzzer(HZ_G);
	HAL_Delay(2500);
	gyro_offset_calc_reset();
	HAL_Delay(2500);

	Maze_Init(&maze);

	flag_motor = TRUE;

	position = 0b1;
	direction = NORTH;
	Maze_Set();

	Motion_Start();

	while (flag_goal_is == FALSE)
	{
		enc.offset = 0;
		Maze_Get_Wall();
		Maze_CreateMap(&maze);
		next_dir = Maze_Next_Motion();
		Update_Position(next_dir);
		if (position == (maze.goal_x << 4 | maze.goal_y))
		{
			flag_goal_is = TRUE;
		}

		switch (next_dir)
		{
		case LEFT:
			//Motion_Left();
			Motion_SlalomLeft();
			break;

		case FRONT:
			Motion_Straight();
			break;

		case RIGHT:
			//Motion_Right();
			Motion_SlalomRight();
			break;

		case UTURN:
			Motion_Uturn();
			break;

		case KABEATE:
			Motion_Kabeate();
			break;
		}
	}
	Maze_Get_Wall();
	Motion_Goal();
	flag_motor = FALSE;
}

void Plan_AllSearch(void)
{
	unsigned char end_flag = 0;

	Output_Buzzer(HZ_G);
	flag_motor = TRUE;

	if (sen_front.is_wall == FALSE)
	{
		Straight_half_accel();
	}
	else
	{
		switch (direction)
		{
		case NORTH:
			direction = SOUTH;
			position--;
			break;
		case SOUTH:
			direction = NORTH;
			position++;
			break;
		case EAST:
			direction = WEST;
			position = position - (0b1 << 4);
			break;
		case WEST:
			direction = EAST;
			position = position + (0b1 << 4);
			break;
		default:
			break;
		}
		Motion_Restart();
	}

	while (1)
	{
		enc.offset = 0;
		unsigned char next_dir;
		Maze_Get_Wall();
		end_flag = Maze_CreateAllMap(&maze);
		if (end_flag == 1)
		{
			break;
		}
		next_dir = Maze_Next_Motion();
		Update_Position(next_dir);
		switch (next_dir)
		{
		case LEFT:
			//Motion_Left();
			Motion_SlalomLeft();
			break;

		case FRONT:
			Motion_Straight();
			break;

		case RIGHT:
			//Motion_Right();
			Motion_SlalomRight();
			break;

		case UTURN:
			Motion_Uturn();
			break;

		case KABEATE:
			Motion_Kabeate();
			break;
		}
	}
	Motion_Goal();
	flag_motor = FALSE;
}

void Plan_Root(void)
{
	unsigned char flag_goal_is = FALSE;

	Maze_CreateFastMap(&maze);

	Que_init_motion(&q_motion);
	position = 0b1;
	direction = NORTH;
	enq_motion(&q_motion, START);
	while (flag_goal_is == FALSE)
	{
		unsigned char tmp;
		tmp = Maze_Next_Motion();
		Update_Position(tmp);
		enq_motion(&q_motion, tmp);
		if (position == (maze.goal_x << 4 | maze.goal_y))
		{
			flag_goal_is = TRUE;
			enq_motion(&q_motion, GOAL);
		}
	}
}

void Plan_Compress(void)
{
	queue_motion_t q_buff;
	unsigned motion;

	Que_init_motion(&q_buff);
	motion = deq_motion(&q_motion);

	while (q_motion.head != q_motion.tail)
	{
		unsigned char buff = 0;
		switch (motion)
		{
		case START: //5
			motion = deq_motion(&q_motion);
			while (motion == FRONT && q_motion.head != q_motion.tail)
			{
				buff++;
				motion = deq_motion(&q_motion);
			}
			buff = START << 4 | buff;
			enq_motion(&q_buff, buff);
			break;
		case FRONT:
			while (motion == FRONT && q_motion.head != q_motion.tail)
			{
				buff++;
				motion = deq_motion(&q_motion);
			}
			if (motion == GOAL)
			{
				buff = GOAL << 4 | buff;
				enq_motion(&q_buff, buff);
			}
			else
			{
				buff = FRONT << 4 | buff;
				enq_motion(&q_buff, buff);
			}
			break;
		case LEFT:
			buff = LEFT << 4;
			enq_motion(&q_buff, buff);
			motion = deq_motion(&q_motion);
			if(motion==GOAL){
				buff = GOAL << 4;
				enq_motion(&q_buff, buff);
			}
			break;
		case RIGHT:
			buff = RIGHT << 4;
			enq_motion(&q_buff, buff);
			motion = deq_motion(&q_motion);
			if(motion==GOAL){
				buff = GOAL << 4;
				enq_motion(&q_buff, buff);
			}
			break;
		default:
			break;
		}
	}
	q_motion = q_buff;
}

void Plan_Fast(void)
{
	unsigned char motion_h, motion_l;

	adcStart();
	while (sen_front.is_wall == FALSE)
	{
	}
	Output_Buzzer(HZ_G);
	gyro_offset_calc_reset();

	HAL_Delay(2500);
	flag_motor = TRUE;

	while (q_motion.head != q_motion.tail)
	{
		enc.offset = 0;
		motion_h = deq_motion(&q_motion);
		motion_l = motion_h & 0b1111;
		motion_h = motion_h >> 4;

		switch (motion_h)
		{
		case START:
			Motion_StartFast(motion_l);
			break;
		case LEFT:
			Motion_SlalomLeft();
			break;
		case FRONT:
			Motion_StraightFast(motion_l);
			break;
		case RIGHT:
			Motion_SlalomRight();
			break;
		case GOAL:
			Motion_GoalFast(motion_l);
			break;
		}
	}
	flag_motor = FALSE;
}