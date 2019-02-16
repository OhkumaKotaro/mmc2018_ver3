#include "maze_analysis.h"

#define MAX_STEP 255

typedef struct {
	unsigned char data[MAX_STEP];
	unsigned char x[MAX_STEP];
	unsigned char y[MAX_STEP];
	unsigned int head;
	unsigned int tail;
}queue_t;

typedef struct {
	unsigned char data;
	unsigned char x;
	unsigned char y;
}buff_queue_t;


void Que_initialize(queue_t *q) {
	int i;

	q->head = 0;
	q->tail = 0;
	for (i = 0; i < MAX_STEP; ++i) {
		q->data[i] = 0;
		q->x[i] = 0;
		q->y[i] = 0;
	}
}

unsigned char enqueue(queue_t *q, unsigned char item, unsigned char px, unsigned char py) {
	if (q->tail >= MAX_STEP) {
		return MAX_STEP;
	}
	else {
		q->data[q->tail] = item;
		q->x[q->tail] = px;
		q->y[q->tail] = py;
		q->tail++;
	}
	return 1;
}

buff_queue_t dequeue(queue_t *q) {
	buff_queue_t tmp;
	tmp.data = q->data[q->head];
	tmp.x = q->x[q->head];
	tmp.y = q->y[q->head];
	q->head++;
	return tmp;
}


void Maze_CreateMap(maze_t *maze) {
	queue_t q;
	buff_queue_t buff;
	unsigned char x;
	unsigned char y;
	unsigned char step;
	unsigned int tmp;

	//init step
	for(unsigned char i=0;i<WALL_NUM;i++){
		for(unsigned char j=0;j<WALL_NUM;j++){
			if(maze->goal_x==i && maze->goal_y==j){
				maze->step[i][j]=0;
			}else{
				maze->step[i][j]=MAX_STEP;
			}
		}
	}

	Que_initialize(&q);
	enqueue(&q, 0, maze->goal_x, maze->goal_y);

	while (1)
	{
		buff = dequeue(&q);
		x = buff.x;
		y = buff.y;
		step = buff.data + 1;
		//north
		if (y < WALL_NUM-1)
		{
			tmp = maze->wall_horizontal[y] & (0b1 << x);
			if (tmp == 0)
			{
				if (maze->step[x][y + 1] == MAX_STEP) {
					enqueue(&q, step, x, y + 1);
					maze->step[x][y + 1] = step;
				}
			}
		}
		//south
		if (y>0) {
			tmp = maze->wall_horizontal[y - 1] & (0b1 << x);
			if ( tmp == 0)
			{
				if (maze->step[x][y - 1] == MAX_STEP) {
					enqueue(&q, step, x, y - 1);
					maze->step[x][y - 1] = step;
				}
			}
		}
		//east
		if (x < WALL_NUM - 1) 
		{
			tmp = maze->wall_vertical[x] & (0b1 << y);
			if ( tmp == 0)
			{
				if (maze->step[x + 1][y] == MAX_STEP) {
					enqueue(&q, step, x + 1, y);
					maze->step[x + 1][y] = step;
				}
			}
		}
		//west
		if (x > 0) 
		{
			tmp = maze->wall_vertical[x - 1] & (0b1 << y);
			if (tmp == 0)
			{
				if (maze->step[x-1][y] == MAX_STEP) {
					enqueue(&q, step, x - 1, y);
					maze->step[x - 1][y] = step;
				}
			}
		}
		if (q.head == q.tail) {
			break;
		}
	}
}

unsigned char Maze_CreateAllMap(maze_t *maze) {
	queue_t q;
	buff_queue_t buff;
	unsigned char x;
	unsigned char y;
	unsigned char step;
	unsigned int tmp;
	unsigned char re_num=0;

	//init Que
	Que_initialize(&q);

	//init step
	for (unsigned char i = 0; i<WALL_NUM; i++) {
		for (unsigned char j = 0; j<WALL_NUM; j++) {
			unsigned char tmp_step=255;
			//north
			if ((maze->wall_hor_search[j] & (0b1<<i)) == 0) {
				tmp_step = 0;
			}
			//south
			if (j>0)
			{
				if ((maze->wall_hor_search[j - 1] & (0b1<<i)) == 0) {
					tmp_step = 0;
				}
			}
			//east
			if ((maze->wall_ver_search[i] & (0b1<<j))== 0)
			{
				tmp_step = 0;
			}
			//west
			if (i>0)
			{
				if ((maze->wall_ver_search[i - 1] & (0b1<<j)) == 0) {
					tmp_step = 0;
				}

			}
			if (tmp_step == 0) {
				enqueue(&q, 0, i, j);
			}
			maze->step[i][j] = tmp_step;

		}
	}

	while (q.head != q.tail)
	{
		buff = dequeue(&q);
		x = buff.x;
		y = buff.y;
		step = buff.data + 1;
		//north
		if (y < WALL_NUM - 1)
		{
			tmp = maze->wall_horizontal[y] & (0b1 << x);
			if (tmp == 0)
			{
				if (maze->step[x][y + 1] == MAX_STEP) {
					enqueue(&q, step, x, y + 1);
					maze->step[x][y + 1] = step;
				}
			}
		}
		//south
		if (y>0) {
			tmp = maze->wall_horizontal[y - 1] & (0b1 << x);
			if (tmp == 0)
			{
				if (maze->step[x][y - 1] == MAX_STEP) {
					enqueue(&q, step, x, y - 1);
					maze->step[x][y - 1] = step;
				}
			}
		}
		//east
		if (x < WALL_NUM - 1)
		{
			tmp = maze->wall_vertical[x] & (0b1 << y);
			if (tmp == 0)
			{
				if (maze->step[x + 1][y] == MAX_STEP) {
					enqueue(&q, step, x + 1, y);
					maze->step[x + 1][y] = step;
				}
			}
		}
		//west
		if (x > 0)
		{
			tmp = maze->wall_vertical[x - 1] & (0b1 << y);
			if (tmp == 0)
			{
				if (maze->step[x - 1][y] == MAX_STEP) {
					enqueue(&q, step, x - 1, y);
					maze->step[x - 1][y] = step;
				}
			}
		}
	}

	if (maze->step[0][0] == 255) {
		re_num = 1;
	}

	return re_num;
}

void Maze_CreateFastMap(maze_t *maze) {
	for (unsigned char i = 0; i < WALL_NUM; i++)
	{
		for (unsigned char j = 0; j < WALL_NUM; j++)
		{
			if ((maze->wall_hor_search[i]>>j & 0b1) != 0)
			{
			}
			else
			{
				maze->wall_horizontal[i] = maze->wall_horizontal[i] | (0b1 << j);
			}
			if ((maze->wall_ver_search[i]>>j & 0b1) != 0)
			{
			}
			else
			{
				maze->wall_vertical[i] = maze->wall_vertical[i] | (0b1 << j);
			}
		}
	}
	Maze_CreateMap(maze);
}



