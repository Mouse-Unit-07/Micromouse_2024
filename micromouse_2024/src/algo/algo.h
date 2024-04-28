#include <stdbool.h>

#define MAZE_LENGTH 5
#define FALSE 0
#define TRUE  1
#define UINT_MAX   65535
#define STACK_SIZE 1000

/* Typedefs */
typedef enum
{
	FIRST_TRAVERSAL = 0,
	BACK_TO_START,
	RUN_TO_GOAL,
	RESET_1,
	RESET_2,
	GO_TO_LAST_POINT,
	FINISHED
} MouseState;

typedef enum
{
	NORTH,
	SOUTH,
	EAST,
	WEST
} Direction;

typedef struct
{
	unsigned int x;
	unsigned int y;
} Point;

typedef struct
{
	bool northWall;
	bool southWall;
	bool eastWall;
	bool westWall;
} MazeCell;

void algoIterate();