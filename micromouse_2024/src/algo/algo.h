#include <stdbool.h>

#define MAZE_LENGTH 5
#define NORTH 0
#define SOUTH 1
#define EAST  2
#define WEST  3
#define FALSE 0
#define TRUE  1
#define UINT_MAX   65535
#define STACK_SIZE 1000

typedef struct {
	bool northWall;
	bool southWall;
	bool eastWall;
	bool westWall;
} MazeCell;

bool traverseCell();
