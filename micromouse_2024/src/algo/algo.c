#include <asf.h>
#include "algo.h"
#include "mouse_hardware_interface/usart_mhi.h"
#include "mouse_control_interface/walldetection_mci.h"
#include "mouse_control_interface/movement_mci.h"
#include "mouse_hardware_interface/clock_mhi.h"

/* Variables */
MouseState   state          = FIRST_TRAVERSAL;
MazeCell     mazeDiscovered [MAZE_LENGTH * MAZE_LENGTH] = {{FALSE, FALSE, FALSE, FALSE}};
unsigned int mazeFlood      [MAZE_LENGTH * MAZE_LENGTH] = {UINT_MAX};
bool         mazeVisited    [MAZE_LENGTH * MAZE_LENGTH] = {FALSE};
Direction    moveStack      [STACK_SIZE];
unsigned int stackTop       = 0;
Direction    curDir         = NORTH;
Point        curPoint       = {0, 0};
Point        startPoint     = {0, 0};
Point        centerPoints[] =
#if MAZE_LENGTH % 2
{
	{MAZE_LENGTH / 2, MAZE_LENGTH / 2}
};
#else
{
	{(MAZE_LENGTH / 2) - 1, (MAZE_LENGTH / 2) - 1},
	{(MAZE_LENGTH / 2) - 1, MAZE_LENGTH / 2},
	{MAZE_LENGTH / 2,       (MAZE_LENGTH / 2) - 1},
	{MAZE_LENGTH / 2,       MAZE_LENGTH / 2}
};
#endif
unsigned int numCenterPoints = sizeof(centerPoints) / sizeof(Point);

/* Traversal */
bool searchCell(Point goalPoints[], unsigned int numGoalPoints);
bool runCell(Point goalPoints[], unsigned int numGoalPoints);

/* Flood fill */
void floodFill(Point destPoints[], unsigned int numPoints, bool open);
void floodFillRecurse(Point point, unsigned int cost, bool open);

/* Utilities */
unsigned int mazeIdx(Point point);
unsigned int mirrorY(unsigned int y);
bool isInRange(Point point);
bool containsPoint(Point pointArr[], unsigned int arrSize, Point point);
bool isExplored(Point point);

/* Stack */
char pop(Direction* stack, unsigned int* top);
void push(Direction* stack, unsigned int* top, char data);

/* Movement */
void move(Direction direction);
void moveBackward(Direction direction);
void moveNorth(void);
void moveSouth(void);
void moveEast(void);
void moveWest(void);
void moveForward(void);
void moveBack(void);
void moveLeft(void);
void moveRight(void);

/* Wall checking */
MazeCell checkWalls(void);
bool checkNorthWall(void);
bool checkSouthWall(void);
bool checkEastWall(void);
bool checkWestWall(void);

bool checkFrontWall(void);
bool checkBackWall(void);
bool checkLeftWall(void);
bool checkRightWall(void);

void traverseCell()
{
	switch(state)
	{
		case FIRST_TRAVERSAL:
			if(searchCell(centerPoints, numCenterPoints))
				state = BACK_TO_START;
			break;
			
		case BACK_TO_START:
			if(runCell(&startPoint, 1))
				state = RUN_TO_GOAL;
			break;
			
		case RUN_TO_GOAL:
			if(runCell(centerPoints, numCenterPoints))
				state = FINISHED;
			break;
			
		default:
			break;
	}
}

bool searchCell(Point goalPoints[], unsigned int numGoalPoints)
{
	MazeCell thisCell;

	if (containsPoint(goalPoints, numGoalPoints, curPoint))
		return TRUE;

	if (!mazeVisited[mazeIdx(curPoint)])
	{
		thisCell = mazeDiscovered[mazeIdx(curPoint)] = checkWalls();
		mazeVisited[mazeIdx(curPoint)] = TRUE;
		floodFill(goalPoints, numGoalPoints, TRUE);
	}
	else
		thisCell = mazeDiscovered[mazeIdx(curPoint)];

	unsigned int cost = UINT_MAX;
	Direction nextDir;
	bool foundUnvisitedCell = FALSE;
	Point northPoint = {curPoint.x, curPoint.y + 1};
	Point eastPoint = {curPoint.x + 1, curPoint.y};
	Point southPoint = {curPoint.x, curPoint.y - 1};
	Point westPoint = {curPoint.x - 1, curPoint.y};

	if (isInRange(northPoint))
		if (!thisCell.northWall && !isExplored(northPoint))
			if (mazeFlood[mazeIdx(northPoint)] < cost)
			{
				nextDir = NORTH;
				cost = mazeFlood[mazeIdx(northPoint)];
				foundUnvisitedCell = TRUE;
			}

	if (isInRange(eastPoint))
		if (!thisCell.eastWall && !isExplored(eastPoint))
			if (mazeFlood[mazeIdx(eastPoint)] < cost)
			{
				nextDir = EAST;
				cost = mazeFlood[mazeIdx(eastPoint)];
				foundUnvisitedCell = TRUE;
			}

	if (isInRange(southPoint))
		if (!thisCell.southWall && !isExplored(southPoint))
			if (mazeFlood[mazeIdx(southPoint)] < cost)
			{
				nextDir = SOUTH;
				cost = mazeFlood[mazeIdx(southPoint)];
				foundUnvisitedCell = TRUE;
			}

	if (isInRange(westPoint))
		if (!thisCell.westWall && !isExplored(westPoint))
			if (mazeFlood[mazeIdx(westPoint)] < cost)
			{
				nextDir = WEST;
				cost = mazeFlood[mazeIdx(westPoint)];
				foundUnvisitedCell = TRUE;
			}

	if (foundUnvisitedCell)
	{
		move(nextDir);
		push(moveStack, &stackTop, nextDir);
	}
	else
	{
		Direction poppedMove = pop(moveStack, &stackTop);
		moveBackward(poppedMove);
	}

	if (containsPoint(goalPoints, numGoalPoints, curPoint))
	{
		mazeDiscovered[mazeIdx(curPoint)] = checkWalls();
		mazeVisited[mazeIdx(curPoint)] = TRUE;
		return TRUE;
	}

	return FALSE;
}

bool runCell(Point goalPoints[], unsigned int numGoalPoints)
{
	static bool firstItr = TRUE;
	MazeCell thisCell;

	if (containsPoint(goalPoints, numGoalPoints, curPoint))
		return TRUE;

	if (firstItr)
	{
		floodFill(goalPoints, numGoalPoints, FALSE);
		firstItr = FALSE;
	}

	thisCell = mazeDiscovered[mazeIdx(curPoint)];

	unsigned int cost = UINT_MAX;
	Direction nextDir;
	Point northPoint = {curPoint.x, curPoint.y + 1};
	Point eastPoint = {curPoint.x + 1, curPoint.y};
	Point southPoint = {curPoint.x, curPoint.y - 1};
	Point westPoint = {curPoint.x - 1, curPoint.y};

	if (isInRange(northPoint))
		if (!thisCell.northWall && isExplored(northPoint))
			if (mazeFlood[mazeIdx(northPoint)] < cost)
			{
				nextDir = NORTH;
				cost = mazeFlood[mazeIdx(northPoint)];
			}

	if (isInRange(eastPoint))
		if (!thisCell.eastWall && isExplored(eastPoint))
			if (mazeFlood[mazeIdx(eastPoint)] < cost)
			{
				nextDir = EAST;
				cost = mazeFlood[mazeIdx(eastPoint)];
			}

	if (isInRange(southPoint))
		if (!thisCell.southWall && isExplored(southPoint))
			if (mazeFlood[mazeIdx(southPoint)] < cost)
			{
				nextDir = SOUTH;
				cost = mazeFlood[mazeIdx(southPoint)];
			}

	if (isInRange(westPoint))
		if (!thisCell.westWall && isExplored(westPoint))
			if (mazeFlood[mazeIdx(westPoint)] < cost)
			{
				nextDir = WEST;
				cost = mazeFlood[mazeIdx(westPoint)];
			}

	move(nextDir);

	if (containsPoint(goalPoints, numGoalPoints, curPoint))
	{
		firstItr = TRUE;
		return TRUE;
	}

	return FALSE;
}

void floodFill(Point destPoints[], unsigned int numPoints, bool open)
{
	for (int i = 0; i < MAZE_LENGTH * MAZE_LENGTH; i++)
		mazeFlood[i] = UINT_MAX;

	for (unsigned int i = 0; i < numPoints; i++)
		if (open || mazeVisited[mazeIdx(destPoints[i])])
			floodFillRecurse(destPoints[i], 0, open);
}

void floodFillRecurse(Point point, unsigned int cost, bool open)
{
	unsigned int x = point.x, y = point.y;
	MazeCell mc = mazeDiscovered[mazeIdx(point)];
	mazeFlood[mazeIdx(point)] = cost;

	Point northPoint = {x, y + 1};
	Point southPoint = {x, y - 1};
	Point eastPoint = {x + 1, y};
	Point westPoint = {x - 1, y};

	if (isInRange(northPoint))
		if (open || mazeVisited[mazeIdx(northPoint)])
			if (!mc.northWall)
				if (mazeFlood[mazeIdx(northPoint)] > cost + 1)
					floodFillRecurse(northPoint, cost + 1, open);

	if (isInRange(southPoint))
		if (open || mazeVisited[mazeIdx(southPoint)])
			if (!mc.southWall)
				if (mazeFlood[mazeIdx(southPoint)] > cost + 1)
					floodFillRecurse(southPoint, cost + 1, open);

	if (isInRange(eastPoint))
		if (open || mazeVisited[mazeIdx(eastPoint)])
			if (!mc.eastWall)
				if (mazeFlood[mazeIdx(eastPoint)] > cost + 1)
					floodFillRecurse(eastPoint, cost + 1, open);

	if (isInRange(westPoint))
		if (open || mazeVisited[mazeIdx(westPoint)])
			if (!mc.westWall)
				if (mazeFlood[mazeIdx(westPoint)] > cost + 1)
					floodFillRecurse(westPoint, cost + 1, open);
}

unsigned int mazeIdx(Point point)
{
	return (mirrorY(point.y) * MAZE_LENGTH) + point.x;
}

unsigned int mirrorY(unsigned int y)
{
	return (MAZE_LENGTH - 1) - y;
}

bool isInRange(Point point)
{
	return point.x >= 0 && point.x < MAZE_LENGTH && point.y >= 0 && point.y < MAZE_LENGTH;
}

bool containsPoint(Point pointArr[], unsigned int arrSize, Point point)
{
	for (int i = 0; i < arrSize; i++)
		if (point.x == pointArr[i].x && point.y == pointArr[i].y)
			return TRUE;

	return FALSE;
}

bool isExplored(Point point)
{
	return mazeVisited[mazeIdx(point)];
}

char pop(Direction *stack, unsigned int *top)
{
	if (*top == 0)
	{
		//printf("ERROR: Popping empty stack!\n\r");
		exit(1);
	}

	(*top)--;
	return stack[*top];
}

void push(Direction *stack, unsigned int *top, char data)
{
	if (*top == STACK_SIZE)
	{
		//printf("ERROR: Pushing full stack!\n\r");
		exit(1);
	}

	stack[*top] = data;
	(*top)++;
}

void move(Direction direction)
{
	switch (direction)
	{
		case NORTH:
			moveNorth();
			break;
		case SOUTH:
			moveSouth();
			break;
		case EAST:
			moveEast();
			break;
		case WEST:
			moveWest();
			break;
	}
}

void moveBackward(Direction direction)
{
	switch (direction)
	{
		case NORTH:
			moveSouth();
			break;
		case SOUTH:
			moveNorth();
			break;
		case EAST:
			moveWest();
			break;
		case WEST:
			moveEast();
			break;
	}
}


void moveForward(void)
{
	mci_MoveForward1MazeSquarePid();
	mhi_DelayMs(80);
}

void moveBack(void)
{
	mci_TurnRight90DegreesPID();
	mhi_DelayMs(100);
	mci_TurnRight90DegreesPID();
	mhi_DelayMs(100);
	moveForward();
}

void moveLeft(void)
{
	mci_TurnLeft90DegreesPID();
	mhi_DelayMs(100);
	moveForward();
}

void moveRight(void)
{
	mci_TurnRight90DegreesPID();
	mhi_DelayMs(100);
	moveForward();
}

void moveNorth(void)
{
	switch (curDir)
	{
		case NORTH:
			moveForward();
			break;
		case SOUTH:
			moveBack();
			break;
		case EAST:
			moveLeft();
			break;
		case WEST:
			moveRight();
			break;
	}
	curPoint.y++;
	curDir = NORTH;
}

void moveSouth(void)
{
	switch (curDir)
	{
		case NORTH:
			moveBack();
			break;
		case SOUTH:
			moveForward();
			break;
		case EAST:
			moveRight();
			break;
		case WEST:
			moveLeft();
			break;
	}
	curPoint.y--;
	curDir = SOUTH;
}

void moveEast(void)
{
	switch (curDir)
	{
		case NORTH:
			moveRight();
			break;
		case SOUTH:
			moveLeft();
			break;
		case EAST:
			moveForward();
			break;
		case WEST:
			moveBack();
			break;
	}
	curPoint.x++;
	curDir = EAST;
}

void moveWest(void)
{
	switch (curDir)
	{
		case NORTH:
			moveLeft();
			break;
		case SOUTH:
			moveRight();
			break;
		case EAST:
			moveBack();
			break;
		case WEST:
			moveForward();
			break;
	}
	curPoint.x--;
	curDir = WEST;
}

MazeCell checkWalls()
{
	MazeCell cell;
	
	cell.northWall = checkNorthWall();
	cell.southWall = checkSouthWall();
	cell.eastWall  = checkEastWall();
	cell.westWall  = checkWestWall();
		
	return cell;
}

bool checkNorthWall(void)
{
	switch(curDir){
		case NORTH:
			return checkFrontWall();
		case SOUTH:
			return checkBackWall();
		case EAST:
			return checkLeftWall();
		case WEST:
			return checkRightWall();
	}	
}

bool checkSouthWall(void)
{
	switch(curDir){
		case NORTH:
			return checkBackWall();
		case SOUTH:
			return checkFrontWall();
		case EAST:
			return checkRightWall();
		case WEST:
			return checkLeftWall();
	}		
}

bool checkEastWall(void)
{
	switch(curDir){
		case NORTH:
			return checkRightWall();
		case SOUTH:
			return checkLeftWall();
		case EAST:
			return checkFrontWall();
		case WEST:
			return checkBackWall();
	}		
}

bool checkWestWall(void)
{
	switch(curDir){
		case NORTH:
			return checkLeftWall();
		case SOUTH:
			return checkRightWall();
		case EAST:
			return checkBackWall();
		case WEST:
			return checkFrontWall();
	}		
}

bool checkFrontWall(void)
{
	if (mci_CheckFrontWall() == MCI_WALL_NOT_FOUND)
		return FALSE;
	
	return TRUE;
}

bool checkBackWall(void)
{
	if(curPoint.x == 0 && curPoint.y == 0)
		return TRUE;
	
	return FALSE;
}

bool checkLeftWall(void)
{
	if (mci_CheckLeftWall() == MCI_WALL_NOT_FOUND)
		return FALSE;
	
	return TRUE;
}

bool checkRightWall(void)
{
	if (mci_CheckRightWall() == MCI_WALL_NOT_FOUND)
		return FALSE;
	
	return TRUE;
}
