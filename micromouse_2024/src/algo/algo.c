#include <asf.h>
#include "algo.h"
#include "mouse_hardware_interface/usart_mhi.h"
#include "mouse_control_interface/movement_mci.h"
#include "mouse_control_interface/walldetection_mci.h"
#include "mouse_hardware_interface/clock_mhi.h"

MazeCell     mazeDiscovered [MAZE_LENGTH * MAZE_LENGTH] = {{FALSE, FALSE, FALSE, FALSE}};
unsigned int mazeFlood      [MAZE_LENGTH * MAZE_LENGTH] = {UINT_MAX};
bool         mazeVisited    [MAZE_LENGTH * MAZE_LENGTH] = {FALSE};
char         moveStack      [STACK_SIZE];
unsigned int stackTop        = 0;
unsigned int curDir          = NORTH;
unsigned int x = 0, y = 0;

MazeCell detectWalls();

void floodFill       (MazeCell* srcMazeCells, unsigned int* destFlood);
void floodFillRecurse(MazeCell* srcMazeCells, unsigned int x, unsigned int y, unsigned int cost, unsigned int* destFlood);

unsigned int mazeIdx   (unsigned int x, unsigned int y);
unsigned int mirrorY   (unsigned int y);
bool         isInRange (unsigned int x, unsigned int y);
bool         isGoal    (unsigned int x, unsigned int y);
bool         isExplored(unsigned int x, unsigned int y);

char pop (char* stack, unsigned int* top);
void push(char* stack, unsigned int* top, char data);

void moveForward(void);
void moveBack(void);
void moveLeft(void);
void moveRight(void);

void moveNorth(void);
void moveSouth(void);
void moveEast(void);
void moveWest(void);

bool checkNorthWall(void);
bool checkSouthWall(void);
bool checkEastWall(void);
bool checkWestWall(void);

bool checkFrontWall(void);
bool checkBackWall(void);
bool checkLeftWall(void);
bool checkRightWall(void);

bool traverseCell()
{
	MazeCell thisCell;

	if(isGoal(x,y))
		return TRUE;

	if(!mazeVisited[mazeIdx(x, y)]){
		thisCell = mazeDiscovered[mazeIdx(x, y)] = detectWalls();
		mazeVisited[mazeIdx(x, y)] = TRUE;
		floodFill(mazeDiscovered, mazeFlood);
	}
	else
		thisCell = mazeDiscovered[mazeIdx(x, y)];

	unsigned int cost = UINT_MAX;
	char nextDir;
	bool foundUnvisitedCell = FALSE;

	//north
	if(isInRange(x, y+1))
		if(!thisCell.northWall && !isExplored(x, y+1))
			if(mazeFlood[mazeIdx(x, y+1)] < cost){
				nextDir = NORTH;
				cost = mazeFlood[mazeIdx(x, y+1)];
				foundUnvisitedCell = TRUE;
			}

	//east
	if(isInRange(x+1, y))
		if(!thisCell.eastWall && !isExplored(x+1, y))
			if(mazeFlood[mazeIdx(x+1, y)] < cost){
				nextDir = EAST;
				cost = mazeFlood[mazeIdx(x+1, y)];
				foundUnvisitedCell = TRUE;
			}

	//south
	if(isInRange(x, y-1))
		if(!thisCell.southWall && !isExplored(x, y-1))
			if(mazeFlood[mazeIdx(x, y-1)] < cost){
				nextDir = SOUTH;
				cost = mazeFlood[mazeIdx(x, y-1)];
				foundUnvisitedCell = TRUE;
			}

	//west
	if(isInRange(x-1, y))
		if(!thisCell.westWall && !isExplored(x-1, y))
			if(mazeFlood[mazeIdx(x-1, y)] < cost){
				nextDir = WEST;
				cost = mazeFlood[mazeIdx(x-1, y)];
				foundUnvisitedCell = TRUE;
			}

	if(foundUnvisitedCell){
		switch(nextDir){
			case NORTH:
				moveNorth();
				push(moveStack, &stackTop, NORTH);
				break;
			case SOUTH:
				moveSouth();
				push(moveStack, &stackTop, SOUTH);
				break;
			case EAST:
				moveEast();
				push(moveStack, &stackTop, EAST);
				break;
			case WEST:
				moveWest();
				push(moveStack, &stackTop, WEST);
				break;
		}
	}
	else{
		char poppedMove = pop(moveStack, &stackTop);
		switch(poppedMove){
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
	
	return isGoal(x,y);
}

void floodFill(MazeCell* srcMazeCells, unsigned int* destFlood){
	for(int i = 0; i < MAZE_LENGTH * MAZE_LENGTH; i++)
		destFlood[i] = UINT_MAX;

	if(MAZE_LENGTH % 2)
		floodFillRecurse(srcMazeCells, MAZE_LENGTH / 2, MAZE_LENGTH / 2, 0, destFlood);
	else{
		floodFillRecurse(srcMazeCells, (MAZE_LENGTH / 2) - 1, (MAZE_LENGTH / 2) - 1, 0, destFlood);
		floodFillRecurse(srcMazeCells, (MAZE_LENGTH / 2) - 1, MAZE_LENGTH / 2,       0, destFlood);
		floodFillRecurse(srcMazeCells, MAZE_LENGTH / 2,       (MAZE_LENGTH / 2) - 1, 0, destFlood);
		floodFillRecurse(srcMazeCells, MAZE_LENGTH / 2,       MAZE_LENGTH / 2,       0, destFlood);
	}
}

void floodFillRecurse(MazeCell* srcMazeCells, unsigned int x, unsigned int y, unsigned int cost, unsigned int* destFlood){
	MazeCell mc = srcMazeCells[mazeIdx(x, y)];
	destFlood[mazeIdx(x, y)] = cost;

	//north
	if(isInRange(x, y+1))
		if(!mc.northWall)
			if(destFlood[mazeIdx(x, y+1)] > cost+1)
				floodFillRecurse(srcMazeCells, x, y+1, cost+1, destFlood);
	//south
	if(isInRange(x, y-1))
		if(!mc.southWall)
			if(destFlood[mazeIdx(x, y-1)] > cost+1)
				floodFillRecurse(srcMazeCells, x, y-1, cost+1, destFlood);
	//east
	if(isInRange(x+1, y))
		if(!mc.eastWall)
			if(destFlood[mazeIdx(x+1, y)] > cost+1)
				floodFillRecurse(srcMazeCells, x+1, y, cost+1, destFlood);
	//west
	if(isInRange(x-1, y))
		if(!mc.westWall)
			if(destFlood[mazeIdx(x-1, y)] > cost+1)
				floodFillRecurse(srcMazeCells, x-1, y, cost+1, destFlood);
}

unsigned int mazeIdx(unsigned int x, unsigned int y){
	return (mirrorY(y) * MAZE_LENGTH) + x;
}

unsigned int mirrorY(unsigned int y){
	return (MAZE_LENGTH - 1) - y;
}

bool isInRange(unsigned int x, unsigned int y){
	return x >= 0 && x < MAZE_LENGTH && y >= 0 && y < MAZE_LENGTH;
}

bool isGoal(unsigned int x, unsigned int y){
	if(MAZE_LENGTH % 2){
		return ((x == (MAZE_LENGTH / 2)) && (y == (MAZE_LENGTH / 2)));
	}
	else{
		return (
		( x == ((MAZE_LENGTH / 2) - 1) || x == (MAZE_LENGTH / 2) ) &&
		( y == ((MAZE_LENGTH / 2) - 1) || y == (MAZE_LENGTH / 2) )
		);
	}
}

bool isExplored(unsigned int x, unsigned int y){
	return mazeVisited[mazeIdx(x, y)];
}

char pop(char* stack, unsigned int* top){
	if(*top == 0){
		mhi_PrintString("ERROR: Popping empty stack!\n\r");
		exit(1);
	}

	(*top)--;
	return stack[*top];
}

void push(char* stack, unsigned int* top, char data){
	if(*top == STACK_SIZE){
		mhi_PrintString("ERROR: Pushing full stack!\n\r");
		exit(1);
	}

	stack[*top] = data;
	(*top)++;
}

void moveForward(void)
{
	mci_MoveForward1MazeSquarePid();
	mhi_DelayMs(80);
}

void moveBack(void)
{
	mci_TurnRight90Degrees();
	mhi_DelayMs(100);
	mci_TurnRight90Degrees();
	mhi_DelayMs(100);
	moveForward();
}

void moveLeft(void)
{
	mci_TurnLeft90Degrees();
	mhi_DelayMs(100);
	moveForward();
}

void moveRight(void)
{
	mci_TurnRight90Degrees();
	mhi_DelayMs(100);
	moveForward();
}

void moveNorth(void)
{
	switch(curDir){
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
	y++;
	curDir = NORTH;
}

void moveSouth(void)
{
	switch(curDir){
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
	y--;
	curDir = SOUTH;
}

void moveEast(void)
{
	switch(curDir){
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
	x++;
	curDir = EAST;
}

void moveWest(void)
{
	switch(curDir){
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
	x--;
	curDir = WEST;
}

MazeCell detectWalls()
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
	if(x == 0 && y == 0)
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
