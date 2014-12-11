#ifndef READ_LIDAR_DATA
#define READ_LIDAR_DATA

#include "utils.h"

//  define the parameters of the grid
#define GRID_X_MIN -10.
#define GRID_X_MAX 10.
#define GRID_Y_MIN 0
#define GRID_Y_MAX 30.
#define GRID_X_STEP 0.2
#define GRID_Y_STEP  0.2
#define GRID_NB_CELLS_X ((GRID_X_MAX-GRID_X_MIN)/GRID_X_STEP)
#define GRID_NB_CELLS_Y ((GRID_Y_MAX-GRID_Y_MIN)/GRID_Y_STEP)

void readLidarData ();

#endif
