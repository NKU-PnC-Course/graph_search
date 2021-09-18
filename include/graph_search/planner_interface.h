#ifndef _PLANNER_INTERFACE_H_
#define _PLANNER_INTERFACE_H_

#include <cstdlib>
#include <vector>

#define CONTXY2DISC(X, CELLSIZE) (((X) >= 0) ? ((int)((X) / (CELLSIZE))) : ((int)((X) / (CELLSIZE)) - 1))
#define DISCXY2CONT(X, CELLSIZE) ((X) * (CELLSIZE) + (CELLSIZE) / 2.0)

struct PointInt
{
  PointInt(int _x, int _y)
  {
    x = _x;
    y = _y;
  }

  int x;
  int y;
};

class PlannerInterface
{
public:
  PlannerInterface()
  {
  }

  virtual ~PlannerInterface()
  {
  }

  virtual bool Search(int startX, int startY, int goalX, int goalY, unsigned char** occMap, int width, int height,
                      unsigned char obsthresh, std::vector<PointInt>& path, std::vector<PointInt>& expands) = 0;
};

#endif  // _PLANNER_INTERFACE_H_