#ifndef _GRAPH_SEARCH_H_
#define _GRAPH_SEARCH_H_

#include "graph_search/planner_interface.h"

class GraphSearch : public PlannerInterface
{
public:
  GraphSearch();
  virtual ~GraphSearch();

  bool Search(int startX, int startY, int goalX, int goalY, unsigned char** occMap, int width, int height,
              unsigned char obsthresh, std::vector<PointInt>& path, std::vector<PointInt>& expands);
};

#endif  // _GRAPH_SEARCH_H_