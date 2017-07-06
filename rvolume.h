#ifndef __RVOLUME_H__
#define __RVOLUME_H__
#include "config.h"
#include <vector>
#include "wbcrrt.h"
#include "Octree.h"
#define OCTREE_DEPTH (5)
#define DEFAULT_INDEX_SIZE (1000)

using namespace std;

class TaskCell : public OctreeEntry
{
	vector<int>		qs;
	int				sizeIndex;
	int				*index;
	unsigned int	neighbor;			
	virtual void	absorb(OctreeEntry *other);

public:
	TaskCell(void);
	int				numIndex;
	int				addIndex(int idx);
};

class RVolume
{
	static int nLink;
public:
	RVolume(int num);
	static Octree **base_octree;
	static void addToOctree(WbcNode *node);
	static void callback(int idx, void(*)(Octree *));
	static Octree *getOctree(int idx);
};

#endif
