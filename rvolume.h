#ifndef __RVOLUME_H__
#define __RVOLUME_H__
#include "config.h"
#include <vector>
#include "wbcrrt.h"
#include "Octree.h"
#define OCTREE_DEPTH (6)
#define DEFAULT_INDEX_SIZE (1000)

using namespace std;

class TaskCell : public OctreeEntry
{
private:
	vector<int>		qs;
	int				sizeIndex;
	unsigned int	neighbor;			

public:
	TaskCell(void);
	virtual void	absorb(OctreeEntry *other);
	int				numIndex;
	int				*index;
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
