#ifndef __OCTREE_H__
#define __OCTREE_H__
#include <Eigen/Dense>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::Vector3d;

class Octree;

class OctreeEntry
{
public:
	double x;
	double y;
	double z;
	Octree *tree;
	virtual void absorb(OctreeEntry *other) {}
};

class OctreeEntryList
{
	public:
	OctreeEntry		*pEntry;
//	OctreeEntryList *pNext;
	
	static int		addEntry(OctreeEntry *pEntry);	
	~OctreeEntryList(void);
	static			OctreeEntryList *list;
	static			void reset(void);
	static			int count;
	static			int size;
};

#define THIS_FILLED	(8)
class Octree
{
public:
	int			depth;
	double		x;
	double		y;
	double		z;
	double		len;
	bool		terminal;
	int			filled;
	int			free;
	int			leaf;
	OctreeEntry	*entry;

	Octree(void);
	Octree(Octree *parent, int leaf_);
	~Octree(void);

	void createChild(void);
	void deleteChild(void);
	
	OctreeEntry *addEntry(OctreeEntry *pEntry, int max_depth);
	void callBack(void (*)(Octree *pEntry));
	void getNeighbor(OctreeEntry *pEntry, double distance);
	void getPlane(double *coeff, double threshold);
	void getLineDistance(double *coeff, double threshold);
	void getLineDistance(const Vector3d &point, const Vector3d &dir, double threshold);
	void getPointDistance(double *coeff, double threshold);
	void checkEmpty(void);
	void removeEmpty(void);
	bool checkValidity(void);
	void checkFreeSpace(const VectorXd &org, const VectorXd &v1, const VectorXd &v2, const MatrixXd &transform, const double threshold);

	static void log(void);

private:
	class Octree *parent;
	class Octree *child[8];

	static int count;
};
#endif
