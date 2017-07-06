#include <iostream>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <assert.h>
#include "Octree.h"

using namespace std;

int Octree::count = 0;
Octree::Octree(void)
{
	int i;
	
	depth	= 0;
	filled	= 0x00;
	free	= 0x00;
	leaf	= 0;
	
	entry = NULL;
	for ( i = 0 ; i < 8 ; i++ )
		child[i] = NULL;

	this->parent = NULL;
	count++;
}

Octree::Octree(Octree *parent, int leaf_)
{
	int i;

	entry = NULL;
	for ( i = 0 ; i < 8 ; i++ )
		child[i] = NULL;
	
	depth	= parent->depth + 1;
	len		= parent->len / 2.;
	filled	= 0x00;
	free	= 0x00;
	leaf	= leaf_;

//	std::cerr << "Octree Creation " << depth << std::endl;

	this->parent = parent;
	count++;
}

Octree::~Octree(void)
{
	int i;

//	std::cerr << "Octree Deletion " << depth << std::endl;
	if ( child[0] )
	{
		for ( i = 0 ; i < 8 ; i++ )
		{
			delete child[i];
			child[i] = NULL;
		}
	}
	if ( parent )
	{
//		assert(parent->child[leaf] == this);
		parent->child[leaf] = NULL;
		parent->filled = parent->filled & (~(1<<leaf));
		parent->checkEmpty();
	}
	count--;
}
double offset[8][3] = 
{
	{-0.5, -0.5, -0.5},	// x- y- z-
	{-0.5, -0.5, +0.5},	// x- y- z+
	{-0.5, +0.5, -0.5}, // x- y+ z-
	{-0.5, +0.5, +0.5},	// x- y+ z+
	{+0.5, -0.5, -0.5},	// x+ y- z-	
	{+0.5, -0.5, +0.5},	// x+ y- z+
	{+0.5, +0.5, -0.5},	// x+ y+ z-
	{+0.5, +0.5, +0.5}	// x+ y+ z+
};

void Octree::createChild(void)
{
	int i; 
	
	for ( i = 0 ; i < 8 ; i++ )
	{
		child[i] = new Octree(this, i);	
		child[i]->x = x + len * offset[i][0];
		child[i]->y = y + len * offset[i][1];
		child[i]->z = z + len * offset[i][2];
	}
}

void Octree::deleteChild(void)
{
	
	if ( child[0] )
	{
		int i; 
		for ( i = 0 ; i < 8 ; i++ )
		{
			delete child[i];
			child[i] = NULL;
		}
	}	
}

OctreeEntry *Octree::addEntry(OctreeEntry *pEntry, int max_depth)
{
	double x_, y_, z_;

	if ( !pEntry )
		return NULL;

	x_ = pEntry->x;
	y_ = pEntry->y;
	z_ = pEntry->z;

	bool ret = true;
	if ( fabs(x_ - x) > len )
		ret = false;
	else if ( fabs(y_ - y) > len )
		ret = false;
	else if ( fabs(z_ - z) > len )
		ret = false;

	if ( ret == false )
	{
//		cerr << "FALSE " << x_ << ":" << y_ << ":" << z_ << " " << x << ":" << y << ":" << z << "." << len <<  endl;
		return NULL;
	}

	if ( depth == max_depth )
	{
        if ( entry )
        {
            delete pEntry;
        }
        else
        {
            entry = pEntry;
            entry->tree= this;
        }
//      pEntry->tree = this;
//      pEntry->absorb(entry);
//		std::cerr << "Added " << depth << std::endl;
//		entry = pEntry;
		filled = filled | (1<<THIS_FILLED);
		if ( parent )
		{
			parent->filled = parent->filled | (1<<leaf);
			parent->checkEmpty();
		}

		return entry;
	}

	if ( child[0] == NULL )
	{
		createChild();
	}

	int index = 0;
	
	if ( x_ > x )
		index |= 0x4;
	if ( y_ > y )
		index |= 0x2;
	if ( z_ > z )
		index |= 0x1;

//	cerr << "ADD " << x_ << ":" << y_ << ":" << z_ << " " << x << ":" << y << ":" << z << "." << len << " " << index << endl;
	

	return child[index]->addEntry(pEntry, max_depth);
}

void Octree::callBack(void (*cbFunc)(Octree *pEntry))
{
//	std::cerr <<"callBack called" << std::endl;

	if ( child[0] != NULL )
	{
		assert(child[0]->parent == this);
		for ( int i = 0 ; i < 8 ; i++ )
			child[i]->callBack(cbFunc);	
	}
	cbFunc(this);
}

OctreeEntryList *OctreeEntryList::list = NULL;
int OctreeEntryList::count = 0;
int OctreeEntryList::size = 0;

int list_index;
void OctreeEntryList::reset(void)
{
#if 0
	if ( OctreeEntryList::list )
		delete OctreeEntryList::list;
	OctreeEntryList::list = new OctreeEntryList;
	OctreeEntryList::list->pEntry 	= NULL;
	OctreeEntryList::list->pNext	= NULL;
#else
	if ( !OctreeEntryList::list )
	{
		OctreeEntryList::list = (OctreeEntryList *)malloc(sizeof(OctreeEntryList) * 1000 );
		OctreeEntryList::size = 1000;
	}
	OctreeEntryList::list[0].pEntry = NULL;
//	OctreeEntryList::list[0].pNext = NULL;
	list_index = 0;
#endif
	count = 0;
	
}

void Octree::getPlane(double *coeff, double threshold)
{
	if ( entry )
		OctreeEntryList::addEntry(entry);

	if ( child[0] )
	{
		for ( int i = 0 ; i < 8 ; i++ )
		{
			double xx, yy, zz;
			double distance;

			xx = child[i]->x;
			yy = child[i]->y;
			zz = child[i]->z;

			distance = coeff[0]*xx + coeff[1]*yy + coeff[2]*zz + coeff[3];

			if ( distance < len/2. + threshold ) 
				child[0]->getPlane(coeff, threshold);
		}
	}
}

void Octree::getLineDistance(double *coeff, double threshold)
{
	if ( entry )
		OctreeEntryList::addEntry(entry);

	if ( child[0] )
	{
		double c0, c1, c2;
		double th;

		th = len/2. + threshold;
		th = th*th;
		
		c0 = coeff[0];
		c1 = coeff[1];
		c2 = coeff[2];
		
		for ( int i = 0 ; i < 8 ; i++ )
		{
			double xx, yy, zz;
			double distance;
			double ii, jj, kk;

			xx = child[i]->x;
			yy = child[i]->y;
			zz = child[i]->z;

			ii = c2*yy - c1*zz;
			jj = c0*zz - c2*xx;
			kk = c1*xx - c0*yy;

			distance = ii*ii + jj*jj + kk*kk;

			if ( distance < th )
				child[i]->getLineDistance(coeff, threshold);
		}
	}
}

// dir should be normalized
void Octree::getLineDistance(const Vector3d &point, const Vector3d &dir, double threshold)
{
	if ( entry )
		OctreeEntryList::addEntry(entry);

	if ( child[0] )
	{
		double th, th2;

		th = len/sqrt(2.) + threshold;
		th2 = th*th;
		
		for ( int i = 0 ; i < 8 ; i++ )
		{
			Vector3d ch, diff;
			double inner;
			double distance;
			double ii, jj, kk;

			ch[0] = child[i]->x;
			ch[1] = child[i]->y;
			ch[2] = child[i]->z;

			inner = (ch - point).transpose() * dir;

			diff = (ch - point) - inner*dir;

			distance = diff.transpose() * diff;

			if ( distance < th2 )
				child[i]->getLineDistance(point, dir, threshold);
		}
	}
}


void Octree::getPointDistance(double *pos, double threshold)
{
	if ( entry )
		OctreeEntryList::addEntry(entry);

	if ( child[0] )
	{
		double c0, c1, c2;
		double th;

		th = len/2. + threshold;
		th = th*th;
		
		for ( int i = 0 ; i < 8 ; i++ )
		{
			double xx, yy, zz;
			double distance;
			double ii, jj, kk;

			xx = child[i]->x - pos[0];
			yy = child[i]->y - pos[1];
			zz = child[i]->z - pos[2];

			distance = xx*xx + yy*yy + zz*zz;

			if ( distance < th )
				child[i]->getPointDistance(pos, threshold);
		}
	}
}

int cond[8][3] = 
{
	{ -1, -1, -1 },
	{ -1, -1, +1 },
	{ -1, +1, -1 },
	{ -1, +1, +1 },
	{ +1, -1, -1 },
	{ +1, -1, +1 },
	{ +1, +1, -1 },
	{ +1, +1, +1 },
};
void Octree::getNeighbor(OctreeEntry *pEntry, double distance)
{
	double xx = pEntry->x, yy = pEntry->y, zz = pEntry->z;

#if 0
	if ( parent )
	{
		if ( xx - distance < x - len || xx + distance > x + len ||
			 yy - distance < y - len || yy + distance > y + len ||
			 zz - distance < z - len || zz + distance > z + len ) 
		{
			parent->getNeighbor(pEntry, distance);
			return;
		}
	}
#endif

	if ( entry )
		OctreeEntryList::addEntry(entry);

	double xp,yp,zp,xm,ym,zm;
	xm = -(xx-distance) + (x);
	ym = -(yy-distance) + (y);
	zm = -(zz-distance) + (z);
	xp = (xx+distance) - (x);
	yp = (yy+distance) - (y);
	zp = (zz+distance) - (z);

	double value[8][3] = 
		{
			{ xm, ym, zm },
			{ xm, ym, zp },
			{ xm, yp, zm },
			{ xm, yp, zp },
			{ xp, ym, zm },
			{ xp, ym, zp },
			{ xp, yp, zm },
			{ xp, yp, zp }
		};


	if ( child[0] )
	{
		int i;

		for ( i = 0 ; i < 8 ; i++ )
		{
			if (	(value[i][0] >= 0) &&
					(value[i][1] >= 0) &&
					(value[i][2] >= 0) &&
					(filled & (1<<i)) )
			{
				child[i]->getNeighbor(pEntry, distance);
			}
		}
	}

	return;
}

int OctreeEntryList::addEntry(OctreeEntry *pEntry)
{
#if 0
	OctreeEntryList *ret = new OctreeEntryList();
	
	ret->pNext	= OctreeEntryList::list;
	ret->pEntry	= pEntry;

	OctreeEntryList::list = ret;
#else

	if (list_index >= size)
	{
		OctreeEntryList *old = list;
		list = (OctreeEntryList *)malloc(sizeof(OctreeEntryList) * size * 2 );
		memcpy((void *)list, (void *)old, sizeof(OctreeEntryList) * size);
		size *= 2;
	}

	OctreeEntryList *ret = &(list[list_index]);
//	ret->pNext	= NULL;
	ret->pEntry	= pEntry;

//	if ( list_index > 0 )
//		OctreeEntryList::list[list_index-1].pNext = ret;

	list_index++;

#endif
	count++;

	return list_index-1;
}

OctreeEntryList::~OctreeEntryList(void)
{
//	if ( pNext )
//		delete pNext;
}

void Octree::checkEmpty(void)
{
	if ( !parent )
		return;

	assert(depth == parent->depth+1);
	if ( filled )
	{
		if ( parent->filled & (1<<leaf) )
			return;
		parent->filled = parent->filled | (1<<leaf);
		parent->checkEmpty();
	}
	else
	{
		if ( !(parent->filled & (1<<leaf)) )
			return;
		parent->filled = parent->filled & (~(1<<leaf));
		parent->checkEmpty();
	}
}

bool Octree::checkValidity(void)
{
	int i;
	int hasChild = 0;
	bool ret = true;

	for ( i = 0 ; i < 8 ; i++ )
		if ( child[i] )
			hasChild |= (1<<i);

	if ( hasChild != 0 && hasChild != 0xFF )
	{
		fprintf(stderr, "ERROR: some of children are NULL %02x\n", hasChild);

		Octree *entry = this;

		while ( entry )
		{
			fprintf(stderr, "%p(%d) > ", entry, entry->leaf); 
			entry = entry->parent;
		}
		
		fprintf(stderr, "\n");

		return false;
	}
			
	

	for ( i = 0 ; i < 8 ; i++ )
		if ( child[i] )
			ret &= child[i]->checkValidity();

	return ret;
}

void Octree::log(void)
{
	fprintf(stderr, "Octree Count: %d\n", count);
}

void Octree::removeEmpty(void)
{
	if ( !filled )
	{
		deleteChild();
	}
	else
	{
		if ( child[0] )
			for ( int i = 0 ; i < 8 ; i++ )
			{
				child[i]->removeEmpty();
			}
	}
}

void Octree::checkFreeSpace(const VectorXd &org, const VectorXd &v1, const VectorXd &v2, const MatrixXd &transform, const double threshold)
{
	int			i, scan_mask = 0xff;
	VectorXd	p = VectorXd::Zero(3), pp;	
	
	p[0] = x, p[1] = y, p[2] = z;

	pp = transform * (p - org);

	if ( abs(pp[2]) > sqrt(2) * len + threshold )
		return;

	// Under V1
	if ( pp[1] < 0. )
	{
	}
	// Over V2
	if ( pp[1] / pp[0] > v2[1] / v2[0] )
	{
	}
	// Far away
	{
	}


	for ( i = 0 ; i < 8 ; i++ )
		if ( child[i] )
			child[i]->checkFreeSpace(org, v1, v2, transform, threshold);
}
