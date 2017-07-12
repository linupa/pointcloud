#include <stdlib.h>
#include <rvolume.h>
#include <pthread.h>
#include "config.h"
#include "wbcrrt.h"
#include "model.h"
#include "wbc.h"

extern VectorXd getQ(const VectorXd &uq);

int RVolume::nLink = 0;
Octree **RVolume::base_octree = NULL;

RVolume::RVolume(int idx)
{
	if ( base_octree )
		return;

	nLink = idx;
	base_octree = (Octree **)malloc(sizeof(Octree *)*idx);
	for ( int i = 0 ; i < nLink ; i++ )
	{
		base_octree[i] = new Octree;
		base_octree[i]->x = 0.;
		base_octree[i]->y = 0.;
		base_octree[i]->z = 0.;
		base_octree[i]->len = 1.;
	}
}

void RVolume::callback(int idx, void(*func)(Octree *))
{
	if ( idx >= nLink )
		return;
	base_octree[idx]->callBack(func);
}

Octree* RVolume::getOctree(int idx)
{
	return base_octree[idx];
}

TaskCell::TaskCell(void)
{
	index		= NULL;
	neighbor	= 0;
	sizeIndex	= 0;
	numIndex	= 0;
}

int TaskCell::addIndex(int idx)
{
	if ( index == NULL )
	{
		index = (int *)malloc(DEFAULT_INDEX_SIZE*sizeof(int));
		sizeIndex = DEFAULT_INDEX_SIZE;
	}

	if ( numIndex == sizeIndex-1 )
	{
		int *temp;
		temp = index;
		index = (int *)malloc(2*sizeIndex*sizeof(int));
		memcpy(index, temp, sizeIndex*sizeof(int));
		sizeIndex *= 2;
		free(temp);
	}

	index[numIndex] = idx;
	numIndex++;

	return numIndex;
}

void TaskCell::absorb(OctreeEntry *other)
{
}

void RVolume::addToOctree(WbcNode *node)
{
	VectorXd x;
	VectorXd q;

	q = getQ(node->q);
	
//	cerr << "add to octree" << node->q << endl;

	if ( node == NULL )
		return;

	pthread_mutex_lock(&link_mutex);
	myModel.updateState(q);

	for ( int i = 1 ; i < myModel.numJoints ; i++ )
	{
		x = myModel.joints[i].getGlobalPos(VectorXd::Zero(3));
		TaskCell *cell = new TaskCell;
		cell->x = x[0];
		cell->y = x[1];
		cell->z = x[2];
		TaskCell *entry = (TaskCell*)base_octree[i-1]->addEntry((OctreeEntry *)cell, OCTREE_DEPTH);
		entry->addIndex(node->index);
	}
	pthread_mutex_unlock(&link_mutex);
}

