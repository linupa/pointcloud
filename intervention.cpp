#include <stdlib.h>
#include <Eigen/Eigen>

#include "config.h"
#include "wbcrrt.h"
#include "prm.hpp"
#include "xml.h"
#include "Octree.h"
#include "model.h"
#include "wbc.h"
#include "rvolume.h"

using namespace std;

VectorXd getQ(const VectorXd &uq);
VectorXd getQa(const VectorXd &q);

void intervention(void)
{
	int i, j, num;
	double ddmin[2] = {1e10, 1e10}, dmin;
	int  idxmin[2] = {0};
	VectorXd d, q;

	num = rrt->numNodes;
	cerr << "Check " << num << " nodes" << endl;
	int link[] = {5,3};
	for ( j = 0 ; j < 2 ; j++ )
	for ( i = 0 ; i < num ; i++ )
	{
		double dd;

		q =	getQ(rrt->nodes[i]->q);
		myModel.updateState(q);
		d = gGoal - myModel.joints[link[j]].getGlobalPos(VectorXd::Zero(3));
		dd = d.transpose()*d;
		if ( ddmin[j] > dd )
		{
			ddmin[j] = dd;
			idxmin[j] = i;
		}
	}

	int idx;
	if ( ddmin[0] > ddmin[1] )
	{
		dmin = sqrt(ddmin[1]);
		idx = idxmin[1];
	}
	else
	{
		dmin = sqrt(ddmin[0]);
		idx = idxmin[0];
	}
	cerr << "Collision anticipation " << gGoal.transpose() << endl;
	cerr << "closest point " << idx << " " << dmin << endl;
//	q =	getQ(rrt->nodes[idxmin]->q);
//	myModel.updateState(q);

	setRobotState(STATE_OPERATE);
	goalIdx = idx;
	Timestamp::resetTime();
}

bool checkMap[LINKS] = {0, 0, 1, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; 
Timestamp ts_intervention("Finding goal");
Vector3d intervention2(void)
{
	int i, j, k, num;
	double nmax[2] = {0., 0.};
	double inner;
	double min_dd = 1e10;
	int  idxmax[2] = {-1, -1};
	VectorXd q;
	Vector3d n, d, pos;
	double maxinner;
	double norm;

	norm = sqrt((gObjVel.transpose()*gObjVel)(0));
	n = gObjVel / norm;
	maxinner = (n.transpose() * (gGoal - gObj))(0);

	ts_intervention.setMode(Timestamp::MODE_SHOW_USEC);
	ts_intervention.setBaseline();

	num = rrt->numNodes;
	cerr << "Check " << num << " nodes" << endl;
	int link[] = {5,3};
	Vector3d dir = gObjVel / gObjVel.norm();
	int totalNumNodes = 0;
	int totalPost = 0;
	int numNodes[DOF] = {0};
//	OctreeEntryList *Nodes[DOF];
	OctreeEntryList *Nodes;
	VectorXd ref = getQa(disp_q1);
	int minIdx = -1;
	double minErr2 = 1e10;
//	Nodes = (OctreeEntryList *)malloc(sizeof(OctreeEntryList) * 1000);
	MatrixXd prod = MatrixXd::Zero(DOF,DOF);
	prod.block(0,0,9,9) = MatrixXd::Identity(9,9);
	for ( i = 0 ; i < LINKS; i++ )
	{

		if ( !checkMap[i] )
			continue;

		RVolume::getOctree(i)->getLineDistance(gObj, dir, 0.05);
		cerr << "Link " << i << ": " << OctreeEntryList::count << " cells" << endl;
//		Nodes[i] = (OctreeEntryList *)malloc(sizeof(OctreeEntryList) * OctreeEntryList::count);
//		memcpy(Nodes[i], OctreeEntryList::list, sizeof(OctreeEntryList) * OctreeEntryList::count);
//		memcpy(Nodes, OctreeEntryList::list, sizeof(OctreeEntryList) * OctreeEntryList::count);
		numNodes[i] = OctreeEntryList::count;
		totalNumNodes += numNodes[i];
		for ( j = 0 ; j < numNodes[i] ; j++ )
		{
//			TaskCell *entry = (TaskCell *)Nodes[i][j].pEntry;
			TaskCell *entry = (TaskCell *)OctreeEntryList::list[j].pEntry;
//			TaskCell *entry = (TaskCell *)Nodes[j].pEntry;
			for ( k = 0 ; entry && k < entry->numIndex ; k++ )
			{
				int idx = entry->index[k];
				VectorXd post =	rrt->nodes[idx]->q;

				VectorXd diff = post - ref; 
				totalPost++;
				double err2 = diff.transpose() * prod * diff;

				if ( minErr2 > err2 )
				{
					minErr2 = err2;
					minIdx = idx;
				}
			}
		}
//		free(Nodes[i]);
	}
//	free(Nodes);

#if 0
#endif
	ts_intervention.checkElapsed(0);
	cerr << "=====================" << endl;
	cerr << "Collision anticipation " << gGoal.transpose() <<  " : " << totalNumNodes << "," << totalPost << endl;
	cerr << "closest point " << " " << minIdx << " " << minErr2 << endl;
//	q =	getQ(rrt->nodes[minIdx]->q);
//	myModel.updateState(q);

#if 1
	if ( minIdx >= 0 )
	{
		setRobotState(STATE_OPERATE);
		if ( minIdx > 0 )
		{
			goalIdx = minIdx;
			Timestamp::resetTime();
		}
	}
#endif

	return gObjVel*10.;
}
