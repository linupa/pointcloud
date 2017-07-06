#include <stdlib.h>
#include <Eigen/Eigen>

#include "config.h"
#include "wbcrrt.h"
#include "prm.hpp"
#include "xml.h"
#include "Octree.h"
#include "model.h"
#include "wbc.h"

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

Vector3d intervention2(void)
{
	int i, j, num;
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

	Timestamp ts1("Finding goal");
	ts1.setMode(Timestamp::MODE_SHOW_USEC);
	ts1.setBaseline();

	num = rrt->numNodes;
	cerr << "Check " << num << " nodes" << endl;
	int link[] = {5,3};
	for ( j = 0 ; j < 2 ; j++ )
	for ( i = 0 ; i < num ; i++ )
	{
		double dd;

		q =	getQ(rrt->nodes[i]->q);
		myModel.updateState(q);
		pos = myModel.joints[link[j]].getGlobalPos(VectorXd::Zero(3));
		inner = ((pos-gObj).transpose()*n)(0);
		d = pos - gObj - inner*n;
		dd = d.transpose()*d;
		if ( inner > maxinner )
			continue;
#if 1
		if ( min_dd > dd )
		{
			min_dd = dd;
//			idxmax[j] = i;
		}
#endif
#if 1
		if ( dd > 0.01 ) 
			continue;
		if ( nmax[j] < inner )
		{
			nmax[j] = inner;
			idxmax[j] = i;
		}
#endif
	}
	ts1.checkElapsed(0);
	cerr << ts1;

	int idx;
	if ( nmax[0] < nmax[1] )
	{
		inner = sqrt(nmax[1]);
		idx = idxmax[1];
	}
	else
	{
		inner = sqrt(nmax[0]);
		idx = idxmax[0];
	}
	cerr << "=====================" << endl;
	cerr << "Collision anticipation " << gGoal.transpose() << endl;
	cerr << "closest point " << idx << " " << inner << " " << min_dd << nmax[0] << ":" << nmax[1] << endl;
	cerr << "Vel " << inner * norm << endl;
//	q =	getQ(rrt->nodes[idxmin]->q);
//	myModel.updateState(q);

	if ( idx >= 0 )
	{
		setRobotState(STATE_OPERATE);
		goalIdx = idx;
		Timestamp::resetTime();
	}

	return inner * n;
}
