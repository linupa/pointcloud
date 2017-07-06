#include <stdlib.h>
#include <Eigen/Eigen>

#include "config.h"
#include "wbcrrt.h"
#include "prm.hpp"
#include "xml.h"
#include "Octree.h"
#include "model.h"
#include "wbc.h"
#include "timestamp.h"

using namespace std;
typedef RRT<DOF> WbcRRT;

extern int push_type;
extern double *value;

VectorXd getQ(const VectorXd &uq);
VectorXd getQa(const VectorXd &q);
double getPotential(int mode, const Vector3d &contact, const WbcNode &node, const vector<Vector3d> &r0, const int verbose=0);

//VectorXd pushError;
int pushType;
Timestamp pushTimestamp;
void push(VectorXd dir)
{
	int i, j ;

	if ( getRobotState() & STATE_LEARNING )
		return;

	pushTimestamp = Timestamp("Push Check");
	pushTimestamp.setBaseline();
	pushTimestamp.setMode(Timestamp::MODE_OVERLAP | Timestamp::MODE_SHOW_MSEC);

	if ( !rrt || rrt->numNodes == 0 )
		return;

	if ( dir(0) > 0.03 )
		push_type = 1;
	else if ( dir(1) < -0.03 )
		push_type = 2;
	else if ( dir(2) > 0.03 )
		push_type = 3;
	else
		return;
	pushType = push_type;

	Vector3d contact = elbow0;

	cerr << "Start Pushing... " << contact.transpose() << " : " << dir.transpose() << " " << push_type << endl;

	double	max = -100000.;
	int		max_idx = 0;

	cerr << contact.transpose() << endl;
	cerr << q0.transpose() << endl;
	value = (double *)malloc(sizeof(double)*rrt->numNodes);

	Timestamp ts0("Total Time"), ts1("Check Potential");
	ts0.setMode(Timestamp::MODE_SHOW_SEC);
	ts1.setMode(Timestamp::MODE_SHOW_USEC | Timestamp::MODE_SHOW_COUNT);
	ts0.setBaseline();

	vector<Vector3d> r0;

	pthread_mutex_lock(&link_mutex);
	myModel.updateState(disp_q1);
	for ( int j = 0 ; j < 10 ; j++ )
	{
		r0.push_back(myModel.joints[j].getGlobalPos(myModel.joints[j].com));
	}
	pthread_mutex_unlock(&link_mutex);

	pthread_mutex_lock(&mutex);
	ts0.checkElapsed(0);
	for ( i = rrt->numNodes-1 ; i >= 0 ; i-- )
//	for ( i = 0 ; i < rrt->numNodes ; i++ )
	{
		ts1.setBaseline();
		ts0.checkElapsed(1);
		WbcNode *pNode = (WbcNode *)rrt->nodes[i];
		VectorXd Q = getQ(pNode->q);

		double sum = getPotential(push_type, contact, *pNode, r0);
		value[i] = sum;
		
		if ( sum > max )
		{
			max		= sum;
			max_idx	= i;
//			cerr << "MAX " << max_idx << ": " << max <<endl;
//			cerr << Q.transpose() << endl;
//			cerr << q0.transpose() << endl;
		}
		ts1.checkElapsed(0);
	}
	ts0.checkElapsed(2);
	pthread_mutex_unlock(&mutex);
	cerr << "MAX " << max_idx << ": " << max << " " << i << endl;
	cerr << ts0;
	cerr << ts1;

	setRobotState(STATE_OPERATE);
	goalIdx = max_idx;
	Timestamp::resetTime();
}

double getPotential(int mode, const Vector3d &contact, const WbcNode &node, const vector<Vector3d>& r0, const int verbose)
{
	int j;
	double sum = 0.;
	double weight[10] = {0.};
	int mask[10] = {0};
	double value[10];

	weight[5] = 12.;
	weight[6] = 12.;
	weight[7] = 12.;
	mask[5] = 1;
	mask[6] = 1;
	mask[7] = 1;
//	VectorXd r[10];

	for ( j = 0 ; j < 10 ; j++ )
	{
		Vector3d d = node.coms[j] - contact;
//		Vector3d d = r[j] - contact;
		
		if ( mask[j] )
		{
			switch (mode)
			{
			case 1:
				value[j] = weight[j] * d(0) * myModel.joints[j].mass; // X-
				break;
			case 2:
				value[j] = - weight[j] * d(1) * myModel.joints[j].mass; // Y-
				break;
			case 3:
				value[j] = weight[j] * d(2) * myModel.joints[j].mass; // Z+
				break;
			}
			if ( verbose )
			{
				cerr << d.transpose() << ":" << myModel.joints[j].mass << "(" << value[j] << ")" << endl;
			}
		}
		else
		{
			value[j] = - sqrt(((r0[j] - node.coms[j]).transpose() * (r0[j] - node.coms[j]))(0)) * myModel.joints[j].mass;
//			value[j] = - sqrt(((r0[j] - r[j]).transpose() * (r0[j] - r[j]))(0)) * myModel.joints[j].mass;
			if ( verbose )
			{
				cerr << (r0[j]-node.coms[j]).transpose() << ":" << myModel.joints[j].mass << "(" << value[j] << ")" << endl;
//				cerr << (r0[j]-r[j]).transpose() << ":" << myModel.joints[j].mass << "(" << value[j] << ")" << endl;
			}
		}
	}

	sum = 0;
	for ( j = 0 ; j < 10 ; j++ )
	{
//		if ( verbose )
//			cerr << value[j] << ":" << myModel.joints[j].mass << " ";
		sum += value[j];
	}
//	if ( verbose )
//		cerr << endl;

	return sum;
}
