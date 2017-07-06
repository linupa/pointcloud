#include <stdlib.h>
#include <Eigen/Eigen>
#include <unistd.h>

#include "config.h"
#include "wbcrrt.h"
#include "prm.hpp"
#include "xml.h"
#include "Octree.h"
#include "model.h"
#include "wbc.h"

#define STEP (M_PI*0.020) // Resolution 0.02PI = 3.6 deg

vector<VectorXd> qp1;
vector<VectorXd> qp2;

vector<double> dist;
vector<double> distp;
vector<double> d_t;
vector<double> eta;

//double mins[] = {	-80.,	-10,	-70,	 -10,	-60,	  0,	-20,	-40,	-40};
//double maxs[] = {	 80.,	 40,	180,	130,	 60,	100,	200,	 40,	 40};
VectorXd Mins;
VectorXd Maxs;
// Nominal Joint Limits
double mins[] = {	-90.,	-20,	
					-80,	-25,	-85,	  0,	-48,	-60,	-60,
					-80,   -150,	-85,	  0,	-48,	-60,	-60};
double maxs[] = {	 90.,	 43,	
					200,	150,	 85,	133,	230,	 60,	 60,
					200,	 25,	 85,	133,	230,	 60,	 60};
// Actual Joint Limits
//double mins[] = {	-90.,	-22,	-80,	-23,	-85,	  0,	-48,	-60,	-60};
//double maxs[] = {	 90.,	 49,	200,	150,	 85,	133,	230,	 60,	 60};

template <int T>
double project2( const Node<T> *p, Node<T> *np );

template <int T>
double project1( Node<T> &np );

VectorXd getQ(const VectorXd &uq);
VectorXd getQa(const VectorXd &q);

void *plan(void *)
{
	fprintf(stderr, "Planning Thread Started...\n");
	int i;
	Mins = Maxs = VectorXd::Zero(DOF);
	for ( i = 0 ; i < DOF ; i++ )
	{
		Mins(i) = mins[i] * M_PI / 180.;
		Maxs(i) = maxs[i] * M_PI / 180.;
	}
	cerr << "PRM init" << endl;
	prm = new PRM<DOF>(Mins, Maxs, STEP);
	cerr << "PRM init done" << endl;
	WbcNode::mins = Mins;
	WbcNode::maxs = Maxs;

	while (1)
	{
		int next_count = 1000;

		//////////////////////////////////
		// Start generating sampling tree
		//////////////////////////////////
		rrt = new WbcRRT(Mins, Maxs, STEP);
		rrt->nodeCreator = WbcNode::create;
		prm->init();
#if 0
		Node<DOF> *newNode = new Node<DOF>;

		newNode->q = body_state.position_;
		newNode->getProjection();
		newNode->parent = NULL;
		rrt->nodes[rrt->numNodes++] = newNode;
#else
		rrt->reset(getQa(disp_q1));
		
#ifdef USE_WBC
		((WbcNode *)rrt->nodes[0])->getProjection();
//		q0 = getQ(body_state.position_);
#endif
#endif

		int count = 0;

		pthread_mutex_lock(&link_mutex);
		myModel.updateState(disp_q1);
		q0 = disp_q1;
		elbow0 = myModel.joints[5].getGlobalPos(VectorXd::Zero(3));
		pthread_mutex_unlock(&link_mutex);

		fprintf(stderr, "Start Planning... %d,%d\n", rrt->numNodes, rrt->numEdges);
		cerr << q0.transpose() << endl;
		while ( getRobotState() & STATE_LEARN )
		{
			setRobotState(STATE_LEARNING);

			rrt->iterate();
//			prm->addNode(project1);

	//		usleep(1000);
			{
				if ( rrt->numNodes > next_count )
				{
					fprintf(stderr, "%d nodes added\n", rrt->numNodes);	
					WbcNode *p = (WbcNode *)(rrt->nodes[rrt->numNodes-1]);

					cerr << p << endl;
					cerr << p->q.transpose() << endl;
					cerr << "Actual " << p->actual.transpose() << endl;
					cerr << *rrt << endl;
					disp_q2 = getQ(p->q);
					next_count += 1000;
				}
				count++;

			}
		}
		clearRobotState(STATE_LEARNING | STATE_OPERATING);

		cerr << "Total " << rrt->numNodes << " added" << endl;
		clearRobotState(STATE_OPERATE);

		while ( !(getRobotState() & STATE_LEARN) )
		{
			if ( (getRobotState()&STATE_OPERATE) && rrt->numNodes > 1 )
			{
				Timestamp ts1("Path Planning Time");
				ts1.setMode(Timestamp::MODE_SHOW_MSEC | Timestamp::MODE_OVERLAP);
				ts1.setBaseline();

				pthread_mutex_lock(&mutex);
				qp1.clear();
				qp2.clear();
				dist.clear();
				distp.clear();

#define dD (0.01)
				int from = 0;
	//			int from = (int)((double)rand() * (double)rrt->numNodes / RAND_MAX);
				int to;
				
				if ( goalIdx < 0 )
					to = (int)((double)rand() * (double)rrt->numNodes / RAND_MAX);
				else
					to = goalIdx;
//					from = rrt->numNodes-1;
//					to = 0;
				fprintf(stderr, "NODE %d->%d/%d\n", from, to, rrt->numNodes);
				WbcPath path(rrt->nodes[from], rrt->nodes[to]);
				ts1.checkElapsed(0);
				path.step = 0.01*M_PI;
				path.optimize(100);
	//			path.optimize(NULL, 10);

				ts1.checkElapsed(1);
				Node<DOF> *p = rrt->nodes[rrt->numNodes-1];
				cerr << "Optimal path " << path.numNewNode << " nodes" << endl;
				for ( int i = 0 ; i < path.numNewNode ; i++ )
				{
					VectorXd q_plan= path.newNodes[i]->q;
					VectorXd Q;
#if 0
					Q = VectorXd::Zero(10);
					for ( int i = 0 ; i < DOF ; i++ )
					{
						Q(qmap[i]) = q_plan(i);
					}
					Q(2) = Q(1);
#else
					Q = getQ(q_plan);
#endif
					qp1.push_back(Q);
#ifdef USE_WBC
					((WbcNode *)path.newNodes[i])->getProjection();
#endif
//					cerr << getPotential(push_type, elbow0, (const WbcNode&)*(path.newNodes[i]), r0) << endl;

					// Derive elbow location to get distance
					pthread_mutex_lock(&link_mutex);
					myModel.updateState(Q);
					VectorXd elbow = myModel.joints[5].getGlobalPos(VectorXd::Zero(3));
					VectorXd dv;
					double dd;
					dv = elbow - elbow0;
					dd = dv.norm();
					dist.push_back(dd);
					if ( distp.size() > 0 && dd < distp.back() + dD ) 
						distp.push_back(distp.back() + dD);
					else
						distp.push_back(dd);
					pthread_mutex_unlock(&link_mutex);
				}
				int i;

//				for ( i = 0 ; i < dist.size() ; i++ )
//					cerr << dist[i] << endl;
//				cerr << endl << endl;
//				for ( i = 0 ; i < distp.size() ; i++ )
//					cerr << distp[i] << endl;
				ts1.checkElapsed(2);

				double d_max = dist.back();
				d_t.clear();
				d_t.resize(500);
				eta.clear();
				eta.resize(500);
				for ( i = 0 ; i < 500 ; i++ )
				{
					double t = i * 0.01;
					d_t[i] = d_max * ( 1. - exp(-2.5*t));
				}

				eta[0] = 0;
				int index = 1;
				for ( i = 1 ; i < 500 ; i++ )
				{
					double d = d_t[i];

					while ( index < 500 && d > distp[index] )
						index++;

					eta[i] = 0.;
					if ( index < 499 )
					{
						double difference = d - distp[index-1];
						double gap = distp[index] - distp[index-1];
						double ratio = difference / gap;

						if (gap <= 0. && ( ratio >= 1 || ratio < 0 ) )
						{
							assert(0);
						}

						eta[i] = index + ratio - 1;
					}
					else
						eta[i] = 500-1;
						
				}

#if 0
#if 1
				cerr << "d_t" << endl;
				for ( i = 0 ; i < 500 ; i++ )
				{
					int idx = (int)eta[i];
					double ratio = eta[i] - idx;
					double value1, value2;

					if ( index < 499 )
					{
						value1 = (1.-ratio) * dist[idx] + ratio * dist[idx+1];
						value2 = (1.-ratio) * distp[idx] + ratio * distp[idx+1];
					}
					else
					{
						value1 = dist[idx];
						value2 = distp[idx];
					}

					VectorXd Q = interpolate(qp1, eta[i]);

					pthread_mutex_lock(&link_mutex);
					myModel.updateState(Q);
					VectorXd elbow = myModel.joints[5].getGlobalPos(VectorXd::Zero(3));
					pthread_mutex_unlock(&link_mutex);


					VectorXd dv;
					double dd;
					dv = elbow - elbow0;
					dd = dv.norm();

					cerr << eta[i] << "  " << value1 << " " << value2 << " " << dd << endl;
				}
#else
				cerr << "eta" << endl;
				for ( int i = 0 ; i < 500 ; i++ )
					cerr << eta[i] << endl;
#endif
#endif

				cerr << "Original path " << path.numNode << " nodes" << endl;
				for ( int i = 0 ; i < path.numNode ; i++ )
				{
					VectorXd q_plan= path.nodes[i]->q;
					VectorXd Q;
#if 0
					Q = VectorXd::Zero(10);
					for ( int i = 0 ; i < DOF ; i++ )
					{
						Q(qmap[i]) = q_plan(i);
					}
					Q(3) = Q(2);
#else
					Q = getQ(q_plan);
#endif
					qp2.push_back(Q);
#if 0
					cerr << value[(*it).index] << " > ";

					double val = getPotential(0, VectorXd::Zero(3), Q, r0);
					cerr << "(" << val << ")"; 
#endif
#ifdef USE_WBC
					((WbcNode *)path.nodes[i])->getProjection();
#endif
//					cerr << getPotential(push_type, elbow0, (const WbcNode&)*(path.nodes[i]), r0) << endl;
				}
				pthread_mutex_unlock(&mutex);
				clearRobotState(STATE_OPERATE);
				ts1.checkElapsed(3);

				cerr << ts1;
			}
			usleep(10);
		}

		delete rrt;
		cerr << "Reset Plan" << endl;
	}
}

template <int T>
double project1( Node<T> *np )
{
	VectorXd dq0;
	VectorXd dq;
	VectorXd dst = np->q;

	assert(np->q.rows() == T);

	int trial;
	double err = 1.e10;

	bool violate = true;
	for ( trial = 100 ; trial >= 0 ; trial-- )
	{
#ifdef USE_WBC
		np->getProjection();
#endif


//	cerr << "New Node" << np->q.transpose()*180./M_PI << endl;


		Vector3d diff1 = desired_pos - np->actual;
		err = diff1.norm();

//	cerr << p.q.transpose() << endl;
//	cerr << err << endl;
#if 1
		if ( err <= 0.01 && !violate )
			break;

		cerr << err << "->";

		VectorXd modified;
//		cerr << "From  : " << p.q.transpose()<<endl;
//		cerr << "Before: " << np->q.transpose()<<endl;
//		modified = np->q + np->traction * diff1 + np->projection * diff2;
		modified = np->q + np->traction * diff1;
		np->q = modified;

		VectorXd diff2 = prm->center - np->q;
		for ( int i = 0 ; i < T ; i++ )
		{
			if ( diff2[i] > M_PI )
				diff2[i] -= 2.*M_PI;
			if ( diff2[i] < -M_PI )
				diff2[i] += 2.*M_PI;
		}
		np->q = prm->center - diff2;
		np->getProjection();
		MatrixXd weight = prm->weight;
		violate = false;
		for ( int i = 0 ; i < T ; i++ )
		{
			if ( (np->q(i) < Mins[i]) || (np->q(i) > Maxs[i]) )
			{
				violate = true;
				break;
			}
			else
			{
				weight(i,i) = 0.;
			}
		}
		modified = np->q + np->projection * weight * diff2;
		np->q = modified;
//		cerr << "After : " << np->q.transpose()<<endl;
//		np->getProjection();
#endif
	}
	if ( err > 0.01 || violate )
	{
		cerr << "Failed" << endl;
		return -1.;
	}

	cerr << "Found" << endl;

#if 0
	VectorXd diff = np->q - prm->center;
	for ( int i = 0 ; i < T ; i++ )
	{
		if ( diff[i] > M_PI )
			diff[i] -= 2.*M_PI;
		if ( diff[i] < -M_PI )
			diff[i] += 2.*M_PI;
	}
	np->q = prm->center + diff;

	violate = true;
	MatrixXd weight = prm->weight;
	for ( trial = 100 ; trial >= 0 && violate == true; trial-- )
	{
		VectorXd modified;
		VectorXd diff2;


		np->getProjection();
		diff2 = prm->center - np->q;

		modified = np->q + np->projection * weight * diff2;
		np->q = modified;

		violate = false;
		VectorXd q = np->q;
		weight = prm->weight;
		for ( int i = 0 ; i < T ; i++ )
		{
			if ( (q(i) < Mins[i]) || (q(i) > Maxs[i]) )
			{
				violate = true;
				break;
			}
			else
			{
				weight(i,i) = 0.;
			}
		}
	}
#if 1
	if ( violate )
	{
		cerr << "Mins  : " << Mins.transpose()*180./M_PI << endl;
		cerr << "EXCEED: " << np->q.transpose()*180./M_PI << endl;
		cerr << "Maxs  : " << Maxs.transpose()*180./M_PI << endl;
		err = -1.;
	}
#endif
#endif

	return err;
}

template <int T>
double project2( const Node<T> *_p, Node<T> *_np )
{
	WbcNode *p, *np;
	VectorXd dq0;
	VectorXd dq;
	VectorXd dst;

	p = (WbcNode *)_p;
	np = (WbcNode *)_np;

	dst = np->q;

	assert(np->q.rows() == T);
	assert(p->q.rows() == T);
	dq0 = np->q - p->q;
//	double mag0 = dq0.norm();

	dq = p->projection * dq0;

	double mag = dq.norm();

//	if ( mag < STEP && mag < mag0/2. )
//		return -1.;

	if ( dq0.transpose() * dq < 0)
	{
		cerr << "ERROR!!!: " << dq0.transpose() << ":" << dq.transpose() << endl;
		cerr << p->projection << endl;
		assert(0);
	}

	dq = dq * STEP / mag;

	np->q	= p->q + dq;

	VectorXd qdeg = np->q * 180. / M_PI;
#ifdef CHECK_LIMIT
	for ( int i = 0 ; i < T ; i++ )
	{
#if 0
		if ( qdeg(i) < mins[i] ) 
			np->q(i) = mins[i]*M_PI/180.;
		if ( qdeg(i) > maxs[i] )
			np->q(i) = maxs[i]*M_PI/180.;
#else
		if ( (qdeg(i) < mins[i]) || (qdeg(i) > maxs[i]) )
		{
//			cerr << "FRM:    " << p.q.transpose()*180./M_PI << endl;
//			cerr << "EXCEED: " << qdeg.transpose() << endl;
//			cerr << "DES:    " << dst.transpose()*180./M_PI << endl;
			return -1.;
		}

#endif
	}
#endif
//	cerr << "New Node" << np->q.transpose()*180./M_PI << endl;

#ifdef USE_WBC
	np->getProjection();
#endif

	Vector3d diff = desired_pos - np->actual;
	double err = diff.norm();


//	cerr << p.q.transpose() << endl;
//	cerr << err << endl;
	if ( err > 0.05 )
	{

//		cerr << "New   : " << np->actual.transpose() << endl;
//		cerr << "Error : " << diff.transpose() << endl;
//		cerr << "DES   : " << desired_pos.transpose() << endl;

		return -2.;
	}
#if 1
	else if ( err > 0.01 )
	{
		VectorXd modified;
//		cerr << "From  : " << p.q.transpose()<<endl;
//		cerr << "Before: " << np.q.transpose()<<endl;
		modified = np->q + np->traction * diff;
		np->q = modified;
//		cerr << "After : " << np.q.transpose()<<endl;
//		np->getProjection();
	}
#endif

	return mag;
}

VectorXd interpolate(const vector<VectorXd> &list, double seq)
{
	VectorXd ret;
	
	if ( list.size() )
	{
		int idx = (int)seq;
		double ratio = seq - idx;
		VectorXd q0, q1;

		if ( idx < list.size()-1 )
		{
			q0 = list[idx];
			q1 = list[idx+1];

			ret = (1.-ratio) * q0 + ratio * q1;
		}
		else
			ret = list[list.size()-1];
	}

	return ret;
}
