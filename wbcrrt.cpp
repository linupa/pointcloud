#include <jspace/Model.hpp>
#include <jspace/State.hpp>
#include <jspace/test/sai_util.hpp>
#include <jspace/pseudo_inverse.hpp>

#include <boost/scoped_ptr.hpp>

#include "wbcrrt.h"
#define CHECK_LIMIT

using namespace std;
using namespace boost;
using namespace jspace;

extern VectorXd desired_pos;
extern vector<Joint> myJoint;
extern pthread_mutex_t	model_mutex;
extern pthread_mutex_t	link_mutex;
extern VectorXd getQ(const VectorXd &uq);

scoped_ptr<jspace::Model> WbcNode::model;
WbcNode::WbcNode(void) 
{
	dim = DOF;
	q = VectorXd::Zero(dim);
	projection = MatrixXd::Identity(dim,dim);
//	edge	= NULL;
	depth	= 0;
}

WbcNode::WbcNode(const double min, const double max) : Node<DOF>(min,max)
{
	int i;

	q = VectorXd::Zero(dim);
	projection = MatrixXd::Identity(dim,dim);
	for ( i = 0 ; i < dim ; i++  )
		q[i] = min + rand() * ((max-min) / RAND_MAX);
//	edge	= NULL;
	parent	= NULL;
	depth	= 0;
}

WbcNode::WbcNode(const VectorXd& min, const VectorXd& max) : Node<DOF>(min,max)
{
	int i;

	projection = MatrixXd::Identity(dim,dim);
}

WbcNode::WbcNode(const WbcNode &src) 
{
	q		= src.q;
	projection	= src.projection;
	actual	= src.actual;
	parent	= src.parent;
	dim		= src.dim;
//	edge	= src.edge;
	depth	= src.depth;
	index	= src.index;
}

double WbcNode::distance(const Node *p2)
{
	double ret;
	double ssum;
	double xx, yy;
	int i;

	VectorXd diff = this->q - p2->q;

	ret = diff.norm();

	return ret;
}

double WbcNode::distance2(const Node *p2)
{
	double ret;
	double ssum;
	double xx, yy;
	int i;

	VectorXd diff = this->q - p2->q;

	ret = diff.transpose() * diff; 

	return ret;
}

WbcNode& WbcNode::operator=(const WbcNode &src)
{
	q			= src.q;
	projection	= src.projection;
	actual		= src.actual;
	parent		= src.parent;
	dim			= src.dim;
//	edge		= src.edge;
	depth		= src.depth;

	return *this;
}

WbcNode& WbcNode::operator=(const Node<DOF> &src)
{
	const WbcNode *pSrc = dynamic_cast<const WbcNode *>(&src);

	if ( pSrc )
		operator=(*pSrc);
	else
		Node<DOF>::operator=(src);

	return *this;
}

extern State body_state;
MatrixXd &WbcNode::getProjection(void)
{
	MatrixXd ainv;
	MatrixXd Nc;
	MatrixXd UNc;
	MatrixXd U;
	MatrixXd UNcBar;
	MatrixXd phi;
	MatrixXd phiinv; 
	MatrixXd J1star;
	MatrixXd J2star;
	MatrixXd Lambda1;
	MatrixXd Lambda2;
	MatrixXd Lambda2P;
	MatrixXd N1;
	MatrixXd J, Jfull;
	VectorXd actual_;

	Constraint *constraint;

	State state = body_state;
	state.position_ = q;
	state.velocity_ = VectorXd::Zero(state.velocity_.rows());

	pthread_mutex_lock(&model_mutex);
	model->update(state);

	model->getInverseMassInertia(ainv);

	constraint = model->getConstraint();
	constraint->getU(U);
	constraint->updateJc(*model);

	constraint->getNc(ainv,Nc);

	UNc = U*Nc;

#if 1
	taoDNode const *end_effector_node_ = model->getNode(9);
	jspace::Transform ee_transform;
	model->computeGlobalFrame(end_effector_node_,
		0.0, -0.05, 0.0, ee_transform);
	actual_ = ee_transform.translation();

	model->computeJacobian(end_effector_node_, actual_[0], actual_[1], actual_[2], Jfull);
	J = Jfull.block(0, 0, 3, Jfull.cols());
#else
	MatrixXd J = getJacobian(*model);
#endif
	pthread_mutex_unlock(&model_mutex);

	phi = UNc * ainv * UNc.transpose();
	//XXXX hardcoded sigma threshold
	pseudoInverse(phi,
		0.0001,
		phiinv, 0);
	UNcBar = ainv * UNc.transpose() * phiinv;

	J1star = J*UNcBar;
	pseudoInverse( J1star * phi * J1star.transpose(),
		0.0001,
		Lambda1, 0);

	N1 = MatrixXd::Identity(DOF,DOF) - phi*J1star.transpose()*Lambda1*J1star;
	J2star = U*UNcBar*N1;
	Lambda2P = J2star*phi*J2star.transpose();
	pseudoInverse(Lambda2P,
		0.0001,
		Lambda2, 0);

	projection		= Lambda2P * Lambda2;
	traction		= phi*J1star.transpose()*Lambda1;
	actual			= actual_;

#if 1
	int j;
	VectorXd		Q = getQ(q);
	pthread_mutex_lock(&link_mutex);
	for ( j = 0 ; j < 10 ; j++ )
	{
		myJoint[j].setTheta(Q[j]);
	}
	for ( j = 0 ; j < 10 ; j++ )
	{
		coms[j] = myJoint[j].getGlobalPos(myJoint[j].com);
	}
	pthread_mutex_unlock(&link_mutex);
#endif

//	return Lambda2;
	return projection;
}

double WbcNode::project( Node<DOF> *_np, double step, double max ) const
{
	int i, j;
	WbcNode *np;
	VectorXd dq0;
	VectorXd dq, ndq;
	VectorXd dst;
	VectorXd hand = VectorXd::Zero(3);
	hand(1) = -0.05;
	bool updated = false;

	np = (WbcNode *)_np;

	dst = np->q;

	assert(np->q.rows() == DOF);
	assert(this->q.rows() == DOF);
	dq0 = np->q - this->q;
//	double mag0 = dq0.norm();

	dq = this->projection * dq0;
	if ( dq0.transpose() * dq < 0)
	{
		cerr << "ERROR!!!: " << dq0.transpose() << ":" << dq.transpose() << endl;
		cerr << this->projection << endl;
		assert(0);
	}

	double mag = dq.norm();
	ndq = dq / mag;

	VectorXd diff, last_diff;
	VectorXd q = this->q;

	// increase step until the task error exceeds
	for ( i = 1 ; i*step <= max && i*step <= mag ; i++ )
	{
		bool violate_joint_limit = false;

		q += step*ndq;

#ifdef CHECK_LIMIT
		for ( j = 0 ; j < DOF ; j++ )
		{
			if ( (q(j) < mins[j]) || (q(j) > maxs[j]) )
			{
	//			cerr << "FRM:    " << this.q.transpose()*180./M_PI << endl;
	//			cerr << "EXCEED: " << q.transpose() << endl;
	//			cerr << "DES:    " << dst.transpose()*180./M_PI << endl;
				violate_joint_limit = true;
				break;
			}
		}
#endif
		if ( violate_joint_limit )
			break;

		VectorXd Q = getQ(q), r1;
		pthread_mutex_lock(&link_mutex);
		for ( int j = 0 ; j < 10 ; j++ )
		{
			myJoint[j].setTheta(Q(j));
		}
		r1 = myJoint[9].getGlobalPos(hand);
		pthread_mutex_unlock(&link_mutex);

	//	cerr << "New Node" << q.transpose()*180./M_PI << endl;
		diff = desired_pos - r1;
		double err = (diff.transpose() * diff)(0);
		if ( err > (0.05*0.05) )
		{

			cerr << "New   : " << r1.transpose() << endl;
			cerr << "DES   : " << desired_pos.transpose() << endl;
			cerr << "Error : " << diff.transpose() << endl;
			break;
		}
		else if ( err > 0.03*0.03 )
		{
//			VectorXd modified;
	//		cerr << "From  : " << this.q.transpose()<<endl;
	//		cerr << "Before: " << q.transpose()<<endl;
//			modified = np->q + np->traction * diff;
//			np->q = modified;
	//		cerr << "After : " << np.q.transpose()<<endl;
	//		np->getProjection();
			break;
		}
		updated = true;
		np->q = q;
		last_diff = diff;
	}

	if ( updated == false )
		return -1.;

//	np->q	= this->q + (i-1)*step*ndq;
	np->getProjection();
//	np->q = np->q + np->traction * last_diff;

#if 0
#endif

//	cerr << r1 - np->actual << endl;

//	cerr << this.q.transpose() << endl;
//	cerr << err << endl;

	return (i-1)*step;
}

Node<DOF> *WbcNode::copy(void) const
{
	WbcNode *ret = new WbcNode(*this);

	return (Node<DOF> *)ret;
}

WbcRRT::WbcRRT(void) 
{
}

WbcRRT::WbcRRT(double min_, double max_, double step_) : RRT<DOF>(min_, max_, step_ )
{
}

WbcRRT::WbcRRT(VectorXd min_, VectorXd max_, double step_) : RRT<DOF>(min_, max_, step_)
{
	cerr << "WbcRRT created" << endl;
}

Node<DOF> *WbcRRT::getNewNode(void)
{
	return (Node<DOF> *)new WbcNode;
}

WbcPath::WbcPath(void)
{
}

WbcPath::WbcPath(Node<DOF> *from, Node<DOF> *to) : Path<DOF>(from, to)
{
}

Node<DOF> *WbcPath::getNewNode(void)
{
	return (Node<DOF> *)new WbcNode;
}

WbcPath::~WbcPath(void)
{
}
