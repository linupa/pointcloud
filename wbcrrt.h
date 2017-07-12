#ifndef __WBCRRT_H__
#define __WBCRRT_H__

#ifdef USE_WBC
#include <boost/scoped_ptr.hpp>
#include <jspace/Model.hpp>
#include <jspace/State.hpp>
#include <jspace/test/sai_util.hpp>
#include <jspace/pseudo_inverse.hpp>
using namespace boost;
using namespace jspace;
#endif
#include <Eigen/Eigen>


#define DOF		(16)
#define LINKS	(17)

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;


#include "rrt.hpp"
#include "kin.h"

typedef RRT<DOF> WbcRRT;

class WbcNode : public Node<DOF>
{
public:
	MatrixXd	projection;
	MatrixXd	traction;
	VectorXd	actual;
	Vector3d 	coms[LINKS];
#ifdef USE_WBC
	State 		state;
#endif

	virtual	WbcNode &operator=(const WbcNode &src);
	virtual	WbcNode &operator=(const Node<DOF> &src);
	virtual double distance(const Node<DOF> *p2);
	virtual double distance2(const Node<DOF> *p2);
	virtual Node<DOF> *copy(void) const;

	VectorXd &setState(const VectorXd &v);

	WbcNode(void);
	WbcNode(const double min, const double max);
	WbcNode(const VectorXd &min, const VectorXd &max);
	WbcNode(const WbcNode &src);
	WbcNode(const Node<DOF> &src);

	MatrixXd &getProjection(void);
	virtual double project( Node<DOF> * , double step, double max) const;

#ifdef USE_WBC
	static scoped_ptr<jspace::Model> model;
#endif
	static Node<DOF> *create(const VectorXd &min, const VectorXd &max);
	static VectorXd	mins;
	static VectorXd	maxs;
};

class WbcPath : public Path<DOF>
{
public:
	WbcPath(void);
	WbcPath(Node<DOF> *from, Node<DOF> *to);
	~WbcPath(void);

	virtual Node<DOF> *getNewNode(void);
};

#endif
