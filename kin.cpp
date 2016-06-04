#include <iostream>
#include "kin.h"

using namespace std;

Links::Links(const Link &link)
{
	const Link *src = &link;
	Link *child = NULL;

	do {
		node = new Link(*src);
		if ( child )
			child->parent = node;
		child = node;
	}
	while (src = src->parent);
}

Links::~Links(void)
{
	do 
	{
		Link *link = node;

		node = node->parent;

		delete link;
	}
	while ( node );

}

Link::Link(void)
{
	rot = MatrixXd::Zero(3,3);
	com = trans = VectorXd::Zero(3);
	mass	= 0.;
	parent = NULL;
	dirty	= true;
}

double Link::setTheta(double theta_)
{
	dirty = true;
	return theta = theta_;
}

double Link::getTheta(void)
{
	return theta;
}

VectorXd Link::getGlobal(const VectorXd &local)
{
	VectorXd	ret;

	if ( dirty )
	{
		MatrixXd	act;
		int			x, y;
		int			axismap[3][2] = {{1,2}, {2,0}, {0,1}};

		x = axismap[axis][0];
		y = axismap[axis][1];

		assert(x < 3);
		assert(y < 3);

		act = MatrixXd::Identity(3,3);

		act(x,x) = cos(theta);
		act(x,y) = -sin(theta);
		act(y,x) = sin(theta);
		act(y,y) = cos(theta);

		rot2 = rot * act;

		dirty = false;
	}

	ret = rot2 * local + trans;

	if ( parent )
		ret = parent->getGlobal(ret);

	return ret;
}

void Link::setRot(VectorXd quat)
{
}

void Link::setRot(VectorXd axis0, double rad)
{
	MatrixXd	Q, T;
	VectorXd	absAxis;
	VectorXd	axis1, axis2;
	int			one;
	int			zero;
	double		x, y, z;

	cerr << axis0.transpose() << endl;
	Q = T = MatrixXd::Zero(3,3);
	Q.block(0,0,3,1) = axis0;
	axis1 = axis2 = VectorXd::Zero(3);
	x = axis0(0);
	y = axis0(1);
	z = axis0(2);

	cerr << axis0 << endl;
	absAxis = axis0.transpose() * axis0.asDiagonal();

	if ( absAxis(0) >= absAxis(1) && absAxis(0) >= absAxis(2) )
	{
		if ( absAxis(1) >= absAxis(2) )
		{
			if ( fabs(y) < 0.0001 )
				axis1(1) = 0.;
			else
				axis1(1) = -z / y;
			axis1(2) = 1.;
			one = 2;
		}
		else
		{
			axis1(1) = 1.;
			if ( fabs(x) < 0.0001 )
				axis1(2) = 0.;
			else
				axis1(2) = -y / z;
			one = 1;
		}
		zero = 0;
	}
	else if ( absAxis(1) >= absAxis(2) )
	{
		if ( absAxis(0) >= absAxis(2) )
		{
			if ( fabs(x) < 0.0001 )
				axis1(0) = 0.;
			else
				axis1(0) = -z / x;
			axis1(2) = 1.;
			one = 2;
		}
		else
		{
			axis1(0) = 1.;
			if ( fabs(z) < 0.0001 )
				axis1(2) = 0.;
			else
				axis1(2) = -x / z;
			one = 0;
		}
		zero = 1;
	}
	else
	{
		if ( absAxis(0) >= absAxis(1) )
		{
			if ( fabs(x) < 0.0001 )
				axis1(0) = 0.;
			else
				axis1(0) = -y / x;
			axis1(1) = 1.;
			one = 1;
		}
		else
		{
			axis1(0) = 1.;
			if ( fabs(y) < 0.0001 )
				axis1(1) = 0.;
			else
				axis1(1) = -x / y;
			one = 0;
		}
		zero = 2;
	}

	cerr << ((one<<2) | zero) << endl;
	switch ((one<<2) | zero)
	{
		case ( (0<<2) | 1 ):
			axis2(2) = 1.;
			axis2(0) = axis1(2) * -1.;
			axis2(1) = -1.*(axis0(0)*axis2(0) + axis0(2)) / axis0(1);
			break;
		case ( (0<<2) | 2 ):
			axis2(1) = 1.;
			axis2(0) = axis1(1) * -1.;
			axis2(2) = -1.*(axis0(0)*axis2(0) + axis0(1)) / axis0(2);
			break;
		case ( (1<<2) | 0 ):
			axis2(2) = 1.;
			axis2(1) = axis1(2) * -1.;
			axis2(0) = -1.*(axis0(1)*axis2(1) + axis0(2)) / axis0(0);
			break;
		case ( (1<<2) | 2 ):
			axis2(0) = 1.;
			axis2(1) = axis1(0) * -1.;
			axis2(2) = -1.*(axis0(1)*axis2(1) + axis0(0)) / axis0(2);
			break;
		case ( (2<<2) | 0 ):
			axis2(1) = 1.;
			axis2(2) = axis1(1) * -1.;
			axis2(0) = -1.*(axis0(2)*axis2(2) + axis0(1)) / axis0(0);
			break;
		case ( (2<<2) | 1 ):
			axis2(0) = 1.;
			axis2(2) = axis1(0) * -1.;
			axis2(1) = -1.*(axis0(2)*axis2(2) + axis0(0)) / axis0(1);
			break;
		default:
			cerr << "ASSERT!!! Impossible combination: " << one << " " << zero << endl;
	}

	axis0 = axis0 / axis0.norm();
	axis1 = axis1 / axis1.norm();
	axis2 = axis2 / axis2.norm();

	Q.block(0,0,3,1) = axis0;
	Q.block(0,1,3,1) = axis1;
	Q.block(0,2,3,1) = axis2;

	double det = Q.determinant();
	if ( det < 0. )
	{
		Q.block(0,1,3,1) = axis2;
		Q.block(0,2,3,1) = axis1;
	}
	cerr << Q << " " << det << endl;

	T(0,0) = 1.;
	T(1,1) = cos(rad);
	T(1,2) = -sin(rad);
	T(2,1) = sin(rad);
	T(2,2) = cos(rad);

	assert(fabs(axis0.transpose()*axis1) < 0.001);
	assert(fabs(axis1.transpose()*axis2) < 0.001);
	assert(fabs(axis2.transpose()*axis0) < 0.001);
	assert(fabs(Q.determinant() - 1) < 0.001);

	rot = Q * T * Q.transpose();

	cerr << rot << endl;

	cerr << "done" << endl;


}

Link &Link::operator=(const Link &src)
{
	com		= src.com;
	trans	= src.trans;
	rot		= src.rot;
	parent	= src.parent;
	theta	= src.theta;
	axis	= src.axis;
	mass	= src.mass;

	return  *this;
}
