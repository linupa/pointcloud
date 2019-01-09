#include <iostream>
#include "kin.h"

using namespace std;

Joints::Joints(const Joint &link)
{
	const Joint *src = &link;
	Joint *child = NULL;

	do {
		node = new Joint(*src);
		if ( child )
			child->parent = node;
		child = node;
	}
	while (src = src->parent);
}

Joints::~Joints(void)
{
	do 
	{
		Joint *link = node;

		node = node->parent;

		delete link;
	}
	while ( node );

}

Joint::Joint(void)
{
	rot = rot2 = Matrix3d::Zero();
	com = trans = Vector3d::Zero();
	mass	= 0.;
	axis	= 0;
	parent	= NULL;
	dirty	= true;
	index	= -1;
}

Joint::Joint(const Joint &src)
{
	strncpy(name, src.name, JOINT_NAME_LEN-1);
	name[0] = 'a';
	name[1] = '\0';
	com		= src.com;
	trans	= src.trans;
	rot		= src.rot;
	rot2	= src.rot2;
	mass	= src.mass;
	axis	= src.axis;
	parent	= src.parent;
	dirty	= src.dirty;
	index	= src.index;

	theta	= src.theta;
}

double Joint::setTheta(double theta_)
{
	if ( theta_ != theta )
		dirty = true;
	return theta = theta_;
}

double Joint::getTheta(void)
{
	return theta;
}

void Joint::updateOri(void)
{
	if ( dirty )
	{
		Matrix3d	act;
		int			x, y;
		int			axismap[3][2] = {{1,2}, {2,0}, {0,1}};

		x = axismap[axis][0];
		y = axismap[axis][1];

		assert(x < 3);
		assert(y < 3);

		act = Matrix3d::Identity(3,3);

		act(x,x) = cos(theta);
		act(x,y) = -sin(theta);
		act(y,x) = sin(theta);
		act(y,y) = cos(theta);

		rot2 = rot * act;

		dirty = false;
	}
}

Vector3d Joint::getGlobalPos(const Vector3d &local)
{
	return getGlobalPos(local, -1);
}

Vector3d Joint::getGlobalPos(const pos3D &local)
{
	VectorXd	_local(3);

	_local(0) = local.get(0);
	_local(1) = local.get(1);
	_local(2) = local.get(2);

	return getGlobalPos(_local, -1);
}

Vector3d Joint::getGlobalPos(const Vector3d &local, int depth_)
{
	Vector3d	ret;

	if ( depth_ == 0 )
		return local;

	updateOri();

	ret = rot2 * local + trans;

	if ( parent )
		ret = parent->getGlobalPos(ret, depth_-1);

	return ret;
}

Vector3d Joint::getGlobalPos(const pos3D &local, int depth_)
{
	Vector3d	_local;

	_local(0) = local.get(0);
	_local(1) = local.get(1);
	_local(2) = local.get(2);

	return getGlobalPos(_local, depth_);
}
Matrix3d Joint::getGlobalOri( void )
{
	Matrix3d	ret;

	updateOri();


	if ( parent )
		ret = parent->getGlobalOri() * rot2;
	else
		ret = rot2;

	return ret;
}

void Joint::getGlobalOri( double *mat )
{
	assert(mat);

	Matrix3d _rot = getGlobalOri();

	for ( int i = 0 ; i < 9 ; i++ )
	{
		int row = i/3;
		int col = i%3;
		mat[col*4+row] = _rot(row,col);
	}
	mat[15] = 1;

	return;
}

void Joint::setRot(VectorXd quat)
{
}

void Joint::setRot(VectorXd axis0, double rad)
{
	MatrixXd	Q, T;
	VectorXd	absAxis;
	VectorXd	axis1, axis2;
	int			one;
	int			zero;
	double		x, y, z;

//	cerr << axis0.transpose() << endl;
	Q = T = MatrixXd::Zero(3,3);
	Q.block(0,0,3,1) = axis0;
	axis1 = axis2 = VectorXd::Zero(3);
	x = axis0(0);
	y = axis0(1);
	z = axis0(2);

//	cerr << axis0 << endl;
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

//	cerr << ((one<<2) | zero) << endl;
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
//	cerr << Q << " " << det << endl;

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

//	cerr << rot << endl;

//	cerr << "done" << endl;


}

Joint &Joint::operator=(const Joint &src)
{
	strncpy(name, src.name, JOINT_NAME_LEN-1);
	com		= src.com;
	trans	= src.trans;
	rot		= src.rot;
	parent	= src.parent;
	theta	= src.theta;
	axis	= src.axis;
	mass	= src.mass;
	dirty	= src.dirty;
	index	= src.index;

	return  *this;
}

Joint &Joint::operator=(const double theta)
{
	setTheta(theta);

	return  *this;
}

bool Joint::isAdjacent(const Joint &j1, const Joint &j2)
{
	if ( j1.parent == &j2 )
		return true;
	if ( j2.parent == &j1 )
		return true;
	return false;
}



#define SMALL_NUM (0.01)

double
Link::getDistance( Link &S1, Link &S2)
{
	pos3D   u;
	pos3D   v;
	pos3D   w;
	u.sub( S1.to, S1.from);
	v.sub( S2.to, S2.from);
	w.sub( S1.from, S2.from);
	double    a = pos3D::dot(u,u);         // always >= 0
	double    b = pos3D::dot(u,v);
	double    c = pos3D::dot(v,v);         // always >= 0
	double    d = pos3D::dot(u,w);
	double    e = pos3D::dot(v,w);
	double    D = a*c - b*b;        // always >= 0
	double    sc, sN, sD = D;       // sc = sN / sD, default sD = D >= 0
	double    tc, tN, tD = D;       // tc = tN / tD, default tD = D >= 0
		
	// compute the line parameters of the two closest points
	if (D < SMALL_NUM) { // the lines are almost parallel
		sN = 0.0;         // force using point P0 on segment S1
		sD = 1.0;         // to prevent possible division by 0.0 later
		tN = e;
		tD = c;
	}
	else {                 // get the closest points on the infinite lines
		sN = (b*e - c*d);
		tN = (a*e - b*d);
		if (sN < 0.0) {        // sc < 0 => the s=0 edge is visible
		sN = 0.0;
		tN = e;
		tD = c;
	}
	else if (sN > sD) {  // sc > 1  => the s=1 edge is visible
		sN = sD;
		tN = e + b;
		tD = c;
		}
	}
		
	if (tN < 0.0) {            // tc < 0 => the t=0 edge is visible
		tN = 0.0;
		// recompute sc for this edge
		if (-d < 0.0)
			sN = 0.0;
		else if (-d > a)
			sN = sD;
		else {
			sN = -d;
			sD = a;
		}
	}
	else if (tN > tD) {      // tc > 1  => the t=1 edge is visible
		tN = tD;
		// recompute sc for this edge
		if ((-d + b) < 0.0)
			sN = 0;
		else if ((-d + b) > a)
			sN = sD;
		else {
			sN = (-d +  b);
			sD = a;
		}
	}
	// finally do the division to get sc and tc
	sc = (abs(sN) < SMALL_NUM ? 0.0 : sN / sD);
	tc = (abs(tN) < SMALL_NUM ? 0.0 : tN / tD);
		
	// get the difference of the two closest points
	pos3D  v1, v2, v3, dP; 
	v1 = u;
	v1.mul(sc);
	v2 = v;
	v2.mul(tc);
	v3.sub(v1,v2);
	dP.add(w, v3);
//	dP.add(w + (sc * u) - (tc * v);  // =  S1(sc) - S2(tc)
			
	return pos3D::norm2(dP);   // return the closest distance
}
	
