#ifndef __KIN_H__
#define __KIN_H__
#include <iostream>
#include <Eigen/Eigen>

using namespace Eigen;

class Joint;
class pos3D;

class Joints
{
public:
	Joint *node;
	Joints(const Joint &link);
	~Joints(void);
};

class Joint
{
public:
	VectorXd	com;
	VectorXd	trans;
	MatrixXd	rot;	
	MatrixXd	rot2;
	double		mass;
	int			axis;
	Joint		*parent;
	bool		dirty;

				Joint(void);
	void		updateOri(void);
	VectorXd	getGlobalPos(const pos3D &local);
	VectorXd	getGlobalPos(const VectorXd &local);
	VectorXd	getGlobalPos(const double *local);
	MatrixXd	getGlobalOri(void);
	void		getGlobalOri(double *mat);
	void		setRot(VectorXd quat);
	void		setRot(VectorXd axis, double deg);
	Joint 		&operator=(const Joint &src);
	double		setTheta(double theta_);
	double		getTheta(void);

private:
	double		theta;
};

class pos3D
{
	double x[3];

	public:
	pos3D& add(pos3D &p1, pos3D & p2)
	{
		x[0] = p1.x[0] + p2.x[0];
		x[1] = p1.x[1] + p2.x[1];
		x[2] = p1.x[2] + p2.x[2];

		return *this;
	}
	pos3D& sub(pos3D &p1, pos3D & p2)
	{
		x[0] = p1.x[0] - p2.x[0];
		x[1] = p1.x[1] - p2.x[1];
		x[2] = p1.x[2] - p2.x[2];

		return *this;
	}
	pos3D& mul(double val)
	{
		x[0] = x[0] * val;
		x[1] = x[1] * val;
		x[2] = x[2] * val;

		return *this;
	}
	double* operator=(double *src)
	{
		x[0] = src[0];	
		x[1] = src[1];	
		x[2] = src[2];	

		return x;
	}
	double* operator=(const pos3D &src)
	{
		x[0] = src.x[0];	
		x[1] = src.x[1];	
		x[2] = src.x[2];	

		return x;
	}

	double* operator=(const VectorXd &src)
	{
		x[0] = src[0];	
		x[1] = src[1];	
		x[2] = src[2];	

		return x;
	}
	
	double get(const int index) const
	{
		return x[index];
	}

	double& operator[](const int index)
	{
		return x[index];
	}
	static double dot(const pos3D &p1, const pos3D &p2)
	{
		return p1.x[0]*p2.x[0] + p1.x[1]*p2.x[1] + p1.x[2]*p2.x[2];
	}
	static double norm2(const pos3D &p) 
	{
		return dot(p,p);
	}
	static double norm(const pos3D &p) 
	{
		return sqrt(dot(p,p));
	}
};

class Link
{
	public:
	int		index;
	pos3D	from;
	pos3D	to;
	double	radius;
	double	length;

	static  double getDistance( Link &p1, Link &p2 );
};



#endif // __KIN_H__
