#ifndef ___KIN_H__
#define __KIN_H__
#include <iostream>
#include <Eigen/Eigen>

using namespace Eigen;

class Link
{
public:
	VectorXd	com;
	VectorXd	trans;
	MatrixXd	rot;	
	double		theta;
	double		mass;
	int			axis;
	Link		*parent;

				Link(void);
	VectorXd	getGlobal(const VectorXd &local);
	void		setRot(VectorXd quat);
	void		setRot(VectorXd axis, double deg);
	Link 		&operator=(const Link &src);
};



#endif // __KIN_H__
