#ifndef __PRM_H__
#define __PRM_H__
#include <iostream>
#include <vector>
#include <list>
#include <Eigen/Eigen>
#include <sys/time.h>
#include "rrt.hpp"

using namespace std;
using std::vector;
using namespace Eigen;

template<int T>
class PRM
{
public:
	Node<T> **nodes;
	int numNodes;
	VectorXd	min;
	VectorXd	max;
	VectorXd	center;
	MatrixXd	weight;
	double		step;

	PRM(void);
	PRM(double min_, double max_, double step_ = 0.);
	PRM(VectorXd min_, VectorXd max_, double step_ = 0.);
	~PRM(void);
	void init(void);
	void post_init(void);
	void reset(void);
	void addNode( double (*projection)(  Node<T> & ) = NULL);
};

template <int T>
void PRM<T>::init(void)
{
	nodes		= (Node<T>**)calloc(sizeof(Node<T>*) , NUM_SAMPLE);
	numNodes = 0;
}

template <int T>
void PRM<T>::post_init(void)
{
	center = (min+max)/2.;

	weight	= MatrixXd::Zero(T,T);

	double max_interval = -1;
	for ( int i ; i < T ; i++ )
	{
		double interval = max(i) - min(i);
		assert(interval > 0.);
		weight(i,i) = 1. / interval;

		if ( interval > max_interval )
			max_interval = interval;
	}

//	weight = weight * max_interval;
}

template <int T>
void PRM<T>::reset(void)
{
	int i;
	for ( i = 0 ; i < numNodes ; i++ )
		delete nodes[i];
	numNodes = 0;
}

template <int T>
PRM<T>::PRM(void)
{
	init();

	min = VectorXd::Ones(T) * -1.;
	max = VectorXd::Ones(T);
	step = 0.01;

	post_init();
}

template <int T>
PRM<T>:: PRM(double min_, double max_, double step_)
{
	init();

	if ( step_ == 0. )
		step_ = max_ * 0.01;
	
	min = VectorXd::Ones(T) * min_;
	max = VectorXd::Ones(T) * max_;
	step = step_;

	post_init();
}

template <int T>
PRM<T>::PRM(VectorXd min_, VectorXd max_, double step_)
{
	init();

	if ( step_ == 0. )
		step_ = max_(0) * 0.01;
	min		= min_;
	max		= max_;
	step	= step_;

	post_init();
}

template <int T>
PRM<T>::~PRM(void)
{
	int i;
	for ( i = 0 ; i < numNodes ; i++ )
		delete nodes[i];
	free(nodes);
}

template <int T>
void PRM<T>::addNode( double (*projection)( Node<T> & ) )
{
	Node<T> goalNode	= Node<T>( min, max );
	Node<T> *newNode	= new Node<T>(goalNode);

	double err;

	err = projection(*newNode);

	if ( err > 0 && err <= 0.02 )
	{
		nodes[numNodes++]	= newNode;
	}
	else
	{
		delete newNode;
	}
}

#endif
