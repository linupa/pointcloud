#ifndef _RRT_H_
#define _RRT_H_
#include<iostream>
#include<vector>
#include<list>
#include<Eigen/Eigen>
#include <sys/time.h>
#include "timestamp.h"
using namespace std;
using std::vector;
using namespace Eigen;

//template<int T> class Edge;

template<int T>
class Node
{
public:
	VectorXd	q;
	Node<T>		*parent = NULL;
//	Edge<T>		*edge;
	int			depth;
	int			index;

	Node(const VectorXd &min, const VectorXd &max);
	Node(const Node<T> &src);

	virtual Node<T> &operator=(const Node<T> &src);
	virtual double project( Node<T> * , double step, double max) const;
	virtual double distance(const Node<T> *p2) const;
	virtual double distance2(const Node<T> *p2) const;
	virtual Node<T> *copy(void) const;

	typedef void (*CALLBACK)(Node<T> *node);
	static	CALLBACK callbackAdd;
	int		dim;

	static Node<T> *create(const VectorXd &min, const VectorXd &max);
};

template <int T>
typename Node<T>::CALLBACK Node<T>::callbackAdd = NULL;


template <int T>
Node<T>::Node(const VectorXd& min, const VectorXd& max)
{
	int i;

	dim = T;
	q = VectorXd::Zero(dim);
	for ( i = 0 ; i < dim ; i++  )
		q[i] = min[i] + rand() * ((max[i]-min[i]) / RAND_MAX);
//	edge	= NULL;
	parent	= NULL;
	depth	= 0;
}

#if 1
template <int T>
Node<T>::Node(const Node<T> &src)
{
//	assert(0);
	q		= src.q;
	parent	= src.parent;
	dim		= src.dim;
//	edge	= src.edge;
	depth	= src.depth;
	index	= src.index;
}
#endif

template <int T>
double Node<T>::distance(const Node *p2) const
{
	double ret;
	double ssum;
	double xx, yy;
	int i;

	VectorXd diff = this->q - p2->q;

	ret = diff.norm();

	return ret;
}

template <int T>
double Node<T>::distance2(const Node *p2) const
{
	double ret;
	double ssum;
	double xx, yy;
	int i;

	VectorXd diff = this->q - p2->q;

	ret = diff.transpose() * diff; 

	return ret;
}

template <int T>
Node<T> *Node<T>::copy(void) const
{
	cerr << "Copy Node" << endl;
	Node<T> *ret = new Node<T>(VectorXd::Zero(2), VectorXd::Zero(2));

	*ret = *this;

	return ret;
}

template <int T>
Node<T>& Node<T>::operator=(const Node<T> &src)
{
	q			= src.q;
	parent		= src.parent;
	dim			= src.dim;
//	edge		= src.edge;
	depth		= src.depth;

	return *this;
}

template <int T>
double Node<T>::project( Node<T> *np, double step, double max) const
{
	VectorXd dq;
	double dl;

	dq = np->q - this->q;
	dl = sqrt((dq.transpose() * dq)(0));

	if ( dl < step )
		return dl;

	dq = dq * (step / dl);

	np->q		= this->q + dq;
	
	return step;
}

template <int T>
Node<T> *Node<T>::create(const VectorXd &min, const VectorXd &max)
{
	return new Node<T>(min, max);
}


#if 0
template<int T>
class Edge
{
public:
	Node<T> p[2];
	Edge(void);
	Edge(const Node<T> &p1, const Node<T> &p2);
};
#endif

#define NUM_SAMPLE (200000)
#define INDEX_TYPE	int

template<int T>
class RRT
{
public:
	Node<T> **nodes;
//	Edge<T> **edges;
	double	**qs;
	int numNodes;
	int numEdges;
	INDEX_TYPE **sortList;
	INDEX_TYPE *temp;
	double sum[T];
	double ssum[T];
	double mean[T];
	double var[T];

	VectorXd	min;
	VectorXd	max;
	double		step;
	Timestamp	ts0;
	Timestamp	ts1;

	void init(void);
	RRT(void);
	RRT(double min_, double max_, double step_ = 0.);
	RRT(VectorXd min, VectorXd max_, double step_ = 0.);

	~RRT(void);
//	void iterate( void (*projection)(  VectorXd &, VectorXd & ) );
	void iterate( void );
	virtual double findPath( const Node<T> &node );
	Node<T>* findNearest(const Node<T> &node, double min);
	int findIndex(int dim, double val);
	void reset(const VectorXd &root);
	virtual ostream &dump(ostream &);
	virtual void addNode(Node<T> *pNode);
	bool checkBoundary(const Node<T> &node);
	Node<T> *(*nodeCreator)(const VectorXd &min, const VectorXd &max);
};

template <int T>
void RRT<T>::init(void)
{
	max = VectorXd::Ones(T);
	min = max * -1.;
	step = 0.01;

	nodes		= (Node<T>**)calloc(sizeof(Node<T>*) , NUM_SAMPLE);
//	edges		= (Edge<T>**)calloc(sizeof(Edge<T>*) , NUM_SAMPLE);
	sortList	= (INDEX_TYPE **)malloc(sizeof(INDEX_TYPE *) * T);
	sortList[0]	= (INDEX_TYPE *)malloc(sizeof(INDEX_TYPE) * NUM_SAMPLE * T);
	for ( int i = 1 ; i < T ; i++ )
		sortList[i] = sortList[0] + i * NUM_SAMPLE;
	temp		= (INDEX_TYPE *)malloc(sizeof(INDEX_TYPE) * NUM_SAMPLE);

	qs			= (double **)malloc(sizeof(double *) * NUM_SAMPLE);
	qs[0]		= (double *)malloc(sizeof(double) * NUM_SAMPLE * T);
	for ( int i = 1 ; i < NUM_SAMPLE ; i++ )
		qs[i] = qs[0] + i * T;

	nodeCreator = Node<T>::create;

//	if ( !nodes || !edges )
	if ( !nodes )
	{
		cerr << "ERROR" << endl;
	}
	else
		cerr << "ALLOC" << endl;

	numNodes = 0;
	numEdges = 0;
	memset((void *)sum, 0, sizeof(double)*T);
	memset((void *)ssum, 0, sizeof(double)*T);
	memset((void *)var, 0, sizeof(double)*T);
	memset((void *)mean, 0, sizeof(double)*T);


	ts0 = Timestamp("Total Elapsed Time");
	ts0.setBaseline();
	ts0.setMode(Timestamp::MODE_OVERLAP | Timestamp::MODE_SHOW_SEC);
	
	ts1 = Timestamp("Planning Time");
	ts1.setMode(Timestamp::MODE_SHOW_USEC | Timestamp::MODE_SHOW_COUNT);
}
template <int T>
RRT<T>::RRT(void)
{
	init();
}

template <int T>
RRT<T>:: RRT(double min_, double max_, double step_)
{
	init();

	if ( step_ == 0. )
		step_ = max_ * 0.01;
	
	VectorXd one = VectorXd::Ones(T);

	min = one * min_;
	max = one * max_;
	step = step_;
}

template <int T>
RRT<T>::RRT(VectorXd min_, VectorXd max_, double step_)
{
	cerr << "RRT created" << endl;
	init();

	if ( step_ == 0. )
		step_ = max_(0) * 0.01;
	min		= min_;
	max		= max_;
	step	= step_;
}

template <int T>
RRT<T>::~RRT(void)
{
	int i;
	for ( i = 0 ; i < numNodes ; i++ )
		delete nodes[i];
//	for ( i = 0 ; i < numEdges ; i++ )
//		delete edges[i];
	free(nodes);
//	free(edges);
	free(sortList[0]);
	free(sortList);
	free(qs[0]);
	free(qs);
	free(temp);
	numNodes = 0;
	numEdges = 0;
}

template <int T>
void RRT<T>::reset(const VectorXd &root)
{
	int i;
	for ( i = 0 ; i < numNodes ; i++ )
		delete nodes[i];
//	for ( i = 0 ; i < numEdges ; i++ )
//		delete edges[i];
	numNodes = 0;
//	numEdges = 0;
	memset((void *)sum, 0, sizeof(double)*T);
	memset((void *)ssum, 0, sizeof(double)*T);
	memset((void *)var, 0, sizeof(double)*T);
	memset((void *)mean, 0, sizeof(double)*T);

	ts0.setBaseline();

	Node<T>  *p_init = nodeCreator(VectorXd::Zero(T), VectorXd::Zero(T));
	p_init->q = root;

	addNode(p_init);

}

template <int T>
ostream &RRT<T>::dump(ostream &c)
{

	ts0.checkElapsed(0);

	c << numNodes << " Nodes are added " << endl;
	c << ts0;
	c << ts1;
}

template <int T>
Node<T>* RRT<T>::findNearest(const Node<T> &node, double min)
{
	double min2, min2_opt;
	Node<T> *ret, *ret_opt;
	int index;
	int i, j, k;
	double nodeq[T];

	for ( i = 0 ; i < T ; i++ )
	{
		nodeq[i] = node.q(i);
	}

#if 0
	VectorXd q = node.q;
	double	min_var = var[0];
	int		min_dim = 0;	
	for ( i = 1 ; i < T ; i++ )
	{
		if ( var[i] < min_var )
		{
			min_var	= var[i];
			min_dim	= i;
		}
	}

	index = findIndex(min_dim, q[min_dim]);
	if ( index >= numNodes )
		index = numNodes-1;

	const int		NUM_SIGN = 2;
	int				sign_map[NUM_SIGN] = {1, -1};
	unsigned int	search_flag[NUM_SIGN];
	
	for ( j = 0 ; j < NUM_SIGN ; j++ )
		search_flag[j] = 0xffffffff;

	min2_opt = 9999999.;

	double axis_val;
	Node<T> *pRef;
	double dist;

	if ( index >= numNodes )
		index = numNodes - 1;

	ret_opt = nodes[sortList[min_dim][index]];
	min2_opt = Node<T>::distance2(ret_opt, &node);

	if ( index > 0 )
		assert(nodes[sortList[min_dim][index-1]]->q[min_dim] < q[min_dim]);
	if ( index+1 < numNodes )
		assert(nodes[sortList[min_dim][index+1]]->q[min_dim] >= q[min_dim]);

	ret		= ret_opt;
	min2	= min2;
	for ( i = 1 ; i < numNodes ; i++ )
	{
		double min_axis = 9999999.;
		double min_dir;
		for ( k = 0 ; k < NUM_SIGN ; k++ )
		{
			int idx;
			bool done = false;

			idx = index + i*sign_map[k];

#if 1
			if ( !search_flag[k] )
			{
//					assert( idx < 0 || idx >= numNodes );
				continue;
			}
#endif
			

			if ( idx < 0 || idx >= numNodes )
			{
				done = true;
			}
			else 
			{
				Node<T> *pRef = nodes[sortList[min_dim][idx]];
				double axis_val;

				axis_val = fabs(pRef->q[min_dim] - q[min_dim]);

				if ( axis_val*axis_val > min2_opt )
				{
					done = true;
				}
			
				double dist2 = Node<T>::distance2(pRef, &node);

				if ( min2_opt > dist2 )
				{
					ret_opt		= pRef;
					min2_opt	= dist2;
				}

			}

			if ( done )
			{
				search_flag[k] = false;
			}
		}
		if ( !search_flag[0] && !search_flag[1] )
			break;
	}
//	cerr << "+-" << i << " Searched" << endl;

#else
	int minIdx = 0;
	double dist;
	double *q1;
	double *q2;
	double diff;
	min2 = 99999999.;
	for ( i = 0 ; i < numNodes && nodes[i] ; i++ )
	{
#if 0
		double dist = Node<T>::distance2(&node, nodes[i]);
#else
		dist = 0.;
		q1 = nodeq;
		q2 = qs[i];
		for ( j = 0 ; j < T ; j++,q1++,q2++ )
		{
//			diff = nodeq[j] - qs[i][j];
			diff = *q1 - *q2;
			dist += diff * diff;
		}
#endif
#if 0
		for ( j = 0 ; j < T ; j++ )
		{
			assert( qs[i][j] == nodes[i]->q[j] );
		}
#endif

		if ( dist < min2 )
		{
			minIdx = i;
			min2 = dist;
			if ( min2 < min*min )
				break;
		}
	}

	ret = nodes[minIdx];
#if 0
	if ( min2 != min2_opt )
	{
		cerr << ret->q.transpose() << endl;
		cerr << ret_opt->q.transpose() << endl;
		cerr << "MIN Diff " << min2 << " " << min2_opt << endl;
		assert(0);
	}
#endif
	ret_opt = ret;
#endif

	return ret_opt;
}

#if 0
template<int T>
Edge<T>::Edge(void)
{
	Node<T> p1;

	p[0] = p1;
	p[1] = p1;
}

template<int T>
Edge<T>::Edge(const Node<T> &p1, const Node<T> &p2)
{
	p[0] = p1;
	p[1] = p2;
}
#endif

template <int T>
void RRT<T>::iterate( void )
{
	Node<T> goalNode = Node<T>( min, max );

	findPath(goalNode);
}

template <int T>
double RRT<T>::findPath( const Node<T> &goalNode )
{
	double length = 0.;
	Node<T> *newNode = NULL;


	double min_distance2 = 1e20;
	while ( 1 )
//	for ( int i = 0 ; i < 1 ; i++ )
	{
//		mark_start_time();
		ts1.setBaseline();

		double distance2;
		Node<T>	*nearNode = findNearest(goalNode, step);

		ts1.checkElapsed(0);

		assert(nearNode);

//		newNode = nodeCreator(VectorXd::Zero(T), VectorXd::Zero(T));
		newNode = nodeCreator(min, max);
		
		distance2 = nearNode->distance2(&goalNode);

		if ( min_distance2 <= distance2 )
			break;
		min_distance2 = distance2;

		double dl;
		*newNode = goalNode;
		dl = nearNode->project(newNode, step, 3.*step);

		ts1.checkElapsed(1);

		if ( dl < 0 )
		{
			if ( dl > -1.5 )
				ts1.checkElapsed(7);
			else
				ts1.checkElapsed(8);
			break;
		}

		length += dl;

		if ( dl < step )
		{
			ts1.checkElapsed(9);
			break;
		}

		VectorXd diff = (*nearNode).q - (goalNode).q;
//		cerr << length << " " << diff.norm() << endl;

		newNode->parent = nearNode;

//	cerr << "NODE " << numNodes << " " <<  numEdges << endl;
#if 0
		if (!checkBoundary(*newNode))
		{
			cerr << dl << ":" << newNode->q.transpose()*180./M_PI << endl;
			assert(0);
		}
#endif
		ts1.checkElapsed(2);
		addNode(newNode);
		newNode = NULL;
		ts1.checkElapsed(3);
	}
	if ( newNode )
		delete newNode;

	return length;

//	cerr << "New Node " << nearNode.q[0] << ", " << nearNode.q[1] << " " << min.transpose() << " " << max.transpose() << " " << step << endl;
}

template <int T>
int RRT<T>::findIndex(int dim, double val)
{
	INDEX_TYPE		*pList	= sortList[dim];
	int		idx		= 0;
	double	upper, lower;
	int		upperIdx, lowerIdx;

	assert( dim < T ); 

	if ( numNodes > 0 )
	{
		upperIdx	= numNodes-1;
		lowerIdx	= 0;

		upper = nodes[pList[upperIdx]]->q(dim);
		lower = nodes[pList[lowerIdx]]->q(dim);

//			cerr << "Data" << dim << " " << val << endl;
		
		if ( val > upper )
		{
			idx = numNodes;
		}
		else  if ( val < lower )
		{
			idx = 0;
		}
		else 
		{
			int distance = upperIdx - lowerIdx;
			int midIdx = upperIdx;

			while ( distance > 1 )
			{
				double mid;
				distance = upperIdx - lowerIdx;
				midIdx = lowerIdx + (distance >> 1);
				mid = nodes[pList[midIdx]]->q(dim);

//					cerr << lowerIdx << ":" << midIdx << ":" << upperIdx << " " << mid << " ";


				if ( val > mid ) 
				{
					lowerIdx	= midIdx;
					lower		= mid;
				}
				else 
				{
					upperIdx	= midIdx;
					upper		= mid;
				}
			}
//				cerr << "[" << upperIdx << "] " ;
			idx = upperIdx;
		}

//			cerr << "->" << idx << endl;
		if ( idx > 0 )
			if (!(nodes[pList[idx-1]]->q(dim) <= val))
			{
				cerr << nodes[pList[idx-1]]->q(dim) << "[" << idx-1 << "] !< " <<  val << " " << numNodes << endl;
				assert(nodes[pList[idx-1]]->q(dim) <= val);
			}
		if ( idx < numNodes - 1 )
			if (!(nodes[pList[idx+1]]->q(dim) >= val))
			{
				cerr << nodes[pList[idx+1]]->q(dim) << "[" << idx+1 << "] !> " <<  val << " " << numNodes << endl;
				assert(nodes[pList[idx+1]]->q(dim) >= val);
			}
	}

	return idx;
}

template <int T>
void RRT<T>::addNode(Node<T> *newNode)
{
	int i, j;

	if ( numNodes == NUM_SAMPLE )
	{
		delete newNode;
		return;
	}

	if ( newNode->parent )
	{
//		Edge<T>	*newEdge = new Edge<T>;

//		newEdge->p[0] = *(newNode->parent);
//		newEdge->p[1] = *newNode;

//		newNode->edge	= newEdge;
		newNode->depth	= newNode->parent->depth + 1;

//		edges[numEdges++] = newEdge;
	}
	else
	{
//		newNode->edge	= NULL;
		newNode->depth	= 0;
	}

#if 0
	for ( i = 0 ; i < T ; i++ )
	{
		INDEX_TYPE	*pList	= sortList[i];
		int	idx = findIndex(i, newNode->q[i]);

		if ( idx < numNodes )
		{
			memcpy(temp, &(pList[idx]), sizeof(INDEX_TYPE)*(numNodes - idx));
			memcpy(&(pList[idx+1]), temp, sizeof(INDEX_TYPE)*(numNodes - idx));
		}
		pList[idx] = numNodes; 
	}
#endif
//	cerr << "Node " << numNodes << " Added " << newNode << endl;
//	cerr << newNode->q.transpose() << endl;

	newNode->index = numNodes;
	nodes[numNodes++] = newNode;

	for ( i = 0 ; i < T ; i++ )
	{
		sum[i] += newNode->q[i];
		ssum[i] += newNode->q[i] * newNode->q[i];
		mean[i] = sum[i] / numNodes;
		var[i] = ssum[i] / numNodes - mean[i]*mean[i];

		qs[newNode->index][i] = newNode->q[i];
	}

	if ( Node<T>::callbackAdd )
		Node<T>::callbackAdd(newNode);

//	cerr << "Add Node " << numNodes << endl;
//	cerr << nodes[numNodes-1]->q.transpose() << endl;

	return;
}

template <int T>
bool RRT<T>::checkBoundary(const Node<T> &node)
{
	int i;
	for ( i = 0 ; i < T ; i++ )
	{
		if ( node.q[i] < min(i) )
			return false;
		if ( node.q[i] > max(i) )
			return false;
	}
	
	return true;
}


template <int T>
ostream &operator<<(ostream &c, RRT<T> &rrt)
{
	rrt.dump(c);
	return c;
}

template <int T>
class Path
{
public:
	int numNode;
	int numNewNode;
	double step;
	Node<T> ** nodes;
	Node<T> ** testNodes;
	Node<T> ** newNodes;
	Path(void);
	Path(Node<T> *from, Node<T> *to);
	~Path(void);

	void optimize( int iteration = 10);
	double findPath( const Node<T> &from, const Node<T> &to, int &num );
};

template <int T>
Path<T>::Path(void)
{
	numNode		= 0;	
	numNewNode	= 0;
	nodes		= NULL;
	testNodes	= NULL;
	newNodes	= NULL;
}

template <int T>
Path<T>::Path(Node<T> *from, Node<T> *to)
{
	numNode	= from->depth + to->depth + 1;
	nodes = (Node<T> **)malloc(sizeof(Node<T> *)*numNode);

	int index = 0;
	Node<T> *pNode = from;
	while ( pNode->parent )
	{
		nodes[index++] = pNode->copy();
		pNode = pNode->parent;
	}
	nodes[index] = pNode->copy();

	pNode = to;
	int rindex = numNode - 1;
	while ( pNode->parent )
	{
		nodes[rindex--]=pNode->copy();
		pNode = pNode->parent;
	}

#if 0
	assert(index == rindex);
#else
	if ( !(index == rindex) )
	{
		cerr << "Different Root" << index << " " << rindex << endl;
		assert(0);
	}
#endif
}

template <int T>
Path<T>::~Path(void)
{
	int i;

	cerr << "Delete Path" << endl;
	if ( numNode > 0 )
	{
		for ( i = 0 ; i < numNode ; i++ )
			delete nodes[i];
		free(nodes);
	}
	if ( numNewNode > 0 )
	{
		for ( i = 0 ; i < numNewNode ; i++ )
			delete newNodes[i];
		free(newNodes);
	}
}
static Timestamp ts2;
template <int T>
void Path<T>::optimize( int iteration )
{
	int fromIdx, toIdx;
	Node<T> *from, *to;
	int i, j, index;
	int success, fail;

	success = fail = 0;

	if ( numNode <= 1 )
		return;

	newNodes = (Node<T> **)malloc(sizeof(Node<T> *) * numNode);
	for ( i = 0 ; i < numNode ; i++ )
	{
		newNodes[i] = nodes[i]->copy();
	}
	numNewNode = numNode;

//	return;

	Timestamp ts("Optimization");
	ts.setMode(Timestamp::MODE_SHOW_USEC | Timestamp::MODE_SHOW_COUNT );
	ts2 = Timestamp("findPath check");
	ts2.setMode(Timestamp::MODE_SHOW_USEC | Timestamp::MODE_SHOW_COUNT);
	cerr << "Optimize " << numNewNode << " nodes" << endl;
	fromIdx	= 0;
	toIdx	= numNewNode - 1;
	for ( int i = 0 ; i < iteration ; i++ )
	{
//		cerr << fromIdx << " " << toIdx << "(" << numNewNode << ")" <<endl;
		double length1 = 0.;
		double length2 = 0.;
		ts.setBaseline();

		for ( j = fromIdx+1 ; j <= toIdx ; j++ )
		{
			length1 += (newNodes[j]->q - newNodes[j-1]->q).norm();
		}
		from	= newNodes[fromIdx];
		to		= newNodes[toIdx];
		int count = toIdx - fromIdx;
		length2 = findPath(*from, *to, count);

		ts.checkElapsed(0);

//		cerr << "PATH " << testNodes.size() << " " << length1 << " " << length2 << endl;
		if ( length2 > 0 && length1 > length2 && count > 0 )
//		if ( length2 > 0 && testNodes.size() > 2 )
		{
			int numDel = toIdx - fromIdx - 1;
			int numAdd = count;
			Node<T> **tempNodes = (Node<T> **)calloc(sizeof(Node<T>*), (numNewNode + numAdd - numDel));
			for ( j = fromIdx+1 ; j <= toIdx-1 ; j++ )
				delete newNodes[j];

			memcpy(&tempNodes[0], &newNodes[0], sizeof(Node<T>*) * (fromIdx+1));
			memcpy(&tempNodes[fromIdx+1], &testNodes[0], sizeof(Node<T>*) * numAdd);
			memcpy(&tempNodes[fromIdx+numAdd+1], &newNodes[toIdx], sizeof(Node<T>*) * (numNewNode-toIdx));

			numNewNode = numNewNode + numAdd - numDel;
			free(newNodes);
			newNodes = tempNodes;
			ts.checkElapsed(1);

//			for ( j = 0 ; j < numNewNode ; j++ )
//				cerr << newNodes[j]->q.transpose() << endl;
//				assert(newNodes[j]);
			success++;
		}
		else
		{
			fail++;
		}
		fromIdx	= (int)((double)rand() * (double)(numNewNode-2) / RAND_MAX);
		toIdx	= fromIdx + 2 +(int)((double)rand() * (double)(numNewNode-fromIdx-2) /RAND_MAX);
		ts.checkElapsed(2);
	}

	cerr << ts;
	cerr << ts2;


	for ( j = 1 ; j < numNewNode ; j++ )
	{
		double dist = newNodes[j]->distance2(newNodes[j-1]);

		if ( dist > 10*step )
		{
			cerr << "Too large step " << dist << ">" << step << endl;
		}
		else
		{
//			cerr << "PATH: " << dist << ">" << step << endl;
		}
	}
	cerr << "Sucess: " << success << " Fail: " << fail << endl;
}

template <int T>
double Path<T>::findPath( const Node<T> &from, const Node<T> &to, int &num)
{
	double length = 0.;
	const Node<T>	*nearNode = &from;
	Node<T>			*newNode;
	double dist2, prev_dist2;
	double diff;
	int		idx;
	VectorXd	dq;

	num = num * 2;
	testNodes = (Node<T> **)calloc(sizeof(Node<T> *), num+1);
	dq = from.q - to.q;
	prev_dist2 = dist2 = (dq.transpose()*dq)(0);

	idx = 0;
	while ( idx < num )
	{
		double		mag;

		ts2.setBaseline();
//		cerr << idx << endl;
		newNode = to.copy();

		if ( nearNode->project(newNode, step, 10.*step) < 0. )
		{
//			cerr << "Collision" << endl;
			delete newNode;
			break;
		}
		ts2.checkElapsed(0);


		VectorXd err = newNode->q - to.q;
		dist2 = (err.transpose()*err)(0);
		if ( dist2 > prev_dist2 )
		{
			cerr << endl << "ERROR " << dist2 << ":" << prev_dist2 << endl;
			delete newNode;
			assert(0);
		}
		else
		{
//			cerr << dist << "->"; 
		}
		ts2.checkElapsed(1);

//		cerr << "NODE " << mag << " " << dq.norm() << " " << length << endl;
		newNode->depth	= nearNode->depth + 1;
		newNode->parent	= NULL;
//		newNode.edge	= NULL;
		testNodes[idx++] = newNode;

		dq = nearNode->q - newNode->q;
		diff = dq.norm();
//		length += step;
//		cerr << diff << "->"; 

		length += diff;
		ts2.checkElapsed(2);

		nearNode = newNode;
		if ( diff < step/2. )
		{
			cerr << "Stuck " << diff << endl;
			break;
		}
		if ( dist2 < step*step )
			break;
		prev_dist2 = dist2;
	}

	if ( dist2 > step*step )
	{
//		cerr << "Failed to find path " << diff << " " << step<< endl;
#if 1
		for ( int j = 0 ; j < num ; j++ )
		{
			if ( testNodes[j] )
				delete testNodes[j];
		}
		free(testNodes);
#endif
		return -1.;
	}
	num = idx;

	return length;

//	cerr << "New Node " << nearNode.q[0] << ", " << nearNode.q[1] << " " << min.transpose() << " " << max.transpose() << " " << step << endl;
}


#endif // _RRT_H_

