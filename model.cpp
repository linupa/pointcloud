#include "model.h"
#include <tinyxml.h>
#include "xml.h"

using namespace std;
extern int link_ui;
extern int link2_ui;
extern int		link_a[100];
extern double alpha[100];
extern int		link_b[100];
extern double beta[100];
extern int num_alpha;

void KinModel::addJoint(const XmlNode *node, Joint *parent, int &index)
{

	Joint joint;
	int myIndex = index;

//	if ( node->name[0] )
//		cerr << node->name << " " << myIndex << endl;
//	else
//		cerr << "---" << endl;

	if ( node->name[0] )
		memcpy(joint.name, node->name, JOINT_NAME_LEN-1);
	else
		memcpy(joint.name, "---", 4);
	joint.com = node->com;
	joint.setRot(node->rot.block(0,0,3,1), node->rot(3));
	joint.trans	= node->pos;
	joint.axis	= 2;
	joint.mass	= node->mass;
	joint.parent = parent;
	joint.index = index;
	joints[index++] = joint;

	if ( node->child )
		addJoint(node->child, &joints[myIndex], index);
	if ( node->sibling )
		addJoint(node->sibling, parent, index);
}

int KinModel::numJoint(const XmlNode *node)
{
	int ret = 1;

	if ( node->child )
		ret += numJoint(node->child);
	if ( node->sibling )
		ret += numJoint(node->sibling);

	return ret;
}

bool KinModel::initJoint(char *filename)
{
	const char *robot_spec;
	bool loaded = false;
	TiXmlDocument doc;

	try 
	{
		doc = TiXmlDocument( filename );
		robot_spec = filename;
		loaded = doc.LoadFile();

		cerr << "Load XML File " << filename << " " << loaded << endl;
	}
	catch (int e)
	{
		std::cerr << "ERROR!!!";
	}

	int i, j;

	XmlNode baseNode;
	cerr << "###################################" << endl;
	baseNode.parseXML(&doc);

	XmlNode *node = baseNode.child;

	cerr << node << endl;
	numJoints = numJoint(node);
	cerr << "Number of joints: " << numJoints << endl;
	cerr << "###################################" << endl;

	joints = new Joint[numJoints];

	int index = 0;
	node = baseNode.child;
#if 1
	addJoint(node, NULL, index);

	cerr << "done." << endl;

	for ( int i = 0 ; i < numJoints ; i++ )
	{
		cerr << joints[i].name << endl;
		if (i != joints[i].index)
		{
			cerr << i << " " << joints[i].index << endl;
			assert(0);
		}
	}
#else
	i = 0;
	while ( node )
	{
		if ( node->name )
			cerr << node->name << endl;
		else
			cerr << "---" << endl;

		Joint joint;

		joint.com = node->com;
		joint.setRot(node->rot.block(0,0,3,1), node->rot(3));
		joint.trans	= node->pos;
		joint.axis	= 2;
		joint.mass	= node->mass;
		if ( i > 0 )
			joint.parent = &(joints[i-1]);
		else
			joint.parent = NULL;
		joints[i] = joint;

		node = node->child;
		i++;
	}
#endif
}

bool KinModel::initLink(char *filename)
{
	bool loaded = false;
	TiXmlDocument doc;

	try 
	{
		doc = TiXmlDocument( filename );
		loaded = doc.LoadFile();

		cerr << "Load XML File " << loaded << endl;
	}
	catch (int e)
	{
		std::cerr << "ERROR!!!";
	}

	int i, j;

	XmlLinkNode baseNode;
	cerr << "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@" << endl;
	baseNode.parseXML(&doc);
	numLinks = baseNode.linkList.size();

	localLinks = new Link[numLinks];
	collision = new int[numLinks];

	cerr << numLinks << " Links" << endl;
	i = 0;
	vector<XmlLinkNode *>::iterator it = baseNode.linkList.begin();
	for ( ; it != baseNode.linkList.end() ; it++ )
	{
		if ( (*it)->name )
			cerr << (*it)->name << endl;
		
		localLinks[i].index = (*it)->index;
		localLinks[i].from = (*it)->from;
		localLinks[i].to = (*it)->to;
		localLinks[i].radius = (*it)->radius;
		localLinks[i].neighbor = (*it)->neighbor;
		collision[i] = -1;
		i++;
	}
	cerr << "Link Init Done" << endl;
}

void KinModel::updateState(double *states)
{
	for ( int j = 0 ; j < numJoints ; j++ )
	{
		joints[j] = states[j];
	}
}

void KinModel::updateState(VectorXd &states)
{
	for ( int j = 0 ; j < numJoints ; j++ )
	{
		joints[j] = states[j];
	}
}

bool KinModel::checkCollision(void)
{
	int i, j, k;
	Link *l1, *l2;
	int idx = 0;
	bool ret = false;
	collision[numLinks-1] = collision[numLinks-2] = -1;
	for ( i = 0 ; i < numLinks ; i++ )
		collision[i] = -1;
	for ( i = 0 ; i < numLinks-1 ; i++ )
	{
		l1 = &(localLinks[i]);
		for ( j = i+1 ; j < numLinks ; j++ )
		{
			bool skip = false;
			l2 = &(localLinks[j]);
			// Collision checking betwen i-th link and j-th link
			for ( k = 0 ; k < l1->neighbor.rows() ; k++ )
			{
				if ( l1->neighbor[k] == l2->index )
					skip = true;
			}

			if ( skip )
				continue;

			Vector3d x[4];
			Vector3d p, d1, d2;
		
			x[0] = joints[l1->index].getGlobalPos(l1->from);
			x[1] = joints[l1->index].getGlobalPos(l1->to);
			x[2] = joints[l2->index].getGlobalPos(l2->from);
			x[3] = joints[l2->index].getGlobalPos(l2->to);
			p = x[0] - x[2];
			d1 = x[1] - x[0];
			d2 = x[3] - x[2];

			double d11, d22, d12, d1p, d2p;
			double _alpha, _beta;
			double mc2;

			d11 = d1.transpose() * d1;
			d22 = d2.transpose() * d2;
			d12 = d1.transpose() * d2;
			d1p = d1.transpose() * p;
			d2p = d2.transpose() * p;

			mc2 = fabs(d11*d22 - d12*d12);
			if ( mc2 > 0.0001 )
			{
				_alpha	= ( d12*d2p - d22*d1p ) / mc2;
				_beta	= ( d11*d2p - d12*d1p ) / mc2;
			}
			else
			{
				_alpha = 0.;
				_beta = d2p / d22;
			}


			if ( _alpha < 0. )
				_alpha = 0.;
			if ( _alpha > 1. )
				_alpha = 1.;
			if ( _beta < 0. )
				_beta = 0.;
			if ( _beta > 1. )
				_beta = 1.;

			VectorXd d = p + _alpha*d1 - _beta*d2;
			double dist2 = d.transpose()*d;
			double r2;

			r2 = (l1->radius+l2->radius)*(l1->radius+l2->radius);

			if ( i == link_ui && j == link2_ui )
			{
				link_a[idx] = i;
				link_b[idx] = j;
				alpha[idx] = _alpha;
				beta[idx] = _beta;
				idx++;
				cerr << x[0].transpose() << " ~ " << x[1].transpose() << endl;
				cerr << x[2].transpose() << " ~ " << x[3].transpose() << endl;
				cerr << _alpha << "," << _beta << "(" << mc2 << ")" << endl;
				cerr << dist2 << " " << r2 << endl;
			}

			if ( dist2 < r2 )
			{
				collision[i] = j;
				collision[j] = i;
				ret = true;
			}

			num_alpha = idx;
		}
	}

	return true;
}

MatrixXd KinModel::getJacobian(int index, VectorXd pos)
{
	MatrixXd ret;
	Joint *current;
	Joint *check;
	Joint *parent;

	ret = MatrixXd::Zero(3, numJoints);
	current	= &joints[index];
	check = current;

	for ( int i = 0 ; i < numJoints &&  check ; i++ )
	{	
		Vector3d w, v, pp;
		w = Vector3d::Zero();
		w(2) = 1.;

		parent = check->parent;
		pp = current->getGlobalPos(pos, i);
		v = check->getGlobalOri()*w.cross(pp);

		ret.block(0, check->index, 3, 1) = v;

		check = check->parent;
	}

	return ret;
}

KinModel::~KinModel(void)
{
	delete joints;
	delete localLinks;
	delete collision;
}
