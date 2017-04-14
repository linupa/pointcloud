#include "model.h"
#include <tinyxml.h>
#include "xml.h"

using namespace std;

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

		cerr << "Load XML File " << loaded << endl;
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
	i = 0;
	while ( node )
	{
		node = node->child;
		i++;
	}
	numJoints = i;
	cerr << numJoints << endl;
	cerr << "###################################" << endl;

	joints = new Joint[numJoints];

	node = baseNode.child;
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
		i++;
	}
	cerr << "Link Init Done" << endl;
}

void KinModel::updateState(double *states)
{
	for ( int j = 0 ; j < numJoints ; j++ )
	{
		joints[j].setTheta(states[j]);
	}
}

void KinModel::updateState(VectorXd &states)
{
	for ( int j = 0 ; j < numJoints ; j++ )
	{
		joints[j].setTheta(states[j]);
	}
}

KinModel::~KinModel(void)
{
	delete joints;
}
