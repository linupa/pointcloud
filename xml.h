#ifndef _XML_H_
#define _XML_H_
#include <tinyxml.h>
#include <Eigen/Dense>
#include <vector>

using namespace Eigen;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

class XmlNode
{
public:
	char name[20];
	VectorXd	rot;
	VectorXd	pos;
	VectorXd	com;
	double		mass;

	XmlNode(void);
	XmlNode* sibling;
	XmlNode *child;

	void parseXML(const TiXmlNode *parent);
};

class XmlLinkNode
{
public:
	char name[20];
	VectorXd	from;
	VectorXd	to;
	int			index;
	double		radius;
	double		length;

	XmlLinkNode(void);
	vector<XmlLinkNode *> linkList;

	void parseXML(const TiXmlNode *parent);
};

#endif
