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
//	vector<XmlNode*> childs;
	XmlNode *childs;

	void parseXML(const TiXmlNode *parent);
};

#endif
