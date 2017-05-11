#ifndef __MODEL_H__
#define __MODEL_H__

#include "kin.h"
#include "xml.h"

class KinModel
{
public:
	int numJoints;
	int numLinks;

	Link* localLinks;
	Link* globalLinks;
	int* collision;
	Joint* joints;

	bool initJoint(char *filename);
	bool initLink(char *filename);
	void addJoint(const XmlNode *node, Joint *parent, int &index);
	int numJoint(const XmlNode *node);
	void updateState(double *states);
	void updateState(VectorXd &states);
	bool checkCollision(void);
	~KinModel(void);
};

#endif
