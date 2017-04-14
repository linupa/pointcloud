#ifndef __MODEL_H__
#define __MODEL_H__

#include "kin.h"

class KinModel
{
public:
	int numJoints;
	int numLinks;

	Link* localLinks;
	Link* globalLinks;
	Joint* joints;

	bool initJoint(char *filename);
	bool initLink(char *filename);
	void updateState(double *states);
	void updateState(VectorXd &states);
	~KinModel(void);
};

#endif
