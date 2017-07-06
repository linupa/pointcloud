#ifndef __WBC_H__
#define __WBC_H__
#include "Octree.h"
#include "prm.hpp"
enum {
	STATE_IDLE		= 0x00000000,
	STATE_SIMUL		= 0x00000001,
	STATE_LEARN		= 0x00000100,
	STATE_LEARNING	= 0x00000200,
	STATE_OPERATE	= 0x00010000,
	STATE_OPERATING	= 0x00020000,
	STATE_SEND		= 0x01000000
};
void setRobotState(int st);
void clearRobotState(int st);
int getRobotState(void);

extern int win_width;
extern int win_height;
extern double hor;
extern double ver;
extern double scale;

extern VectorXd disp_q1;
extern VectorXd disp_q2;
extern VectorXd q0;
extern vector<VectorXd> qp1;
extern vector<VectorXd> qp2;
extern KinModel myModel;

extern PRM<DOF> *prm;
extern WbcRRT *rrt;

extern int link_ui;
extern int link2_ui;
extern int num_alpha;
extern int		link_a[100];
extern double alpha[100];
extern int		link_b[100];
extern double beta[100];

extern double dt;
extern Vector3d elbow0;
extern Vector3d elbow;
extern Vector3d endeffector;
extern Vector3d desired_pos;
extern vector<Vector3d> elbow_log;
extern int goalIdx;
extern double	gGoalTime;
extern Vector3d gGoal;
extern Vector3d gObj;
extern Vector3d	gObjVel;
extern int pushType;

extern pthread_mutex_t	mutex;
extern pthread_mutex_t	model_mutex;
extern pthread_mutex_t link_mutex;
#endif
