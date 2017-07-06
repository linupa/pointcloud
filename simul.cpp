#include "config.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <Eigen/Eigen>

#include "wbcrrt.h"
#include "model.h"
#include "wbc.h"

using namespace std;
using namespace Eigen;

#ifdef USE_WBC
static scoped_ptr<jspace::Model> model;
#endif

Vector3d endeffector;
Timestamp ts1("Period");

VectorXd getQ(const VectorXd &uq);
VectorXd getQa(const VectorXd &q);

void initSimulation(const char *robot_spec)
{
#ifdef USE_WBC
	WbcNode::model.reset(test::parse_sai_xml_file(robot_spec, false));
	WbcNode::model->setConstraint("Dreamer_Torso");
	model.reset(test::parse_sai_xml_file(robot_spec, false));
	model->setConstraint("Dreamer_Torso");
#endif
}

#ifdef USE_WBC
MatrixXd getJacobian(const Model &model)
{
	MatrixXd Jfull, J;
	VectorXd actual_;

	taoDNode const *end_effector_node_ = model.getNode(DOF);
	jspace::Transform ee_transform;
	model.computeGlobalFrame(end_effector_node_,
		0.0, -0.05, 0.0, ee_transform);
	actual_ = ee_transform.translation();

	model.computeJacobian(end_effector_node_, actual_[0], actual_[1], actual_[2], Jfull);
	J = Jfull.block(0, 0, 3, Jfull.cols());

	return J;
}

void simulate(State &body_state)
{
	VectorXd desired_posture = VectorXd::Zero(DOF);
	VectorXd tau1, tau2, tau;
	VectorXd fullJpos_, fullJvel_;
	VectorXd actual_;
	MatrixXd UNcBar;
	MatrixXd phi;
	MatrixXd phiinv; 
	MatrixXd J1star;
	MatrixXd Lambda1;
	MatrixXd ainv;
	VectorXd grav;
	MatrixXd Nc;
	MatrixXd UNc;
	MatrixXd U;
	MatrixXd Jfull, J;
	MatrixXd N1, Lambda2;
	MatrixXd J2star;
	VectorXd	ddq;

	ts1.setBaseline();

	model->update(body_state);

	ts1.checkElapsed(0);

	fullJpos_ = model->getFullState().position_;
	fullJvel_ = model->getFullState().velocity_;

	model->getInverseMassInertia(ainv);

	model->getGravity(grav);

	Constraint * constraint = model->getConstraint();
	
	constraint->updateJc(*model);
	constraint->getNc(ainv,Nc);
	constraint->getU(U);
	UNc = U*Nc;

	ts1.checkElapsed(1);

#if 1
	taoDNode const *end_effector_node_ = model->getNode(9);
	jspace::Transform ee_transform;
	model->computeGlobalFrame(end_effector_node_,
		0.0, -0.15, 0.0, ee_transform);
	actual_ = ee_transform.translation();


	model->computeJacobian(end_effector_node_, actual_[0], actual_[1], actual_[2], Jfull);
	J = Jfull.block(0, 0, 3, Jfull.cols());
	ts1.checkElapsed(2);

#if 0 // Debugging for Jacobian function
	{
		static int count = 0;
		
		if ( (count %100) == 0 )
		{
			cerr << "====================" << endl;
			cerr << actual_.transpose() << endl;
			model->computeJacobian(end_effector_node_, actual_[0], actual_[1], actual_[2], Jfull);
			J = Jfull.block(0, 0, 3, Jfull.cols());
//				cerr << J.block(0,9,3,1).transpose() << endl;
//				cerr << J.block(0,8,3,1).transpose() << endl;
			cerr << J.block(0,0,3,10) << endl;
			Vector3d p = VectorXd::Zero(3);
			Vector3d w = VectorXd::Zero(3);
			Vector3d v, j, pp;
			VectorXd px;
			cerr << "--------------" << endl;
			p(1) = -0.15;
#if 0
			w(2) = 1;
			v = w.cross(p);
			pthread_mutex_lock(&link_mutex);

			q = getQ(body_state.position_); 
			myModel.updateState(q);

			j = myModel.joints[9].getGlobalOri()*v;
			cerr << j.transpose() << endl;

			pp = myModel.joints[9].getLocalPos(p,1);
			v = w.cross(pp);
			j = myModel.joints[8].getGlobalOri()*v;
			pthread_mutex_unlock(&link_mutex);
			cerr << j.transpose() << endl;
#else
			pthread_mutex_lock(&link_mutex);
			q = getQ(body_state.position_); 
			myModel.updateState(q);
			cerr << myModel.joints[9].getGlobalPos(p).transpose() << endl;
			pthread_mutex_unlock(&link_mutex);
			cerr << myModel.getJacobian(9, p) << endl << endl;
#endif
		}
		count++;
	}
#endif

#else
	MatrixXd J = getJacobian(model);
#endif
	
//		pthread_mutex_lock(&link_mutex);
	VectorXd q = getQ(body_state.position_); 
	myModel.updateState(q);
	VectorXd p = VectorXd::Zero(3);
	VectorXd x = myModel.joints[9].getGlobalPos(p);
	ts1.checkElapsed(3);
	J = myModel.getJacobian(9, p);
//		pthread_mutex_unlock(&link_mutex);

	ts1.checkElapsed(4);

	double kp = 100., kd = 50.0;

	phi = UNc * ainv * UNc.transpose();
	//XXXX hardcoded sigma threshold
	pseudoInverse(phi,
		0.0001,
		phiinv, 0);
	UNcBar = ainv * UNc.transpose() * phiinv;
	J1star = J * UNcBar;

	pseudoInverse( J1star * phi * J1star.transpose(),
		0.0001,
		Lambda1, 0);

	ts1.checkElapsed(5);

	desired_pos(0) = 0.4;
	desired_pos(1) = -0.2;
	desired_pos(2) = 0.2;

	desired_posture(3) = M_PI/6.;
	desired_posture(5) = M_PI/2.;
	desired_posture(10) = -M_PI/12.;
	desired_posture(12) = 0.;//M_PI/4.;

	N1 = MatrixXd::Identity(DOF,DOF) - phi*J1star.transpose()*Lambda1*J1star;

	J2star = U*UNcBar*N1;
	pseudoInverse(J2star*phi*J2star.transpose(),
		0.0001,
		Lambda2, 0);
	ts1.checkElapsed(6);

	tau1 =  J1star.transpose() * Lambda1 * kp * ( desired_pos - actual_ );
	double mag = tau1.norm();
//		if ( mag > 10. )
//			tau1 = tau1 / mag*10.;
	tau1 -=  J1star.transpose() * Lambda1 * kd * J * fullJvel_;
//	cout << "Lambda2 " << Lambda2.rows() << "x" << Lambda2.cols() << endl;
//	cout << "N1 " << N1.rows() << "x" << N1.cols() << endl;
//	cout << "vel " << body_state.position_.rows() << "x" << body_state.position_.cols() << endl;
//	cout << "UNcBar " << UNcBar.rows() << "x" << UNcBar.cols() << endl;
//	cout << "U " << U.rows() << "x" << U.cols() << endl;
//	tau2 =  UNcBar.transpose() * U.transpose() * Lambda2 * (kp * (-body_state.position_) - kd * body_state.velocity_);
	tau2 =  J2star.transpose() * Lambda2 * (10.*kp * ( desired_posture - body_state.position_) - kd * body_state.velocity_ - tau1);
//	cout << "ainv " << ainv.rows() << "x" << ainv.cols() << endl;
//	tau		= tau1 + UNcBar.transpose() * grav;
	tau		= tau1 + tau2 + UNcBar.transpose() * grav;
//	cout << "JAN1 " << endl << J * ainv * UNc.transpose() * N1.transpose() << endl;

//	cout << "ainv " << ainv.rows() << "x" << ainv.cols() << endl;
//	cout << "J1star " << J1star.rows() << "x" << J1star.cols() << endl;
//	cout << "Nc " << NcT.rows() << "x" << NcT.cols() << endl;
//	cout << "GRAV " << grav.rows() << "x" << grav.cols() << endl;
//	VectorXd ddq =  -ainv * NcT * grav;
	ddq = ainv * ( UNc.transpose()*tau - Nc.transpose() * grav );

//	cout << "ddt " << ddq.rows() << "x" << ddq.cols() << endl;
//	cout << "jVel " << fullJvel_.rows() << "x" << fullJvel_.cols() << endl;

	fullJvel_ += ddq * dt;
	fullJpos_ += fullJvel_ * dt;

#if 0
	for ( int i = 0 ; i < DOF ; i++ )
	{
		body_state.position_(i) = fullJpos_[qmap[i]];
		body_state.velocity_(i) = fullJvel_[qmap[i]];
		disp_q1[qmap[i]] = body_state.position_(i);
	}
	disp_q1[3] = fullJpos_[2];
#else
	body_state.position_ = getQa(fullJpos_);
	body_state.velocity_ = getQa(fullJvel_);
	endeffector				= actual_;
#endif
//	cerr << fullJvel_(1) << " " << fullJvel_(2) << endl;
	ts1.checkElapsed(7);

	{
		static int count = 0;

		if ( (count % 1000) == 0 )
		{
			{
				cout << ts1;
				cout << "stat " << body_state.position_.transpose() << endl;
				cout << "actual_ " << endl << actual_.transpose() << endl;
				cout << "full " << fullJpos_.transpose() << endl;
				cout << "full " << (fullJpos_*180./M_PI).transpose() << endl;
				cout << "tau1 " << tau1.transpose() << endl;
				cout << "tau2 " << tau2.transpose() << endl;
				VectorXd ddx = J * ddq; // ainv * UNc.transpose() * tau;
				cout << "ddx " << ddx.transpose() << endl;
				cout << "ddq " << ddq.transpose() << endl;
				cout << "J " << J << endl;
//				cout << "U " << U << endl;
			}
		}
		count++;
	}
}
#endif

