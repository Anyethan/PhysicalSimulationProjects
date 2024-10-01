#include "IK.h"
#include "FK.h"
#include "minivectorTemplate.h"
#include <Eigen/Dense>
#include <adolc/adolc.h>
#include <cassert>
#if defined(_WIN32) || defined(WIN32)
  #ifndef _USE_MATH_DEFINES
    #define _USE_MATH_DEFINES
  #endif
#endif
#include <math.h>
using namespace std;

// CSCI 520 Computer Animation and Simulation
// Jernej Barbic and Yijing Li

namespace
{

// Converts degrees to radians.
template<typename real>
inline real deg2rad(real deg) { return deg * M_PI / 180.0; }

template<typename real>
Mat3<real> euler2Rotation(const vector <real> angle , RotateOrder order)
{
	Mat3<real> RX = Mat3<real>::getElementRotationMatrix(0, deg2rad(angle[0]));
	Mat3<real> RY = Mat3<real>::getElementRotationMatrix(1, deg2rad(angle[1]));
	Mat3<real> RZ = Mat3<real>::getElementRotationMatrix(2, deg2rad(angle[2]));

	switch (order)
	{
	case RotateOrder::XYZ:
		return RZ * RY * RX;
	case RotateOrder::YZX:
		return RX * RZ * RY;
	case RotateOrder::ZXY:
		return RY * RX * RZ;
	case RotateOrder::XZY:
		return RY * RZ * RX;
	case RotateOrder::YXZ:
		return RZ * RX * RY;
	case RotateOrder::ZYX:
		return RX * RY * RZ;
	}
	//assert(0);
}

// Performs forward kinematics, using the provided "fk" class.
// This is the function whose Jacobian matrix will be computed using adolc.
// numIKJoints and IKJointIDs specify which joints serve as handles for IK:
//   IKJointIDs is an array of integers of length "numIKJoints"
// Input: numIKJoints, IKJointIDs, fk, eulerAngles (of all joints)
// Output: handlePositions (world-coordinate positions of all the IK joints; length is 3 * numIKJoints)
template<typename real>
void forwardKinematicsFunction(
    int numIKJoints, const int * IKJointIDs, const FK & fk,
    const std::vector<real> & eulerAngles, std::vector<real> & handlePositions)
{
  // Students should implement this.
  // The implementation of this function is very similar to function computeLocalAndGlobalTransforms in the FK class.
  // The recommended approach is to first implement FK::computeLocalAndGlobalTransforms.
  // Then, implement the same algorithm into this function. To do so,
  // you can use fk.getJointUpdateOrder(), fk.getJointRestTranslation(), and fk.getJointRotateOrder() functions.
  // Also useful is the multiplyAffineTransform4ds function in minivectorTemplate.h .
  // It would be in principle possible to unify this "forwardKinematicsFunction" and FK::computeLocalAndGlobalTransforms(),
  // so that code is only written once. We considered this; but it is actually not easily doable.
  // If you find a good approach, feel free to document it in the README file, for extra credit.
	
	//compute the current IKhandle positions from its euler angles, and then we get F(thelta), so we can compute J
	//real is adouble here, tracing all involved calculations

	int numJoints = fk.getNumJoints();

	vector<Mat3<real>>  localRotate(numJoints);
	vector<Vec3<real>>  localTrans(numJoints);

	vector<Mat3<real>>  globalRotate(numJoints);
	vector<Vec3<real>>  globalTrans(numJoints);

	vector<Mat3<real>> R_ori(numJoints);  //R_child_o
	vector <vector<real>> ori(numJoints);
	for (int i = 0; i < ori.size(); i++)
		ori[i].resize(3);
	vector<Mat3<real>> R_angle(numJoints); //R_child_local
	vector <vector<real>> euler(numJoints);
	for (int i = 0; i < euler.size(); i++)
		euler[i].resize(3);
	

	for (int i = 0; i < numJoints; i++)
	{
		//int i = IKJointIDs[d];
		
		ori[i][0]= fk.getJointOrient(i)[0];
		ori[i][1] = fk.getJointOrient(i)[1];
		ori[i][2] = fk.getJointOrient(i)[2];


		R_ori[i] = euler2Rotation(ori[i], fk.getJointRotateOrder(i));

		
		euler[i][0] = eulerAngles[3 * i];
		euler[i][1] = eulerAngles[3 * i + 1];
		euler[i][2] = eulerAngles[3 * i + 2];
		R_angle[i] = euler2Rotation(euler[i], fk.getJointRotateOrder(i));

		localRotate[i] = R_ori[i]*R_angle[i]; // R_ori[i]*

		localTrans[i][0] = fk.getJointRestTranslation(i)[0];
		localTrans[i][1] = fk.getJointRestTranslation(i)[1];
		localTrans[i][2] = fk.getJointRestTranslation(i)[2];
		
	}
	real flag=0.0;;
	for (int i = 0; i < numJoints; i++) {
		//int i = IKJointIDs[d];  //id for d_th IKjoints in all joints list
		//M_child_global = localrotate[fk.getJointUpdateOrder(i)] * globalTransforms[fk.getJointParent(fk.getJointUpdateOrder(i))];

		if (fk.getJointParent(fk.getJointUpdateOrder(i)) >= 0) { 
			multiplyAffineTransform4ds(globalRotate[fk.getJointParent(fk.getJointUpdateOrder(i))], globalTrans[fk.getJointParent(fk.getJointUpdateOrder(i))]
				, localRotate[fk.getJointUpdateOrder(i)], localTrans[fk.getJointUpdateOrder(i)], globalRotate[fk.getJointUpdateOrder(i)], globalTrans[fk.getJointUpdateOrder(i)]);
			flag = 1.0;
		}
		else {
			globalRotate[fk.getJointUpdateOrder(i)] = localRotate[fk.getJointUpdateOrder(i)]; 
			globalTrans[fk.getJointUpdateOrder(i)] = localTrans[fk.getJointUpdateOrder(i)];

		}
	}
	vector<Vec3<real>> posi(numIKJoints);
	vector<Vec3<real>> handle(numIKJoints);
	for (int d = 0; d < numIKJoints; d++) {
		int i = IKJointIDs[d]; //joint ID of IK handler
		Vec3d pos = fk.getJointRestTranslation(i); //positions of rest gesture
		cout <<i<<" "<< pos<<endl;
		posi[d] = { fk.getJointRestTranslation(IKJointIDs[d])[0], fk.getJointRestTranslation(IKJointIDs[d])[1],fk.getJointRestTranslation(IKJointIDs[d])[2] };
		handle[d] = globalRotate[IKJointIDs[d]] *posi[d] + globalTrans[IKJointIDs[d]];
		handlePositions[3 * d] = handle[d][0];  
		handlePositions[3 * d + 1] = handle[d][1];  
		handlePositions[3 * d + 2] = handle[d][2];

	}
}
	
} // end anonymous namespaces

IK::IK(int numIKJoints, const int * IKJointIDs, FK * inputFK, int adolc_tagID)
{
  this->numIKJoints = numIKJoints;
  this->IKJointIDs = IKJointIDs;
  this->fk = inputFK;
  this->adolc_tagID = adolc_tagID;

  FKInputDim = fk->getNumJoints() * 3;
  FKOutputDim = numIKJoints * 3;

  train_adolc();
}

void IK::train_adolc()
{
  // Students should implement this.
  // Here, you should setup adol_c:
  //   Define adol_c inputs and outputs. 
  //   Use the "forwardKinematicsFunction" as the function that will be computed by adol_c.
  //   This will later make it possible for you to compute the gradient of this function in IK::doIK
  //   (in other words, compute the "Jacobian matrix" J).
  // See ADOLCExample.cpp .

	int n = FKInputDim; // input dimension is n, eulerangles
	int m = FKOutputDim; // output dimension is m, newPositions

	// first, call trace_on to ask ADOL-C to begin recording how function f is implemented
	trace_on(adolc_tagID); // start tracking computation with ADOL-C

	// ADOL-C uses its own double implementation: adouble to track floating-point operations used in f
	// adouble overloads basic C++ arithmetic operations like =,+,-,*,/ so that ADOL-C knows when and where
	// adoubles are involved in which operations.
	// Note: since adoubles are not real doubles, they have complicated data structures inside. So
	// you shouldn't use direct memory functions like memcpy((adouble*)..., ..., ...) to edit adoubles.
	vector<adouble> x(n); // define the input of the function f
	for (int i = 0; i < n ; i++) {
		x[i] <<= 0.0;
	}

	vector<adouble> y(m); // define the output of the function f
	//for (int j = 0; j < 1e7; ++j) { x[0] = x[0] + 0 * j; }
	// The computation of f goes here:
	
	forwardKinematicsFunction<adouble>(numIKJoints, IKJointIDs, *fk, x , y);

	vector<double> output(m);
	for (int i = 0; i < m; i++) {
		y[i] >>= output[i]; // Use >>= to tell ADOL-C that y[i] are the output variables
	}
	  // Finally, call trace_off to stop recording the function f.
	trace_off(); // ADOL-C tracking finished
}

void IK::doIK(const Vec3d * targetHandlePositions, Vec3d * jointEulerAngles)
{
	// You may find the following helpful:
	int numJoints = fk->getNumJoints(); // Note that is NOT the same as numIKJoints!

	// Students should implement this.
	// Use adolc to evalute the forwardKinematicsFunction and its gradient (Jacobian). It was trained in train_adolc().
	// Specifically, use ::function, and ::jacobian .
	// See ADOLCExample.cpp .
	//
	// Use it implement the Tikhonov IK method (or the pseudoinverse method for extra credit).
	// Note that at entry, "jointEulerAngles" contains the input Euler angles. 
	// Upon exit, jointEulerAngles should contain the new Euler angles.

	// now, you can call ::function(adolc_tagID, ...) as many times as you like to ask ADOL-C to evaluate f for different x:
	Eigen::VectorXd actualDiff(numIKJoints * 3);
	//restTranslation is local; globalTransform[*][3] is global location
	for (int i = 0; i < numIKJoints; i++) {
		Vec3d rest_pose = fk->getJointRestTranslation(IKJointIDs[i]); //local posi 
		Vec4d rest = fk->getJointGlobalTransform(fk->getJointParent(IKJointIDs[i])) * Vec4d(rest_pose[0], rest_pose[1], rest_pose[2], 1);

		actualDiff(3 * i) = targetHandlePositions[i][0] - rest[0];
		actualDiff(3 * i + 1) = targetHandlePositions[i][1] - rest[1];
		actualDiff(3 * i + 2) = targetHandlePositions[i][2] - rest[2];
		//Extra--------
		if (abs(actualDiff[3 * i]) > 0.4 || abs(actualDiff[3 * i+1]) > 0.4|| abs(actualDiff[3 * i+2]) > 0.4) {
			Vec3d* reduced_targetposi = new Vec3d[numIKJoints];
			for (int j = 0; j < numIKJoints; j++) {
				reduced_targetposi[j][0] = targetHandlePositions[j][0];
				reduced_targetposi[j][1] = targetHandlePositions[j][1];
				reduced_targetposi[j][2] = targetHandlePositions[j][2];
			}
			double step = 0.2;
			if (abs(actualDiff[3 * i]) > 0.4) {
				
					if (actualDiff[3 * i] < 0) step = -0.2;
					reduced_targetposi[i][0] = targetHandlePositions[i][0] - step;
				
				doIK(reduced_targetposi, jointEulerAngles);
			}
			if (abs(actualDiff[3 * i + 1]) > 0.4) {
				
					if (actualDiff[3 * i+1] < 0) step = -0.2;
					reduced_targetposi[i][1] = targetHandlePositions[i][1] - step;
				
				doIK(reduced_targetposi, jointEulerAngles);

			}if (abs(actualDiff[3 * i + 2]) > 0.4) {
				
					if (actualDiff[3 * i+2] < 0) step = -0.2;
					reduced_targetposi[i][2] = targetHandlePositions[i][2] - step;
				
				doIK(reduced_targetposi, jointEulerAngles);
			}
			delete reduced_targetposi;
			rest = fk->getJointGlobalTransform(fk->getJointParent(IKJointIDs[i])) * Vec4d(rest_pose[0], rest_pose[1], rest_pose[2], 1);

			actualDiff(3 * i) = targetHandlePositions[i][0] - rest[0];
			actualDiff(3 * i + 1) = targetHandlePositions[i][1] - rest[1];
			actualDiff(3 * i + 2) = targetHandlePositions[i][2] - rest[2];
		}

		//if (IKJointIDs[i] == 15)cout << "handler" << IKJointIDs[i] << ": " << targetHandlePositions[i][0] << "  " << rest[0] << " / " << targetHandlePositions[i][1] << "  " << rest[1]
			//<< " / " << targetHandlePositions[i][2] << "  " << rest[2] << endl;
	}

	double* input_x_values = new double[FKInputDim];
	for (int i = 0; i < FKInputDim/3; i++) {
		input_x_values[3*i] = jointEulerAngles[i ][0];
		input_x_values[3*i+1] = jointEulerAngles[i ][1];
		input_x_values[3*i+2] = jointEulerAngles[i][2];
		//if(i==15)
		//cout << input_x_values[3 * i] << " " << input_x_values[3 * i + 1] << " " << input_x_values[3 * i + 2]<<" / ";
	}
	//cout << endl;
	double* output_y_values = new double[FKOutputDim];
	for (int i = 0; i < FKOutputDim; i++) {
		output_y_values[i] = 0.0;
	}

	::function(adolc_tagID, FKOutputDim, FKInputDim, input_x_values, output_y_values);
	//for (int i = 0; i < FKOutputDim; i++) {
		//if(i<=2)
		//cout<<output_y_values[i]<<" "; //meibian !=0
	//}
	//cout << endl;
	// You can call ::jacobian(adolc_tagID, ...) as many times as you like to ask ADOL-C to evalute the jacobian matrix of f on different x:
	double** jacobianMatrix = new double* [FKOutputDim]; // We store the matrix in row-major order.
	for (int i = 0; i < FKOutputDim; i++) {
		jacobianMatrix[i] = new double[FKInputDim];
		for (int j = 0; j < FKInputDim; j++) {
			jacobianMatrix[i][j] = 1.0;
		}
	}
	//double ** jacobianMatrixEachRow; // pointer array where each pointer points to one row of the jacobian matrix
	
	::jacobian(adolc_tagID, FKOutputDim, FKInputDim, input_x_values, jacobianMatrix); // each row is the gradient of one output component of the function
	//J»áÎª0£¡
	
	Eigen::MatrixXd J(numIKJoints*3, numJoints*3);/////////////////////////
			
	//cout << endl;
	for (int i = 0; i < numIKJoints*3; i++) {
		
		for (int j = 0; j < numJoints*3; j++) {
			J(i,j) = jacobianMatrix[i][j];
			//cout << J(i, j); //all 0 now
		}	
		//cout << output_y_values[i];
		//cout << endl;
	}
	
	//---------------------------------------------IK Method------------------------------------------------
	int IKmode = 1;

	//-------------------Init------------------
	
	Eigen::VectorXd x;
	//---------------------1. Tikhonov-------------
	if (IKmode == 0) {
		Eigen::MatrixXd I = Eigen::MatrixXd::Identity(numJoints * 3, numJoints * 3);
		// assign values to A
		Eigen::MatrixXd A;
		A = J.transpose()*J + 0.001*I; //larger alpha, smaller change; 0 is too unstables

		Eigen::VectorXd b = J.transpose()*actualDiff;

		// now solve for x in A x = b
	  // note: here, we assume that A is symmetric; hence we can use the LDLT decomposition
		x = A.ldlt().solve(b);
		
		// check the accuracy of the solution by computing: ||Ax-b||_2
		//double error1 = (A * x).norm();
		//double error2 = b.norm();
		//double error = error1 - error2;
		//cout << "System solve error: " << error << endl;
	}
	//-------------------2. Pseudo Inverse----------
	else if (IKmode == 1) {
		Eigen::MatrixXd J_dagger = J.transpose()*(J*J.transpose()).inverse();
		x = J_dagger*actualDiff;
	}
	
	//--------------------------------------------finalize-----------------------------------
	for (int i = 0; i < FKInputDim / 3; i++) {
		//if(i==15)cout << i<<": "<<jointEulerAngles[i];
		jointEulerAngles[i][0] = jointEulerAngles[i][0] +x[3*i]; 
		jointEulerAngles[i][1] = jointEulerAngles[i][1] + x[3 * i + 1];
		jointEulerAngles[i][2] = jointEulerAngles[i][2] + x[3 * i + 2];
		//if(i==15)cout << jointEulerAngles[i]<<endl;
	}

	delete input_x_values, output_y_values, jacobianMatrix;
	
}

