#include "skinning.h"
#include "vec3d.h"
#include <algorithm>
#include <cassert>
#include <iostream>
#include <fstream>

#include <quaternion.h>
using namespace std;
namespace {
	void Quaternion2Rot(Quaternion<double> & q, double* R)
	{
		// students should implement this
		//double R[9];
		q.Quaternion<double>::Quaternion2Matrix(R);
		//Rotation2Euler(R, angles );
	}
}

// CSCI 520 Computer Animation and Simulation
// Jernej Barbic and Yijing Li

Skinning::Skinning(int numMeshVertices, const double * restMeshVertexPositions,
    const std::string & meshSkinningWeightsFilename)
{
  this->numMeshVertices = numMeshVertices;
  this->restMeshVertexPositions = restMeshVertexPositions;

  cout << "Loading skinning weights..." << endl;
  ifstream fin(meshSkinningWeightsFilename.c_str());
  assert(fin);
  int numWeightMatrixRows = 0, numWeightMatrixCols = 0;
  fin >> numWeightMatrixRows >> numWeightMatrixCols;
  assert(fin.fail() == false);
  assert(numWeightMatrixRows == numMeshVertices);
  int numJoints = numWeightMatrixCols;

  vector<vector<int>> weightMatrixColumnIndices(numWeightMatrixRows);
  vector<vector<double>> weightMatrixEntries(numWeightMatrixRows);
  fin >> ws;
  while(fin.eof() == false)
  {
    int rowID = 0, colID = 0;
    double w = 0.0;
    fin >> rowID >> colID >> w;
    weightMatrixColumnIndices[rowID].push_back(colID);
    weightMatrixEntries[rowID].push_back(w);
    assert(fin.fail() == false);
    fin >> ws;
  }
  fin.close();

  // Build skinning joints and weights.
  numJointsInfluencingEachVertex = 0;
  for (int i = 0; i < numMeshVertices; i++)
    numJointsInfluencingEachVertex = std::max(numJointsInfluencingEachVertex, (int)weightMatrixEntries[i].size());
  assert(numJointsInfluencingEachVertex >= 2);

  // Copy skinning weights from SparseMatrix into meshSkinningJoints and meshSkinningWeights.
  meshSkinningJoints.assign(numJointsInfluencingEachVertex * numMeshVertices, 0);
  meshSkinningWeights.assign(numJointsInfluencingEachVertex * numMeshVertices, 0.0);
  for (int vtxID = 0; vtxID < numMeshVertices; vtxID++)
  {
    vector<pair<double, int>> sortBuffer(numJointsInfluencingEachVertex);
    for (size_t j = 0; j < weightMatrixEntries[vtxID].size(); j++)
    {
      int frameID = weightMatrixColumnIndices[vtxID][j];
      double weight = weightMatrixEntries[vtxID][j];
      sortBuffer[j] = make_pair(weight, frameID);
    }
    sortBuffer.resize(weightMatrixEntries[vtxID].size());
    assert(sortBuffer.size() > 0);
    sort(sortBuffer.rbegin(), sortBuffer.rend()); // sort in descending order using reverse_iterators
    for(size_t i = 0; i < sortBuffer.size(); i++)
    {
      meshSkinningJoints[vtxID * numJointsInfluencingEachVertex + i] = sortBuffer[i].second;
      meshSkinningWeights[vtxID * numJointsInfluencingEachVertex + i] = sortBuffer[i].first;
    }

    // Note: When the number of joints used on this vertex is smaller than numJointsInfluencingEachVertex,
    // the remaining empty entries are initialized to zero due to vector::assign(XX, 0.0) .
  }
}

void Skinning::applySkinning(const RigidTransform4d * jointSkinTransforms, double * newMeshVertexPositions) const
{
  // Students should implement this

  // The following below is just a dummy implementation.
  /*for(int i=0; i<numMeshVertices; i++)
  {
    newMeshVertexPositions[3 * i + 0] = restMeshVertexPositions[3 * i + 0];
    newMeshVertexPositions[3 * i + 1] = restMeshVertexPositions[3 * i + 1];
    newMeshVertexPositions[3 * i + 2] = restMeshVertexPositions[3 * i + 2];
  }*/
	
	for (int i = 0; i < numMeshVertices; i++)
	{
		Vec4d pi = { 0,0,0,0 };
		
		int skinningMode = 1;
		//-----------------------1. LBS-----------------------------
		if (skinningMode == 0) {
			for (int j = 0; j < numJointsInfluencingEachVertex; j++) {
				pi += meshSkinningWeights[i*numJointsInfluencingEachVertex + j] * jointSkinTransforms[meshSkinningJoints[i * numJointsInfluencingEachVertex + j]]
					* Vec4d(restMeshVertexPositions[3 * i + 0], restMeshVertexPositions[3 * i + 1], restMeshVertexPositions[3 * i + 2], 1);
			}
		}
		//-----------------------2. Dual Quaternion---------------
		else if (skinningMode == 1) {
			Quaternion<double>q0sum(0, 0, 0, 0);
			Quaternion<double>q1sum(0, 0, 0, 0);

			for (int j = 0; j < numJointsInfluencingEachVertex; j++) {
				Mat4d jointTransform = jointSkinTransforms[meshSkinningJoints[i * numJointsInfluencingEachVertex + j]];

				double* rot = new double[9];
				Vec3d trans;
				for (int k = 0; k < 3; k++) {
					for (int d = 0; d < 3; d++) {
						rot[3 * k + d] = jointTransform[k][d];
					}
					trans[k] = jointTransform[k][3];
				}
				Quaternion<double> q0_head = Quaternion<double>::Matrix2Quaternion(rot);
				q0_head.MoveToRightHalfSphere();
				Quaternion<double> q1_head = 0.5*Quaternion<double>(0, trans[0], trans[1], trans[2])*q0_head;

				q0sum =q0sum+ meshSkinningWeights[i*numJointsInfluencingEachVertex + j]* q0_head;

				q1sum = q1sum + meshSkinningWeights[i*numJointsInfluencingEachVertex + j] * q1_head;
			}
			Quaternion<double> q0_head_curve = q0sum / q0sum.Norm();
			Quaternion<double> q1_head_curve = q1sum / q0sum.Norm() - (q1sum*q0sum / pow(q0sum.Norm(), 3)*q0sum);

			Quaternion<double> q0_head_cure_inverse(q0_head_curve.Gets(), -q0_head_curve.Getx(), -q0_head_curve.Gety(), -q0_head_curve.Getz());
			Quaternion<double> trans_ans = 2 * q1_head_curve* (q0_head_cure_inverse / q0_head_cure_inverse.Norm2());

			double rot_ans[9];
			Quaternion2Rot(q0_head_curve, rot_ans);
			Mat3d rotation(rot_ans);

			Vec3d trans(trans_ans.Getx(), trans_ans.Gety(), trans_ans.Getz());
			pi = RigidTransform4d(rotation, trans)*Vec4d(restMeshVertexPositions[3 * i + 0], restMeshVertexPositions[3 * i + 1], restMeshVertexPositions[3 * i + 2], 1);

		}
		newMeshVertexPositions[3*i] =  pi[0];
		newMeshVertexPositions[3 * i + 1] = pi[1];
		newMeshVertexPositions[3 * i + 2] = pi[2];
	}
	
}





