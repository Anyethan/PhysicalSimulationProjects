#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <float.h>
#include <fstream>
#include <iostream>
#include "motion.h"
#include "interpolator.h"
#include "types.h"
#include "transform.h"
#include "quaternion.h"


Interpolator::Interpolator()
{
  //Set default interpolation type
  m_InterpolationType = LINEAR;

  //set default angle representation to use for interpolation
  m_AngleRepresentation = EULER;
}

Interpolator::~Interpolator()
{
}

//Create interpolated motion
void Interpolator::Interpolate(Motion * pInputMotion, Motion ** pOutputMotion, int N) 
{
  //Allocate new motion
  *pOutputMotion = new Motion(pInputMotion->GetNumFrames(), pInputMotion->GetSkeleton()); 

  //Perform the interpolation
  if ((m_InterpolationType == LINEAR) && (m_AngleRepresentation == EULER))
    LinearInterpolationEuler(pInputMotion, *pOutputMotion, N);
  else if ((m_InterpolationType == LINEAR) && (m_AngleRepresentation == QUATERNION))
    LinearInterpolationQuaternion(pInputMotion, *pOutputMotion, N);
  else if ((m_InterpolationType == BEZIER) && (m_AngleRepresentation == EULER))
    BezierInterpolationEuler(pInputMotion, *pOutputMotion, N);
  else if ((m_InterpolationType == BEZIER) && (m_AngleRepresentation == QUATERNION))
    BezierInterpolationQuaternion(pInputMotion, *pOutputMotion, N);
  //extra
  else if (((m_InterpolationType == Non_uniform) && (m_AngleRepresentation == QUATERNION))) {
	  NonUniform_BezierInterpolationQuaternion(pInputMotion, *pOutputMotion);
  }
  else
  {
    printf("Error: unknown interpolation / angle representation type.\n");
    exit(1);
  }
}

void Interpolator::LinearInterpolationEuler(Motion * pInputMotion, Motion * pOutputMotion, int N)
{
  int inputLength = pInputMotion->GetNumFrames(); // frames are indexed 0, ..., inputLength-1

  int startKeyframe = 0;

  while (startKeyframe + N + 1 < inputLength)
  {
    int endKeyframe = startKeyframe + N + 1;

    Posture * startPosture = pInputMotion->GetPosture(startKeyframe);
    Posture * endPosture = pInputMotion->GetPosture(endKeyframe);

    // copy start and end keyframe
    pOutputMotion->SetPosture(startKeyframe, *startPosture);
    pOutputMotion->SetPosture(endKeyframe, *endPosture);

    // interpolate in between
    for(int frame=1; frame<=N; frame++)
    {
      Posture interpolatedPosture;
      double t = 1.0 * frame / (N+1);

      // interpolate root position
      interpolatedPosture.root_pos = startPosture->root_pos * (1-t) + endPosture->root_pos * t;

      // interpolate bone rotations
      for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++)
        interpolatedPosture.bone_rotation[bone] = startPosture->bone_rotation[bone] * (1-t) + endPosture->bone_rotation[bone] * t;
		
      pOutputMotion->SetPosture(startKeyframe + frame, interpolatedPosture);
	
    }

    startKeyframe = endKeyframe;
  }

  for(int frame=startKeyframe+1; frame<inputLength; frame++)
    pOutputMotion->SetPosture(frame, *(pInputMotion->GetPosture(frame)));

  /*std::fstream f;
  f.open("linearEuler_root.txt", std::ios::out);
  f << "#data" << std::endl;
  for (int fr = 200; fr < 501; fr++) {
	  f << fr << "      " << pOutputMotion->GetPosture(fr)->bone_rotation[0][2] << std::endl;
  }*/
}

void Interpolator::Rotation2Euler(double R[9], double angles[3])
{
  double cy = sqrt(R[0]*R[0] + R[3]*R[3]);

  if (cy > 16*DBL_EPSILON) 
  {
    angles[0] = atan2(R[7], R[8]);
    angles[1] = atan2(-R[6], cy);
    angles[2] = atan2(R[3], R[0]);
  } 
  else 
  {
    angles[0] = atan2(-R[5], R[4]);
    angles[1] = atan2(-R[6], cy);
    angles[2] = 0;
  }

  for(int i=0; i<3; i++)
    angles[i] *= 180 / M_PI;
}

void Interpolator::Euler2Rotation(double angles[3], double R[9])
{
  // students should implement this
	double R_1[4][4];
	rotationX(R_1, angles[0]);
	double R_2[4][4];
	rotationY(R_2, angles[1]);
	double R_3[4][4];
	rotationZ(R_3, angles[2]);

	double R_res[4][4], R_ans[4][4];
	matrix_mult(R_3, R_2, R_res);
	matrix_mult(R_res, R_1, R_ans);

	for (int i = 0; i < 9; i++)
	{
		R[i] = R_ans[i / 3][i % 3];
	}
}


void Interpolator::BezierInterpolationEuler(Motion * pInputMotion, Motion * pOutputMotion, int N)
{
  // students should implement this
	int inputLength = pInputMotion->GetNumFrames(); // frames are indexed 0, ..., inputLength-1

	int startKeyframe = 0;
	while (startKeyframe + N + 1 < inputLength)
	{
		int endKeyframe = startKeyframe + N + 1;

		Posture * q1Posture;
		Posture * q2Posture; 
		Posture * q3Posture;
		Posture * q4Posture;

		vector an, bn, a_n_head,a_nplus1_head, b_nplus1;
		if (startKeyframe == 0)
		{
			int q3Keyframe = endKeyframe + N + 1;
			q1Posture = pInputMotion->GetPosture(startKeyframe);
			q2Posture = pInputMotion->GetPosture(endKeyframe);
			q3Posture = pInputMotion->GetPosture(q3Keyframe);

			pOutputMotion->SetPosture(startKeyframe, *q1Posture);
			pOutputMotion->SetPosture(endKeyframe, *q2Posture);

			for (int frame = 1; frame <= N; frame++)
			{
				Posture interpolatedPosture;
				double t = 1.0 * frame / (N + 1);

				// interpolate root position
				an = lerp(q1Posture->root_pos, lerp(q3Posture->root_pos, q2Posture->root_pos, 2.0), 1.0 / 3);
				a_nplus1_head = lerp(lerp(q1Posture->root_pos, q2Posture->root_pos, 2.0), q3Posture->root_pos, 0.5);
				b_nplus1 = lerp(q2Posture->root_pos, a_nplus1_head, -1.0 / 3);
				interpolatedPosture.root_pos = DeCasteljauEuler(t, q1Posture->root_pos, an, b_nplus1, q2Posture->root_pos);

				for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++) {
					an = lerp(q1Posture->bone_rotation[bone], lerp(q3Posture->bone_rotation[bone], q2Posture->bone_rotation[bone], 2.0), 1.0 / 3);

					a_nplus1_head = lerp(lerp(q1Posture->bone_rotation[bone], q2Posture->bone_rotation[bone], 2.0), q3Posture->bone_rotation[bone], 0.5);
					b_nplus1 = lerp(q2Posture->bone_rotation[bone], a_nplus1_head, -1.0 / 3);
					
					interpolatedPosture.bone_rotation[bone] = DeCasteljauEuler(t, q1Posture->bone_rotation[bone], an, b_nplus1, q2Posture->bone_rotation[bone]);
				}
				pOutputMotion->SetPosture(startKeyframe + frame, interpolatedPosture);
			}
		}
		else if (startKeyframe + 2 * N + 2 > inputLength) {
			int nminus1_Keyframe = startKeyframe- N - 1;
			q1Posture = pInputMotion->GetPosture(nminus1_Keyframe);
			q2Posture = pInputMotion->GetPosture(startKeyframe);
			q3Posture = pInputMotion->GetPosture(endKeyframe);

			pOutputMotion->SetPosture(startKeyframe, *q2Posture);
			pOutputMotion->SetPosture(endKeyframe, *q3Posture);

			for (int frame = 1; frame <= N; frame++)
			{
				Posture interpolatedPosture;
				double t = 1.0 * frame / (N + 1);

				// interpolate root position
				a_n_head = lerp(lerp(q1Posture->root_pos, q2Posture->root_pos, 2.0), q3Posture->root_pos, 0.5);
				an = lerp(q2Posture->root_pos, a_n_head, 1.0 / 3);
				b_nplus1 = lerp(q3Posture->root_pos, lerp(q1Posture->root_pos, q2Posture->root_pos, 2.0), 1.0 / 3);
				interpolatedPosture.root_pos = DeCasteljauEuler(t, q2Posture->root_pos, an, b_nplus1, q3Posture->root_pos);

				for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++){
					a_n_head = lerp(lerp(q1Posture->bone_rotation[bone], q2Posture->bone_rotation[bone], 2.0),q3Posture->bone_rotation[bone],0.5);
					an = lerp(q2Posture->bone_rotation[bone], a_n_head, 1.0 / 3);
					b_nplus1 = lerp(q3Posture->bone_rotation[bone], lerp(q1Posture->bone_rotation[bone], q2Posture->bone_rotation[bone], 2.0), 1.0 / 3);

					interpolatedPosture.bone_rotation[bone] = DeCasteljauEuler(t, q2Posture->bone_rotation[bone], an, b_nplus1, q3Posture->bone_rotation[bone]);
				}
				pOutputMotion->SetPosture(startKeyframe + frame, interpolatedPosture);
			}
		}
		else {
			int nminus1_Keyframe = startKeyframe - N - 1;
			int nplus2_Keyframe = endKeyframe + N + 1;

			q1Posture = pInputMotion->GetPosture(nminus1_Keyframe);
			q2Posture = pInputMotion->GetPosture(startKeyframe);
			q3Posture = pInputMotion->GetPosture(endKeyframe);
			q4Posture = pInputMotion->GetPosture(nplus2_Keyframe);

			pOutputMotion->SetPosture(startKeyframe, *q2Posture);
			pOutputMotion->SetPosture(endKeyframe, *q3Posture);

			for (int frame = 1; frame <= N; frame++)
			{
				Posture interpolatedPosture;
				double t = 1.0 * frame / (N + 1);

				// interpolate root position
				a_n_head = lerp(lerp(q1Posture->root_pos, q2Posture->root_pos, 2.0), q3Posture->root_pos, 0.5);
				an = lerp(q2Posture->root_pos, a_n_head, 1.0 / 3);
				a_nplus1_head = lerp(lerp(q2Posture->root_pos, q3Posture->root_pos, 2.0), q4Posture->root_pos, 0.5);
				b_nplus1 = lerp(q3Posture->root_pos, a_nplus1_head, -1.0 / 3);
				interpolatedPosture.root_pos = DeCasteljauEuler(t, q2Posture->root_pos, an, b_nplus1, q3Posture->root_pos);

				for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++) {
					a_n_head = lerp(lerp(q1Posture->bone_rotation[bone], q2Posture->bone_rotation[bone], 2.0), q3Posture->bone_rotation[bone], 0.5);
					an = lerp(q2Posture->bone_rotation[bone], a_n_head, 1.0 / 3);
					a_nplus1_head = lerp(lerp(q2Posture->bone_rotation[bone], q3Posture->bone_rotation[bone], 2.0), q4Posture->bone_rotation[bone], 0.5);
					b_nplus1 = lerp(q3Posture->bone_rotation[bone], a_nplus1_head, -1.0 / 3);
				
					interpolatedPosture.bone_rotation[bone] = DeCasteljauEuler(t, q2Posture->bone_rotation[bone], an, b_nplus1, q3Posture->bone_rotation[bone]);
				}
				pOutputMotion->SetPosture(startKeyframe + frame, interpolatedPosture);
			}
		}	
		startKeyframe = endKeyframe;
	}

	for (int frame = startKeyframe + 1; frame < inputLength; frame++)
		pOutputMotion->SetPosture(frame, *(pInputMotion->GetPosture(frame)));

	/*std::fstream f;
	f.open("BezierEuler_root.txt", std::ios::out);
	f << "#data" << std::endl;
	for (int fr = 200; fr < 501; fr++) {
		f << fr << "      " << pOutputMotion->GetPosture(fr)->bone_rotation[0][2] << std::endl;
	}*/
}

void Interpolator::LinearInterpolationQuaternion(Motion * pInputMotion, Motion * pOutputMotion, int N)
{
  // students should implement this
	int inputLength = pInputMotion->GetNumFrames(); // frames are indexed 0, ..., inputLength-1

	int startKeyframe = 0;
	while (startKeyframe + N + 1 < inputLength)
	{
		int endKeyframe = startKeyframe + N + 1;

		Posture * startPosture = pInputMotion->GetPosture(startKeyframe);
		Posture * endPosture = pInputMotion->GetPosture(endKeyframe);

		// copy start and end keyframe
		pOutputMotion->SetPosture(startKeyframe, *startPosture);
		pOutputMotion->SetPosture(endKeyframe, *endPosture);

		// interpolate in between
		for (int frame = 1; frame <= N; frame++)
		{
			Posture interpolatedPosture;
			double t = 1.0 * frame / (N + 1);

			// interpolate root position
			interpolatedPosture.root_pos = startPosture->root_pos * (1 - t) + endPosture->root_pos * t;

			// interpolate bone rotations
			for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++) {
				Quaternion<double> q;
				double d[3];
				startPosture->bone_rotation[bone].getValue(d);
				Euler2Quaternion(d, q);
				Quaternion<double> q2;
				double d2[3];
				endPosture->bone_rotation[bone].getValue(d2);
				Euler2Quaternion(d2, q2);
				
				//shortest path
				if (q.Gets()*q2.Gets()+q.Getx()*q2.Getx() + q.Gety()*q2.Gety() + q.Getz()*q2.Getz() < 0) {
					q2.Set(q2.Gets()*-1, q2.Getx()*-1, q2.Gety()*-1, q2.Getz()*-1);
				}
				Quaternion<double> qres= Slerp(t, q, q2);
				double dres[3];
				Quaternion2Euler(qres,dres);
				interpolatedPosture.bone_rotation[bone] = dres;

			}

			pOutputMotion->SetPosture(startKeyframe + frame, interpolatedPosture);
		}

		startKeyframe = endKeyframe;
	}

	for (int frame = startKeyframe + 1; frame < inputLength; frame++)
		pOutputMotion->SetPosture(frame, *(pInputMotion->GetPosture(frame)));

	/*std::fstream f;
	f.open("SlerpQuaternion_root.txt", std::ios::out);
	f << "#data" << std::endl;
	for (int fr = 200; fr < 501; fr++) {
		f << fr << "      " << pOutputMotion->GetPosture(fr)->bone_rotation[0][2] << std::endl;
	}*/
}

void Interpolator::BezierInterpolationQuaternion(Motion * pInputMotion, Motion * pOutputMotion, int N)
{
  // students should implement this
	int inputLength = pInputMotion->GetNumFrames(); // frames are indexed 0, ..., inputLength-1

	int startKeyframe = 0;
	while (startKeyframe + N + 1 < inputLength)
	{
		int endKeyframe = startKeyframe + N + 1;

		Posture * q1Posture;
		Posture * q2Posture;
		Posture * q3Posture;
		Posture * q4Posture;


		Quaternion<double> an, bn, a_n_head, a_nplus1_head, b_nplus1;
		vector an_l, bn_l, a_n_head_l, a_nplus1_head_l, b_nplus1_l;
		if (startKeyframe == 0)
		{
			int q3Keyframe = endKeyframe + N + 1;
			q1Posture = pInputMotion->GetPosture(startKeyframe);
			q2Posture = pInputMotion->GetPosture(endKeyframe);
			q3Posture = pInputMotion->GetPosture(q3Keyframe);

			pOutputMotion->SetPosture(startKeyframe, *q1Posture);
			pOutputMotion->SetPosture(endKeyframe, *q2Posture);

			for (int frame = 1; frame <= N; frame++)
			{
				Posture interpolatedPosture;
				double t = 1.0 * frame / (N + 1);

				// interpolate root position
				an_l = lerp(q1Posture->root_pos, lerp(q3Posture->root_pos, q2Posture->root_pos, 2.0), 1.0 / 3);
				a_nplus1_head_l = lerp(lerp(q1Posture->root_pos, q2Posture->root_pos, 2.0), q3Posture->root_pos, 0.5);
				b_nplus1_l = lerp(q2Posture->root_pos, a_nplus1_head_l, -1.0 / 3);
				interpolatedPosture.root_pos = DeCasteljauEuler(t, q1Posture->root_pos, an_l, b_nplus1_l, q2Posture->root_pos);

				for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++) {
					Quaternion<double> q1;
					double d[3];
					q1Posture->bone_rotation[bone].getValue(d);
					Euler2Quaternion(d, q1);
					Quaternion<double> q2;
					double d2[3];
					q2Posture->bone_rotation[bone].getValue(d2);
					Euler2Quaternion(d2, q2);
					Quaternion<double> q3;
					double d3[3];
					q3Posture->bone_rotation[bone].getValue(d3);
					Euler2Quaternion(d3, q3);

					//shortest path
					if(q1.Gets()*q2.Gets() + q1.Getx()*q2.Getx() + q1.Gety()*q2.Gety() + q1.Getz()*q2.Getz() < 0) {
						q2.Set(q2.Gets()*-1, q2.Getx()*-1, q2.Gety()*-1, q2.Getz()*-1);
					}

					an = Slerp(1.0/3, q1, Slerp(2, q3, q2));
					a_nplus1_head = Slerp(0.5, Slerp(2.0, q1, q2), q3);

					b_nplus1 = Slerp(-1.0 / 3, q2, a_nplus1_head);
					double dres[3];
					Quaternion2Euler(DeCasteljauQuaternion(t, q1, an, b_nplus1, q2), dres);
					interpolatedPosture.bone_rotation[bone] = dres;

				}
				pOutputMotion->SetPosture(startKeyframe + frame, interpolatedPosture);
			}
		}
		else if (startKeyframe + 2 * N + 2 > inputLength) {
			int nminus1_Keyframe = startKeyframe - N - 1;
			q1Posture = pInputMotion->GetPosture(nminus1_Keyframe);
			q2Posture = pInputMotion->GetPosture(startKeyframe);
			q3Posture = pInputMotion->GetPosture(endKeyframe);

			pOutputMotion->SetPosture(startKeyframe, *q2Posture);
			pOutputMotion->SetPosture(endKeyframe, *q3Posture);

			for (int frame = 1; frame <= N; frame++)
			{
				Posture interpolatedPosture;
				double t = 1.0 * frame / (N + 1);

				// interpolate root position
				a_n_head_l = lerp(lerp(q1Posture->root_pos, q2Posture->root_pos, 2.0), q3Posture->root_pos, 0.5);
				an_l = lerp(q2Posture->root_pos, a_n_head_l, 1.0 / 3);
				b_nplus1_l = lerp(q3Posture->root_pos, lerp(q1Posture->root_pos, q2Posture->root_pos, 2.0), 1.0 / 3);
				interpolatedPosture.root_pos = DeCasteljauEuler(t, q2Posture->root_pos, an_l, b_nplus1_l, q3Posture->root_pos);

				for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++) {
					Quaternion<double> q1;  //N-2
					double d[3];
					q1Posture->bone_rotation[bone].getValue(d);
					Euler2Quaternion(d, q1);
					Quaternion<double> q2;  //N-1 current startFrame
					double d2[3];
					q2Posture->bone_rotation[bone].getValue(d2);
					Euler2Quaternion(d2, q2);
					Quaternion<double> q3;  //N
					double d3[3];
					q3Posture->bone_rotation[bone].getValue(d3);
					Euler2Quaternion(d3, q3);

					//shortest path
					if (q1.Gets()*q2.Gets() + q1.Getx()*q2.Getx() + q1.Gety()*q2.Gety() + q1.Getz()*q2.Getz() < 0) {
						q2.Set(q2.Gets()*-1, q2.Getx()*-1, q2.Gety()*-1, q2.Getz()*-1);
					}
					if (q2.Gets()*q3.Gets() + q2.Getx()*q3.Getx() + q2.Gety()*q3.Gety() + q2.Getz()*q3.Getz() < 0) {
						q3.Set(q3.Gets()*-1, q3.Getx()*-1, q3.Gety()*-1, q3.Getz()*-1);
					}

					a_n_head = Slerp(0.5, Slerp(2.0, q1, q2), q3);
					an = Slerp(1.0 / 3, q2, a_n_head);
					b_nplus1 = Slerp(-1.0 / 3, q3, Slerp(2, q1, q2));

					double dres[3];
					Quaternion2Euler(DeCasteljauQuaternion(t, q2, an, b_nplus1, q3), dres);
					interpolatedPosture.bone_rotation[bone] = dres;
				}
				pOutputMotion->SetPosture(startKeyframe + frame, interpolatedPosture);
			}
		}
		else {
			int nminus1_Keyframe = startKeyframe - N - 1;
			int nplus2_Keyframe = endKeyframe + N + 1;

			q1Posture = pInputMotion->GetPosture(nminus1_Keyframe);
			q2Posture = pInputMotion->GetPosture(startKeyframe);
			q3Posture = pInputMotion->GetPosture(endKeyframe);
			q4Posture = pInputMotion->GetPosture(nplus2_Keyframe);

			pOutputMotion->SetPosture(startKeyframe, *q2Posture);
			pOutputMotion->SetPosture(endKeyframe, *q3Posture);

			for (int frame = 1; frame <= N; frame++)
			{
				Posture interpolatedPosture;
				double t = 1.0 * frame / (N + 1);

				// interpolate root position
				a_n_head_l = lerp(lerp(q1Posture->root_pos, q2Posture->root_pos, 2.0), q3Posture->root_pos, 0.5);
				an_l = lerp(q2Posture->root_pos, a_n_head_l, 1.0 / 3);
				a_nplus1_head_l = lerp(lerp(q2Posture->root_pos, q3Posture->root_pos, 2.0), q4Posture->root_pos, 0.5);
				b_nplus1_l = lerp(q3Posture->root_pos, a_nplus1_head_l, -1.0 / 3);
				interpolatedPosture.root_pos = DeCasteljauEuler(t, q2Posture->root_pos, an_l, b_nplus1_l, q3Posture->root_pos);

				for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++) {
			
					Quaternion<double> q1;  //N-1
					double d[3];
					q1Posture->bone_rotation[bone].getValue(d);
					Euler2Quaternion(d, q1);
					Quaternion<double> q2;  //N current startFrame
					double d2[3];
					q2Posture->bone_rotation[bone].getValue(d2);
					Euler2Quaternion(d2, q2);
					Quaternion<double> q3;  //N+1
					double d3[3];
					q3Posture->bone_rotation[bone].getValue(d3);
					Euler2Quaternion(d3, q3);
					Quaternion<double> q4;  //N+2
					double d4[3];
					q4Posture->bone_rotation[bone].getValue(d4);
					Euler2Quaternion(d4, q4);

					//shortest path
					if (q1.Gets()*q2.Gets() + q1.Getx()*q2.Getx() + q1.Gety()*q2.Gety() + q1.Getz()*q2.Getz() < 0) {
						q2.Set(q2.Gets()*-1, q2.Getx()*-1, q2.Gety()*-1, q2.Getz()*-1);
					}
					if (q2.Gets()*q3.Gets() + q2.Getx()*q3.Getx() + q2.Gety()*q3.Gety() + q2.Getz()*q3.Getz() < 0) {
						q3.Set(q3.Gets()*-1, q3.Getx()*-1, q3.Gety()*-1, q3.Getz()*-1);
					}
					if (q3.Gets()*q4.Gets() + q3.Getx()*q4.Getx() + q3.Gety()*q4.Gety() + q3.Getz()*q4.Getz() < 0) {
						q4.Set(q4.Gets()*-1, q4.Getx()*-1, q4.Gety()*-1, q4.Getz()*-1);
					}

					a_n_head = Slerp(0.5, Slerp(2.0, q1, q2), q3);
					an = Slerp(1.0 / 3, q2, a_n_head);
					a_nplus1_head = Slerp(0.5, Slerp(2.0, q2, q3), q4);
					b_nplus1 = Slerp(-1.0 / 3, q3, a_nplus1_head);

					double dres[3];
					Quaternion2Euler(DeCasteljauQuaternion(t, q2, an, b_nplus1, q3), dres);
					interpolatedPosture.bone_rotation[bone] = dres;
				}
				pOutputMotion->SetPosture(startKeyframe + frame, interpolatedPosture);
			}
		}
		startKeyframe = endKeyframe;
	}

	for (int frame = startKeyframe + 1; frame < inputLength; frame++)
		pOutputMotion->SetPosture(frame, *(pInputMotion->GetPosture(frame)));

	std::fstream f;
	f.open("BezierQuaternion_root.txt", std::ios::out);
	f << "#data" << std::endl;
	for (int fr = 200; fr < 501; fr++) {
		f << fr << "      " << pOutputMotion->GetPosture(fr)->bone_rotation[0][2] << std::endl;
	}

	/*std::fstream f2;
	f2.open("inputMotion_root.txt", std::ios::out);
	f2 << "#data" << std::endl;
	for (int fr = 200; fr < 501; fr++) {
		f2 << fr << "      " << pInputMotion->GetPosture(fr)->bone_rotation[0][2] << std::endl;
	}*/
}

void Interpolator::Euler2Quaternion(double angles[3], Quaternion<double> & q) 
{
  // students should implement this
	double R[9];
	Euler2Rotation(angles,R);
	q = Quaternion<double>::Matrix2Quaternion(R);
	//q.MoveToRightHalfSphere();
}

void Interpolator::Quaternion2Euler(Quaternion<double> & q, double angles[3]) 
{
  // students should implement this
	double R[9];
	q.Quaternion<double>::Quaternion2Matrix(R);
	Rotation2Euler(R, angles);
}


vector Interpolator::lerp( vector start, vector end, double t) {
	return (end - start)*t + start ;
}

Quaternion<double> Interpolator::Slerp(double t, Quaternion<double> & qStart, Quaternion<double> & qEnd_)
{
  // students should implement this
	
	double rad = acos(qStart.Gets()*qEnd_.Gets() + qStart.Getx()*qEnd_.Getx() + qStart.Gety()*qEnd_.Gety() + qStart.Getz()*qEnd_.Getz());
	if (sin(rad) == 0) {
		return 0;
	}
	Quaternion<double> result = sin((1-t)*rad)/sin(rad) * qStart + sin(t*rad)/sin(rad)*qEnd_;
	return result;
}

Quaternion<double> Interpolator::Double(Quaternion<double> p, Quaternion<double> q)
{
  // students should implement this
  Quaternion<double> result;
  return result;
}

vector Interpolator::DeCasteljauEuler(double t, vector p0, vector p1, vector p2, vector p3)
{
  // students should implement this

	vector Q0 = lerp(p0, p1, t);
	vector Q1 = lerp(p1, p2, t);
	vector Q2 = lerp(p2, p3, t);

	vector R0 = lerp(Q0, Q1, t);
	vector R1 = lerp(Q1, Q2, t);

	vector result = lerp(R0, R1, t);
	return result;
}

Quaternion<double> Interpolator::DeCasteljauQuaternion(double t, Quaternion<double> p0, Quaternion<double> p1, Quaternion<double> p2, Quaternion<double> p3)
{
  // students should implement this
	Quaternion<double> Q0 = Slerp(t, p0, p1);
	Quaternion<double> Q1 = Slerp(t,p1, p2);
	Quaternion<double> Q2 = Slerp(t,p2, p3);

	Quaternion<double> R0 = Slerp(t, Q0, Q1);
	Quaternion<double> R1 = Slerp(t, Q1, Q2);

	
  Quaternion<double> result = Slerp(t, R0, R1);;
  return result;
}


//Extra

void Interpolator::NonUniform_BezierInterpolationQuaternion(Motion * pInputMotion, Motion * pOutputMotion)
{

	int inputLength = pInputMotion->GetNumFrames(); // frames are indexed 0, ..., inputLength-1
	int startKeyframe = 0;
	int N=0;
	int count=0;
	std::fstream fn;
	fn.open("Nlists.txt", std::ios::in);
	while (fn >> N) {
		count++;
	}
	fn.close();

	fn.open("Nlists.txt", std::ios::in);
	int* Nlists = new int[count];
	int i = 0;
	while (fn >> N) {
		Nlists[i] = N;
		i++;
	}

	N = Nlists[0];
	int n_idx = 0;
	int N_pre = 0;
	int N_next = Nlists[1];
	while (startKeyframe + N + 1 < inputLength){
			int endKeyframe = startKeyframe + N + 1;

			Posture * q1Posture;
			Posture * q2Posture;
			Posture * q3Posture;
			Posture * q4Posture;


			Quaternion<double> an, bn, a_n_head, a_nplus1_head, b_nplus1;
			vector an_l, bn_l, a_n_head_l, a_nplus1_head_l, b_nplus1_l;
			if (startKeyframe == 0)
			{
				if (n_idx == count - 1) {
					N_next = Nlists[0];
				}
				else {
					N_next = Nlists[n_idx + 1];
				}
				int q3Keyframe = endKeyframe + N_next + 1;
				q1Posture = pInputMotion->GetPosture(startKeyframe);
				q2Posture = pInputMotion->GetPosture(endKeyframe);
				q3Posture = pInputMotion->GetPosture(q3Keyframe);

				pOutputMotion->SetPosture(startKeyframe, *q1Posture);
				pOutputMotion->SetPosture(endKeyframe, *q2Posture);

				for (int frame = 1; frame <= N; frame++)
				{
					Posture interpolatedPosture;
					double t = 1.0 * frame / (N + 1);

					// interpolate root position
					an_l = lerp(q1Posture->root_pos, lerp(q3Posture->root_pos, q2Posture->root_pos, 2.0), 1.0 / 3);
					a_nplus1_head_l = lerp(lerp(q1Posture->root_pos, q2Posture->root_pos, 2.0), q3Posture->root_pos, 0.5);
					b_nplus1_l = lerp(q2Posture->root_pos, a_nplus1_head_l, -1.0 / 3);
					interpolatedPosture.root_pos = DeCasteljauEuler(t, q1Posture->root_pos, an_l, b_nplus1_l, q2Posture->root_pos);

					for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++) {
						Quaternion<double> q1;
						double d[3];
						q1Posture->bone_rotation[bone].getValue(d);
						Euler2Quaternion(d, q1);
						Quaternion<double> q2;
						double d2[3];
						q2Posture->bone_rotation[bone].getValue(d2);
						Euler2Quaternion(d2, q2);
						Quaternion<double> q3;
						double d3[3];
						q3Posture->bone_rotation[bone].getValue(d3);
						Euler2Quaternion(d3, q3);

						//shortest path
						if (q1.Gets()*q2.Gets() + q1.Getx()*q2.Getx() + q1.Gety()*q2.Gety() + q1.Getz()*q2.Getz() < 0) {
							q2.Set(q2.Gets()*-1, q2.Getx()*-1, q2.Gety()*-1, q2.Getz()*-1);
						}

						an = Slerp(1.0 / 3, q1, Slerp(2, q3, q2));
						a_nplus1_head = Slerp(0.5, Slerp(2.0, q1, q2), q3);

						b_nplus1 = Slerp(-1.0 / 3, q2, a_nplus1_head);
						double dres[3];
						Quaternion2Euler(DeCasteljauQuaternion(t, q1, an, b_nplus1, q2), dres);
						interpolatedPosture.bone_rotation[bone] = dres;

					}
					pOutputMotion->SetPosture(startKeyframe + frame, interpolatedPosture);
				}
			}
			else if (startKeyframe + 2 * N + 2 > inputLength) {
				if (startKeyframe != 0 && n_idx == 0) {
					N_pre = Nlists[count - 1];
				}
				else {
					N_pre = Nlists[n_idx - 1];
				}
				int nminus1_Keyframe = startKeyframe - N_pre - 1;
				q1Posture = pInputMotion->GetPosture(nminus1_Keyframe);
				q2Posture = pInputMotion->GetPosture(startKeyframe);
				q3Posture = pInputMotion->GetPosture(endKeyframe);

				pOutputMotion->SetPosture(startKeyframe, *q2Posture);
				pOutputMotion->SetPosture(endKeyframe, *q3Posture);

				for (int frame = 1; frame <= N; frame++)
				{
					Posture interpolatedPosture;
					double t = 1.0 * frame / (N + 1);

					// interpolate root position
					a_n_head_l = lerp(lerp(q1Posture->root_pos, q2Posture->root_pos, 2.0), q3Posture->root_pos, 0.5);
					an_l = lerp(q2Posture->root_pos, a_n_head_l, 1.0 / 3);
					b_nplus1_l = lerp(q3Posture->root_pos, lerp(q1Posture->root_pos, q2Posture->root_pos, 2.0), 1.0 / 3);
					interpolatedPosture.root_pos = DeCasteljauEuler(t, q2Posture->root_pos, an_l, b_nplus1_l, q3Posture->root_pos);

					for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++) {
						Quaternion<double> q1;  //N-2
						double d[3];
						q1Posture->bone_rotation[bone].getValue(d);
						Euler2Quaternion(d, q1);
						Quaternion<double> q2;  //N-1 current startFrame
						double d2[3];
						q2Posture->bone_rotation[bone].getValue(d2);
						Euler2Quaternion(d2, q2);
						Quaternion<double> q3;  //N
						double d3[3];
						q3Posture->bone_rotation[bone].getValue(d3);
						Euler2Quaternion(d3, q3);

						//shortest path
						if (q1.Gets()*q2.Gets() + q1.Getx()*q2.Getx() + q1.Gety()*q2.Gety() + q1.Getz()*q2.Getz() < 0) {
							q2.Set(q2.Gets()*-1, q2.Getx()*-1, q2.Gety()*-1, q2.Getz()*-1);
						}
						if (q2.Gets()*q3.Gets() + q2.Getx()*q3.Getx() + q2.Gety()*q3.Gety() + q2.Getz()*q3.Getz() < 0) {
							q3.Set(q3.Gets()*-1, q3.Getx()*-1, q3.Gety()*-1, q3.Getz()*-1);
						}

						a_n_head = Slerp(0.5, Slerp(2.0, q1, q2), q3);
						an = Slerp(1.0 / 3, q2, a_n_head);
						b_nplus1 = Slerp(-1.0 / 3, q3, Slerp(2, q1, q2));

						double dres[3];
						Quaternion2Euler(DeCasteljauQuaternion(t, q2, an, b_nplus1, q3), dres);
						interpolatedPosture.bone_rotation[bone] = dres;
					}
					pOutputMotion->SetPosture(startKeyframe + frame, interpolatedPosture);
				}
			}
			else {

				if (n_idx == count - 1) {
					N_next = Nlists[0];
				}
				else {
					N_next = Nlists[n_idx + 1];
				}
				if (startKeyframe != 0 && n_idx == 0) {
					N_pre = Nlists[count - 1];
				}
				else {
					N_pre = Nlists[n_idx - 1];
				}
				int nminus1_Keyframe = startKeyframe - N_pre - 1;
				int nplus2_Keyframe = endKeyframe + N_next + 1;

				q1Posture = pInputMotion->GetPosture(nminus1_Keyframe);
				q2Posture = pInputMotion->GetPosture(startKeyframe);
				q3Posture = pInputMotion->GetPosture(endKeyframe);
				q4Posture = pInputMotion->GetPosture(nplus2_Keyframe);

				pOutputMotion->SetPosture(startKeyframe, *q2Posture);
				pOutputMotion->SetPosture(endKeyframe, *q3Posture);

				for (int frame = 1; frame <= N; frame++)
				{
					Posture interpolatedPosture;
					double t = 1.0 * frame / (N + 1);

					// interpolate root position
					a_n_head_l = lerp(lerp(q1Posture->root_pos, q2Posture->root_pos, 2.0), q3Posture->root_pos, 0.5);
					an_l = lerp(q2Posture->root_pos, a_n_head_l, 1.0 / 3);
					a_nplus1_head_l = lerp(lerp(q2Posture->root_pos, q3Posture->root_pos, 2.0), q4Posture->root_pos, 0.5);
					b_nplus1_l = lerp(q3Posture->root_pos, a_nplus1_head_l, -1.0 / 3);
					interpolatedPosture.root_pos = DeCasteljauEuler(t, q2Posture->root_pos, an_l, b_nplus1_l, q3Posture->root_pos);

					for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++) {

						Quaternion<double> q1;  //N-1
						double d[3];
						q1Posture->bone_rotation[bone].getValue(d);
						Euler2Quaternion(d, q1);
						Quaternion<double> q2;  //N current startFrame
						double d2[3];
						q2Posture->bone_rotation[bone].getValue(d2);
						Euler2Quaternion(d2, q2);
						Quaternion<double> q3;  //N+1
						double d3[3];
						q3Posture->bone_rotation[bone].getValue(d3);
						Euler2Quaternion(d3, q3);
						Quaternion<double> q4;  //N+2
						double d4[3];
						q4Posture->bone_rotation[bone].getValue(d4);
						Euler2Quaternion(d4, q4);

						//shortest path
						if (q1.Gets()*q2.Gets() + q1.Getx()*q2.Getx() + q1.Gety()*q2.Gety() + q1.Getz()*q2.Getz() < 0) {
							q2.Set(q2.Gets()*-1, q2.Getx()*-1, q2.Gety()*-1, q2.Getz()*-1);
						}
						if (q2.Gets()*q3.Gets() + q2.Getx()*q3.Getx() + q2.Gety()*q3.Gety() + q2.Getz()*q3.Getz() < 0) {
							q3.Set(q3.Gets()*-1, q3.Getx()*-1, q3.Gety()*-1, q3.Getz()*-1);
						}
						if (q3.Gets()*q4.Gets() + q3.Getx()*q4.Getx() + q3.Gety()*q4.Gety() + q3.Getz()*q4.Getz() < 0) {
							q4.Set(q4.Gets()*-1, q4.Getx()*-1, q4.Gety()*-1, q4.Getz()*-1);
						}

						a_n_head = Slerp(0.5, Slerp(2.0, q1, q2), q3);
						an = Slerp(1.0 / 3, q2, a_n_head);
						a_nplus1_head = Slerp(0.5, Slerp(2.0, q2, q3), q4);
						b_nplus1 = Slerp(-1.0 / 3, q3, a_nplus1_head);

						double dres[3];
						Quaternion2Euler(DeCasteljauQuaternion(t, q2, an, b_nplus1, q3), dres);
						interpolatedPosture.bone_rotation[bone] = dres;
					}
					pOutputMotion->SetPosture(startKeyframe + frame, interpolatedPosture);
				}
			}
			startKeyframe = endKeyframe;
			if (n_idx + 1 == count) {
				n_idx = -1;
			}
			N = Nlists[n_idx + 1];
			n_idx++;
	}
				
	for (int frame = startKeyframe + 1; frame < inputLength; frame++)
		pOutputMotion->SetPosture(frame, *(pInputMotion->GetPosture(frame)));
	

	std::fstream f;
	f.open("nu_BezierQuaternion_root.txt", std::ios::out);
	f << "#data" << std::endl;
	for (int fr = 200; fr < 501; fr++) {
		f << fr << "      " << pOutputMotion->GetPosture(fr)->bone_rotation[0][2] << std::endl;
	}

	/*std::fstream f2;
	f2.open("inputMotion_root.txt", std::ios::out);
	f2 << "#data" << std::endl;
	for (int fr = 200; fr < 501; fr++) {
		f2 << fr << "      " << pInputMotion->GetPosture(fr)->bone_rotation[0][2] << std::endl;
	}*/
}

