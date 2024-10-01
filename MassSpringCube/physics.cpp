/*

  USC/Viterbi/Computer Science
  "Jello Cube" Assignment 1 starter code

*/

#include "jello.h"
#include "physics.h"

void Hook(double kElastic, double rest, double force[3], struct point a, struct point b) {
	
	double L_R = sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2) + pow(a.z - b.z, 2)) - rest;
	double len_L = sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2) + pow(a.z - b.z, 2));

	force[0] = -(a.x - b.x) / len_L * L_R * kElastic;
	force[1] = -(a.y - b.y) / len_L * L_R * kElastic;
	force[2] = -(a.z - b.z) / len_L * L_R * kElastic;

}

void Dump(double dElastic, double force[3], struct point a, struct point b, struct point v_a, struct point v_b) {

	double len_L = sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2) + pow(a.z - b.z, 2));
	double dot = ((v_a.x - v_b.x) * (a.x - b.x) + (v_a.y - v_b.y) * (a.y - b.y) + (v_a.z - v_b.z) * (a.z - b.z)) / len_L;

	force[0] = -(a.x - b.x) / len_L * dot * dElastic;
	force[1] = -(a.y - b.y) / len_L * dot * dElastic;
	force[2] = -(a.z - b.z) / len_L * dot * dElastic;

}



/* Computes acceleration to every control point of the jello cube, 
   which is in state given by 'jello'.
   Returns result in array 'a'. */
void computeAcceleration(struct world * jello, struct point a[8][8][8])
{
	/* for you to implement ... */
	int i, j, k;




	for (i = 0; i <= 7; i++)
		for (j = 0; j <= 7; j++)
			for (k = 0; k <= 7; k++)
			{
				double F_hook_s[6][3] = { 0 };
				double F_hook_shear[20][3] = { 0 };
				double F_hook_bend[6][3] = { 0 };
				double F_dump_s[6][3] = { 0 };
				double F_dump_shear[20][3] = { 0 };
				double F_dump_bend[6][3] = { 0 };
				double F_total[3] = { 0 };

				//check its position and compute hook
				//-------------------------------structure------------------------------------
				if (i + 1 < 8)Hook(jello->kElastic, 1.0 / 7, F_hook_s[0],jello->p[i][j][k], jello->p[i + 1][j][k]);
				if (i - 1 >= 0) Hook(jello->kElastic, 1.0 / 7, F_hook_s[1], jello->p[i][j][k], jello->p[i - 1][j][k]);
				if (j + 1 < 8)Hook(jello->kElastic, 1.0 / 7, F_hook_s[2], jello->p[i][j][k], jello->p[i][j + 1][k]);
				if (j - 1 >= 0)Hook(jello->kElastic, 1.0 / 7, F_hook_s[3], jello->p[i][j][k], jello->p[i][j - 1][k]);
				if (k + 1 < 8)Hook(jello->kElastic, 1.0 / 7, F_hook_s[4], jello->p[i][j][k], jello->p[i][j][k + 1]);
				if (k - 1 >= 0)Hook(jello->kElastic, 1.0 / 7, F_hook_s[5], jello->p[i][j][k], jello->p[i][j][k - 1]);

				if (i + 1 < 8)Dump(jello->dElastic, F_dump_s[0], jello->p[i][j][k], jello->p[i + 1][j][k], jello->v[i][j][k], jello->v[i + 1][j][k]);
				if (i - 1 >= 0)Dump(jello->dElastic, F_dump_s[1], jello->p[i][j][k], jello->p[i - 1][j][k], jello->v[i][j][k], jello->v[i - 1][j][k]);
				if (j + 1 < 8)Dump(jello->dElastic, F_dump_s[2], jello->p[i][j][k], jello->p[i][j + 1][k], jello->v[i][j][k], jello->v[i][j + 1][k]);
				if (j - 1 >= 0)Dump(jello->dElastic, F_dump_s[3], jello->p[i][j][k], jello->p[i][j - 1][k], jello->v[i][j][k], jello->v[i][j - 1][k]);
				if (k + 1 < 8)Dump(jello->dElastic, F_dump_s[4], jello->p[i][j][k], jello->p[i][j][k + 1], jello->v[i][j][k], jello->v[i][j][k + 1]);
				if (k - 1 >= 0)Dump(jello->dElastic, F_dump_s[5], jello->p[i][j][k], jello->p[i][j][k - 1], jello->v[i][j][k], jello->v[i][j][k - 1]);

				for (int h = 0; h < 6; h++) {
					F_total[0] += F_hook_s[h][0] + F_dump_s[h][0];
					F_total[1] += F_hook_s[h][1] + F_dump_s[h][1];
					F_total[2] += F_hook_s[h][2] + F_dump_s[h][2];

				}
				
				//--------------------------------Shear---------------------------------------
				//hook
				if (i + 1 < 8 && j + 1 < 8 )Hook(jello->kElastic, sqrt(2*pow(1.0 / 7,2)), F_hook_shear[0], jello->p[i][j][k], jello->p[i + 1][j+1][k]);
				if (i + 1 < 8 && j - 1 >= 0 ) Hook(jello->kElastic, sqrt(2 * pow(1.0 / 7, 2)), F_hook_shear[1], jello->p[i][j][k], jello->p[i + 1][j-1][k]);
				if (i + 1 < 8 && k + 1 < 8 )Hook(jello->kElastic, sqrt(2 * pow(1.0 / 7, 2)), F_hook_shear[2], jello->p[i][j][k], jello->p[i+1][j][k+1]);
				if (i + 1 < 8 && k - 1 >= 0)Hook(jello->kElastic, sqrt(2 * pow(1.0 / 7, 2)), F_hook_shear[3], jello->p[i][j][k], jello->p[i + 1][j][k - 1]);

				if (i - 1 >= 0 && j + 1 < 8)Hook(jello->kElastic, sqrt(2 * pow(1.0 / 7, 2)), F_hook_shear[4], jello->p[i][j][k], jello->p[i - 1][j + 1][k]);
				if (i - 1 >= 0 && j - 1 >= 0) Hook(jello->kElastic, sqrt(2 * pow(1.0 / 7, 2)), F_hook_shear[5], jello->p[i][j][k], jello->p[i - 1][j - 1][k]);
				if (i - 1 >= 0 && k + 1 < 8)Hook(jello->kElastic, sqrt(2 * pow(1.0 / 7, 2)), F_hook_shear[6], jello->p[i][j][k], jello->p[i - 1][j][k + 1]);
				if (i - 1 >= 0  && k - 1 >= 0)Hook(jello->kElastic, sqrt(2 * pow(1.0 / 7, 2)), F_hook_shear[7], jello->p[i][j][k], jello->p[i - 1][j][k - 1]);

				if (j + 1 < 8 && k + 1 < 8)Hook(jello->kElastic, sqrt(2 * pow(1.0 / 7, 2)), F_hook_shear[8], jello->p[i][j][k], jello->p[i ][j + 1][k+1]);
				if (j + 1 < 8 && k - 1 >= 0) Hook(jello->kElastic, sqrt(2 * pow(1.0 / 7, 2)), F_hook_shear[9], jello->p[i][j][k], jello->p[i ][j + 1][k-1]);
				if (j - 1 >= 0 && k + 1 < 8)Hook(jello->kElastic, sqrt(2 * pow(1.0 / 7, 2)), F_hook_shear[10], jello->p[i][j][k], jello->p[i ][j-1][k + 1]);
				if (j - 1 >= 0  && k - 1 >= 0)Hook(jello->kElastic, sqrt(2 * pow(1.0 / 7, 2)), F_hook_shear[11], jello->p[i][j][k], jello->p[i ][j-1][k - 1]);

				if (i - 1 >= 0 && j - 1 >= 0 && k - 1 >= 0 )Hook(jello->kElastic, sqrt(2 * pow(1.0 / 7, 2)+pow(1.0/7,2)), F_hook_shear[12], jello->p[i][j][k], jello->p[i - 1][j - 1][k-1]);
				if (i - 1 >= 0 && j - 1 >= 0 && k + 1 < 8 ) Hook(jello->kElastic, sqrt(2 * pow(1.0 / 7, 2) + pow(1.0 / 7, 2)), F_hook_shear[13], jello->p[i][j][k], jello->p[i - 1][j - 1][k+1]);
				if (i - 1 >= 0 && j + 1 < 8 && k - 1 >= 0 )Hook(jello->kElastic, sqrt(2 * pow(1.0 / 7, 2) + pow(1.0 / 7, 2)), F_hook_shear[14], jello->p[i][j][k], jello->p[i - 1][j+1][k - 1]);
				if (i - 1 >= 0 && j + 1 < 8 && k + 1 < 8 )Hook(jello->kElastic, sqrt(2 * pow(1.0 / 7, 2) + pow(1.0 / 7, 2)), F_hook_shear[15], jello->p[i][j][k], jello->p[i - 1][j+1][k + 1]);
				if (i + 1 < 8 && j + 1 < 8 && k + 1 < 8)Hook(jello->kElastic, sqrt(2 * pow(1.0 / 7, 2) + pow(1.0 / 7, 2)), F_hook_shear[16], jello->p[i][j][k], jello->p[i+1][j + 1][k + 1]);
				if (i + 1 < 8 && j + 1 < 8 && k - 1 >= 0) Hook(jello->kElastic, sqrt(2 * pow(1.0 / 7, 2) + pow(1.0 / 7, 2)), F_hook_shear[17], jello->p[i][j][k], jello->p[i+1][j + 1][k - 1]);
				if (i + 1 < 8 && j - 1 >= 0 && k + 1 < 8)Hook(jello->kElastic, sqrt(2 * pow(1.0 / 7, 2) + pow(1.0 / 7, 2)), F_hook_shear[18], jello->p[i][j][k], jello->p[i+1][j - 1][k + 1]);
				if (i + 1 < 8 && j - 1 >= 0 && k - 1 >= 0)Hook(jello->kElastic, sqrt(2 * pow(1.0 / 7, 2) + pow(1.0 / 7, 2)), F_hook_shear[19], jello->p[i][j][k], jello->p[i+1][j - 1][k - 1]);

				//dump
				if (i + 1 < 8 && j + 1 < 8)Dump(jello->dElastic, F_dump_shear[0], jello->p[i][j][k], jello->p[i + 1][j + 1][k], jello->v[i][j][k], jello->v[i + 1][j + 1][k]);
				if (i + 1 < 8 && j - 1 >= 0) Dump(jello->dElastic, F_dump_shear[1], jello->p[i][j][k], jello->p[i + 1][j - 1][k], jello->v[i][j][k], jello->v[i + 1][j-1][k]);
				if (i + 1 < 8 && k + 1 < 8)Dump(jello->dElastic, F_dump_shear[2], jello->p[i][j][k], jello->p[i + 1][j][k + 1], jello->v[i][j][k], jello->v[i + 1][j][k+1]);
				if (i + 1 < 8 && k - 1 >= 0)Dump(jello->dElastic, F_dump_shear[3], jello->p[i][j][k], jello->p[i + 1][j][k - 1], jello->v[i][j][k], jello->v[i + 1][j][k-1]);

				if (i - 1 >= 0 && j + 1 < 8)Dump(jello->dElastic, F_dump_shear[4], jello->p[i][j][k], jello->p[i - 1][j + 1][k], jello->v[i][j][k], jello->v[i - 1][j+1][k]);
				if (i - 1 >= 0 && j - 1 >= 0) Dump(jello->dElastic, F_dump_shear[5], jello->p[i][j][k], jello->p[i - 1][j - 1][k], jello->v[i][j][k], jello->v[i - 1][j-1][k]);
				if (i - 1 >= 0 && k + 1 < 8)Dump(jello->dElastic, F_dump_shear[6], jello->p[i][j][k], jello->p[i - 1][j][k + 1], jello->v[i][j][k], jello->v[i - 1][j][k+1]);
				if (i - 1 >= 0 && k - 1 >= 0)Dump(jello->dElastic, F_dump_shear[7], jello->p[i][j][k], jello->p[i - 1][j][k - 1], jello->v[i][j][k], jello->v[i - 1][j][k-1]);

				if (j + 1 < 8 && k + 1 < 8)Dump(jello->dElastic, F_dump_shear[8], jello->p[i][j][k], jello->p[i][j + 1][k + 1], jello->v[i][j][k], jello->v[i ][j+1][k+1]);
				if (j + 1 < 8 && k - 1 >= 0) Dump(jello->dElastic, F_dump_shear[9], jello->p[i][j][k], jello->p[i][j + 1][k - 1], jello->v[i][j][k], jello->v[i][j+1][k-1]);
				if (j - 1 >= 0 && k + 1 < 8)Dump(jello->dElastic, F_dump_shear[10], jello->p[i][j][k], jello->p[i][j - 1][k + 1], jello->v[i][j][k], jello->v[i][j-1][k+1]);
				if (j - 1 >= 0 && k - 1 >= 0)Dump(jello->dElastic, F_dump_shear[11], jello->p[i][j][k], jello->p[i][j - 1][k - 1], jello->v[i][j][k], jello->v[i][j-1][k-1]);
				
				if (i - 1 >= 0 && j - 1 >= 0 && k - 1 >= 0)Dump(jello->dElastic, F_dump_shear[12], jello->p[i][j][k], jello->p[i - 1][j - 1][k - 1], jello->v[i][j][k], jello->v[i - 1][j - 1][k - 1]);
				if (i - 1 >= 0 && j - 1 >= 0 && k + 1 < 8) Dump(jello->dElastic, F_dump_shear[13], jello->p[i][j][k], jello->p[i - 1][j - 1][k + 1], jello->v[i][j][k], jello->v[i - 1][j - 1][k + 1]);
				if (i - 1 >= 0 && j + 1 < 8 && k - 1 >= 0)Dump(jello->dElastic, F_dump_shear[14], jello->p[i][j][k], jello->p[i - 1][j + 1][k - 1], jello->v[i][j][k], jello->v[i - 1][j + 1][k - 1]);
				if (i - 1 >= 0 && j + 1 < 8 && k + 1 < 8)Dump(jello->dElastic, F_dump_shear[15], jello->p[i][j][k], jello->p[i - 1][j + 1][k + 1], jello->v[i][j][k], jello->v[i - 1][j + 1][k + 1]);
				if (i + 1 < 8 && j + 1 < 8 && k + 1 < 8)Dump(jello->dElastic, F_dump_shear[16], jello->p[i][j][k], jello->p[i + 1][j + 1][k + 1], jello->v[i][j][k], jello->v[i + 1][j + 1][k + 1]);
				if (i + 1 < 8 && j + 1 < 8 && k - 1 >= 0) Dump(jello->dElastic, F_dump_shear[17], jello->p[i][j][k], jello->p[i + 1][j + 1][k - 1], jello->v[i][j][k], jello->v[i + 1][j + 1][k - 1]);
				if (i + 1 < 8 && j - 1 >= 0 && k + 1 < 8)Dump(jello->dElastic, F_dump_shear[18], jello->p[i][j][k], jello->p[i + 1][j - 1][k + 1], jello->v[i][j][k], jello->v[i + 1][j - 1][k + 1]);
				if (i + 1 < 8 && j - 1 >= 0 && k - 1 >= 0)Dump(jello->dElastic, F_dump_shear[19], jello->p[i][j][k], jello->p[i + 1][j - 1][k - 1], jello->v[i][j][k], jello->v[i + 1][j - 1][k - 1]);
				

				for (int h = 0; h < 20; h++) {
					F_total[0] += F_hook_shear[h][0] + F_dump_shear[h][0];
					F_total[1] += F_hook_shear[h][1] + F_dump_shear[h][1];
					F_total[2] += F_hook_shear[h][2] + F_dump_shear[h][2];

				}
	
				
				//---------------------------------bend------------------------------
				if (i + 2 < 8)Hook(jello->kElastic, 2.0 / 7, F_hook_bend[0], jello->p[i][j][k], jello->p[i + 2][j][k]);
				if (i - 2 >= 0) Hook(jello->kElastic, 2.0 / 7, F_hook_bend[1], jello->p[i][j][k], jello->p[i - 2][j][k]);
				if (j + 2 < 8)Hook(jello->kElastic, 2.0 / 7, F_hook_bend[2], jello->p[i][j][k], jello->p[i][j + 2][k]);
				if (j - 2 >= 0)Hook(jello->kElastic, 2.0 / 7, F_hook_bend[3], jello->p[i][j][k], jello->p[i][j - 2][k]);
				if (k + 2 < 8)Hook(jello->kElastic, 2.0 / 7, F_hook_bend[4], jello->p[i][j][k], jello->p[i][j][k + 2]);
				if (k - 2 >= 0)Hook(jello->kElastic, 2.0 / 7, F_hook_bend[5], jello->p[i][j][k], jello->p[i][j][k - 2]);
				
				if (i + 2 < 8)Dump(jello->dElastic, F_dump_bend[0], jello->p[i][j][k], jello->p[i + 2][j][k], jello->v[i][j][k], jello->v[i + 2][j][k]);
				if (i - 2 >= 0)Dump(jello->dElastic, F_dump_bend[1], jello->p[i][j][k], jello->p[i - 2][j][k], jello->v[i][j][k], jello->v[i - 2][j][k]);
				if (j + 2 < 8)Dump(jello->dElastic, F_dump_bend[2], jello->p[i][j][k], jello->p[i][j + 2][k], jello->v[i][j][k], jello->v[i][j + 2][k]);
				if (j - 2 >= 0)Dump(jello->dElastic, F_dump_bend[3], jello->p[i][j][k], jello->p[i][j - 2][k], jello->v[i][j][k], jello->v[i][j- 2][k]);
				if (k + 2 < 8)Dump(jello->dElastic, F_dump_bend[4], jello->p[i][j][k], jello->p[i][j][k + 2], jello->v[i][j][k], jello->v[i][j][k + 2]);
				if (k - 2 >= 0)Dump(jello->dElastic, F_dump_bend[5], jello->p[i][j][k], jello->p[i][j][k - 2], jello->v[i][j][k], jello->v[i][j][k - 2]);

				for (int h = 0; h < 6; h++) {
					F_total[0] += F_hook_bend[h][0] + F_dump_bend[h][0];
					F_total[1] += F_hook_bend[h][1] + F_dump_bend[h][1];
					F_total[2] += F_hook_bend[h][2] + F_dump_bend[h][2];

				}

				//Collision
				double F_collision[3] = { 0 };
				collision_detect(jello, F_collision,i,j,k);
				F_total[0] += F_collision[0];
				F_total[1] += F_collision[1];				
				F_total[2] += F_collision[2];

				//Force Field
				double F_field[3] = { 0 };
				Interpolate_Force(jello, F_field, i, j, k);
				F_total[0] += F_field[0];
				F_total[1] += F_field[1];
				F_total[2] += F_field[2];

				//F=ma
				a[i][j][k].x = F_total[0]  / jello->mass;
				a[i][j][k].y = F_total[1] / jello->mass; 
				a[i][j][k].z = F_total[2] / jello->mass;

		
				//a[i][j][k].x = 0;
				//a[i][j][k].y = 0;
				//a[i][j][k].z = 0;

			}

}

void collision_detect(struct world * jello, double* F_collision,int i , int j, int k) {
	
				double F_hook[3] = { 0 };
				double F_dump[3] = { 0 };
				//------------------Extra---------------------
				//inclined surface
				if (jello->incPlanePresent == 1) {
					boolean inside = false;
					if (jello->d > 0) {
						 inside = jello->p[i][j][k].x*jello->a + jello->p[i][j][k].y*jello->b + jello->p[i][j][k].z*jello->c + jello->d <= 0;
					}
					else {
						 inside = jello->p[i][j][k].x*jello->a + jello->p[i][j][k].y*jello->b + jello->p[i][j][k].z*jello->c + jello->d >= 0;
					}
					if (inside) {
						double length_normal = sqrt(pow(jello->a, 2) + pow(jello->b, 2) + pow(jello->c, 2));
						double dis =fabs( (jello->p[i][j][k].x*jello->a + jello->p[i][j][k].y*jello->b + jello->p[i][j][k].z*jello->c + jello->d)/ length_normal);

						F_hook[0] = -jello->kCollision* dis * (-jello->a)/length_normal;
						F_hook[1] = -jello->kCollision* dis * (-jello->b)/length_normal;
						F_hook[2] = -jello->kCollision* dis * (-jello->c)/length_normal;

						double dot = ((jello->v[i][j][k].x) * (-jello->a) + (jello->v[i][j][k].y) * (-jello->b) + (jello->v[i][j][k].z * (-jello->c))) / length_normal;
						F_dump[0] = -jello->dCollision* dot*(-jello->a) / length_normal;
						F_dump[1] = -jello->dCollision* dot*(-jello->b )/ length_normal;
						F_dump[2] = -jello->dCollision* dot*(-jello->c) / length_normal; 

						F_collision[0] = F_hook[0] + F_dump[0];
						F_collision[1] = F_hook[1] + F_dump[1];
						F_collision[2] = F_hook[2] + F_dump[2];
					}
				}

				if (jello->p[i][j][k].x > 2) {
					double d = jello->p[i][j][k].x - 2;
					F_hook[0] = -jello->kCollision* d;
					F_dump[0] = -jello->dCollision* jello->v[i][j][k].x;
					//extra force

					//total
					F_collision[0] = F_hook[0] + F_dump[0];
				}
				else if (jello->p[i][j][k].x < -2) {
					double d = jello->p[i][j][k].x + 2;
					F_hook[0] = -jello->kCollision* d;
					F_dump[0] = -jello->dCollision* jello->v[i][j][k].x;
					//extra force

					//total
					F_collision[0] = F_hook[0] + F_dump[0];
				}
				if (jello->p[i][j][k].y > 2) {
					double d = jello->p[i][j][k].y - 2;
					F_hook[1] = -jello->kCollision* d;
					F_dump[1] = -jello->dCollision* jello->v[i][j][k].y;
					//extra force

					//total
					F_collision[1] = F_hook[1] + F_dump[1];
				}
				else if (jello->p[i][j][k].y < -2) {
					double d = jello->p[i][j][k].y + 2;
					F_hook[1] = -jello->kCollision* d;
					F_dump[1] = -jello->dCollision* jello->v[i][j][k].y;
					//extra force

					//total
					F_collision[1] = F_hook[1] + F_dump[1];
				}if (jello->p[i][j][k].z > 2) {
					double d = jello->p[i][j][k].z - 2;
					F_hook[2] = -jello->kCollision* d;
					F_dump[2] = -jello->dCollision* jello->v[i][j][k].z;
					//extra force

					//total
					F_collision[2] = F_hook[2] + F_dump[2];
				}
				else if (jello->p[i][j][k].z < -2) {
					double d = jello->p[i][j][k].z + 2;
					F_hook[2] = -jello->kCollision* d;
					F_dump[2] = -jello->dCollision* jello->v[i][j][k].z;
					//extra force

					//total
					F_collision[2] = F_hook[2] + F_dump[2];
				}

				

}

void Interpolate_Force (struct world * jello, double* F_collision, int i, int j, int k) {
	double step = 4.0 / (jello->resolution-1); //
	int x_nearest = floor((jello->p[i][j][k].x+2 )/ step);
	int y_nearest = floor((jello->p[i][j][k].y+2) / step);
	int z_nearest = floor(( jello->p[i][j][k].z +2)/ step);
	if (x_nearest >= 29 || y_nearest >= 29 || z_nearest >= 29|| x_nearest <0  || y_nearest <0 || z_nearest <0) {
		F_collision = { 0 };
	}
	else {
		double alpha = (jello->p[i][j][k].x + 2 - x_nearest * step) / step;
		double beta = (jello->p[i][j][k].y + 2 - y_nearest * step) / step;
		double garma = (jello->p[i][j][k].z + 2 - z_nearest * step) / step;

		struct point A_000 = jello->forceField[x_nearest * jello->resolution * jello->resolution + y_nearest * jello->resolution + z_nearest];
		struct point A_001 = jello->forceField[x_nearest * jello->resolution * jello->resolution + y_nearest * jello->resolution + z_nearest + 1];
		struct point A_010 = jello->forceField[x_nearest * jello->resolution * jello->resolution + (y_nearest + 1) * jello->resolution + z_nearest];
		struct point A_011 = jello->forceField[x_nearest * jello->resolution * jello->resolution + (y_nearest + 1) * jello->resolution + z_nearest + 1];
		struct point A_100 = jello->forceField[(1 + x_nearest) * jello->resolution * jello->resolution + y_nearest * jello->resolution + z_nearest];
		struct point A_101 = jello->forceField[(1 + x_nearest) * jello->resolution * jello->resolution + y_nearest * jello->resolution + z_nearest + 1];
		struct point A_110 = jello->forceField[(1 + x_nearest) * jello->resolution * jello->resolution + (1 + y_nearest) * jello->resolution + z_nearest];
		struct point A_111 = jello->forceField[(1 + x_nearest) * jello->resolution * jello->resolution + (1 + y_nearest) * jello->resolution + z_nearest + 1];

		F_collision[0] = (1 - alpha)*(1 - beta)*(1 - garma)*A_000.x + (1 - alpha)*(1 - beta)*(garma)*A_001.x + (1 - alpha)*(beta)*(1 - garma)*A_010.x
			+ (1 - alpha)*(beta)*(garma)*A_011.x
			+ (alpha)*(1 - beta)*(1 - garma)*A_100.x + (alpha)*(1 - beta)*(garma)*A_101.x + (alpha)*(beta)*(1 - garma)*A_110.x + (alpha)*(beta)*(garma)*A_111.x;
		F_collision[1] = (1 - alpha)*(1 - beta)*(1 - garma)*A_000.y + (1 - alpha)*(1 - beta)*(garma)*A_001.y + (1 - alpha)*(beta)*(1 - garma)*A_010.y
			+ (1 - alpha)*(beta)*(garma)*A_011.y
			+ (alpha)*(1 - beta)*(1 - garma)*A_100.y + (alpha)*(1 - beta)*(garma)*A_101.y + (alpha)*(beta)*(1 - garma)*A_110.y + (alpha)*(beta)*(garma)*A_111.y;
		F_collision[2] = (1 - alpha)*(1 - beta)*(1 - garma)*A_000.z + (1 - alpha)*(1 - beta)*(garma)*A_001.z + (1 - alpha)*(beta)*(1 - garma)*A_010.z
			+ (1 - alpha)*(beta)*(garma)*A_011.z
			+ (alpha)*(1 - beta)*(1 - garma)*A_100.z + (alpha)*(1 - beta)*(garma)*A_101.z + (alpha)*(beta)*(1 - garma)*A_110.z + (alpha)*(beta)*(garma)*A_111.z;
	}
}

//-------------------------------Extra-------------------
double screen_z;
//When a point is clicked
void ProcessClick(struct world *jello, int* selected_p, int x, int y) {
	double pos[3] = { 0 };
	GLint    viewport[4];
	GLdouble modelview[16];
	GLdouble projection[16];
	//GLfloat  winX, winY, winZ;
	GLdouble posX, posY, posZ;
	GLdouble z;
	
	glPushMatrix();

	glGetIntegerv(GL_VIEWPORT, viewport); // returns four values: the x and y, window coordinates of the viewport, followed by its width and height.
	glGetDoublev(GL_MODELVIEW_MATRIX, modelview); //returns sixteen values: the modelview matrix on the top of the modelview matrix stack. Initially this matrix is the identity matrix.
												//here we have modelview mode, and its matrix has at least 32 depth.
	glGetDoublev(GL_PROJECTION_MATRIX, projection); //returns sixteen values : the projection matrix on the top of the projection matrix stack

	glPopMatrix();
	//If it's clicking, compute current frame buffer's pixel's depth as z.
	glReadPixels(x, viewport[3] - y, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &z);//read the depth of the frame buffer pixel
	if (z == 1.0) {
		selected_p[0] = 10000;
		return;
	}
	gluUnProject(x, viewport[3] - y, z, modelview, projection, viewport, &posX, &posY, &posZ);

	pos[0] = posX;
	pos[1] = posY;
	pos[2] = posZ;
	screen_z = z; //store this z

	//find the nearst grid point
	int i, j, k;
	double dis=10000;
	
	for (i = 0; i <= 7; i++)
		for (j = 0; j <= 7; j++)
			for (k = 0; k <= 7; k ++)
			{
				if (i*j*k == 0 || i == 7 || j == 7 || k == 7) {
					GLdouble winx, winy, winz;
					gluProject(jello->p[i][j][k].x, jello->p[i][j][k].y, jello->p[i][j][k].z, modelview, projection, viewport, &winx, &winy, &winz);
					double temp_dis = sqrt(pow( winx- x, 2) + pow(winy -(viewport[3]- y), 2) );//choose the nearst one on the plane
					if (temp_dis < dis) {
						dis = temp_dis;
						selected_p[0] = i;
						selected_p[1] = j;
						selected_p[2] = k;
						
							int i, j, k;
							for (i = 0; i <= 7; i++)
								for (j = 0; j <= 7; j++)
									for (k = 0; k <= 7; k++)
									{
										jello->v[i][j][k].x = 0;
										jello->v[i][j][k].y = 0;
										jello->v[i][j][k].z = 0;
									}
						
					}
					
					
				}
				
			}

}

//Sucesfully select the point and process drag:
void ProcessDrag(struct world *jello, int* selected_p, int x, int y) {
	double pos[3] = { 0 };
	
	GLint    viewport[4];
	GLdouble modelview[16];
	GLdouble projection[16];
	//GLfloat  winX, winY, winZ;
	GLdouble posX, posY, posZ;
	GLdouble z;

	glPushMatrix();

	//glRotatef(rotate_x, 1.0, 0.0, 0.0);
	//glRotatef(rotate_y, 0.0, 1.0, 0.0);
	//glRotatef(rotate_z, 0.0, 0.0, 1.0);
	glGetIntegerv(GL_VIEWPORT, viewport); // returns four values: the x and y, window coordinates of the viewport, followed by its width and height.
	glGetDoublev(GL_MODELVIEW_MATRIX, modelview); //returns sixteen values: the modelview matrix on the top of the modelview matrix stack. Initially this matrix is the identity matrix.
												//here we have modelview mode, and its matrix has at least 32 depth.
	glGetDoublev(GL_PROJECTION_MATRIX, projection); //returns sixteen values : the projection matrix on the top of the projection matrix stack

	glPopMatrix();
	//if we are draging, just use the z given in parameters (we will convey the depth of point when clicking). 
	
	gluUnProject(x, viewport[3]- y, screen_z, modelview, projection, viewport, &posX, &posY, &posZ); //use the z same as previous vertex screen depth
	
	jello->p[selected_p[0]][selected_p[1]][selected_p[2]].x = posX;
	jello->p[selected_p[0]][selected_p[1]][selected_p[2]].y = posY;
	jello->p[selected_p[0]][selected_p[1]][selected_p[2]].z = posZ;

	 
}


/* performs one step of Euler Integration */
/* as a result, updates the jello structure */
void Euler(struct world * jello)
{
  int i,j,k;
  point a[8][8][8];

  computeAcceleration(jello, a);
  
  for (i=0; i<=7; i++)
    for (j=0; j<=7; j++)
      for (k=0; k<=7; k++)
      {
        jello->p[i][j][k].x += jello->dt * jello->v[i][j][k].x;
        jello->p[i][j][k].y += jello->dt * jello->v[i][j][k].y;
        jello->p[i][j][k].z += jello->dt * jello->v[i][j][k].z;
        jello->v[i][j][k].x += jello->dt * a[i][j][k].x;
        jello->v[i][j][k].y += jello->dt * a[i][j][k].y;
        jello->v[i][j][k].z += jello->dt * a[i][j][k].z;

      }
}

/* performs one step of RK4 Integration */
/* as a result, updates the jello structure */
void RK4(struct world * jello)
{
  point F1p[8][8][8], F1v[8][8][8], 
        F2p[8][8][8], F2v[8][8][8],
        F3p[8][8][8], F3v[8][8][8],
        F4p[8][8][8], F4v[8][8][8];

  point a[8][8][8];


  struct world buffer;

  int i,j,k;

  buffer = *jello; // make a copy of jello

  computeAcceleration(jello, a);

  for (i=0; i<=7; i++)
    for (j=0; j<=7; j++)
      for (k=0; k<=7; k++)
      {
         pMULTIPLY(jello->v[i][j][k],jello->dt,F1p[i][j][k]);
         pMULTIPLY(a[i][j][k],jello->dt,F1v[i][j][k]);
         pMULTIPLY(F1p[i][j][k],0.5,buffer.p[i][j][k]);
         pMULTIPLY(F1v[i][j][k],0.5,buffer.v[i][j][k]);
         pSUM(jello->p[i][j][k],buffer.p[i][j][k],buffer.p[i][j][k]);
         pSUM(jello->v[i][j][k],buffer.v[i][j][k],buffer.v[i][j][k]);
      }

  computeAcceleration(&buffer, a);

  for (i=0; i<=7; i++)
    for (j=0; j<=7; j++)
      for (k=0; k<=7; k++)
      {
         // F2p = dt * buffer.v;
         pMULTIPLY(buffer.v[i][j][k],jello->dt,F2p[i][j][k]);
         // F2v = dt * a(buffer.p,buffer.v);     
         pMULTIPLY(a[i][j][k],jello->dt,F2v[i][j][k]);
         pMULTIPLY(F2p[i][j][k],0.5,buffer.p[i][j][k]);
         pMULTIPLY(F2v[i][j][k],0.5,buffer.v[i][j][k]);
         pSUM(jello->p[i][j][k],buffer.p[i][j][k],buffer.p[i][j][k]);
         pSUM(jello->v[i][j][k],buffer.v[i][j][k],buffer.v[i][j][k]);
      }

  computeAcceleration(&buffer, a);

  for (i=0; i<=7; i++)
    for (j=0; j<=7; j++)
      for (k=0; k<=7; k++)
      {
         // F3p = dt * buffer.v;
         pMULTIPLY(buffer.v[i][j][k],jello->dt,F3p[i][j][k]);
         // F3v = dt * a(buffer.p,buffer.v);     
         pMULTIPLY(a[i][j][k],jello->dt,F3v[i][j][k]);
         pMULTIPLY(F3p[i][j][k],1.0,buffer.p[i][j][k]);
         pMULTIPLY(F3v[i][j][k],1.0,buffer.v[i][j][k]);
         pSUM(jello->p[i][j][k],buffer.p[i][j][k],buffer.p[i][j][k]);
         pSUM(jello->v[i][j][k],buffer.v[i][j][k],buffer.v[i][j][k]);
      }
         
  computeAcceleration(&buffer, a);


  for (i=0; i<=7; i++)
    for (j=0; j<=7; j++)
      for (k=0; k<=7; k++)
      {
         // F3p = dt * buffer.v;
         pMULTIPLY(buffer.v[i][j][k],jello->dt,F4p[i][j][k]);
         // F3v = dt * a(buffer.p,buffer.v);     
         pMULTIPLY(a[i][j][k],jello->dt,F4v[i][j][k]);

         pMULTIPLY(F2p[i][j][k],2,buffer.p[i][j][k]);
         pMULTIPLY(F3p[i][j][k],2,buffer.v[i][j][k]);
         pSUM(buffer.p[i][j][k],buffer.v[i][j][k],buffer.p[i][j][k]);
         pSUM(buffer.p[i][j][k],F1p[i][j][k],buffer.p[i][j][k]);
         pSUM(buffer.p[i][j][k],F4p[i][j][k],buffer.p[i][j][k]);
         pMULTIPLY(buffer.p[i][j][k],1.0 / 6,buffer.p[i][j][k]);
         pSUM(buffer.p[i][j][k],jello->p[i][j][k],jello->p[i][j][k]);

         pMULTIPLY(F2v[i][j][k],2,buffer.p[i][j][k]);
         pMULTIPLY(F3v[i][j][k],2,buffer.v[i][j][k]);
         pSUM(buffer.p[i][j][k],buffer.v[i][j][k],buffer.p[i][j][k]);
         pSUM(buffer.p[i][j][k],F1v[i][j][k],buffer.p[i][j][k]);
         pSUM(buffer.p[i][j][k],F4v[i][j][k],buffer.p[i][j][k]);
         pMULTIPLY(buffer.p[i][j][k],1.0 / 6,buffer.p[i][j][k]);
         pSUM(buffer.p[i][j][k],jello->v[i][j][k],jello->v[i][j][k]);
      }

  return;  
}
