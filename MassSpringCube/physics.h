/*

  USC/Viterbi/Computer Science
  "Jello Cube" Assignment 1 starter code

*/

#ifndef _PHYSICS_H_
#define _PHYSICS_H_

void computeAcceleration(struct world * jello, struct point a[8][8][8]);
void	collision_detect(struct world * jello, double* F_collision, int i, int j, int k);
void Interpolate_Force(struct world * jello, double* F_collision, int i, int j, int k);
void ProcessDrag(struct world *jello, int* vec, int x, int y);
void ProcessClick(struct world *jello, int* vec, int x, int y);

// perform one step of Euler and Runge-Kutta-4th-order integrators
// updates the jello structure accordingly
void Euler(struct world * jello);
void RK4(struct world * jello);

#endif

