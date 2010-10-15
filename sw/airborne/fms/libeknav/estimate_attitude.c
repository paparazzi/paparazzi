#include "estimate_attitude.h"

struct DoubleMat44 square_skaled(struct DoubleMat44 A){
  double _1_max = 1/INFTY_NORM16(A);
  struct DoubleMat44 A2;
  int row,col;
  for(row=0; row<4; row++){
    for(col=0; col<4; col++){
      M4(A2, row, col) = M4(A, row,0)*M4(A,0,col) + M4(A, row,1)*M4(A,1,col) + M4(A, row,2)*M4(A,2,col) + M4(A, row,3)*M4(A,3,col);
      M4(A2, row, col) *= _1_max;   // pays attention that the values don't grow too far
    }
  }
  return A2;
}

/* the following solver uses the Power Iteration
 * 
 * It is rather simple:
 * 1. You choose a starting vektor x_0 (shouldn't be zero)
 * 2. apply it on the Matrix
 *               x_(k+1) = A * x_k
 * 3. Repeat this very often.
 * 
 * The vector converges to the dominat eigenvector, which belongs to the eigenvalue with the biggest absolute value.
 * 
 * But there is a problem:
 * With every step, the norm of vector grows. Therefore it's necessary to scale it with every step.
 * 
 * Important warnings:
 * I.   This function does not converge if the Matrix is singular
 * II.  Pay attention to the loop-condition! It does not end if it's close to the eigenvector!
 *      It ends if the steps are getting too close to each other.
 * 
 */ 
DoubleVect4 dominant_Eigenvector(struct DoubleMat44 A, unsigned int maximum_iterations, double precision){
  unsigned int  k;
  DoubleVect4 x_k,
              x_kp1;
  double delta = 1,
         scale;
  
  FLOAT_QUAT_ZERO(x_k);
  
  for(k=0; (k<maximum_iterations) && (delta>precision); k++){
    
    // Next step
    DOUBLE_MAT_VMULT4(x_kp1, A, x_k);
    
    // Scale the vector
    scale = 1/INFTY_NORM4(x_kp1);
    QUAT_SMUL(x_kp1, x_kp1, scale);
    
    // Calculate the difference between to steps for the loop condition. Store temporarily in x_k
    QUAT_DIFF(x_k, x_k, x_kp1);
    delta = INFTY_NORM4(x_k);
    
    // Update the next step
    x_k = x_kp1;
    
  }
  return x_k;
}

/*    This function estimates a quaternion from a set of observations
 * 
 * B is the "attitude profile matrix". Use the other functions to fill it with the attitude observations.
 * 
 * The function solves Wahba's problem using Paul Davenport's solution.
 * Unfortunatly Davenport unpublished his solution, but you can still find descriptions of it in the web
 * ( e.g: home.comcast.net/~mdshuster2/PUB_2006c_J_GenWahba_AAS.pdf )
 * 
 */
struct DoubleQuat estimated_attitude(struct DoubleMat33 B, unsigned int maximum_iterations, double precision){
  double      traceB,
              z1, z2, z3;
  struct DoubleMat44 K;
  struct DoubleQuat  q_guessed;
  
  // pre-computation of some values
  traceB  = RMAT_TRACE(B);
  z1      = M3(B,1,2) - M3(B, 2,1);
  z2      = M3(B,2,0) - M3(B, 0,2);
  z3      = M3(B,0,1) - M3(B, 1,0);
  
  /** The "K"-Matrix. See the references for it **/
  /* Fill the upper triangle matrix */
  M4(K,0,0) = traceB; M4(K,0,1) = z1;                 M4(K,0,2) = z2;                   M4(K,0,3) = z3;
                      M4(K,1,1) = 2*M3(B,0,0)-traceB; M4(K,1,2) = M3(B,0,1)+M3(B,1,0);  M4(K,1,3) = M3(B,0,2)+M3(B,2,0); 
                                                      M4(K,2,2) = 2*M3(B,1,1)-traceB;   M4(K,2,3) = M3(B,1,2)+M3(B,2,1);
                                                                                        M4(K,3,3) = 2*M3(B,2,2)-traceB;
  /* Copy to  lower triangle matrix */
  M4(K,1,0) = M4(K,0,1);
  M4(K,2,0) = M4(K,0,2);  M4(K,2,1) = M4(K,1,2);
  M4(K,3,0) = M4(K,0,3);  M4(K,3,1) = M4(K,1,3);  M4(K,3,2) = M4(K,2,3);
  
  /* compute the estimated quaternion */
  q_guessed = dominant_Eigenvector(square_skaled(K), maximum_iterations, precision);    // K² is tricky. I'm mean, I know
  
  /* Final scaling, because the eigenvector hat not the length = 1 */
  QUAT_SMUL(q_guessed, q_guessed, 1/NORM_VECT4(q_guessed));
  return q_guessed;
}

/*  Adds an orientation observation to the "attitude profile matrix" B
 *  
 *  This is done with
 *        B += a * W * V'
 *  where
 *      a     is the weight of the current measurment (how good it is, how often...)
 *      W     is the observed vector
 *      V'    is the (transposed) reference direction, that belongs to W.
 * 
 *  See Davenport's solution for Wabha's problem for more information
 */
void add_orientation_measurement(struct DoubleMat33* B, struct Orientation_Measurement a){
  M3(*B,0,0) += a.weight_of_the_measurement * a.measured_direction.x * a.reference_direction.x;
  M3(*B,0,1) += a.weight_of_the_measurement * a.measured_direction.x * a.reference_direction.y;
  M3(*B,0,2) += a.weight_of_the_measurement * a.measured_direction.x * a.reference_direction.z;
  M3(*B,1,0) += a.weight_of_the_measurement * a.measured_direction.y * a.reference_direction.x;
  M3(*B,1,1) += a.weight_of_the_measurement * a.measured_direction.y * a.reference_direction.y;
  M3(*B,1,2) += a.weight_of_the_measurement * a.measured_direction.y * a.reference_direction.z;
  M3(*B,2,0) += a.weight_of_the_measurement * a.measured_direction.z * a.reference_direction.x;
  M3(*B,2,1) += a.weight_of_the_measurement * a.measured_direction.z * a.reference_direction.y;
  M3(*B,2,2) += a.weight_of_the_measurement * a.measured_direction.z * a.reference_direction.z;
}

/*  Generates a "faked" orientation measurement out of two true observations
 *  
 *  This is necessary because othwerwise the attitude profile matrix has only two degrees of freedom.
 *  
 *  The third reference direction is the cross product of the other two, same for the measurement.
 */
struct Orientation_Measurement fake_orientation_measurement(struct Orientation_Measurement a, struct Orientation_Measurement b){
  struct Orientation_Measurement fake;
  DOUBLE_VECT3_CROSS_PRODUCT(fake.reference_direction, a.reference_direction, b.reference_direction);
  DOUBLE_VECT3_CROSS_PRODUCT(fake.measured_direction,  a.measured_direction,  b.measured_direction );
  fake.weight_of_the_measurement = a.weight_of_the_measurement * b.weight_of_the_measurement;
  return fake;
}

/*  Add two true orientation measurements to the "attitude profile matrix" and a faked measurement */
void add_set_of_three_measurements(struct DoubleMat33* B, struct Orientation_Measurement a, struct Orientation_Measurement b){
  struct Orientation_Measurement fake = fake_orientation_measurement(a, b);
  add_orientation_measurement(B, a);
  add_orientation_measurement(B, b);
  add_orientation_measurement(B, fake);
}
