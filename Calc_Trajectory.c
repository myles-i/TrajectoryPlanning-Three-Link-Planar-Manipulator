#include <stdio.h>
#include <math.h>
#define L1 5
#define L2 4
#define L3 3



// Direct inverse of a 3x3 matrix m

void mat3x3_inv(double minv[3][3], const double m[3][3])

{

  double det;

  double invdet;

  det = m[0][0] * (m[1][1] * m[2][2] - m[2][1] * m[1][2]) -

  m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) +

  m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);

  invdet = 1.0 / det;

  minv[0][0] = (m[1][1] * m[2][2] - m[2][1] * m[1][2]) * invdet;

  minv[0][1] = (m[0][2] * m[2][1] - m[0][1] * m[2][2]) * invdet;

  minv[0][2] = (m[0][1] * m[1][2] - m[0][2] * m[1][1]) * invdet;

  minv[1][0] = (m[1][2] * m[2][0] - m[1][0] * m[2][2]) * invdet;

  minv[1][1] = (m[0][0] * m[2][2] - m[0][2] * m[2][0]) * invdet;

  minv[1][2] = (m[1][0] * m[0][2] - m[0][0] * m[1][2]) * invdet;

  minv[2][0] = (m[1][0] * m[2][1] - m[2][0] * m[1][1]) * invdet;

  minv[2][1] = (m[2][0] * m[0][1] - m[0][0] * m[2][1]) * invdet;

  minv[2][2] = (m[0][0] * m[1][1] - m[1][0] * m[0][1]) * invdet;

}

/* Explicit 3x3 matrix * vector. b = A x where A is 3x3 */

void mat3x3_vec_mult(double b[3], const double A[3][3], const double x[3])

  {

  const int N = 3;

  int idx, jdx;

  for( idx = 0; idx < N; idx++ )

  {

    b[idx] = 0.0;

  for( jdx = 0; jdx < N; jdx++)

  {

    b[idx] += A[idx][jdx] * x[jdx] ;

  }

  }

}


/* Explicit vector addition */
void vec3_add(double c[3], const double a[3], const double b[3]){
  const int N = 3;
  for (int idx = 0; idx<N; idx++){
    c[idx] = a[idx] + b[idx];
  }
}

/* Explicit vector subtraction */
void vec3_subt(double c[3], const double a[3], const double b[3]){
  const int N = 3;
  for (int idx = 0; idx<N; idx++){
    c[idx] = a[idx] - b[idx];
  }
}

/* Explicit vector magnitude */
double vec3_mag(double a[3]){
  return sqrt(a[0]*a[0] + a[1]*a[1] + a[2]*a[2]);
}

void calculate_jacobian(double J[3][3], const double theta[3]){
  //precalculate some values
  double S12 =  sin(theta[0] + theta[1]);
  double S123 = sin(theta[0] + theta[1] + theta[2]);
  double C12 =  cos(theta[0] + theta[1]);
  double C123 = cos(theta[0] + theta[1] + theta[2]);


  J[0][0] = -L1*sin(theta[0]) - L2*S12 - L3*S123;
  J[0][1] = -L2*S12 - L3*S123;
  J[0][2] = -L3*S123;

  J[1][0] = L1*cos(theta[0]) + L2*C12 + L3*C123;
  J[1][1] = L2*C12 + L3*C123;
  J[1][2] = L3*C123;

  J[2][0] = 1;
  J[2][1] = 1;
  J[2][2] = 1;
}

void forward_kinematics(double location[3], const double theta[3]){
  location[0] = L1*cos(theta[0]) + L2*cos(theta[0] + theta[1]) + L3*cos(theta[0] + theta[1] + theta[2]);
  location[1] = L1*sin(theta[0]) + L2*sin(theta[0] + theta[1]) + L3*sin(theta[0] + theta[1] + theta[2]);
  location[2] = theta[0] + theta[1] + theta[2];
}


// }

  /* This function computes the inverse kinematics of the 3D manipulator
  - Input p: is the position in cartesian coordinates [x,y,alpha] where alpha
    is the manipulator orientation measured from x axis, and
  - Output theta: The joint angles that would result in the p position
    of the actuator*/ 
void inverse_kinematics_anal(double theta[3], const double p[3]){

  // Compute joint 2 location and angles
  double x2 = p[0] - L3*cos(p[2]); //x position of joint 2
  double y2 = p[1] - L3*sin(p[2]); //y position of joint 2

  //TODO: consider adding divide by zero protection here
  //TODO consider wrapping angle to between -PI and PI
  double beta = atan2(y2,x2);// angle to joint to from x axis
  // Precompute some values
  double r2_sqrd = x2*x2 + y2*y2;
  double r2 = sqrt(r2_sqrd); //distance of joint 2 from origin
  double L1_sqrd = L1*L1;
  double L2_sqrd = L2*L2;

  // Compute Theta1
  theta[0] = beta - acos( (L1_sqrd + r2_sqrd - L2_sqrd)/(2*L1*r2) );
  //TODO consider wrapping angle to between -PI and PI

  // Compute Theta2
  theta[1] = M_PI - acos( (L1_sqrd + L2_sqrd - r2_sqrd)/(2*L1*L2) );
  //TODO consider wrapping angle to between -PI and PI

  // Compute Theta3
  theta[2] = p[2] - theta[1] - theta[0];
  //TODO consider wrapping angle to between -PI and PI

}

void print_trajectory_k(const double pos[3], const double theta[3], const double theta_dot[3],const int idx, FILE *ofp){
  // //print output with descriptive text to file
  // fprintf(ofp,"Timestep %i:\n",idx);
  // fprintf(ofp, "Manipulator Position [x,y,phi]: [%f, %f, %f]\n", pos[0], pos[1], pos[2]);
  // fprintf(ofp, "Joint Angles: [%f, %f, %f]\n", theta[0], theta[1], theta[2]);
  // fprintf(ofp, "Joint Velocities: [%f, %f, %f]\n", theta_dot[0], theta_dot[1], theta_dot[2]);
  // fprintf(ofp,"\n");

  //print just values
  fprintf(stdout,"%f %f %f %f %f %f\n",theta[0], theta[1], theta[2], theta_dot[0], theta_dot[1], theta_dot[2]);
  fprintf(ofp,"[%f, %f, %f, %f, %f, %f]\n",theta[0], theta[1], theta[2], theta_dot[0], theta_dot[1], theta_dot[2]);
}

void calculate_trajectory(const int n, const double start[3], const double end[3], FILE *ofp){
  /* Declare Variables*/
  const int N = 3;
  double theta[3]; //joint angles for current position
  double theta_dot[3]; //joing angle velocities for current position 
  double pos[3];   // current cartesion/alpha position
  double delta[3]; // change in cartesian/alpha position per timestep
  double J[3][3]; //Jacobian
  double J_inv[3][3]; //Jacobian inverse

  // Initialize data  
  for (int idx = 0; idx<N; idx++){
    pos[idx] = start[idx];
    delta[idx] =  (end[idx] - start[idx])/(n-1); //rate at which x, y, and alpha increase each timestep
  }
  for (int idx=0; idx<n; idx++){

    // Compute theta angles for this position
    inverse_kinematics_anal(theta,pos);

    // Compute Jacobian and inverse
    calculate_jacobian(J, theta);
    mat3x3_inv(J_inv,J);

    //Compute theta dot from jacobian i
    mat3x3_vec_mult(theta_dot,J_inv,delta);

    //Print results for previous timestep
    print_trajectory_k(pos, theta, theta_dot,idx, ofp);

    // Compute next position
    vec3_add(pos,pos,delta);
  }

}


void main(int argc, char *argv[]){
  
  /* Check to make sure input and output files are specified*/
  if (argc != 3){
    fprintf(stderr, "Error: Must specify input and output file names as first and second arguments to call!\n");
    return;
  }

  double start[3];
  double end[3];
  int n;

  /* Grab parameters from input file*/
  FILE *ifp;
  ifp = fopen(argv[1], "r");
  if (ifp == NULL) {
    fprintf(stderr, "Error: Can't open input file!\n");
  }
  
  fscanf(ifp, "n=%i\n",&n);
  fscanf(ifp, "start=[%lf,%lf,%lf]\n", &start[0],&start[1],&start[2]);
  fscanf(ifp, "end=[%lf,%lf,%lf]", &end[0],&end[1],&end[2]);
  fclose(ifp);

  /* Open output file*/
  FILE *ofp;
  ofp = fopen(argv[2], "w");
  if (ofp == NULL) {
    fprintf(stderr, "Error: Can't open output file!\n");
  }
  
  fprintf(ofp,"Trajectory computed with the following parameters from input file name '%s':\n",argv[1]);
  fprintf(ofp, "n=%i\n",n);
  fprintf(ofp, "start=[%lf,%lf,%lf]\n", start[0],start[1],start[2]);
  fprintf(ofp, "end=[%lf,%lf,%lf]\n\n", end[0],end[1],end[2]);

  calculate_trajectory(n, start, end, ofp);
  fclose(ofp);

}


