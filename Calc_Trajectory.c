# include <stdio.h>


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


void main(int argc, char *argv[]){
  
  /* Check to make sure input and output files are specified*/
  if (argc != 3){
    fprintf(stderr, "Error: Must specify input and output file names as first and second arguements to call!\n");
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

  fprintf(stdout,"Read in the following trajectory parameters from input.txt:\n");
  fprintf(stdout, "n=%i\n",n);
  fprintf(stdout, "start=[%lf,%lf,%lf]\n", start[0],start[1],start[2]);
  fprintf(stdout, "end=[%lf,%lf,%lf]\n\n", end[0],end[1],end[2]);

  /* Open output file*/
  FILE *ofp;
  ofp = fopen(argv[2], "w");
  if (ofp == NULL) {
    fprintf(stderr, "Error: Can't open output file!\n");
  }
  fprintf(ofp,"Succesfully writing to output file\n");
  fclose(ofp);

}


