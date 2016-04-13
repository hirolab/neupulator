#include <stlport.h>
#include <Eigen30.h>
#include <Eigen/Dense>
#include <Servo.h>
#include <Arduino.h>

using namespace Eigen;

void pVector(const char *str, Eigen::VectorXf v) {

  Serial.println(str);
  for(int k = 0; k < v.size(); k++) {
    Serial.print(v(k));
    Serial.print('\t');
  }
  Serial.println();
}

void pMatrix(const char *str, Eigen::MatrixXf m) {

  Serial.println(str);
  for(int i = 0; i < m.outerSize(); i++) {
    for(int j = 0; j < m.innerSize(); j++) {
      Serial.print(m(i, j));
      Serial.print('\t');
    }
    Serial.println();
  }
  Serial.println();
}

Eigen::Vector4f fk(Eigen::Vector4f q) {

  Eigen::Vector4f s;
  Matrix4f T, A;
  int dof = q.size();

  // DH table
  float angle[] = {q(0), q(1)+PI/2, q(2)-PI/2, q(3)};
  float offset[] = {.07, 0, 0, 0};
  float twist[] = {PI/2, 0, -PI/2, 0};
  float length[] = {.02, .145, .185, .075};

  // Initialize transformation matrix
  T << 1, 0, 0, 0,
       0, 1, 0, 0,
       0, 0, 1, 0,
       0, 0, 0, 1;

  // Apply DH transformations
  for(int k = 0; k < dof; k++) {

    float cz = cos(angle[k]), 
          sz = sin(angle[k]), 
          cx = cos(twist[k]), 
          sx = sin(twist[k]);

    A << cz,  -sz*cx,   sz*sx,  length[k]*cz,
         sz,  cz*cx,    -cz*sx, length[k]*sz,
         0,   sx,       cx,     offset[k],
         0,   0,        0,      1;

    T = T*A;
  }

  // Extract coordinates
  s << T(0, 3),
       T(1, 3),
       T(2, 3),
       -atan(T(0, 1)/T(0, 0));

  return(s);
}

Eigen::MatrixXf jacobian(Eigen::VectorXf q) {

  int dof = q.size();
  MatrixXf jac(dof, dof);
  VectorXf qt(q), s1(dof), s2(dof);
  const float EPS = 1e-3;
  
  for(int k = 0; k < dof; k++) {
    
    qt(k) += EPS;
    s1 = fk(qt);
    qt(k) -= 2*EPS;
    s2 = fk(qt);
    qt(k) += EPS;
    
    jac.col(k) = (s1 - s2)/(2*EPS);
  }

  return(jac);
}

float ik(float *t_, size_t dof, float *q_) {
// Apply IK for the fk function
//  t: pointer to target pose
//  q: pointer to joint solution
//  dof: degrees of freedom
// Returns the norm error

  const int MAX_ITER = 30;
  const float MAX_NORM = 4e-2;

  VectorXf t = Map<VectorXf> (t_, dof);
  VectorXf q = Map<VectorXf> (q_, dof);
  MatrixXf jac(dof, dof);
  VectorXf s = fk(q), e(dof), dq(dof);

  for(int k = 0; k < MAX_ITER; k++) {

    e = t - s;          // error
    if(e.norm() < MAX_NORM) break;

    jac = jacobian(q);  // jacobian of fk(q)
    dq = jac.householderQr().solve(e); //solve linear system

    // saturate maximum angle increment
    float norm = dq.norm();
    float factor = norm > 3 ? 1/(norm*norm) : 1;

    q += dq*factor;
    s = fk(q);        // ee coordinates
  }

  Map<VectorXf>(q_, dof) = q;   // overwrite _q
  return(e.norm());
}

