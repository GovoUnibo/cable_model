#include "RKDP45.hpp"
#include <algorithm>
#include <cmath>

RKDP45::RKDP45(double aTol, double rTol)
  : absTol(aTol), relTol(rTol) {}

bool RKDP45::stepAdaptive(double &t,
                          std::vector<double> &y,
                          double &dt,
                          std::function<void(double,const std::vector<double>&,std::vector<double>&)> f)
{
  int n = y.size();
  std::vector<double> k1(n),k2(n),k3(n),k4(n),k5(n),k6(n),k7(n);
  std::vector<double> yTemp(n), yHigh(n), yLow(n);

  // Coefficienti Dormand–Prince
  const double c2=1./5, c3=3./10, c4=4./5, c5=8./9, c6=1., c7=1.;
  const double a21=1./5,
    a31=3./40,a32=9./40,
    a41=44./45,a42=-56./15,a43=32./9,
    a51=19372./6561,a52=-25360./2187,a53=64448./6561,a54=-212./729,
    a61=9017./3168,a62=-355./33,a63=46732./5247,a64=49./176,a65=-5103./18656,
    a71=35./384,a72=0,a73=500./1113,a74=125./192,a75=-2187./6784,a76=11./84;
  const double b1=35./384,b2=0,b3=500./1113,b4=125./192,b5=-2187./6784,b6=11./84,b7=0;
  const double bs1=5179./57600,bs2=0,bs3=7571./16695,
               bs4=393./640,bs5=-92097./339200,bs6=187./2100,bs7=1./40;

  // Stage 1
  f(t, y, k1);
  // Stage 2
  for(int i=0;i<n;++i) yTemp[i]=y[i]+dt*(a21*k1[i]);
  f(t + c2*dt, yTemp, k2);
  // Stage 3
  for(int i=0;i<n;++i) yTemp[i]=y[i]+dt*(a31*k1[i]+a32*k2[i]);
  f(t + c3*dt, yTemp, k3);
  // Stage 4
  for(int i=0;i<n;++i) yTemp[i]=y[i]+dt*(a41*k1[i]+a42*k2[i]+a43*k3[i]);
  f(t + c4*dt, yTemp, k4);
  // Stage 5
  for(int i=0;i<n;++i) yTemp[i]=y[i]+dt*(a51*k1[i]+a52*k2[i]+a53*k3[i]+a54*k4[i]);
  f(t + c5*dt, yTemp, k5);
  // Stage 6
  for(int i=0;i<n;++i) yTemp[i]=y[i]+dt*(a61*k1[i]+a62*k2[i]+a63*k3[i]+a64*k4[i]+a65*k5[i]);
  f(t + c6*dt, yTemp, k6);
  // Stage 7
  for(int i=0;i<n;++i) yTemp[i]=y[i]+dt*(a71*k1[i]+a72*k2[i]+a73*k3[i]+a74*k4[i]+a75*k5[i]+a76*k6[i]);
  f(t + c7*dt, yTemp, k7);

  // Calcola soluzioni high (5ᵗʰ) e low (4ᵗʰ)
  for(int i=0;i<n;++i) {
    yHigh[i] = y[i] + dt*(b1*k1[i]+b2*k2[i]+b3*k3[i]+b4*k4[i]+b5*k5[i]+b6*k6[i]);
    yLow [i] = y[i] + dt*(bs1*k1[i]+bs2*k2[i]+bs3*k3[i]+bs4*k4[i]+bs5*k5[i]+bs6*k6[i]+bs7*k7[i]);
  }

  // Errore normalizzato
  double err2 = 0;
  for(int i=0;i<n;++i){
    double sc = absTol + relTol * std::max(std::abs(yHigh[i]), std::abs(y[i]));
    err2 += std::pow((yHigh[i] - yLow[i]) / sc, 2);
  }
  double errNorm = std::sqrt(err2 / n);

  // Adattamento del passo
  const double safety=0.9, minF=0.2, maxF=5.0;
  if (errNorm <= 1.0) {
    t += dt;
    y = yHigh;
    dt *= std::clamp(safety * std::pow(errNorm, -0.2), minF, maxF);
    return true;
  } else {
    dt *= std::clamp(safety * std::pow(errNorm, -0.25), minF, maxF);
    return false;
  }
}
