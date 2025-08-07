#pragma once
#include <vector>
#include <functional>

/// Dormand–Prince RK45 a passo adattivo
class RKDP45
{
public:
  RKDP45(double absTol, double relTol);

  /**
   * Avanza y(t) → y(t+dt) adattando dt in base all’errore.
   * @param t    tempo corrente (in/out)
   * @param y    vettore stato (in/out)
   * @param dt   passo proposto (in/out)
   * @param f    derivata f(t,y,Ydot)
   * @return     true se il passo è accettato, false se rifiutato (e dt ridotto)
   */
  bool stepAdaptive(double &t,
                    std::vector<double> &y,
                    double &dt,
                    std::function<void(double,const std::vector<double>&,std::vector<double>&)> f);

private:
  double absTol, relTol;
};
