#include <math.h>
#include <tuple>
#include <vector>
#include <unordered_map>

#include "world.hpp"
#include "../include/gnuplot-iostream.h"

//!
//! \file     montecarlo.hpp
//! \author   Jilles S. Dibangoye
//! \brief    montecarlo class
//! \version  1.0
//! \date     15 Octobre 2017
//!
//! This class provides the montecarlo's robot cleaner public interface.
//!

//! \namespace  cleaner
//!
//! Namespace grouping all tools required for the robot cleaner project.
namespace cleaner{
  class montecarlo{
  protected:
    world w;
    int cepisode = 0, episodes;
    //Gnuplot* gp = nullptr;
    Gnuplot gp;
    double MIN = -100000, MAX = 100000;
    double gamma, epsilon, learning_rate;
    std::vector<std::pair<double, double>> points;
    std::vector<std::tuple<int, int, int>> episode;
    std::unordered_map<int, std::unordered_map<int, int>> pf;
    int nb_pi;
    std::vector<double> theta;
    double finalGain;

    double getReturn(int i);
    void setEpisode();
    void backup();
    void plots();
    void init();

  public:
    ~montecarlo();
    montecarlo(world const&, double, double, double, int);
    void solve();
    action greedy(int);
    double getValueAt(int);
    double getScalar(int, int);
    std::vector<double> defPhi(int s, int a);
    double getGainAt();
  };
}
