#include <math.h>
#include <tuple>
#include <vector>
#include <unordered_map>

#include "world.hpp"
#include "../include/gnuplot-iostream.h"

//!
//! \file     qlearning.hpp
//! \author   Jilles S. Dibangoye
//! \brief    qlearning class
//! \version  1.0
//! \date     15 Octobre 2017
//!
//! This class provides the qlearning's robot cleaner public interface.
//!

//! \namespace  cleaner
//!
//! Namespace grouping all tools required for the robot cleaner project.
namespace cleaner{
  class qlearning{
  protected:
    world w;
    int episode = 0, episodes;
    Gnuplot* gp = nullptr;
    double MIN = -100000, MAX = 100000;
    double gamma, epsilon, learning_rate;
    std::vector<std::pair<double, double>> points;
    int nb_pi;
    std::unordered_map<int, std::unordered_map<int, double>> theta;
    std::unordered_map<int, std::unordered_map<int, double>> phi;

    void backup(int, int, int, double);
    //void plots();
    void init();

  public:
    ~qlearning();
    qlearning(world const&, double, double, double, int);
    //void solve();
    //int greedy(int);
    //double getValueAt(int);
    //double qlearning::getScalar(int s, int a);
    //double qlearning::phi(int s, int a);
  };
}
