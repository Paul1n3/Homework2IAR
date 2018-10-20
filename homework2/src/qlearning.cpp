#include "../include/qlearning.hpp"


namespace cleaner{
    qlearning::qlearning(world const& w, double epsilon, double learning_rate, double gamma, int episodes) : w(w), episodes(episodes), gamma(gamma), epsilon(epsilon), learning_rate(learning_rate){
      gp = new Gnuplot;
    }

    qlearning::~qlearning(){
      delete gp;
    }

    void qlearning::plots(){
      // std::cout << this->getValueAt(0) << std::endl;
      points.push_back(std::make_pair(this->episode, this->getValueAt(0)));
      *gp << "'-' binary" << gp->binFmt1d(points, "record") << "with lines title 'Value at initial state'\n";
      gp->sendBinary1d(points);
      gp->flush();
  }

    void qlearning::solve(){
      double r;
      int s, a, ss;
      this->init();

      do{
        s=0;
        for(int i=0; i<100; i++){
          a = greedy(s);
          w.execute(s, static_cast<action>(a), ss, r);
          this->backup(s,a,ss,r);
          s = ss;
        }

        // this->plots();
      }while( ++this->episode < this->episodes );
    }

    double qlearning::getValueAt(int s){
      double value = MIN;
      for(int a=0; a<action::END; ++a){
        value = std::max(value, getScalar());
      } return value;
    }

    int qlearning::greedy(int s){
      int agreedy;
      double value = MIN;
      double rd = rand() / ((double) RAND_MAX);

      if( rd > this->epsilon ) {
        for(int a=0; a<action::END; ++a){
          if( value < getScalar(s, a) ){
            agreedy = a;
            value = getScalar(s, a);
          }
        }
      }

      else {
        agreedy = rand() % 7;
      }

      return agreedy;
    }

    void qlearning::backup(int s, int a, int ss, double r){
      std::vector<double> p = phi(s,a);
      for(int i = 0; i < p.size; i++){
        this->theta[i] += this->learning_rate * (r + this->gamma*getValueAt(ss) - getScalar(s,a))) * p[i];
      }
    }

    void qlearning::init(){
      for(int s=0; s<this->w.getNumStates(); ++s){
        this->theta.emplace(s,  std::unordered_map<int, double>());
        this->phi.emplace(s,  std::unordered_map<int, double>());
        for(int a=0; a<action::END; ++a){
          this->theta.at(s).emplace(a, 0.0);
          this->phi.at(s).emplace(a, 0.0);
        }
      }
    }

    std::vector<double> qlearning::phi(int s, int a){
      std::vector<double> p;
      for(int i = 0; i < this->nb_pi; i++){
        this->p.emplace(i, 0.0);
      }
      // Si on est sur la base, on veut que le robot se recharge
      if(s.getBase() && s.getBattery < this->cbattery && a  == action::CHARGE){
        p[0]= 10;
      }
      // Si on a juste assez de batterie pour revenir à la base, on revient
      // TODO: Position base?
      if(){

      }
      // Si case sale, on nettoie
      if(){

      }
      // ! Si pas de mur à gauche ou case de gauche est clean
      if( !(s.getPose() % this->width == 0 || s.getGrid(s.getPose()-1)) && a == action::LEFT){

      }
      // ! Si pas de mur en haut ou case en haut est clean
      else if(){

      }
      // ! Si pas de mur à droite ou case à droit est clean
      else if(){

      }
      // ! Si pas de mur en bas ou case en bas est clean
      else if(){

      }
      // ! Si que des murs et des cases nettoyées autour, case sale la plus proche
      else if(){
      }
      return p;
    }

    double qlearning::getScalar(int s, int a){
      double result;
      std::vector<double> p = phi(s,a);
      for(int i = 0; i < p.size; i++){
          result+=this->theta[i] * p[i];
        }
      }
      return result;
  }
}
