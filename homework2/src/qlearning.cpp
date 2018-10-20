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
        value = std::max(value, this->getScalar(s, a));
      } return value;
    }

    int qlearning::greedy(int s){
      int agreedy;
      double value = MIN;
      double rd = rand() / ((double) RAND_MAX);

      if( rd > this->epsilon ) {
        for(int a=0; a<action::END; ++a){
          if( value < this->getScalar(s, a)){
            agreedy = a;
            value = this->getScalar(s, a);
          }
        }
      }

      else {
        agreedy = rand() % 7;
      }

      return agreedy;
    }

    void qlearning::backup(int s, int a, int ss, double r){
      std::vector<double> p = defPhi(s,a);
      for(int i = 0; i < this->nb_pi; i++){
        this->theta[i] += this->learning_rate * (r + this->gamma*getValueAt(ss) - this->getScalar(s,a)) * p[i];
      }
    }

    void qlearning::init(){
      this->nb_pi = 2;//11;
      for(int i=0; i<this->nb_pi; i++){
        this->theta.push_back(0.0);
      }
    }

    action qlearning::NearestDirtyDirection(){
      action a = action::LEFT;
      return a;
    }

    std::vector<double> qlearning::defPhi(int s, int a){
      std::vector<double> p;
      for(int i = 0; i < this->nb_pi; i++){
        p.push_back(0.0);
      }
      // Si on est sur la base, on veut que le robot se recharge
      if(w.getState(s)->getBase() && w.getState(s)->getBattery() < w.getCBattery() && a  == action::CHARGE){
        p[0]= 10.0;
      }
      // Si on a juste assez de batterie pour revenir à la base, on revient
      // TODO: Position base?
      if((int(w.getState(s)->getBase()) / w.getHeight() + int(w.getState(s)->getBase()) % w.getWidth() == w.getState(s)->getBattery()) && (a  == action::LEFT || a  == action::UP)){
        p[1]= 10.0;
      }
      /*// Si case sale, on nettoie
      for(int i = 0; i < w->)
      if( s.getGrid(s.getPose()) && a == action::CLEAN ){
        p[2]= 10.0;
      }
      // ! Si pas de mur à gauche ou case de gauche est clean
      if( !(s.getPose() % this->width == 0 || s.getGrid(s.getPose()-1)) && a == action::LEFT){
        p[3]= 10.0;
      }
      // ! Si pas de mur en haut ou case en haut est clean
      else if( !(s.getPose() % this->height == 0 || s.getGrid(s.getPose()-this->width)) && a == action::UP ){
        p[4]= 10.0;
      }
      // ! Si pas de mur à droite ou case à droit est clean
      else if( !(s.getPose() % this->width == this->width || s.getGrid(s.getPose()+1)) && a == action::RIGHT ){
        p[5]= 10.0;
      }
      // ! Si pas de mur en bas ou case en bas est clean
      else if( !(s.getPose() % this->height == this->height || s.getGrid(s.getPose()+this->width)) && a == action::DOWN ){
        p[6]= 10.0;
      }
      // ! Si que des murs et des cases nettoyées autour, case sale la plus proche
      else if((s.getPose() % this->width == 0 || s.getGrid(s.getPose()-1)) && (s.getPose() % this->height == 0 || s.getGrid(s.getPose()-this->width)) && (s.getPose() % this->width == this->width || s.getGrid(s.getPose()+1)) && (s.getPose() % this->height == this->height || s.getGrid(s.getPose()+this->width))) {
        action a = NearestDirtyDirection();
        if( a == action:LEFT ){
          p[7] = 10.0;
        }
        else if ( a == action:UP ){
          p[8] = 10.0;
        }
        else if ( a == action:RIGHT ){
          p[9] = 10.0;
        }
        else if ( a == action:DOWN ){
          p[10] = 10.0;
        }
      }*/
      return p;
    }

    double qlearning::getScalar(int s, int a){
      double result = 0.0;
      std::vector<double> p = defPhi(s,a);
      for(int i = 0; i < this->nb_pi; i++){
          result+=this->theta[i] * p[i];
      }
      return result;
    }
}
