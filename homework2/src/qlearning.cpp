#include "../include/qlearning.hpp"


namespace cleaner{
    qlearning::qlearning(world const& w, double epsilon, double learning_rate, double gamma, int episodes) : w(w), episodes(episodes), gamma(gamma), epsilon(epsilon), learning_rate(learning_rate){
      gp = new Gnuplot;
    }

    qlearning::~qlearning(){
      //delete gp;
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
        for(int i=0; i<1000; i++){
          a = greedy(s);
          w.execute(s, static_cast<action>(a), ss, r);
          this->backup(s,a,ss,r);
          s = ss;
        }
        printf("episode : %d = %f\n",this->episode, this->getValueAt(0));
        //this->plots();
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
        //printf(" theta de %d : %f\n", i, this->theta[i]);
      }
      //sleep(1);
    }

    void qlearning::init(){
      this->nb_pi = 7;
      for(int i=0; i<this->nb_pi; i++){
        this->theta.push_back(0.0);
      }
    }

    action qlearning::NearestDirtyDirection(){
      action a = action::LEFT;
      return a;
    }

    std::vector<double> qlearning::defPhi(int s, int a){
      //printf("Current state: %d, Position: %d, Base? : %d, Sale? %d, action : %d\n", s, w.getState(s)->getPose(), w.getState(s)->getBase(), w.getGrid(w.getState(s)->getGrid(), w.getState(s)->getPose()), a);
      /*std::vector<bool> grid = w.getState(s)->getGrid();
      printf("Grid :   ");
      for (std::vector<bool>::iterator it = grid.begin() ; it != grid.end(); ++it){
        std::cout << std::boolalpha << *it;
        //printf("%s ", *it);
      }
      printf("\n");*/

      std::vector<double> p;
      for(int i = 0; i < this->nb_pi; i++){
        p.push_back(0.1);
      }
      // Si on est sur la base, on veut que le robot se recharge
      if(w.getState(s)->getBase() && w.getState(s)->getBattery() < w.getCBattery() && a  == action::CHARGE){
        p[0]= 1.0;
      }
      // Si on a juste assez de batterie pour revenir à la base, on revient
      // TODO: Position base?
      if((int(w.getState(s)->getBase()) / w.getHeight() + int(w.getState(s)->getBase()) % w.getWidth() == w.getState(s)->getBattery()) && (a  == action::LEFT || a  == action::UP)){
        p[1]= 1.0;
      }
      // Si case sale, on nettoie
      if(!w.getGrid(w.getState(s)->getGrid(), w.getState(s)->getPose()) && a == action::CLEAN){
        p[2]= 1.0;
      }

      bool condition = true;
      // ! Si pas de mur à gauche ou case de gauche est clean
      if(w.getState(s)->getPose() > 0){
        if(!(w.getState(s)->getPose() % w.getWidth() == 0 /*|| !w.getGrid(w.getState(s)->getGrid(), w.getState(s)->getPose() - 1)*/) && a == action::LEFT){
          p[3]= 1.0;
          condition = false;
        }
      }
      // ! Si pas de mur en haut ou case en haut est clean
      if(w.getState(s)->getPose() > (w.getWidth() - 1) && condition){
        if( !(w.getState(s)->getPose() % w.getHeight() == 0 /*|| !w.getGrid(w.getState(s)->getGrid(), w.getState(s)->getPose() - w.getWidth())*/) && a == action::UP ){
          p[4]= 1.0;
          condition = false;
        }
      }
      // ! Si pas de mur à droite ou case à droite est clean
      if(w.getState(s)->getPose() < w.getHeight() * w.getWidth() - 1 && condition){
        if(!(w.getState(s)->getPose() % w.getWidth() == w.getWidth() /*|| !w.getGrid(w.getState(s)->getGrid(), w.getState(s)->getPose() + 1)*/) && a == action::RIGHT ){
          p[5]= 1.0;
          condition = false;
        }
      }
      // ! Si pas de mur en bas ou case en bas est clean
      if(w.getState(s)->getPose() < ((w.getWidth()*w.getHeight())-(w.getWidth() - 1)) && condition){
        if( !(w.getState(s)->getPose() % w.getHeight() == 0 /*|| !w.getGrid(w.getState(s)->getGrid(), w.getState(s)->getPose() + w.getWidth())*/) && a == action::UP ){
          p[6]= 1.0;
          condition = false;
        }
      }
      if(condition){
        p[7] = 1;
      }
      /*// ! Si que des murs et des cases nettoyées autour, case sale la plus proche
      else if((s.getPose() % w.getWidth() == 0 || s.getGrid(s.getPose()-1)) && (s.getPose() % w.getHeight() == 0 || s.getGrid(s.getPose()-w.getWidth())) && (s.getPose() % w.getWidth() == w.getWidth() || s.getGrid(s.getPose()+1)) && (s.getPose() % w.getHeight() == w.getHeight() || s.getGrid(s.getPose()+w.getWidth()))) {
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
