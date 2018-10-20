#include <time.h>

#include "../include/dp.hpp"
#include "../include/world.hpp"
#include "../include/qlearning.hpp"
#include "../include/montecarlo.hpp"

int main(){
  srand(time(NULL));
  cleaner::world w(10,10,3,3);
  std::cout << w << std::endl;

  printf("%d", w.getNumStates());
  std::vector<bool> grid = w.getState(0)->getGrid();
  printf("Grid :   ");
  for (std::vector<bool>::iterator it = grid.begin() ; it != grid.end(); ++it){
    std::cout << std::boolalpha << *it;
    //printf("%s ", *it);
  }
  printf("\n");
  int nbBases = 0;
  for(int i = 0; i < w.getNumStates(); i++){
    if(w.getState(i)->getBase() == true){
      nbBases += 1;
      printf("Base on state %d\n", i);
    }
  }
  printf("Nb d'état sur bases : %d\n", nbBases);
  //printf("%d",w.getState(2399)->getPose());

  // cleaner::dp dp_solver(w, 0.001, 0.99);
  // dp_solver.solve();
  // std::cout << "dp_solver("<< *w.getState(0) << ") = " << dp_solver.getValueAt(0) << std::endl;

  // cleaner::montecarlo mc_solver(w, 0.1, 0.1, 0.99, 3000);
  // mc_solver.solve();
  // std::cout << "mc_solver("<< *w.getState(0) << ") = " << mc_solver.getValueAt(0) << std::endl;

   cleaner::qlearning q_solver(w, 0.1, 0.1, 0.99, 1000);
   q_solver.solve();
   std::cout << "q_solver("<< *w.getState(0) << ") = " << q_solver.getValueAt(0) << std::endl;

  return 0;
}
