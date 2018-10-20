#include "../include/state.hpp"

namespace cleaner{
  state::state(std::vector<bool>const& grid, bool base, size battery, size pose): base(base), battery(battery), pose(pose), grid(grid){}

  state::~state(){}

  std::vector<bool>const& state::getGrid() const{
    return grid;
  }

  bool state::getBase() const{
    return base;
  }

  size state::getBattery() const{
    return battery;
  }

  size state::getPose() const{
    return pose;
  }
}
