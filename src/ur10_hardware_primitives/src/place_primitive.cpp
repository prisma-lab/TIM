#include "ur10_hardware_primitives/manipulation_primitive.hpp"
#include <pluginlib/class_list_macros.hpp>

namespace ur10_hardware_primitives
{
class PlacePrimitive final : public ManipulationPrimitive
{
protected:
  const char * operation() const override {return "place";}
  std::vector<Step> stages() const override
  {
    return {Step::DOWN, Step::OPEN, Step::DETACH_SCENE, Step::RETREAT, Step::VERIFY};
  }
};
}
PLUGINLIB_EXPORT_CLASS(ur10_hardware_primitives::PlacePrimitive, primitive_manager::PrimitiveBase)
