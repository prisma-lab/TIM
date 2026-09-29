#include "ur10_hardware_primitives/manipulation_primitive.hpp"
#include <pluginlib/class_list_macros.hpp>

namespace ur10_hardware_primitives
{
class PickPrimitive final : public ManipulationPrimitive
{
protected:
  const char * operation() const override {return "pick";}
  std::vector<Step> stages() const override
  {
    return {Step::PREPARE_SCENE, Step::OPEN, Step::DOWN, Step::GRASP, Step::ATTACH_SCENE, Step::RETREAT, Step::VERIFY};
  }
};
}
PLUGINLIB_EXPORT_CLASS(ur10_hardware_primitives::PickPrimitive, primitive_manager::PrimitiveBase)
