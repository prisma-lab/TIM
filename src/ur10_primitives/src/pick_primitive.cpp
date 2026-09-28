#include "ur10_primitives/manipulation_primitive.hpp"
#include <pluginlib/class_list_macros.hpp>

namespace ur10_primitives
{
class PickPrimitive final : public ManipulationPrimitive
{
protected:
  const char * operation() const override {return "pick";}
  std::vector<Step> stages() const override
  {
    return {Step::OPEN, Step::DOWN, Step::GRASP, Step::ATTACH, Step::ATTACH_SCENE, Step::RETREAT, Step::VERIFY};
  }
};
}
PLUGINLIB_EXPORT_CLASS(ur10_primitives::PickPrimitive, primitive_manager::PrimitiveBase)
