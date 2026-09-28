#include "ur10_primitives/manipulation_primitive.hpp"
#include <pluginlib/class_list_macros.hpp>

namespace ur10_primitives
{
class MoveABPrimitive final : public ManipulationPrimitive
{
protected:
  const char * operation() const override {return "move_a_b";}
  std::vector<Step> stages() const override
  {
    return {Step::ABOVE, Step::VERIFY};
  }
};
}
PLUGINLIB_EXPORT_CLASS(ur10_primitives::MoveABPrimitive, primitive_manager::PrimitiveBase)
