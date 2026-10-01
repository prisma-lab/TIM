#include "ur10_primitives/parameterized_primitive.hpp"
#include <pluginlib/class_list_macros.hpp>

namespace ur10_primitives
{
class ParameterizedMoveABPrimitive final : public ParameterizedPrimitive
{
  const char * operation() const override {return "move_a_b";}
  std::vector<Step> stages() const override {return {Step::ABOVE, Step::VERIFY};}
};
class ParameterizedPickPrimitive final : public ParameterizedPrimitive
{
  const char * operation() const override {return "pick";}
  std::vector<Step> stages() const override
  {return {Step::OPEN, Step::DOWN, Step::GRASP, Step::ATTACH, Step::ATTACH_SCENE, Step::RETREAT, Step::VERIFY};}
};
class ParameterizedPlacePrimitive final : public ParameterizedPrimitive
{
  const char * operation() const override {return "place";}
  std::vector<Step> stages() const override
  {return {Step::DOWN, Step::OPEN, Step::DETACH, Step::DETACH_SCENE, Step::RETREAT, Step::VERIFY};}
};
}
PLUGINLIB_EXPORT_CLASS(ur10_primitives::ParameterizedMoveABPrimitive, primitive_manager::PrimitiveBase)
PLUGINLIB_EXPORT_CLASS(ur10_primitives::ParameterizedPickPrimitive, primitive_manager::PrimitiveBase)
PLUGINLIB_EXPORT_CLASS(ur10_primitives::ParameterizedPlacePrimitive, primitive_manager::PrimitiveBase)
