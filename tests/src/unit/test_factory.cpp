#include <utility>

#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "testing/mock_factory.hpp"

namespace ap = autopilot;

using testing::ConcreteMockComponent;

// TEST(MockFactoryTest, PreventDoubleRegistration) {
//   ASSERT_FALSE(testing::MockComponentFactory::Register<ConcreteMockComponent>(
//       ConcreteMockComponent::kName));
// }

TEST(MockFactoryTest, CreateConfig) {
  auto config =
      testing::MockComponentFactory::CreateConfig(ConcreteMockComponent::kName);
  ASSERT_THAT(config, testing::NotNull());
  EXPECT_THAT(config->name(), testing::StrEq(ConcreteMockComponent::kName));
  auto concrete_config =
      std::dynamic_pointer_cast<ConcreteMockComponent::Config>(config);
  ASSERT_THAT(concrete_config, testing::NotNull());
  EXPECT_EQ(concrete_config->some_parameter, 42);
}

TEST(MockFactoryTest, CreateConfigInvalidName) {
  auto config =
      testing::MockComponentFactory::CreateConfig("NonExistentComponent");
  EXPECT_THAT(config, testing::IsNull());
}

TEST(MockFactoryTest, CreateInstanceByName) {
  auto model = std::make_shared<ap::QuadrotorModel>();
  auto instance = testing::MockComponentFactory::Create(
      ConcreteMockComponent::kName, model);
  ASSERT_THAT(instance, testing::NotNull());
  EXPECT_THAT(instance->name(), testing::StrEq(ConcreteMockComponent::kName));
}

TEST(MockFactoryTest, CreateInstanceInvalidName) {
  auto model = std::make_shared<ap::QuadrotorModel>();
  auto instance =
      testing::MockComponentFactory::Create("NonExistentComponent", model);
  EXPECT_THAT(instance, testing::IsNull());
}

TEST(MockFactoryTest, CreateInstanceByConfig) {
  auto model = std::make_shared<ap::QuadrotorModel>();
  auto config = std::make_shared<ConcreteMockComponent::Config>();
  config->some_parameter = 99;

  auto instance = testing::MockComponentFactory::Create(config, model);
  ASSERT_THAT(instance, testing::NotNull());
  EXPECT_THAT(instance->name(), testing::StrEq(ConcreteMockComponent::kName));
}
