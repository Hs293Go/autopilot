#include <vector>

#include "autopilot/base/config_base.hpp"
#include "autopilot/core/common.hpp"
#include "autopilot/extensions/json_loader.hpp"
#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "testing/matchers.hpp"
#include "testing/mock_factory.hpp"

namespace ap = autopilot;
using ap::Bounds;
using ap::Describe;
using ap::F64Properties;
using ap::I64Properties;
using ap::Properties;
using ap::StrProperties;

// =============================================================================
// 1. Define Test Configuration Structures
// =============================================================================

struct NestedConfig : public ap::ReflectiveConfigBase<NestedConfig> {
  std::string_view name() const override { return "NestedConfig"; }
  double inner_val = 1.0;

  static constexpr auto kDescriptors =
      std::make_tuple(Describe("inner_val", &NestedConfig::inner_val,
                               F64Properties{.desc = "Inner Value"}));
};

enum class TestEnum : std::int64_t {
  kOptionA = 0,
  kOptionB = 1,
  kOptionC = 2,
};

using ap::operator""_s;

struct TestConfig : public ap::ReflectiveConfigBase<TestConfig> {
  std::string_view name() const override { return "TestConfig"; }

  // 1. Numeric with Bounds (Inclusive/Exclusive)
  double limited_f64 = 5.0;  // Bounds: (0.0, 10.0]

  // 2. Required Field
  double required_val = 0.0;

  // 3. String validation
  std::string id = "default";

  // 4. Container (Vector)
  std::vector<double> gains = {1.0, 1.0, 1.0};

  // 5. Nested Object
  NestedConfig child;

  ap::Polymorphic<testing::MockComponentFactory> polymorphic_component;

  TestEnum enum_field = TestEnum::kOptionA;

  static constexpr auto kDescriptors = std::make_tuple(
      Describe("limited_f64", &TestConfig::limited_f64,
               F64Properties{
                   .desc = "Limited Double",
                   .bounds = Bounds<double>::OpenInterval(0.0, 10.0)  // (0, 10)
               }),
      Describe("required_val", &TestConfig::required_val,
               F64Properties{.desc = "Required Value",
                             .prefer_user_provided = true}),
      Describe("id", &TestConfig::id,
               StrProperties{.desc = "Identifier", .non_empty = true}),
      Describe("gains", &TestConfig::gains,
               F64Properties{.desc = "Gains Vector",
                             .bounds = Bounds<double>::Positive()}),
      Describe("child", &TestConfig::child,
               Properties{.desc = "Nested Child Config"}),
      Describe("polymorphic_component", &TestConfig::polymorphic_component,
               Properties{.desc = "Polymorphic Component Config"}),
      Describe("enum_field", &TestConfig::enum_field,
               I64Properties{
                   .desc = "Test Enum Field",
                   .map = ap::kEnumMapping<"OptionA"_s, TestEnum::kOptionA,
                                           "OptionB"_s, TestEnum::kOptionB,
                                           "OptionC"_s, TestEnum::kOptionC>}));
};

// =============================================================================
// 2. Test Fixture
// =============================================================================

class TestConfigLoader : public ::testing::Test {
 protected:
  TestConfig cfg_;

  // Helper to load from string and check error code
  std::error_code load(std::string_view json_str) {
    auto loader = ap::JsonLoader::FromString(json_str);
    if (!loader) {
      // If parsing failed entirely (invalid JSON syntax)
      return std::make_error_code(std::errc::invalid_argument);
    }
    auto result = cfg_.accept(*loader);
    return result.ec;
  }
};

// =============================================================================
// 3. Test Cases
// =============================================================================

TEST_F(TestConfigLoader, LoadValidConfig) {
  const char* json = R"({
    "limited_f64": 5.5,
    "required_val": 42.0,
    "id": "test_unit",
    "gains": [0.1, 0.2, 0.3],
    "child": {
      "inner_val": 99.9
    }
  })";

  auto ec = load(json);
  ASSERT_EQ(ec, std::error_code()) << "Message: " << ec.message();

  EXPECT_DOUBLE_EQ(cfg_.limited_f64, 5.5);
  EXPECT_DOUBLE_EQ(cfg_.required_val, 42.0);
  EXPECT_EQ(cfg_.id, "test_unit");
  EXPECT_THAT(cfg_.gains, testing::ElementsAre(0.1, 0.2, 0.3));
  EXPECT_DOUBLE_EQ(cfg_.child.inner_val, 99.9);
}

TEST_F(TestConfigLoader, PartialLoadUsesDefaults) {
  // 'limited_f64' is missing, should retain default 5.0
  const char* json = R"({
    "required_val": 10.0
  })";

  ASSERT_EQ(load(json), std::error_code());
  EXPECT_DOUBLE_EQ(cfg_.limited_f64, 5.0);
}

TEST_F(TestConfigLoader, MissingRequiredKey) {
  // 'required_val' has .prefer_user_provided = true
  const char* json = R"({
    "limited_f64": 5.0
  })";

  auto ec = load(json);
  EXPECT_EQ(ec, make_error_code(ap::AutopilotErrc::kConfigKeyMissing));
}

TEST_F(TestConfigLoader, OutOfBoundsDouble) {
  // Bounds are (0.0, 10.0).
  // Case A: Too low
  EXPECT_EQ(load(R"({"required_val": 1, "limited_f64": 0.0})"),
            make_error_code(ap::AutopilotErrc::kOutOfBounds));

  // Case B: Too high
  EXPECT_EQ(load(R"({"required_val": 1, "limited_f64": 10.0})"),
            make_error_code(
                ap::AutopilotErrc::kOutOfBounds));  // Exclusive upper bound?
                                                    // Checking impl...
  // Implementation: OpenInterval uses Exclusive for both.
  // 10.0 < 10.0 is false. So 10.0 should fail.
}

TEST_F(TestConfigLoader, OutOfBoundsVectorElement) {
  // 'gains' must be Positive (> 0)
  const char* json = R"({
    "required_val": 1,
    "gains": [1.0, -0.5, 1.0]
  })";

  EXPECT_EQ(load(json), make_error_code(ap::AutopilotErrc::kOutOfBounds));
}

TEST_F(TestConfigLoader, EmptyStringNotAllowed) {
  // 'id' has .non_empty = true
  const char* json = R"({
    "required_val": 1,
    "id": ""
  })";

  EXPECT_EQ(load(json),
            make_error_code(ap::AutopilotErrc::kEmptyValueNotAllowed));
}

TEST_F(TestConfigLoader, TypeMismatch) {
  // 'limited_f64' expects double, got string
  const char* json = R"({
    "required_val": 1,
    "limited_f64": "five"
  })";

  EXPECT_EQ(load(json),
            make_error_code(ap::AutopilotErrc::kConfigTypeMismatch));
}

TEST_F(TestConfigLoader, ArraySizeMismatch) {
  const char* json = R"({
    "required_val": 1,
    "gains": [1.0, 2.0]
  })";

  EXPECT_EQ(load(json),
            make_error_code(ap::AutopilotErrc::kConfigSizeMismatch));
}

TEST_F(TestConfigLoader, PolymorphicConfigLoading) {
  const char* json = R"({
    "required_val": 1,
    "polymorphic_component": {
      "type": "ConcreteMockComponent",
      "config": {
        "some_parameter": 2
      }
    }
  })";

  auto ec = load(json);
  ASSERT_EQ(ec, std::error_code()) << "Message: " << ec.message();

  auto& poly = cfg_.polymorphic_component;
  ASSERT_TRUE(poly.config);
  EXPECT_EQ(poly.type, "ConcreteMockComponent");

  // Downcast to MockComponentCfg to check parameters
  auto mock_cfg =
      std::dynamic_pointer_cast<testing::ConcreteMockComponent::Config>(
          poly.config);
  ASSERT_NE(mock_cfg, nullptr);
  EXPECT_EQ(mock_cfg->some_parameter, 2L);
}

TEST_F(TestConfigLoader, PolymorphicMissingKeys) {
  // Missing 'type' key
  const char* json_missing_type = R"({
    "required_val": 1,
    "polymorphic_component": {
      "config": {
        "some_parameter": 2
      }
    }
  })";

  EXPECT_EQ(load(json_missing_type),
            make_error_code(ap::AutopilotErrc::kConfigKeyMissing));

  // Missing 'type' key
  const char* json_empty_type = R"({
    "required_val": 1,
    "polymorphic_component": {
      "type": ""
    }
  })";
  EXPECT_EQ(load(json_empty_type),
            make_error_code(ap::AutopilotErrc::kEmptyValueNotAllowed));

  // This should NOT error out and use default config
  const char* json_missing_config = R"({
    "required_val": 1,
    "polymorphic_component": {
      "type": "ConcreteMockComponent"
    }
  })";

  EXPECT_EQ(load(json_missing_config), std::error_code());
  auto& poly = cfg_.polymorphic_component;
  EXPECT_EQ(poly.type, "ConcreteMockComponent");
  EXPECT_THAT(poly.config, testing::NotNull());
}

TEST_F(TestConfigLoader, EnumFieldLoading) {
  // Valid enum string
  const char* json_valid = R"({
    "required_val": 1,
    "enum_field": "OptionB"
  })";

  EXPECT_EQ(load(json_valid), std::error_code());
  EXPECT_EQ(cfg_.enum_field, TestEnum::kOptionB);

  // Invalid enum string
  const char* json_invalid = R"({
    "required_val": 1,
    "enum_field": "InvalidOption"
  })";

  EXPECT_EQ(load(json_invalid),
            make_error_code(ap::AutopilotErrc::kInvalidStringOption));

  // Empty enum string
  const char* json_empty = R"({
    "required_val": 1,
    "enum_field": ""
  })";
  EXPECT_EQ(load(json_empty),
            make_error_code(ap::AutopilotErrc::kEmptyValueNotAllowed));

  // 'enum_field' expects string, got number
  const char* json = R"({
    "required_val": 1,
    "enum_field": 1
  })";

  EXPECT_EQ(load(json),
            make_error_code(ap::AutopilotErrc::kConfigTypeMismatch));
}
