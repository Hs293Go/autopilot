#include "testing/mock_factory.hpp"

static const autopilot::Registrar<testing::ConcreteMockComponent,
                                  testing::MockComponentFactory>
    kRegistrar(testing::ConcreteMockComponent::kName);
