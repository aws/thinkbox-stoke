// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <stoke/advector_interface.hpp>
#include <stoke/field_interface.hpp>

namespace stoke {

class rk3_advector : public advector_interface {
  public:
    virtual ~rk3_advector() {}

    virtual vec3 advect_particle( const vec3& p, const vec3& v, const field_interface& velocityField,
                                  float timeStepSeconds ) const;

    virtual vec3 get_offset( const vec3& p, const vec3& v, const field_interface& velocityField,
                             float timeStepSeconds ) const;
};

inline vec3 rk3_advector::advect_particle( const vec3& p, const vec3& v, const field_interface& velocityField,
                                           float timeStepSeconds ) const {
    return p + get_offset( p, v, velocityField, timeStepSeconds );
}

inline vec3 rk3_advector::get_offset( const vec3& p, const vec3& v, const field_interface& velocityField,
                                      float timeStepSeconds ) const {
    vec3 v2 = velocityField.evaluate_velocity( p + 0.5f * timeStepSeconds *
                                                       v ); // Should technically evaluate velocityField at - 1/2 the
                                                            // timestep. TODO: Could self advect the velocity.
    vec3 v3 = velocityField.evaluate_velocity( p + 0.75f * timeStepSeconds * v2 );

    return ( timeStepSeconds / 9.f ) * ( 2.f * v + 3.f * v2 + 4.f * v3 );
}

} // namespace stoke
