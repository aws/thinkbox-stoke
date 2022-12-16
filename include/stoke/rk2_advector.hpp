// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <stoke/advector_interface.hpp>
#include <stoke/field_interface.hpp>

namespace stoke {

class rk2_advector : public advector_interface {
  public:
    virtual ~rk2_advector() {}

    virtual vec3 advect_particle( const vec3& p, const vec3& v, const field_interface& velocityField,
                                  float timeStepSeconds ) const;

    virtual vec3 get_offset( const vec3& p, const vec3& v, const field_interface& velocityField,
                             float timeStepSeconds ) const;
};

inline vec3 rk2_advector::advect_particle( const vec3& p, const vec3& v, const field_interface& velocityField,
                                           float timeStepSeconds ) const {
    return p + get_offset( p, v, velocityField, timeStepSeconds );
}

inline vec3 rk2_advector::get_offset( const vec3& p, const vec3& v, const field_interface& velocityField,
                                      float timeStepSeconds ) const {
    vec3 pTemp = p + timeStepSeconds * v;
    vec3 vTemp = velocityField.evaluate_velocity( pTemp );

    return 0.5f * timeStepSeconds * ( v + vTemp );
}

} // namespace stoke
