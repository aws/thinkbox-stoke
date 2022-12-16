// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <stoke/advector_interface.hpp>

namespace stoke {

class dummy_advector : public advector_interface {
  public:
    virtual ~dummy_advector() {}

    virtual vec3 advect_particle( const vec3& p, const vec3& v, const field_interface& velocityField,
                                  float timeStepSeconds ) const;

    virtual vec3 get_offset( const vec3& p, const vec3& v, const field_interface& velocityField,
                             float timeStepSeconds ) const;
};

inline vec3 dummy_advector::advect_particle( const vec3& p, const vec3& /*v*/, const field_interface& /*velocityField*/,
                                             float /*timeStepSeconds*/ ) const {
    return p;
}

inline vec3 dummy_advector::get_offset( const vec3& /*p*/, const vec3& /*v*/, const field_interface& /*velocityField*/,
                                        float /*timeStepSeconds*/ ) const {
    return vec3( 0.0f );
}

} // namespace stoke
