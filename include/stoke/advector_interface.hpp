// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <stoke/data_types.hpp>
#include <stoke/field_interface.hpp>

#include <boost/shared_ptr.hpp>

namespace stoke {

class advector_interface {
  public:
    virtual ~advector_interface() {}

    /**
     * This will return the actual advected position.
     */
    virtual vec3 advect_particle( const vec3& p, const vec3& v, const field_interface& velocityField,
                                  float timeStepSeconds ) const = 0;

    /**
     * This will just return the offset that the position would have if it had been advected.
     */
    virtual vec3 get_offset( const vec3& p, const vec3& v, const field_interface& velocityField,
                             float timeStepSeconds ) const = 0;
};

typedef boost::shared_ptr<advector_interface> advector_interface_ptr;

} // namespace stoke
