// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <stoke/data_types.hpp>
#include <stoke/time_interface.hpp>

#include <boost/shared_ptr.hpp>

namespace stoke {

class field_interface {
  public:
    virtual ~field_interface() {}

    virtual float get_velocity_scale() = 0;

    virtual void set_velocity_scale( float newScale ) = 0;

    virtual void update( const time_interface& updateTime ) = 0;

    // virtual void reset_simulation() = 0;

    // virtual void advance_simulation() = 0;

    virtual vec3 evaluate_velocity( const vec3& p ) const = 0;
};

typedef boost::shared_ptr<field_interface> field_interface_ptr;

} // namespace stoke
