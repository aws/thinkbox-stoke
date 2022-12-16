// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <stoke/field_interface.hpp>

#include <frantic/volumetrics/field_interface.hpp>

namespace stoke {

// This class partially implements some feature of field_interface, but it must still be subclassed to implement some
// member functions.
class field : public field_interface {
  public:
    field();

    virtual ~field() {}

    virtual float get_velocity_scale();

    virtual void set_velocity_scale( float newScale );

    virtual void update( const time_interface& updateTime ) = 0;

    // virtual void reset_simulation() = 0;

    // virtual void advance_simulation() = 0;

    virtual vec3 evaluate_velocity( const vec3& p ) const;

  protected:
    void set_field( boost::shared_ptr<frantic::volumetrics::field_interface> pNewField );

  private:
    boost::shared_ptr<frantic::volumetrics::field_interface> m_pCurField;

    float m_velocityScale;

    frantic::channels::channel_map m_resultMap;

    frantic::channels::channel_cvt_accessor<frantic::graphics::vector3f> m_velocityAccessor;
};

} // namespace stoke
