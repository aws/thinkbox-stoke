// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#include "stdafx.h"

#include <stoke/additive_velocity_field.hpp>

namespace stoke {

additive_velocity_field::~additive_velocity_field() {}

float additive_velocity_field::get_velocity_scale() { return m_velocityScale; }

void additive_velocity_field::set_velocity_scale( float newScale ) { m_velocityScale = newScale; }

void additive_velocity_field::update( const time_interface& updateTime ) {
    for( std::vector<field_interface_ptr>::iterator it = m_implFields.begin(), itEnd = m_implFields.end(); it != itEnd;
         ++it )
        ( *it )->update( updateTime );
}

// void additive_velocity_field::reset_simulation(){
//	for( std::vector< field_interface_ptr >::iterator it = m_implFields.begin(), itEnd = m_implFields.end(); it !=
//itEnd; ++it )
//		(*it)->reset_simulation();
// }
//
// void additive_velocity_field::advance_simulation(){
//	for( std::vector< field_interface_ptr >::iterator it = m_implFields.begin(), itEnd = m_implFields.end(); it !=
//itEnd; ++it )
//		(*it)->advance_simulation();
// }

vec3 additive_velocity_field::evaluate_velocity( const vec3& p ) const {
    vec3 result( 0, 0, 0 );

    for( std::vector<field_interface_ptr>::const_iterator it = m_implFields.begin(), itEnd = m_implFields.end();
         it != itEnd; ++it )
        result += ( *it )->evaluate_velocity( p );

    return m_velocityScale * result;
}

} // namespace stoke
