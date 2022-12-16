// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#include "stdafx.h"

#include <stoke/field.hpp>

#include <stdexcept>

namespace stoke {

field::field() {
    m_velocityScale = 1.f;
    m_velocityAccessor.reset( vec3( 0, 0, 0 ) );
}

float field::get_velocity_scale() { return m_velocityScale; }

void field::set_velocity_scale( float newScale ) { m_velocityScale = newScale; }

vec3 field::evaluate_velocity( const vec3& p ) const {
    char* buffer = (char*)alloca( m_resultMap.structure_size() );

    if( !m_pCurField->evaluate_field( buffer, p ) )
        m_resultMap.construct_structure( buffer ); // Default init if we couldn't grab from the field.

    return m_velocityScale * m_velocityAccessor.get( buffer );
}

void field::set_field( boost::shared_ptr<frantic::volumetrics::field_interface> pNewField ) {
    m_pCurField = pNewField;
    m_resultMap = pNewField->get_channel_map();

    frantic::tstring channelName = _T("Velocity");

    if( !m_resultMap.has_channel( channelName ) ) {
        if( m_resultMap.channel_count() == 1 && m_resultMap[0].arity() == 3 &&
            m_resultMap[0].data_type() == frantic::channels::data_type_float32 )
            channelName = m_resultMap[0].name();
        else
            throw std::runtime_error( "Field source did not expose \"Velocity\" channel" );
    }

    m_velocityAccessor = m_resultMap.get_cvt_accessor<frantic::graphics::vector3f>( channelName );
}

} // namespace stoke
