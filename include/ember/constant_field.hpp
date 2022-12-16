// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <frantic/strings/tstring.hpp>
#include <frantic/volumetrics/field_interface.hpp>

namespace ember {

template <class T>
class constant_field : public frantic::volumetrics::field_interface {
  public:
    explicit constant_field( const T& value, const frantic::tstring& channelName = _T("Value") );

    virtual bool evaluate_field( void* dest, const frantic::graphics::vector3f& pos ) const;

    virtual const frantic::channels::channel_map& get_channel_map() const;

  private:
    frantic::channels::channel_map m_map;
    T m_value;
};

template <class T>
inline constant_field<T>::constant_field( const T& value, const frantic::tstring& channelName )
    : m_value( value ) {
    m_map.define_channel<T>( channelName );
    m_map.end_channel_definition();
}

template <class T>
inline bool constant_field<T>::evaluate_field( void* dest, const frantic::graphics::vector3f& /*pos*/ ) const {
    static_cast<T*>( dest )[0] = m_value;
    return true;
}

template <class T>
inline const frantic::channels::channel_map& constant_field<T>::get_channel_map() const {
    return m_map;
}

} // namespace ember
