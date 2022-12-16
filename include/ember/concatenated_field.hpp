// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <boost/shared_ptr.hpp>
#include <frantic/volumetrics/field_interface.hpp>

namespace ember {

class concatenated_field : public frantic::volumetrics::field_interface {
    std::vector<std::pair<boost::shared_ptr<frantic::volumetrics::field_interface>, std::ptrdiff_t>> m_fields;
    frantic::channels::channel_map m_combinedMap;

  private:
    inline static void combine_maps( frantic::channels::channel_map& outResult,
                                     const frantic::channels::channel_map& first,
                                     const frantic::channels::channel_map& second ) {
        outResult.reset();

        for( std::size_t i = 0, iEnd = first.channel_count(); i < iEnd; ++i )
            outResult.define_channel( first[i].name(), first[i].arity(), first[i].data_type(), first[i].offset() );

        std::size_t structOffset = first.structure_size(); // TODO: Consider aligning the second structure.

        for( std::size_t i = 0, iEnd = second.channel_count(); i < iEnd; ++i )
            outResult.define_channel( second[i].name(), second[i].arity(), second[i].data_type(),
                                      structOffset + second[i].offset() );

        outResult.end_channel_definition( 4u, true, false );
    }

  public:
    concatenated_field() { m_combinedMap.end_channel_definition(); }

    void add_field( const boost::shared_ptr<frantic::volumetrics::field_interface>& field ) {
        m_fields.push_back( std::make_pair( field, static_cast<std::ptrdiff_t>( m_combinedMap.structure_size() ) ) );

        frantic::channels::channel_map newMap;

        combine_maps( newMap, m_combinedMap, field->get_channel_map() );

        newMap.swap( m_combinedMap );
    }

    std::size_t get_num_fields() const { return m_fields.size(); }

    // This is likely going to be awkward if some of the fields create multiple channels.
    boost::shared_ptr<frantic::volumetrics::field_interface> get_field( std::size_t fieldIndex ) const {
        return m_fields[fieldIndex].first;
    }

    // This does an O(N) search for a field producing the named channel.
    boost::shared_ptr<frantic::volumetrics::field_interface>
    find_field_producing( const frantic::tstring& channelName ) const {
        boost::shared_ptr<frantic::volumetrics::field_interface> result;

        for( std::vector<
                 std::pair<boost::shared_ptr<frantic::volumetrics::field_interface>, std::ptrdiff_t>>::const_iterator
                 it = m_fields.begin(),
                 itEnd = m_fields.end();
             it != itEnd && !result; ++it ) {
            if( it->first->get_channel_map().has_channel( channelName ) )
                result = it->first;
        }

        return result;
    }

    virtual const frantic::channels::channel_map& get_channel_map() const { return m_combinedMap; }

    virtual bool evaluate_field( void* dest, const frantic::graphics::vector3f& pos ) const {
        for( std::vector<
                 std::pair<boost::shared_ptr<frantic::volumetrics::field_interface>, std::ptrdiff_t>>::const_iterator
                 it = m_fields.begin(),
                 itEnd = m_fields.end();
             it != itEnd; ++it )
            it->first->evaluate_field( reinterpret_cast<char*>( dest ) + it->second, pos );

        return true;
    }
};

} // namespace ember
