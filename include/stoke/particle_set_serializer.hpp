// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <stoke/particle_set_interface.hpp>

#include <frantic/particles/streams/particle_ostream.hpp>

#include <boost/cstdint.hpp>
#include <boost/function.hpp>

namespace stoke {

/**
 * This is a simple pimpl serialization class that can be easily copied with ptr semantics.
 */
class particle_set_serializer {
  public:
    particle_set_serializer();

    // Provide a specific map that should be written to disk.
    void override_output_map( const frantic::channels::channel_map& map ) const;

    void serialize( const frantic::tstring& filePath, const particle_set_interface_ptr& pParticleSet ) const;

    // This version does not capture exceptions, so errors are immediately propogated to the caller.
    void serialize_immediate( const frantic::tstring& filePath, const particle_set_interface_ptr& pParticleSet ) const;

    particle_set_interface_ptr deserialize( const frantic::tstring& filePath ) const;

    void process_errors() const;

    void set_serialize_callback( const boost::function<void( const frantic::tstring& )>& cb );

  private:
    struct impl;

    boost::shared_ptr<impl> m_pImpl;
};

void set_particle_file_ostream_factory_temp_directory( const frantic::tstring& directoryPath );

boost::shared_ptr<frantic::particles::streams::particle_ostream>
particle_file_ostream_factory( const frantic::tstring& file, const frantic::channels::channel_map& particleChannelMap,
                               const frantic::channels::channel_map& particleChannelMapForFile,
                               boost::int64_t expectedParticleCount = -1, int zlibCompressionLevel = -1 );

} // namespace stoke
