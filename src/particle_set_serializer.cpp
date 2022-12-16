// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#include "stdafx.h"

#include <stoke/particle_set_istream.hpp>
#include <stoke/particle_set_serializer.hpp>

#include <frantic/logging/progress_logger.hpp>
#include <frantic/particles/particle_file_stream_factory.hpp>
#include <frantic/particles/streams/apply_function_particle_istream.hpp>

#include <boost/bind.hpp>
#include <boost/exception/all.hpp>
#include <boost/make_shared.hpp>

#include <tbb/concurrent_queue.h>

namespace stoke {

struct particle_set_serializer::impl {
    void override_channel_map( const frantic::channels::channel_map& map );

    void serialize( const frantic::tstring& filePath, const particle_set_interface_ptr& pParticleSet ) const;

    particle_set_interface_ptr deserialize( const frantic::tstring& filePath ) const;

    void store_error( const boost::exception_ptr& );

    void process_errors();

    tbb::concurrent_queue<boost::exception_ptr> m_exceptions;

    frantic::channels::channel_map m_outMap;
};

particle_set_serializer::particle_set_serializer()
    : m_pImpl( new impl ) {}

void particle_set_serializer::override_output_map( const frantic::channels::channel_map& map ) const {
    m_pImpl->override_channel_map( map );
}

void particle_set_serializer::serialize( const frantic::tstring& filePath,
                                         const particle_set_interface_ptr& pParticleSet ) const {
    try {
        m_pImpl->serialize( filePath, pParticleSet );
    } catch( ... ) {
        m_pImpl->store_error( boost::current_exception() );
    }
}

void particle_set_serializer::serialize_immediate( const frantic::tstring& filePath,
                                                   const particle_set_interface_ptr& pParticleSet ) const {
    m_pImpl->serialize( filePath, pParticleSet );
}

particle_set_interface_ptr particle_set_serializer::deserialize( const frantic::tstring& filePath ) const {
    return m_pImpl->deserialize( filePath );
}

void particle_set_serializer::process_errors() const { m_pImpl->process_errors(); }

void particle_set_serializer::impl::override_channel_map( const frantic::channels::channel_map& map ) {
    m_outMap = map;
}

void particle_set_serializer::impl::serialize( const frantic::tstring& filePath,
                                               const particle_set_interface_ptr& pParticleSet ) const {
    FF_LOG( debug ) << _T("Flushing cache entry: ") << filePath << std::endl;

    frantic::channels::channel_map map;
    frantic::channels::channel_map fileMap = m_outMap;

    // Leave this default since write_particle_set() will assign the channel map itself.
    map.end_channel_definition();

    // If we didn't override the channels we want to save, then specify the default Stoke particle channels.
    if( m_outMap.channel_count() == 0 ) {
        fileMap.define_channel( _T("Position"), 3, frantic::channels::data_type_float32 );
        fileMap.define_channel( _T("Velocity"), 3, frantic::channels::data_type_float32 );
        fileMap.define_channel( _T("Age"), 1, frantic::channels::data_type_float32 );
        fileMap.define_channel( _T("LifeSpan"), 1, frantic::channels::data_type_float32 );
        fileMap.define_channel( _T("ID"), 1, frantic::channels::data_type_int64 );
        fileMap.define_channel( _T( "AdvectionOffset" ), 3, frantic::channels::data_type_float32 );
        fileMap.define_channel( _T( "FieldVelocity" ), 3, frantic::channels::data_type_float32 );

        for( std::size_t i = 0, iEnd = pParticleSet->get_num_channels(); i < iEnd; ++i ) {
            std::pair<frantic::channels::data_type_t, std::size_t> type;
            frantic::tstring channelName;

            pParticleSet->get_particle_channel( i, &channelName, &type );

            fileMap.define_channel( channelName, type.second, type.first );
        }

        fileMap.end_channel_definition( 1u );
    }

    frantic::particles::particle_istream_ptr pSourceStream;
    frantic::particles::particle_ostream_ptr pOutStream;
    frantic::logging::null_progress_logger logger;

    pSourceStream = boost::make_shared<particle_set_istream>( pParticleSet );
    pSourceStream->set_channel_map( fileMap );

    pOutStream = frantic::particles::particle_file_ostream_factory( filePath, pSourceStream->get_channel_map(), fileMap,
                                                                    pParticleSet->get_count() );

    frantic::particles::save_particle_stream( pSourceStream, pOutStream, logger );
}

particle_set_interface_ptr particle_set_serializer::impl::deserialize( const frantic::tstring& filePath ) const {
    FF_LOG( debug ) << _T("Reloading cache entry: ") << filePath << std::endl;

    frantic::particles::particle_istream_ptr pStream = frantic::particles::particle_file_istream_factory( filePath );

    const frantic::channels::channel_map& nativeMap = pStream->get_native_channel_map();

    if( !nativeMap.has_channel( _T("Position") ) || nativeMap[_T("Position")].arity() != 3 ||
        !frantic::channels::is_channel_data_type_float( nativeMap[_T("Position")].data_type() ) )
        throw frantic::invalid_particle_file_exception(
            frantic::strings::to_string( _T("Particles from: \"") + filePath + _T("\" were not valid") ) );

    pStream->set_channel_map( pStream->get_native_channel_map() );

    return read_particle_set( *pStream );
}

void particle_set_serializer::impl::store_error( const boost::exception_ptr& pError ) {
    m_exceptions.push( pError );

    FF_LOG( error ) << frantic::strings::to_tstring( boost::diagnostic_information( pError ) ) << std::endl;
}

void particle_set_serializer::impl::process_errors() {
    boost::exception_ptr pError;

    // TBB interface changed in 2.2
#if TBB_VERSION_MAJOR > 2 || ( TBB_VERSION_MAJOR == 2 && TBB_VERSION_MINOR > 1 )
    // If the handler doesn't rethrow we keep processing the rest of the exceptions.
    if( m_exceptions.try_pop( pError ) )
        boost::rethrow_exception( pError );
#else
    if( m_exceptions.pop_if_present( pError ) )
        boost::rethrow_exception( pError );
#endif
}

namespace {
frantic::particles::particle_file_stream_factory_object g_streamFactory;
}

void set_particle_file_ostream_factory_temp_directory( const boost::filesystem::path& directoryPath ) {
    g_streamFactory.set_temp_directory( directoryPath );
}

boost::shared_ptr<frantic::particles::streams::particle_ostream>
particle_file_ostream_factory( const frantic::tstring& file, const frantic::channels::channel_map& particleChannelMap,
                               const frantic::channels::channel_map& particleChannelMapForFile,
                               boost::int64_t expectedParticleCount, int zlibCompressionLevel ) {
    return g_streamFactory.create_ostream( file, particleChannelMap, particleChannelMapForFile, expectedParticleCount,
                                           zlibCompressionLevel );
}

} // namespace stoke
