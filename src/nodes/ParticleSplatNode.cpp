// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#include "stdafx.h"
#include <ember/ember_compiler.hpp>
#include <ember/grid.hpp>
#include <ember/nodes/ParticleSplatNode.hpp>
#include <ember/staggered_grid.hpp>

#include <ember/advection.hpp>

#include <frantic/magma/nodes/magma_input_particles_interface.hpp>
#include <frantic/magma/nodes/magma_node_impl.hpp>

#include <frantic/channels/channel_accessor.hpp>
#include <frantic/graphics/boundbox3f.hpp>

namespace ember {
namespace nodes {

MAGMA_DEFINE_TYPE( "ParticleSplat", "Stoke", ParticleSplatNode )
MAGMA_INPUT( "Particles", boost::blank() )
MAGMA_INPUT( "Position", boost::blank() )
MAGMA_EXPOSE_PROPERTY( channels, std::vector<frantic::tstring> )
MAGMA_EXPOSE_PROPERTY( absoluteSpacing, float )
MAGMA_EXPOSE_PROPERTY( relativeSpacing, float )
MAGMA_ENUM_PROPERTY( spacingType, "Absolute", "Relative" )
MAGMA_EXPOSE_PROPERTY( autoBounds, bool )
MAGMA_EXPOSE_PROPERTY( boundsPadding, int )
MAGMA_DESCRIPTION( "Splats particles onto a grid. The grid is reconstructed with trilinear sampling." )
MAGMA_DEFINE_TYPE_END;

namespace {
template <class T>
class particle_accessor {
  public:
    particle_accessor( const frantic::channels::channel_accessor<vec3>& posAccessor,
                       const frantic::channels::channel_const_cvt_accessor<T>& dataAccessor,
                       const frantic::channels::channel_const_cvt_accessor<float>& densityAccessor );

    void operator()( const char* pParticle, vec3& outPos, T& outValue ) const;

  private:
    frantic::channels::channel_accessor<vec3> m_posAccessor;
    frantic::channels::channel_const_cvt_accessor<T> m_dataAccessor;
    frantic::channels::channel_const_cvt_accessor<float> m_densityAccessor;
};

template <class T>
inline particle_accessor<T>::particle_accessor(
    const frantic::channels::channel_accessor<vec3>& posAccessor,
    const frantic::channels::channel_const_cvt_accessor<T>& dataAccessor,
    const frantic::channels::channel_const_cvt_accessor<float>& densityAccessor )
    : m_posAccessor( posAccessor )
    , m_dataAccessor( dataAccessor )
    , m_densityAccessor( densityAccessor ) {}

template <class T>
inline void particle_accessor<T>::operator()( const char* pParticle, vec3& outPos, T& outValue ) const {
    outPos = m_posAccessor( pParticle );
    outValue = m_dataAccessor.get( pParticle ) * m_densityAccessor.get( pParticle );
}

class staggered_float_accessor {
  public:
    staggered_float_accessor( const frantic::channels::channel_accessor<vec3>& posAccessor,
                              const frantic::channels::channel_const_cvt_accessor<float>& dataAccessor,
                              const frantic::channels::channel_const_cvt_accessor<float>& densityAccessor );

    void operator()( const char* pParticle, vec3& outPos, vec3& outValue ) const;

  private:
    frantic::channels::channel_accessor<vec3> m_posAccessor;
    frantic::channels::channel_const_cvt_accessor<float> m_dataAccessor;
    frantic::channels::channel_const_cvt_accessor<float> m_densityAccessor;
};

inline staggered_float_accessor::staggered_float_accessor(
    const frantic::channels::channel_accessor<vec3>& posAccessor,
    const frantic::channels::channel_const_cvt_accessor<float>& dataAccessor,
    const frantic::channels::channel_const_cvt_accessor<float>& densityAccessor )
    : m_posAccessor( posAccessor )
    , m_dataAccessor( dataAccessor )
    , m_densityAccessor( densityAccessor ) {}

inline void staggered_float_accessor::operator()( const char* pParticle, vec3& outPos, vec3& outValue ) const {
    outPos = m_posAccessor( pParticle );
    outValue.set( m_dataAccessor.get( pParticle ) );
    outValue *= m_densityAccessor.get( pParticle );
}

class staggered_density_accessor {
  public:
    staggered_density_accessor( const frantic::channels::channel_accessor<vec3>& posAccessor,
                                const frantic::channels::channel_const_cvt_accessor<float>& densityAccessor );

    void operator()( const char* pParticle, vec3& outPos, vec3& outValue ) const;

  private:
    frantic::channels::channel_accessor<vec3> m_posAccessor;
    frantic::channels::channel_const_cvt_accessor<float> m_densityAccessor;
};

inline staggered_density_accessor::staggered_density_accessor(
    const frantic::channels::channel_accessor<vec3>& posAccessor,
    const frantic::channels::channel_const_cvt_accessor<float>& densityAccessor )
    : m_posAccessor( posAccessor )
    , m_densityAccessor( densityAccessor ) {}

inline void staggered_density_accessor::operator()( const char* pParticle, vec3& outPos, vec3& outValue ) const {
    outPos = m_posAccessor( pParticle );
    outValue.set( m_densityAccessor.get( pParticle ) );
}

class discretized_particle_field : public frantic::volumetrics::field_interface {
  public:
    discretized_particle_field( const int ( &bounds )[6], float spacing, const frantic::channels::channel_map& outMap );

    virtual ~discretized_particle_field();

    virtual const frantic::channels::channel_map& get_channel_map() const;

    virtual bool evaluate_field( void* dest, const frantic::graphics::vector3f& pos ) const;

    grid<float>& get_float_grid( const frantic::tstring& channelName );

    const grid<float>& get_float_grid( const frantic::tstring& channelName ) const;

    grid<vec3>& get_vec3_grid( const frantic::tstring& channelName );

    const grid<vec3>& get_vec3_grid( const frantic::tstring& channelName ) const;

  private:
    std::map<frantic::tstring, std::pair<frantic::channels::channel_accessor<float>, grid<float>>> m_floatStorage;
    std::map<frantic::tstring, std::pair<frantic::channels::channel_accessor<vec3>, grid<vec3>>> m_vectorStorage;

    std::pair<frantic::channels::channel_accessor<float>, grid<float>> m_densityStorage;

    frantic::channels::channel_map m_channelMap;

    float m_spacingFactor;
};

inline discretized_particle_field::discretized_particle_field( const int ( &bounds )[6], float spacing,
                                                               const frantic::channels::channel_map& outMap )
    : m_channelMap( outMap ) {
    grid<float>( bounds, spacing ).swap( m_densityStorage.second );

    for( std::size_t i = 0, iEnd = m_channelMap.channel_count(); i < iEnd; ++i ) {
        const frantic::channels::channel& ch = m_channelMap[i];
        if( ch.name() == _T("Density") ) {
            m_densityStorage.first = frantic::channels::channel_accessor<float>( ch.offset() );
        } else if( ch.data_type() == frantic::channels::data_type_float32 ) {
            if( ch.arity() == 1 ) {
                std::pair<frantic::channels::channel_accessor<float>, grid<float>>& channelData =
                    m_floatStorage[ch.name()];

                channelData.first = frantic::channels::channel_accessor<float>( ch.offset() );

                grid<float>( bounds, spacing ).swap( channelData.second );
            } else if( ch.arity() == 3 ) {
                std::pair<frantic::channels::channel_accessor<vec3>, grid<vec3>>& channelData =
                    m_vectorStorage[ch.name()];

                channelData.first = frantic::channels::channel_accessor<vec3>( ch.offset() );

                grid<vec3>( bounds, spacing ).swap( channelData.second );
            }
        }
    }

    m_spacingFactor = spacing * spacing * spacing;
}

discretized_particle_field::~discretized_particle_field() {}

const frantic::channels::channel_map& discretized_particle_field::get_channel_map() const { return m_channelMap; }

bool discretized_particle_field::evaluate_field( void* dest, const frantic::graphics::vector3f& pos ) const {
    float density = ember::trilerp( m_densityStorage.second, pos );

    if( density > 1e-5f ) {
        if( m_densityStorage.first.is_valid() )
            m_densityStorage.first.get( reinterpret_cast<char*>( dest ) ) = density / m_spacingFactor;

        for( std::map<frantic::tstring,
                      std::pair<frantic::channels::channel_accessor<float>, grid<float>>>::const_iterator
                 it = m_floatStorage.begin(),
                 itEnd = m_floatStorage.end();
             it != itEnd; ++it )
            it->second.first.get( reinterpret_cast<char*>( dest ) ) =
                ember::trilerp( it->second.second, pos ) / density;

        for( std::map<frantic::tstring,
                      std::pair<frantic::channels::channel_accessor<vec3>, grid<vec3>>>::const_iterator
                 it = m_vectorStorage.begin(),
                 itEnd = m_vectorStorage.end();
             it != itEnd; ++it )
            it->second.first.get( reinterpret_cast<char*>( dest ) ) =
                ember::trilerp( it->second.second, pos ) / density;
    } else {
        // Sets all memory to 0.
        m_channelMap.construct_structure( reinterpret_cast<char*>( dest ) );
    }

    return true;
}

grid<float>& discretized_particle_field::get_float_grid( const frantic::tstring& channelName ) {
    if( channelName == _T("Density") )
        return m_densityStorage.second;

    std::map<frantic::tstring, std::pair<frantic::channels::channel_accessor<float>, grid<float>>>::iterator it =
        m_floatStorage.find( channelName );

    assert( it != m_floatStorage.end() );

    return it->second.second;
}

const grid<float>& discretized_particle_field::get_float_grid( const frantic::tstring& channelName ) const {
    if( channelName == _T("Density") )
        return m_densityStorage.second;

    std::map<frantic::tstring, std::pair<frantic::channels::channel_accessor<float>, grid<float>>>::const_iterator it =
        m_floatStorage.find( channelName );

    assert( it != m_floatStorage.end() );

    return it->second.second;
}

grid<vec3>& discretized_particle_field::get_vec3_grid( const frantic::tstring& channelName ) {
    std::map<frantic::tstring, std::pair<frantic::channels::channel_accessor<vec3>, grid<vec3>>>::iterator it =
        m_vectorStorage.find( channelName );

    assert( it != m_vectorStorage.end() );

    return it->second.second;
}

const grid<vec3>& discretized_particle_field::get_vec3_grid( const frantic::tstring& channelName ) const {
    std::map<frantic::tstring, std::pair<frantic::channels::channel_accessor<vec3>, grid<vec3>>>::const_iterator it =
        m_vectorStorage.find( channelName );

    assert( it != m_vectorStorage.end() );

    return it->second.second;
}

class staggered_discretized_particle_field : public frantic::volumetrics::field_interface {
  public:
    staggered_discretized_particle_field( const int ( &bounds )[6], float spacing,
                                          const frantic::channels::channel_map& outMap );

    virtual ~staggered_discretized_particle_field();

    virtual const frantic::channels::channel_map& get_channel_map() const;

    virtual bool evaluate_field( void* dest, const frantic::graphics::vector3f& pos ) const;

    staggered_grid& get_staggered_grid( const frantic::tstring& channelName );

    const staggered_grid& get_staggered_grid( const frantic::tstring& channelName ) const;

  private:
    std::map<frantic::tstring, std::pair<frantic::channels::channel_accessor<vec3>, staggered_grid>> m_gridStorage;

    // staggered_grid m_densityGrid;

    frantic::channels::channel_map m_channelMap;

    float m_spacingFactor;
};

inline staggered_discretized_particle_field::staggered_discretized_particle_field(
    const int ( &bounds )[6], float spacing, const frantic::channels::channel_map& outMap )
    : m_channelMap( outMap ) {
    // staggered_grid( bounds, spacing ).swap( m_densityGrid );

    for( std::size_t i = 0, iEnd = m_channelMap.channel_count(); i < iEnd; ++i ) {
        const frantic::channels::channel& ch = m_channelMap[i];
        if( ch.data_type() == frantic::channels::data_type_float32 ) {
            if( ch.arity() != 3 ) {
                throw frantic::magma::magma_exception() << frantic::magma::magma_exception::error_name(
                    _T("Staggered type grids only support Vector3 channels for splatting") );
            } else {
                std::pair<frantic::channels::channel_accessor<vec3>, staggered_grid>& channelData =
                    m_gridStorage[ch.name()];

                channelData.first = frantic::channels::channel_accessor<vec3>( ch.offset() );

                staggered_grid( bounds, spacing ).swap( channelData.second );
            }
        }
    }

    m_spacingFactor = spacing * spacing * spacing;
}

inline staggered_discretized_particle_field::~staggered_discretized_particle_field() {}

inline const frantic::channels::channel_map& staggered_discretized_particle_field::get_channel_map() const {
    return m_channelMap;
}

inline bool staggered_discretized_particle_field::evaluate_field( void* dest,
                                                                  const frantic::graphics::vector3f& pos ) const {
    // vec3 density = ember::trilerp( m_densityGrid, pos );

    for( std::map<frantic::tstring,
                  std::pair<frantic::channels::channel_accessor<vec3>, staggered_grid>>::const_iterator
             it = m_gridStorage.begin(),
             itEnd = m_gridStorage.end();
         it != itEnd; ++it ) {
        it->second.first.get( reinterpret_cast<char*>( dest ) ) = ember::trilerp( it->second.second, pos );
        /*vec3 value = ember::trilerp( it->second.second, pos );
        if( density.x < 1e-5f )
                value.x = 0.f;
        else
                value.x /= density.x;

        if( density.y < 1e-5f )
                value.y = 0.f;
        else
                value.y /= density.y;

        if( density.z < 1e-5f )
                value.z = 0.f;
        else
                value.z /= density.z;

        it->second.first.get( reinterpret_cast<char*>( dest ) ) = value;*/
    }

    return true;
}

staggered_grid& staggered_discretized_particle_field::get_staggered_grid( const frantic::tstring& channelName ) {
    // if( channelName == _T("Density") )
    //	return m_densityGrid;

    std::map<frantic::tstring, std::pair<frantic::channels::channel_accessor<vec3>, staggered_grid>>::iterator it =
        m_gridStorage.find( channelName );

    assert( it != m_gridStorage.end() );

    return it->second.second;
}

const staggered_grid&
staggered_discretized_particle_field::get_staggered_grid( const frantic::tstring& channelName ) const {
    // if( channelName == _T("Density") )
    //	return m_densityGrid;

    std::map<frantic::tstring, std::pair<frantic::channels::channel_accessor<vec3>, staggered_grid>>::const_iterator
        it = m_gridStorage.find( channelName );

    assert( it != m_gridStorage.end() );

    return it->second.second;
}
} // namespace

int ParticleSplatNode::get_num_outputs() const { return static_cast<int>( this->get_channels().size() ); }

void ParticleSplatNode::get_output_description( int index, frantic::tstring& outDescription ) const {
    outDescription = this->get_channels()[index];
}

void ParticleSplatNode::compile_as_extension_type( frantic::magma::magma_compiler_interface& compiler ) {
    if( ember_compiler* bc2 = dynamic_cast<ember_compiler*>( &compiler ) ) {
        ember_compiler::field_ptr pField = bc2->find_cached_field( this->get_id() );
        if( !pField ) {
            frantic::graphics::boundbox3f boundBox;
            float spacing = 1.f;

            if( !bc2->get_context_data().get_property( _T("Bounds"), boundBox ) )
                boundBox.set_to_empty();
            if( this->get_spacingType() == _T("Absolute") ) {
                spacing = this->get_absoluteSpacing();
            } else if( this->get_spacingType() == _T("Relative") ) {
                if( !compiler.get_context_data().get_property( _T("Spacing"), spacing ) )
                    spacing = 1.f;

                spacing *= this->get_relativeSpacing();
            } else {
                THROW_MAGMA_INTERNAL_ERROR( frantic::strings::to_string( this->get_spacingType() ) );
            }

            const frantic::graphics::vector3f& boundsMin = boundBox.minimum();
            const frantic::graphics::vector3f& boundsMax = boundBox.maximum();

            int bounds[] = {
                (int)std::floor( boundsMin.x / spacing - 0.5f ), (int)std::ceil( boundsMax.x / spacing - 0.5f ) + 1,
                (int)std::floor( boundsMin.y / spacing - 0.5f ), (int)std::ceil( boundsMax.y / spacing - 0.5f ) + 1,
                (int)std::floor( boundsMin.z / spacing - 0.5f ), (int)std::ceil( boundsMax.z / spacing - 0.5f ) + 1,
            };

            frantic::magma::nodes::magma_input_particles_interface* pi =
                bc2->get_particles_interface( this->get_input( 0 ).first, this->get_input( 0 ).second );
            frantic::magma::nodes::magma_input_particles_interface::const_particle_array_ptr particles =
                pi->get_particles();

            if( particles->get_channel_map().channel_count() == 0 )
                THROW_MAGMA_INTERNAL_ERROR( this->get_id(), _T("Particles have no channels") );

            frantic::channels::channel_accessor<vec3> posAccessor =
                particles->get_channel_map().get_accessor<vec3>( _T("Position") );
            frantic::channels::channel_const_cvt_accessor<float> densityAccessor( 1.f );

            if( particles->get_channel_map().has_channel( _T("Density") ) )
                densityAccessor = particles->get_channel_map().get_const_cvt_accessor<float>( _T("Density") );

            frantic::channels::channel_map fieldMap;
            fieldMap.end_channel_definition();

            for( std::vector<frantic::tstring>::const_iterator it = this->get_channels().begin(),
                                                               itEnd = this->get_channels().end();
                 it != itEnd; ++it ) {
                if( !particles->get_channel_map().has_channel( *it ) ) {
                    if( *it != _T("Density") )
                        throw frantic::magma::magma_exception()
                            << frantic::magma::magma_exception::node_id( this->get_id() )
                            << frantic::magma::magma_exception::property_name( _T("channels") )
                            << frantic::magma::magma_exception::input_index( 0 )
                            << frantic::magma::magma_exception::connected_id( this->get_input( 0 ).first )
                            << frantic::magma::magma_exception::connected_output_index( this->get_input( 0 ).second )
                            << frantic::magma::magma_exception::error_name( _T("The particles do not have a \"") + *it +
                                                                            _T("\" channel") );

                    fieldMap.append_channel( _T("Density"), 1, frantic::channels::data_type_float32 );
                } else {
                    const frantic::channels::channel& ch = particles->get_channel_map()[*it];

                    if( !frantic::channels::is_channel_data_type_float( ch.data_type() ) ||
                        ( ch.arity() != 1 && ch.arity() != 3 ) ) {
                        frantic::magma::magma_data_type foundType;
                        foundType.m_elementType = ch.data_type();
                        foundType.m_elementCount = ch.arity();

                        throw frantic::magma::magma_exception()
                            << frantic::magma::magma_exception::node_id( this->get_id() )
                            << frantic::magma::magma_exception::input_index( 0 )
                            << frantic::magma::magma_exception::connected_id( this->get_input( 0 ).first )
                            << frantic::magma::magma_exception::connected_output_index( this->get_input( 0 ).second )
                            << frantic::magma::magma_exception::found_type( foundType )
                            << frantic::magma::magma_exception::error_name(
                                   _T("The particles \"") + *it +
                                   _T("\" channel is not a supported type. Only Float and Vector channels are ")
                                   _T("supported.") );
                    }

                    fieldMap.append_channel( *it, ch.arity(), frantic::channels::data_type_float32 );
                }
            }

            if( this->get_autoBounds() ) {
                int voxelCoord[3];

                bounds[0] = bounds[2] = bounds[4] = INT_MAX;
                bounds[1] = bounds[3] = bounds[5] = INT_MIN;

                for( frantic::particles::particle_array::const_iterator it = particles->begin(),
                                                                        itEnd = particles->end();
                     it != itEnd; ++it ) {
                    ember::float_to_integer_coord( posAccessor.get( *it ), spacing, voxelCoord );

                    if( voxelCoord[0] < bounds[0] )
                        bounds[0] = voxelCoord[0];
                    if( voxelCoord[0] >= bounds[1] )
                        bounds[1] = voxelCoord[0];

                    if( voxelCoord[1] < bounds[2] )
                        bounds[2] = voxelCoord[1];
                    if( voxelCoord[1] >= bounds[3] )
                        bounds[3] = voxelCoord[1];

                    if( voxelCoord[2] < bounds[4] )
                        bounds[4] = voxelCoord[2];
                    if( voxelCoord[2] >= bounds[5] )
                        bounds[5] = voxelCoord[2];
                }

                const int radius = 1; // If this isn't constant, we could use the max splat radius.
                const int boundsPadding = this->get_boundsPadding();

                bounds[0] -=
                    radius - 1 + boundsPadding; // -1 because we left justify the int coordinate (ie. 5.345 becomes 5)
                bounds[2] -= radius - 1 + boundsPadding;
                bounds[4] -= radius - 1 + boundsPadding;

                bounds[1] += radius + 1 + boundsPadding; // +1 beccause we use half-open intervals.
                bounds[3] += radius + 1 + boundsPadding;
                bounds[5] += radius + 1 + boundsPadding;
            }

            boost::shared_ptr<discretized_particle_field> gridField(
                new discretized_particle_field( bounds, spacing, fieldMap ) );

            ember::splat_particles(
                gridField->get_float_grid( _T("Density") ), particles->begin(), particles->end(),
                particle_accessor<float>( posAccessor, densityAccessor,
                                          frantic::channels::channel_const_cvt_accessor<float>( 1.f ) ) );

            for( std::size_t i = 0, iEnd = fieldMap.channel_count(); i < iEnd; ++i ) {
                if( fieldMap[i].name() == _T("Density") )
                    continue;

                if( fieldMap[i].arity() == 1 ) {
                    frantic::channels::channel_const_cvt_accessor<float> dataAccessor =
                        particles->get_channel_map().get_const_cvt_accessor<float>( fieldMap[i].name() );

                    ember::splat_particles( gridField->get_float_grid( fieldMap[i].name() ), particles->begin(),
                                            particles->end(),
                                            particle_accessor<float>( posAccessor, dataAccessor, densityAccessor ) );
                } else {
                    frantic::channels::channel_const_cvt_accessor<vec3> dataAccessor =
                        particles->get_channel_map().get_const_cvt_accessor<vec3>( fieldMap[i].name() );

                    ember::splat_particles( gridField->get_vec3_grid( fieldMap[i].name() ), particles->begin(),
                                            particles->end(),
                                            particle_accessor<vec3>( posAccessor, dataAccessor, densityAccessor ) );
                }
            }

            pField = gridField;
        }

        bc2->compile_field( this->get_id(), pField, compiler.get_node_input( *this, 1 ) );
    } else {
        frantic::magma::nodes::magma_simple_operator<2>::compile_as_extension_type( compiler );
    }
}

MAGMA_DEFINE_TYPE( "PVelocitySplat", "Stoke", ParticleSplatVelocityNode )
MAGMA_INPUT( "Particles", boost::blank() )
MAGMA_INPUT( "Position", boost::blank() )
MAGMA_EXPOSE_PROPERTY( channelName, frantic::tstring )
MAGMA_EXPOSE_PROPERTY( absoluteSpacing, float )
MAGMA_EXPOSE_PROPERTY( relativeSpacing, float )
MAGMA_ENUM_PROPERTY( spacingType, "Absolute", "Relative" )
MAGMA_EXPOSE_PROPERTY( autoBounds, bool )
MAGMA_EXPOSE_PROPERTY( boundsPadding, int )
MAGMA_EXPOSE_PROPERTY( removeDivergence, bool )
MAGMA_EXPOSE_PROPERTY( solverTimestep, float )
MAGMA_DESCRIPTION( "Splats particles onto a grid. The grid is reconstructed with trilinear sampling." )
MAGMA_DEFINE_TYPE_END;

int ParticleSplatVelocityNode::get_num_outputs() const { return 2; }

void ParticleSplatVelocityNode::get_output_description( int index, frantic::tstring& outDescription ) const {
    if( index == 0 )
        outDescription = this->get_densityChannelName();
    else if( index == 1 )
        outDescription = this->get_channelName();
    else
        frantic::magma::nodes::magma_simple_operator<2>::get_output_description( index, outDescription );
}

void ParticleSplatVelocityNode::compile_as_extension_type( frantic::magma::magma_compiler_interface& compiler ) {
    if( ember_compiler* bc2 = dynamic_cast<ember_compiler*>( &compiler ) ) {
        ember_compiler::field_ptr pField = bc2->find_cached_field( this->get_id() );
        if( !pField ) {
            frantic::graphics::boundbox3f boundBox;
            float spacing = 1.f;

            if( !bc2->get_context_data().get_property( _T("Bounds"), boundBox ) )
                boundBox.set_to_empty();
            if( this->get_spacingType() == _T("Absolute") ) {
                spacing = this->get_absoluteSpacing();
            } else if( this->get_spacingType() == _T("Relative") ) {
                if( !compiler.get_context_data().get_property( _T("Spacing"), spacing ) )
                    spacing = 1.f;

                spacing *= this->get_relativeSpacing();
            } else {
                THROW_MAGMA_INTERNAL_ERROR( frantic::strings::to_string( this->get_spacingType() ) );
            }

            const frantic::graphics::vector3f& boundsMin = boundBox.minimum();
            const frantic::graphics::vector3f& boundsMax = boundBox.maximum();

            int bounds[] = {
                (int)std::floor( boundsMin.x / spacing - 0.5f ), (int)std::ceil( boundsMax.x / spacing - 0.5f ) + 1,
                (int)std::floor( boundsMin.y / spacing - 0.5f ), (int)std::ceil( boundsMax.y / spacing - 0.5f ) + 1,
                (int)std::floor( boundsMin.z / spacing - 0.5f ), (int)std::ceil( boundsMax.z / spacing - 0.5f ) + 1,
            };

            frantic::magma::nodes::magma_input_particles_interface* pi =
                bc2->get_particles_interface( this->get_input( 0 ).first, this->get_input( 0 ).second );
            frantic::magma::nodes::magma_input_particles_interface::const_particle_array_ptr particles =
                pi->get_particles();

            if( particles->get_channel_map().channel_count() == 0 )
                THROW_MAGMA_INTERNAL_ERROR( this->get_id(), _T("Particles have no channels") );

            frantic::channels::channel_accessor<vec3> posAccessor =
                particles->get_channel_map().get_accessor<vec3>( _T("Position") );
            frantic::channels::channel_const_cvt_accessor<float> densityAccessor( 1.f );
            frantic::channels::channel_const_cvt_accessor<vec3> dataAccessor;

            if( particles->get_channel_map().has_channel( this->get_densityChannelName() ) ) {
                const frantic::channels::channel& ch = particles->get_channel_map()[this->get_densityChannelName()];

                if( !frantic::channels::is_channel_data_type_float( ch.data_type() ) || ch.arity() != 1 ) {
                    frantic::magma::magma_data_type foundType;
                    foundType.m_elementType = ch.data_type();
                    foundType.m_elementCount = ch.arity();

                    throw frantic::magma::magma_exception()
                        << frantic::magma::magma_exception::node_id( this->get_id() )
                        << frantic::magma::magma_exception::input_index( 0 )
                        << frantic::magma::magma_exception::connected_id( this->get_input( 0 ).first )
                        << frantic::magma::magma_exception::connected_output_index( this->get_input( 0 ).second )
                        << frantic::magma::magma_exception::found_type( foundType )
                        << frantic::magma::magma_exception::error_name(
                               _T("The particles \"") + this->get_channelName() +
                               _T("\" channel is not a supported type. Only Float channels are supported as the splat ")
                               _T("density channel.") );
                }

                densityAccessor =
                    particles->get_channel_map().get_const_cvt_accessor<float>( this->get_densityChannelName() );
            }

            if( !particles->get_channel_map().has_channel( this->get_channelName() ) )
                throw frantic::magma::magma_exception()
                    << frantic::magma::magma_exception::node_id( this->get_id() )
                    << frantic::magma::magma_exception::property_name( _T("channels") )
                    << frantic::magma::magma_exception::input_index( 0 )
                    << frantic::magma::magma_exception::connected_id( this->get_input( 0 ).first )
                    << frantic::magma::magma_exception::connected_output_index( this->get_input( 0 ).second )
                    << frantic::magma::magma_exception::error_name( _T("The particles do not have a \"") +
                                                                    this->get_channelName() + _T("\" channel") );

            const frantic::channels::channel& ch = particles->get_channel_map()[this->get_channelName()];

            if( !frantic::channels::is_channel_data_type_float( ch.data_type() ) || ch.arity() != 3 ) {
                frantic::magma::magma_data_type foundType;
                foundType.m_elementType = ch.data_type();
                foundType.m_elementCount = ch.arity();

                throw frantic::magma::magma_exception()
                    << frantic::magma::magma_exception::node_id( this->get_id() )
                    << frantic::magma::magma_exception::input_index( 0 )
                    << frantic::magma::magma_exception::connected_id( this->get_input( 0 ).first )
                    << frantic::magma::magma_exception::connected_output_index( this->get_input( 0 ).second )
                    << frantic::magma::magma_exception::found_type( foundType )
                    << frantic::magma::magma_exception::error_name(
                           _T("The particles \"") + this->get_channelName() +
                           _T("\" channel is not a supported type. Only Vector channels are supported.") );
            }

            dataAccessor = particles->get_channel_map().get_const_cvt_accessor<vec3>( this->get_channelName() );

            frantic::channels::channel_map fieldMap;
            fieldMap.define_channel( this->get_densityChannelName(), 3, frantic::channels::data_type_float32, 0u );
            fieldMap.define_channel( ch.name(), 3, frantic::channels::data_type_float32, 3 * sizeof( float ) );
            fieldMap.end_channel_definition( 4, true, false );

            if( this->get_autoBounds() ) {
                int voxelCoord[3];

                bounds[0] = bounds[2] = bounds[4] = INT_MAX;
                bounds[1] = bounds[3] = bounds[5] = INT_MIN;

                for( frantic::particles::particle_array::const_iterator it = particles->begin(),
                                                                        itEnd = particles->end();
                     it != itEnd; ++it ) {
                    ember::float_to_integer_coord( posAccessor.get( *it ), spacing, voxelCoord );

                    if( voxelCoord[0] < bounds[0] )
                        bounds[0] = voxelCoord[0];
                    if( voxelCoord[0] >= bounds[1] )
                        bounds[1] = voxelCoord[0];

                    if( voxelCoord[1] < bounds[2] )
                        bounds[2] = voxelCoord[1];
                    if( voxelCoord[1] >= bounds[3] )
                        bounds[3] = voxelCoord[1];

                    if( voxelCoord[2] < bounds[4] )
                        bounds[4] = voxelCoord[2];
                    if( voxelCoord[2] >= bounds[5] )
                        bounds[5] = voxelCoord[2];
                }

                const int radius = 1; // If this isn't constant, we could use the max splat radius.
                const int boundsPadding = this->get_boundsPadding();

                bounds[0] -=
                    radius - 1 + boundsPadding; // -1 because we left justify the int coordinate (ie. 5.345 becomes 5)
                bounds[2] -= radius - 1 + boundsPadding;
                bounds[4] -= radius - 1 + boundsPadding;

                bounds[1] += radius + 1 + boundsPadding; // +1 beccause we use half-open intervals.
                bounds[3] += radius + 1 + boundsPadding;
                bounds[5] += radius + 1 + boundsPadding;
            }

            boost::shared_ptr<staggered_discretized_particle_field> gridField(
                new staggered_discretized_particle_field( bounds, spacing, fieldMap ) );

            ember::splat_particles( gridField->get_staggered_grid( this->get_densityChannelName() ), particles->begin(),
                                    particles->end(), staggered_density_accessor( posAccessor, densityAccessor ) );
            ember::splat_particles( gridField->get_staggered_grid( this->get_channelName() ), particles->begin(),
                                    particles->end(),
                                    particle_accessor<vec3>( posAccessor, dataAccessor, densityAccessor ) );

            // TODO: We could perform this normalization lazily (ie. when trilerping) ...
            ember::normalize_grid_from_weights( gridField->get_staggered_grid( this->get_channelName() ),
                                                gridField->get_staggered_grid( this->get_densityChannelName() ) );

            if( this->get_removeDivergence() )
                do_poisson_solve( gridField->get_staggered_grid( this->get_channelName() ),
                                  this->get_solverTimestep() );

            pField = gridField;
        }

        bc2->compile_field( this->get_id(), pField, compiler.get_node_input( *this, 1 ) );

    } else {
        frantic::magma::nodes::magma_simple_operator<2>::compile_as_extension_type( compiler );
    }
}

} // namespace nodes
} // namespace ember
