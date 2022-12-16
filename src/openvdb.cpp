// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#include "stdafx.h"

#include <ember/concatenated_field.hpp>
#include <ember/data_types.hpp>
#include <ember/openvdb.hpp>

#include <frantic/channels/named_channel_data.hpp>
#include <frantic/diagnostics/profiling_section.hpp>
#include <frantic/logging/logging_level.hpp>

#pragma warning( push, 3 )
#pragma warning( disable : 4800 4244 4146 4267 4355 4503 )
#include <openvdb\OpenVDB.h>
#include <openvdb\io\File.h>
#include <openvdb\tools\Dense.h>
#include <openvdb\tools\Interpolation.h>
#pragma warning( pop )

#pragma warning( disable : 4503 ) // "decorated name length exceeded, name was truncated" casued by OpenVDB template
                                  // instantiations.

#include <tbb/blocked_range3d.h>
#include <tbb/concurrent_vector.h>
#include <tbb/parallel_reduce.h>
#include <tbb/tbb_allocator.h>

#include <boost/make_shared.hpp>

#include <iterator>

#include <frantic/misc/hybrid_assert.hpp>

namespace {

template <class TDest, class TSrc>
struct cvt {
    inline static void apply( void* pDest, const void* pSrc ) {
        static_cast<TDest*>( pDest )[0] = static_cast<TDest>( static_cast<const TSrc*>( pSrc )[0] );
    }
};

template <class T>
struct cvt<T, T> {
    inline static void apply( void* pDest, const void* pSrc ) {
        static_cast<T*>( pDest )[0] = static_cast<const T*>( pSrc )[0];
    }
};

template <class T>
struct cvt<half, T> {
    inline static void apply( void* pDest, const void* pSrc ) {
        static_cast<half*>( pDest )[0] = static_cast<float>( static_cast<const T*>( pSrc )[0] );
    }
};

template <class T>
struct cvt<ember::vec3, openvdb::math::Vec3<T>> {
    inline static void apply( void* pDest, const void* pSrc ) {
        static_cast<ember::vec3*>( pDest ).x =
            static_cast<float>( static_cast<const openvdb::math::Vec3<T>*>( pSrc ).x() );
        static_cast<ember::vec3*>( pDest ).y =
            static_cast<float>( static_cast<const openvdb::math::Vec3<T>*>( pSrc ).y() );
        static_cast<ember::vec3*>( pDest ).z =
            static_cast<float>( static_cast<const openvdb::math::Vec3<T>*>( pSrc ).z() );
    }
};

// TODO: Add more conversions!

struct fancy_cvt_interface {
    virtual ~fancy_cvt_interface() {}

    virtual const openvdb::GridBase& get_grid() const = 0;

    virtual openvdb::GridBase::ConstPtr get_grid_ptr() const = 0;

    virtual void apply( void*, const openvdb::Vec3d& p ) const = 0;
};

template <class TDest, class TSrcGrid, class SamplerType = openvdb::tools::BoxSampler>
class fancy_cvt : public fancy_cvt_interface {
    // Boost TSS claims to delete the data only once all the threads using are destroyed. This is a problem if we are
    // using a thread pool (perhaps via TBB) since those threads are not destroyed. We really want something similar
    // except the lifetime is controlled by the thread_specific_ptr object.
    /*struct accessor_data{
            typename TSrcGrid::Accessor accessor;
            openvdb::tools::GridSampler< typename TSrcGrid::Accessor, openvdb::tools::BoxSampler > sampler;

            accessor_data( TSrcGrid::Ptr grid )
                    : accessor( grid->getAccessor() ), sampler( accessor, grid->transform() )
            {}
    };

    boost::thread_specific_ptr<accessor_data> m_accessorData;*/
    typename TSrcGrid::ConstPtr m_grid;
    openvdb::tools::GridSampler<typename TSrcGrid::TreeType, SamplerType> m_sampler;

  private:
    fancy_cvt( const fancy_cvt& rhs );
    fancy_cvt& operator=( const fancy_cvt& rhs );

  public:
    fancy_cvt( typename TSrcGrid::ConstPtr grid )
        : m_grid( grid )
        , m_sampler(
              grid->tree(),
              grid->transform() ) // TODO: An accessor would perform better, but I need a way to make one per thread.
    {}

    virtual const openvdb::GridBase& get_grid() const { return *m_grid; }

    virtual openvdb::GridBase::ConstPtr get_grid_ptr() const { return m_grid; }

    virtual void apply( void* dest, const openvdb::Vec3d& p ) const {
        /*accessor_data* pAccessorData = m_accessorData.get();
        if( !pAccessorData )
                m_accessorData.reset( pAccessorData = new accessor_data( m_grid ) );

        static_cast<TDest*>( dest )[0] = pAccessorData->wsSample( p );*/

        static_cast<TDest*>( dest )[0] = static_cast<TDest>( m_sampler.wsSample( p ) );
    }
};

template <class TSrcGrid, class SamplerType>
class fancy_cvt<ember::vec3, TSrcGrid, SamplerType> : public fancy_cvt_interface {
    // Boost TSS claims to delete the data only once all the threads using are destroyed. This is a problem if we are
    // using a thread pool (perhaps via TBB) since those threads are not destroyed. We really want something similar
    // except the lifetime is controlled by the thread_specific_ptr object.
    /*struct accessor_data{
            typename TSrcGrid::Accessor accessor;
            openvdb::tools::GridSampler< typename TSrcGrid::Accessor, openvdb::tools::BoxSampler > sampler;

            accessor_data( TSrcGrid::Ptr grid )
                    : accessor( grid->getAccessor() ), sampler( accessor, grid->transform() )
            {}
    };

    boost::thread_specific_ptr<accessor_data> m_accessorData;*/
    typename TSrcGrid::ConstPtr m_grid;
    openvdb::tools::GridSampler<typename TSrcGrid::TreeType, openvdb::tools::BoxSampler> m_sampler;

  private:
    fancy_cvt( const fancy_cvt& rhs );
    fancy_cvt& operator=( const fancy_cvt& rhs );

  public:
    fancy_cvt( typename TSrcGrid::ConstPtr grid )
        : m_grid( grid )
        , m_sampler(
              grid->tree(),
              grid->transform() ) // TODO: An accessor would perform better, but I need a way to make one per thread.
    {}

    virtual const openvdb::GridBase& get_grid() const { return *m_grid; }

    virtual openvdb::GridBase::ConstPtr get_grid_ptr() const { return m_grid; }

    virtual void apply( void* dest, const openvdb::Vec3d& p ) const {
        /*accessor_data* pAccessorData = m_accessorData.get();
        if( !pAccessorData )
                m_accessorData.reset( pAccessorData = new accessor_data( m_grid ) );

        static_cast<TDest*>( dest )[0] = pAccessorData->wsSample( p );*/

        TSrcGrid::ValueType result = ( m_sampler.wsSample( p ) );

        static_cast<ember::vec3*>( dest )->set( static_cast<float>( result.x() ), static_cast<float>( result.y() ),
                                                static_cast<float>( result.z() ) );
    }
};

} // namespace

namespace ember {

class openvdb_field : public openvdb_field_interface {
  public:
    openvdb_field( const openvdb::GridCPtrVec& grids );
    openvdb_field( const openvdb::GridPtrVec& grids );
    openvdb_field( const openvdb::GridBase::ConstPtr& grid );

    virtual ~openvdb_field();

    virtual bool evaluate_field( void* dest, const frantic::graphics::vector3f& pos ) const;

    virtual const frantic::channels::channel_map& get_channel_map() const;

    virtual void get_grids( openvdb::GridCPtrVec& outGrids ) const;

    virtual std::size_t get_memory_usage() const;

  private:
    template <class Iterator>
    void init( Iterator gridBegin, Iterator gridEnd );

  private:
    struct grid_binding {
        frantic::channels::channel_general_accessor acc;
        boost::shared_ptr<fancy_cvt_interface> gridAccess;
    };

    std::vector<grid_binding> m_grids;

    frantic::channels::channel_map m_map;
};

openvdb_field::openvdb_field( const openvdb::GridCPtrVec& grids ) { this->init( grids.begin(), grids.end() ); }

openvdb_field::openvdb_field( const openvdb::GridPtrVec& grids ) { this->init( grids.begin(), grids.end() ); }

openvdb_field::openvdb_field( const openvdb::GridBase::ConstPtr& grid ) { this->init( &grid, &grid + 1 ); }

openvdb_field::~openvdb_field() {}

template <class Iterator>
void openvdb_field::init( Iterator gridBegin, Iterator gridEnd ) {
    m_grids.reserve( std::distance( gridBegin, gridEnd ) );

    FF_LOG( debug ) << _T("openvdb_field::init() w/ ") << m_grids.size() << _T(" grids") << std::endl;

    for( Iterator it = gridBegin; it != gridEnd; ++it ) {
        grid_binding binding;

        frantic::tstring gridName = frantic::strings::to_tstring( ( *it )->getName() );

        FF_LOG( debug ) << _T("\tProcessing grid: \"") << gridName << _T("\" of type: ")
                        << frantic::strings::to_tstring( ( *it )->valueType() ) << std::endl;

        if( gridName.empty() )
            continue;

        if( ( *it )->isType<openvdb::FloatGrid>() ) {
            m_map.define_channel<float>( frantic::strings::to_tstring( ( *it )->getName() ) );

            binding.gridAccess.reset(
                new fancy_cvt<float, openvdb::FloatGrid>( openvdb::gridConstPtrCast<openvdb::FloatGrid>( *it ) ) );
        } else if( ( *it )->isType<openvdb::DoubleGrid>() ) {
            m_map.define_channel<float>( frantic::strings::to_tstring( ( *it )->getName() ) );

            binding.gridAccess.reset(
                new fancy_cvt<float, openvdb::DoubleGrid>( openvdb::gridConstPtrCast<openvdb::DoubleGrid>( *it ) ) );
        } else if( ( *it )->isType<openvdb::Vec3fGrid>() ) {
            m_map.define_channel<ember::vec3>( frantic::strings::to_tstring( ( *it )->getName() ) );

            if( ( *it )->getGridClass() == openvdb::GRID_STAGGERED ) {
                binding.gridAccess.reset(
                    new fancy_cvt<ember::vec3, openvdb::Vec3fGrid, openvdb::tools::StaggeredBoxSampler>(
                        openvdb::gridConstPtrCast<openvdb::Vec3fGrid>( *it ) ) );
            } else {
                binding.gridAccess.reset( new fancy_cvt<ember::vec3, openvdb::Vec3fGrid>(
                    openvdb::gridConstPtrCast<openvdb::Vec3fGrid>( *it ) ) );
            }
        } else if( ( *it )->isType<openvdb::Vec3dGrid>() ) {
            m_map.define_channel<ember::vec3>( frantic::strings::to_tstring( ( *it )->getName() ) );

            if( ( *it )->getGridClass() == openvdb::GRID_STAGGERED ) {
                binding.gridAccess.reset(
                    new fancy_cvt<ember::vec3, openvdb::Vec3dGrid, openvdb::tools::StaggeredBoxSampler>(
                        openvdb::gridConstPtrCast<openvdb::Vec3dGrid>( *it ) ) );
            } else {
                binding.gridAccess.reset( new fancy_cvt<ember::vec3, openvdb::Vec3dGrid>(
                    openvdb::gridConstPtrCast<openvdb::Vec3dGrid>( *it ) ) );
            }
        } else {
            // TODO: Support more datatypes.
            continue;
        }

        // m_tempSize = std::max( m_tempSize, binding.srcSize );

        m_grids.push_back( binding );
    }
    m_map.end_channel_definition();

    for( std::vector<grid_binding>::iterator it = m_grids.begin(), itEnd = m_grids.end(); it != itEnd; ++it )
        it->acc = m_map.get_general_accessor( frantic::strings::to_tstring( it->gridAccess->get_grid().getName() ) );
}

bool openvdb_field::evaluate_field( void* dest, const frantic::graphics::vector3f& pos ) const {
    // char* tempBuffer = static_cast<char*>( alloca( m_tempSize ) );

    if( m_grids.empty() )
        return false;

    openvdb::Vec3d p( pos.x, pos.y, pos.z );

    for( std::vector<grid_binding>::const_iterator it = m_grids.begin(), itEnd = m_grids.end(); it != itEnd; ++it )
        it->gridAccess->apply( it->acc.get_channel_data_pointer( static_cast<char*>( dest ) ), p );

    return true;
}

const frantic::channels::channel_map& openvdb_field::get_channel_map() const { return m_map; }

void openvdb_field::get_grids( openvdb::GridCPtrVec& outGrids ) const {
    outGrids.reserve( outGrids.size() + m_grids.size() );

    for( std::vector<grid_binding>::const_iterator it = m_grids.begin(), itEnd = m_grids.end(); it != itEnd; ++it )
        outGrids.push_back( it->gridAccess->get_grid_ptr() );
}

std::size_t openvdb_field::get_memory_usage() const {
    std::size_t result = 0;

    for( std::vector<grid_binding>::const_iterator it = m_grids.begin(), itEnd = m_grids.end(); it != itEnd; ++it )
        result += static_cast<std::size_t>( it->gridAccess->get_grid_ptr()->memUsage() );

    return result;
}

std::unique_ptr<openvdb_field_interface> create_field_interface( const openvdb::GridCPtrVec& grids ) {
    return std::unique_ptr<openvdb_field_interface>( new openvdb_field( grids ) );
}

std::unique_ptr<openvdb_field_interface> create_field_interface( const openvdb::GridPtrVec& grids ) {
    return std::unique_ptr<openvdb_field_interface>( new openvdb_field( grids ) );
}

std::unique_ptr<openvdb_field_interface> create_field_interface( const openvdb::GridBase::ConstPtr& grid ) {
    return std::unique_ptr<openvdb_field_interface>( new openvdb_field( grid ) );
}

namespace {
class openvdb_file_task {
    BOOST_MOVABLE_BUT_NOT_COPYABLE( openvdb_file_task );

    boost::shared_ptr<openvdb::io::File> m_pFile;

  public:
    openvdb_file_task( const frantic::tstring& path )
        : m_pFile( new openvdb::io::File( frantic::strings::to_string( path ) ) ) {
        m_pFile->open();
    }

    openvdb_file_task( BOOST_RV_REF( openvdb_file_task ) rhs )
        : m_pFile( boost::move( rhs.m_pFile ) ) {}

    openvdb_file_task& operator=( BOOST_RV_REF( openvdb_file_task ) rhs ) { m_pFile = boost::move( rhs.m_pFile ); }

    void populate_metadata( frantic::channels::channel_map& outMap, float& outSpacing,
                            frantic::graphics::boundbox3f& outBounds, std::size_t& outCost ) {
        openvdb::GridPtrVecPtr gridMetadata = m_pFile->readAllGridMetadata();

        if( frantic::logging::is_logging_debug() ) {
            openvdb::MetaMap::ConstPtr pFileMetadata = m_pFile->getMetadata();
            frantic::logging::debug << _T("Metadata for file: \"")
                                    << frantic::strings::to_tstring( m_pFile->filename() ) << _T("\"") << std::endl;
            for( openvdb::MetaMap::ConstMetaIterator it = pFileMetadata->beginMeta(), itEnd = pFileMetadata->endMeta();
                 it != itEnd; ++it )
                frantic::logging::debug << _T("\tName: \"") << frantic::strings::to_tstring( it->first )
                                        << _T("\"  Value: \"") << frantic::strings::to_tstring( it->second->str() )
                                        << std::endl;
        }

        outSpacing = std::numeric_limits<float>::quiet_NaN();
        outBounds.set_to_empty();
        outCost = 0;
        outMap.reset();

        for( openvdb::GridPtrVecCIter it = gridMetadata->begin(), itEnd = gridMetadata->end(); it != itEnd; ++it ) {
            if( frantic::logging::is_logging_debug() ) {
                frantic::logging::debug << _T("Metadata for File: \"")
                                        << frantic::strings::to_tstring( m_pFile->filename() ) << _T("\"  Grid: \"")
                                        << frantic::strings::to_tstring( ( *it )->getName() ) << _T("\"") << std::endl;
                for( openvdb::MetaMap::ConstMetaIterator itMeta = ( *it )->beginMeta(), itMetaEnd = ( *it )->endMeta();
                     itMeta != itMetaEnd; ++itMeta )
                    frantic::logging::debug << _T("\tName: \"") << frantic::strings::to_tstring( itMeta->first )
                                            << _T("\"  Value: \"")
                                            << frantic::strings::to_tstring( itMeta->second->str() ) << std::endl;
            }

            if( ( *it )->isType<openvdb::FloatGrid>() || ( *it )->isType<openvdb::DoubleGrid>() ) {
                outMap.define_channel<float>( frantic::strings::to_tstring( ( *it )->getName() ) );
            } else if( ( *it )->isType<openvdb::Vec3fGrid>() || ( *it )->isType<openvdb::Vec3dGrid>() ) {
                outMap.define_channel<ember::vec3>( frantic::strings::to_tstring( ( *it )->getName() ) );
            } else {
                continue;
            }

            openvdb::TypedMetadata<openvdb::Vec3i>::ConstPtr pmin =
                ( *it )->getMetadata<openvdb::TypedMetadata<openvdb::Vec3i>>( openvdb::GridBase::META_FILE_BBOX_MIN );
            openvdb::TypedMetadata<openvdb::Vec3i>::ConstPtr pmax =
                ( *it )->getMetadata<openvdb::TypedMetadata<openvdb::Vec3i>>( openvdb::GridBase::META_FILE_BBOX_MAX );
            openvdb::TypedMetadata<openvdb::Int64>::ConstPtr memusage =
                ( *it )->getMetadata<openvdb::TypedMetadata<openvdb::Int64>>( openvdb::GridBase::META_FILE_MEM_BYTES );

            if( pmin && pmax && memusage ) {
                openvdb::CoordBBox pbounds( openvdb::Coord( pmin->value() ), openvdb::Coord( pmax->value() ) );
                if( !pbounds.empty() ) {
                    openvdb::Vec3d pminWorld = ( *it )->transform().indexToWorld( pbounds.getStart() );
                    openvdb::Vec3d pmaxWorld = ( *it )->transform().indexToWorld( pbounds.getEnd() );

                    outBounds += ember::vec3( static_cast<float>( pminWorld.x() ), static_cast<float>( pminWorld.y() ),
                                              static_cast<float>( pminWorld.z() ) );
                    outBounds += ember::vec3( static_cast<float>( pmaxWorld.x() ), static_cast<float>( pmaxWorld.y() ),
                                              static_cast<float>( pmaxWorld.z() ) );
                }
                outCost += static_cast<std::size_t>( memusage->value() );
            } else {
                // We need to load the whole grid to determine its size and bounds.
                openvdb::GridBase::ConstPtr grid = m_pFile->readGrid( ( *it )->getName() );

                openvdb::CoordBBox box = ( *it )->evalActiveVoxelBoundingBox();
                openvdb::Vec3d pminWorld = ( *it )->transform().indexToWorld( box.min() );
                openvdb::Vec3d pmaxWorld = ( *it )->transform().indexToWorld( box.max() );
                outBounds += ember::vec3( static_cast<float>( pminWorld.x() ), static_cast<float>( pminWorld.y() ),
                                          static_cast<float>( pminWorld.z() ) );
                outBounds += ember::vec3( static_cast<float>( pmaxWorld.x() ), static_cast<float>( pmaxWorld.y() ),
                                          static_cast<float>( pmaxWorld.z() ) );
                outCost += static_cast<std::size_t>( grid->memUsage() );
            }

            openvdb::Vec3d voxelSize = ( *it )->voxelSize();
            double thisSpacing = static_cast<float>( voxelSize.x() );
            if( std::abs( voxelSize.y() ) < std::abs( thisSpacing ) )
                thisSpacing = voxelSize.y();
            if( std::abs( voxelSize.z() ) < std::abs( thisSpacing ) )
                thisSpacing = voxelSize.z();

            // Using greater than and a not to deal with the initial NaN value. We want the smallest voxel spacing.
            if( !( static_cast<float>( thisSpacing ) >= outSpacing ) )
                outSpacing = static_cast<float>( thisSpacing );
        }

        outMap.end_channel_definition();
    }

    boost::shared_ptr<frantic::volumetrics::field_interface>
    operator()( frantic::logging::progress_logger& progress ) const {
        openvdb::GridPtrVec grids;

        for( openvdb::io::File::NameIterator it = m_pFile->beginName(), itEnd = m_pFile->endName(); it != itEnd;
             ++it ) {
            progress.check_for_abort();

            grids.push_back( m_pFile->readGrid( *it ) );
        }

        m_pFile->close();

        return boost::shared_ptr<frantic::volumetrics::field_interface>( create_field_interface( grids ).release() );
    }
};
} // namespace

future_field_base::ptr_type create_field_interface_from_file( const frantic::tstring& path ) {
    openvdb_file_task task( path );

    return create_field_task( boost::move( task ) );
}

future_field_base::ptr_type create_field_interface_from_file( const frantic::tstring& path,
                                                              openvdb_meta& outMetadata ) {
    openvdb_file_task taskImpl( path );

    taskImpl.populate_metadata( outMetadata.map, outMetadata.spacing, outMetadata.bounds, outMetadata.memoryUsage );

    // It would be better to allow for cancellation of a file being loaded. At the very least we should have a maximum
    // number of threads associated with loading files at one time. Perhaps a thread-pool for file i/o?
    return create_field_task( boost::move( taskImpl ) );
}

void write_openvdb_file( const frantic::tstring& path,
                         const boost::shared_ptr<frantic::volumetrics::field_interface>& pField,
                         const frantic::graphics::boundbox3f& bounds, float spacing ) {
    if( !pField ) {
        throw std::runtime_error( "write_openvdb_file Error: pField is NULL" );
    }

    openvdb::GridCPtrVec grids;

    openvdb::CoordBBox coordBounds = ember::get_ijk_inner_bounds( bounds.minimum(), bounds.maximum(), spacing );
    openvdb::math::Transform::Ptr xform = openvdb::math::Transform::createLinearTransform( spacing );

    xform->postTranslate( openvdb::Vec3d( spacing / 2.f ) );

    ember::convert_to_grids( grids, pField, xform, coordBounds );

    assert( grids.size() == pField->get_channel_map().channel_count() );

    openvdb::io::File file( frantic::strings::to_string( path ) );

    // Use old default compression settings from OpenVDB 1.1.1, for the sake
    // of backward compatibility.
    // TODO: Add new compression type (COMPRESS_BLOSC) as an option.
    file.setCompression( openvdb::io::COMPRESS_ZIP | openvdb::io::COMPRESS_ACTIVE_MASK );

    file.write( grids );
    file.close();
}

namespace detail {
class grid_builder_interface {
  public:
    virtual ~grid_builder_interface() {}

    virtual openvdb::GridBase::Ptr get_grid() const = 0;

    virtual void begin_leaf( void*& leafPtr ) const = 0;

    // ijkLocal is in [0,0,0] - [sx, sy, sz]
    // data == NULL means there was no data at this location (ie. OpenVDB state is "Off")
    virtual void set_leaf_voxel( void* leafPtr, const openvdb::Coord& ijkLocal, const void* data ) = 0;

    virtual void end_leaf( void*& leafPtr, const openvdb::Coord& origin ) = 0;

    virtual void delete_leaf( void* leafPtr ) const = 0;

    virtual void finalize() = 0;
};

template <class GridT>
class grid_builder : public grid_builder_interface {
  public:
    typedef GridT grid_type;
    typedef typename grid_type::ValueType value_type;
    typedef typename grid_type::TreeType tree_type;
    typedef typename tree_type::LeafNodeType leaf_type;

  public:
    grid_builder( const value_type& backgroundValue, const value_type& tolerance, std::ptrdiff_t dataOffset,
                  std::size_t blockEstimate = 128 );

    virtual ~grid_builder() {}

    virtual openvdb::GridBase::Ptr get_grid() const;

    virtual void begin_leaf( void*& leafPtr ) const;

    virtual void set_leaf_voxel( void* leafPtr, const openvdb::Coord& ijkLocal, const void* data );

    virtual void end_leaf( void*& leafPtr, const openvdb::Coord& origin );

    virtual void delete_leaf( void* leafPtr ) const;

    virtual void finalize();

  private:
    typename grid_type::Ptr m_grid;

    struct leaf_data {
        openvdb::Coord origin;
        leaf_type* leaf;
        value_type tileValue;
        bool tileState;
    };

    tbb::concurrent_vector<leaf_data> m_leafData;

    value_type m_tolerance;

    std::ptrdiff_t m_offset;
};

template <class GridT>
grid_builder<GridT>::grid_builder( const value_type& backgroundValue, const value_type& tolerance,
                                   std::ptrdiff_t dataOffset, std::size_t blockEstimate )
    : m_tolerance( tolerance )
    , m_grid( GridT::create( backgroundValue ) )
    , m_offset( dataOffset ) {
    m_leafData.reserve( blockEstimate );
}

template <class GridT>
openvdb::GridBase::Ptr grid_builder<GridT>::get_grid() const {
    return m_grid;
}

template <class GridT>
void grid_builder<GridT>::finalize() {
    typename GridT::Accessor acc( m_grid->getAccessor() );
    for( tbb::concurrent_vector<leaf_data>::iterator it = m_leafData.begin(), itEnd = m_leafData.end(); it != itEnd;
         ++it ) {
        if( it->leaf ) {
            acc.addLeaf( it->leaf );
        } else if( it->tileState ) {
            acc.addTile( 1, it->origin, it->tileValue, true );
        }
    }

    m_grid->tree().prune( m_tolerance );
}

template <class GridT>
void grid_builder<GridT>::begin_leaf( void*& leafPtr ) const {
    if( !leafPtr )
        leafPtr = new leaf_type;

    // Set the leaf nodde back to uninitialized and having the background value.
    static_cast<leaf_type*>( leafPtr )->fill( m_grid->background(), false );
}

template <class GridT>
void grid_builder<GridT>::set_leaf_voxel( void* leafPtr, const openvdb::Coord& ijkLocal, const void* data ) {
    // if( data ){
    const value_type& dataValue =
        *static_cast<const value_type*>( static_cast<const void*>( static_cast<const char*>( data ) + m_offset ) );

    if( !openvdb::math::isApproxEqual( m_grid->background(), dataValue, m_tolerance ) ) //{
        static_cast<leaf_type*>( leafPtr )->setValueOn( ijkLocal, dataValue );
    //	}else{
    //		static_cast<leaf_type*>(leafPtr)->setValueOff( ijkLocal );
    //	}
    //}else{
    //	static_cast<leaf_type*>(leafPtr)->setValueOff( ijkLocal );
    //}
}

template <class GridT>
void grid_builder<GridT>::end_leaf( void*& leafPtr, const openvdb::Coord& origin ) {
    leaf_type* leaf = static_cast<leaf_type*>( leafPtr );

    assert( leaf != NULL );

    tbb::concurrent_vector<leaf_data>::iterator itNewLeaf = m_leafData.grow_by( 1 );

    // isConstant will set the tile flag & value as a side-effect.
    if( !leaf->isConstant( itNewLeaf->tileValue, itNewLeaf->tileState, m_tolerance ) ) {
        leaf->setOrigin( origin );

        itNewLeaf->leaf = leaf;
        itNewLeaf->origin = origin;
        itNewLeaf->tileState = false;

        leafPtr = NULL;
    } else {
        itNewLeaf->leaf = NULL;
        itNewLeaf->origin = origin;

        // We can reuse this leaf node for the next block.
    }
}

template <class GridT>
void grid_builder<GridT>::delete_leaf( void* leafPtr ) const {
    delete static_cast<leaf_type*>( leafPtr );
}

class grid_builder_facade {
  public:
    grid_builder_facade( const frantic::volumetrics::field_interface& field, const openvdb::CoordBBox& bounds,
                         openvdb::math::Transform::Ptr xform, double tolerance );

    void copy( openvdb::GridPtrVec& outGrids, bool serial = false );

  private:
    grid_builder_facade( const grid_builder_facade& );
    grid_builder_facade& operator=( const grid_builder_facade& rhs );

    void copy_impl( const tbb::blocked_range<std::size_t>& range ) const;
    void finalize_impl( const tbb::blocked_range<std::size_t>& range ) const;

    // Calculate the absolute coordinate of the origin of the specified leaf. Index 0 maps to m_leafBounds.min();
    openvdb::Coord::ValueType leaf_index_to_x( std::size_t index ) const;
    openvdb::Coord::ValueType leaf_index_to_y( std::size_t index ) const;
    openvdb::Coord::ValueType leaf_index_to_z( std::size_t index ) const;

  private:
    openvdb::CoordBBox m_bounds;
    openvdb::CoordBBox m_leafBounds; // Modified from m_bounds to align to leaf node boundaries. This means it is a
                                     // superset of m_bounds.
    std::size_t m_strideY,
        m_strideX; // How many leaf nodes are there in each of Y & X. Allows us to convert to/from leaf indices.

    openvdb::math::Transform::Ptr m_transform;
    double m_tolerance;

    const frantic::volumetrics::field_interface* m_field;
    std::vector<boost::shared_ptr<grid_builder_interface>> m_gridBuilders;
};

namespace {
const openvdb::Index LEAF_DIM = openvdb::FloatGrid::TreeType::LeafNodeType::DIM;
}

grid_builder_facade::grid_builder_facade( const frantic::volumetrics::field_interface& field,
                                          const openvdb::CoordBBox& bounds, openvdb::math::Transform::Ptr xform,
                                          double tolerance )
    : m_field( &field )
    , m_bounds( bounds )
    , m_transform( xform )
    , m_tolerance( tolerance ) {
    openvdb::Coord leafMin = ( bounds.min() & ~( LEAF_DIM - 1 ) ); // Round down to start of containing leaf
    openvdb::Coord leafMax =
        ( bounds.max() & ~( LEAF_DIM - 1 ) ).offsetBy( LEAF_DIM - 1 ); // Round up to end of containing leaf
    m_strideY = ( leafMax.z() - leafMin.z() + 1 ) / LEAF_DIM;
    m_strideX = m_strideY * ( ( leafMax.y() - leafMin.y() + 1 ) / LEAF_DIM );

    assert( leafMin.x() <= bounds.min().x() && leafMin.y() <= bounds.min().y() && leafMin.z() <= bounds.min().z() );
    assert( leafMax.x() >= bounds.max().x() && leafMax.y() >= bounds.max().y() && leafMax.z() >= bounds.max().z() );
    assert( ( leafMin.x() % LEAF_DIM ) == 0 && ( leafMin.y() % LEAF_DIM ) == 0 && ( leafMin.z() % LEAF_DIM ) == 0 );
    assert( ( ( leafMax.x() + 1 ) % LEAF_DIM ) == 0 && ( ( leafMax.y() + 1 ) % LEAF_DIM ) == 0 &&
            ( ( leafMax.z() + 1 ) % LEAF_DIM ) == 0 );
    assert( ( ( leafMin - leafMax.offsetBy( 1 ) ).x() % LEAF_DIM ) == 0 &&
            ( ( leafMin - leafMax.offsetBy( 1 ) ).y() % LEAF_DIM ) == 0 &&
            ( ( leafMin - leafMax.offsetBy( 1 ) ).z() % LEAF_DIM ) == 0 );

    m_leafBounds.reset( leafMin, leafMax );
}

inline openvdb::Coord::ValueType grid_builder_facade::leaf_index_to_x( std::size_t index ) const {
    return m_leafBounds.min().x() + static_cast<openvdb::Coord::ValueType>( LEAF_DIM * ( index / m_strideX ) );
}

inline openvdb::Coord::ValueType grid_builder_facade::leaf_index_to_y( std::size_t index ) const {
    return m_leafBounds.min().y() +
           static_cast<openvdb::Coord::ValueType>( LEAF_DIM * ( ( index % m_strideX ) / m_strideY ) );
}

inline openvdb::Coord::ValueType grid_builder_facade::leaf_index_to_z( std::size_t index ) const {
    return m_leafBounds.min().z() + static_cast<openvdb::Coord::ValueType>( LEAF_DIM * ( index % m_strideY ) );
}

void grid_builder_facade::copy( openvdb::GridPtrVec& outGrids, bool serial ) {
    // A heuristic for estimating the number of leaf nodes we will generate. I figure a good starting point is 1/16th of
    // the maximum number of leaf nodes given the bounding region. I then make sure to reserve for at least 128 leaf
    // nodes.
    std::size_t blockTotal = m_strideX * ( ( m_leafBounds.max().x() - m_leafBounds.min().x() + 1 ) / LEAF_DIM );
    std::size_t blockEstimate = std::max<std::size_t>( 128, blockTotal / 16 );
    std::size_t numChannels = m_field->get_channel_map().channel_count();

    // Make sure the last addressable leaf is in bounds.
    assert( this->leaf_index_to_x( blockTotal - 1 ) <= m_bounds.max().x() );
    assert( this->leaf_index_to_y( blockTotal - 1 ) <= m_bounds.max().y() );
    assert( this->leaf_index_to_z( blockTotal - 1 ) <= m_bounds.max().z() );

    assert( this->leaf_index_to_x( blockTotal - 1 ) == ( m_leafBounds.max().x() - LEAF_DIM + 1 ) );
    assert( this->leaf_index_to_y( blockTotal - 1 ) == ( m_leafBounds.max().y() - LEAF_DIM + 1 ) );
    assert( this->leaf_index_to_z( blockTotal - 1 ) == ( m_leafBounds.max().z() - LEAF_DIM + 1 ) );

    m_gridBuilders.reserve( numChannels );

    for( std::size_t i = 0; i < numChannels; ++i ) {
        const frantic::channels::channel& ch = m_field->get_channel_map()[i];
        if( ch.data_type() == frantic::channels::data_type_float32 ) {
            if( ch.arity() == 1 ) {
                boost::shared_ptr<grid_builder<openvdb::FloatGrid>> gridBuilder(
                    new grid_builder<openvdb::FloatGrid>( 0.f, static_cast<float>( m_tolerance ),
                                                          static_cast<std::ptrdiff_t>( ch.offset() ), blockEstimate ) );

                openvdb::GridBase::Ptr grid = gridBuilder->get_grid();
                grid->setTransform( m_transform );
                grid->setName( frantic::strings::to_string( ch.name() ) );
                if( ch.name() == _T("Density") )
                    grid->setGridClass( openvdb::GRID_FOG_VOLUME );

                m_gridBuilders.push_back( gridBuilder );
            } else if( ch.arity() == 3 ) {
                boost::shared_ptr<grid_builder<openvdb::Vec3fGrid>> gridBuilder( new grid_builder<openvdb::Vec3fGrid>(
                    openvdb::Vec3f( 0.f ), openvdb::Vec3f( static_cast<float>( m_tolerance ) ),
                    static_cast<std::ptrdiff_t>( ch.offset() ), blockEstimate ) );

                openvdb::GridBase::Ptr grid = gridBuilder->get_grid();
                grid->setTransform( m_transform );
                grid->setName( frantic::strings::to_string( ch.name() ) );
                if( ch.name() == _T("Velocity") ) {
                    // gridBuilder->set_grid_class( openvdb::GRID_STAGGERED );
                    grid->setVectorType( openvdb::VEC_CONTRAVARIANT_RELATIVE );
                } else if( ch.name() == _T("Normal") || ch.name() == _T("Tangent") ) {
                    grid->setVectorType( openvdb::VEC_COVARIANT_NORMALIZE );
                } else {
                    grid->setVectorType( openvdb::VEC_INVARIANT );
                }

                m_gridBuilders.push_back( gridBuilder );
            }
        }
    }

    if( !m_bounds.empty() ) {
        if( !serial ) {
            // Use bind so that we don't copy around the vectors held inside *this.
            tbb::parallel_for( tbb::blocked_range<std::size_t>( 0, blockTotal, 20 ),
                               boost::bind( &grid_builder_facade::copy_impl, this, _1 ) );
            tbb::parallel_for( tbb::blocked_range<std::size_t>( 0, m_gridBuilders.size() ),
                               boost::bind( &grid_builder_facade::finalize_impl, this, _1 ) );
        } else {
            this->copy_impl( tbb::blocked_range<std::size_t>( 0, blockTotal ) );
            this->finalize_impl( tbb::blocked_range<std::size_t>( 0, m_gridBuilders.size() ) );
        }
    }

    outGrids.reserve( outGrids.size() + m_gridBuilders.size() );
    for( std::vector<boost::shared_ptr<grid_builder_interface>>::iterator it = m_gridBuilders.begin(),
                                                                          itEnd = m_gridBuilders.end();
         it != itEnd; ++it )
        outGrids.push_back( ( *it )->get_grid() );
}

void grid_builder_facade::copy_impl( const tbb::blocked_range<std::size_t>& range ) const {
    // We need temporary storage of a leaf node for each grid_builder. They will be populated by passing to
    // grid_builder::begin_leaf().

    // std::vector< void* > leafNodes( m_gridBuilders.size(), NULL );
    // std::vector< void* >::iterator itLeaf;

    // Its cheaper to allocate on the stack compared to the heap. The heap was shown to be quite expensive when TBB
    // chose to make small ranges (ie. 1 element long) since we allocated and deallocated the temporary storage very
    // often.
    void** leafNodes = static_cast<void**>( alloca( sizeof( void* ) * m_gridBuilders.size() ) );
    void** itLeaf = &leafNodes[0];

    memset( leafNodes, 0, sizeof( void* ) * m_gridBuilders.size() );

    try {
        // Calculate the origin of the first leaf we will process. This is determined by interpreting the leaf index
        // appropriately.
        openvdb::Coord leafStart( this->leaf_index_to_x( range.begin() ), this->leaf_index_to_y( range.begin() ),
                                  this->leaf_index_to_z( range.begin() ) );

        char* buffer = static_cast<char*>( alloca( m_field->get_channel_map().structure_size() ) );

        // Iterate over the collection of leaf nodes (as defined by the contiguous indices) assigned to this
        // parallel_for body invocation.
        openvdb::Coord cur = leafStart;
        for( std::size_t i = range.begin(), iEnd = range.end(); i < iEnd; ++i, cur.z() += LEAF_DIM ) {
            if( cur.z() > m_bounds.max().z() ) {
                cur.z() = m_leafBounds.min().z();
                cur.y() += LEAF_DIM;
                if( cur.y() > m_bounds.max().y() ) {
                    cur.y() = m_leafBounds.min().y();
                    cur.x() += LEAF_DIM;
                    assert( cur.x() <= m_bounds.max().x() );
                }
            }

            // Allocate a leaf node for each grid we are constructing. We might be building a variety of grid types, so
            // we need a heterogenous collection. We can't store the current leaf node in the grid_builder itself since
            // we are building from multiple threads. Each thread gets its own leaf node that it passes to the grid
            // builder as appropriate.
            //
            // TODO: In retrospect there is no reason I couldn't make a thin wrapper around the leaf for each grid as a
            // separate object. It was simpler to keep the
            //       knowledge of the leaf nodes in the builder since there are a bunch of other parameters affecting
            //       node population that I would have to copy around.
            // itLeaf = leafNodes.begin();
            itLeaf = &leafNodes[0];
            for( std::vector<boost::shared_ptr<grid_builder_interface>>::const_iterator it = m_gridBuilders.begin(),
                                                                                        itEnd = m_gridBuilders.end();
                 it != itEnd; ++it, ++itLeaf )
                ( *it )->begin_leaf( *itLeaf );

            openvdb::Coord start = openvdb::Coord::maxComponent( m_bounds.min() - cur, openvdb::Coord( 0 ) );
            openvdb::Coord end = openvdb::Coord::minComponent( m_bounds.max() - cur, openvdb::Coord( LEAF_DIM - 1 ) );
            for( openvdb::Coord ijk = start; ijk.x() <= end.x(); ++ijk.x() ) {
                for( ijk.y() = start.y(); ijk.y() <= end.y(); ++ijk.y() ) {
                    for( ijk.z() = start.z(); ijk.z() <= end.z(); ++ijk.z() ) {
                        // TODO: We could evaluate this once per z-row and just increment the z-coordinate to speed it
                        // up. Assuming there aren't any really funky transforms.
                        openvdb::Vec3d p = m_transform->indexToWorld( ijk + cur );

                        if( m_field->evaluate_field( buffer, ember::vec3( static_cast<float>( p.x() ),
                                                                          static_cast<float>( p.y() ),
                                                                          static_cast<float>( p.z() ) ) ) ) {
                            // itLeaf = leafNodes.begin();
                            itLeaf = &leafNodes[0];
                            for( std::vector<boost::shared_ptr<grid_builder_interface>>::const_iterator
                                     it = m_gridBuilders.begin(),
                                     itEnd = m_gridBuilders.end();
                                 it != itEnd; ++it, ++itLeaf )
                                ( *it )->set_leaf_voxel( *itLeaf, ijk, buffer );
                        } // else{
                        //	//itLeaf = leafNodes.begin();
                        //	itLeaf = &leafNodes[0];
                        //	for( std::vector< boost::shared_ptr<grid_builder_interface> >::const_iterator it =
                        //m_gridBuilders.begin(), itEnd = m_gridBuilders.end(); it != itEnd; ++it, ++itLeaf )
                        //		(*it)->set_leaf_voxel( *itLeaf, ijk, NULL );
                        //}
                    } // for z in leaf
                }     // for y in leaf
            }         // for x in leaf

            // These leaf nodes are finished, so pass them to the grid builders.
            // itLeaf = leafNodes.begin();
            itLeaf = &leafNodes[0];
            for( std::vector<boost::shared_ptr<grid_builder_interface>>::const_iterator it = m_gridBuilders.begin(),
                                                                                        itEnd = m_gridBuilders.end();
                 it != itEnd; ++it, ++itLeaf )
                ( *it )->end_leaf( *itLeaf, cur );
        } // for i in range

        // Delete any leaf nodes we allocated but didn't use.
        // itLeaf = leafNodes.begin();
        itLeaf = &leafNodes[0];
        for( std::vector<boost::shared_ptr<grid_builder_interface>>::const_iterator it = m_gridBuilders.begin(),
                                                                                    itEnd = m_gridBuilders.end();
             it != itEnd; ++it, ++itLeaf )
            ( *it )->delete_leaf( *itLeaf );
    } catch( ... ) {
        // itLeaf = leafNodes.begin();
        itLeaf = &leafNodes[0];
        for( std::vector<boost::shared_ptr<grid_builder_interface>>::const_iterator it = m_gridBuilders.begin(),
                                                                                    itEnd = m_gridBuilders.end();
             it != itEnd; ++it, ++itLeaf )
            ( *it )->delete_leaf( *itLeaf );
        throw;
    }
}

void grid_builder_facade::finalize_impl( const tbb::blocked_range<std::size_t>& range ) const {
    for( std::vector<boost::shared_ptr<grid_builder_interface>>::const_iterator
             it = m_gridBuilders.begin() + range.begin(),
             itEnd = m_gridBuilders.begin() + range.end();
         it != itEnd; ++it )
        ( *it )->finalize();
}
} // namespace detail

// Import the name into the ember namespace
using detail::grid_builder_facade;

void create_grids( openvdb::GridPtrVec& outGrids,
                   const boost::shared_ptr<frantic::volumetrics::field_interface>& pField,
                   const openvdb::math::Transform::Ptr& xform, const openvdb::math::CoordBBox& bounds ) {
    grid_builder_facade op( *pField, bounds, xform, 1e-4 );

    frantic::diagnostics::profiling_section ps( _T("CopyFromField") );
    frantic::diagnostics::scoped_profile sp( ps );

    op.copy( outGrids, false );

    if( frantic::logging::is_logging_stats() ) {
        std::stringstream ssXForm, ssBounds;
        xform->print( ssXForm );
        ssBounds << bounds;

        frantic::logging::stats << ps << std::endl;
        frantic::logging::stats << _T("Field contained: \n") << pField->get_channel_map() << std::endl;
        frantic::logging::stats << _T("Resulting grids each have: ") << bounds.volume() << _T(" voxels in bounds: \n")
                                << frantic::strings::to_tstring( ssBounds.str() ) << std::endl;
        frantic::logging::stats << _T("Sample spacing was: ")
                                << frantic::strings::to_tstring( xform->voxelSize().str() ) << _T(" with transform: \n")
                                << frantic::strings::to_tstring( ssXForm.str() ) << std::endl;
    }
}

namespace {
template <typename TreeT>
class CopyFromDense {
  public:
    typedef typename TreeT::ValueType ValueT;
    typedef typename TreeT::LeafNodeType LeafT;

    CopyFromDense( const staggered_grid& dense, TreeT& tree, const ValueT& tolerance )
        : mDense( dense )
        , mTree( tree )
        , mBlocks( NULL )
        , mTolerance( tolerance ) {}

    /// @brief Copy values from the dense grid to the sparse tree.
    void copy( bool serial = false ) {
        std::vector<Block> blocks;
        mBlocks = &blocks;
        // const CoordBBox& bbox = mDense.bbox();
        // openvdb::CoordBBox bbox( Coord( 0,0,0 ), Coord( mDense.get_size(0) - 1, mDense.get_size(1) - 1,
        // m_dense.get_size(2) - 1 ) );
        openvdb::CoordBBox bbox(
            openvdb::Coord( mDense.get_min( 0 ), mDense.get_min( 1 ), mDense.get_min( 2 ) ),
            openvdb::Coord( mDense.get_max( 0 ) - 1, mDense.get_max( 1 ) - 1, mDense.get_max( 2 ) - 1 ) );

        // Pre-process: Construct a list of blocks alligned with (potential) leaf nodes
        for( openvdb::CoordBBox sub = bbox; sub.min()[0] <= bbox.max()[0]; sub.min()[0] = sub.max()[0] + 1 ) {
            for( sub.min()[1] = bbox.min()[1]; sub.min()[1] <= bbox.max()[1]; sub.min()[1] = sub.max()[1] + 1 ) {
                for( sub.min()[2] = bbox.min()[2]; sub.min()[2] <= bbox.max()[2]; sub.min()[2] = sub.max()[2] + 1 ) {
                    sub.max() = openvdb::Coord::minComponent(
                        bbox.max(), ( sub.min() & ( ~( LeafT::DIM - 1u ) ) ).offsetBy( LeafT::DIM - 1u ) );
                    blocks.push_back( Block( sub ) );
                }
            }
        }
        // Multi-threaded process: Convert dense grid into leaf nodes and tiles
        if( serial ) {
            ( *this )( tbb::blocked_range<size_t>( 0, blocks.size() ) );
        } else {
            tbb::parallel_for( tbb::blocked_range<size_t>( 0, blocks.size() ), *this );
        }
        // Post-process: Insert leaf nodes and tiles into the tree, and prune the tiles only!
        openvdb::tree::ValueAccessor<TreeT> acc( mTree );
        for( size_t m = 0, size = blocks.size(); m < size; ++m ) {
            Block& block = blocks[m];
            if( block.leaf ) {
                acc.addLeaf( block.leaf );
            } else if( block.tile.second ) {                                // only background tiles are inactive
                acc.addTile( 1, block.bbox.min(), block.tile.first, true ); // leaf tile
            }
        }
        mTree.prune( mTolerance );
    }

    /// @brief Public method called by tbb::parallel_for
    void operator()( const tbb::blocked_range<size_t>& r ) const {
        std::unique_ptr<LeafT> leaf;

        for( size_t m = r.begin(), n = 0, mEnd = r.end(); m != mEnd; ++m, ++n ) {
            if( !leaf.get() ) {
                leaf.reset( new LeafT );
                leaf->fill( mTree.background(), false ); // TODO: Figure out why this is necessary!!!!
            } else {
                leaf->setValuesOff();
            }

            Block& block = ( *mBlocks )[m];
            const openvdb::CoordBBox& bbox = block.bbox;
            const std::vector<vec3>& data = mDense.get_data();

            std::size_t index =
                bbox.min().x() - mDense.get_min( 0 ) +
                mDense.get_size( 0 ) * ( bbox.min().y() - mDense.get_min( 1 ) +
                                         mDense.get_size( 1 ) * ( bbox.min().z() - mDense.get_min( 2 ) ) );

            std::size_t strideY = mDense.get_size( 0 );
            std::size_t strideZ = mDense.get_size( 0 ) * mDense.get_size( 1 );

            std::vector<vec3>::const_iterator itBase = data.begin() + index;

            openvdb::Coord leafMin =
                bbox.min() &
                ~( LeafT::DIM - 1 ); // Since the target bounds might not start directly on a leaf boundary, we
                                     // calculate it here by rounding down to the nearest leaf multiple.
            openvdb::Coord start =
                bbox.min() -
                leafMin; // The offset of the start of the defined area, relative to the leaf's min. corner.
            openvdb::Coord end =
                bbox.max() - leafMin; // The offset of the end of the defined area, relative to the leaf's min. corner.

            // Since OpenVDB is organized with Z changing fastest, but our internal dense grids have X changing fastest
            // we need to pick one. By the theory that memory allocations in OpenVDB are in chunks (ie. Leafs) iterating
            // out of order in VDB should be less expensive in terms of cache if we can assume that minimizing the
            // memory address between neighbors is optimal. That means we iterate over X changing fastest to match our
            // dense grids.
            for( openvdb::Coord ijk( start ); ijk.z() <= end.z();
                 ++ijk.z(), ijk.setY( start.y() ), itBase += strideZ ) {
                std::vector<vec3>::const_iterator itYZ = itBase;
                for( ; ijk.y() <= end.y(); ++ijk.y(), ijk.setX( start.x() ), itYZ += strideY ) {
                    std::vector<vec3>::const_iterator it = itYZ;
                    for( ; ijk.x() <= end.x(); ++ijk.x(), ++it ) {
                        openvdb::Vec3f v( it->x, it->y, it->z );
                        if( openvdb::math::isApproxEqual( mTree.background(), v, mTolerance ) ) {
                            leaf->setValueOff( ijk, mTree.background() );
                        } else {
                            leaf->setValueOn( ijk, v );
                        }
                    }
                }
            }

            if( !leaf->isConstant( block.tile.first, block.tile.second, mTolerance ) ) {
                leaf->setOrigin( leafMin );
                block.leaf = leaf.release();
            }
        } // loop over blocks
    }

  private:
    // CopyFromDense( const CopyFromDense& rhs );
    CopyFromDense& operator=( const CopyFromDense& rhs );

    struct Block {
        openvdb::CoordBBox bbox;
        LeafT* leaf;
        std::pair<ValueT, bool> tile;
        Block( const openvdb::CoordBBox& b )
            : bbox( b )
            , leaf( NULL ) {}
    };

    // const Dense<ValueT>& mDense;
    const staggered_grid& mDense;
    TreeT& mTree;
    std::vector<Block>* mBlocks;
    ValueT mTolerance;
}; // CopyFromDense

} // namespace

openvdb::Vec3fGrid::Ptr copy_from_dense( const staggered_grid& src ) {
    openvdb::Vec3fGrid::Ptr result = openvdb::Vec3fGrid::create( openvdb::Vec3f( 0.f ) );

    openvdb::math::Transform::Ptr xform = openvdb::math::Transform::createLinearTransform( src.get_spacing() );

    result->setTransform( xform );

    typedef openvdb::TreeAdapter<openvdb::Vec3fGrid> Adapter;
    typedef Adapter::TreeType TreeT;

    CopyFromDense<TreeT> op( src, Adapter::tree( result->tree() ), openvdb::Vec3f( 1e-2f ) );
    op.copy( /*serial*/ false );

    return result;
}

void convert_to_grids( openvdb::GridCPtrVec& outGrids,
                       const boost::shared_ptr<frantic::volumetrics::field_interface>& field,
                       const openvdb::math::Transform::Ptr& xform, const openvdb::CoordBBox& bounds ) {
    if( boost::shared_ptr<ember::concatenated_field> fieldGroup =
            boost::dynamic_pointer_cast<ember::concatenated_field>( field ) ) {
        for( std::size_t i = 0, iEnd = fieldGroup->get_num_fields(); i < iEnd; ++i )
            convert_to_grids( outGrids, fieldGroup->get_field( i ), xform, bounds );
    } else if( boost::shared_ptr<ember::openvdb_field_interface> gridField =
                   boost::dynamic_pointer_cast<ember::openvdb_field_interface>( field ) ) {
        gridField->get_grids( outGrids );
    } else if( boost::shared_ptr<ember::staggered_discretized_field> staggeredField =
                   boost::dynamic_pointer_cast<ember::staggered_discretized_field>( field ) ) {
        openvdb::Vec3fGrid::Ptr result = ember::copy_from_dense( staggeredField->get_grid() );
        result->setName( frantic::strings::to_string( staggeredField->get_channel_map()[0].name() ) );
        result->setVectorType( openvdb::VEC_CONTRAVARIANT_RELATIVE );
        result->setGridClass( openvdb::GRID_STAGGERED );

        outGrids.push_back( result );

        // TODO: Why not non-staggered grids too?
    } else {
        openvdb::GridPtrVec nonConstGrids;

        // Samples a multi-channel field into multiple grids.
        ember::create_grids( nonConstGrids, field, xform, bounds );

        outGrids.insert( outGrids.end(), nonConstGrids.begin(), nonConstGrids.end() );
    }
}

} // namespace ember
