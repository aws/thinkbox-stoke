// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <ember/async_field_task.hpp>
#include <ember/staggered_grid.hpp>

#include <frantic/graphics/boundbox3f.hpp>
#include <frantic/volumetrics/field_interface.hpp>

// Stupid 3ds Max SDK feels the need to use macros that conflict with OpenVDB.
#undef X_AXIS
#undef Y_AXIS
#undef Z_AXIS

#pragma warning( push, 3 )
#pragma warning( disable : 4800 4244 4146 4267 4355 4503 4996 )
#include <openvdb\OpenVDB.h>
#pragma warning( pop )

namespace ember {

class openvdb_field_interface : public frantic::volumetrics::field_interface {
  public:
    virtual ~openvdb_field_interface() {}

    // TODO: Add OpenVDB specific stuff here. Maybe expose some iterations patterns or something?
    virtual void get_grids( openvdb::GridCPtrVec& outGrids ) const = 0;

    virtual std::size_t get_memory_usage() const = 0;
};

class progress_logger_interrupter {
    frantic::logging::progress_logger* m_impl;

  public:
    progress_logger_interrupter( frantic::logging::progress_logger& logger )
        : m_impl( &logger ) {}

    void start( const char* name = NULL ) { m_impl->set_title( frantic::strings::to_tstring( name ) ); }

    void end() {}

    bool wasInterrupted( int percent = -1 ) {
        try {
            if( percent >= 0 ) {
                m_impl->update_progress( percent, 100 );
            } else {
                m_impl->check_for_abort();
            }
        } catch( const frantic::logging::progress_cancel_exception& ) {
            return false;
        }

        return true;
    }
};

std::unique_ptr<openvdb_field_interface> create_field_interface( const openvdb::GridCPtrVec& grids );
std::unique_ptr<openvdb_field_interface> create_field_interface( const openvdb::GridPtrVec& grids );
std::unique_ptr<openvdb_field_interface> create_field_interface( const openvdb::GridBase::ConstPtr& grid );

struct openvdb_meta {
    frantic::channels::channel_map map;
    frantic::graphics::boundbox3f bounds;
    float spacing;
    std::size_t memoryUsage;
};

future_field_base::ptr_type create_field_interface_from_file( const frantic::tstring& path );
future_field_base::ptr_type create_field_interface_from_file( const frantic::tstring& path, openvdb_meta& outMetadata );

void write_openvdb_file( const frantic::tstring& path,
                         const boost::shared_ptr<frantic::volumetrics::field_interface>& pField,
                         const frantic::graphics::boundbox3f& bounds, float spacing );

// Returns an index space bounding box (inclusive on either end) that fits outside the worldspace bounding box specified
// by the two vectors
template <class VectorType>
inline openvdb::CoordBBox get_ijk_outer_bounds( const VectorType& pmin, const VectorType& pmax, float spacing ) {
    openvdb::Coord ijkMin( static_cast<openvdb::Int32>( std::floor( pmin.x / spacing + 0.5f ) ) -
                               1, // x < 0.5 -> -1, x >= 0.5 -> 0
                           static_cast<openvdb::Int32>( std::floor( pmin.y / spacing + 0.5f ) ) - 1,
                           static_cast<openvdb::Int32>( std::floor( pmin.z / spacing + 0.5f ) ) - 1 );
    openvdb::Coord ijkMax(
        static_cast<openvdb::Int32>( std::ceil( pmax.x / spacing - 0.5f ) ), // x <= 0.5 -> 0, x > 0.5 -> 1
        static_cast<openvdb::Int32>( std::ceil( pmax.y / spacing - 0.5f ) ),
        static_cast<openvdb::Int32>( std::ceil( pmax.z / spacing - 0.5f ) ) );

    return openvdb::CoordBBox( ijkMin, ijkMax );
}

// Returns an index space bounding box (inclusive on either end) that fits inside the worldspace bounding box specified
// by the two vectors
template <class VectorType>
inline openvdb::CoordBBox get_ijk_inner_bounds( const VectorType& pmin, const VectorType& pmax, float spacing ) {
    openvdb::Coord ijkMin(
        static_cast<openvdb::Int32>( std::ceil( pmin.x / spacing - 0.5f ) ), // x <= 0.5 -> 0, x > 0.5 -> 1
        static_cast<openvdb::Int32>( std::ceil( pmin.y / spacing - 0.5f ) ),
        static_cast<openvdb::Int32>( std::ceil( pmin.z / spacing - 0.5f ) ) );
    openvdb::Coord ijkMax( static_cast<openvdb::Int32>( std::floor( pmax.x / spacing + 0.5f ) ) -
                               1, // x < 0.5 -> -1, x >= 0.5 -> 0
                           static_cast<openvdb::Int32>( std::floor( pmax.y / spacing + 0.5f ) ) - 1,
                           static_cast<openvdb::Int32>( std::floor( pmax.z / spacing + 0.5f ) ) - 1 );

    return openvdb::CoordBBox( ijkMin, ijkMax );
}

/**
 * Converts a field (or field collection) to equivalent OpenVDB grids. Will avoid coverting fields that are already
 * OpenVDB grids.
 */
void convert_to_grids( openvdb::GridCPtrVec& outGrids,
                       const boost::shared_ptr<frantic::volumetrics::field_interface>& field,
                       const openvdb::math::Transform::Ptr& xform, const openvdb::CoordBBox& bounds );

// Don't use this
void create_grids( openvdb::GridPtrVec& outGrids,
                   const boost::shared_ptr<frantic::volumetrics::field_interface>& pField,
                   const openvdb::math::Transform::Ptr& xform, const openvdb::math::CoordBBox& bounds );

// Don't use this
// openvdb::GridBase::Ptr create_grid( const boost::shared_ptr<frantic::volumetrics::field_interface>& pField, const
// openvdb::math::Transform::Ptr& xform, const openvdb::math::CoordBBox& bounds );

// Don't use this
openvdb::Vec3fGrid::Ptr copy_from_dense( const staggered_grid& src );

} // namespace ember
