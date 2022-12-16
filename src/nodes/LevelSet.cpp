// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#include "stdafx.h"

#include <ember/ember_compiler.hpp>
#include <ember/nodes/LevelSet.hpp>

#include <frantic/magma/nodes/magma_node_impl.hpp>
#include <frantic/volumetrics/levelset/geometry_to_levelset.hpp>

namespace ember {
namespace nodes {

MAGMA_DEFINE_TYPE( "LevelSet", "Stoke", LevelSetNode )
MAGMA_EXPOSE_PROPERTY( explicitSpacing, float )
MAGMA_EXPOSE_PROPERTY( relativeSpacing, float )
MAGMA_ENUM_PROPERTY( spacingType, "Absolute", "Relative" )
MAGMA_EXPOSE_PROPERTY( exposeGradient, bool )
MAGMA_EXPOSE_PROPERTY( fillVolume, bool )
MAGMA_EXPOSE_PROPERTY( bandWidthVoxels, int )
MAGMA_INPUT( "Geometry", boost::blank() )
MAGMA_INPUT( "Position", boost::blank() )
MAGMA_OUTPUT_NAMES( "Distance", "Gradient" )
MAGMA_DESCRIPTION( "Converts geometry to a signed distance field. Negative values are inside. The implementation "
                   "currently provides a thin band around the surface as well as the entire interior of the solid." )
MAGMA_DEFINE_TYPE_END;

LevelSetNode::LevelSetNode()
    : m_explicitSpacing( 1.f )
    , m_relativeSpacing( 1.f )
    , m_spacingType( _T("Relative") ) {
    this->set_exposeGradient( false );
    this->set_fillVolume( false );
    this->set_bandWidthVoxels( 5 );

    m_cachedBandWidth = -1;
    m_cachedFill = false;
}

int LevelSetNode::get_num_outputs() const { return this->get_exposeGradient() ? 2 : 1; }

void LevelSetNode::compile_as_extension_type( frantic::magma::magma_compiler_interface& compiler ) {
    frantic::magma::nodes::magma_input_geometry_interface* geom =
        compiler.get_geometry_interface( this->get_input( 0 ).first, this->get_input( 0 ).second );
    if( !geom || geom->size() < 1 )
        THROW_MAGMA_INTERNAL_ERROR();

    // frantic::magma::magma_geometry_ptr inMesh = geom->get_geometry( 0 );
    ////if( !inMesh )
    //	THROW_MAGMA_INTERNAL_ERROR();

    float spacing;
    if( this->get_spacingType() == _T("Absolute") ) {
        spacing = this->get_explicitSpacing();
    } else if( this->get_spacingType() == _T("Relative") ) {
        if( !compiler.get_context_data().get_property( _T("Spacing"), spacing ) )
            spacing = 1.f;

        spacing *= this->get_relativeSpacing();
    } else {
        THROW_MAGMA_INTERNAL_ERROR( frantic::strings::to_string( this->get_spacingType() ) );
    }

    int bandWidth = this->get_bandWidthVoxels();
    bool fillVolume = this->get_fillVolume();
    bool geomSame = geom->size() == m_cachedGeometry.size() && m_cachedGeometry.size() == m_cachedTMs.size();

    if( geomSame ) {
        // Make sure we are using the same meshes. If we have multiple meshes, we need to check their transforms since
        // we are making a world-space levelset in that case.
        for( std::size_t i = 0, iEnd = m_cachedGeometry.size(); i < iEnd && geomSame; ++i ) {
            frantic::magma::magma_geometry_ptr inMesh = geom->get_geometry( i );
            if( !inMesh )
                THROW_MAGMA_INTERNAL_ERROR();

            geomSame = ( &m_cachedGeometry[i]->get_mesh() == &inMesh->get_mesh() ) &&
                       ( m_cachedGeometry.size() == 1 || m_cachedTMs[i] == inMesh->get_toworld_transform() );
        }
    }

    // TODO: We should develop an asynchronous architecture for computing this levelset.
    // TODO: We should develop a dependency graph that allows us to cache the levelset (or compile time data in general)
    // in a smarter manner.

    if( !geomSame || !m_cachedLevelSet || m_cachedLevelSet->get_voxel_coord_system().voxel_length() != spacing ||
        m_cachedBandWidth != bandWidth || m_cachedFill != fillVolume ) {
        frantic::volumetrics::voxel_coord_system vcs( frantic::graphics::vector3f(), spacing );

        m_cachedBandWidth = bandWidth;
        m_cachedFill = fillVolume;

        m_cachedLevelSet.reset( new frantic::volumetrics::levelset::rle_level_set( vcs ) );

        m_cachedGeometry.clear();
        m_cachedTMs.clear();

        geom->get_all_geometry( std::back_inserter( m_cachedGeometry ) );

        frantic::geometry::trimesh3 tempMesh; // Gotta adapt to the convert_geometry_to_levelset() interface.

        std::size_t totalVerts = 0, totalFaces = 0;
        for( std::vector<frantic::magma::magma_geometry_ptr>::const_iterator it = m_cachedGeometry.begin(),
                                                                             itEnd = m_cachedGeometry.end();
             it != itEnd; ++it ) {
            totalVerts += ( *it )->get_mesh().get_num_verts();
            totalFaces += ( *it )->get_mesh().get_num_faces();
        }

        tempMesh.set_vertex_count( totalVerts );
        tempMesh.set_face_count( totalFaces );

        // Combine all the geometry into a single mesh.
        // NOTE: Alternatively we could store multiple levelsets and do a union at eval-time. I dunno ...
        std::size_t vertBase = 0, faceBase = 0;
        for( std::vector<frantic::magma::magma_geometry_ptr>::const_iterator it = m_cachedGeometry.begin(),
                                                                             itEnd = m_cachedGeometry.end();
             it != itEnd; ++it ) {
            frantic::geometry::mesh_interface& mesh = ( *it )->get_mesh();
            const frantic::graphics::transform4f& tm = ( *it )->get_toworld_transform();

            if( m_cachedGeometry.size() == 1 ) {
                for( std::size_t i = 0, iEnd = mesh.get_num_verts(); i < iEnd; ++i )
                    mesh.get_vert( i, *( float( * )[3] ) & tempMesh.get_vertex( i + vertBase ).x );
            } else {
                for( std::size_t i = 0, iEnd = mesh.get_num_verts(); i < iEnd; ++i ) {
                    float vert[3];
                    mesh.get_vert( i, vert );
                    tempMesh.get_vertex( i + vertBase ) = tm * frantic::graphics::vector3f( vert );
                }
            }

            for( std::size_t i = 0, iEnd = mesh.get_num_faces(); i < iEnd; ++i ) {
                std::size_t f[3];
                mesh.get_face_vert_indices( i, f );
                tempMesh.get_face( i + faceBase )
                    .set( (int)f[0] + vertBase, (int)f[1] + vertBase, (int)f[2] + vertBase );
            }

            vertBase += mesh.get_num_verts();
            faceBase += mesh.get_num_faces();

            // Record the TM so we know if it changed.
            m_cachedTMs.push_back( tm );
        }

        static const float SQRT3 = 1.7320508075688772935274463415059f; // sqrt 3
        frantic::volumetrics::levelset::convert_geometry_to_levelset( tempMesh, -SQRT3, SQRT3, *m_cachedLevelSet );

        // Dilate the region around the the geometry so we have a reasonable band for doing distance queries.
        if( m_cachedBandWidth > 0 ) {
            frantic::volumetrics::levelset::rle_index_spec risDilated;
            risDilated.build_from_dilation( m_cachedLevelSet->get_rle_index_spec(), m_cachedBandWidth );

            if( m_cachedFill ) {
                // We want all the levelset values in the volume to be filled, so now we do that using extrapolation.
                frantic::volumetrics::levelset::rle_index_spec ris;
                ris.build_by_filling( risDilated );

                m_cachedLevelSet->switch_rle_index_spec_with_swap( ris, _T("Populated") );
            } else {
                m_cachedLevelSet->switch_rle_index_spec_with_swap( risDilated, _T("Populated") );
            }

            m_cachedLevelSet->reinitialize_signed_distance_from_populated( _T("Populated") );
            m_cachedLevelSet->erase_channel( _T("Populated") );
        } else if( m_cachedFill ) {
            frantic::volumetrics::levelset::rle_index_spec ris;
            ris.build_by_filling( m_cachedLevelSet->get_rle_index_spec() );

            m_cachedLevelSet->switch_rle_index_spec_with_swap( ris, _T("Populated") );

            m_cachedLevelSet->reinitialize_signed_distance_from_populated( _T("Populated") );
            m_cachedLevelSet->erase_channel( _T("Populated") );
        }
    }

    if( ember_compiler* bc2 = dynamic_cast<ember_compiler*>( &compiler ) ) {
        // Our levelset is in world-space if we have multiple meshes. With a single mesh we keep the levelset in
        // object-space.
        frantic::graphics::transform4f tm;
        if( m_cachedGeometry.size() == 1 )
            tm = m_cachedGeometry.front()->get_toworld_transform();

        bc2->compile_levelset( this->get_id(), m_cachedLevelSet, tm, compiler.get_node_input( *this, 1 ),
                               this->get_exposeGradient() );
    } else {
        frantic::magma::nodes::magma_simple_operator<2>::compile_as_extension_type( compiler );
    }
}

} // namespace nodes
} // namespace ember
