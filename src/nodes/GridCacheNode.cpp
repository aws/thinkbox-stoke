// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#include "stdafx.h"
#include <ember/ember_compiler.hpp>
#include <ember/nodes/GridCacheNode.hpp>

#include <frantic/magma/nodes/magma_node_impl.hpp>

#include <frantic/graphics/boundbox3f.hpp>

namespace ember {
namespace nodes {

MAGMA_DEFINE_TYPE( "Grid", "Stoke", GridCacheNode )
MAGMA_INPUT_BEGIN( "Field" )
MAGMA_INPUT_ATTR( visitable, false )
MAGMA_INPUT_END
MAGMA_INPUT( "Position", boost::blank() )
MAGMA_EXPOSE_PROPERTY( absoluteSpacing, float )
MAGMA_EXPOSE_PROPERTY( relativeSpacing, float )
MAGMA_ENUM_PROPERTY( spacingType, "Absolute", "Relative" )
MAGMA_DESCRIPTION( "Samples the incoming field at grid points and uses trilinear interpolation between samples." )
MAGMA_DEFINE_TYPE_END;

void GridCacheNode::compile_as_extension_type( frantic::magma::magma_compiler_interface& compiler ) {
    if( ember_compiler* bc2 = dynamic_cast<ember_compiler*>( &compiler ) ) {
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

        if( this->get_input( 0 ).first == frantic::magma::magma_interface::INVALID_ID &&
            ( this->get_input_default_value( 0 ).type() == typeid( float ) ||
              this->get_input_default_value( 0 ).type() == typeid( frantic::magma::vec3 ) ) ) {
            // TODO This doesn't exactly match the behavior of making a grid (ex. it is defined everywhere, not just
            // inside the bounds). Should this be allowed?!
            bc2->compile_constant( this->get_id(), this->get_input_default_value( 0 ) );
        } else {
            bc2->compile_grid_cache( this->get_id(), this->get_input( 0 ), compiler.get_node_input( *this, 1 ), bounds,
                                     spacing );
        }
    } else {
        frantic::magma::nodes::magma_simple_operator<2>::compile_as_extension_type( compiler );
    }
}

MAGMA_DEFINE_TYPE( "VelocityGrid", "Stoke", StaggeredGridCacheNode )
MAGMA_INPUT_BEGIN( "VectorField" )
MAGMA_INPUT_ATTR( visitable, false )
MAGMA_INPUT_END
MAGMA_INPUT( "Position", boost::blank() )
MAGMA_EXPOSE_PROPERTY( absoluteSpacing, float )
MAGMA_EXPOSE_PROPERTY( relativeSpacing, float )
MAGMA_ENUM_PROPERTY( spacingType, "Absolute", "Relative" )
MAGMA_DESCRIPTION( "Samples the incoming velocity field (ie [vx,vy,vz]) at staggered grid points and uses trilinear "
                   "interpolation between samples. Also known as a MAC grid." )
MAGMA_DEFINE_TYPE_END;

void StaggeredGridCacheNode::compile_as_extension_type( frantic::magma::magma_compiler_interface& compiler ) {
    if( ember_compiler* bc2 = dynamic_cast<ember_compiler*>( &compiler ) ) {
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

        if( this->get_input( 0 ).first == frantic::magma::magma_interface::INVALID_ID &&
            this->get_input_default_value( 0 ).type() == typeid( frantic::magma::vec3 ) ) {
            // TODO This doesn't exactly match the behavior of making a grid (ex. it is defined everywhere, not just
            // inside the bounds). Should this be allowed?!
            bc2->compile_constant( this->get_id(), this->get_input_default_value( 0 ) );
        } else {
            bc2->compile_staggered_cache( this->get_id(), this->get_input( 0 ), compiler.get_node_input( *this, 1 ),
                                          bounds, spacing );
        }
    } else {
        frantic::magma::nodes::magma_simple_operator<2>::compile_as_extension_type( compiler );
    }
}

} // namespace nodes
} // namespace ember
