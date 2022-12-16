// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#include "stdafx.h"

#include <ember/ember_compiler.hpp>
#include <ember/nodes/PressureSolveNode.hpp>

#include <frantic/graphics/boundbox3f.hpp>
#include <frantic/magma/nodes/magma_node_impl.hpp>

namespace ember {
namespace nodes {

MAGMA_DEFINE_TYPE( "SimpleFluid", "Stoke", DivergenceFreeCacheNode )
MAGMA_INPUT_BEGIN( "VectorField" )
MAGMA_INPUT_ATTR( visitable, false )
MAGMA_INPUT_END
MAGMA_INPUT( "Position", boost::blank() )
MAGMA_EXPOSE_PROPERTY( absoluteSpacing, float )
MAGMA_EXPOSE_PROPERTY( relativeSpacing, float )
MAGMA_ENUM_PROPERTY( spacingType, "Absolute", "Relative" )
MAGMA_EXPOSE_PROPERTY( timestep, float )
MAGMA_DESCRIPTION( "Modifies the incoming velocity field to behave like an incompressible fluid. The result is sampled "
                   "to a StaggeredGrid." )
MAGMA_DEFINE_TYPE_END;

void DivergenceFreeCacheNode::compile_as_extension_type( frantic::magma::magma_compiler_interface& compiler ) {
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

        // Can't do anything fancy with default value inputs because the solver might need to change the field (ex. to
        // conform to boundary conditions)
        bc2->compile_divergence_solve( this->get_id(), this->get_input( 0 ), compiler.get_node_input( *this, 1 ),
                                       bounds, spacing, m_timestep );
    } else {
        frantic::magma::nodes::magma_simple_operator<2>::compile_as_extension_type( compiler );
    }
}

} // namespace nodes
} // namespace ember
