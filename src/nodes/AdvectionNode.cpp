// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#include "stdafx.h"
#include <ember/ember_compiler.hpp>
#include <ember/nodes/AdvectionNode.hpp>

#include <frantic/magma/nodes/magma_node_impl.hpp>

namespace ember {
namespace nodes {

MAGMA_DEFINE_TYPE( "Advect", "Stoke", AdvectionNode )
MAGMA_INPUT_BEGIN( "Field" )
MAGMA_INPUT_ATTR( visitable, false )
MAGMA_INPUT_END
MAGMA_INPUT_BEGIN( "VelocityField" )
MAGMA_INPUT_ATTR( visitable, false )
MAGMA_INPUT_END
MAGMA_INPUT( "Steps", 1 )
MAGMA_INPUT( "Position", boost::blank() )
MAGMA_EXPOSE_PROPERTY( timeStep, float )
MAGMA_DESCRIPTION( "A single step of semi-lagrangian advection. Moves around the input field based on the velocity "
                   "field and time step." )
MAGMA_DEFINE_TYPE_END;

void AdvectionNode::compile_as_extension_type( frantic::magma::magma_compiler_interface& compiler ) {
    if( ember_compiler* bc2 = dynamic_cast<ember_compiler*>( &compiler ) ) {
        // Special case if the incoming input field is a constant. Advecting it does nothing.
        if( this->get_input( 0 ).first == frantic::magma::magma_interface::INVALID_ID &&
            this->get_input_default_value( 0 ).type() != typeid( boost::blank ) )
            bc2->compile_constant( this->get_id(), this->get_input_default_value( 0 ) );
        else
            // TODO Default values are not being passed correctly into ember nodes that work with fields.
            bc2->compile_advect( this->get_id(), this->get_input( 0 ), this->get_input( 1 ),
                                 compiler.get_node_input( *this, 2 ), compiler.get_node_input( *this, 3 ),
                                 this->get_timeStep() );
    } else {
        frantic::magma::nodes::magma_simple_operator<4>::compile_as_extension_type( compiler );
    }
}

} // namespace nodes
} // namespace ember
