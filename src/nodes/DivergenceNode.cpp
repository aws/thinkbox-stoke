// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#include "stdafx.h"
#include <ember/ember_compiler.hpp>
#include <ember/nodes/DivergenceNode.hpp>

#include <frantic/magma/nodes/magma_node_impl.hpp>

namespace ember {
namespace nodes {

MAGMA_DEFINE_TYPE( "Divergence", "Stoke", DivergenceNode )
MAGMA_INPUT_BEGIN( "Field" )
MAGMA_INPUT_ATTR( visitable, false )
MAGMA_INPUT_END
MAGMA_INPUT( "Position", boost::blank() )
MAGMA_EXPOSE_PROPERTY( delta, float )
MAGMA_DESCRIPTION( "Calculates the divergence of a vector field. Intuitively, this quantity represents the balance of "
                   "outward flow versus inward flow at the point." )
MAGMA_DEFINE_TYPE_END;

void DivergenceNode::compile_as_extension_type( frantic::magma::magma_compiler_interface& compiler ) {
    if( ember_compiler* bc2 = dynamic_cast<ember_compiler*>( &compiler ) ) {
        // Special case of a constant input field which produces no divergence.
        if( this->get_input( 0 ).first == frantic::magma::magma_interface::INVALID_ID &&
            this->get_input_default_value( 0 ).type() == typeid( frantic::magma::vec3 ) )
            bc2->compile_constant( this->get_id(), 0.f );
        else
            bc2->compile_divergence( this->get_id(), this->get_input( 0 ), compiler.get_node_input( *this, 1 ),
                                     this->get_delta() );
    } else {
        frantic::magma::nodes::magma_simple_operator<2>::compile_as_extension_type( compiler );
    }
}

} // namespace nodes
} // namespace ember
