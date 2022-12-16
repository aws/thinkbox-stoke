// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#include "stdafx.h"
#include <ember/ember_compiler.hpp>
#include <ember/nodes/GradientNode.hpp>

#include <frantic/magma/nodes/magma_node_impl.hpp>

namespace ember {
namespace nodes {

MAGMA_DEFINE_TYPE( "Gradient", "Stoke", GradientNode )
MAGMA_INPUT_BEGIN( "Field" )
MAGMA_INPUT_ATTR( visitable, false )
MAGMA_INPUT_END
MAGMA_INPUT( "Position", boost::blank() )
MAGMA_EXPOSE_PROPERTY( delta, float )
MAGMA_DESCRIPTION( "Calculates the gradient of a scalar field. The result points in the direction of greatest change." )
MAGMA_DEFINE_TYPE_END;

void GradientNode::compile_as_extension_type( frantic::magma::magma_compiler_interface& compiler ) {
    if( ember_compiler* bc2 = dynamic_cast<ember_compiler*>( &compiler ) ) {
        // TODO Move this constant check inside the compiler and have it handle more complex constant inputs (ex.
        // InputValue, etc.)
        if( this->get_input( 0 ).first == frantic::magma::magma_interface::INVALID_ID &&
            this->get_input_default_value( 0 ).type() == typeid( float ) )
            bc2->compile_constant(
                this->get_id(),
                frantic::magma::vec3(
                    0, 0,
                    0 ) ); // If we have a constant scalar input, the result is constant [0,0,0]. Woot for thinking!
        else
            bc2->compile_gradient( this->get_id(), this->get_input( 0 ), compiler.get_node_input( *this, 1 ),
                                   this->get_delta() );
    } else {
        frantic::magma::nodes::magma_simple_operator<2>::compile_as_extension_type( compiler );
    }
}

} // namespace nodes
} // namespace ember
