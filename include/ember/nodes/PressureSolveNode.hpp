// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <frantic/magma/nodes/magma_node_property.hpp>
#include <frantic/magma/nodes/magma_simple_operator.hpp>

namespace ember {
namespace nodes {

class DivergenceFreeCacheNode : public frantic::magma::nodes::magma_simple_operator<2> {
  public:
    MAGMA_REQUIRED_METHODS( DivergenceFreeCacheNode );
    MAGMA_PROPERTY( absoluteSpacing, float );
    MAGMA_PROPERTY( relativeSpacing, float );
    MAGMA_PROPERTY( spacingType, frantic::tstring );
    MAGMA_PROPERTY( timestep, float );

    DivergenceFreeCacheNode()
        : m_absoluteSpacing( 1.f )
        , m_relativeSpacing( 1.f )
        , m_spacingType( _T("Relative") )
        , m_timestep( 1.f / 30.f ) {}

    virtual ~DivergenceFreeCacheNode() {}

    virtual void compile_as_extension_type( frantic::magma::magma_compiler_interface& compiler );
};

} // namespace nodes
} // namespace ember
