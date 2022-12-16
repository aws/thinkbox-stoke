// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <frantic/magma/nodes/magma_node_property.hpp>
#include <frantic/magma/nodes/magma_simple_operator.hpp>

namespace ember {
namespace nodes {

class GridCacheNode : public frantic::magma::nodes::magma_simple_operator<2> {
  public:
    MAGMA_REQUIRED_METHODS( GridCacheNode );
    MAGMA_PROPERTY( absoluteSpacing, float );
    MAGMA_PROPERTY( relativeSpacing, float );
    MAGMA_PROPERTY( spacingType, frantic::tstring );

    GridCacheNode()
        : m_absoluteSpacing( 1.f )
        , m_relativeSpacing( 1.f )
        , m_spacingType( _T("Relative") ) {}

    virtual ~GridCacheNode() {}

    virtual void compile_as_extension_type( frantic::magma::magma_compiler_interface& compiler );
};

class StaggeredGridCacheNode : public frantic::magma::nodes::magma_simple_operator<2> {
  public:
    MAGMA_REQUIRED_METHODS( StaggeredGridCacheNode );
    MAGMA_PROPERTY( absoluteSpacing, float );
    MAGMA_PROPERTY( relativeSpacing, float );
    MAGMA_PROPERTY( spacingType, frantic::tstring );

    StaggeredGridCacheNode()
        : m_absoluteSpacing( 1.f )
        , m_relativeSpacing( 1.f )
        , m_spacingType( _T("Relative") ) {}

    virtual ~StaggeredGridCacheNode() {}

    virtual void compile_as_extension_type( frantic::magma::magma_compiler_interface& compiler );
};

} // namespace nodes
} // namespace ember
