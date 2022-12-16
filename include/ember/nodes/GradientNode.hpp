// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <frantic/magma/nodes/magma_node_property.hpp>
#include <frantic/magma/nodes/magma_simple_operator.hpp>

namespace ember {
namespace nodes {

class GradientNode : public frantic::magma::nodes::magma_simple_operator<2> {
  public:
    MAGMA_REQUIRED_METHODS( GradientNode );
    MAGMA_PROPERTY( delta, float );

    GradientNode()
        : m_delta( 1e-1f ) {}

    virtual ~GradientNode() {}

    virtual void compile_as_extension_type( frantic::magma::magma_compiler_interface& compiler );
};

} // namespace nodes
} // namespace ember
