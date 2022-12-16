// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <frantic/magma/nodes/magma_node_property.hpp>
#include <frantic/magma/nodes/magma_simple_operator.hpp>

namespace ember {
namespace nodes {

class ParticleSplatNode : public frantic::magma::nodes::magma_simple_operator<2> {
  public:
    MAGMA_REQUIRED_METHODS( ParticleSplatNode );
    MAGMA_PROPERTY( channels, std::vector<frantic::tstring> );
    MAGMA_PROPERTY( absoluteSpacing, float );
    MAGMA_PROPERTY( relativeSpacing, float );
    MAGMA_PROPERTY( spacingType, frantic::tstring );
    MAGMA_PROPERTY( gridType, frantic::tstring );
    MAGMA_PROPERTY( autoBounds, bool );
    MAGMA_PROPERTY( boundsPadding, int );

    ParticleSplatNode()
        : m_absoluteSpacing( 1.f )
        , m_relativeSpacing( 1.f )
        , m_spacingType( _T("Relative") )
        , m_gridType( _T("Normal") )
        , m_autoBounds( true )
        , m_boundsPadding( 5 ) {
        m_channels.push_back( _T("Density") );
    }

    virtual ~ParticleSplatNode() {}

    virtual int get_num_outputs() const;

    virtual void get_output_description( int index, frantic::tstring& outDescription ) const;

    virtual void compile_as_extension_type( frantic::magma::magma_compiler_interface& compiler );
};

class ParticleSplatVelocityNode : public frantic::magma::nodes::magma_simple_operator<2> {
  public:
    MAGMA_REQUIRED_METHODS( ParticleSplatVelocityNode );
    MAGMA_PROPERTY( channelName, frantic::tstring );
    MAGMA_PROPERTY( absoluteSpacing, float );
    MAGMA_PROPERTY( relativeSpacing, float );
    MAGMA_PROPERTY( spacingType, frantic::tstring );
    MAGMA_PROPERTY( autoBounds, bool );
    MAGMA_PROPERTY( boundsPadding, int );
    MAGMA_PROPERTY( removeDivergence, bool );
    MAGMA_PROPERTY( solverTimestep, float );

    frantic::tstring m_densityChannelName;

    ParticleSplatVelocityNode()
        : m_channelName( _T("Velocity") )
        , m_absoluteSpacing( 1.f )
        , m_relativeSpacing( 1.f )
        , m_spacingType( _T("Relative") )
        , m_autoBounds( true )
        , m_boundsPadding( 5 )
        , m_removeDivergence( true )
        , m_solverTimestep( 1.f / 30.f )
        , m_densityChannelName( _T("Density") ) {}

    virtual ~ParticleSplatVelocityNode() {}

    inline const frantic::tstring& get_densityChannelName() const { return m_densityChannelName; }

    virtual int get_num_outputs() const;

    virtual void get_output_description( int index, frantic::tstring& outDescription ) const;

    virtual void compile_as_extension_type( frantic::magma::magma_compiler_interface& compiler );
};

} // namespace nodes
} // namespace ember
