// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <frantic/magma/magma_geometry_interface.hpp>
#include <frantic/magma/nodes/magma_node_property.hpp>
#include <frantic/magma/nodes/magma_simple_operator.hpp>

#include <ember/levelset.hpp>

namespace ember {
namespace nodes {

class LevelSetNode : public frantic::magma::nodes::magma_simple_operator<2> {
    std::vector<frantic::magma::magma_geometry_ptr> m_cachedGeometry;
    std::vector<frantic::graphics::transform4f> m_cachedTMs;

    // frantic::magma::magma_geometry_ptr m_cachedGeometry;
    levelset_functor::level_set_ptr m_cachedLevelSet;
    float m_explicitSpacing, m_relativeSpacing;
    frantic::tstring m_spacingType;

    int m_cachedBandWidth;
    bool m_cachedFill;

  public:
    MAGMA_REQUIRED_METHODS( LevelSetNode );
    MAGMA_PROPERTY( exposeGradient, bool );
    MAGMA_PROPERTY( fillVolume, bool );
    MAGMA_PROPERTY( bandWidthVoxels, int );

    LevelSetNode();

    virtual ~LevelSetNode() {}

    inline void set_explicitSpacing( float val ) {
        m_explicitSpacing = std::max( 1e-3f, val );
        m_cachedLevelSet.reset();
    } // Clear the cached levelset since the spacing changed.
    inline const float get_explicitSpacing() const { return m_explicitSpacing; }

    inline void set_relativeSpacing( float val ) {
        m_relativeSpacing = std::max( 1e-3f, val );
        m_cachedLevelSet.reset();
    } // Clear the cached levelset since the spacing changed.
    inline const float get_relativeSpacing() const { return m_relativeSpacing; }

    inline void set_spacingType( const frantic::tstring& val ) {
        if( m_spacingType != val ) {
            m_spacingType = val;
            m_cachedLevelSet.reset();
        }
    } // Clear the cached levelset since the spacing changed.
    inline const frantic::tstring& get_spacingType() const { return m_spacingType; }

    virtual void compile_as_extension_type( frantic::magma::magma_compiler_interface& compiler );

    virtual int get_num_outputs() const;
};

} // namespace nodes
} // namespace ember
