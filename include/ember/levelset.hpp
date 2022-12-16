// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <ember/data_types.hpp>

#include <frantic/channels/channel_map.hpp>
#include <frantic/graphics/transform4f.hpp>
#include <frantic/volumetrics/levelset/rle_level_set.hpp>
#include <frantic/volumetrics/levelset/rle_trilerp.hpp>

#include <boost/mpl/vector.hpp>
#include <boost/shared_ptr.hpp>

namespace ember {

class levelset_functor {
  public:
    typedef float return_type;
    typedef boost::mpl::vector<vec3> param_types;

    typedef frantic::volumetrics::levelset::rle_level_set level_set_type;
    typedef boost::shared_ptr<level_set_type> level_set_ptr;

  private:
    level_set_ptr m_levelSet;
    frantic::graphics::transform4f m_preTransform;

  public:
    levelset_functor() {}

    levelset_functor( level_set_ptr lvlSet,
                      const frantic::graphics::transform4f& preTM = frantic::graphics::transform4f::identity() )
        : m_levelSet( lvlSet )
        , m_preTransform( preTM ) {}

    inline void set_levelset( level_set_ptr lvlSet ) { m_levelSet = lvlSet; }

    // This TM is applied before looking into the levelset.
    inline void set_pre_transform( const frantic::graphics::transform4f& tm ) { m_preTransform = tm; }

    inline float operator()( const vec3& pos ) const {
        using namespace frantic::graphics;
        using namespace frantic::volumetrics;
        using namespace frantic::volumetrics::levelset;

        vector3f vc = m_levelSet->get_voxel_coord_system().get_voxel_coord( m_preTransform * pos );

        float result;
        if( !trilerp_float( m_levelSet->get_rle_index_spec(), &m_levelSet->get_distance_data().front(), vc, &result ) )
            result = std::numeric_limits<float>::max();

        return result;
    }

    virtual void operator()( float& out, const vec3& pos ) const { out = operator()( pos ); }
};

class levelset_functor2 {
    struct results_holder {
        float distance;
        vec3 gradient;
    };

    static frantic::channels::channel_map s_outMap;

  public:
    typedef void* return_type;
    typedef boost::mpl::vector<vec3> param_types;

    inline void calculate_result_layout( frantic::channels::channel_map& map ) const {
        map.reset();
        map.define_channel( _T("Distance"), 1, frantic::channels::data_type_float32,
                            offsetof( results_holder, distance ) );
        map.define_channel( _T("Gradient"), 3, frantic::channels::data_type_float32,
                            offsetof( results_holder, gradient ) );

        map.end_channel_definition( 4u, true, false );
    }

    const frantic::channels::channel_map& get_output_map() const {
        if( !s_outMap.channel_definition_complete() ) {
            s_outMap.define_channel( _T("Distance"), 1, frantic::channels::data_type_float32,
                                     offsetof( results_holder, distance ) );
            s_outMap.define_channel( _T("Gradient"), 3, frantic::channels::data_type_float32,
                                     offsetof( results_holder, gradient ) );
            s_outMap.end_channel_definition( 4u, true, false );
        }
        return s_outMap;
    }

    typedef frantic::volumetrics::levelset::rle_level_set level_set_type;
    typedef boost::shared_ptr<level_set_type> level_set_ptr;

  private:
    level_set_ptr m_levelSet;
    frantic::graphics::transform4f m_preTransform;

  public:
    levelset_functor2() {}

    levelset_functor2( level_set_ptr lvlSet,
                       const frantic::graphics::transform4f& preTM = frantic::graphics::transform4f::identity() )
        : m_levelSet( lvlSet )
        , m_preTransform( preTM ) {}

    inline void set_levelset( level_set_ptr lvlSet ) { m_levelSet = lvlSet; }

    // This TM is applied before looking into the levelset.
    inline void set_pre_transform( const frantic::graphics::transform4f& tm ) { m_preTransform = tm; }

    virtual void operator()( void* dest, const vec3& pos ) const {
        using namespace frantic::graphics;
        using namespace frantic::volumetrics;
        using namespace frantic::volumetrics::levelset;

        results_holder* result = static_cast<results_holder*>( dest );

        vector3f vc = m_levelSet->get_voxel_coord_system().get_voxel_coord( m_preTransform * pos );
        if( !trilerp_float( m_levelSet->get_rle_index_spec(), &m_levelSet->get_distance_data().front(), vc,
                            &result->distance, &result->gradient ) ) {
            result->distance = std::numeric_limits<float>::max();
            result->gradient = vec3( 0, 0, 0 );
        }
    }
};

} // namespace ember
