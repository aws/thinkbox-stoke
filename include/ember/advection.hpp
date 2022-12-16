// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <ember/staggered_grid.hpp>

#include <frantic/magma/magma_movable.hpp>
#include <frantic/magma/nodes/magma_node_impl.hpp>

#include <frantic/channels/named_channel_data.hpp>
#include <frantic/logging/progress_logger.hpp>

#include <boost/mpl/vector.hpp>
#include <boost/noncopyable.hpp>

namespace ember {

using frantic::magma::movable;

// Using MKL's FFT, solve the poisson equation on velocity field sampled from the provided field_interface. This will
// make the resulting velocity field divergence free.
void do_poisson_solve( staggered_grid& velocityGrid, float stepInSeconds, const char boundaryTypes[] = "DDDDDD",
                       frantic::logging::progress_logger* progress = NULL );

template <class FieldFunctor, class VelocityFunctor>
class advect_functor : public boost::noncopyable {
    FieldFunctor m_fieldSubexpr;
    VelocityFunctor m_velocitySubexpr;

    float m_timeStep;

  public:
    typedef void* return_type;
    typedef boost::mpl::vector<vec3> param_types;

  public:
    advect_functor() {}

    advect_functor( const movable<advect_functor>& wrapper )
        : m_fieldSubexpr( make_movable( wrapper.get().m_fieldSubexpr ) )
        , m_velocitySubexpr( make_movable( wrapper.get().m_velocitySubexpr ) ) {
        m_timeStep = wrapper.get().m_timeStep;
    }

    advect_functor operator=( const movable<advect_functor>& wrapper ) {
        m_fieldSubexpr = make_movable( wrapper.get().m_fieldSubexpr );
        m_velocitySubexpr = make_movable( wrapper.get().m_velocitySubexpr );
        m_timeStep = wrapper.get().m_timeStep;
        return *this;
    }

    const frantic::channels::channel_map& get_channel_map() const { return m_fieldSubexpr.get_channel_map(); }

    const frantic::channels::channel_map& get_output_map() const { return m_fieldSubexpr.get_output_map(); }

    void calculate_result_layout( frantic::channels::channel_map& map ) const {
        map = m_fieldSubexpr.get_channel_map();
    }

    void set_timestep( float timestep ) { m_timeStep = timestep; }

    void set_field( const movable<FieldFunctor>& subexpr ) {
        m_fieldSubexpr = subexpr;

        if( m_fieldSubexpr.get_output_map().channel_count() != 1 )
            THROW_MAGMA_INTERNAL_ERROR();
    }

    void set_velocity( const movable<VelocityFunctor>& subexpr ) {
        m_velocitySubexpr = subexpr;

        if( m_velocitySubexpr.get_output_map().channel_count() != 1 )
            THROW_MAGMA_INTERNAL_ERROR();

        if( m_velocitySubexpr.get_output_map()[0].data_type() !=
                frantic::channels::channel_data_type_traits<vec3>::data_type() ||
            m_velocitySubexpr.get_output_map()[0].arity() !=
                frantic::channels::channel_data_type_traits<vec3>::arity() ) {
            throw magma_exception() << magma_exception::error_name( _T("Invalid velocity field") );
        }
    }

    void operator()( void* out, const vec3& pos ) const {
        vec3 v1, v2;

        // This is an RK2 advection step.
        m_velocitySubexpr( &v1, pos );

        vec3 p1 = pos - m_timeStep * v1;

        m_velocitySubexpr( &v2, p1 );

        vec3 p2 = pos - 0.5f * m_timeStep * ( v1 + v2 );

        m_fieldSubexpr( out, p2 );
    }
};

template <class FieldFunctor, class VelocityFunctor>
class advect2_functor : public boost::noncopyable {
    FieldFunctor m_fieldSubexpr;
    VelocityFunctor m_velocitySubexpr;

    float m_timeStep;

  public:
    typedef void* return_type;
    typedef boost::mpl::vector<int, vec3> param_types;

  public:
    advect2_functor() {}

    advect2_functor( const movable<advect2_functor>& wrapper )
        : m_fieldSubexpr( make_movable( wrapper.get().m_fieldSubexpr ) )
        , m_velocitySubexpr( make_movable( wrapper.get().m_velocitySubexpr ) ) {
        m_timeStep = wrapper.get().m_timeStep;
    }

    advect2_functor& operator=( const movable<advect2_functor>& wrapper ) {
        m_fieldSubexpr = make_movable( wrapper.get().m_fieldSubexpr );
        m_velocitySubexpr = make_movable( wrapper.get().m_velocitySubexpr );
        m_timeStep = wrapper.get().m_timeStep;
        return *this;
    }

    const frantic::channels::channel_map& get_channel_map() const { return m_fieldSubexpr.get_channel_map(); }

    const frantic::channels::channel_map& get_output_map() const { return m_fieldSubexpr.get_output_map(); }

    void calculate_result_layout( frantic::channels::channel_map& map ) const {
        map = m_fieldSubexpr.get_channel_map();
    }

    void set_timestep( float timestep ) { m_timeStep = timestep; }

    void set_field( const movable<FieldFunctor>& subexpr ) {
        m_fieldSubexpr = subexpr;

        if( m_fieldSubexpr.get_output_map().channel_count() != 1 )
            THROW_MAGMA_INTERNAL_ERROR();
    }

    void set_velocity( const movable<VelocityFunctor>& subexpr ) {
        m_velocitySubexpr = subexpr;

        if( m_velocitySubexpr.get_output_map().channel_count() != 1 )
            THROW_MAGMA_INTERNAL_ERROR();

        if( m_velocitySubexpr.get_output_map()[0].data_type() !=
                frantic::channels::channel_data_type_traits<vec3>::data_type() ||
            m_velocitySubexpr.get_output_map()[0].arity() !=
                frantic::channels::channel_data_type_traits<vec3>::arity() ) {
            throw magma_exception() << magma_exception::error_name( _T("Invalid velocity field") );
        }
    }

    void operator()( void* out, int steps, const vec3& pos ) const {
        vec3 v1, v2;
        vec3 p0 = pos, p1;

        // Trace several timesteps backwards (which is like advecting multiple steps with a time-constant velocity
        // field)
        for( int i = 0; i < steps; ++i ) {
            // This is an RK2 advection step.
            m_velocitySubexpr( &v1, p0 );

            vec3 p1 = p0 - m_timeStep * v1;

            m_velocitySubexpr( &v2, p1 );

            p0 = p0 - 0.5f * m_timeStep * ( v1 + v2 );
        }

        m_fieldSubexpr( out, p0 );
    }
};

class advection_decorator : public frantic::volumetrics::field_interface {
  public:
    advection_decorator( const boost::shared_ptr<frantic::volumetrics::field_interface>& pField,
                         const boost::shared_ptr<frantic::volumetrics::field_interface>& pVelocity, float timeStep );

    virtual bool evaluate_field( void* dest, const frantic::graphics::vector3f& pos ) const;

    virtual const frantic::channels::channel_map& get_channel_map() const;

  private:
    boost::shared_ptr<frantic::volumetrics::field_interface> m_pField, m_pVelocity;
    float m_timeStep;
};

inline advection_decorator::advection_decorator(
    const boost::shared_ptr<frantic::volumetrics::field_interface>& pField,
    const boost::shared_ptr<frantic::volumetrics::field_interface>& pVelocity, float timeStep )
    : m_pField( pField )
    , m_pVelocity( pVelocity )
    , m_timeStep( timeStep ) {
    assert( m_pVelocity->get_channel_map().channel_count() != 1 ||
            m_pVelocity->get_channel_map()[0].data_type() != frantic::channels::data_type_float32 ||
            m_pVelocity->get_channel_map()[0].arity() != 1 );
}

inline bool advection_decorator::evaluate_field( void* dest, const frantic::graphics::vector3f& pos ) const {
    vec3 v1, v2;

    // This is an RK2 advection step.
    m_pVelocity->evaluate_field( &v1, pos );

    vec3 p1 = pos - m_timeStep * v1;

    m_pVelocity->evaluate_field( &v2, p1 );

    vec3 p2 = pos - 0.5f * m_timeStep * ( v1 + v2 );

    return m_pField->evaluate_field( dest, p2 );
}

inline const frantic::channels::channel_map& advection_decorator::get_channel_map() const {
    return m_pField->get_channel_map();
}

} // namespace ember
