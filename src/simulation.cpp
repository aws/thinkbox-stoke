// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#include "stdafx.h"

#include <ember/advection.hpp>
#include <ember/concatenated_field.hpp>
#include <ember/ember_compiler.hpp>
#include <ember/grid.hpp>
#include <ember/simulation.hpp>
#include <ember/staggered_grid.hpp>

#include <frantic/magma/magma_movable.hpp>
#include <frantic/magma/nodes/magma_input_channel_node.hpp>
#include <frantic/magma/nodes/magma_output_node.hpp>

#include <frantic/volumetrics/field_interface.hpp>

#include <boost/integer.hpp>

namespace frantic {
namespace channels {

// TODO: Move the ember/magma type-traits implementation somewhere else and make it work consistently.
template <>
struct channel_data_type_traits<bool> {
    typedef boost::int_t<sizeof( bool ) * CHAR_BIT>::fast value_type;
    inline static size_t arity() { return 1; }
    inline static data_type_t data_type() { return channel_data_type_traits<value_type>::data_type(); }
    inline static frantic::tstring type_str() { return channel_data_type_str( arity(), data_type() ); }
    inline static value_type barycentric_combine( const frantic::graphics::vector3f& barycentricCoords, value_type a,
                                                  value_type b, value_type c ) {
        return barycentric_integer_combine( barycentricCoords, a, b, c );
    }
    inline static void weighted_sum_combine_general( const float* weights, const char* const* data,
                                                     std::size_t weightCount, std::size_t arity, char* out ) {
        weighted_sum_integer_combine_general<value_type>( weights, data, weightCount, arity, out );
    }
};

} // namespace channels
} // namespace frantic

namespace ember {

using frantic::magma::movable;

typedef frantic::volumetrics::field_interface field;
typedef boost::shared_ptr<field> field_ptr;

template <class T>
class constant_field : public field {
    frantic::channels::channel_map m_channelMap;
    T m_value;

  public:
    constant_field( const T& value, const frantic::tstring& valueName = _T("Value") )
        : m_value( value ) {
        m_channelMap.define_channel<T>( valueName );
        m_channelMap.end_channel_definition();
    }

    virtual const frantic::channels::channel_map& get_channel_map() const { return m_channelMap; }

    virtual bool evaluate_field( void* dest, const frantic::graphics::vector3f& ) const {
        *reinterpret_cast<T*>( dest ) = m_value;
        return true;
    }

    const T& get_constant() const { return m_value; }
};

inline bool is_constant_zero( field& theField ) {
    return ( typeid( theField ) == typeid( constant_field<vec3> ) )
               ? static_cast<constant_field<vec3>&>( theField ).get_constant() == vec3( 0, 0, 0 )
               : false;
}

class sim_compiler : public ember_compiler {
    std::map<frantic::tstring, field_ptr> m_channels;

  public:
    sim_compiler() {}

    sim_compiler& operator=( const movable<sim_compiler>& rhs ) {
        this->m_channels.swap( rhs.get().m_channels );
#ifndef BOOST_NO_CXX11_RVALUE_REFERENCES
        this->ember_compiler::operator=( boost::move( rhs.get() ) );
#else
        this->ember_compiler::operator=( boost::move<ember_compiler>( rhs.get() ) );
#endif
        return *this;
    }

    void set_channel_value( const frantic::tstring& channelName, field_ptr field );

    field_ptr get_channel_value( const frantic::tstring& channelName );

    virtual std::unique_ptr<expression_field> create_empty_expression_field();

    // Override these nodes to handle how we want evaluation to work.
    virtual void compile( frantic::magma::nodes::magma_input_channel_node* );

    // virtual boost::shared_ptr<frantic::volumetrics::field_interface> build_subtree_as_field( const
    // std::pair<frantic::magma::magma_interface::magma_id,int>& nodeOutput, const frantic::tstring& outputName, const
    // frantic::magma::magma_data_type* expectedType = NULL );
};

void sim_compiler::set_channel_value( const frantic::tstring& channelName, field_ptr field ) {
    m_channels[channelName] = field;
}

field_ptr sim_compiler::get_channel_value( const frantic::tstring& channelName ) { return m_channels[channelName]; }

std::unique_ptr<sim_compiler::expression_field> sim_compiler::create_empty_expression_field() {
    class expression_field_impl : public expression_field {
        sim_compiler m_expression;

      public:
        expression_field_impl( sim_compiler& parentExpression ) {
            m_expression.init_as_child_of( parentExpression );
            m_expression.m_channels = parentExpression.m_channels;
        }

        virtual ~expression_field_impl() {}

        virtual void add_output( frantic::magma::magma_interface::magma_id exprID, int outputIndex,
                                 const frantic::tstring& outputName = _T("Value") ) {
            m_expression.build_subtree( exprID );

            const base_compiler::temporary_meta& meta = m_expression.get_value( exprID, outputIndex );

            m_expression.append_output_channel( outputName, meta.first.m_elementCount, meta.first.m_elementType );
            m_expression.compile_output( -42, std::make_pair( exprID, outputIndex ), outputName, meta.first );
        }

        virtual const frantic::channels::channel_map& get_channel_map() const { return m_expression.get_output_map(); }

        virtual bool evaluate_field( void* dest, const frantic::graphics::vector3f& pos ) const {
            m_expression.eval( static_cast<char*>( dest ), pos );
            return true;
        }
    };

    return std::unique_ptr<expression_field>( new expression_field_impl( *this ) );
}

void sim_compiler::compile( frantic::magma::nodes::magma_input_channel_node* node ) {
    if( node->get_channelName() == _T("Position") ) {
        ember_compiler::compile( node );
    } else {
        std::map<frantic::tstring, field_ptr>::iterator it = m_channels.find( node->get_channelName() );
        if( it == m_channels.end() ) {
            field_ptr defaultField;

            if( node->get_channelType() == *frantic::magma::magma_singleton::get_named_data_type( _T("Float") ) )
                defaultField.reset( new constant_field<float>( 0.f ) );
            else if( node->get_channelType() == *frantic::magma::magma_singleton::get_named_data_type( _T("Vec3") ) )
                defaultField.reset( new constant_field<vec3>( vec3() ) );
            else
                defaultField.reset( new constant_field<float>( 0.f ) );

            it = m_channels.insert( std::make_pair( node->get_channelName(), defaultField ) ).first;
        }

        this->compile_field( node->get_id(), it->second, std::make_pair( this->get_position_expression_id(), 0 ) );
    }
}

// class sim_expression_adapter_field : public frantic::volumetrics::field_interface{
// public:
//	explicit sim_expression_adapter_field( const movable< sim_compiler >& expression );
//
//	virtual const frantic::channels::channel_map& get_channel_map() const;
//
//	virtual bool evaluate_field( void* dest, const frantic::graphics::vector3f& pos ) const;
//
// private:
//	sim_compiler m_expression;
// };

// boost::shared_ptr<frantic::volumetrics::field_interface> sim_compiler::build_subtree_as_field( const
// std::pair<frantic::magma::magma_interface::magma_id,int>& nodeOutput, const frantic::tstring& outputName, const
// frantic::magma::magma_data_type* expectedType ){ 	sim_compiler childCompiler; 	childCompiler.init_as_child_of( *this );
//	childCompiler.build_subtree( nodeOutput.first );
//	childCompiler.compile_output( -1, nodeOutput, outputName, expectedType ? *expectedType :
//frantic::magma::magma_data_type() );
//
//	boost::shared_ptr<frantic::volumetrics::field_interface> result( new sim_expression_adapter_field( make_movable(
//childCompiler ) ) );
//
//	return result;
// }

// sim_expression_adapter_field::sim_expression_adapter_field( const movable< sim_compiler >& expression ){
//	m_expression = expression;
// }
//
// const frantic::channels::channel_map& sim_expression_adapter_field::get_channel_map() const { return
// m_expression.get_output_map(); }
//
// bool sim_expression_adapter_field::evaluate_field( void* dest, const frantic::graphics::vector3f& pos ) const {
// m_expression.eval( (char*)dest, pos ); return true; }

// namespace{
//	class advection_decorator : public frantic::volumetrics::field_interface{
//	public:
//		advection_decorator( field_ptr _delegate, field_ptr _velocity, float _timeStep = 1.f );
//
//		virtual const frantic::channels::channel_map& get_channel_map() const ;
//
//		virtual bool evaluate_field( void* dest, const frantic::graphics::vector3f& pos ) const ;
//
//	private:
//		field_ptr m_delegate;
//		field_ptr m_velocityChannel;
//		float m_timeStep;
//	};
//
//	inline advection_decorator::advection_decorator( field_ptr _delegate, field_ptr _velocity, float _timeStep )
//		: m_delegate( _delegate ), m_velocityChannel( _velocity ), m_timeStep( _timeStep )
//	{}
//
//	inline const frantic::channels::channel_map& advection_decorator::get_channel_map() const {
//		return m_delegate->get_channel_map();
//	}
//
//	inline bool advection_decorator::evaluate_field( void* dest, const frantic::graphics::vector3f& pos ) const {
//		vec3 v1, v2;
//
//		//This is an RK2 advection step.
//		m_velocityChannel->evaluate_field( &v1, pos );
//
//		vec3 p1 = pos - m_timeStep * v1;
//
//		m_velocityChannel->evaluate_field( &v2, p1 );
//
//		vec3 p2 = pos - 0.5f * m_timeStep * ( v1 + v2 );
//
//		m_delegate->evaluate_field( dest, p2 );
//
//		return true;
//	}
// }

class simulation_impl : public simulation /*, private basic_compiler2*/ {
  public:
    simulation_impl();
    virtual ~simulation_impl();

    virtual void set_initial_simulation_expression( boost::shared_ptr<frantic::magma::magma_interface> magma );
    virtual void set_simulation_expression( boost::shared_ptr<frantic::magma::magma_interface> magma );
    virtual void set_simulation_context( boost::shared_ptr<simulation_context> ctx );

    virtual void set_timestep( float step );
    virtual void set_domain( int ( &bounds )[6], float spacing );
    virtual void set_velocity_domain( int ( &bounds )[6], float spacing );
    virtual void set_discretize( bool enabled = true );
    // virtual void set_divergence_free( bool enabled = true );
    virtual void set_velocity_solver( boost::shared_ptr<simulation_solver> solver );

    virtual void reset_simulation();
    virtual void simulate( float timeDelta );

    virtual field_ptr get_current_state();
    virtual field_ptr get_current_state( const frantic::tstring& name );

  private:
    void correct_velocity_field();
    void apply_discretization();

  private:
    // basic_compiler2 m_expr;
    float m_curTime;
    float m_timeStep;
    bool m_initialized;
    bool m_hasVelocity;

    boost::shared_ptr<frantic::magma::magma_interface> m_magma, m_initMagma;
    boost::shared_ptr<simulation_context> m_context;
    boost::shared_ptr<simulation_solver> m_solver;

    std::map<frantic::tstring, field_ptr> m_channels;

    int m_simBounds[6];
    float m_simSpacing;

    bool m_discretize; // Do we store channels on a grid between timesteps?

    field_ptr m_velocityChannel;

    int m_velocityBounds[6]; // This should correspond to m_simBounds, except take into account the different spacing.
    float m_velocitySpacing;

    // bool m_divergenceFree; //Do we solve for a divergence free velocity field?

    frantic::diagnostics::profiling_section psInit, psSimulate, psPressureSolve, psDiscretizing,
        psPressureSolveDiscretize, psPressureSolveSolve;

    // frantic::magma::magma_interface::magma_id m_implicitPosInput;
};

std::unique_ptr<simulation> simulation::create_instance() { return std::unique_ptr<simulation>( new simulation_impl ); }

simulation_impl::simulation_impl()
    : m_curTime( 0 )
    , m_timeStep( 1.f )
    , m_discretize( false )
    , /*m_divergenceFree( false ),*/ m_hasVelocity( true )
    , psInit( _T("Initialization") )
    , psSimulate( _T("Simulation") )
    , psPressureSolve( _T("VelocitySolve") )
    , psDiscretizing( _T("Discretizing") )
    , psPressureSolveDiscretize( _T("VelocitySolve:Discretize") )
    , psPressureSolveSolve( _T("VelocitySolve:Solve") ) {
    m_simBounds[0] = m_simBounds[1] = m_simBounds[2] = m_simBounds[3] = m_simBounds[4] = m_simBounds[5] = 0;
    m_simSpacing = 1.f;

    m_velocityBounds[0] = m_velocityBounds[1] = m_velocityBounds[2] = m_velocityBounds[3] = m_velocityBounds[4] =
        m_velocityBounds[5] = 0;
    m_velocitySpacing = 0.f;

    m_initialized = false;
}

simulation_impl::~simulation_impl() {}

void simulation_impl::set_initial_simulation_expression( boost::shared_ptr<frantic::magma::magma_interface> magma ) {
    m_initMagma = magma;
}

void simulation_impl::set_simulation_expression( boost::shared_ptr<frantic::magma::magma_interface> magma ) {
    // this->set_abstract_syntax_tree( magma );
    m_magma = magma;
}

void simulation_impl::set_simulation_context( boost::shared_ptr<simulation_context> ctx ) {
    // this->set_compilation_context( ctx );
    m_context = ctx;
}

void simulation_impl::set_velocity_solver( boost::shared_ptr<simulation_solver> solver ) { m_solver = solver; }

void simulation_impl::set_timestep( float step ) { m_timeStep = step; }

void simulation_impl::set_domain( int ( &bounds )[6], float spacing ) {
    memcpy( m_simBounds, bounds, sizeof( int ) * 6 );
    m_simSpacing = std::max( 1e-4f, spacing );
}

void simulation_impl::set_velocity_domain( int ( &bounds )[6], float spacing ) {
    memcpy( m_velocityBounds, bounds, sizeof( int ) * 6 );
    m_velocitySpacing = std::max( 1e-4f, spacing );
}

void simulation_impl::set_discretize( bool enabled ) { m_discretize = enabled; }

// void simulation_impl::set_divergence_free( bool enabled ){
//	m_divergenceFree = enabled;
// }

void simulation_impl::reset_simulation() {
    m_curTime = 0.f;
    m_channels.clear();
    m_velocityChannel.reset();
    m_initialized = false;
}

void simulation_impl::correct_velocity_field() {
    // We need to discretize the velocity in order to make it divergence free.
    if( m_solver || m_discretize ) {
        frantic::diagnostics::scoped_profile spsPressureSolve( psPressureSolve );

        // OPTIMIZATION: We don't need to do anything for constant (in space) velocity fields.
        std::unique_ptr<staggered_discretized_field> result;

        {
            frantic::diagnostics::scoped_profile spsPressureSolveDiscretize( psPressureSolveDiscretize );

            if( m_velocitySpacing > 0.f )
                result.reset( new staggered_discretized_field( m_velocityBounds, m_velocitySpacing, _T("Velocity") ) );
            else
                result.reset( new staggered_discretized_field( m_simBounds, m_simSpacing, _T("Velocity") ) );

            result->assign( *m_velocityChannel );
        }

        // TODO: Offer different methods for doing this solve.
        if( m_solver ) {
            frantic::diagnostics::scoped_profile spsPressureSolveSolve( psPressureSolveSolve );

            m_solver->solve( result->get_grid(), m_timeStep );
            // ember::do_poisson_solve( result->get_grid(), m_timeStep );
        }

        m_velocityChannel.reset( result.release() );
    }
}

void simulation_impl::apply_discretization() {
    // Discretize to fields
    // TODO: If we didn't change from the previous frame, this shouldn't be necessary.
    if( m_discretize ) {
        for( std::map<frantic::tstring, field_ptr>::iterator it = m_channels.begin(), itEnd = m_channels.end();
             it != itEnd; ++it ) {
            frantic::diagnostics::scoped_profile spsDiscretizing( psDiscretizing );

            if( it->second->get_channel_map().channel_count() == 1 ) {
                if( it->second->get_channel_map()[0].data_type() == frantic::channels::data_type_float32 &&
                    it->second->get_channel_map()[0].arity() == 1 ) {
                    std::unique_ptr<discretized_field<float>> gridResult(
                        new discretized_field<float>( m_simBounds, m_simSpacing, it->first ) );

                    gridResult->assign( *it->second );

                    it->second.reset( gridResult.release() );
                } else if( it->second->get_channel_map()[0].data_type() == frantic::channels::data_type_float32 &&
                           it->second->get_channel_map()[0].arity() == 3 ) {
                    std::unique_ptr<discretized_field<vec3>> gridResult(
                        new discretized_field<vec3>( m_simBounds, m_simSpacing, it->first ) );

                    gridResult->assign( *it->second );

                    it->second.reset( gridResult.release() );
                }
            } else {
                // TODO: Handle creating multiple grids from one field
                FF_LOG( error ) << _T("Field: \"") << it->first << _T("\" produced ")
                                << it->second->get_channel_map().channel_count()
                                << _T(" values which is not currently supported") << std::endl;
            }
        }
    }
}

// namespace{
//	void collect_input_channels_recursive( std::set<frantic::tstring>& outChannels, frantic::magma::magma_interface&
//magma, frantic::magma::magma_interface::magma_id currentNode ){ 		if( frantic::magma::magma_node_base* node =
//magma.get_node( currentNode ) ){ 			if( frantic::magma::nodes::magma_input_channel_node* inputChNode =
//dynamic_cast<frantic::magma::nodes::magma_input_channel_node*>( node ) ){ 				outChannels.insert(
//inputChNode->get_channelName() ); 			}else{ 				if( frantic::magma::nodes::magma_blop_node* blopNode =
//dynamic_cast<frantic::magma::nodes::magma_blop_node*>( node ) ) 					collect_input_channels_recursive( outChannels, magma,
//blopNode->get_internal_output()->get_id() );
//
//				for( int i = 0, iEnd = node->get_num_inputs(); i < iEnd; ++i )
//					collect_input_channels_recursive( outChannels, magma, node->get_input(i).first
//);
//			}
//		}
//	}
//
//	void collect_input_channels( std::set<frantic::tstring>& outChannels, frantic::magma::magma_interface& magma,
//frantic::magma::magma_interface::magma_id currentContainer = frantic::magma::magma_interface::TOPMOST_LEVEL ){ 		for(
//int i = 0, iEnd = magma.get_num_outputs( currentContainer ); i < iEnd; ++i ){ 			if( frantic::magma::magma_node_base*
//node = magma.get_node( magma.get_output( currentContainer ).first ) ){ 				if( node->get_enabled() )
//					collect_input_channels_recursive( outChannels, magma, node->get_id() );
//			}
//		}
//
//		//We don't care about position.
//		std::set<frantic::tstring>::iterator it = outChannels.find( _T("Position") );
//		if( it != outChannels.end() )
//			outChannels.erase( it );
//	}
// }

void simulation_impl::simulate( float endTime ) {
    if( m_timeStep <= 0.f )
        return;

    // frantic::diagnostics::profiling_section psInit( _T("Initialization") ), psSimulate( _T("Simulation") ),
    // psPressureSolve( _T("VelocitySolve") ), psDiscretizing( _T("Discretizing") ), psPressureSolveDiscretize(
    // _T("VelocitySolve:Discretize") ), psPressureSolveSolve( _T("VelocitySolve:Solve") );
    psInit.reset();
    psSimulate.reset();
    psPressureSolve.reset();
    psDiscretizing.reset();
    psPressureSolveDiscretize.reset();
    psPressureSolveSolve.reset();

    if( !m_initialized ) {
        frantic::diagnostics::scoped_profile spsInit( psInit );

        m_initialized = true;
        m_curTime = 0.f;

        m_channels.clear();
        m_velocityChannel.reset();

        if( m_initMagma ) {
            for( int i = 0, iEnd = m_initMagma->get_num_outputs( frantic::magma::magma_interface::TOPMOST_LEVEL );
                 i < iEnd; ++i ) {
                // TODO: Can we share any of the computation btw. channels?

                frantic::magma::nodes::magma_output_node* outNode =
                    dynamic_cast<frantic::magma::nodes::magma_output_node*>( m_initMagma->get_node(
                        m_initMagma->get_output( frantic::magma::magma_interface::TOPMOST_LEVEL, i ).first ) );
                if( outNode && outNode->get_enabled() ) {
                    field_ptr result;

                    sim_compiler compiler;
                    compiler.set_abstract_syntax_tree( m_initMagma );
                    compiler.set_compilation_context( m_context );
                    result = compiler.build_output_as_field( *outNode );

                    if( outNode->get_channelName() == _T("Velocity") )
                        m_velocityChannel = result;
                    else
                        m_channels[outNode->get_channelName()] = result;
                }
            }
        }

        if( m_velocityChannel ) {
            correct_velocity_field();
            m_hasVelocity = true;
        } else {
            m_velocityChannel.reset( new constant_field<vec3>( vec3(), _T("Velocity") ) );
            m_hasVelocity = false;

            if( m_magma ) {
                // Traverse the per-step expression's outputs to see if a Velocity will ever be defined. If not, we
                // don't have to simulate anything!
                for( int i = 0, iEnd = m_magma->get_num_outputs( frantic::magma::magma_interface::TOPMOST_LEVEL );
                     i < iEnd; ++i ) {
                    frantic::magma::nodes::magma_output_node* outNode =
                        dynamic_cast<frantic::magma::nodes::magma_output_node*>( m_magma->get_node(
                            m_magma->get_output( frantic::magma::magma_interface::TOPMOST_LEVEL, i ).first ) );
                    if( outNode && outNode->get_enabled() && outNode->get_channelName() == _T("Velocity") )
                        m_hasVelocity = true;
                }
            }
        }

        // Sample the initial value'd fields to a grid. Do this BEFORE assigning the rest constant values because
        // discretizing those is pointless.
        apply_discretization();

        if( m_magma ) {
            // Traverse the per-step expression's outputs adding an initial value for those that aren't already assigned
            // one.
            for( int i = 0, iEnd = m_magma->get_num_outputs( frantic::magma::magma_interface::TOPMOST_LEVEL ); i < iEnd;
                 ++i ) {
                frantic::magma::nodes::magma_output_node* outNode =
                    dynamic_cast<frantic::magma::nodes::magma_output_node*>( m_magma->get_node(
                        m_magma->get_output( frantic::magma::magma_interface::TOPMOST_LEVEL, i ).first ) );
                if( outNode && outNode->get_enabled() && outNode->get_channelName() != _T("Velocity") &&
                    m_channels.find( outNode->get_channelName() ) == m_channels.end() ) {
                    if( outNode->get_channelType() ==
                        *frantic::magma::magma_singleton::get_named_data_type( _T("Float") ) )
                        m_channels[outNode->get_channelName()].reset(
                            new constant_field<float>( 0.f, outNode->get_channelName() ) );
                    else if( outNode->get_channelType() ==
                             *frantic::magma::magma_singleton::get_named_data_type( _T("Vec3") ) )
                        m_channels[outNode->get_channelName()].reset(
                            new constant_field<vec3>( vec3(), outNode->get_channelName() ) );
                }
            }
        }
    }

    if( !m_hasVelocity )
        m_curTime = endTime - m_timeStep; // Make it so it only runs once.

    if( !m_magma )
        m_curTime = endTime;

    for( ; m_curTime < endTime; m_curTime += m_timeStep ) {
        frantic::diagnostics::scoped_profile spsSimulate( psSimulate );

        boost::shared_ptr<simulation_context> curContext(
            m_context->clone_for_simulation_time( m_curTime + m_timeStep ).release() ); // TODO: Justify this

        // We are moving from time T to time T+x. We apply an advection step to achieve this, then allow the user to
        // modify the moved field.
        // TODO: Don't bother if we know our velocity is [0,0,0].
        if( m_hasVelocity && m_velocityChannel && m_velocityChannel->get_channel_map().channel_count() == 1 &&
            m_velocityChannel->get_channel_map()[0].data_type() == frantic::channels::data_type_float32 &&
            m_velocityChannel->get_channel_map()[0].arity() == 3 ) {
            for( std::map<frantic::tstring, field_ptr>::iterator it = m_channels.begin(), itEnd = m_channels.end();
                 it != itEnd; ++it )
                it->second.reset( new advection_decorator( it->second, m_velocityChannel, m_timeStep ) );
            m_velocityChannel.reset( new advection_decorator( m_velocityChannel, m_velocityChannel, m_timeStep ) );
        }

        apply_discretization();

        std::map<frantic::tstring, field_ptr> nextChannels;
        field_ptr nextVelocity;

        sim_compiler compiler;
        compiler.set_abstract_syntax_tree( m_magma );
        compiler.set_compilation_context( curContext );

        for( int i = 0, iEnd = m_magma->get_num_outputs( frantic::magma::magma_interface::TOPMOST_LEVEL ); i < iEnd;
             ++i ) {
            // TODO: Can we share any of the computation btw. channels?

            frantic::magma::nodes::magma_output_node* outNode = dynamic_cast<frantic::magma::nodes::magma_output_node*>(
                m_magma->get_node( m_magma->get_output( frantic::magma::magma_interface::TOPMOST_LEVEL, i ).first ) );
            if( outNode && outNode->get_enabled() ) {
                // Check if we are just passing through from an input to an output, so we can skip any extra processing.
                // TODO: This can be generalize and made a responsibility of sim_compiler. It should indicate when it's
                // expression could be
                //       replaced with a *simpler* field_interface instance.
                frantic::magma::nodes::magma_input_channel_node* inNode =
                    dynamic_cast<frantic::magma::nodes::magma_input_channel_node*>(
                        m_magma->get_node( outNode->get_input( 0 ).first ) );
                if( !inNode || !inNode->get_enabled() || inNode->get_channelName() != outNode->get_channelName() ) {
                    field_ptr result;

                    for( std::map<frantic::tstring, field_ptr>::iterator it = m_channels.begin(),
                                                                         itEnd = m_channels.end();
                         it != itEnd; ++it )
                        compiler.set_channel_value( it->first, it->second );
                    compiler.set_channel_value( _T("Velocity"), m_velocityChannel );

                    result = compiler.build_output_as_field( *outNode );

                    if( outNode->get_channelName() == _T("Velocity") )
                        nextVelocity = result;
                    else
                        nextChannels[outNode->get_channelName()] = result;
                } else {
                    if( outNode->get_channelName() == _T("Velocity") )
                        nextVelocity = m_velocityChannel;
                    else
                        nextChannels[outNode->get_channelName()] = m_channels[outNode->get_channelName()];
                }
            }
        }

        // Run through m_channels forwarding any channels that weren't updated as part of the simulation step.
        for( std::map<frantic::tstring, field_ptr>::const_iterator it = m_channels.begin(), itEnd = m_channels.end();
             it != itEnd; ++it ) {
            if( nextChannels.find( it->first ) == nextChannels.end() )
                nextChannels.insert( *it );
        }

        if( nextChannels.find( _T("Velocity") ) !=
            nextChannels.end() ) // Make sure we special cased all the Velocity stuff properly.
            THROW_MAGMA_INTERNAL_ERROR();

        m_channels.swap( nextChannels );

        if( nextVelocity )
            m_velocityChannel.swap( nextVelocity );

        if( m_hasVelocity )
            correct_velocity_field();
    }

    FF_LOG( progress ) << _T("Simulated to time: ") << m_curTime << std::endl;
    if( frantic::logging::is_logging_stats() ) {
        frantic::logging::stats << _T("Simulated: ") << m_channels.size() << _T(" channels\n");

        for( std::map<frantic::tstring, field_ptr>::const_iterator it = m_channels.begin(), itEnd = m_channels.end();
             it != itEnd; ++it )
            frantic::logging::stats << _T( '\t' ) << it->first << _T( '\n' );

        frantic::logging::stats << psInit << _T( '\n' ) << psSimulate << _T( '\n' ) << std::endl;

        if( this->m_discretize ) {
            frantic::logging::stats << _T("Grid is [") << m_simBounds[0] << _T( ',' ) << m_simBounds[2] << _T( ',' )
                                    << m_simBounds[4] << _T("] to [") << m_simBounds[1] << _T( ',' ) << m_simBounds[3]
                                    << _T( ',' ) << m_simBounds[5] << _T("]\n")
                                    << _T("There are: ")
                                    << ( ( m_simBounds[1] - m_simBounds[0] ) * ( m_simBounds[3] - m_simBounds[2] ) *
                                         ( m_simBounds[5] - m_simBounds[4] ) )
                                    << _T(" samples per grid\n")
                                    << _T("Samples are spaced: ") << m_simSpacing << _T(" units apart\n")
                                    << psDiscretizing << std::endl;

            float vSpacing;
            int* bounds;

            if( m_velocitySpacing > 0.f ) {
                vSpacing = m_velocitySpacing;
                bounds = m_velocityBounds;
            } else {
                vSpacing = m_simSpacing;
                bounds = m_simBounds;
            }

            frantic::logging::stats << _T("Velocity grid is [") << bounds[0] << _T( ',' ) << bounds[2] << _T( ',' )
                                    << bounds[4] << _T("] to [") << bounds[1] << _T( ',' ) << bounds[3] << _T( ',' )
                                    << bounds[5] << _T("]\n")
                                    << _T("There are: ")
                                    << ( ( bounds[1] - bounds[0] + 1 ) * ( bounds[3] - bounds[2] + 1 ) *
                                         ( bounds[5] - bounds[4] + 1 ) )
                                    << _T(" samples for Velocity\n")
                                    << _T("Velocity samples are spaced: ") << vSpacing << _T(" units apart\n")
                                    << psPressureSolve << _T( '\n' ) << psPressureSolveDiscretize << _T( '\n' )
                                    << psPressureSolveSolve << std::endl;
        } else {
            frantic::logging::stats << psPressureSolve << _T( '\n' ) << psPressureSolveDiscretize << _T( '\n' )
                                    << psPressureSolveSolve << std::endl;
        }
    }
}

field_ptr simulation_impl::get_current_state() {
    std::unique_ptr<concatenated_field> result( new concatenated_field );

    for( std::map<frantic::tstring, field_ptr>::const_iterator it = m_channels.begin(), itEnd = m_channels.end();
         it != itEnd; ++it )
        result->add_field( it->second );

    if( m_velocityChannel )
        result->add_field( m_velocityChannel );

    return field_ptr( result.release() );
}

field_ptr simulation_impl::get_current_state( const frantic::tstring& channelName ) {
    std::map<frantic::tstring, field_ptr>::iterator it = m_channels.find( channelName );
    if( it != m_channels.end() )
        return it->second;
    return field_ptr();
}

} // namespace ember
