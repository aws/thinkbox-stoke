// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#include "stdafx.h"

#include <ember/advection.hpp>
#include <ember/ember_compiler.hpp>
#include <ember/grid.hpp>
#include <ember/levelset.hpp>
#include <ember/staggered_grid.hpp>

#include <frantic/magma/simple_compiler/base_compiler_impl.hpp>

using frantic::magma::magma_data_type;
using frantic::magma::magma_exception;
using frantic::magma::simple_compiler::base_compiler;

using frantic::magma::make_movable;
using frantic::magma::movable;

namespace ember {

ember_compiler::ember_compiler()
    : m_cachedFunctions( new std::map<std::pair<expression_id, int>, field_ptr> ) {
    m_singletonPosExpr = frantic::magma::magma_interface::INVALID_ID;
}

ember_compiler::ember_compiler( BOOST_RV_REF( ember_compiler ) rhs ) {
    this->base_compiler::do_move( rhs );

    m_singletonPosExpr = rhs.m_singletonPosExpr;

    this->m_outMap.swap( rhs.m_outMap );
    this->m_cachedFunctions.swap( rhs.m_cachedFunctions );
}

ember_compiler::~ember_compiler() {}

ember_compiler& ember_compiler::operator=( BOOST_RV_REF( ember_compiler ) rhs ) {
    this->base_compiler::do_move( rhs );

    m_singletonPosExpr = rhs.m_singletonPosExpr;

    this->m_outMap.swap( rhs.m_outMap );
    this->m_cachedFunctions.swap( rhs.m_cachedFunctions );

    return *this;
}

class ember_compiler::expression_field_impl : public ember_compiler::expression_field {
    ember_compiler m_expression;

  public:
    expression_field_impl( ember_compiler& parentExpression ) { m_expression.init_as_child_of( parentExpression ); }

    virtual ~expression_field_impl() {}

    virtual void add_output( frantic::magma::magma_interface::magma_id exprID, int outputIndex,
                             const frantic::tstring& outputName = _T("Value") ) {
        m_expression.build_subtree( exprID );

        const base_compiler::temporary_meta& meta = m_expression.get_value( exprID, outputIndex );

        m_expression.m_outMap.define_channel( outputName, meta.first.m_elementCount, meta.first.m_elementType );
        m_expression.m_outMap.end_channel_definition();

        m_expression.compile_output( -42, std::make_pair( exprID, outputIndex ), outputName, meta.first );
    }

    virtual const frantic::channels::channel_map& get_channel_map() const { return m_expression.get_output_map(); }

    virtual bool evaluate_field( void* dest, const frantic::graphics::vector3f& pos ) const {
        m_expression.eval( static_cast<char*>( dest ), pos );
        return true;
    }
};

std::unique_ptr<ember_compiler::expression_field> ember_compiler::create_empty_expression_field() {
    return std::unique_ptr<expression_field>( new expression_field_impl( *this ) );
}

void ember_compiler::init_as_child_of( ember_compiler& parent ) {
    this->set_abstract_syntax_tree( parent.get_abstract_syntax_tree_ptr() );
    this->set_compilation_context( parent.get_context_ptr() );

    // By copying cached functions, we are able to reuse them.
    this->m_cachedFunctions = parent.m_cachedFunctions;

    // Don't copy m_outMap since we are making a different one.
}

void ember_compiler::append_output_channel( const frantic::tstring& channelName, std::size_t arity,
                                            frantic::channels::data_type_t type ) {
    if( !m_outMap.channel_definition_complete() )
        m_outMap.end_channel_definition();

    m_outMap.append_channel( channelName, arity, type );
}

ember_compiler::expression_id ember_compiler::get_position_expression_id() {
    if( m_singletonPosExpr == frantic::magma::magma_interface::INVALID_ID )
        this->compile_input_position( -5 );
    return m_singletonPosExpr;
}

void ember_compiler::build() {
    boost::shared_ptr<frantic::magma::magma_interface> magma = this->get_abstract_syntax_tree_ptr();

    if( !magma )
        THROW_MAGMA_INTERNAL_ERROR();

    m_outMap.reset();

    // Determine the output map before building via the parent class
    for( int i = 0, iEnd = magma->get_num_outputs( frantic::magma::magma_interface::TOPMOST_LEVEL ); i < iEnd; ++i ) {
        std::pair<frantic::magma::magma_interface::magma_id, int> output =
            magma->get_output( frantic::magma::magma_interface::TOPMOST_LEVEL, i );

        bool enabled = false;

        // If an output node is enabled and it isn't disconnected
        if( magma->get_property( output.first, _T("enabled"), enabled ) && enabled &&
            magma->get_input( output.first, 0 ).first != frantic::magma::magma_interface::INVALID_ID ) {
            frantic::tstring channelName;
            frantic::magma::magma_data_type channelType;

            magma->get_property( output.first, _T("channelName"), channelName );
            magma->get_property( output.first, _T("channelType"), channelType );

            // It might be possible that the flow doesn't specify the type, so we can't add it to the map beforehand. It
            // will get done via compile_output().
            if( !channelName.empty() && channelType.m_elementType != frantic::channels::data_type_invalid ) {
                // Force all float channels to float32.
                if( frantic::channels::is_channel_data_type_float( channelType.m_elementType ) )
                    channelType.m_elementType = frantic::channels::data_type_float32;

                m_outMap.define_channel( channelName, channelType.m_elementCount, channelType.m_elementType );
            }
        }
    }

    m_outMap.end_channel_definition();

    base_compiler::build();
}

ember_compiler::field_ptr ember_compiler::build_output_as_field( frantic::magma::nodes::magma_output_node& rootNode ) {
    // Force Ember to use Float32 everywhere.
    frantic::magma::magma_data_type dt = rootNode.get_channelType();
    if( frantic::channels::is_channel_data_type_float( dt.m_elementType ) )
        dt.m_elementType = frantic::channels::data_type_float32;

    try {
        return this->build_subtree_as_field( rootNode.get_input( 0 ), rootNode.get_channelName(), &dt );
    } catch( frantic::magma::magma_exception& e ) {
        // If we didn't have a specific node throw the error, then it must have been an error in the call to
        // build_subtree_as_field() which means we should use the Output node's ID.
        if( e.get_node_id() == frantic::magma::magma_interface::INVALID_ID )
            e << frantic::magma::magma_exception::node_id( rootNode.get_id() )
              << frantic::magma::magma_exception::input_index( 0 );

        e << frantic::magma::magma_exception::source_expression_id(
            this->get_abstract_syntax_tree().get_expression_id() );

        throw;
    }
}

ember_compiler::field_ptr
ember_compiler::build_subtree_as_field( const std::pair<base_compiler::expression_id, int>& nodeOutput,
                                        const frantic::tstring& outputName,
                                        const frantic::magma::magma_data_type* expectedType ) {
    std::map<std::pair<expression_id, int>, field_ptr>::const_iterator it = m_cachedFunctions->find( nodeOutput );
    if( it != m_cachedFunctions->end() )
        return it->second;

    // std::map< std::pair<expression_id,int>,field_ptr >::const_iterator it = m_cachedFunctions->find( std::make_pair(
    // nodeOutput.first, -1 ) ); if( it != m_cachedFunctions->end() ){
    //	//We need to bind the position input of this field to something ...
    // }

    boost::shared_ptr<expression_field> result( this->create_empty_expression_field() );

    if( nodeOutput.first != frantic::magma::magma_interface::INVALID_ID )
        result->add_output( nodeOutput.first, nodeOutput.second, outputName );

    if( expectedType && ( result->get_channel_map().channel_count() != 1 ||
                          result->get_channel_map()[0].data_type() != expectedType->m_elementType ||
                          result->get_channel_map()[0].arity() != expectedType->m_elementCount ) ) {
        frantic::magma::magma_data_type foundType;
        foundType.m_elementType = result->get_channel_map()[0].data_type();
        foundType.m_elementCount = result->get_channel_map()[0].arity();

        throw frantic::magma::magma_exception()
            << frantic::magma::magma_exception::connected_id( nodeOutput.first )
            << frantic::magma::magma_exception::connected_output_index( nodeOutput.second )
            << frantic::magma::magma_exception::expected_type( *expectedType )
            << frantic::magma::magma_exception::found_type( foundType )
            << frantic::magma::magma_exception::error_name( _T("Invalid type") );
    }

    // result.reset(); //Return NULL to indicate an error. Maybe an exception instead?

    // Cache this for later re-use. Ex. Taking the gradient & curl of the same output allows re-use.
    //  TODO Is this worth doing? It basically only prevents re-compilation of the subtree. That will be handy if a grid
    //  node is inside the subtree I suppose...

    // BUG: Since 'm_cachedFunctions' is a shared_ptr, we were causing a reference loop if 'result' held an
    // ember_compiler using the same shared_ptr. Then we were boned.
    // m_cachedFunctions->insert( std::make_pair( nodeOutput, result ) );

    // TODO We could get the result->m_expression.m_cachedFunctions and bring them back into *this* object to
    // potentially prevent some re-compilation. Ex

    return result;
}

ember_compiler::field_ptr ember_compiler::find_cached_field( expression_id exprID, int outputIndex ) const {
    std::map<std::pair<expression_id, int>, field_ptr>::const_iterator it =
        m_cachedFunctions->find( std::make_pair( exprID, outputIndex ) );
    if( it != m_cachedFunctions->end() )
        return it->second;
    return field_ptr();
}

struct ember_state : public base_compiler::state {
    frantic::graphics::vector3f pos;
    char* outBuffer;

    ember_state( char* _outBuffer, const frantic::graphics::vector3f& _pos )
        : outBuffer( _outBuffer )
        , pos( _pos ) {}
};

void ember_compiler::eval( char* outBuffer, const frantic::graphics::vector3f& pos ) const {
    ember_state data( outBuffer, pos );

    this->do_eval( data );
}

void ember_compiler::eval_debug( char* outBuffer, const frantic::graphics::vector3f& pos,
                                 frantic::magma::debug_data& outDebugData ) const {
    ember_state data( outBuffer, pos );

    this->do_eval_debug( data, outDebugData );
}

namespace {
class input_position_expression : public base_compiler::expression {
    std::ptrdiff_t m_outPtr;

    static void internal_apply( const base_compiler::expression* _this, base_compiler::state& state );

  public:
    virtual void set_input( std::size_t /*inputIndex*/, std::ptrdiff_t /*relPtr*/ ) {}
    virtual void set_output( std::ptrdiff_t relPtr ) { m_outPtr = relPtr; }

    virtual const frantic::channels::channel_map& get_output_map() const {
        return frantic::magma::simple_compiler::traits<frantic::graphics::vector3f>::get_static_map();
    }

    virtual void apply( base_compiler::state& data ) const {
        data.set_temporary( m_outPtr, static_cast<ember_state&>( data ).pos );
    }

    virtual runtime_ptr get_runtime_ptr() const { return &internal_apply; }
};

void input_position_expression::internal_apply( const base_compiler::expression* _this, base_compiler::state& state ) {
    static_cast<const input_position_expression*>( _this )->input_position_expression::apply( state );
}
} // namespace

void ember_compiler::compile_input_position( expression_id exprID ) {
    if( m_singletonPosExpr == frantic::magma::magma_interface::INVALID_ID ) {
        std::unique_ptr<base_compiler::expression> result( new input_position_expression );

        result->set_output(
            this->allocate_temporary( exprID,
                                      frantic::magma::simple_compiler::traits<frantic::graphics::vector3f>::get_type() )
                .second );

        this->register_expression( std::move( result ) );

        m_singletonPosExpr = exprID;
    } else {
        const temporary_meta& meta = this->get_value( m_singletonPosExpr, 0 );

        // Just copy the existing one.
        this->register_temporary( exprID, meta.first, meta.second );
    }
}

namespace {
class output_expression : public base_compiler::expression {
    frantic::channels::channel_general_accessor m_outAccessor;
    frantic::channels::channel_type_convertor_function_t m_cvtFn;
    frantic::channels::channel_map m_outMap;
    std::ptrdiff_t m_inputRelPtr;

    static void internal_apply( const base_compiler::expression* _this, base_compiler::state& data );

  public:
    output_expression( const frantic::magma::magma_data_type& inputType,
                       const frantic::channels::channel& outChannel ) {
        m_cvtFn = frantic::channels::get_channel_type_convertor_function( inputType.m_elementType,
                                                                          outChannel.data_type(), outChannel.name() );
        m_outAccessor = frantic::channels::channel_general_accessor( outChannel.offset(), outChannel.arity(),
                                                                     outChannel.data_type() );

        m_outMap.define_channel( outChannel.name(), outChannel.arity(), outChannel.data_type() );
        m_outMap.end_channel_definition();
    }

    virtual void set_input( std::size_t index, std::ptrdiff_t relPtr ) {
        if( index == 0 )
            m_inputRelPtr = relPtr;
    }

    virtual void set_output( std::ptrdiff_t /*relPtr*/ ) {}

    virtual const frantic::channels::channel_map& get_output_map() const { return m_outMap; }

    virtual void apply( base_compiler::state& data ) const {
        // Convert and copy into the destination buffer.
        // NOTE: const_cast because channel_general_accessor is brokenly assuming a const channel_general_accessor
        // should return a const ptr. See const_iterator vs. iterator for
        //       the correct approach (hint: const pointer, versus pointer to const data).
        m_cvtFn(
            const_cast<char*>( m_outAccessor.get_channel_data_pointer( static_cast<ember_state&>( data ).outBuffer ) ),
            (char*)data.get_output_pointer( m_inputRelPtr ), m_outAccessor.arity() );
    }
};

void output_expression::internal_apply( const base_compiler::expression* _this, base_compiler::state& data ) {
    static_cast<const output_expression*>( _this )->output_expression::apply( data );
}
} // namespace

void ember_compiler::compile_output( expression_id exprID, const std::pair<expression_id, int>& inputValue,
                                     const frantic::tstring& channelName,
                                     const frantic::magma::magma_data_type& expectedType ) {
    const temporary_meta& input = this->get_value( inputValue.first, inputValue.second );

    if( !this->get_output_map().has_channel( channelName ) ) {
        // THROW_MAGMA_INTERNAL_ERROR();
        // I allowed this primarily for test writing where we don't care what the type is ...
        if( !m_outMap.channel_definition_complete() )
            m_outMap.end_channel_definition();

        m_outMap.append_channel( channelName, input.first.m_elementCount, input.first.m_elementType );
    }

    const frantic::channels::channel& ch = this->get_output_map()[channelName];

    if( input.first.m_elementCount != ch.arity() ) {
        throw magma_exception() << magma_exception::node_id( exprID ) << magma_exception::input_index( 0 )
                                << magma_exception::connected_id( inputValue.first )
                                << magma_exception::connected_output_index( inputValue.second )
                                << magma_exception::found_type( input.first )
                                << magma_exception::expected_type( expectedType )
                                << magma_exception::error_name( _T("Arity mismatch") );
    }

    std::unique_ptr<base_compiler::expression> result( new output_expression( input.first, ch ) );

    result->set_input( 0, input.second );

    this->register_expression( std::move( result ) );
}

void ember_compiler::compile( frantic::magma::nodes::magma_input_channel_node* node ) {
    if( node->get_channelName() != _T("Position") )
        throw magma_exception() << magma_exception::node_id( node->get_id() )
                                << magma_exception::property_name( _T("channelName") )
                                << magma_exception::error_name( _T("The only valid InputChannel is \"Position\"") );

    this->compile_input_position( node->get_id() );
}

void ember_compiler::compile( frantic::magma::nodes::magma_output_node* node ) {
    this->compile_output( node->get_id(), node->get_input( 0 ), node->get_channelName(), node->get_channelType() );
}

struct levelset_meta {
    enum { ARITY = 1 };
    typedef ember::levelset_functor type;
    typedef boost::mpl::vector<float( vec3 )> bindings;
};

struct levelset2_meta {
    enum { ARITY = 1 };
    typedef ember::levelset_functor2 type;
    typedef boost::mpl::vector<void( void*, vec3 )> bindings;
};

frantic::channels::channel_map levelset_functor2::s_outMap; // HACK This shouldn't be here.

void ember_compiler::compile_levelset( expression_id id,
                                       boost::shared_ptr<frantic::volumetrics::levelset::rle_level_set> levelSet,
                                       const frantic::graphics::transform4f& toWorldTM,
                                       const std::pair<expression_id, int>& posInput, bool doGradient ) {
    if( !doGradient ) {
        this->compile_impl<levelset_meta>(
            id,
            ember::levelset_functor( levelSet,
                                     toWorldTM.to_inverse() * this->get_context_data().get_world_transform() ),
            &this->get_value( posInput.first, posInput.second ), 1, 1 );
    } else {
        this->compile_impl<levelset2_meta>(
            id,
            ember::levelset_functor2( levelSet,
                                      toWorldTM.to_inverse() * this->get_context_data().get_world_transform() ),
            &this->get_value( posInput.first, posInput.second ), 1, 2 );
    }
}

class expression_functor : boost::noncopyable {
    ember_compiler m_expression;

  public:
    expression_functor() {}

    explicit expression_functor( ember_compiler& parent ) {
        m_expression.init_as_child_of( parent );
        m_expression.m_outMap.end_channel_definition();
    }

    expression_functor( const movable<expression_functor>& other ) {
        m_expression = boost::move( other.get().m_expression );
    }

    expression_functor& operator=( const movable<expression_functor>& other ) {
        m_expression = boost::move( other.get().m_expression );
        return *this;
    }

    inline void add_output( const frantic::tstring& name, frantic::magma::magma_interface::magma_id rootID,
                            int outputIndex ) {
        if( !m_expression.is_visited( rootID ) )
            m_expression.build_subtree( rootID );

        const base_compiler::temporary_meta& meta = m_expression.get_value( rootID, outputIndex );

        m_expression.m_outMap.append_channel( name, meta.first.m_elementCount, meta.first.m_elementType );
        m_expression.compile_output( -42, std::make_pair( rootID, outputIndex ), name, meta.first );
    }

    inline void add_output( const frantic::tstring& name, const frantic::magma::variant_t& constantValue ) {
        const frantic::magma::magma_data_type& type = frantic::magma::get_variant_type( constantValue );

        // Create a temporary output node that will write the resulting value into the output buffer.
        frantic::magma::nodes::magma_output_node tempNode;
        tempNode.set_channelName( name );
        tempNode.set_channelType( type );
        tempNode.set_id( frantic::magma::magma_interface::INVALID_ID );
        tempNode.set_input_default_value( 0, constantValue );

        // Make space in the output map.
        m_expression.m_outMap.append_channel( name, type.m_elementCount, type.m_elementType );
        m_expression.compile( &tempNode );
    }

    void set_expression( const movable<ember_compiler>& expr ) { m_expression = boost::move( expr.get() ); }

    // Access to the result layout.
    inline const frantic::channels::channel_map& get_output_map() const { return m_expression.get_output_map(); }

    void operator()( void* out, const vec3& pos ) const { m_expression.eval( (char*)out, pos ); }
};

namespace {
class gradient_functor : public boost::noncopyable {
    expression_functor m_subexpr;
    float m_delta;

    gradient_functor( const gradient_functor& rhs );
    gradient_functor& operator=( const gradient_functor& rhs );

  public:
    typedef vec3 return_type;
    typedef boost::mpl::vector<vec3> param_types;

  public:
    gradient_functor()
        : m_delta( 1e-3f ) {}

    gradient_functor( const movable<gradient_functor>& movableRef )
        : m_subexpr( make_movable( movableRef.get().m_subexpr ) ) {
        m_delta = movableRef.get().m_delta;
    }

    gradient_functor& operator=( const movable<gradient_functor>& movableRef ) {
        m_subexpr = make_movable( movableRef.get().m_subexpr );
        m_delta = movableRef.get().m_delta;
        return *this;
    }

    void set_delta( float delta ) { m_delta = delta; }

    void set_subexpression( const movable<expression_functor>& subexpr ) {
        m_subexpr = subexpr;

        const frantic::channels::channel_map& outMap = m_subexpr.get_output_map();

        if( outMap.channel_count() != 1u || outMap[0].arity() != 1u ||
            outMap[0].data_type() != frantic::channels::data_type_float32 ) {
            magma_data_type foundType;
            if( outMap.channel_count() >= 1u ) {
                foundType.m_elementType = outMap[0].data_type();
                foundType.m_elementCount = outMap[0].arity();
            }

            throw magma_exception() << magma_exception::input_index( 0 ) << magma_exception::found_type( foundType )
                                    << magma_exception::expected_type(
                                           frantic::magma::simple_compiler::traits<float>::get_type() )
                                    << magma_exception::error_name( _T("Invalid input to GradientNode") );
        }
    }

    vec3 operator()( const vec3& pos ) const {
        float temp1, temp2;
        float denom = 2.f * m_delta;

        m_subexpr( &temp1, pos + vec3( m_delta, 0, 0 ) );
        m_subexpr( &temp2, pos - vec3( m_delta, 0, 0 ) );
        float x = ( temp1 - temp2 ) / denom;

        m_subexpr( &temp1, pos + vec3( 0, m_delta, 0 ) );
        m_subexpr( &temp2, pos - vec3( 0, m_delta, 0 ) );
        float y = ( temp1 - temp2 ) / denom;

        m_subexpr( &temp1, pos + vec3( 0, 0, m_delta ) );
        m_subexpr( &temp2, pos - vec3( 0, 0, m_delta ) );
        float z = ( temp1 - temp2 ) / denom;

        return vec3( x, y, z );
    }
};
} // namespace

struct gradient_meta {
    enum { ARITY = 1 };
    typedef movable<gradient_functor> type;
    typedef boost::mpl::vector<vec3( vec3 )> bindings;
};

void ember_compiler::compile_gradient( expression_id id, const std::pair<expression_id, int>& fieldInput,
                                       const std::pair<expression_id, int>& posInput, float delta ) {
    expression_functor fieldFn( *this );

    try {
        fieldFn.add_output( _T("Value"), fieldInput.first, fieldInput.second );
    } catch( magma_exception& e ) {
        e << magma_exception::node_id( id );
        throw;
    }

    gradient_functor fn;
    fn.set_subexpression( frantic::magma::make_movable( fieldFn ) );
    fn.set_delta( delta );

    this->compile_impl<gradient_meta>( id, frantic::magma::make_movable( fn ),
                                       &this->get_value( posInput.first, posInput.second ), 1, 1 );
}

namespace {
class divergence_functor : public boost::noncopyable {
    expression_functor m_subexpr;
    float m_delta;

  public:
    typedef float return_type;
    typedef boost::mpl::vector<vec3> param_types;

  public:
    divergence_functor()
        : m_delta( 1e-3f ) {}

    divergence_functor( const movable<divergence_functor>& movableRef )
        : m_subexpr( frantic::magma::make_movable( movableRef.get().m_subexpr ) ) {
        m_delta = movableRef.get().m_delta;
    }

    divergence_functor& operator=( const movable<divergence_functor>& movableRef ) {
        m_subexpr = frantic::magma::make_movable( movableRef.get().m_subexpr );
        m_delta = movableRef.get().m_delta;
        return *this;
    }

    void set_delta( float delta ) { m_delta = delta; }

    void set_subexpression( const movable<expression_functor>& subexpr ) {
        m_subexpr = subexpr;

        const frantic::channels::channel_map& outMap = m_subexpr.get_output_map();

        if( outMap.channel_count() != 1u || outMap[0].arity() != 3u ||
            outMap[0].data_type() != frantic::channels::data_type_float32 ) {
            magma_data_type foundType;
            if( outMap.channel_count() >= 1u ) {
                foundType.m_elementType = outMap[0].data_type();
                foundType.m_elementCount = outMap[0].arity();
            }

            throw magma_exception() << magma_exception::input_index( 0 ) << magma_exception::found_type( foundType )
                                    << magma_exception::expected_type(
                                           frantic::magma::simple_compiler::traits<vec3>::get_type() )
                                    << magma_exception::error_name( _T("Invalid input to DivergenceNode") );
        }
    }

    float operator()( const vec3& pos ) const {
        vec3 temp1, temp2;
        float result;

        m_subexpr( &temp1, pos + vec3( m_delta, 0, 0 ) );
        m_subexpr( &temp2, pos - vec3( m_delta, 0, 0 ) );
        result = temp1.x - temp2.x;

        m_subexpr( &temp1, pos + vec3( 0, m_delta, 0 ) );
        m_subexpr( &temp2, pos - vec3( 0, m_delta, 0 ) );
        result += temp1.y - temp2.y;

        m_subexpr( &temp1, pos + vec3( 0, 0, m_delta ) );
        m_subexpr( &temp2, pos - vec3( 0, 0, m_delta ) );
        result += temp1.z - temp2.z;

        return result / ( 2.f * m_delta );
    }
};
} // namespace

struct divergence_meta {
    enum { ARITY = 1 };
    typedef movable<divergence_functor> type;
    typedef boost::mpl::vector<float( vec3 )> bindings;
};

void ember_compiler::compile_divergence( expression_id id, const std::pair<expression_id, int>& fieldInput,
                                         const std::pair<expression_id, int>& posInput, float delta ) {
    expression_functor fieldFn( *this );

    try {
        fieldFn.add_output( _T("Value"), fieldInput.first, fieldInput.second );
    } catch( magma_exception& e ) {
        e << magma_exception::node_id( id );
        throw;
    }

    divergence_functor fn;
    fn.set_subexpression( frantic::magma::make_movable( fieldFn ) );
    fn.set_delta( delta );

    this->compile_impl<divergence_meta>( id, frantic::magma::make_movable( fn ),
                                         &this->get_value( posInput.first, posInput.second ), 1, 1 );
}

namespace {
class curl_functor : public boost::noncopyable {
    expression_functor m_subexpr;
    float m_delta;

  public:
    typedef vec3 return_type;
    typedef boost::mpl::vector<vec3> param_types;

  public:
    curl_functor()
        : m_delta( 1e-3f ) {}

    curl_functor( const movable<curl_functor>& movableRef )
        : m_subexpr( make_movable( movableRef.get().m_subexpr ) ) {
        m_delta = movableRef.get().m_delta;
    }

    curl_functor& operator=( const movable<curl_functor>& movableRef ) {
        m_subexpr = make_movable( movableRef.get().m_subexpr );
        m_delta = movableRef.get().m_delta;
        return *this;
    }

    void set_delta( float delta ) { m_delta = delta; }

    void set_subexpression( const movable<expression_functor>& subexpr ) {
        m_subexpr = subexpr;

        const frantic::channels::channel_map& outMap = m_subexpr.get_output_map();

        if( outMap.channel_count() != 1u || outMap[0].arity() != 3u ||
            outMap[0].data_type() != frantic::channels::data_type_float32 ) {
            magma_data_type foundType;
            if( outMap.channel_count() >= 1u ) {
                foundType.m_elementType = outMap[0].data_type();
                foundType.m_elementCount = outMap[0].arity();
            }

            throw magma_exception() << magma_exception::input_index( 0 ) << magma_exception::found_type( foundType )
                                    << magma_exception::expected_type(
                                           frantic::magma::simple_compiler::traits<vec3>::get_type() )
                                    << magma_exception::error_name( _T("Invalid input to CurlNode") );
        }
    }

    vec3 operator()( const vec3& pos ) const {
        vec3 vals[6];

        m_subexpr( &vals[0], pos - vec3( m_delta, 0, 0 ) );
        m_subexpr( &vals[1], pos + vec3( m_delta, 0, 0 ) );
        m_subexpr( &vals[2], pos - vec3( 0, m_delta, 0 ) );
        m_subexpr( &vals[3], pos + vec3( 0, m_delta, 0 ) );
        m_subexpr( &vals[4], pos - vec3( 0, 0, m_delta ) );
        m_subexpr( &vals[5], pos + vec3( 0, 0, m_delta ) );

        float denom = 2.f * m_delta;
        float x = ( vals[3].z - vals[2].z - vals[5].y + vals[4].y ) / denom;
        float y = ( vals[5].x - vals[4].x - vals[1].z + vals[0].z ) / denom;
        float z = ( vals[1].y - vals[0].y - vals[3].x + vals[2].x ) / denom;

        return vec3( x, y, z );
    }
};
} // namespace

struct curl_meta {
    enum { ARITY = 1 };
    typedef movable<curl_functor> type;
    typedef boost::mpl::vector<vec3( vec3 )> bindings;
};

void ember_compiler::compile_curl( expression_id id, const std::pair<expression_id, int>& fieldInput,
                                   const std::pair<expression_id, int>& posInput, float delta ) {
    expression_functor fieldFn( *this );

    try {
        fieldFn.add_output( _T("Value"), fieldInput.first, fieldInput.second );
    } catch( magma_exception& e ) {
        e << magma_exception::node_id( id );
        throw;
    }

    curl_functor fn;
    fn.set_subexpression( frantic::magma::make_movable( fieldFn ) );
    fn.set_delta( delta );

    this->compile_impl<curl_meta>( id, frantic::magma::make_movable( fn ),
                                   &this->get_value( posInput.first, posInput.second ), 1, 1 );
}

struct advect_meta {
    enum { ARITY = 1 };
    typedef movable<advect_functor<expression_functor, expression_functor>> type;
    typedef boost::mpl::vector<void( void*, vec3 )> bindings;
};

void ember_compiler::compile_advect( expression_id id, const std::pair<expression_id, int>& fieldInput,
                                     const std::pair<expression_id, int>& velInput,
                                     const std::pair<expression_id, int>& posInput, float timestep ) {
    expression_functor fieldExpr( *this ), velocityFunctor( *this );
    advect_functor<expression_functor, expression_functor> fn;

    try {
        fieldExpr.add_output( _T("Value"), fieldInput.first, fieldInput.second );
        velocityFunctor.add_output( _T("Velocity"), velInput.first, velInput.second );

        fn.set_field( make_movable( fieldExpr ) );
        fn.set_velocity( make_movable( velocityFunctor ) );
        fn.set_timestep( timestep );
    } catch( magma_exception& e ) {
        e << magma_exception::node_id( id );
        throw;
    }

    this->compile_impl<advect_meta>( id, make_movable( fn ), &this->get_value( posInput.first, posInput.second ), 1,
                                     1 );
}

struct advect2_meta {
    enum { ARITY = 2 };
    typedef movable<advect2_functor<expression_functor, expression_functor>> type;
    typedef boost::mpl::vector<void( void*, int, vec3 )> bindings;
};

void ember_compiler::compile_advect( expression_id id, const std::pair<expression_id, int>& fieldInput,
                                     const std::pair<expression_id, int>& velInput,
                                     const std::pair<expression_id, int>& stepsInput,
                                     const std::pair<expression_id, int>& posInput, float timestep ) {
    expression_functor fieldExpr( *this ), velocityFunctor( *this );
    advect2_functor<expression_functor, expression_functor> fn;

    try {
        fieldExpr.add_output( _T("Value"), fieldInput.first, fieldInput.second );
        velocityFunctor.add_output( _T("Velocity"), velInput.first, velInput.second );

        fn.set_field( make_movable( fieldExpr ) );
        fn.set_velocity( make_movable( velocityFunctor ) );
        fn.set_timestep( timestep );
    } catch( magma_exception& e ) {
        e << magma_exception::node_id( id );
        throw;
    }

    const std::pair<magma_data_type, std::ptrdiff_t> inputs[] = {
        this->get_value( stepsInput.first, stepsInput.second ), this->get_value( posInput.first, posInput.second ) };

    this->compile_impl<advect2_meta>( id, make_movable( fn ), inputs, 2, 1 );
}

namespace {
template <class T>
class grid_functor {
    grid<T> m_grid;

    grid_functor( const grid_functor& rhs );
    grid_functor& operator=( const grid_functor& rhs );

  public:
    typedef T return_type;
    typedef boost::mpl::vector<vec3> param_types;

    grid_functor( int ( &bounds )[6], float spacing )
        : m_grid( bounds, spacing ) {}

    grid_functor( const movable<grid_functor>& rhs ) { m_grid.swap( rhs.get().m_grid ); }

    grid_functor& operator=( const movable<grid_functor>& rhs ) {
        m_grid.swap( rhs.get().m_grid );
        return *this;
    }

    void assign_grid_values( const frantic::volumetrics::field_interface& field ) { ember::assign( m_grid, field ); }

    void operator()( return_type& out, const vec3& pos ) const { out = ember::trilerp( m_grid, pos ); }
};
} // namespace

void ember_compiler::compile_grid_cache( expression_id id, const std::pair<expression_id, int>& fieldInput,
                                         const std::pair<expression_id, int>& posInput, int ( &bounds )[6],
                                         float spacing ) {
    // Check if we have already cached this grid.
    std::map<std::pair<expression_id, int>, boost::shared_ptr<frantic::volumetrics::field_interface>>::const_iterator
        it = m_cachedFunctions->find( std::make_pair( id, -1 ) );
    if( it == m_cachedFunctions->end() ) {
        boost::shared_ptr<frantic::volumetrics::field_interface> inputField =
            this->build_subtree_as_field( fieldInput, _T("Value") );

        const frantic::channels::channel_map& fieldMap = inputField->get_channel_map();

        if( fieldMap.channel_count() != 1 )
            THROW_MAGMA_INTERNAL_ERROR( id, fieldInput.first, fieldInput.second, fieldMap.channel_count() );

        if( fieldMap[0].data_type() != frantic::channels::data_type_float32 ) {
            frantic::magma::magma_data_type dt;
            dt.m_elementType = fieldMap[0].data_type();
            dt.m_elementCount = fieldMap[0].arity();

            throw magma_exception() << magma_exception::node_id( id ) << magma_exception::input_index( 0 )
                                    << magma_exception::found_type( dt )
                                    << magma_exception::error_name( _T("Input must Float or Vec3") );
        }

        if( fieldMap[0].arity() == 1 ) {
            boost::shared_ptr<discretized_field<float>> grid( new discretized_field<float>( bounds, spacing ) );
            grid->assign( *inputField );
            it = m_cachedFunctions
                     ->insert(
                         std::make_pair( std::make_pair( id, -1 ),
                                         boost::dynamic_pointer_cast<frantic::volumetrics::field_interface>( grid ) ) )
                     .first;
        } else if( fieldMap[0].arity() == 3 ) {
            boost::shared_ptr<discretized_field<vec3>> grid( new discretized_field<vec3>( bounds, spacing ) );
            grid->assign( *inputField );
            it = m_cachedFunctions
                     ->insert(
                         std::make_pair( std::make_pair( id, -1 ),
                                         boost::dynamic_pointer_cast<frantic::volumetrics::field_interface>( grid ) ) )
                     .first;
        } else {
            frantic::magma::magma_data_type dt;
            dt.m_elementType = fieldMap[0].data_type();
            dt.m_elementCount = fieldMap[0].arity();

            throw magma_exception() << magma_exception::node_id( id ) << magma_exception::input_index( 0 )
                                    << magma_exception::found_type( dt )
                                    << magma_exception::error_name( _T("Input must Float or Vec3") );
        }
    }

    this->compile_field( id, it->second, posInput );
}

void ember_compiler::compile_staggered_cache( expression_id id, const std::pair<expression_id, int>& fieldInput,
                                              const std::pair<expression_id, int>& posInput, int ( &bounds )[6],
                                              float spacing ) {
    // Check if we have already cached this grid.
    std::map<std::pair<expression_id, int>, boost::shared_ptr<frantic::volumetrics::field_interface>>::const_iterator
        it = m_cachedFunctions->find( std::make_pair( id, -1 ) );
    if( it == m_cachedFunctions->end() ) {
        boost::shared_ptr<frantic::volumetrics::field_interface> inputField =
            this->build_subtree_as_field( fieldInput, _T("Value") );

        const frantic::channels::channel_map& fieldMap = inputField->get_channel_map();

        if( fieldMap.channel_count() != 1 )
            THROW_MAGMA_INTERNAL_ERROR( id, fieldInput.first, fieldInput.second, fieldMap.channel_count() );

        if( fieldMap[0].data_type() != frantic::channels::data_type_float32 || fieldMap[0].arity() != 3 ) {
            frantic::magma::magma_data_type dt;
            dt.m_elementType = fieldMap[0].data_type();
            dt.m_elementCount = fieldMap[0].arity();

            throw magma_exception() << magma_exception::node_id( id ) << magma_exception::input_index( 0 )
                                    << magma_exception::expected_type(
                                           frantic::magma::simple_compiler::traits<vec3>::get_type() )
                                    << magma_exception::found_type( dt )
                                    << magma_exception::error_name( _T("Input must Vec3") );
        }

        boost::shared_ptr<staggered_discretized_field> grid( new staggered_discretized_field( bounds, spacing ) );
        grid->assign( *inputField );
        it =
            m_cachedFunctions
                ->insert( std::make_pair( std::make_pair( id, -1 ),
                                          boost::dynamic_pointer_cast<frantic::volumetrics::field_interface>( grid ) ) )
                .first;
    }

    this->compile_field( id, it->second, posInput );
}

void ember_compiler::compile_divergence_solve( expression_id id, const std::pair<expression_id, int>& velocityInput,
                                               const std::pair<expression_id, int>& posInput, int ( &bounds )[6],
                                               float spacing, float timestep ) {
    // Check if we have already cached this grid.
    std::map<std::pair<expression_id, int>, boost::shared_ptr<frantic::volumetrics::field_interface>>::const_iterator
        it = m_cachedFunctions->find( std::make_pair( id, -1 ) );
    if( it == m_cachedFunctions->end() ) {
        boost::shared_ptr<frantic::volumetrics::field_interface> inputField =
            this->build_subtree_as_field( velocityInput, _T("Value") );

        const frantic::channels::channel_map& fieldMap = inputField->get_channel_map();

        if( fieldMap.channel_count() != 1 )
            THROW_MAGMA_INTERNAL_ERROR( id, velocityInput.first, velocityInput.second, fieldMap.channel_count() );

        if( fieldMap[0].data_type() != frantic::channels::data_type_float32 || fieldMap[0].arity() != 3 ) {
            frantic::magma::magma_data_type dt;
            dt.m_elementType = fieldMap[0].data_type();
            dt.m_elementCount = fieldMap[0].arity();

            throw magma_exception() << magma_exception::node_id( id ) << magma_exception::input_index( 0 )
                                    << magma_exception::expected_type(
                                           frantic::magma::simple_compiler::traits<vec3>::get_type() )
                                    << magma_exception::found_type( dt )
                                    << magma_exception::error_name( _T("Input must Vec3") );
        }

        boost::shared_ptr<staggered_discretized_field> grid( new staggered_discretized_field( bounds, spacing ) );

        // Fill the grid by sampling the incoming field.
        grid->assign( *inputField );

        // Remove divergence from the grid using an FFT-based poisson solver.
        ember::do_poisson_solve( grid->get_grid(), timestep );

        it =
            m_cachedFunctions
                ->insert( std::make_pair( std::make_pair( id, -1 ),
                                          boost::dynamic_pointer_cast<frantic::volumetrics::field_interface>( grid ) ) )
                .first;
    }

    this->compile_field( id, it->second, posInput );
}

} // namespace ember
