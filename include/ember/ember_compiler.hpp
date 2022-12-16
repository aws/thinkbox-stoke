// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <frantic/magma/magma_movable.hpp>
#include <frantic/magma/simple_compiler/base_compiler.hpp>

#include <frantic/volumetrics/field_interface.hpp>
#include <frantic/volumetrics/levelset/rle_level_set.hpp>

#include <boost/move/move.hpp>

namespace ember {

using frantic::magma::movable;

// Ember compiler class
//  TODO Move ember specific stuff into its own interface class if we need to have multiple implementations of
//  ember_compiler (ex. LLVM based).
class ember_compiler : public frantic::magma::simple_compiler::base_compiler {
  public:
    typedef frantic::volumetrics::field_interface field_type;
    typedef boost::shared_ptr<field_type> field_ptr;

  public:
    ember_compiler();

    ember_compiler( BOOST_RV_REF( ember_compiler ) rhs );

    virtual ~ember_compiler();

    ember_compiler& operator=( BOOST_RV_REF( ember_compiler ) rhs );

    const frantic::channels::channel_map& get_output_map() const;

    virtual void build();

    virtual field_ptr build_output_as_field( frantic::magma::nodes::magma_output_node& rootNode );

    virtual field_ptr build_subtree_as_field( const std::pair<expression_id, int>& nodeOutput,
                                              const frantic::tstring& outputName,
                                              const frantic::magma::magma_data_type* expectedType = NULL );

    field_ptr find_cached_field( expression_id exprID, int outputIndex = -1 ) const;

    void eval( char* outBuffer, const frantic::graphics::vector3f& pos ) const;

    void eval_debug( char* outBuffer, const frantic::graphics::vector3f& pos,
                     frantic::magma::debug_data& outDebugData ) const;

  public:
    /**
     * Registers an expression that returns the current position in the field that is being evaluated.
     * @exprID The id of the new expression to create.
     */
    virtual void compile_input_position( expression_id exprID );

    /**
     * Registers an expression that writes its input value to the result buffer (ie. the one passed to eval()). Can also
     * verify that the channel is a specific type.
     * @exprID The id of the new expression to create.
     * @inputValue The id & value index of the expression to write to the particle.
     * @channelName The name of the particle's channel to write to.
     * @expectedType The type that the particle's channel should be if it isn't already in the native channel map.
     */
    virtual void compile_output( expression_id exprID, const std::pair<expression_id, int>& inputValue,
                                 const frantic::tstring& channelName,
                                 const frantic::magma::magma_data_type& expectedType );

    /**
     * Exposes a level set's signed distance field.
     */
    virtual void compile_levelset( expression_id id,
                                   boost::shared_ptr<frantic::volumetrics::levelset::rle_level_set> levelSet,
                                   const frantic::graphics::transform4f& toWorldTM,
                                   const std::pair<expression_id, int>& posInput, bool doGradient = false );

    /**
     * Creates an registers an expression that calculates the gradient of the input scalar field. Uses finite
     * differences if the input field does not have its own analytic calculation.
     * @id The id to assign to the result of evaluating the divergence field
     * @fieldInput The scalar field to evaluate the gradient of
     * @posInput Where to evaluate the gradient in 'inputField'
     * @delta If finite differences are used, this is the offset from the center to evaluate.
     */
    virtual void compile_gradient( expression_id id, const std::pair<expression_id, int>& fieldInput,
                                   const std::pair<expression_id, int>& posInput, float delta );

    /**
     * Creates an registers an expression that calculates the divergence of the input vector field. Uses finite
     * differences if the input field does not have its own analytic calculation.
     * @id The id to assign to the result of evaluating the divergence field
     * @fieldInput The vector field to evaluate the divergence of
     * @posInput Where to evaluate the divergence in 'inputField'
     * @delta If finite differences are used, this is the offset from the center to evaluate.
     */
    virtual void compile_divergence( expression_id id, const std::pair<expression_id, int>& fieldInput,
                                     const std::pair<expression_id, int>& posInput, float delta );

    /**
     * Creates an registers an expression that calculates the curl of the input vector field. Uses finite differences if
     * the input field does not have its own analytic calculation.
     * @id The id to assign to the result of evaluating the curl field
     * @fieldInput The vector field to evaluate the curl of
     * @posInput Where to evaluate the curl in 'inputField'
     * @delta If finite differences are used, this is the offset from the center to evaluate.
     */
    virtual void compile_curl( expression_id id, const std::pair<expression_id, int>& fieldInput,
                               const std::pair<expression_id, int>& posInput, float delta );

    /**
     * Creates an registers an expression that computes the sem-lagrangian advection of 'fieldInput' by the velocity
     * field 'velInput'. This essentially traces a path backwards through 'velInput' and returns the result of
     * 'fieldInput' at the end of the path. Uses a single RK2 step.
     * @id The id to assign to the result of advecting 'fieldInput'.
     * @fieldInput The field to advect
     * @velInput The velocity to advect 'fieldInput' by.
     * @posInput Where to evaluate the advected 'fieldInput'.
     * @timestep The amount of time to advect by. This controls the length of the path traced through 'velInput'.
     */
    virtual void compile_advect( expression_id id, const std::pair<expression_id, int>& fieldInput,
                                 const std::pair<expression_id, int>& velInput,
                                 const std::pair<expression_id, int>& posInput, float timestep );

    /**
     * @overload
     * This version allows 'stepsInput' consecutive paths to be traced through 'velInput'. Each path segment is
     * 'timestep' seconds. This is equivalent to chaining multiple advections with 1 step.
     */
    virtual void compile_advect( expression_id id, const std::pair<expression_id, int>& fieldInput,
                                 const std::pair<expression_id, int>& velInput,
                                 const std::pair<expression_id, int>& stepsInput,
                                 const std::pair<expression_id, int>& posInput, float timestep );

    /**
     * Evaluates the specified field at all points on a regulat grid with the given dimensions, then creates and
     * registers an expression that reconstructs the continous field between gridpoints using trilinear interpolation,
     * at the given input position.
     * @id The id to assign to the result of reconstructing the cached grid.
     * @fieldInput The field to sample onto the grid.
     * @posInput Where to reconstruct and evaluate the sampled field from the grid.
     * @bounds The integer bounding rectangle for the grid. Packed as -X,+X,-Y,+Y,-Z,+Z. Multiplying by 'spacing'
     * produces the non-voxelized object space coordinates.
     * @spacing The distance between grid points.
     */
    virtual void compile_grid_cache( expression_id id, const std::pair<expression_id, int>& fieldInput,
                                     const std::pair<expression_id, int>& posInput, int ( &bounds )[6], float spacing );

    /**
     * Evaluates the specified vector field at all points on a staggered grid with the given dimensions, then creates
     * and registers an expression that reconstructs the continous field between gridpoints using trilinear
     * interpolation on each staggered dimension, at the given input position.
     * @note This is also known as a MAC grid.
     * @id The id to assign to the result of reconstructing the cached grid.
     * @fieldInput The field to sample onto the grid.
     * @posInput Where to reconstruct and evaluate the sampled field from the grid.
     * @bounds The integer bounding rectangle for the grid. Packed as -X,+X,-Y,+Y,-Z,+Z. See class ember::staggered_grid
     * for more details.
     * @spacing The distance between grid points.
     */
    virtual void compile_staggered_cache( expression_id id, const std::pair<expression_id, int>& fieldInput,
                                          const std::pair<expression_id, int>& posInput, int ( &bounds )[6],
                                          float spacing );

    /**
     * Takes a velocity field, samples it onto a staggered grid, removes divergence with a linear solver, then
     * reconstructs the continuous velocity field at the specified position.
     * @id The id to assign to the result of reconstructing the cached grid.
     * @fieldInput The field to sample onto the grid.
     * @posInput Where to reconstruct and evaluate the sampled field from the grid.
     * @bounds The integer bounding rectangle for the grid. Packed as -X,+X,-Y,+Y,-Z,+Z. See class ember::staggered_grid
     * for more details.
     * @spacing The distance between grid points.
     */
    virtual void compile_divergence_solve( expression_id id, const std::pair<expression_id, int>& velocityInput,
                                           const std::pair<expression_id, int>& posInput, int ( &bounds )[6],
                                           float spacing, float timestep );

    // virtual void compile_field( expression_id id, boost::shared_ptr<frantic::volumetrics::field_interface> field,
    // const std::pair<expression_id,int>& posInput );

    // virtual void compile_field( expression_id id, boost::shared_ptr<frantic::volumetrics::field_interface> field,
    // const std::pair<expression_id,int>& posInput, const std::vector<frantic::tstring>& reorderedChannels );

  public:
    virtual void compile( frantic::magma::nodes::magma_input_channel_node* );

    virtual void compile( frantic::magma::nodes::magma_output_node* );

  protected:
    /**
     * This class is used to store an instance (or subclass instance) of ember_compiler, exposed through
     * frantic::volumetrics::field_interface.
     */
    class expression_field : public frantic::volumetrics::field_interface {
      public:
        virtual void add_output( frantic::magma::magma_interface::magma_id nodeID, int outputIndex,
                                 const frantic::tstring& outputName = _T("Value") ) = 0;
    };

    class expression_field_impl;

    /**
     * This function creates an expression_field instance that is bound to the same context & magma nodes as 'this'. The
     * results of individual subtrees can be exposed by calling expression_field::add_output() on the returned object.
     * @return a new instance of expression_field, roughly equivalent to the result of this->reset_expression().
     */
    virtual std::unique_ptr<expression_field> create_empty_expression_field();

    virtual void init_as_child_of( ember_compiler& parent );

    void append_output_channel( const frantic::tstring& channelName, std::size_t arity,
                                frantic::channels::data_type_t type );

    expression_id get_position_expression_id();

  private:
    BOOST_MOVABLE_BUT_NOT_COPYABLE( ember_compiler );

  private:
    frantic::channels::channel_map m_outMap;

    boost::shared_ptr<std::map<std::pair<expression_id, int>, field_ptr>>
        m_cachedFunctions; // Shared to allow init_as_child_of to share same container.

    expression_id m_singletonPosExpr; // We only need a single position input for the expression, so we store it here.

    friend class expression_functor;
    friend class expression_field;
};

inline const frantic::channels::channel_map& ember_compiler::get_output_map() const { return m_outMap; }

} // namespace ember
