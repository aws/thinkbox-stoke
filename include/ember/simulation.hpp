// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <boost/shared_ptr.hpp>
#include <frantic/magma/magma_compiler_interface.hpp>

// Forward decl.
namespace frantic {
namespace volumetrics {
class field_interface;
}

namespace magma {
class magma_interface;
}
} // namespace frantic

namespace ember {

class simulation_context;
class simulation_solver;

class simulation {
  public:
    static std::unique_ptr<simulation> create_instance();

    virtual ~simulation() {}

    virtual void set_initial_simulation_expression( boost::shared_ptr<frantic::magma::magma_interface> magma ) = 0;
    virtual void set_simulation_expression( boost::shared_ptr<frantic::magma::magma_interface> magma ) = 0;
    virtual void set_simulation_context( boost::shared_ptr<simulation_context> ctx ) = 0;

    virtual void set_timestep( float step ) = 0;
    virtual void set_domain( int ( &bounds )[6], float spacing ) = 0;
    virtual void set_velocity_domain( int ( &bounds )[6], float spacing ) = 0;
    virtual void set_discretize( bool enabled = true ) = 0;
    virtual void set_velocity_solver( boost::shared_ptr<simulation_solver> solver ) = 0;
    // virtual void set_divergence_free( bool enabled = true ) = 0;

    virtual void reset_simulation() = 0;
    virtual void simulate( float endTime ) = 0;

    virtual boost::shared_ptr<frantic::volumetrics::field_interface> get_current_state() = 0;
    virtual boost::shared_ptr<frantic::volumetrics::field_interface>
    get_current_state( const frantic::tstring& channelName ) = 0;
};

class simulation_context : public frantic::magma::magma_compiler_interface::context_base {
  public:
    // The simulation must make a copy of the context for each time that it going to evaluate itself.
    // TODO: Can I analyze the use of the context and determine its OK to change the single context object? For example,
    // TexmapEval uses the time but only when compiled. It
    //       does not store a reference to the context so its probably ok to change the context later.
    virtual std::unique_ptr<simulation_context> clone_for_simulation_time( float simTime ) const = 0;
};

class staggered_grid;

class simulation_solver {
  public:
    virtual ~simulation_solver() {}

    virtual void solve( staggered_grid& grid, float timeStepSeconds ) = 0;
};

} // namespace ember
