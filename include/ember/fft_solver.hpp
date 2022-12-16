// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <ember/simulation.hpp>

namespace ember {

class fft_solver : public simulation_solver {
  public:
    fft_solver();

    virtual ~fft_solver() {}

    struct boundary_type {
        enum enum_t { dirchlet, neumann };
    };

    struct boundary_id {
        enum enum_t { negative_x, positive_x, negative_y, positive_y, negative_z, positive_z };
    };

    void set_boundary_condition( boundary_id::enum_t boundaryID, boundary_type::enum_t type );

    virtual void solve( staggered_grid& grid, float timeStepSeconds );

  private:
    char m_boundaryFlags[6];
};

} // namespace ember
