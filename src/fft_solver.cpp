// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#include "stdafx.h"

#include <ember/advection.hpp>
#include <ember/fft_solver.hpp>

namespace ember {

fft_solver::fft_solver() {
    m_boundaryFlags[boundary_id::negative_x] = m_boundaryFlags[boundary_id::positive_x] =
        m_boundaryFlags[boundary_id::negative_y] = m_boundaryFlags[boundary_id::positive_y] =
            m_boundaryFlags[boundary_id::negative_z] = m_boundaryFlags[boundary_id::positive_z] = 'D';
}

void fft_solver::set_boundary_condition( boundary_id::enum_t boundaryID, boundary_type::enum_t type ) {
    switch( type ) {
    case boundary_type::dirchlet:
        m_boundaryFlags[boundaryID] = 'D';
        break;
    case boundary_type::neumann:
        m_boundaryFlags[boundaryID] = 'N';
        break;
    };
}

void fft_solver::solve( staggered_grid& grid, float timeStepSeconds ) {
    do_poisson_solve( grid, timeStepSeconds, m_boundaryFlags );
}

} // namespace ember
