// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <stoke/id_generator_interface.hpp>

// TODO: Switch to C++ 11 atomics when possible.
#pragma warning( push, 3 )
#include <tbb/atomic.h>
#pragma warning( pop )

namespace stoke {

class id_generator : public id_generator_interface {
  public:
    id_generator();

    virtual ~id_generator();

    virtual id_type allocate_ids( std::size_t numIDs );

  private:
    tbb::atomic<id_type> m_nextID;
};

} // namespace stoke
