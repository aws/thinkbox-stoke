// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <boost/cstdint.hpp>
#include <boost/shared_ptr.hpp>

namespace stoke {

class id_generator_interface {
  public:
    typedef boost::int64_t id_type;

  public:
    virtual ~id_generator_interface() {}

    /**
     * Allocates a contiguous block of IDs as an atomic operation.
     * @param numIDs The number of contiguous IDs to allocate.
     * @return The first ID in the range. The caller is able to use this, as well as the next 'numIDs' IDs.
     */
    virtual id_type allocate_ids( std::size_t numIDs ) = 0;
};

typedef boost::shared_ptr<id_generator_interface> id_generator_interface_ptr;

} // namespace stoke
