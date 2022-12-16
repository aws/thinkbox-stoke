// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#include "stdafx.h"

#include <stoke/id_generator.hpp>

namespace stoke {

id_generator::id_generator() { m_nextID = 0; }

id_generator::~id_generator() {}

id_generator::id_type id_generator::allocate_ids( std::size_t numIDs ) {
    return m_nextID.fetch_and_add( static_cast<id_type>( numIDs ) );
}

} // namespace stoke
