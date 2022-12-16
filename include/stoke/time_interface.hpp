// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <boost/cstdint.hpp>

namespace stoke {

class time_interface {
  public:
    enum format {
        kPicoseconds,  // Value is in units of picoseconds (1x10^-6 seconds)
        kMilliseconds, // Value is in units of milliseconds (1x10^-3 seconds)
        kSeconds,      // Value is in units of seconds
        kFrames,       // Value is in units of frames (relies on global information ie. framerate)
        kNative // Value is in an unknown format native to the current executable. In 3ds Max this means Ticks (1/4800th
                // of a second).
    };

    /**
     * Returns the value in the specified format.
     */
    virtual double as( format requestedType ) const = 0;

    /**
     * Returns the value in the specified format, with fractional portions truncated. This method is generally useful if
     * the native format is an integral type and the native format is being requested.
     */
    virtual boost::int64_t as_int( format requestedType ) const = 0;
};

} // namespace stoke
