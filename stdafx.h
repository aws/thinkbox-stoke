// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
// stdafx.h : include file for standard system include files,
// or project specific include files that are used frequently, but
// are changed infrequently
//

#pragma once

#define WIN32_LEAN_AND_MEAN // Exclude rarely-used stuff from Windows headers
#define NOMINMAX

#include <frantic/strings/tstring.hpp>

#pragma warning( push, 3 )
#pragma warning( disable : 4512 )
#include <boost/config.hpp>
#include <boost/function.hpp>
#include <boost/random.hpp>
#pragma warning( pop )

// TODO: reference additional headers your program requires here
