// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#pragma once

#if defined( FIELD3D_AVAILABLE )
#include <ember/concatenated_field.hpp>

#define WIN32

#pragma warning( push, 3 )
#pragma warning( disable : 4251 4244 4101 )
#include <Field3D/Field.h>
#pragma warning( pop )

namespace ember {

class field3d_field_interface : public frantic::volumetrics::field_interface {
  public:
    virtual ~field3d_field_interface() {}

    virtual Field3D::FieldRes& get_field() const = 0;

    virtual Field3D::FieldRes::Ptr get_field_ptr() const = 0;

    virtual std::size_t get_memory_usage() const = 0;
};

boost::shared_ptr<concatenated_field> load_fields_from_file( const frantic::tstring& filePath );

struct field3d_meta {
    frantic::channels::channel_map map;
    frantic::graphics::boundbox3f bounds;
    float spacing;
    std::size_t memoryUsage;
};

// TODO: Convert this to an asynchronous task!
boost::shared_ptr<concatenated_field> load_fields_from_file( const frantic::tstring& filePath,
                                                             field3d_meta& outMetadata );

void sample_to_field3d( const frantic::volumetrics::field_interface& field,
                        std::vector<Field3D::FieldRes::Ptr>& outFields, const Field3D::Box3i& extents,
                        const Field3D::Box3i& dataWindow, float spacing );

void write_fields_to_file( const frantic::tstring& filePath, const std::vector<Field3D::FieldRes::Ptr>& fields );

// void write_fields_to_file( const frantic::tstring& filePath, const std::vector<
// boost::shared_ptr<frantic::volumetrics::field_interface> >& fields, Field3D::Box3i& extents, Field3D::Box3i&
// dataWindow, float spacing );

// void write_fields_to_file( const frantic::tstring& filePath, const boost::shared_ptr<concatenated_field>& fields,
// Field3D::Box3i& extents, Field3D::Box3i& dataWindow, float spacing );

} // namespace ember
#endif
