// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#include <frantic/graphics/transform4f.hpp>
#include <frantic/volumetrics/field_interface.hpp>

namespace ember {

// Some fields need to be transformed, and this class uses the decorator pattern to hide that from the eventual client.
// TODO: We should improve field_interface to allow clients aware of this class to extract the non-transformed field.
// TODO: We should annotate the channel_map with metadata for applying transformations to the output value(s).
class transformed_field : public frantic::volumetrics::field_interface {
  public:
    explicit transformed_field( boost::shared_ptr<frantic::volumetrics::field_interface> pDelegateField );

    transformed_field( boost::shared_ptr<frantic::volumetrics::field_interface> pDelegateField,
                       const frantic::graphics::transform4f& toWorldTM );

    void set_toworld_transform( const frantic::graphics::transform4f& toWorldTM );

    const frantic::graphics::transform4f& get_toworld_transform() const;

    virtual bool evaluate_field( void* dest, const frantic::graphics::vector3f& pos ) const;

    virtual const frantic::channels::channel_map& get_channel_map() const;

  private:
    boost::shared_ptr<frantic::volumetrics::field_interface> m_pDelegateField;

    frantic::graphics::transform4f m_toWorldTM, m_fromWorldTM;

    // TODO: We should figure out which channels need to be transformed. Like a Velocity field, etc.
    // TODO: Decompose the transformation into the various sub-components that are used on different geometric types.
    // ex. rotation only on orientation, or
    //       inverse-transpose for normals.
};

inline transformed_field::transformed_field( boost::shared_ptr<frantic::volumetrics::field_interface> pDelegateField )
    : m_pDelegateField( pDelegateField ) {}

inline transformed_field::transformed_field( boost::shared_ptr<frantic::volumetrics::field_interface> pDelegateField,
                                             const frantic::graphics::transform4f& toWorldTM )
    : m_pDelegateField( pDelegateField )
    , m_toWorldTM( toWorldTM )
    , m_fromWorldTM( toWorldTM.to_inverse() ) {}

void transformed_field::set_toworld_transform( const frantic::graphics::transform4f& toWorldTM ) {
    m_toWorldTM = toWorldTM;
    m_fromWorldTM = toWorldTM.to_inverse();
}

const frantic::graphics::transform4f& transformed_field::get_toworld_transform() const { return m_toWorldTM; }

bool transformed_field::evaluate_field( void* dest, const frantic::graphics::vector3f& pos ) const {
    return m_pDelegateField->evaluate_field( dest, m_fromWorldTM * pos );

    // TODO: Transform appropriate values into world-space.
}

const frantic::channels::channel_map& transformed_field::get_channel_map() const {
    return m_pDelegateField->get_channel_map();
}

} // namespace ember
