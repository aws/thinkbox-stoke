// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#include "stdafx.h"

#if defined( FIELD3D_AVAILABLE )

#include <ember/field3d.hpp>

#include <frantic/logging/logging_level.hpp>

#pragma warning( push, 3 )
#pragma warning( disable : 4251 4244 4101 )
#include <Field3D/DenseField.h>
#include <Field3D/Field3DFile.h>
#include <Field3D/FieldInterp.h>
#include <Field3D/InitIO.h>
#include <Field3D/MACField.h>
#pragma warning( pop )

#include <boost/make_shared.hpp>

namespace ember {

namespace {

template <class FieldType, class InterpType>
class field3d_field_impl : public field3d_field_interface {
  public:
    field3d_field_impl( typename FieldType::Ptr pField );

    virtual Field3D::FieldRes& get_field() const;

    virtual Field3D::FieldRes::Ptr get_field_ptr() const;

    virtual std::size_t get_memory_usage() const;

    virtual bool evaluate_field( void* dest, const frantic::graphics::vector3f& pos ) const;

    virtual const frantic::channels::channel_map& get_channel_map() const;

  private:
    typedef typename InterpType::value_type value_type;

    typename FieldType::Ptr m_impl;
    InterpType m_interp;

    frantic::channels::channel_map m_map;
};

template <class T>
struct output_type {
    typedef float type;
};

template <class T>
struct output_type<Imath::Vec3<T>> {
    typedef frantic::graphics::vector3f type;
};

template <class F, class I>
field3d_field_impl<F, I>::field3d_field_impl( typename F::Ptr pField )
    : m_impl( pField ) {
    m_map.define_channel<typename output_type<value_type>::type>( frantic::strings::to_tstring( pField->attribute ) );
    m_map.end_channel_definition();
}

template <class F, class I>
Field3D::FieldRes& field3d_field_impl<F, I>::get_field() const {
    return *m_impl;
}

template <class F, class I>
Field3D::FieldRes::Ptr field3d_field_impl<F, I>::get_field_ptr() const {
    return m_impl;
}

template <class F, class I>
std::size_t field3d_field_impl<F, I>::get_memory_usage() const {
    return static_cast<std::size_t>( m_impl->memSize() );
}

template <class T>
struct output_cast {
    typedef float type;
};

template <class T>
struct output_cast<Imath::Vec3<T>> {
    typedef Imath::Vec3<float> type;
};

template <class F, class I>
bool field3d_field_impl<F, I>::evaluate_field( void* dest, const frantic::graphics::vector3f& pos ) const {
    Field3D::V3d vp;

    m_impl->mapping()->worldToVoxel( Field3D::V3d( pos.x, pos.y, pos.z ), vp );

    *static_cast<value_type*>( dest ) =
        static_cast<typename output_cast<value_type>::type>( m_interp.sample( *m_impl, vp ) );

    return true;
}

template <class F, class I>
const frantic::channels::channel_map& field3d_field_impl<F, I>::get_channel_map() const {
    return m_map;
}

void dump_file( const Field3D::Field3DInputFile& file, const std::string& filePath ) {
    std::stringstream ss;

    std::vector<std::string> partitionNames;
    std::vector<std::string> scalarLayerNames, vectorLayerNames;

    file.getPartitionNames( partitionNames );

    ss << "File: \"" << filePath << std::endl;

    for( std::vector<std::string>::const_iterator it = partitionNames.begin(), itEnd = partitionNames.end();
         it != itEnd; ++it ) {
        ss << "\tPartition: " << *it << std::endl;

        // Field3D::File::Partition::Ptr pPart = file.getPartition( *it );
        // if( !pPart )
        //	continue;

        scalarLayerNames.clear();
        vectorLayerNames.clear();

        file.getScalarLayerNames( scalarLayerNames, *it );
        file.getVectorLayerNames( vectorLayerNames, *it );

        for( std::vector<std::string>::const_iterator it = scalarLayerNames.begin(), itEnd = scalarLayerNames.end();
             it != itEnd; ++it )
            ss << "\t\tScalar layer: " << *it << std::endl;

        for( std::vector<std::string>::const_iterator it = vectorLayerNames.begin(), itEnd = vectorLayerNames.end();
             it != itEnd; ++it )
            ss << "\t\tVector layer: " << *it << std::endl;
    }

    FF_LOG( debug ) << frantic::strings::to_tstring( ss.str() );
};

template <class T>
boost::shared_ptr<field3d_field_interface> process_scalar_field( const typename Field3D::Field<T>::Ptr& pField ) {
    if( Field3D::DenseField<T>::Ptr pDense = Field3D::field_dynamic_cast<Field3D::DenseField<T>>( pField ) )
        return boost::make_shared<field3d_field_impl<Field3D::DenseField<T>, Field3D::DenseField<T>::LinearInterp>>(
            pDense );
    else
        return boost::make_shared<field3d_field_impl<Field3D::Field<T>, Field3D::LinearFieldInterp<T>>>( pField );
}

template <class T>
void process_scalar_fields( Field3D::Field3DInputFile& file, concatenated_field& resultField, field3d_meta& outMeta ) {
    Field3D::Field<T>::Vec scalarFields = file.readScalarLayers<T>();

    for( Field3D::Field<T>::Vec::const_iterator it = scalarFields.begin(), itEnd = scalarFields.end(); it != itEnd;
         ++it ) {
        boost::shared_ptr<field3d_field_interface> pField = process_scalar_field<T>( *it );

        const Field3D::FieldRes& impl = pField->get_field();

        Field3D::V3d wsMin, wsMax;
        impl.mapping()->voxelToWorld(
            static_cast<Field3D::V3d>( static_cast<Field3D::V3d>( impl.dataWindow().min ) + Field3D::V3d( 0.5 ) ),
            wsMin );
        impl.mapping()->voxelToWorld(
            static_cast<Field3D::V3d>( static_cast<Field3D::V3d>( impl.dataWindow().max ) + Field3D::V3d( 0.5 ) ),
            wsMax );

        outMeta.bounds += frantic::graphics::vector3f( static_cast<float>( wsMin.x ), static_cast<float>( wsMin.y ),
                                                       static_cast<float>( wsMin.z ) );
        outMeta.bounds += frantic::graphics::vector3f( static_cast<float>( wsMax.x ), static_cast<float>( wsMax.y ),
                                                       static_cast<float>( wsMax.z ) );
        outMeta.spacing = std::min(
            outMeta.spacing,
            static_cast<float>(
                impl.mapping()->wsVoxelSize( impl.extents().min.x, impl.extents().min.y, impl.extents().min.z ).x ) );
        outMeta.memoryUsage += static_cast<std::size_t>( impl.memSize() );
        outMeta.map.define_channel<float>( frantic::strings::to_tstring( impl.attribute ) );

        resultField.add_field( pField );
    }
}

template <class T>
boost::shared_ptr<field3d_field_interface>
process_vector_field( const typename Field3D::Field<Imath::Vec3<T>>::Ptr& pField ) {
    if( Field3D::DenseField<Imath::Vec3<T>>::Ptr pDense =
            Field3D::field_dynamic_cast<Field3D::DenseField<Imath::Vec3<T>>>( pField ) )
        return boost::make_shared<
            field3d_field_impl<Field3D::DenseField<Imath::Vec3<T>>, Field3D::DenseField<Imath::Vec3<T>>::LinearInterp>>(
            pDense );
    else
        return boost::make_shared<
            field3d_field_impl<Field3D::Field<Imath::Vec3<T>>, Field3D::LinearFieldInterp<Imath::Vec3<T>>>>( pField );
}

template <class T>
void process_vector_fields( Field3D::Field3DInputFile& file, concatenated_field& resultField, field3d_meta& outMeta ) {
    Field3D::Field<Imath::Vec3<T>>::Vec vectorFields = file.readVectorLayers<T>();

    for( Field3D::Field<Imath::Vec3<T>>::Vec::const_iterator it = vectorFields.begin(), itEnd = vectorFields.end();
         it != itEnd; ++it ) {
        boost::shared_ptr<field3d_field_interface> pField = process_vector_field<T>( *it );

        const Field3D::FieldRes& impl = pField->get_field();

        Field3D::V3d wsMin, wsMax;
        impl.mapping()->voxelToWorld(
            static_cast<Field3D::V3d>( static_cast<Field3D::V3d>( impl.dataWindow().min ) + Field3D::V3d( 0.5 ) ),
            wsMin );
        impl.mapping()->voxelToWorld(
            static_cast<Field3D::V3d>( static_cast<Field3D::V3d>( impl.dataWindow().max ) + Field3D::V3d( 0.5 ) ),
            wsMax );

        outMeta.bounds += frantic::graphics::vector3f( static_cast<float>( wsMin.x ), static_cast<float>( wsMin.y ),
                                                       static_cast<float>( wsMin.z ) );
        outMeta.bounds += frantic::graphics::vector3f( static_cast<float>( wsMax.x ), static_cast<float>( wsMax.y ),
                                                       static_cast<float>( wsMax.z ) );
        outMeta.spacing = std::min(
            outMeta.spacing,
            static_cast<float>(
                impl.mapping()->wsVoxelSize( impl.extents().min.x, impl.extents().min.y, impl.extents().min.z ).x ) );
        outMeta.memoryUsage += static_cast<std::size_t>( impl.memSize() );
        outMeta.map.define_channel<frantic::graphics::vector3f>( frantic::strings::to_tstring( impl.attribute ) );

        resultField.add_field( pField );
    }
}

template <class TDest, class TSrc>
struct convert {
    inline static TDest apply( const TSrc& src ) { return static_cast<TDest>( src ); }
};

template <>
struct convert<Field3D::V3f, frantic::graphics::vector3f> {
    inline static Field3D::V3f apply( const frantic::graphics::vector3f& src ) {
        return Field3D::V3f( src.x, src.y, src.z );
    }
};

template <class TDest, class TSrc>
struct dense_grid_writer {
    static void write( Field3D::FieldRes& outField_, const void* srcData, const Field3D::V3i& pos ) {
        static_cast<Field3D::DenseField<TDest>&>( outField_ ).fastLValue( pos.x, pos.y, pos.z ) =
            convert<TDest, TSrc>::apply( *static_cast<const TSrc*>( srcData ) );
    }
};

template <class TDest, class TSrc>
struct generic_grid_writer {
    static void write( Field3D::FieldRes& outField_, const void* srcData, const Field3D::V3i& pos ) {
        static_cast<Field3D::WritableField<TDest>&>( outField_ ).lvalue( pos.x, pos.y, pos.z ) =
            convert<TDest, TSrc>::apply( *static_cast<const TSrc*>( srcData ) );
    }
};

template <class TDest, class TSrc>
typename Field3D::DenseField<TDest>::Ptr convert_to_field3d( const frantic::volumetrics::field_interface& field,
                                                             const Field3D::Box3i& extents,
                                                             const Field3D::Box3i& dataWindow, float spacing ) {
    Field3D::DenseField<TDest>::Ptr result( new Field3D::DenseField<TDest> );

    result->setSize( extents, dataWindow );

    if( spacing != 1.f ) {
        Field3D::M44d tm;
        tm.setScale( Field3D::V3f( spacing * ( extents.max.x - extents.min.x + 1 ),
                                   spacing * ( extents.max.y - extents.min.y + 1 ),
                                   spacing * ( extents.max.z - extents.min.z + 1 ) ) );

        Field3D::MatrixFieldMapping::Ptr map( new Field3D::MatrixFieldMapping );

        map->setExtents( extents );
        map->setLocalToWorld( tm );

        result->setMapping( map );
    }

    TSrc temp;
    for( Field3D::DenseField<TDest>::iterator it = result->begin(), itEnd = result->end(); it != itEnd; ++it ) {
        Imath::V3d wp, vp( it.x, it.y, it.z );

        result->mapping()->voxelToWorld( vp, wp );

        if( field.evaluate_field( &temp,
                                  frantic::graphics::vector3f( static_cast<float>( wp.x ), static_cast<float>( wp.y ),
                                                               static_cast<float>( wp.z ) ) ) )
            *it = static_cast<TDest>( temp );
    }

    return result;
}

struct scoped_redirect {
    std::ostream* pStream;
    std::streambuf* pOldBuf;

    scoped_redirect( std::ostream& stream, std::streambuf* newBuf )
        : pStream( &stream ) {
        pStream->flush();

        pOldBuf = pStream->rdbuf( newBuf );
    }

    ~scoped_redirect() { pStream->rdbuf( pOldBuf ); }
};

} // End of anonymous namespace

boost::shared_ptr<concatenated_field> load_fields_from_file( const frantic::tstring& filePath ) {
    field3d_meta meta;

    return load_fields_from_file( filePath, meta );
}

boost::shared_ptr<concatenated_field> load_fields_from_file( const frantic::tstring& filePath, field3d_meta& outMeta ) {
    std::stringstream ssout;
    scoped_redirect redir( std::cout, ssout.rdbuf() );

    Field3D::Field3DInputFile file;

    boost::shared_ptr<concatenated_field> outField = boost::make_shared<concatenated_field>();

    std::string realPath = frantic::strings::to_string( filePath );

    bool result = file.open( realPath );

    if( !result ) {
        FF_LOG( error ) << _T("Failed to load file: ") << filePath << std::endl;
        FF_LOG( error ) << _T("COUT:") << std::endl << frantic::strings::to_tstring( ssout.str() ) << std::endl;

        throw std::runtime_error( "load_fields_from_file() Failed to load file: \"" + realPath + "\"" );
    }

    try {
        dump_file( file, realPath );

        outMeta.bounds.set_to_empty();
        outMeta.map.reset();
        outMeta.memoryUsage = 0;
        outMeta.spacing = std::numeric_limits<float>::max();

        process_scalar_fields<float>( file, *outField, outMeta );
        process_scalar_fields<half>( file, *outField, outMeta );
        process_scalar_fields<double>( file, *outField, outMeta );

        process_vector_fields<float>( file, *outField, outMeta );
        process_vector_fields<half>( file, *outField, outMeta );
        process_vector_fields<double>( file, *outField, outMeta );

        outMeta.map.end_channel_definition();
        if( outMeta.spacing == std::numeric_limits<float>::max() )
            outMeta.spacing = 0.f;

        result = file.close();
        if( !result ) {
            FF_LOG( error ) << _T("Failed to close opened file: ") << filePath << std::endl;
            FF_LOG( error ) << _T("COUT:") << std::endl << frantic::strings::to_tstring( ssout.str() ) << std::endl;

            throw std::runtime_error( "load_fields_from_file() Failed to close file: \"" + realPath +
                                      "\" after load." );
        }
    } catch( ... ) {
        file.close();
        throw;
    }

    return outField;
}

void sample_to_field3d( const frantic::volumetrics::field_interface& field,
                        std::vector<Field3D::FieldRes::Ptr>& outFields, const Field3D::Box3i& extents,
                        const Field3D::Box3i& dataWindow, float spacing ) {
    outFields.clear();

    typedef std::pair<std::ptrdiff_t, void ( * )( Field3D::FieldRes&, const void*, const Field3D::V3i& )> accessor_type;

    std::vector<accessor_type> accessors;

    Field3D::MatrixFieldMapping::Ptr pMap( new Field3D::MatrixFieldMapping );

    Field3D::M44d tm;
    tm.setTranslation( Field3D::V3f( spacing * extents.min.x, spacing * extents.min.y, spacing * extents.min.z ) );
    tm.scale( Field3D::V3f( spacing * ( extents.max.x - extents.min.x + 1 ),
                            spacing * ( extents.max.y - extents.min.y + 1 ),
                            spacing * ( extents.max.z - extents.min.z + 1 ) ) );

    // Why is it like this?!

    pMap->setExtents( extents );
    pMap->setLocalToWorld( tm );

    const frantic::channels::channel_map& map = field.get_channel_map();

    for( std::size_t i = 0, iEnd = map.channel_count(); i < iEnd; ++i ) {
        switch( map[i].data_type() ) {
        case frantic::channels::data_type_float32:
            if( map[i].arity() == 1 ) {
                Field3D::DenseFieldf::Ptr pF( new Field3D::DenseFieldf );

                pF->name = "default";
                pF->attribute = frantic::strings::to_string( map[i].name() );
                pF->setMapping( pMap );
                pF->setSize( extents, dataWindow );

                outFields.push_back( pF );
                accessors.push_back( accessor_type( map[i].offset(), &dense_grid_writer<float, float>::write ) );
            } else if( map[i].arity() == 3 ) {
                Field3D::DenseField3f::Ptr pF( new Field3D::DenseField3f );

                pF->name = "default";
                pF->attribute = frantic::strings::to_string( map[i].name() );
                pF->setMapping( pMap );
                pF->setSize( extents, dataWindow );

                outFields.push_back( pF );
                accessors.push_back( accessor_type(
                    map[i].offset(), &dense_grid_writer<Field3D::V3f, frantic::graphics::vector3f>::write ) );
            } else {
                throw std::runtime_error( "Invalid type for sample_to_field3d(): " +
                                          frantic::strings::to_string( frantic::channels::channel_data_type_str(
                                              map[i].arity(), map[i].data_type() ) ) );
            }

            break;
        default:
            throw std::runtime_error( "Invalid type for sample_to_field3d(): " +
                                      frantic::strings::to_string( frantic::channels::channel_data_type_str(
                                          map[i].arity(), map[i].data_type() ) ) );
        }
    }

    Field3D::V3i p;

    void* buffer = alloca( map.structure_size() );

    // TODO: Parallelize this.
    for( p = dataWindow.min; p.z <= dataWindow.max.z; ++p.z, p.y = dataWindow.min.y ) {
        for( ; p.y <= dataWindow.max.y; ++p.y, p.x = dataWindow.min.x ) {
            for( ; p.x <= dataWindow.max.x; ++p.x ) {
                Field3D::V3d wsP;

                pMap->voxelToWorld( Field3D::V3d( static_cast<double>( p.x ) + 0.5, static_cast<double>( p.y ) + 0.5,
                                                  static_cast<double>( p.z ) + 0.5 ),
                                    wsP );

                if( field.evaluate_field( buffer, frantic::graphics::vector3f( static_cast<float>( wsP.x ),
                                                                               static_cast<float>( wsP.y ),
                                                                               static_cast<float>( wsP.z ) ) ) ) {
                    // Write to the grids.
                    for( std::size_t i = 0, iEnd = outFields.size(); i < iEnd; ++i )
                        accessors[i].second( *outFields[i], static_cast<char*>( buffer ) + accessors[i].first, p );
                }
            }
        }
    }
}

void write_fields_to_file( const frantic::tstring& filePath, const std::vector<Field3D::FieldRes::Ptr>& fields ) {
    std::stringstream ssout;
    scoped_redirect redir( std::cout, ssout.rdbuf() );

    Field3D::Field3DOutputFile file;

    std::string realPath = frantic::strings::to_string( filePath );

    if( !file.create( realPath ) ) {
        FF_LOG( error ) << _T("Failed to create file: ") << filePath << std::endl;
        FF_LOG( error ) << _T("COUT:") << std::endl << frantic::strings::to_tstring( ssout.str() ) << std::endl;

        throw std::runtime_error( "write_fields_to_file() Failed to create file: \"" + realPath + "\"" );
    }

    bool result = true;
    for( std::vector<Field3D::FieldRes::Ptr>::const_iterator it = fields.begin(), itEnd = fields.end();
         it != itEnd && result; ++it ) {
        if( Field3D::Field<float>::Ptr pField = Field3D::field_dynamic_cast<Field3D::Field<float>>( *it ) ) {
            result = file.writeScalarLayer<float>( "default", pField->attribute, pField );
        } else if( Field3D::Field<half>::Ptr pField = Field3D::field_dynamic_cast<Field3D::Field<half>>( *it ) ) {
            result = file.writeScalarLayer<half>( "default", pField->attribute, pField );
        } else if( Field3D::Field<double>::Ptr pField = Field3D::field_dynamic_cast<Field3D::Field<double>>( *it ) ) {
            result = file.writeScalarLayer<double>( "default", pField->attribute, pField );
        } else if( Field3D::Field<Field3D::V3f>::Ptr pField =
                       Field3D::field_dynamic_cast<Field3D::Field<Field3D::V3f>>( *it ) ) {
            result = file.writeVectorLayer<float>( "default", pField->attribute, pField );
        } else if( Field3D::Field<Field3D::V3h>::Ptr pField =
                       Field3D::field_dynamic_cast<Field3D::Field<Field3D::V3h>>( *it ) ) {
            result = file.writeVectorLayer<half>( "default", pField->attribute, pField );
        } else if( Field3D::Field<Field3D::V3d>::Ptr pField =
                       Field3D::field_dynamic_cast<Field3D::Field<Field3D::V3d>>( *it ) ) {
            result = file.writeVectorLayer<double>( "default", pField->attribute, pField );
        } else {
            FF_LOG( warning ) << _T("Unknown Field3D type: ") << frantic::strings::to_tstring( ( *it )->classType() )
                              << std::endl;
        }
    }

    if( !result ) {
        FF_LOG( error ) << _T("Failed to write file: ") << filePath << std::endl;
        FF_LOG( error ) << _T("COUT:") << std::endl << frantic::strings::to_tstring( ssout.str() ) << std::endl;
    } else {
        result = file.close();
        if( !result ) {
            FF_LOG( error ) << _T("Failed to close written file: ") << filePath << std::endl;
            FF_LOG( error ) << _T("COUT:") << std::endl << frantic::strings::to_tstring( ssout.str() ) << std::endl;
        }
    }
}

// void write_fields_to_file( const frantic::tstring& filePath, const std::vector<
// boost::shared_ptr<frantic::volumetrics::field_interface> >& fields, Field3D::Box3i& extents, Field3D::Box3i&
// dataWindow, float spacing ){ 	Field3D::Field3DOutputFile file;
//
//	std::string realPath = frantic::strings::to_string( filePath );
//
//	file.create( realPath );
//
//	for( std::vector< boost::shared_ptr<frantic::volumetrics::field_interface> >::const_iterator it =
//fields.begin(), itEnd = fields.end(); it != itEnd; ++it ){ 		if( boost::shared_ptr<field3d_field_interface> pField3D =
//boost::dynamic_pointer_cast<field3d_field_interface>( *it ) ){ 			Field3D::FieldRes::Ptr pImpl =
//pField3D->get_field_ptr();
//
//			std::string dtString = pImpl->dataTypeString();
//
//			// TODO: Should I be applying the extents & data window to these fields?
//			if( Field3D::Field<float>::Ptr pField = Field3D::field_dynamic_cast< Field3D::Field<float> >( pImpl
//) ){ 				file.writeScalarLayer<float>( pField->attribute, pField ); 			} else if( Field3D::Field<half>::Ptr pField =
//Field3D::field_dynamic_cast< Field3D::Field<half> >( pImpl ) ){ 				file.writeScalarLayer<half>( pField->attribute, pField
//); 			} else if( Field3D::Field<double>::Ptr pField = Field3D::field_dynamic_cast< Field3D::Field<double> >( pImpl ) ){
//				file.writeScalarLayer<double>( pField->attribute, pField );
//			} else if( Field3D::Field<Field3D::V3f>::Ptr pField = Field3D::field_dynamic_cast<
//Field3D::Field<Field3D::V3f> >( pImpl ) ){ 				file.writeVectorLayer<float>( pField->attribute, pField ); 			} else if(
//Field3D::Field<Field3D::V3h>::Ptr pField = Field3D::field_dynamic_cast< Field3D::Field<Field3D::V3h> >( pImpl ) ){
//				file.writeVectorLayer<half>( pField->attribute, pField );
//			} else if( Field3D::Field<Field3D::V3d>::Ptr pField = Field3D::field_dynamic_cast<
//Field3D::Field<Field3D::V3d> >( pImpl ) ){ 				file.writeVectorLayer<double>( pField->attribute, pField ); 			}else{
//				FF_LOG(warning) << _T("Unknown Field3D type: ") << frantic::strings::to_tstring(
//pImpl->classType() ) << std::endl;
//			}
//
//			continue;
//		}
//
//		// We need to sample our field into a Field3D dense grid.
//
//		const frantic::channels::channel_map& map = (*it)->get_channel_map();
//		if( map.channel_count() != 1 )
//			throw std::runtime_error( "write_fields_to_file() does not support fields with multiple channels"
//);
//
//		switch( map[0].data_type() ){
//		case frantic::channels::data_type_float32:
//			if( map[0].arity() == 1 ){
//				Field3D::DenseField<float>::Ptr pF = convert_to_field3d<float, float>( **it, extents,
//dataWindow, spacing );
//
//				pF->name = "";
//				pF->attribute = frantic::strings::to_string( map[0].name() );
//
//				file.writeScalarLayer<float>( pF->attribute, pF );
//			}else if( map[0].arity() == 3 ){
//				Field3D::DenseField<Field3D::V3f>::Ptr pF = convert_to_field3d<Field3D::V3f, Field3D::V3f>(
//**it, extents, dataWindow, spacing );
//
//				pF->name = "";
//				pF->attribute = frantic::strings::to_string( map[0].name() );
//
//				file.writeVectorLayer<float>( pF->attribute, pF );
//			}else{
//				throw std::runtime_error( "write_fields_to_file() does not support " +
//frantic::strings::to_string( frantic::channels::channel_data_type_str( map[0].arity(), map[0].data_type() ) ) );
//			}
//
//			break;
//		default:
//			throw std::runtime_error( "write_fields_to_file() does not support " + frantic::strings::to_string(
//frantic::channels::channel_data_type_str( map[0].arity(), map[0].data_type() ) ) );
//		}
//	}
//
//	file.close();
// }
//
// void write_fields_to_file( const frantic::tstring& filePath, const boost::shared_ptr<concatenated_field>& fields,
// Field3D::Box3i& extents, Field3D::Box3i& dataWindow, float spacing ){ 	std::vector<
//boost::shared_ptr<frantic::volumetrics::field_interface> > separateFields;
//
//	for( std::size_t i = 0, iEnd = fields->get_num_fields(); i < iEnd; ++i )
//		separateFields.push_back( fields->get_field(i) );
//
//	write_fields_to_file( filePath, separateFields, extents, dataWindow, spacing );
// }

} // namespace ember
#endif
