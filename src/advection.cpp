// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#include "stdafx.h"

#include <ember/advection.hpp>

#include <frantic/logging/logging_level.hpp>

#include <mkl.h>
#include <mkl_poisson.h>

#include <boost/scoped_array.hpp>

namespace ember {

void do_poisson_solve( staggered_grid& velocityGrid, float stepInSeconds, const char _boundaryType[],
                       frantic::logging::progress_logger* progress ) {
    char boundaryType[6];
    memcpy( boundaryType, _boundaryType, sizeof( char ) * 6 );

    frantic::logging::null_progress_logger nullLogger;
    if( !progress )
        progress = &nullLogger;

    int sizeX = velocityGrid.get_size( 0 ) - 1;
    int sizeY = velocityGrid.get_size( 1 ) - 1;
    int sizeZ = velocityGrid.get_size( 2 ) - 1;
    float spacing = velocityGrid.get_spacing();

    // std::size_t vectorSize = (std::size_t)( sizeX * sizeY * sizeZ );

    float density = 1.f;
    float rhsFactor = ( density * spacing ) / stepInSeconds;
    float updateScale = stepInSeconds / ( density * spacing );

    int nx = sizeX - 1;
    int ny = sizeY - 1;
    int nz = sizeZ - 1;

    boost::scoped_array<float> spar( new float[5 * ( nx + ny ) / 2 + 9] );
    boost::scoped_array<int> ipar( new int[128] );

    // Examples show this being initialized to 0.
    memset( ipar.get(), 0, sizeof( int ) * 128 );

    float ax = 0, bx = (float)( nx );
    float ay = 0, by = (float)( ny );
    float az = 0, bz = (float)( nz );

    int stat = 0;
    float q = 0.f;

    progress->check_for_abort();

    boost::scoped_array<float> f( new float[( nx + 1 ) * ( ny + 1 ) * ( nz + 1 )] );

    float* it = f.get();
    float values[6];

    for( int k = 0; k < sizeZ; ++k ) {
        for( int j = 0; j < sizeY; ++j ) {
            for( int i = 0; i < sizeX; ++i, ++it ) {
                velocityGrid.get_staggered_values( i, j, k, values );

                float divergence = 0.f;
                // if( i > 0 )
                divergence -= values[0];
                // if( i+1 < sizeX )
                divergence += values[1];

                // if( j > 0 )
                divergence -= values[2];
                // if( j+1 < sizeY )
                divergence += values[3];

                // if( k > 0 )
                divergence -= values[4];
                // if( k+1 < sizeZ )
                divergence += values[5];

                *it = -divergence * rhsFactor;
            }
        }

        progress->check_for_abort();
    }

    boost::scoped_array<float> bd_ax( new float[( ny + 1 ) * ( nz + 1 )] ), bd_bx( new float[( ny + 1 ) * ( nz + 1 )] );
    boost::scoped_array<float> bd_ay( new float[( nx + 1 ) * ( nz + 1 )] ), bd_by( new float[( nx + 1 ) * ( nz + 1 )] );
    boost::scoped_array<float> bd_az( new float[( nx + 1 ) * ( ny + 1 )] ), bd_bz( new float[( nx + 1 ) * ( ny + 1 )] );

    // Let's set up the boundary conditions for the Neumann conditions
    if( boundaryType[0] == 'N' ) {
        for( int k = 0; k < sizeZ; ++k ) {
            for( int j = 0; j < sizeY; ++j )
                bd_ax[j + sizeY * k] = -rhsFactor * velocityGrid.get_staggered_voxel( 0, j, k ).x;
        }
    } else {
        memset( bd_ax.get(), 0, sizeof( float ) * ( ny + 1 ) * ( nz + 1 ) );
    }

    if( boundaryType[1] == 'N' ) {
        for( int k = 0; k < sizeZ; ++k ) {
            for( int j = 0; j < sizeY; ++j )
                bd_bx[j + sizeY * k] = rhsFactor * velocityGrid.get_staggered_voxel( sizeX, j, k ).x;
        }
    } else {
        memset( bd_bx.get(), 0, sizeof( float ) * ( ny + 1 ) * ( nz + 1 ) );
    }

    if( boundaryType[2] == 'N' ) {
        for( int k = 0; k < sizeZ; ++k ) {
            for( int i = 0; i < sizeX; ++i )
                bd_ay[i + sizeX * k] = -rhsFactor * velocityGrid.get_staggered_voxel( i, 0, k ).y;
        }
    } else {
        memset( bd_ay.get(), 0, sizeof( float ) * ( nx + 1 ) * ( nz + 1 ) );
    }

    if( boundaryType[3] == 'N' ) {
        for( int k = 0; k < sizeZ; ++k ) {
            for( int i = 0; i < sizeX; ++i )
                bd_by[i + sizeX * k] = rhsFactor * velocityGrid.get_staggered_voxel( i, sizeY, k ).y;
        }
    } else {
        memset( bd_by.get(), 0, sizeof( float ) * ( nx + 1 ) * ( nz + 1 ) );
    }

    if( boundaryType[4] == 'N' ) {
        for( int j = 0; j < sizeY; ++j ) {
            for( int i = 0; i < sizeX; ++i )
                bd_az[i + sizeX * j] = -rhsFactor * velocityGrid.get_staggered_voxel( i, j, 0 ).z;
        }
    } else {
        memset( bd_az.get(), 0, sizeof( float ) * ( nx + 1 ) * ( ny + 1 ) );
    }

    if( boundaryType[5] == 'N' ) {
        for( int j = 0; j < sizeY; ++j ) {
            for( int i = 0; i < sizeX; ++i )
                bd_bz[i + sizeX * j] = rhsFactor * velocityGrid.get_staggered_voxel( i, j, sizeZ ).z;
        }
    } else {
        memset( bd_bz.get(), 0, sizeof( float ) * ( nx + 1 ) * ( ny + 1 ) );
    }

    progress->check_for_abort();

    DFTI_DESCRIPTOR_HANDLE xhandle, yhandle;

    FF_LOG( debug ) << _T("MKL Solver boundary types: ") << boundaryType[0] << boundaryType[1] << boundaryType[2]
                    << boundaryType[3] << boundaryType[4] << boundaryType[5] << std::endl;

    s_init_Helmholtz_3D( &ax, &bx, &ay, &by, &az, &bz, &nx, &ny, &nz, boundaryType, &q, ipar.get(), spar.get(), &stat );
    FF_LOG( debug ) << _T("s_init_Helmholtz_3D: ") << stat << std::endl;

    progress->check_for_abort();

    s_commit_Helmholtz_3D( f.get(), bd_ax.get(), bd_bx.get(), bd_ay.get(), bd_by.get(), bd_az.get(), bd_bz.get(),
                           &xhandle, &yhandle, ipar.get(), spar.get(), &stat );
    FF_LOG( debug ) << _T("s_commit_Helmholtz_3D: ") << stat << std::endl;

    // Write messages to file
    // ipar[1] = -1;
    // ipar[2] = -1;

    progress->check_for_abort();

    s_Helmholtz_3D( f.get(), bd_ax.get(), bd_bx.get(), bd_ay.get(), bd_by.get(), bd_az.get(), bd_bz.get(), &xhandle,
                    &yhandle, ipar.get(), spar.get(), &stat );
    FF_LOG( debug ) << _T("s_Helmholtz_3D: ") << stat << std::endl;

    progress->check_for_abort();

    free_Helmholtz_3D( &xhandle, &yhandle, ipar.get(), &stat );
    FF_LOG( debug ) << _T("free_Helmholtz_3D: ") << stat << std::endl;

    int elementIndex = 0;
    for( int z = 0, zEnd = sizeZ; z < zEnd; ++z ) {
        for( int y = 0, yEnd = sizeY; y < yEnd; ++y ) {
            for( int x = 0, xEnd = sizeX; x < xEnd; ++x, ++elementIndex ) {
                // float(&v)[3] = velocityGrid.get_staggered_voxel( x, y, z );
                vec3& v = velocityGrid.get_staggered_voxel( x, y, z );

                if( x > 0 ) {
                    v[0] -= ( f[elementIndex] - f[elementIndex - 1] ) * updateScale;
                } else if( boundaryType[0] == 'N' ) {
                    v[0] = 0.f; // Neumann constant 0 velocity normal to the wall.
                } else {
                    v[0] -= f[elementIndex] * updateScale; // Dirchlet, pressure is 0 outside the grid.
                }

                if( y > 0 ) {
                    v[1] -= ( f[elementIndex] - f[elementIndex - sizeX] ) * updateScale;
                } else if( boundaryType[2] == 'N' ) {
                    v[1] = 0.f; // Neumann constant 0 velocity normal to the wall.
                } else {
                    v[1] -= f[elementIndex] * updateScale; // Dirchlet, pressure is 0 outside the grid.
                }

                if( z > 0 ) {
                    v[2] -= ( f[elementIndex] - f[elementIndex - sizeX * sizeY] ) * updateScale;
                } else if( boundaryType[4] == 'N' ) {
                    v[2] = 0.f; // Neumann constant 0 velocity normal to the wall.
                } else {
                    v[2] -= f[elementIndex] * updateScale; // Dirchlet, pressure is 0 outside the grid.
                }
            }

            if( boundaryType[1] == 'N' ) {
                velocityGrid.get_staggered_voxel( sizeX, y, z )[0] =
                    0.f; // Neumann constant 0 velocity normal to the wall.
            } else {
                velocityGrid.get_staggered_voxel( sizeX, y, z )[0] +=
                    f[elementIndex - 1] * updateScale; // Dirchlet, pressure is 0 outside the grid.
            }
        }
        if( boundaryType[3] == 'N' ) {
            for( int x = 0, xEnd = sizeX; x < xEnd; ++x )
                velocityGrid.get_staggered_voxel( x, sizeY, z )[1] =
                    0.f; // Neumann constant 0 velocity normal to the wall.
        } else {
            for( int x = 0, xEnd = sizeX; x < xEnd; ++x )
                velocityGrid.get_staggered_voxel( x, sizeY, z )[1] +=
                    f[elementIndex + x - sizeX] * updateScale; // Dirchlet, pressure is 0 outside the grid.
        }

        progress->check_for_abort();
    }
    if( boundaryType[5] == 'N' ) {
        for( int y = 0, yEnd = sizeY; y < yEnd; ++y ) {
            for( int x = 0, xEnd = sizeX; x < xEnd; ++x )
                velocityGrid.get_staggered_voxel( x, y, sizeZ )[2] =
                    0.f; // Neumann constant 0 velocity normal to the wall.
        }
    } else {
        for( int y = 0, yEnd = sizeY, off = 0; y < yEnd; ++y ) {
            for( int x = 0, xEnd = sizeX; x < xEnd; ++x, ++off )
                velocityGrid.get_staggered_voxel( x, y, sizeZ )[2] +=
                    f[elementIndex + off - sizeX * sizeY] * updateScale; // Dirchlet, pressure is 0 outside the grid.
        }
    }
}

// inline field_interface_ptr do_pressure_solve( field_interface& velField, int bounds[], float spacing, float
// stepInSeconds ){ 	boost::intrusive_ptr< staggered_grid > result( new staggered_grid( bounds, spacing ) );
//
//	//TODO: If already a compatible grid, we should just copy it or re-use it instead.
//	result->sample( velField/*->optimize()*/ );
//
//	int sizeX = bounds[1] - bounds[0];
//	int sizeY = bounds[3] - bounds[2];
//	int sizeZ = bounds[5] - bounds[4];
//
//	std::size_t vectorSize = (std::size_t)( sizeX * sizeY * sizeZ );
//
//	CGSolver<double> cg;
//	SparseMatrix<double> matrix((unsigned int) vectorSize);
//	std::vector<double> rhsVector(vectorSize);
//	std::vector<double> resultVector(vectorSize);
//
//	cg.set_solver_parameters( 1e-6*spacing,200 );
//
//	float density = 1.f;
//	//float stepInSeconds = 1.f / 30.f;
//
//	double updateScale = stepInSeconds / ( density * spacing );
//	double rhsFactor = (density * spacing) / stepInSeconds ;
//
//	float offset = spacing * 0.5f;
//
//	//Consider having the staggered values "outside" of the field be velocity 0.
//
//	//Iterating over cell centers
//	int elementIndex = 0;
//
//	for( int z = 0, zEnd = sizeZ; z < zEnd; ++z ){
//		for( int y = 0, yEnd = sizeY; y < yEnd; ++y ){
//			for( int x = 0, xEnd = sizeX; x < xEnd; ++x, ++elementIndex ){
//				float faceVelocities[6];
//				double divergence = 0.0;
//				double coefficient = 0.0;
//
//				result->get_staggered_values( x, y, z, faceVelocities );
//
//				if( x > 0 ){
//					matrix.set_element( elementIndex, elementIndex - 1, -1.0 );
//					coefficient += 1.0;
//					divergence -= faceVelocities[0];
//				}
//
//				if( x+1 < sizeX ){
//					matrix.set_element( elementIndex, elementIndex + 1, -1.0 );
//					coefficient += 1.0;
//					divergence += faceVelocities[1];
//				}
//
//				if( y > 0 ){
//					matrix.set_element( elementIndex, elementIndex - sizeX, -1.0 );
//					coefficient += 1.0;
//					divergence -= faceVelocities[2];
//				}
//
//				if( y+1 < sizeY ){
//					matrix.set_element( elementIndex, elementIndex + sizeX, -1.0 );
//					coefficient += 1.0;
//					divergence += faceVelocities[3];
//				}
//
//				if( z > 0 ){
//					matrix.set_element( elementIndex, elementIndex - sizeX * sizeY, -1.0 );
//					coefficient += 1.0;
//					divergence -= faceVelocities[4];
//				}
//
//				if( z+1 < sizeZ ){
//					matrix.set_element( elementIndex, elementIndex + sizeX * sizeY, -1.0 );
//					coefficient += 1.0;
//					divergence += faceVelocities[5];
//				}
//
//				matrix.set_element( elementIndex, elementIndex, coefficient );
//				rhsVector[elementIndex] = -divergence * rhsFactor;
//			}
//		}
//	}
//
//	// solve the matrix using the CG solver
//	int iterations=0;
//	double residual=0.0;
//
//	FixedSparseMatrix<double> fixed_matrix;
//	fixed_matrix.construct_from_matrix(matrix);
//
//	if( !cg.solve( fixed_matrix, rhsVector, resultVector, residual, iterations ) ) {
//		throw std::runtime_error( "Failed to solve the matrix" );
//	}
//
//	elementIndex = 0;
//
//	for( int z = 0, zEnd = sizeZ; z < zEnd; ++z ){
//		for( int y = 0, yEnd = sizeY; y < yEnd; ++y ){
//			for( int x = 0, xEnd = sizeX; x < xEnd; ++x, ++elementIndex ){
//				vec3& v = result->get_staggered_voxel( x, y, z );
//
//				if( x > 0 ){
//					v.x -= ( resultVector[elementIndex] - resultVector[elementIndex - 1] ) *
//updateScale; 				}else{
//					//v.x -= ( resultVector[elementIndex] ) * updateScale;
//					v.x = 0.f;
//				}
//
//				if( y > 0 ){
//					v.y -= ( resultVector[elementIndex] - resultVector[elementIndex - sizeX] ) *
//updateScale; 				}else{
//					//v.y -= ( resultVector[elementIndex] ) * updateScale;
//					v.y = 0.f;
//				}
//
//				if( z > 0 ){
//					v.z -= ( resultVector[elementIndex] - resultVector[elementIndex - sizeX * sizeY] ) *
//updateScale; 				}else{
//					//v.z -= ( resultVector[elementIndex] ) * updateScale;
//					v.z = 0.f;
//				}
//			}
//
//			//result->get_staggered_voxel( sizeX, y, z ).x -= ( -resultVector[ sizeX - 1 + sizeX * ( y + sizeY *
//z )] ) * updateScale; 			result->get_staggered_voxel( sizeX, y, z ).x = 0.f;
//		}
//		for( int x = 0, xEnd = sizeX; x < xEnd; ++x )
//			//result->get_staggered_voxel( x, sizeY, z ).y -= ( -resultVector[ x + sizeX * ( sizeY - 1 + sizeY *
//z )] ) * updateScale; 			result->get_staggered_voxel( x, sizeY, z ).y = 0.f;
//	}
//	for( int y = 0, yEnd = sizeY; y < yEnd; ++y ){
//		for( int x = 0, xEnd = sizeX; x < xEnd; ++x )
//			//result->get_staggered_voxel( x, y, sizeZ ).z -= ( -resultVector[ x + sizeX * ( y + sizeY * (sizeZ
//- 1) )] ) * updateScale; 			result->get_staggered_voxel( x, y, sizeZ ).z = 0.f;
//	}
//
//	return result;
// }

} // namespace ember
