# Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
# SPDX-License-Identifier: Apache-2.0
import os 
from conans import ConanFile, CMake
from typing import Any

SETTINGS: list[str] = [
    'os',
    'compiler',
    'build_type',
    'arch'
]

TOOL_REQUIRES: list[str]= [
    'cmake/3.24.1',
    'thinkboxcmlibrary/1.0.0'
]

REQUIRES: list[str] = [
    'thinkboxlibrary/1.0.1',
    'magma/1.0.1',
    'openvdb/8.0.1',
    'hdf5/1.13.1',
    'tinyxml2/9.0.0',
    # Resolve upstream version conflicts
    'boost/1.79.0',
    'lz4/1.9.4',
    'zlib/1.2.13'
]

class StokeConan(ConanFile):
    name: str = 'stoke'
    version: str = '1.0.0'
    license: str = 'Apache-2.0'
    description: str = 'Shared Stoke code.'
    settings: list[str] = SETTINGS
    requires: list[str] = REQUIRES
    tool_requires: list[str] = TOOL_REQUIRES
    generators: str | list[str] = 'cmake_find_package'
    default_options: dict[str, Any] = {
        'openvdb:shared': False
    }

    def build(self) -> None:
        cmake = CMake(self)
        cmake.configure()
        cmake.build()

    def export_sources(self) -> None:
        self.copy('**.h', src='', dst='')
        self.copy('**.hpp', src='', dst='')
        self.copy('**.cpp', src='', dst='')
        self.copy('**.cc', src='', dst='')
        self.copy('CMakeLists.txt', src='', dst='')
        self.copy('NOTICE.txt', src='', dst='')
        self.copy('LICENSE.txt', src='', dst='')

    def package(self) -> None:
        cmake = CMake(self)
        cmake.install()

        with open(os.path.join(self.source_folder, 'NOTICE.txt'), 'r', encoding='utf8') as notice_file:
            notice_contents = notice_file.readlines()
        with open(os.path.join(self.source_folder, 'LICENSE.txt'), 'r', encoding='utf8') as license_file:
            license_contents = license_file.readlines()
        os.makedirs(os.path.join(self.package_folder, 'licenses'), exist_ok=True)
        with open(os.path.join(self.package_folder, 'licenses', 'LICENSE'), 'w', encoding='utf8') as cat_license_file:
            cat_license_file.writelines(notice_contents)
            cat_license_file.writelines(license_contents)

    def deploy(self) -> None:
        self.copy("*", dst="lib", src="lib")
        self.copy("*", dst="include", src="include")

    def package_info(self) -> None:
        self.cpp_info.libs = ["stoke"]
