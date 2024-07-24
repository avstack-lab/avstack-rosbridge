// Copyright 2023 Georg Novotny
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef AVSTACK_MSGS_RVIZ_PLUGINS__VISIBILITY_CONTROL_HPP_
#define AVSTACK_MSGS_RVIZ_PLUGINS__VISIBILITY_CONTROL_HPP_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
    #ifdef __GNUC__
        #define BOX_TRACK_COMMON_DISPLAY_HPP_EXPORT __attribute__((dllexport))
        #define BOX_TRACK_COMMON_DISPLAY_HPP_IMPORT __attribute__((dllimport))

        #define BOX_TRACK_DISPLAY_HPP_EXPORT __attribute__((dllexport))
        #define BOX_TRACK_DISPLAY_HPP_IMPORT __attribute__((dllimport))

        #define BOX_TRACK_ARRAY_DISPLAY_HPP_EXPORT __attribute__((dllexport))
        #define BOX_TRACK_ARRAY_DISPLAY_HPP_IMPORT __attribute__((dllimport))

        #define OBJECT_STATE_COMMON_DISPLAY_HPP_EXPORT __attribute__((dllexport))
        #define OBJECT_STATE_COMMON_DISPLAY_HPP_IMPORT __attribute__((dllimport))

        #define OBJECT_STATE_DISPLAY_HPP_EXPORT __attribute__((dllexport))
        #define OBJECT_STATE_DISPLAY_HPP_IMPORT __attribute__((dllimport))

        #define OBJECT_STATE_ARRAY_DISPLAY_HPP_EXPORT __attribute__((dllexport))
        #define OBJECT_STATE_ARRAY_DISPLAY_HPP_IMPORT __attribute__((dllimport))
    #else
        #define BOX_TRACK_COMMON_DISPLAY_HPP_EXPORT __declspec(dllexport)
        #define BOX_TRACK_COMMON_DISPLAY_HPP_IMPORT __declspec(dllimport)

        #define BOX_TRACK_DISPLAY_HPP_EXPORT __declspec(dllexport)
        #define BOX_TRACK_DISPLAY_HPP_IMPORT __declspec(dllimport)

        #define BOX_TRACK_ARRAY_DISPLAY_HPP_EXPORT __declspec(dllexport)
        #define BOX_TRACK_ARRAY_DISPLAY_HPP_IMPORT __declspec(dllimport)

        #define OBJECT_STATE_COMMON_DISPLAY_HPP_EXPORT __declspec(dllexport)
        #define OBJECT_STATE_COMMON_DISPLAY_HPP_IMPORT __declspec(dllimport)

        #define OBJECT_STATE_DISPLAY_HPP_EXPORT __declspec(dllexport)
        #define OBJECT_STATE_DISPLAY_HPP_IMPORT __declspec(dllimport)

        #define OBJECT_STATE_ARRAY_DISPLAY_HPP_EXPORT __declspec(dllexport)
        #define OBJECT_STATE_ARRAY_DISPLAY_HPP_IMPORT __declspec(dllimport)
    #endif

    #ifdef BOX_TRACK_COMMON_DISPLAY_HPP_BUILDING_LIBRARY
        #define BOX_TRACK_COMMON_DISPLAY_HPP_PUBLIC BOX_TRACK_COMMON_DISPLAY_HPP_EXPORT
    #else
        #define BOX_TRACK_COMMON_DISPLAY_HPP_PUBLIC BOX_TRACK_COMMON_DISPLAY_HPP_IMPORT
    #endif
    #define BOX_TRACK_COMMON_DISPLAY_HPP_PUBLIC_TYPE BOX_TRACK_COMMON_DISPLAY_HPP_PUBLIC
    #define BOX_TRACK_COMMON_DISPLAY_HPP_LOCAL

    #ifdef BOX_TRACK_DISPLAY_HPP_BUILDING_LIBRARY
        #define BOX_TRACK_DISPLAY_HPP_PUBLIC BOX_TRACK_DISPLAY_HPP_EXPORT
    #else
        #define BOX_TRACK_DISPLAY_HPP_PUBLIC BOX_TRACK_DISPLAY_HPP_IMPORT
    #endif
    #define BOX_TRACK_DISPLAY_HPP_PUBLIC_TYPE BOX_TRACK_DISPLAY_HPP_PUBLIC
    #define BOX_TRACK_DISPLAY_HPP_LOCAL

    #ifdef BOX_TRACK_ARRAY_DISPLAY_HPP_BUILDING_LIBRARY
    #define BOX_TRACK_ARRAY_DISPLAY_HPP_PUBLIC BOX_TRACK_ARRAY_DISPLAY_HPP_EXPORT
    #else
        #define BOX_TRACK_ARRAY_DISPLAY_HPP_PUBLIC BOX_TRACK_ARRAY_DISPLAY_HPP_IMPORT
    #endif
    #define BOX_TRACK_ARRAY_DISPLAY_HPP_PUBLIC_TYPE BOX_TRACK_ARRAY_DISPLAY_HPP_PUBLIC
    #define BOX_TRACK_ARRAY_DISPLAY_HPP_LOCAL

    #ifdef OBJECT_STATE_COMMON_DISPLAY_HPP_BUILDING_LIBRARY
        #define OBJECT_STATE_COMMON_DISPLAY_HPP_PUBLIC OBJECT_STATE_COMMON_DISPLAY_HPP_EXPORT
    #else
        #define OBJECT_STATE_COMMON_DISPLAY_HPP_PUBLIC OBJECT_STATE_COMMON_DISPLAY_HPP_IMPORT
    #endif
    #define OBJECT_STATE_COMMON_DISPLAY_HPP_PUBLIC_TYPE OBJECT_STATE_COMMON_DISPLAY_HPP_PUBLIC
    #define OBJECT_STATE_COMMON_DISPLAY_HPP_LOCAL

    #ifdef OBJECT_STATE_DISPLAY_HPP_BUILDING_LIBRARY
        #define OBJECT_STATE_DISPLAY_HPP_PUBLIC OBJECT_STATE_DISPLAY_HPP_EXPORT
    #else
        #define OBJECT_STATE_DISPLAY_HPP_PUBLIC OBJECT_STATE_DISPLAY_HPP_IMPORT
    #endif
    #define OBJECT_STATE_DISPLAY_HPP_PUBLIC_TYPE OBJECT_STATE_DISPLAY_HPP_PUBLIC
    #define OBJECT_STATE_DISPLAY_HPP_LOCAL

    #ifdef OBJECT_STATE_ARRAY_DISPLAY_HPP_BUILDING_LIBRARY
    #define OBJECT_STATE_ARRAY_DISPLAY_HPP_PUBLIC OBJECT_STATE_ARRAY_DISPLAY_HPP_EXPORT
    #else
        #define OBJECT_STATE_ARRAY_DISPLAY_HPP_PUBLIC OBJECT_STATE_ARRAY_DISPLAY_HPP_IMPORT
    #endif
    #define OBJECT_STATE_ARRAY_DISPLAY_HPP_PUBLIC_TYPE OBJECT_STATE_ARRAY_DISPLAY_HPP_PUBLIC
    #define OBJECT_STATE_ARRAY_DISPLAY_HPP_LOCAL

#else
    #define BOX_TRACK_COMMON_DISPLAY_HPP_EXPORT __attribute__((visibility("default")))
    #define BOX_TRACK_COMMON_DISPLAY_HPP_IMPORT

    #define BOX_TRACK_DISPLAY_HPP_EXPORT __attribute__((visibility("default")))
    #define BOX_TRACK_DISPLAY_HPP_IMPORT

    #define BOX_TRACK_ARRAY_DISPLAY_HPP_EXPORT __attribute__((visibility("default")))
    #define BOX_TRACK_ARRAY_DISPLAY_HPP_IMPORT

    #define OBJECT_STATE_COMMON_DISPLAY_HPP_EXPORT __attribute__((visibility("default")))
    #define OBJECT_STATE_COMMON_DISPLAY_HPP_IMPORT

    #define OBJECT_STATE_DISPLAY_HPP_EXPORT __attribute__((visibility("default")))
    #define OBJECT_STATE_DISPLAY_HPP_IMPORT

    #define OBJECT_STATE_ARRAY_DISPLAY_HPP_EXPORT __attribute__((visibility("default")))
    #define OBJECT_STATE_ARRAY_DISPLAY_HPP_IMPORT
    #if __GNUC__ >= 4
        #define BOX_TRACK_COMMON_DISPLAY_HPP_PUBLIC __attribute__((visibility("default")))
        #define BOX_TRACK_COMMON_DISPLAY_HPP_LOCAL __attribute__((visibility("hidden")))

        #define BOX_TRACK_DISPLAY_HPP_PUBLIC __attribute__((visibility("default")))
        #define BOX_TRACK_DISPLAY_HPP_LOCAL __attribute__((visibility("hidden")))

        #define BOX_TRACK_ARRAY_DISPLAY_HPP_PUBLIC __attribute__((visibility("default")))
        #define BOX_TRACK_ARRAY_DISPLAY_HPP_LOCAL __attribute__((visibility("hidden")))

        #define OBJECT_STATE_COMMON_DISPLAY_HPP_PUBLIC __attribute__((visibility("default")))
        #define OBJECT_STATE_COMMON_DISPLAY_HPP_LOCAL __attribute__((visibility("hidden")))

        #define OBJECT_STATE_DISPLAY_HPP_PUBLIC __attribute__((visibility("default")))
        #define OBJECT_STATE_DISPLAY_HPP_LOCAL __attribute__((visibility("hidden")))

        #define OBJECT_STATE_ARRAY_DISPLAY_HPP_PUBLIC __attribute__((visibility("default")))
        #define OBJECT_STATE_ARRAY_DISPLAY_HPP_LOCAL __attribute__((visibility("hidden")))

    #else
        #define BOX_TRACK_COMMON_DISPLAY_HPP_PUBLIC
        #define BOX_TRACK_COMMON_DISPLAY_HPP_LOCAL

        #define BOX_TRACK_DISPLAY_HPP_PUBLIC
        #define BOX_TRACK_DISPLAY_HPP_LOCAL

        #define BOX_TRACK_ARRAY_DISPLAY_HPP_PUBLIC
        #define BOX_TRACK_ARRAY_DISPLAY_HPP_LOCAL

        #define OBJECT_STATE_COMMON_DISPLAY_HPP_PUBLIC
        #define OBJECT_STATE_COMMON_DISPLAY_HPP_LOCAL

        #define OBJECT_STATE_DISPLAY_HPP_PUBLIC
        #define OBJECT_STATE_DISPLAY_HPP_LOCAL

        #define OBJECT_STATE_ARRAY_DISPLAY_HPP_PUBLIC
        #define OBJECT_STATE_ARRAY_DISPLAY_HPP_LOCAL
    #endif
    #define BOX_TRACK_COMMON_DISPLAY_HPP_PUBLIC_TYPE
    #define BOX_TRACK_DISPLAY_HPP_PUBLIC_TYPE
    #define BOX_TRACK_ARRAY_DISPLAY_HPP_PUBLIC_TYPE

    #define OBJECT_STATE_COMMON_DISPLAY_HPP_PUBLIC_TYPE
    #define OBJECT_STATE_DISPLAY_HPP_PUBLIC_TYPE
    #define OBJECT_STATE_ARRAY_DISPLAY_HPP_PUBLIC_TYPE
#endif

#endif  // AVSTACK_MSGS_RVIZ_PLUGINS__VISIBILITY_CONTROL_HPP_